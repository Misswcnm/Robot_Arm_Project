"""Compatibility server for the vehicle's legacy NX TCP JSON protocol.

The vehicle application remains the TCP client.  This process looks like the
old NX endpoint on port 8888 and translates each legacy message into one or
more calls to the local vision_arm_executor JSON-lines RPC service.
"""

import argparse
import codecs
import ipaddress
import json
import os
import socket
import threading
import time
import uuid


TERMINAL = {'succeeded', 'failed', 'timeout', 'cancelled'}
COMMAND_ACTIONS = {
    1: 'robot_reset',
    2: 'robot_enable',
    3: 'robot_disable',
    4: 'execution_enable',
    5: 'execution_disable',
    6: 'robot_clear_error',
}
EXPLICIT_ACTIONS = {
    'vision_icp_record_a',
    'vision_icp_record_b',
    'vision_icp_align',
    'vision_icp_align_and_move_b',
    'apriltag_locate',
    'apriltag_validate',
    'apriltag_pick',
    'arm_status',
}


def _request_id(prefix):
    return '%s-%s' % (prefix, uuid.uuid4().hex)


class JsonStreamDecoder:
    """Incrementally decode concatenated or whitespace-delimited JSON values."""

    def __init__(self, max_bytes=65536):
        self.max_bytes = int(max_bytes)
        self.byte_count = 0
        self.text = ''
        self.utf8 = codecs.getincrementaldecoder('utf-8')()
        self.decoder = json.JSONDecoder()

    def feed(self, data):
        self.byte_count += len(data)
        if self.byte_count > self.max_bytes:
            raise ValueError('legacy_message_too_large')
        self.text += self.utf8.decode(data)
        values = []
        while True:
            stripped = self.text.lstrip()
            if not stripped:
                self.text = ''
                self.byte_count = 0
                return values
            try:
                value, end = self.decoder.raw_decode(stripped)
            except ValueError:
                return values
            values.append(value)
            self.text = stripped[end:]
            self.byte_count = len(self.text.encode('utf-8'))


class VisionRpcClient:
    def __init__(self, host, port, auth_token='', socket_timeout=5.0,
                 max_bytes=65536, poll_interval=0.2):
        self.host = host
        self.port = int(port)
        self.auth_token = str(auth_token or '')
        self.socket_timeout = float(socket_timeout)
        self.max_bytes = int(max_bytes)
        self.poll_interval = float(poll_interval)

    def call(self, action, params=None, timeout_sec=120.0, dry_run=False,
             request_id=None):
        request = {
            'request_id': request_id or _request_id('nx'),
            'action': action,
            'params': dict(params or {}),
            'timeout_sec': float(timeout_sec),
            'dry_run': bool(dry_run),
        }
        if self.auth_token:
            request['auth_token'] = self.auth_token
        payload = (json.dumps(
            request, ensure_ascii=False, separators=(',', ':')) + '\n').encode(
                'utf-8')
        client = socket.create_connection(
            (self.host, self.port), timeout=self.socket_timeout)
        try:
            client.settimeout(self.socket_timeout)
            client.sendall(payload)
            response = b''
            while b'\n' not in response:
                chunk = client.recv(4096)
                if not chunk:
                    break
                response += chunk
                if len(response) > self.max_bytes:
                    raise ValueError('rpc_response_too_large')
            if not response:
                raise RuntimeError('vision_rpc_closed_without_response')
            result = json.loads(response.split(b'\n', 1)[0].decode('utf-8'))
            if not isinstance(result, dict):
                raise RuntimeError('vision_rpc_response_must_be_object')
            return result
        finally:
            client.close()

    def run(self, action, params=None, timeout_sec=120.0, dry_run=False,
            request_id=None):
        deadline = time.monotonic() + max(0.1, float(timeout_sec)) + 2.0
        result = self.call(
            action, params, timeout_sec, dry_run, request_id=request_id)
        target = result.get('request_id')
        while result.get('status') not in TERMINAL:
            if time.monotonic() >= deadline:
                return {
                    'request_id': target,
                    'action': action,
                    'backend': 'vision',
                    'status': 'timeout',
                    'error_code': 'gateway_timeout',
                    'message': 'NX gateway timed out waiting for vision task',
                    'metrics': {},
                    'artifacts': [],
                }
            time.sleep(self.poll_interval)
            result = self.call(
                'task_status', {'request_id': target}, 2.0, True,
                request_id=_request_id('nx-status'))
        return result


class RouteTable:
    def __init__(self, config):
        self.config = dict(config or {})
        defaults = self.config.get('defaults', {})
        self.default_timeout = float(defaults.get('timeout_sec', 200.0))
        self.default_dry_run = bool(defaults.get('dry_run', True))
        self.routes = list(self.config.get('routes', []))
        self.teaching_routes = list(self.config.get('teaching_routes', []))

    @classmethod
    def load(cls, path):
        with open(path, 'r', encoding='utf-8') as stream:
            value = json.load(stream)
        if not isinstance(value, dict):
            raise ValueError('route_file_must_be_object')
        return cls(value)

    @staticmethod
    def _match_value(expected, actual):
        return expected in (None, '', '*') or str(expected) == str(actual)

    def match(self, message, teaching=False):
        candidates = self.teaching_routes if teaching else self.routes
        matches = []
        for index, route in enumerate(candidates):
            if not isinstance(route, dict) or not route.get('action'):
                continue
            fields = ('mapid', 'poseid', 'label')
            if all(self._match_value(route.get(field), message.get(field, ''))
                   for field in fields):
                specificity = sum(
                    route.get(field) not in (None, '', '*') for field in fields)
                matches.append((specificity, -index, route))
        if not matches:
            return None
        return max(matches, key=lambda item: (item[0], item[1]))[2]

    def invocation(self, route):
        return {
            'action': route['action'],
            'params': dict(route.get('params', {})),
            'timeout_sec': float(route.get(
                'timeout_sec', self.default_timeout)),
            'dry_run': bool(route.get('dry_run', self.default_dry_run)),
        }


class NxMessageHandler:
    def __init__(self, routes, rpc):
        self.routes = routes
        self.rpc = rpc
        self.resource_lock = threading.Lock()

    @staticmethod
    def _legacy_response(message, result, completion=False):
        succeeded = result.get('status') == 'succeeded'
        response = {
            'type': 'response',
            'status': 'done' if completion and succeeded else result.get(
                'status', 'failed'),
            'mapid': message.get('mapid', ''),
            'poseid': message.get('poseid', ''),
            'action': result.get('action', ''),
            'request_id': result.get('request_id', ''),
            'message': result.get('message', ''),
            'error_code': result.get('error_code', ''),
            'metrics': result.get('metrics', {}),
            'artifacts': result.get('artifacts', []),
        }
        return response

    def _run(self, invocation):
        return self.rpc.run(
            invocation['action'], invocation.get('params'),
            invocation.get('timeout_sec', 120.0),
            invocation.get('dry_run', False),
            request_id=_request_id('nx-' + invocation['action']))

    def handle(self, message):
        if not isinstance(message, dict):
            return {'type': 'error', 'status': 'failed',
                    'error_code': 'legacy_request_must_be_object'}
        msg_type = str(message.get('type', '') or '')
        if msg_type == 'mechanical_arm_command':
            try:
                command = int(message.get('command'))
            except (TypeError, ValueError):
                command = 0
            action = COMMAND_ACTIONS.get(command)
            if not action:
                return {'type': 'error', 'status': 'failed',
                        'error_code': 'unsupported_legacy_command'}
            # Auto/manual are the executor's immediate permission gate.  In
            # particular, command 5 must reach an active task so it can request
            # cancellation at the next safety checkpoint.
            if action in {'execution_enable', 'execution_disable'}:
                result = self.rpc.run(
                    action, {}, 30.0, False,
                    request_id=_request_id('nx-command-%d' % command))
                return self._legacy_response(message, result)
            with self.resource_lock:
                result = self.rpc.run(
                    action, {}, 30.0, False,
                    request_id=_request_id('nx-command-%d' % command))
            return self._legacy_response(message, result)

        with self.resource_lock:
            if msg_type in EXPLICIT_ACTIONS:
                invocation = {
                    'action': msg_type,
                    'params': dict(message.get('params', {})),
                    'timeout_sec': float(message.get('timeout_sec', 200.0)),
                    'dry_run': bool(message.get('dry_run', False)),
                }
                result = self._run(invocation)
                return self._legacy_response(
                    message, result, completion=msg_type not in {
                        'arm_status', 'vision_icp_record_a',
                        'vision_icp_record_b', 'apriltag_locate',
                        'apriltag_validate'})

            teaching = msg_type in {'demo_point_recorded', 'type1'}
            route = self.routes.match(message, teaching=teaching)
            if route is None:
                return {
                    'type': 'error',
                    'status': 'failed',
                    'mapid': message.get('mapid', ''),
                    'poseid': message.get('poseid', ''),
                    'error_code': ('teaching_route_not_found' if teaching
                                   else 'execution_route_not_found'),
                    'message': 'no matching NX compatibility route',
                }
            invocation = self.routes.invocation(route)
            invocation['params'].setdefault(
                'legacy_map_id', message.get('mapid', ''))
            invocation['params'].setdefault(
                'legacy_point_id', message.get('poseid', ''))
            if message.get('label'):
                invocation['params'].setdefault(
                    'legacy_label', message.get('label'))
            result = self._run(invocation)
            return self._legacy_response(
                message, result, completion=not teaching)


class NxCompatServer:
    def __init__(self, host, port, handler, allowed_clients, timeout=1.0,
                 max_bytes=65536):
        self.host = host
        self.port = int(port)
        self.handler = handler
        self.allowed_clients = [
            ipaddress.ip_network(item, strict=False)
            for item in allowed_clients]
        self.timeout = float(timeout)
        self.max_bytes = int(max_bytes)
        self.stop = threading.Event()
        self.sock = None

    def _allowed(self, address):
        peer = ipaddress.ip_address(address)
        return any(peer in network for network in self.allowed_clients)

    def serve_forever(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(4)
        self.sock.settimeout(0.5)
        print('NX compatibility gateway listening on %s:%d; allowed=%s' % (
            self.host, self.port,
            [str(network) for network in self.allowed_clients]), flush=True)
        while not self.stop.is_set():
            try:
                client, peer = self.sock.accept()
            except socket.timeout:
                continue
            if not self._allowed(peer[0]):
                client.close()
                continue
            threading.Thread(
                target=self._client, args=(client, peer), daemon=True).start()

    def close(self):
        self.stop.set()
        if self.sock:
            self.sock.close()

    def _client(self, client, peer):
        decoder = JsonStreamDecoder(self.max_bytes)
        send_lock = threading.Lock()
        client.settimeout(self.timeout)
        print('NX vehicle connected: %s:%s' % peer, flush=True)
        try:
            while not self.stop.is_set():
                try:
                    chunk = client.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    return
                for message in decoder.feed(chunk):
                    print('NX request: type=%s mapid=%s poseid=%s command=%s' % (
                        message.get('type', ''), message.get('mapid', ''),
                        message.get('poseid', ''), message.get('command', '')),
                        flush=True)
                    threading.Thread(
                        target=self._process_and_send,
                        args=(client, send_lock, message), daemon=True).start()
        except Exception as error:
            try:
                client.sendall((json.dumps({
                    'type': 'error', 'status': 'failed',
                    'error_code': 'nx_gateway_error',
                    'message': str(error),
                }, separators=(',', ':')) + '\n').encode('utf-8'))
            except Exception:
                pass
        finally:
            client.close()
            print('NX vehicle disconnected: %s:%s' % peer, flush=True)

    def _process_and_send(self, client, send_lock, message):
        try:
            response = self.handler.handle(message)
        except Exception as error:
            response = {
                'type': 'error',
                'status': 'failed',
                'mapid': message.get('mapid', ''),
                'poseid': message.get('poseid', ''),
                'error_code': 'nx_handler_error',
                'message': str(error),
            }
        payload = (json.dumps(
            response, ensure_ascii=False,
            separators=(',', ':')) + '\n').encode('utf-8')
        print('NX result: action=%s status=%s error=%s' % (
            response.get('action', ''), response.get('status', ''),
            response.get('error_code', '')), flush=True)
        try:
            with send_lock:
                client.sendall(payload)
        except (OSError, socket.error):
            return


def _csv(value):
    return [item.strip() for item in str(value).split(',') if item.strip()]


def main(argv=None):
    parser = argparse.ArgumentParser(description='Legacy NX to vision RPC bridge')
    parser.add_argument('--host', default=os.environ.get(
        'NX_COMPAT_HOST', '0.0.0.0'))
    parser.add_argument('--port', type=int, default=int(os.environ.get(
        'NX_COMPAT_PORT', '8888')))
    parser.add_argument('--allowed-clients', default=os.environ.get(
        'NX_COMPAT_ALLOWED_CLIENTS', '127.0.0.1/32'))
    parser.add_argument('--route-file', default=os.environ.get(
        'NX_ROUTE_FILE', ''))
    parser.add_argument('--rpc-host', default=os.environ.get(
        'NX_VISION_RPC_HOST', os.environ.get('VISION_RPC_HOST', '127.0.0.1')))
    parser.add_argument('--rpc-port', type=int, default=int(os.environ.get(
        'NX_VISION_RPC_PORT', '17881')))
    parser.add_argument('--rpc-auth-token', default=os.environ.get(
        'VISION_RPC_AUTH_TOKEN', ''))
    args = parser.parse_args(argv)
    if not args.route_file:
        raise SystemExit('NX_ROUTE_FILE or --route-file is required')
    rpc_host = '127.0.0.1' if args.rpc_host == '0.0.0.0' else args.rpc_host
    routes = RouteTable.load(os.path.expanduser(args.route_file))
    rpc = VisionRpcClient(
        rpc_host, args.rpc_port, auth_token=args.rpc_auth_token)
    server = NxCompatServer(
        args.host, args.port, NxMessageHandler(routes, rpc),
        _csv(args.allowed_clients))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == '__main__':
    main()
