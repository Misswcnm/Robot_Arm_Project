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

from .nx_commands import (
    COMMAND_ACTIONS, EXPLICIT_ACTIONS, parse_nx_request)


TERMINAL = {'succeeded', 'failed', 'timeout', 'cancelled'}
LEGACY_COMMAND_FIELDS = {
    1: 'fuwei',
    2: 'shangdian',
    3: 'xiadian',
    4: 'tuozhuai',
    5: 'quxiaotuozhuai',
    6: 'qingchu',
}
LEGACY_COMMAND_FIELD_ALIASES = {
    # Keep old response keys during the transition.  Their names no longer
    # describe the CR5 action, but an old application receiver may still look
    # for them.
    4: 'zidong',
    5: 'shoudong',
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


class GatewayConfig:
    def __init__(self, config):
        self.config = dict(config or {})
        unsupported = set(self.config) - {'schema_version', 'defaults'}
        if unsupported:
            raise ValueError(
                'unsupported gateway config fields: %s' %
                ','.join(sorted(unsupported)))
        defaults = self.config.get('defaults', {})
        if not isinstance(defaults, dict):
            raise ValueError('gateway defaults must be an object')
        self.default_timeout = float(defaults.get('timeout_sec', 200.0))

    @classmethod
    def load(cls, path):
        with open(path, 'r', encoding='utf-8') as stream:
            value = json.load(stream)
        if not isinstance(value, dict):
            raise ValueError('gateway_config_must_be_object')
        return cls(value)


class NxMessageHandler:
    def __init__(self, config, rpc):
        self.config = config
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
        if result.get('action') != 'vision_station_execute':
            if message.get('label') not in (None, ''):
                response['label'] = message.get('label')
            point_type = message.get(
                'point_type', message.get('pointType'))
            if point_type not in (None, ''):
                response['point_type'] = int(point_type)
        return response

    @classmethod
    def _legacy_command_response(cls, message, result, command,
                                 teaching_command=None):
        response = cls._legacy_response(message, result)
        response['command'] = int(command)
        if teaching_command:
            response['teaching_command'] = teaching_command
            # A station-scoped command is exclusively a teaching message.  Do
            # not add fuwei/shangdian/xiadian/tuozhuai fields that would claim
            # a controller action ran.
            return response
        value = (
            'success' if result.get('status') == 'succeeded' else 'failed')
        field = LEGACY_COMMAND_FIELDS[int(command)]
        response[field] = value
        alias = LEGACY_COMMAND_FIELD_ALIASES.get(int(command))
        if alias:
            response[alias] = value
        # RobotDemo.cpp misspelled the failed command-2 key.  Keep the correct
        # field above and add the historical alias for strict old consumers.
        if int(command) == 2 and value == 'failed':
            response['sahngdian'] = value
        return response

    def _run(self, invocation):
        return self.rpc.run(
            invocation['action'], invocation.get('params'),
            invocation.get('timeout_sec', 120.0),
            invocation.get('dry_run', False),
            request_id=_request_id('nx-' + invocation['action']))

    def _run_vehicle_production(self, invocation):
        """Authorize routed production without cycling permission per task."""
        if invocation.get('dry_run', False):
            return self._run(invocation)
        action = invocation['action']
        enabled = self.rpc.run(
            'execution_enable', {}, 10.0, False,
            request_id=_request_id('nx-auto-execution-enable'))
        if enabled.get('status') != 'succeeded':
            result = dict(enabled)
            result['action'] = action
            result['message'] = (
                'vehicle task could not acquire motion permission: ' +
                str(enabled.get('message', 'execution_enable failed')))
            return result
        # Keep permission enabled after a completed or failed station task.
        # An explicit execution_disable remains available for cancellation and
        # teaching start also clears production permission.
        return self._run(invocation)

    @staticmethod
    def _failure(message, error):
        return {
            'type': 'error',
            'status': 'failed',
            'mapid': message.get('mapid', '') if isinstance(message, dict) else '',
            'poseid': (
                message.get('poseid', '') if isinstance(message, dict) else ''),
            'error_code': 'invalid_request',
            'message': str(error),
            'metrics': {},
            'artifacts': [],
        }

    @staticmethod
    def _executor_params(message):
        params = dict(message.get('params', {}) or {})
        params.pop('pointType', None)
        for field in ('mapid', 'poseid', 'task_command'):
            if message.get(field) not in (None, ''):
                params.setdefault(field, message[field])
        # These fields historically appeared both at top level and in params.
        # Preserve an explicit params value; otherwise forward the top-level
        # value.  Omitting both leaves executor defaults unchanged.
        for field in ('tag_id', 'tag_offset_xyz_mm', 'frames'):
            if message.get(field) not in (None, ''):
                params.setdefault(field, message[field])
        point_type = message.get('point_type', message.get('pointType'))
        if point_type not in (None, ''):
            params.setdefault('point_type', int(point_type))
        return params

    def _handle_command(self, parsed):
        message = parsed.message
        command = parsed.command
        action = COMMAND_ACTIONS[command]
        # Do not queue reset/enable/drag retries behind a long vision task.
        # A vehicle may reconnect and resend while it is waiting; executing all
        # those stale commands later would cause repeated robot motion.
        if not self.resource_lock.acquire(blocking=False):
            result = {
                'action': action,
                'status': 'failed',
                'error_code': 'busy',
                'message': (
                    'another robot/vision task is running; command was not '
                    'queued'),
                'metrics': {},
                'artifacts': [],
            }
            return self._legacy_command_response(message, result, command)
        try:
            # ESTUN Codroid may return the movJ response only after the reset
            # motion completes.  Keep the legacy command alive longer than the
            # controller's 30 second motion-request timeout.
            timeout = 45.0 if command == 1 else 30.0
            result = self.rpc.run(
                action, {}, timeout, False,
                request_id=_request_id('nx-command-%d' % command))
        finally:
            self.resource_lock.release()
        return self._legacy_command_response(message, result, command)

    def _handle_teaching_command(self, parsed):
        """Forward a station-scoped Ref/Work/finish/cancel command."""
        message = parsed.message
        command = parsed.command
        params = self._executor_params(message)
        params['command'] = command
        timeout = 90.0 if command > 0 else 30.0
        teaching_command = (
            'cancel' if command == -1 else
            ('finish' if command == 0 else
             ('record_reference_or_work_1' if command == 1
              else 'record_work')))
        result = self._run({
            'action': 'vision_point_teach',
            'params': params,
            'timeout_sec': timeout,
            'dry_run': bool(message.get('dry_run', False)),
        })
        return self._legacy_command_response(
            message, result, command, teaching_command=teaching_command)

    def _handle_locked(self, parsed):
        message = parsed.message
        if parsed.kind == 'explicit':
            params = self._executor_params(message)
            if parsed.message_type == 'vision_station_execute':
                # A production station request always executes the complete
                # manifest.  Point type and label only belong to teaching and
                # stored point records.
                params.pop('point_type', None)
                params.pop('task_command', None)
            elif parsed.point_type is not None:
                params['point_type'] = parsed.point_type
            if ('command' in message and
                    parsed.message_type == 'vision_point_teach'):
                params.setdefault('command', message['command'])
            invocation = {
                'action': parsed.message_type,
                'params': params,
                'timeout_sec': float(message.get('timeout_sec', 200.0)),
                'dry_run': bool(message.get('dry_run', False)),
            }
            result = self._run(invocation)
            return self._legacy_response(
                message, result, completion=parsed.message_type not in {
                    'arm_status', 'vision_station_points',
                    'vision_point_teach', 'apriltag_locate',
                    'apriltag_validate'})

        if parsed.kind == 'query':
            params = self._executor_params(message)
            health = self.rpc.run(
                'health', {}, 2.0, True,
                request_id=_request_id('nx-type2-teach-state'))
            if health.get('status') != 'succeeded':
                return self._legacy_response(message, health)
            metrics = health.get('metrics', {})
            teaching_active = bool(metrics.get(
                'teaching_active',
                metrics.get('icp_teaching_active', False)))
            finish_result = None
            if teaching_active:
                active_mapid = metrics.get(
                    'teaching_mapid', metrics.get('icp_mapid'))
                active_poseid = metrics.get(
                    'teaching_poseid', metrics.get('icp_poseid'))
                if (str(active_mapid) != str(params.get('mapid')) or
                        str(active_poseid) != str(params.get('poseid'))):
                    raise RuntimeError(
                        'type2 station does not match active teaching: '
                        '%s/%s' % (active_mapid, active_poseid))
                finish_result = self._run({
                    'action': 'vision_point_teach',
                    'params': {'command': 0,
                               'mapid': params.get('mapid'),
                               'poseid': params.get('poseid')},
                    'timeout_sec': 30.0,
                    'dry_run': False,
                })
                if finish_result.get('status') != 'succeeded':
                    return self._legacy_response(message, finish_result)
            result = self._run({
                'action': 'vision_station_points',
                'params': params,
                'timeout_sec': 10.0,
                'dry_run': True,
            })
            if finish_result is not None and result.get('status') == 'succeeded':
                result.setdefault('metrics', {})['teaching_finished'] = True
                result['metrics']['finish_metrics'] = finish_result.get(
                    'metrics', {})
            return self._legacy_response(message, result)

        if parsed.kind == 'delete':
            params = self._executor_params(message)
            if parsed.command is None:
                params.pop('task_command', None)
            else:
                params['task_command'] = str(parsed.command)
            result = self._run({
                'action': 'vision_station_delete',
                'params': params,
                'timeout_sec': 30.0,
                'dry_run': bool(message.get('dry_run', False)),
            })
            return self._legacy_response(message, result)

        if parsed.kind == 'teaching_command':
            return self._handle_teaching_command(parsed)

        if parsed.kind == 'teaching_start':
            params = self._executor_params(message)
            params.pop('task_command', None)
            params['command'] = 'start'
            if parsed.point_type is not None:
                params['point_type'] = parsed.point_type
            result = self._run({
                'action': 'vision_point_teach',
                'params': params,
                'timeout_sec': float(message.get('timeout_sec', 30.0)),
                'dry_run': bool(message.get('dry_run', False)),
            })
            return self._legacy_response(message, result)

        if parsed.kind == 'production':
            params = self._executor_params(message)
            # Preserve the Asdun vehicle contract: mapid + poseid triggers all
            # saved points at that station in manifest order.
            params.pop('point_type', None)
            params.pop('task_command', None)
            invocation = {
                'action': 'vision_station_execute',
                'params': params,
                'timeout_sec': float(message.get(
                    'timeout_sec', self.config.default_timeout)),
                'dry_run': bool(message.get('dry_run', False)),
            }
            result = self._run_vehicle_production(invocation)
            return self._legacy_response(message, result, completion=True)

        raise RuntimeError('unsupported NX request kind')

    def handle(self, message):
        try:
            parsed = parse_nx_request(message)
        except Exception as error:
            return self._failure(message, error)

        try:
            # Permission cancellation intentionally bypasses the resource lock
            # so a second TCP connection can stop a long task at its next
            # checkpoint.
            if parsed.kind == 'permission':
                result = self.rpc.run(
                    parsed.message_type,
                    dict(message.get('params', {}) or {}), 30.0, False,
                    request_id=_request_id('nx-' + parsed.message_type))
                return self._legacy_response(message, result)
            if parsed.kind == 'command':
                return self._handle_command(parsed)
            with self.resource_lock:
                return self._handle_locked(parsed)
        except Exception as error:
            return self._failure(message, error)


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
                    # One connection is one ordered command stream. Processing
                    # it synchronously prevents response reordering and removes
                    # the old worker/half-close race. Other vehicle connections
                    # are still served concurrently by the accept loop.
                    self._process_and_send(
                        client, send_lock, message)
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
        print('NX result: action=%s status=%s error=%s message=%s' % (
            response.get('action', ''), response.get('status', ''),
            response.get('error_code', ''), response.get('message', '')),
            flush=True)
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
    rpc_host = '127.0.0.1' if args.rpc_host == '0.0.0.0' else args.rpc_host
    config = (
        GatewayConfig.load(os.path.expanduser(args.route_file))
        if args.route_file else GatewayConfig({}))
    rpc = VisionRpcClient(
        rpc_host, args.rpc_port, auth_token=args.rpc_auth_token)
    server = NxCompatServer(
        args.host, args.port, NxMessageHandler(config, rpc),
        _csv(args.allowed_clients))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == '__main__':
    main()
