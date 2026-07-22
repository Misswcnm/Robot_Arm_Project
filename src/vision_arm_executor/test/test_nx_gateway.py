import json
import socket
import threading
import unittest

from vision_arm_executor.nx_gateway import (
    JsonStreamDecoder, NxCompatServer, NxMessageHandler, RouteTable)


class FakeRpc:
    def __init__(self, status='succeeded'):
        self.status = status
        self.calls = []

    def run(self, action, params=None, timeout_sec=120.0, dry_run=False,
            request_id=None):
        self.calls.append({
            'action': action,
            'params': params,
            'timeout_sec': timeout_sec,
            'dry_run': dry_run,
            'request_id': request_id,
        })
        return {
            'request_id': request_id,
            'action': action,
            'status': self.status,
            'message': 'ok' if self.status == 'succeeded' else 'failed',
            'error_code': '' if self.status == 'succeeded' else 'test_error',
            'metrics': {},
            'artifacts': [],
        }


class BlockingRpc(FakeRpc):
    def __init__(self):
        super().__init__()
        self.started = threading.Event()
        self.release = threading.Event()

    def run(self, action, params=None, timeout_sec=120.0, dry_run=False,
            request_id=None):
        if action == 'apriltag_pick':
            self.started.set()
            self.release.wait(1.0)
        return super().run(
            action, params, timeout_sec, dry_run, request_id)


class JsonStreamDecoderTests(unittest.TestCase):
    def test_fragmented_and_concatenated_messages(self):
        decoder = JsonStreamDecoder()
        self.assertEqual([], decoder.feed(b'{"mapid":"m'))
        values = decoder.feed(
            b'","poseid":"p"}{"type":"mechanical_arm_command",'
            b'"command":2}\n')
        self.assertEqual('m', values[0]['mapid'])
        self.assertEqual(2, values[1]['command'])

    def test_utf8_split_across_packets(self):
        payload = json.dumps(
            {'mapid': '地图', 'poseid': '点位'}, ensure_ascii=False).encode(
                'utf-8')
        decoder = JsonStreamDecoder()
        split = payload.index('地'.encode('utf-8')) + 1
        self.assertEqual([], decoder.feed(payload[:split]))
        self.assertEqual('地图', decoder.feed(payload[split:])[0]['mapid'])


class RouteTableTests(unittest.TestCase):
    def test_most_specific_route_wins(self):
        routes = RouteTable({
            'routes': [
                {'mapid': '*', 'poseid': 'p', 'action': 'apriltag_locate'},
                {'mapid': 'm', 'poseid': 'p', 'label': 'pick',
                 'action': 'apriltag_pick'},
            ]})
        route = routes.match({'mapid': 'm', 'poseid': 'p', 'label': 'pick'})
        self.assertEqual('apriltag_pick', route['action'])

    def test_teaching_routes_are_separate(self):
        routes = RouteTable({
            'teaching_routes': [
                {'mapid': 'm', 'poseid': 'a',
                 'action': 'vision_icp_record_a'}],
            'routes': [
                {'mapid': 'm', 'poseid': 'a',
                 'action': 'vision_icp_align'}],
        })
        message = {'mapid': 'm', 'poseid': 'a'}
        self.assertEqual(
            'vision_icp_record_a',
            routes.match(message, teaching=True)['action'])
        self.assertEqual(
            'vision_icp_align', routes.match(message)['action'])


class HandlerTests(unittest.TestCase):
    def test_legacy_enable_calls_real_robot_enable(self):
        rpc = FakeRpc()
        handler = NxMessageHandler(RouteTable({}), rpc)
        response = handler.handle({
            'type': 'mechanical_arm_command', 'command': 2})
        self.assertEqual('robot_enable', rpc.calls[0]['action'])
        self.assertEqual('succeeded', response['status'])

    def test_execution_route_returns_done_only_on_success(self):
        rpc = FakeRpc()
        routes = RouteTable({'routes': [{
            'mapid': 'm', 'poseid': 'p',
            'action': 'vision_icp_align_and_move_b',
            'dry_run': False,
        }]})
        response = NxMessageHandler(routes, rpc).handle({
            'mapid': 'm', 'poseid': 'p'})
        self.assertEqual('done', response['status'])
        self.assertEqual('vision_icp_align_and_move_b', rpc.calls[0]['action'])
        self.assertFalse(rpc.calls[0]['dry_run'])

    def test_failed_execution_is_never_reported_done(self):
        rpc = FakeRpc(status='failed')
        routes = RouteTable({'routes': [{
            'mapid': 'm', 'poseid': 'p', 'action': 'apriltag_pick'}]})
        response = NxMessageHandler(routes, rpc).handle({
            'mapid': 'm', 'poseid': 'p'})
        self.assertEqual('failed', response['status'])
        self.assertEqual('test_error', response['error_code'])

    def test_missing_route_fails_closed(self):
        response = NxMessageHandler(RouteTable({}), FakeRpc()).handle({
            'mapid': 'unknown', 'poseid': 'unknown'})
        self.assertEqual('failed', response['status'])
        self.assertEqual('execution_route_not_found', response['error_code'])

    def test_manual_command_can_revoke_permission_during_long_task(self):
        rpc = BlockingRpc()
        routes = RouteTable({'routes': [{
            'mapid': 'm', 'poseid': 'pick', 'action': 'apriltag_pick'}]})
        handler = NxMessageHandler(routes, rpc)
        worker = threading.Thread(
            target=handler.handle, args=({'mapid': 'm', 'poseid': 'pick'},))
        worker.start()
        self.assertTrue(rpc.started.wait(.5))
        response = handler.handle({
            'type': 'mechanical_arm_command', 'command': 5})
        self.assertEqual('execution_disable', response['action'])
        rpc.release.set()
        worker.join(1.0)
        self.assertFalse(worker.is_alive())


class ServerTests(unittest.TestCase):
    def test_handler_exception_is_returned_as_failed_response(self):
        class BrokenHandler:
            @staticmethod
            def handle(unused_message):
                raise RuntimeError('vision RPC unavailable')

        server = NxCompatServer(
            '127.0.0.1', 0, BrokenHandler(), ['127.0.0.1/32'])
        server_side, client_side = socket.socketpair()
        try:
            server._process_and_send(
                server_side, threading.Lock(), {'mapid': 'm', 'poseid': 'p'})
            response = json.loads(client_side.recv(4096).decode('utf-8'))
        finally:
            server_side.close()
            client_side.close()
        self.assertEqual('failed', response['status'])
        self.assertEqual('nx_handler_error', response['error_code'])
        self.assertEqual('m', response['mapid'])


if __name__ == '__main__':
    unittest.main()
