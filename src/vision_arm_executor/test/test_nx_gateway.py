import json
import socket
import threading
import time
import unittest

from vision_arm_executor.nx_gateway import (
    GatewayConfig, JsonStreamDecoder, NxCompatServer, NxMessageHandler)


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
        if action == 'vision_station_execute':
            self.started.set()
            self.release.wait(1.0)
        return super().run(
            action, params, timeout_sec, dry_run, request_id)


class TeachingRpc(FakeRpc):
    def __init__(self, point_type=1):
        super().__init__()
        self.teaching_active = True
        self.point_type = point_type

    def run(self, action, params=None, timeout_sec=120.0, dry_run=False,
            request_id=None):
        result = super().run(
            action, params, timeout_sec, dry_run, request_id)
        if action == 'health':
            result['metrics'].update(
                teaching_active=self.teaching_active,
                teaching_point_type=self.point_type)
        elif action == 'vision_point_teach' and int(params['command']) in {
                2, 3}:
            self.teaching_active = False
        return result


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


class GatewayConfigTests(unittest.TestCase):
    def test_timeout_default_is_read(self):
        config = GatewayConfig({'defaults': {'timeout_sec': 45}})
        self.assertEqual(45.0, config.default_timeout)

    def test_removed_route_fields_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'unsupported gateway'):
            GatewayConfig({'operation_mode': 1})


class HandlerTests(unittest.TestCase):
    def test_legacy_enable_calls_real_robot_enable(self):
        rpc = FakeRpc()
        handler = NxMessageHandler(GatewayConfig({}), rpc)
        response = handler.handle({
            'type': 'mechanical_arm_command', 'command': 2})
        self.assertEqual(
            ['health', 'robot_enable'],
            [call['action'] for call in rpc.calls])
        self.assertEqual('succeeded', response['status'])
        self.assertEqual(2, response['command'])
        self.assertEqual('success', response['shangdian'])

    def test_legacy_reset_timeout_exceeds_codroid_motion_timeout(self):
        rpc = FakeRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'type': 'mechanical_arm_command', 'command': 1})
        self.assertEqual('succeeded', response['status'])
        self.assertEqual('robot_reset', rpc.calls[-1]['action'])
        self.assertEqual(45.0, rpc.calls[-1]['timeout_sec'])

    def test_all_six_idle_commands_match_cr5_actions_and_fields(self):
        expected = {
            1: ('robot_reset', 'fuwei'),
            2: ('robot_enable', 'shangdian'),
            3: ('robot_disable', 'xiadian'),
            4: ('robot_start_drag', 'tuozhuai'),
            5: ('robot_stop_drag', 'quxiaotuozhuai'),
            6: ('robot_clear_error', 'qingchu'),
        }
        for command, (action, field) in expected.items():
            with self.subTest(command=command):
                rpc = FakeRpc()
                response = NxMessageHandler(GatewayConfig({}), rpc).handle({
                    'type': 'mechanical_arm_command',
                    'command': command})
                self.assertEqual(action, rpc.calls[-1]['action'])
                self.assertEqual(command, response['command'])
                self.assertEqual('success', response[field])
                if command == 4:
                    self.assertEqual('success', response['zidong'])
                if command == 5:
                    self.assertEqual('success', response['shoudong'])

    def test_asdun_command_precedence_does_not_require_type(self):
        rpc = FakeRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'command': 6, 'mapid': 'ignored', 'poseid': 'ignored'})
        self.assertEqual('robot_clear_error', rpc.calls[-1]['action'])
        self.assertEqual('success', response['qingchu'])

    def test_failed_enable_has_correct_and_historical_typo_fields(self):
        response = NxMessageHandler(
            GatewayConfig({}), FakeRpc(status='failed')).handle({
                'type': 'mechanical_arm_enable', 'command': 2})
        self.assertEqual('failed', response['shangdian'])
        self.assertEqual('failed', response['sahngdian'])

    def test_active_teaching_reinterprets_legacy_commands_one_and_two(self):
        rpc = TeachingRpc()
        handler = NxMessageHandler(GatewayConfig({}), rpc)
        first = handler.handle({
            'type': 'mechanical_arm_command', 'command': 1,
            'params': {'frames': 5}})
        second = handler.handle({
            'type': 'mechanical_arm_command', 'command': 2})
        self.assertEqual('succeeded', first['status'])
        self.assertEqual('succeeded', second['status'])
        teaching_calls = [
            call for call in rpc.calls
            if call['action'] == 'vision_point_teach']
        self.assertEqual([1, 2], [
            call['params']['command'] for call in teaching_calls])
        self.assertNotIn('frames', teaching_calls[0]['params'])
        self.assertFalse(rpc.teaching_active)
        self.assertEqual('record_a', first['teaching_command'])
        self.assertEqual('record_b', second['teaching_command'])
        self.assertNotIn('fuwei', first)
        self.assertNotIn('shangdian', second)

    def test_drag_commands_are_idempotent_or_abort_during_teaching(self):
        rpc = TeachingRpc()
        handler = NxMessageHandler(GatewayConfig({}), rpc)
        start = handler.handle({
            'type': 'mechanical_arm_command', 'command': 4})
        stop = handler.handle({
            'type': 'mechanical_arm_command', 'command': 5})
        teaching_calls = [
            call for call in rpc.calls
            if call['action'] == 'vision_point_teach']
        self.assertEqual([4, 3], [
            call['params']['command'] for call in teaching_calls])
        self.assertEqual('drag_already_active', start['teaching_command'])
        self.assertEqual('success', start['tuozhuai'])
        self.assertEqual('abort', stop['teaching_command'])
        self.assertEqual('success', stop['quxiaotuozhuai'])

    def test_execution_route_returns_done_only_on_success(self):
        rpc = FakeRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'mapid': 'm', 'poseid': 'p'})
        self.assertEqual('done', response['status'])
        self.assertEqual(
            ['execution_enable', 'vision_station_execute'],
            [call['action'] for call in rpc.calls])
        self.assertFalse(rpc.calls[1]['dry_run'])

    def test_real_vehicle_task_does_not_cycle_permission_after_failure(self):
        class FailingTaskRpc(FakeRpc):
            def run(self, action, params=None, timeout_sec=120.0,
                    dry_run=False, request_id=None):
                result = super().run(
                    action, params, timeout_sec, dry_run, request_id)
                if action == 'vision_station_execute':
                    result.update(
                        status='failed', error_code='motion_failed',
                        message='failed')
                return result

        rpc = FailingTaskRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'mapid': 'm', 'poseid': 'p'})
        self.assertEqual('failed', response['status'])
        self.assertEqual('motion_failed', response['error_code'])
        self.assertEqual(
            ['execution_enable', 'vision_station_execute'],
            [call['action'] for call in rpc.calls])

    def test_failed_execution_is_never_reported_done(self):
        rpc = FakeRpc(status='failed')
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'mapid': 'm', 'poseid': 'p'})
        self.assertEqual('failed', response['status'])
        self.assertEqual('test_error', response['error_code'])

    def test_backend_can_send_numeric_teaching_command(self):
        rpc = FakeRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'type': 'vision_point_teach', 'command': 1,
            'point_type': 1,
            'mapid': 'm', 'poseid': 'p', 'label': 'P2'})
        self.assertEqual('vision_point_teach', rpc.calls[0]['action'])
        self.assertEqual(1, rpc.calls[0]['params']['command'])
        self.assertEqual('m', rpc.calls[0]['params']['mapid'])
        self.assertEqual('p', rpc.calls[0]['params']['poseid'])
        self.assertEqual(1, rpc.calls[0]['params']['point_type'])
        self.assertEqual('P2', rpc.calls[0]['params']['label'])
        self.assertEqual('succeeded', response['status'])

    def test_type1_starts_scoped_icp_teaching(self):
        rpc = FakeRpc()
        response = NxMessageHandler(
            GatewayConfig({}), rpc).handle({
                'type': 'type1', 'mapid': 'm', 'poseid': 'p',
                'point_type': 1})
        self.assertEqual('succeeded', response['status'])
        self.assertEqual('vision_point_teach', rpc.calls[0]['action'])
        self.assertEqual('start', rpc.calls[0]['params']['command'])
        self.assertEqual('m', rpc.calls[0]['params']['mapid'])
        self.assertEqual('p', rpc.calls[0]['params']['poseid'])
        self.assertEqual(1, rpc.calls[0]['params']['point_type'])

    def test_type2_string_and_numeric_forms_only_query_points(self):
        for message_type in ('type2', '2', 2):
            with self.subTest(message_type=message_type):
                rpc = FakeRpc()
                response = NxMessageHandler(
                    GatewayConfig({}), rpc).handle({
                        'type': message_type, 'mapid': 'm', 'poseid': 'p'})
                self.assertEqual('succeeded', response['status'])
                self.assertEqual(1, len(rpc.calls))
                self.assertEqual(
                    'vision_station_points', rpc.calls[0]['action'])

    def test_numeric_type1_starts_teaching(self):
        rpc = FakeRpc()
        response = NxMessageHandler(
            GatewayConfig({}), rpc).handle({
                'type': 1, 'mapid': 'm', 'poseid': 'p',
                'point_type': 1})
        self.assertEqual('succeeded', response['status'])
        self.assertEqual('vision_point_teach', rpc.calls[0]['action'])
        self.assertEqual('start', rpc.calls[0]['params']['command'])

    def test_production_forwards_only_station_identity(self):
        rpc = FakeRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'mapid': 'm', 'poseid': 'p', 'label': 'P1',
            'point_type': 99})
        self.assertEqual('done', response['status'])
        self.assertEqual('vision_station_execute', rpc.calls[1]['action'])
        self.assertEqual(
            {'mapid': 'm', 'poseid': 'p'},
            rpc.calls[1]['params'])
        self.assertNotIn('label', response)
        self.assertNotIn('point_type', response)

    def test_explicit_execution_disable_can_cancel_during_long_task(self):
        rpc = BlockingRpc()
        handler = NxMessageHandler(GatewayConfig({}), rpc)
        worker = threading.Thread(
            target=handler.handle, args=({
                'mapid': 'm', 'poseid': 'pick'},))
        worker.start()
        self.assertTrue(rpc.started.wait(.5))
        response = handler.handle({
            'type': 'execution_disable'})
        self.assertEqual('execution_disable', response['action'])
        rpc.release.set()
        worker.join(1.0)
        self.assertFalse(worker.is_alive())

    def test_type1_apriltag_forwards_type_and_offset(self):
        rpc = FakeRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'type': 'type1',
            'mapid': 'm',
            'poseid': 'tag',
            'point_type': 0,
            'params': {
                'tag_id': 0,
                'tag_offset_xyz_mm': [10, 20, 0],
            },
        })
        self.assertEqual('succeeded', response['status'])
        call = rpc.calls[0]
        self.assertEqual('vision_point_teach', call['action'])
        self.assertEqual(0, call['params']['point_type'])
        self.assertEqual([10, 20, 0], call['params']['tag_offset_xyz_mm'])
        self.assertEqual(1, len(rpc.calls))
        self.assertNotIn(
            'vision_station_points',
            [current['action'] for current in rpc.calls])

    def test_apriltag_record_command_does_not_query_saved_points(self):
        rpc = TeachingRpc(point_type=0)
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'type': 'mechanical_arm_command',
            'command': 1,
        })
        self.assertEqual('succeeded', response['status'])
        self.assertEqual(
            ['health', 'vision_point_teach'],
            [call['action'] for call in rpc.calls])
        self.assertNotIn(
            'vision_station_points',
            [call['action'] for call in rpc.calls])

    def test_active_apriltag_teaching_command_one_records_pose(self):
        rpc = TeachingRpc(point_type=0)
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'type': 'mechanical_arm_command', 'command': 1})
        teaching = [
            call for call in rpc.calls
            if call['action'] == 'vision_point_teach'][0]
        self.assertEqual(0, teaching['params']['point_type'])
        self.assertEqual('record_apriltag_pose', response['teaching_command'])
        self.assertNotIn('fuwei', response)

    def test_invalid_point_type_fails_before_rpc(self):
        rpc = FakeRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'type': 'type1', 'mapid': 'm', 'poseid': 'p',
            'point_type': 2})
        self.assertEqual('failed', response['status'])
        self.assertEqual('invalid_request', response['error_code'])
        self.assertEqual([], rpc.calls)

    def test_missing_point_type_fails_before_rpc(self):
        rpc = FakeRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'type': 'type1', 'mapid': 'm', 'poseid': 'p'})
        self.assertEqual('failed', response['status'])
        self.assertEqual('invalid_request', response['error_code'])
        self.assertEqual([], rpc.calls)

    def test_explicit_station_execution_ignores_point_filter_fields(self):
        rpc = FakeRpc()
        response = NxMessageHandler(GatewayConfig({}), rpc).handle({
            'type': 'vision_station_execute',
            'mapid': 'm',
            'poseid': 'p',
            'label': 'P1',
            'params': {'pointType': 99},
            'dry_run': True,
        })
        self.assertEqual('done', response['status'])
        self.assertEqual(
            {'mapid': 'm', 'poseid': 'p'},
            rpc.calls[0]['params'])
        self.assertNotIn('pointType', rpc.calls[0]['params'])


class ServerTests(unittest.TestCase):
    def test_half_close_waits_for_response_then_closes_connection(self):
        class DelayedHandler:
            @staticmethod
            def handle(unused_message):
                __import__('time').sleep(.05)
                return {
                    'type': 'response', 'status': 'succeeded',
                    'action': 'arm_status'}

        server = NxCompatServer(
            '127.0.0.1', 0, DelayedHandler(), ['127.0.0.1/32'])
        server_side, client_side = socket.socketpair()
        worker = threading.Thread(
            target=server._client,
            args=(server_side, ('127.0.0.1', 12345)))
        worker.start()
        try:
            client_side.sendall(b'{"type":"arm_status"}\n')
            client_side.shutdown(socket.SHUT_WR)
            received = b''
            while True:
                chunk = client_side.recv(4096)
                if not chunk:
                    break
                received += chunk
        finally:
            client_side.close()
        worker.join(1.0)
        self.assertFalse(worker.is_alive())
        response = json.loads(received.decode('utf-8'))
        self.assertEqual('succeeded', response['status'])

    def test_concatenated_requests_keep_response_order(self):
        class OrderedHandler:
            @staticmethod
            def handle(message):
                if message['sequence'] == 1:
                    time.sleep(.05)
                return {
                    'type': 'response',
                    'status': 'succeeded',
                    'sequence': message['sequence'],
                }

        server = NxCompatServer(
            '127.0.0.1', 0, OrderedHandler(), ['127.0.0.1/32'])
        server_side, client_side = socket.socketpair()
        worker = threading.Thread(
            target=server._client,
            args=(server_side, ('127.0.0.1', 12345)))
        worker.start()
        try:
            client_side.sendall(
                b'{"sequence":1}\n{"sequence":2}\n')
            client_side.shutdown(socket.SHUT_WR)
            received = b''
            while True:
                chunk = client_side.recv(4096)
                if not chunk:
                    break
                received += chunk
        finally:
            client_side.close()
        worker.join(1.0)
        self.assertFalse(worker.is_alive())
        responses = [
            json.loads(line) for line in received.decode('utf-8').splitlines()]
        self.assertEqual([1, 2], [
            response['sequence'] for response in responses])

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
