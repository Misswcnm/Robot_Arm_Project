#!/usr/bin/env python3
# coding=utf-8

import json
import os
import socket
import sys
import threading
import time
import unittest


MODULE_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'src', 'master_node'))
sys.path.insert(0, MODULE_DIR)

from mechanical_arm_controller import MechanicalArmController


class OneRequestServer(threading.Thread):
    def __init__(self, response):
        threading.Thread.__init__(self)
        self.daemon = True
        self.response = response
        self.request = None
        self.ready = threading.Event()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(('127.0.0.1', 0))
        self.port = self.server.getsockname()[1]

    def run(self):
        self.server.listen(1)
        self.ready.set()
        client, _ = self.server.accept()
        try:
            buffer = b''
            while b'\n' not in buffer:
                chunk = client.recv(4096)
                if not chunk:
                    break
                buffer += chunk
            self.request = json.loads(
                buffer.split(b'\n', 1)[0].decode('utf-8'))
            client.sendall(
                (json.dumps(self.response) + '\n').encode('utf-8'))
        finally:
            client.close()
            self.server.close()


def bare_controller():
    controller = MechanicalArmController.__new__(MechanicalArmController)
    controller.nx_host = '127.0.0.1'
    controller.nx_port = 0
    controller.arm_vendor = 'dobot'
    controller.mechanical_arm_completed = False
    controller.last_completion_response = None
    controller.completion_condition = threading.Condition()
    return controller


class MechanicalArmControllerTest(unittest.TestCase):
    def test_synchronous_request_uses_newline_json_and_returns_response(self):
        expected = {
            'type': 'response',
            'status': 'succeeded',
            'metrics': {'points': []},
        }
        server = OneRequestServer(expected)
        server.start()
        server.ready.wait(1)

        controller = bare_controller()
        controller.nx_port = server.port
        result = controller.send_request_and_wait({
            'type': 2, 'mapid': 'map-1', 'poseid': 'station-1'
        }, 2)
        server.join(1)

        self.assertEqual(expected, result)
        self.assertEqual({
            'type': 2, 'mapid': 'map-1', 'poseid': 'station-1'
        }, server.request)

    def test_teaching_start_builds_current_icp_contract(self):
        controller = bare_controller()
        captured = {}

        def send(message, timeout):
            captured['message'] = message
            captured['timeout'] = timeout
            return {'status': 'succeeded'}

        controller.send_request_and_wait = send
        result = controller.send_teaching_start(
            'map-1', 'station-1', 1, {}, 30)

        self.assertEqual('succeeded', result['status'])
        self.assertEqual({
            'type': 1,
            'mapid': 'map-1',
            'poseid': 'station-1',
            'point_type': 1,
            'params': {}
        }, captured['message'])
        self.assertEqual(30, captured['timeout'])

    def test_apriltag_teaching_requires_tag_id_and_defaults_offset(self):
        controller = bare_controller()
        missing = controller.send_teaching_start(
            'map-1', 'station-1', 0, {})
        self.assertEqual('invalid_request', missing['error_code'])

        captured = {}
        controller.send_request_and_wait = lambda message, timeout: (
            captured.setdefault('message', message) or
            {'status': 'succeeded'})
        controller.send_teaching_start(
            'map-1', 'station-1', 0, {'tag_id': 7})
        self.assertEqual(
            [0, 0, 0],
            captured['message']['params']['tag_offset_xyz_mm'])

    def test_production_wait_returns_on_done_and_failed_terminal_states(self):
        for status, expected in (('done', True), ('failed', False)):
            controller = bare_controller()

            def finish():
                time.sleep(0.02)
                controller._handle_status_message({
                    'type': 'response',
                    'status': status
                })

            worker = threading.Thread(target=finish)
            worker.start()
            self.assertEqual(expected, controller._wait_for_completion(1))
            worker.join(1)

    def test_production_request_contains_only_station_identity(self):
        controller = bare_controller()
        captured = {}
        controller._send_message = lambda message, **kwargs: (
            captured.setdefault('message', message) is not None)

        self.assertTrue(controller.send_navigation_point_reached(
            'map-1', 'station-1'))
        self.assertEqual({
            'mapid': 'map-1',
            'poseid': 'station-1'
        }, captured['message'])


if __name__ == '__main__':
    unittest.main()
