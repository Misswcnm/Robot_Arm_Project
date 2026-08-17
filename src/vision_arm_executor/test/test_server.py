import unittest

from vision_arm_executor.server import JsonLineServer


class ServerSecurityTests(unittest.TestCase):
    def test_non_loopback_requires_token(self):
        server = JsonLineServer(
            '0.0.0.0', 0, lambda request: request,
            allowed_clients=['192.168.2.10/32'])
        with self.assertRaises(RuntimeError):
            server.start()

    def test_client_allowlist(self):
        server = JsonLineServer(
            '127.0.0.1', 0, lambda request: request,
            allowed_clients=['192.168.2.10/32'])
        self.assertTrue(server._allowed('192.168.2.10'))
        self.assertFalse(server._allowed('192.168.2.11'))

    def test_token_is_removed_before_dispatch(self):
        server = JsonLineServer(
            '127.0.0.1', 0, lambda request: request,
            auth_token='secret')
        request = {'auth_token': 'secret', 'action': 'health'}
        server._authenticate(request)
        self.assertNotIn('auth_token', request)
