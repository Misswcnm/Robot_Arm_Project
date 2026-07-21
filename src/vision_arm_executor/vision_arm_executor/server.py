import hmac
import ipaddress
import socket
import threading

from .protocol import decode_line, encode_line


class JsonLineServer:
    def __init__(self, host, port, handler, timeout=5.0, max_bytes=65536,
                 allowed_clients=None, auth_token=''):
        self.host = host
        self.port = int(port)
        self.handler = handler
        self.timeout = float(timeout)
        self.max_bytes = int(max_bytes)
        self.allowed_clients = [
            ipaddress.ip_network(item, strict=False)
            for item in (allowed_clients or ['127.0.0.1/32'])]
        self.auth_token = str(auth_token or '')
        self.stop = threading.Event()
        self.sock = None

    def start(self):
        if self.host not in ('127.0.0.1', '::1', 'localhost'):
            if not self.auth_token:
                raise RuntimeError(
                    'rpc_auth_token is required for non-loopback RPC')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(16)
        self.sock.settimeout(0.5)
        threading.Thread(target=self._accept, daemon=True).start()

    def close(self):
        self.stop.set()
        if self.sock:
            self.sock.close()

    def _allowed(self, address):
        peer = ipaddress.ip_address(address)
        return any(peer in network for network in self.allowed_clients)

    def _accept(self):
        while not self.stop.is_set():
            try:
                client, peer = self.sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            if not self._allowed(peer[0]):
                client.close()
                continue
            threading.Thread(
                target=self._client, args=(client,), daemon=True).start()

    def _authenticate(self, request):
        supplied = str(request.pop('auth_token', '') or '')
        if self.auth_token and not hmac.compare_digest(
                supplied, self.auth_token):
            raise ValueError('authentication_failed')

    def _client(self, client):
        try:
            client.settimeout(self.timeout)
            buffer = b''
            while not self.stop.is_set():
                chunk = client.recv(4096)
                if not chunk:
                    return
                buffer += chunk
                if len(buffer) > self.max_bytes:
                    raise ValueError('message_too_large')
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    if not line:
                        continue
                    request = decode_line(line, self.max_bytes)
                    self._authenticate(request)
                    client.sendall(encode_line(self.handler(request)))
                    # One request per connection keeps the ROS1/Python2 client
                    # and command-line nc workflow deterministic. Long tasks
                    # return accepted and are queried through task_status on a
                    # new connection.
                    return
        except socket.timeout:
            # Idle connection expiry is a transport close, not a task failure.
            return
        except Exception as error:
            try:
                client.sendall(encode_line({
                    'backend': 'vision',
                    'status': 'failed',
                    'error_code': 'rpc_error',
                    'message': str(error),
                }))
            except Exception:
                pass
        finally:
            client.close()
