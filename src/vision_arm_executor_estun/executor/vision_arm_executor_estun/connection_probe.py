"""Read-only network probe for ESTUN Codroid and ERI endpoints."""

import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import time


WEBSOCKET_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


def tcp_probe(host, port, timeout):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True, 'open'
    except OSError as error:
        return False, str(error)


def _receive_exact(connection, size):
    result = bytearray()
    while len(result) < size:
        block = connection.recv(size - len(result))
        if not block:
            raise RuntimeError('WebSocket closed before response completed')
        result.extend(block)
    return bytes(result)


def _client_text_frame(text):
    payload = text.encode('utf-8')
    mask = os.urandom(4)
    header = bytearray([0x81])
    length = len(payload)
    if length < 126:
        header.append(0x80 | length)
    elif length <= 0xffff:
        header.append(0x80 | 126)
        header.extend(struct.pack('!H', length))
    else:
        header.append(0x80 | 127)
        header.extend(struct.pack('!Q', length))
    header.extend(mask)
    header.extend(
        value ^ mask[index % 4] for index, value in enumerate(payload))
    return bytes(header)


def _server_frame(connection):
    first, second = _receive_exact(connection, 2)
    opcode = first & 0x0f
    masked = bool(second & 0x80)
    length = second & 0x7f
    if length == 126:
        length = struct.unpack('!H', _receive_exact(connection, 2))[0]
    elif length == 127:
        length = struct.unpack('!Q', _receive_exact(connection, 8))[0]
    mask = _receive_exact(connection, 4) if masked else None
    payload = bytearray(_receive_exact(connection, length))
    if mask:
        for index in range(len(payload)):
            payload[index] ^= mask[index % 4]
    return opcode, bytes(payload)


def codroid_robot_state(host, port=9000, timeout=4.0):
    """Query only Robot/Control/state using the old project's protocol."""
    with socket.create_connection((host, port), timeout=timeout) as connection:
        connection.settimeout(timeout)
        key = base64.b64encode(os.urandom(16)).decode('ascii')
        request = (
            'GET / HTTP/1.1\r\n'
            'Host: %s:%d\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            'Sec-WebSocket-Key: %s\r\n'
            'Sec-WebSocket-Version: 13\r\n\r\n' % (host, port, key))
        connection.sendall(request.encode('ascii'))
        headers = bytearray()
        while b'\r\n\r\n' not in headers:
            block = connection.recv(4096)
            if not block:
                raise RuntimeError('WebSocket handshake closed by peer')
            headers.extend(block)
            if len(headers) > 16384:
                raise RuntimeError('WebSocket handshake is too large')
        header_text = headers.decode('iso-8859-1')
        if ' 101 ' not in header_text.split('\r\n', 1)[0]:
            raise RuntimeError(
                'server rejected WebSocket handshake: ' +
                header_text.split('\r\n', 1)[0])
        expected = base64.b64encode(hashlib.sha1(
            (key + WEBSOCKET_GUID).encode('ascii')).digest()).decode('ascii')
        if ('sec-websocket-accept: ' + expected).lower() not in \
                header_text.lower():
            raise RuntimeError('invalid WebSocket handshake response')

        payload = {
            'id': 1,
            'type': 'common',
            'action': 'getparam',
            'data': ['Robot/Control/state'],
        }
        connection.sendall(_client_text_frame(json.dumps(payload)))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            opcode, response = _server_frame(connection)
            if opcode == 0x1:
                return json.loads(response.decode('utf-8'))
            if opcode == 0x8:
                raise RuntimeError('WebSocket closed before state response')
        raise RuntimeError('Codroid state query timed out')


def udp_bind_probe(port):
    connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        connection.bind(('0.0.0.0', port))
        return True, 'local UDP port is available'
    except OSError as error:
        return False, str(error)
    finally:
        connection.close()


def parser():
    result = argparse.ArgumentParser(
        description='Read-only ESTUN Codroid/ERI connection probe')
    result.add_argument('--host', default='192.168.2.5')
    result.add_argument('--codroid-port', type=int, default=9000)
    result.add_argument('--eri-cmd-port', type=int, default=61210)
    result.add_argument('--eri-servo-port', type=int, default=61211)
    result.add_argument('--eri-status-port', type=int, default=61212)
    result.add_argument('--timeout-sec', type=float, default=3.0)
    return result


def main(args=None):
    options = parser().parse_args(args)
    checks = [
        ('Codroid TCP', options.codroid_port),
        ('ERI CMD TCP', options.eri_cmd_port),
        ('ERI SERVO TCP', options.eri_servo_port),
    ]
    for name, port in checks:
        success, detail = tcp_probe(
            options.host, port, options.timeout_sec)
        print('%-16s %s:%d  %s (%s)' % (
            name, options.host, port,
            'OK' if success else 'FAILED', detail))
    success, detail = udp_bind_probe(options.eri_status_port)
    print('%-16s 0.0.0.0:%d  %s (%s)' % (
        'ERI STATUS UDP', options.eri_status_port,
        'OK' if success else 'FAILED', detail))
    try:
        response = codroid_robot_state(
            options.host, options.codroid_port, options.timeout_sec)
        print('Codroid只读状态返回:')
        print(json.dumps(response, ensure_ascii=False, indent=2))
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print('Codroid只读状态查询失败: %s' % error)


if __name__ == '__main__':
    main()
