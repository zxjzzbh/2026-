"""Exercise real sockets without ever sending vehicle commands."""
import importlib.util
import socket
import socketserver
import threading
from contextlib import contextmanager
from pathlib import Path


spec = importlib.util.spec_from_file_location('console_forward', Path(__file__).parents[1] / 'tools/console_forward.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@contextmanager
def running(server):
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    thread.start()
    try:
        yield server.server_address
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def receive_until(sock, ending):
    data = b''
    while not data.endswith(ending):
        chunk = sock.recv(4096)
        if not chunk:
            break
        data += chunk
    return data


def test_forwards_origin_body_and_stream_bytes_unchanged():
    payload = b'POST /test HTTP/1.1\r\nHost: 127.0.0.1:8082\r\nOrigin: http://127.0.0.1:8082\r\nContent-Length: 2\r\n\r\n{}'
    response = b'HTTP/1.1 200 OK\r\nContent-Length: 4\r\n\r\n\x00\xffok'
    captured = []

    class Echo(socketserver.BaseRequestHandler):
        def handle(self):
            self.request.settimeout(2)
            captured.append(receive_until(self.request, b'{}'))
            self.request.sendall(response[:20])
            self.request.sendall(response[20:])

    with running(socketserver.ThreadingTCPServer(('127.0.0.1', 0), Echo)) as target:
        relay = module.ConsoleForward(('127.0.0.1', 0), module.Connection)
        relay.vehicle_address = target
        with running(relay) as address, socket.create_connection(address, timeout=2) as client:
            client.sendall(payload)
            assert receive_until(client, b'\x00\xffok') == response
    assert captured == [payload]  # No rewritten host/origin or duplicated request.


def test_upstream_disconnect_is_not_retried():
    captured = []

    class Disconnect(socketserver.BaseRequestHandler):
        def handle(self):
            captured.append(self.request.recv(4096))

    with running(socketserver.ThreadingTCPServer(('127.0.0.1', 0), Disconnect)) as target:
        relay = module.ConsoleForward(('127.0.0.1', 0), module.Connection)
        relay.vehicle_address = target
        with running(relay) as address, socket.create_connection(address, timeout=2) as client:
            client.sendall(b'GET /test HTTP/1.0\r\n\r\n')
            assert client.recv(4096) == b''
    assert len(captured) == 1


def test_offline_vehicle_returns_readable_503():
    with socket.socket() as unavailable:
        unavailable.bind(('127.0.0.1', 0))  # Bound but not listening.
        relay = module.ConsoleForward(('127.0.0.1', 0), module.Connection)
        relay.vehicle_address = unavailable.getsockname()
        with running(relay) as address, socket.create_connection(address, timeout=8) as client:
            client.sendall(b'GET / HTTP/1.0\r\n\r\n')
            response = receive_until(client, b'</button>')
    assert response.startswith(b'HTTP/1.1 503 Service Unavailable\r\n')
    assert '本机入口已启动，车端暂未连通'.encode() in response
