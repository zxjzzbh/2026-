"""Actual byte relay check: loopback only, one request, no control replay."""
import importlib.machinery
import importlib.util
from pathlib import Path
import socket
import threading
from types import SimpleNamespace
import pytest


def launcher():
    path=Path(__file__).parents[2]/'pc-launcher/oneclick.pyw'
    loader=importlib.machinery.SourceFileLoader('pc_test_launcher',str(path))
    spec=importlib.util.spec_from_loader(loader.name,loader)
    module=importlib.util.module_from_spec(spec);loader.exec_module(module)
    return module


def test_forwarder_sends_one_fresh_request_and_does_not_replay_on_disconnect(monkeypatch):
    module=launcher()
    no_delay = []
    original_select = module.select.select
    def inspect_select(read, write, exceptional, timeout):
        no_delay.append([s.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY) for s in read])
        return original_select(read, write, exceptional, timeout)
    monkeypatch.setattr(module, 'select', SimpleNamespace(select=inspect_select))
    with socket.socket() as upstream:
        upstream.bind(('127.0.0.1',0));upstream.listen(2);upstream.settimeout(2)
        received=[]
        def respond():
            with upstream.accept()[0] as conn:
                conn.settimeout(2);received.append(conn.recv(4096));conn.sendall(b'reply')
        thread=threading.Thread(target=respond);thread.start()
        with module.Relay(port=0,upstream=upstream.getsockname()) as relay:
            assert relay.server_address[0]=='127.0.0.1'
            server=threading.Thread(target=relay.serve_forever);server.start()
            try:
                with socket.create_connection(relay.server_address,timeout=2) as client:
                    client.sendall(b'fresh-control');assert client.recv(4096)==b'reply'
            finally:relay.shutdown();server.join(2)
        thread.join(2)
        assert received==[b'fresh-control'] and not thread.is_alive()
        assert no_delay and all(options == [1, 1] for options in no_delay)


def test_existing_port_must_match_current_car_session(monkeypatch):
    module=launcher()
    monkeypatch.setattr(module,'car_status',lambda *args:{'control_session_id':'old'})
    assert not module.existing_relay_matches({'control_session_id':'new'})
    monkeypatch.setattr(module,'car_status',lambda *args:{'control_session_id':'new'})
    assert module.existing_relay_matches({'control_session_id':'new'})
    assert not module.existing_relay_matches({})


def test_second_launcher_cannot_steal_an_active_relay_port():
    module = launcher()
    with module.Relay(port=0) as first:
        if module.os.name == 'nt':
            assert first.socket.getsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE) == 1
        with pytest.raises(OSError):
            module.Relay(port=first.server_address[1])


def test_missing_public_host_fails_before_any_network_or_gui(monkeypatch):
    monkeypatch.delenv('SMARTCAR_HOST', raising=False)
    module = launcher()
    monkeypatch.setattr('sys.argv', ['oneclick.pyw', '--check'])
    def forbidden(*args, **kwargs):
        pytest.fail('missing configuration must not contact a vehicle')
    monkeypatch.setattr(module, 'car_status', forbidden)
    monkeypatch.setattr(module, 'connect_dashboard', forbidden)
    with pytest.raises(SystemExit) as failure:
        module.main()
    assert failure.value.code == 2


def test_public_launcher_uses_configured_host_for_status_and_relay(monkeypatch):
    monkeypatch.setenv('SMARTCAR_HOST', 'smartcar.example.invalid')
    module = launcher()
    urls = []
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self): return b'{"boot_test":true}'
    def read_status(url, timeout):
        urls.append(url)
        return Response()
    monkeypatch.setattr(module.OPENER, 'open', read_status)
    assert module.car_status() == {'boot_test': True}
    assert urls == ['http://smartcar.example.invalid:8080/api/drive/status']
    with module.Relay(port=0) as relay:
        assert relay.upstream == ('smartcar.example.invalid', 8080)
