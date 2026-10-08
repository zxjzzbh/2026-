"""Round records can exceed the Unix socket pathname limit without losing IPC."""
import json
import os
from pathlib import Path
import socket
import tempfile
import threading

import pytest

from carvision.manual_drive import WorkerDrive, worker_control_endpoint


def test_short_log_folder_keeps_existing_endpoint():
    # This pure lookup must neither create nor remove a socket folder.
    output = Path('/cv-round')
    assert worker_control_endpoint(output) == output.resolve()/'control.sock'


def test_long_round_log_folder_has_short_unique_endpoint_without_side_effects(tmp_path):
    output = tmp_path/('long-session-record-'*9)/'round-000001'
    assert len(os.fsencode(output/'control.sock')) > 107
    first = worker_control_endpoint(output)
    assert len(os.fsencode(first)) <= 103
    assert first == worker_control_endpoint(output)
    assert first != worker_control_endpoint(output.parent/'round-000002')
    assert not output.exists() and not first.parent.exists()


def test_invalid_socket_root_is_rejected_before_hardware_start(tmp_path):
    with pytest.raises(ValueError, match='too long'):
        worker_control_endpoint(tmp_path/('long-session-'*9), socket_root=tmp_path/('root-'*30))


@pytest.mark.skipif(os.name != 'posix', reason='actual Linux Unix-socket transport check')
def test_worker_status_connects_with_long_round_record_path(tmp_path):
    output = tmp_path/('nested-round-record-'*9)/'round-000001'
    with tempfile.TemporaryDirectory(prefix='cv-ipc-', dir='/tmp') as short_root:
        endpoint = worker_control_endpoint(output, socket_root=Path(short_root))
        endpoint.parent.mkdir(mode=0o700)
        assert endpoint.parent.stat().st_mode & 0o777 == 0o700
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(endpoint)); server.listen(1); server.settimeout(2)
            os.chmod(endpoint, 0o600)
            requests = []

            def respond():
                with server.accept()[0] as connection:
                    connection.settimeout(2)
                    with connection.makefile('rb') as stream:
                        requests.append(json.loads(stream.readline()))
                    connection.sendall(b'{"status":{"mode":"disabled","motor_pulse_us":1500}}\n')

            thread = threading.Thread(target=respond)
            thread.start()
            state = WorkerDrive(endpoint, steering_available=False).status()
            thread.join(timeout=3)
            assert not thread.is_alive() and requests[0]['action'] == 'status'
            assert state['mode'] == 'disabled' and state['motor_pulse_us'] == 1500
        endpoint.unlink()
        endpoint.parent.rmdir()
