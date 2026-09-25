import json
import threading
import time
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from carvision.web_preview import PreviewState, make_server


def observation():
    return {"frame_id": 5, "receive_monotonic_ms": time.monotonic()*1000,
            "lane": {"valid": False, "offset_normalized": None},
            "detector_status": "disabled", "processing_ms": 5.0}


@pytest.fixture
def server():
    state = PreviewState(stale_ms=5000)
    http = make_server(('127.0.0.1', 0), state)
    thread = threading.Thread(target=http.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
    thread.start()
    yield state, f'http://127.0.0.1:{http.server_port}'
    state.finish()
    http.shutdown()
    http.server_close()
    thread.join(timeout=2)


def test_http_waits_for_camera_and_does_not_serve_missing_image(server):
    state, base = server
    with urlopen(base) as response:
        assert '识别叠加画面' in response.read().decode('utf-8')
    with urlopen(base+'/api/status') as response:
        assert json.load(response)['state'] == 'starting'
    with pytest.raises(HTTPError) as error:
        urlopen(base+'/frame.jpg')
    assert error.value.code == 503


def test_http_view_routing_and_multiple_stream_consumers(server):
    state, base = server
    # Transport fixtures only; real-image decoding is checked separately.
    state.publish({'raw': b'raw-jpeg', 'processed': b'processed-jpeg', 'mask': b'mask-jpeg'}, observation())
    for view, payload in [('raw', b'raw-jpeg'), ('processed', b'processed-jpeg'), ('mask', b'mask-jpeg')]:
        with urlopen(base+'/frame.jpg?view='+view) as response:
            assert response.headers['Content-Type'] == 'image/jpeg'
            assert response.read() == payload
    for _ in range(2):
        with urlopen(base+'/stream.mjpg?view=processed', timeout=3) as response:
            assert response.readline() == b'--frame\r\n'
            assert response.readline() == b'Content-Type: image/jpeg\r\n'
            length = int(response.readline().split(b':')[1])
            assert response.readline() == b'\r\n'
            assert response.read(length) == b'processed-jpeg'
    with pytest.raises(HTTPError) as error:
        urlopen(base+'/frame.jpg?view=../../secrets')
    assert error.value.code == 400


def test_stale_and_failed_frames_cannot_be_served_as_live(server):
    state, base = server
    result = observation()
    result['receive_monotonic_ms'] -= 6000
    state.publish({'processed': b'old-jpeg'}, result)
    with urlopen(base+'/api/status') as response:
        status = json.load(response)
    assert status['state'] == 'stale' and status['result'] is None
    with pytest.raises(HTTPError) as error:
        urlopen(base+'/frame.jpg')
    assert error.value.code == 503
    state.finish('camera unplugged')
    with urlopen(base+'/api/status') as response:
        status = json.load(response)
    assert status['state'] == 'error' and status['result'] is None
    assert status['error'] == 'camera unplugged'


def test_preview_keeps_only_latest_frame_and_wakes_on_stop():
    state = PreviewState()
    state.publish({'processed': b'first'}, observation())
    state.publish({'processed': b'latest'}, observation())
    seq, frame, status = state.next_frame(0, 'processed', timeout=0)
    assert (seq, frame, status) == (2, b'latest', 'ok')
    state.finish()
    seq, frame, status = state.next_frame(2, 'processed', timeout=0)
    assert frame is None and status == 'stopped'
    state.publish({'processed': b'late'}, observation())
    assert state.status()['state'] == 'stopped'
