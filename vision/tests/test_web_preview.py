import json
from http.client import HTTPConnection
import re
import threading
import time
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from carvision.web_preview import PreviewState, make_server


def test_low_rate_preview_confirms_repeated_cues_and_resets_after_camera_gap():
    from carvision.config import Config
    from carvision.pipeline import Pipeline
    from carvision.results import Detection
    from carvision.sources import Frame
    from carvision.web_preview import preview_pipeline_config
    import numpy as np

    class Detector:
        def detect(self, image):
            return [Detection('blue_board', .65, [50, 50, 150, 200])]

    original = Config()
    adjusted = preview_pipeline_config(original, preview_fps=3, stale_ms=1500)
    pipeline = Pipeline(adjusted, Detector())
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    states = []
    for i, timestamp in enumerate([0, 430, 850, 2000]):
        result, _ = pipeline.process(Frame(image, i, None, timestamp), live=True)
        states.append(result.presence['blue_board'])
    assert states == ['unknown', 'present', 'present', 'unknown']
    assert original.temporal.max_gap_ms == 250
    assert adjusted.temporal.confirm_ms == original.temporal.confirm_ms


def test_preview_confirmation_gap_stays_bounded_by_stale_limit_at_one_fps():
    from carvision.config import Config
    from carvision.web_preview import preview_pipeline_config

    assert preview_pipeline_config(Config(), 1, 1500).temporal.max_gap_ms == 1500
    assert preview_pipeline_config(Config(), 10, 1500).temporal.max_gap_ms == 250


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


def test_raw_frames_stay_fresh_when_recognition_is_stale(server):
    from types import SimpleNamespace

    state, base = server
    old = observation()
    old['receive_monotonic_ms'] -= 6000
    state.publish({'raw': b'old-raw', 'processed': b'old-overlay'}, old)
    frame = SimpleNamespace(frame_id=10, receive_monotonic_ms=time.monotonic()*1000)
    state.publish_raw(b'fresh-raw', frame)
    with urlopen(base+'/api/status') as response:
        status = json.load(response)
    assert status['state'] == 'stale' and status['result'] is None
    assert status['raw_preview']['state'] == 'ok'
    assert status['raw_preview']['frame_id'] == 10
    with urlopen(base+'/frame.jpg?view=raw') as response:
        assert response.read() == b'fresh-raw'
    with pytest.raises(HTTPError) as error:
        urlopen(base+'/frame.jpg?view=processed')
    assert error.value.code == 503


def test_raw_timeout_does_not_fall_back_to_a_recognition_frame(server):
    from types import SimpleNamespace

    state, base = server
    frame = SimpleNamespace(frame_id=10, receive_monotonic_ms=time.monotonic()*1000-6000)
    state.publish_raw(b'expired-raw', frame)
    state.publish({'raw': b'recognition-frame', 'processed': b'fresh-overlay'}, observation())
    with pytest.raises(HTTPError) as error:
        urlopen(base+'/frame.jpg?view=raw')
    assert error.value.code == 503
    with urlopen(base+'/frame.jpg?view=processed') as response:
        assert response.read() == b'fresh-overlay'


def test_recognition_cannot_replace_latest_raw_and_stop_clears_both():
    from types import SimpleNamespace

    state = PreviewState()
    now = time.monotonic()*1000
    state.publish_raw(b'first', SimpleNamespace(frame_id=1, receive_monotonic_ms=now))
    state.publish_raw(b'latest', SimpleNamespace(frame_id=2, receive_monotonic_ms=now))
    state.publish({'raw': b'older-recognition-frame', 'processed': b'overlay'}, observation())
    assert state.next_frame(0, 'raw', timeout=0) == (2, b'latest', 'ok')
    state.finish('camera disconnected')
    state.publish_raw(b'late', SimpleNamespace(frame_id=3, receive_monotonic_ms=now))
    assert state.next_frame(0, 'raw', timeout=0) == (2, None, 'error')
    assert state.status()['raw_preview']['state'] == 'error'


def test_raw_preview_advances_while_recognition_is_blocked(monkeypatch):
    from types import SimpleNamespace
    import numpy as np
    from carvision.config import Config
    from carvision.sources import Frame
    from carvision.web_preview import PreviewWorker

    processing_started, release_processing, source_closed = (threading.Event() for _ in range(3))

    class Source:
        identity = None
        reported = {'width': 64, 'height': 48}

        def __iter__(self):
            seq = 0
            while not source_closed.wait(.03):
                yield Frame(np.full((48, 64, 3), seq % 255, dtype=np.uint8),
                            seq, None, time.monotonic()*1000)
                seq += 1

        def close(self):
            source_closed.set()

    class SlowPipeline:
        def process(self, frame, live):
            processing_started.set()
            assert release_processing.wait(3)
            data = {**observation(), 'frame_id': frame.frame_id,
                    'receive_monotonic_ms': frame.receive_monotonic_ms}
            return SimpleNamespace(to_dict=lambda: data), np.zeros((48,64), np.uint8)

    monkeypatch.setattr('carvision.web_preview.CameraSource', lambda *a, **kw: Source())
    monkeypatch.setattr('carvision.web_preview.Pipeline', lambda *a: SlowPipeline())
    monkeypatch.setattr('carvision.web_preview.overlay', lambda image, *a: image)
    args = SimpleNamespace(camera=0, width=64, height=48, fps=30,
                           preview_fps=3, raw_preview_fps=30, stale_ms=1500,
                           preview_width=480, jpeg_quality=55, weights=None)
    state = PreviewState()
    worker = PreviewWorker(state, Config(), args)
    worker.start()
    try:
        assert processing_started.wait(1)
        with state.condition:
            assert state.condition.wait_for(lambda: state.raw_sequence >= 3, timeout=1)
        assert state.sequence == 0  # recognition has not finished its first frame
        raw_sequence, jpeg, status = state.next_frame(0, 'raw', timeout=0)
        assert raw_sequence >= 3 and status == 'ok' and jpeg.startswith(b'\xff\xd8')
    finally:
        release_processing.set()
        worker.close()
    assert source_closed.is_set()
    assert not worker.thread.is_alive() and not worker.raw_thread.is_alive()


def test_two_camera_streams_are_independent_and_never_fallback():
    primary,secondary=PreviewState(),PreviewState()
    primary.publish({'processed':b'primary'},observation())
    secondary.publish({'processed':b'secondary'},observation())
    http=make_server(('127.0.0.1',0),primary,secondary)
    thread=threading.Thread(target=http.serve_forever,kwargs={'poll_interval':.02},daemon=True)
    thread.start()
    base=f'http://127.0.0.1:{http.server_port}'
    try:
        for key,payload in [('primary',b'primary'),('secondary',b'secondary')]:
            with urlopen(base+'/frame.jpg?camera='+key) as r:
                assert r.read()==payload
            with urlopen(base+'/api/status?camera='+key) as r:
                assert json.load(r)['camera_id']==key
        with urlopen(base+'/api/cameras') as r:
            assert len(json.load(r)['sources'])==2
        secondary.finish('second camera unplugged')
        with pytest.raises(HTTPError) as error:
            urlopen(base+'/frame.jpg?camera=secondary')
        assert error.value.code==503
        with urlopen(base+'/frame.jpg?camera=primary') as r:
            assert r.read()==b'primary'
        with pytest.raises(HTTPError) as error:
            urlopen(base+'/frame.jpg?camera=missing')
        assert error.value.code==400
    finally:
        primary.finish()
        secondary.finish()
        http.shutdown()
        http.server_close()
        thread.join(timeout=2)


def test_assistance_http_observes_both_sources_and_returns_to_main():
    primary, secondary = PreviewState(), PreviewState()
    http = make_server(('127.0.0.1', 0), primary, secondary)
    thread = threading.Thread(target=http.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{http.server_port}'

    def publish(main_presence, aux_presence):
        for source, presence in ((primary, main_presence), (secondary, aux_presence)):
            result = {**observation(), 'detector_status': 'experimental_color_shape',
                      'presence': {'crosswalk': presence}}
            source.publish({'raw': b'camera-frame'}, result)

    try:
        for main, aux, expected in [('present', 'present', 'primary'),
                                    ('absent', 'present', 'secondary'),
                                    ('present', 'present', 'primary')]:
            publish(main, aux)
            with urlopen(base+'/api/assist?target=crosswalk') as response:
                result = json.load(response)
            assert result['recognition_source'] == result['recommended_view'] == expected
            assert result['hardware_output'] is False and result['autonomous_launch_authorized'] is False
        secondary.finish('unplugged')
        publish_result = {**observation(), 'detector_status': 'experimental_color_shape',
                          'presence': {'crosswalk': 'absent'}}
        primary.publish({'raw': b'camera-frame'}, publish_result)
        with urlopen(base+'/api/assist') as response:
            assert json.load(response)['recommended_view'] == 'primary'
        with pytest.raises(HTTPError) as error:
            urlopen(base+'/api/assist?target=invalid')
        assert error.value.code == 400
        with pytest.raises(HTTPError) as error:
            urlopen(base+'/api/assist', data=b'{}')
        assert error.value.code == 404
    finally:
        primary.finish()
        secondary.finish()
        http.shutdown()
        http.server_close()
        thread.join(timeout=2)


@pytest.fixture
def control_transport():
    class FakeDrive:
        def __init__(self):
            self.requests = []

        def status(self):
            return {'mode': 'disabled', 'hardware_output': False}

        def request(self, action, payload):
            self.requests.append((action, payload))
            return self.status()

    state, drive = PreviewState(), FakeDrive()
    http = make_server(('127.0.0.1', 0), state, drive=drive)
    thread = threading.Thread(target=http.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
    thread.start()
    connection = HTTPConnection('127.0.0.1', http.server_port, timeout=2)
    try:
        connection.request('GET', '/')
        response = connection.getresponse()
        page = response.read().decode()
        key = json.loads(re.search(r'const key\s*=\s*(".*?")', page).group(1))
        yield state, drive, connection, key
    finally:
        connection.close()
        state.finish()
        http.shutdown()
        http.server_close()
        thread.join(timeout=2)


def test_control_heartbeats_reuse_one_tcp_connection(control_transport):
    _, drive, connection, key = control_transport
    original_socket = connection.sock
    assert original_socket is not None
    for sequence in range(4):
        connection.request('POST', '/api/drive/command',
                           json.dumps({'sequence': sequence, 'motor': 'stop', 'steering': 'center'}),
                           {'Content-Type': 'application/json', 'X-Drive-Key': key})
        response = connection.getresponse()
        assert response.version == 11
        assert json.loads(response.read())['hardware_output'] is False
        assert not response.will_close
        assert connection.sock is original_socket
    assert [payload['sequence'] for _, payload in drive.requests] == list(range(4))


@pytest.mark.parametrize('path,valid_key,body,extra_headers,expected_status', [
    ('/api/drive/command', False, '{}', {}, 403),
    ('/missing', True, '{}', {}, 404),
    ('/api/drive/command', True, '{}', {'Content-Length': '3000'}, 400),
    ('/api/drive/command', True, '{}', {'Transfer-Encoding': 'chunked'}, 400),
])
def test_rejected_control_bodies_close_the_connection(
        control_transport, path, valid_key, body, extra_headers, expected_status):
    _, drive, connection, key = control_transport
    headers = {'Content-Type': 'application/json', 'X-Drive-Key': key if valid_key else 'expired'}
    headers.update(extra_headers)
    connection.request('POST', path, body, headers)
    response = connection.getresponse()
    assert response.status == expected_status
    assert response.headers['Connection'] == 'close'
    response.read()
    assert connection.sock is None
    assert not drive.requests


def test_failed_video_stream_closes_at_eof(control_transport):
    state, _, connection, _ = control_transport
    state.finish('camera disconnected')
    connection.request('GET', '/stream.mjpg')
    response = connection.getresponse()
    assert response.headers['Connection'] == 'close'
    assert response.read() == b''
    assert connection.sock is None
