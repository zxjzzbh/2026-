"""Native camera and best-effort diagnostics cannot acquire motion authority."""
import json
import queue
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from carvision.camera_frames import control_frame
from carvision.config import Config
from carvision.event_journal import EventJournal
from carvision.sources import Frame
from carvision.web_preview import PreviewState, PreviewWorker
from test_race_flow import bundle


def wait_for(predicate):
    deadline=time.monotonic()+3
    while not predicate() and time.monotonic()<deadline:time.sleep(.005)
    assert predicate()


def test_native_capture_bypasses_failed_or_old_jpeg_and_keeps_real_identity():
    state=PreviewState();pixels=np.full((720,960,3),71,np.uint8)
    state.publish_raw(b'old jpeg',Frame(pixels,4,None,1000.))
    state.publish_capture(Frame(pixels,5,None,1100.))
    state.preview_failed('optional overlay failed')
    view=control_frame(state)
    assert view['native'] and view['jpeg'] is None
    assert view['frame_id']==5 and view['captured_ms']==1100.
    assert view['image'].shape==(360,480,3)
    view['image'][:]=0
    assert np.all(pixels==71)
    state.finish('capture lost')
    with pytest.raises(ValueError):control_frame(state)


def test_actual_capture_thread_survives_preview_and_raw_encoder_failure(monkeypatch):
    images=queue.Queue();released=threading.Event()
    class Capture:
        def isOpened(self):return True
        def set(self,*a):return True
        def get(self,*a):return 0
        def grab(self):
            self.next=images.get(timeout=3)
            return self.next is not None
        def retrieve(self):return True,self.next
        def release(self):released.set()
    class Pipeline:
        def __init__(self,*a):pass
        def process(self,*a,**k):raise RuntimeError('optional detector unavailable')
    monkeypatch.setattr('carvision.sources.cv2.VideoCapture',lambda *a:Capture())
    monkeypatch.setattr('carvision.web_preview.Pipeline',Pipeline)
    state=PreviewState()
    args=SimpleNamespace(camera=0,width=480,height=360,fps=60,weights=None,
        camera_format=None,preview_fps=60,raw_preview_fps=60,stale_ms=1500,
        preview_width=480,jpeg_quality=80)
    worker=PreviewWorker(state,Config(),args)
    def broken_encode(image):raise RuntimeError('optional jpeg unavailable')
    monkeypatch.setattr(worker,'_encode',broken_encode)
    worker.start()
    try:
        images.put(np.full((360,480,3),60,np.uint8))
        wait_for(lambda:state.preview_error is not None and state.raw_error is not None)
        first=control_frame(state)
        time.sleep(.03)
        images.put(np.full((360,480,3),90,np.uint8))
        wait_for(lambda:state.capture_frame.frame_id>first['frame_id'])
        second=control_frame(state)
        assert second['captured_ms']>first['captured_ms']
        assert np.all(second['image']==90) and not released.is_set()
        images.put(None)
        wait_for(lambda:state.state=='error')
        with pytest.raises(ValueError):control_frame(state)
    finally:
        images.put(None);worker.close()


def test_disk_failure_is_reported_without_raising_in_control_submit(tmp_path):
    destination=tmp_path/'not-a-directory';destination.write_text('file')
    journal=EventJournal(destination)
    try:
        wait_for(lambda:journal.error is not None)
        assert journal.submit({'event':'decision'}) is False
        assert journal.status()['dropped']==1
    finally:journal.close(.5)


def test_slow_disk_has_bounded_queue_and_no_control_thread_io(tmp_path,monkeypatch):
    entered,release=threading.Event(),threading.Event()
    original=Path.mkdir
    def slow_mkdir(path,*a,**k):
        if path==tmp_path/'journal':
            entered.set();assert release.wait(3)
        return original(path,*a,**k)
    monkeypatch.setattr(Path,'mkdir',slow_mkdir)
    journal=EventJournal(tmp_path/'journal',capacity=2)
    try:
        assert entered.wait(2)
        row={'value':[1]};assert journal.submit(row)
        row['value'][0]=9
        assert journal.submit({'value':[2]})
        assert not journal.submit({'value':[3]})
        assert journal.status()['pending']==2 and journal.status()['dropped']==1
        release.set();assert journal.flush(2)
        rows=[json.loads(line) for line in (journal.folder/'events.jsonl').read_text().splitlines()]
        assert [r['value'] for r in rows]==[[1],[2]]
    finally:release.set();journal.close(.5)


def test_race_worker_keeps_current_task_when_its_journal_cannot_open(bundle,tmp_path,monkeypatch):
    from test_race_cancel_inflight import make_flow,Reader
    reader=Reader(None)
    flow,sender,actuator=make_flow(bundle,tmp_path,monkeypatch,reader)
    flow.record=tmp_path/'occupied-by-file';flow.record.write_text('unwritable as a directory')
    flow.thread.start()
    try:
        assert reader.entered.wait(2)
        wait_for(lambda:flow.journal is not None and flow.journal.error is not None)
        assert flow.active and flow.stage=='arming' and not actuator.closed.is_set()
        assert flow.status()['race_flow']['record_error']
    finally:
        stopping=threading.Thread(target=flow.cancel,args=('operator exit',));stopping.start()
        reader.resume.set();stopping.join(3);flow.thread.join(3);flow.close()
    assert flow.stage=='cancelled' and actuator.closed.is_set()
