"""Real threads, fake output: slow vision/disk cannot starve the parking lease."""
import json
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.parking_bay_console import ParkingBayDrive
from carvision.parking_log import ParkingLog
from test_parking_entry_run import Actuator, base, configurations, preparation, sample, command


def wrapper(tmp_path):
    configs=tmp_path/'vision/configs';configs.mkdir(parents=True)
    p,b,c=configurations()
    (configs/'parking-bay.json').write_text(json.dumps(p),encoding='utf-8')
    now=[.1];actuator=Actuator()
    drive=ParkingBayDrive(SimpleNamespace(status=base),{},tmp_path,
        clock=lambda:now[0],entry_factory=lambda s:actuator)
    drive.entry.prepare(p,b,c,base(),preparation(),0.)
    drive.entry_sample=sample(.1)
    drive._control_tick()
    return drive,actuator,now


def wait_until(predicate):
    deadline=time.monotonic()+2
    while not predicate():
        if time.monotonic()>deadline:pytest.fail('worker did not make expected progress')
        time.sleep(.005)


def test_slow_detector_does_not_starve_ready_output_but_stale_frame_stops(tmp_path,monkeypatch):
    import cv2
    import numpy as np
    import carvision.parking_bay_console as module
    w,a,now=wrapper(tmp_path)
    _,jpg=cv2.imencode('.jpg',np.zeros((360,480,3),dtype=np.uint8))
    w.states={'secondary':SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),
        raw_frame_id=2,raw_received_ms=100.,state='ok')}
    entered=threading.Event();release=threading.Event()
    def delayed_detection(*args):
        entered.set()
        assert release.wait(3)
        raise ValueError('test ends observation')
    monkeypatch.setattr(module,'detect_parking',delayed_detection)
    w.active=True
    thread=threading.Thread(target=w._run);thread.start()
    try:
        assert entered.wait(2)
        count=len(a.commands)
        wait_until(lambda:len(a.commands)>=count+3)
        assert w.entry.active and all(x['esc_us']==1500 for x in a.commands)
        assert w.entry.s['brake']['actuator_lease_s']==.2
        now[0]=.36
        wait_until(lambda:a.closed)
        assert not w.entry.active and '画面' in w.entry.reason
        # New pixels alone cannot reopen a cancelled session.
        w.entry_sample=sample(.36);w._control_tick()
        assert a.closed and not w.entry.active
    finally:
        w.stop.set();release.set();thread.join(3);w.close()
    assert not thread.is_alive() and not w.control_thread.is_alive()


def test_page_expiry_still_stops_with_fresh_camera(tmp_path):
    w,a,now=wrapper(tmp_path)
    now[0]=.51;w.entry_sample=sample(.51)
    w._control_tick()
    assert a.closed and not w.entry.active and '心跳' in w.entry.reason
    assert w.entry.lease_until==.5
    w.close()


def test_repeat_ticks_do_not_repeat_controller_evidence_and_cancel_cannot_resume(tmp_path):
    w,a,now=wrapper(tmp_path)
    class Controller:
        steps=0
        def step(self,view,t):
            self.steps+=1
            return dict(action='search',esc_us=1575,steering_us=1650,phase='approach',reason='test')
    controller=Controller();w.entry.controller=controller
    for t in (.11,.15,.19):now[0]=t;w._control_tick()
    assert controller.steps==1 and len(a.commands)==4
    w.request('parking_trial_cancel',{})
    count=len(a.commands);w._control_tick()
    assert len(a.commands)==count and a.closed
    w.close()


def test_slow_decision_log_does_not_block_control_writes(tmp_path,monkeypatch):
    original=Path.open;entered=threading.Event();release=threading.Event()
    class SlowFile:
        def write(self,text):entered.set();assert release.wait(3)
        def flush(self):pass
        def close(self):pass
    monkeypatch.setattr(Path,'open',lambda path,*a,**kw:SlowFile() if path.name=='slow.jsonl' else original(path,*a,**kw))
    log=ParkingLog(tmp_path/'slow.jsonl')
    try:
        log.write('first\n');assert entered.wait(2)
        for _ in range(10):log.write('next\n');log.flush()
        assert log.pending.qsize()==10 and not log.closed
    finally:release.set();log.close()


def test_log_failure_and_queue_overflow_fail_closed(tmp_path,monkeypatch):
    w,a,now=wrapper(tmp_path)
    w.entry.log.close()
    class FailedLog:
        closed=False
        def write(self,text):raise OSError('parking decision log queue full')
        def flush(self):pass
        def close(self):self.closed=True
    w.entry.log=FailedLog();now[0]=.2;w.entry_sample=sample(.2)
    w._control_tick()
    assert not w.entry.active and a.closed and 'queue full' in w.entry.reason
    w.close()
