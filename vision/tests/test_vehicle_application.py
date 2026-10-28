"""Application/transition invariants; all executors and I/O are fake."""
import copy
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from carvision.handoff import StillnessWindow
from carvision.race_flow import RaceFlowController, load_bundle
from carvision.race_flow_console import RaceFlowDrive
from carvision.application import DebugApplication, VehicleBootDrive

ROOT = Path(__file__).resolve().parents[2]


class Base:
    def __init__(self):self.calls=[]
    def status(self):
        return {'mode':'disabled','hardware_output':False,'worker_started':False,
                'pending_hardware_start':False,'boot_test_phase':'esc_off','setup_token':'boot'}
    def request(self,action,payload):self.calls.append(action);return self.status()
    def close(self):self.calls.append('close')


def payload():
    return {'client':'test-operator','setup_token':'boot','stationary_confirmed':True,
            'in_switch_zone':True,'fb_mode_confirmed':True}


def test_missing_parking_calibration_defers_parking_without_blocking_start_or_opening_output(tmp_path,monkeypatch):
    bundle=load_bundle(ROOT)
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    made=[];drive=RaceFlowDrive(Base(),{},tmp_path,live=True,actuator_factory=lambda _:made.append(True))
    report=drive.status()['race_flow']
    assert not report['readiness']['ready'] and report['parking_missing']
    assert report['can_enter'] and report['start_readiness']['ready']
    assert report['start_readiness']['scope']=='through_cones'
    assert not report['start_readiness']['parking_ready']
    assert not made and not drive.active and drive.thread is None


def test_stillness_requires_distinct_capture_time_and_a_continuous_window():
    window=StillnessWindow()
    def row(t,moving=False):return {'captured_s':t,'valid':True,'moving':moving}
    assert not window.update(row(0),0)
    for now in (.1,.2,.24):assert not window.update(row(0),now)
    assert not window.update(row(.3),.3)  # Capture gap starts a new window.
    assert not window.update(row(.4),.4)
    assert not window.update(row(.5,True),.5)
    assert not window.update(row(.6),.6)
    assert not window.update(row(.7),.7)
    assert not window.update(row(.8),.8)
    assert window.update(row(.9),.9)
    assert not window.update(row(.9),1.)
    assert not window.update({},1.1)
    assert window.first is None


def test_cancelled_slow_initialization_cannot_overlap_a_new_round(tmp_path,monkeypatch):
    bundle=load_bundle(ROOT)
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    entered=threading.Event();finish=threading.Event();closed=[]
    class Handle:
        run=True
        def close(self):closed.append(True)
        def send(self,_):pytest.fail('cancelled initialization must not send')
    def factory(_):entered.set();assert finish.wait(8);return Handle()
    class Reader:
        preview=None
        def __init__(self,*_):pass
    drive=RaceFlowDrive(Base(),{},tmp_path,live=True,require_ready=False,
                        actuator_factory=factory,reader_factory=Reader)
    drive.request('race_enter',payload());assert entered.wait(1)
    token=drive.token
    real_join=drive.thread.join
    monkeypatch.setattr(drive.thread,'join',lambda timeout=None:real_join(.01))
    drive.cancel('cancel first initialization')
    assert drive.thread.is_alive() and not drive.status()['race_flow']['can_enter']
    with pytest.raises(ValueError,match='清理'):drive.request('race_enter',payload())
    assert drive.token==token
    finish.set();real_join(1)
    assert not drive.thread.is_alive() and closed==[True] and drive.actuator is None


def test_logging_failure_cannot_publish_a_running_session(tmp_path,monkeypatch):
    bundle=load_bundle(ROOT)
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    drive=RaceFlowDrive(Base(),{},tmp_path,live=True,require_ready=False)
    monkeypatch.setattr(Path,'mkdir',lambda *a,**k:(_ for _ in ()).throw(PermissionError('full disk')))
    with pytest.raises(PermissionError):drive.request('race_enter',payload())
    assert not drive.active and drive.thread is None and drive.token is None


def test_failed_actuator_close_latches_unconfirmed_output(tmp_path,monkeypatch):
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:load_bundle(ROOT))
    drive=RaceFlowDrive(Base(),{},tmp_path,live=True,require_ready=False)
    drive.active=True;drive.stage='arming'
    def fail():raise RuntimeError('release not acknowledged')
    drive.actuator=SimpleNamespace(run=True,close=fail)
    drive.cancel('test')
    report=drive.status()['race_flow']
    assert report['cleanup_error'] and report['hardware_output'] is None and not report['can_enter']


def test_global_emergency_remains_effective_during_payment_without_resetting_clock(tmp_path,monkeypatch):
    bundle=load_bundle(ROOT)
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    base=Base();drive=RaceFlowDrive(base,{},tmp_path,live=True,clock=lambda:15.)
    drive.active=True;drive.owner='owner';drive.stage='payment'
    drive.controller=RaceFlowController(bundle,0.)
    drive.controller.stage='payment';drive.controller.payment_started=10.
    report=drive.request('emergency',{'client':'observer'})['race_flow']
    assert base.calls==['emergency'] and report['stage']=='payment' and report['payment_remaining_s']==25.


def test_debug_application_explicit_registry_uses_one_manual_base():
    base=Base();app=DebugApplication(base,{},ROOT)
    try:
        assert set(app.tasks)=={'brake','traffic','cones','departure','parking','payment'}
        assert all(node.drive.application is app for node in app.tasks.values())
        assert app.status()['application_mode']=='debug'
        with pytest.raises(ValueError,match='不运行完整比赛'):app.request('race_enter',{})
    finally:app.close()


def test_independent_competition_entry_does_not_load_historical_launcher_chain():
    import subprocess,os,sys
    code="import sys;import carvision.application;assert not any(x in sys.modules for x in ('serve_plan_b','serve_race_flow','serve_parking_payment','serve_crosswalk_trial'))"
    result=subprocess.run([sys.executable,'-c',code],env={**os.environ,'PYTHONPATH':str(ROOT/'vision/src')},capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def test_shared_boot_retains_the_existing_fb_reverse_prohibition(monkeypatch):
    from carvision.boot_test import BootTestDrive
    calls=[]
    monkeypatch.setattr(BootTestDrive,'status',lambda self:{'reverse_available':True})
    monkeypatch.setattr(BootTestDrive,'request',lambda self,action,payload:calls.append((action,payload)))
    boot=VehicleBootDrive.__new__(VehicleBootDrive)
    assert not boot.status()['reverse_available'] and boot.status()['esc_mode']=='F/B'
    with pytest.raises(ValueError,match='禁止倒车'):
        boot.request('command',{'client':'owner','motor':'reverse'})
    assert [action for action,_ in calls]==['stop']
