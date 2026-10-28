"""Shared manual/autonomous ownership with a fake backend; no hardware opened."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from carvision.manual_drive import ManualDrive
from carvision.shared_executor import BorrowedActuator, WorkerAutonomyOwner


CLIENT='operator-1'


def test_worker_guard_imports_with_only_standard_library_and_source_path():
    source=Path(__file__).resolve().parents[1]/'src'
    result=subprocess.run([sys.executable,'-S','-c',
        "import carvision.shared_executor, carvision.manual_hardware, sys; "
        "assert 'cv2' not in sys.modules and 'numpy' not in sys.modules"],
        env={**os.environ,'PYTHONPATH':str(source)},capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr


class Backend:
    hardware_output=True
    steering_center_us=1610
    steering_available=motor_available=True

    def __init__(self,clock):self.clock=clock;self.writes=[];self.closed=False;self.checks=0
    def motion(self,pulse,seconds):
        assert 1000<=pulse<=2000 and .02<=seconds<=.2
        self.writes.append(('motion',self.clock(),pulse,seconds))
    def neutral(self):self.writes.append(('neutral',self.clock()))
    def steering(self,pulse):self.writes.append(('steering',self.clock(),pulse))
    def check(self):self.checks+=1
    def close(self):self.closed=True


@pytest.fixture
def rig():
    now=[0.];clock=lambda:now[0];backend=Backend(clock)
    controller=ManualDrive(backend,clock=clock,settling_s=0,session_s=None)
    controller.request('enable',{'client':CLIENT,'bench_ready':True});controller.tick()
    controller.request('stop',{})
    owner=WorkerAutonomyOwner(controller,backend,enabled=True,clock=clock)
    settings={'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':1600,'creep_us':1575,'brake_us':1400},
              'brake_pulse_s':.35,'brake_release_s':.1,'actuator_lease_s':.2}
    yield now,clock,backend,controller,owner,settings
    owner.close();controller.close()


class Worker:
    def __init__(self,owner):self.owner=owner;self.calls=[]
    def request(self,action,payload):
        self.calls.append((action,copy.deepcopy(payload)))
        return self.owner.dispatch(action,payload)


def borrow(rig):
    now,clock,backend,controller,owner,settings=rig
    worker=Worker(owner)
    return BorrowedActuator(worker,settings,CLIENT,clock=clock),worker


def intent(pulse=1600):return {'action':'search','esc_us':pulse,'steering_us':1610}


def test_borrow_reuses_backend_and_close_does_not_shutdown_worker(rig):
    now,clock,backend,controller,owner,settings=rig
    actuator,worker=borrow(rig)
    assert owner.backend is backend and controller.mode=='emergency'
    assert actuator.startup_readback=={'warm_handoff':True,'physical_stop_verified':False}
    actuator.send({**intent(),'large_telemetry':'x'*15000})
    action,payload=worker.calls[-1]
    assert action=='autonomy_intent' and len(json.dumps(payload))<4097
    assert set(payload['intent'])=={'action','esc_us','steering_us','sequence','sent_s'}
    now[0]=.05;actuator.close()
    assert not owner.active and owner.status()['autonomy_draining']
    assert not backend.closed and all(action!='shutdown' for action,_ in worker.calls)
    now[0]=.40;owner.tick()
    assert owner.guard is None and controller.mode=='emergency' and not backend.closed


@pytest.mark.parametrize('change',[{'client':'another-operator'},{'fb_mode_confirmed':False}])
def test_acquire_requires_current_manual_owner_and_explicit_fb_mode(rig,change):
    now,clock,backend,controller,owner,settings=rig
    before=list(backend.writes)
    with pytest.raises(ValueError):
        owner.dispatch('autonomy_acquire',{'client':CLIENT,'settings':settings,'fb_mode_confirmed':True,**change})
    assert backend.writes==before and not owner.active


def test_default_worker_refuses_autonomy_and_enabled_worker_requires_stopped_motor(rig):
    now,clock,backend,controller,owner,settings=rig
    payload={'client':CLIENT,'settings':settings,'fb_mode_confirmed':True}
    owner.enabled=False
    with pytest.raises(ValueError,match='未启用'):owner.dispatch('autonomy_acquire',payload)
    owner.enabled=True;controller.motor='forward';controller.pulse=1575
    with pytest.raises(ValueError,match='停车'):owner.dispatch('autonomy_acquire',payload)
    assert not owner.active


@pytest.mark.parametrize('action',['command','enable','reset','speed','steering_pwm'])
def test_old_manual_actions_are_rejected_during_autonomous_ownership(rig,action):
    actuator,worker=borrow(rig);owner=rig[4];backend=rig[2]
    actuator.send(intent());before=list(backend.writes)
    with pytest.raises(ValueError,match='持有'):owner.dispatch(action,{'client':CLIENT,'motor':'forward'})
    assert backend.writes==before and owner.active


def test_old_token_cannot_send_or_release_a_new_round(rig):
    now,clock,backend,controller,owner,settings=rig
    first,worker=borrow(rig);old_token=first.token;first.close()
    controller.request('reset',{});controller.request('enable',{'client':CLIENT,'bench_ready':True});controller.tick()
    second=BorrowedActuator(worker,settings,CLIENT,clock=clock)
    for action in ('autonomy_intent','autonomy_release'):
        with pytest.raises(ValueError,match='失效'):
            owner.dispatch(action,{'client':CLIENT,'worker_token':old_token,'intent':{**intent(),'sequence':0,'sent_s':clock()}})
        assert owner.active and owner.token==second.token
    first.close()
    second.send(intent())


def test_forward_lease_loss_runs_only_finite_brake_then_neutral(rig):
    now,clock,backend,controller,owner,settings=rig
    actuator,worker=borrow(rig);actuator.send(intent())
    now[0]=.21;owner.tick()
    assert not owner.active and owner.guard.esc==1400
    brake_end=owner.guard.emergency_until
    assert brake_end==pytest.approx(.56)
    for t in (.31,.41,.51,.56):now[0]=t;owner.tick()
    assert owner.guard is None and controller.mode=='emergency'
    brake_segments=[row for row in backend.writes if row[0]=='motion' and row[2]==1400]
    assert brake_segments and all(t+duration<=brake_end+1e-9 for _,t,pulse,duration in brake_segments)
    assert backend.writes[-2][0]=='neutral' and not backend.closed
    with pytest.raises(ValueError,match='失效'):actuator.send(intent())


def test_repeated_late_brake_cannot_extend_guardian_segment_beyond_350ms(rig):
    now,clock,backend,controller,owner,settings=rig
    actuator,worker=borrow(rig)
    brake={'action':'brake','esc_us':1400,'steering_us':1610}
    for t in (0.,.1,.2,.3,.349):
        now[0]=t;actuator.send(brake)
    segments=[row for row in backend.writes if row[0]=='motion' and row[2]==1400]
    assert segments and all(t+duration<=.35+1e-9 for _,t,pulse,duration in segments)
    assert segments[-1][1]+segments[-1][3]==pytest.approx(.35)
    assert not any(row[0]=='motion' and row[1]==.349 for row in backend.writes)
    now[0]=.35;owner.tick()
    assert owner.guard.esc==1500


@pytest.mark.parametrize('action',['stop','emergency','shutdown'])
def test_global_stop_invalidates_autonomy_without_replaying_manual_commands(rig,action):
    actuator,worker=borrow(rig);owner=rig[4]
    actuator.send(intent());reply=owner.dispatch(action,{})
    assert not reply['autonomy_active'] and reply['autonomy_draining']
    with pytest.raises(ValueError):actuator.send(intent())
    assert rig[3].mode=='emergency'


def test_auto_guard_does_not_inherit_manual_2000us_forward_range(rig):
    actuator,worker=borrow(rig);owner=rig[4]
    with pytest.raises(ValueError):actuator.send(intent(2000))
    assert not owner.active
    assert not any(row[0]=='motion' and row[2]==2000 for row in rig[2].writes)


def test_status_has_no_output_or_lease_refresh_side_effects(rig):
    actuator,worker=borrow(rig);owner=rig[4];backend=rig[2]
    actuator.send(intent());before=list(backend.writes);deadline=owner.guard.deadline
    for _ in range(5):
        assert not owner.dispatch('status',{})['physical_stop_verified']
    assert backend.writes==before and owner.guard.deadline==deadline


def test_manual_watcher_is_paused_for_borrow_and_rebuilt_after_release(rig):
    controller=rig[3];owner=rig[4];controller.start();old=controller.thread
    actuator,worker=borrow(rig)
    assert not old.is_alive() and controller.stop_event.is_set()
    actuator.close()
    assert owner.guard is None and controller.thread is not old and controller.thread.is_alive()
    assert not controller.stop_event.is_set() and controller.mode=='emergency'


@pytest.mark.parametrize('late_exit',[False,True])
def test_watcher_join_timeout_latches_worker_even_after_old_thread_exits(rig,monkeypatch,late_exit):
    controller=rig[3];owner=rig[4];backend=rig[2]
    class DelayedWatcher:
        alive=True
        joins=[]
        def is_alive(self):return self.alive
        def join(self,timeout=None):self.joins.append(timeout)
    watcher=DelayedWatcher();controller.thread=watcher
    monkeypatch.setattr(controller,'start',lambda:pytest.fail('a second watcher was started'))
    with pytest.raises(RuntimeError,match='尚未释放'):borrow(rig)
    assert watcher.joins==[1] and controller.stop_event.is_set()
    assert controller.mode=='fault' and owner.guard is None and not owner.active
    assert owner.status()['autonomy_handoff_fault'] and not owner.resume_watcher
    if late_exit:watcher.alive=False
    writes=list(backend.writes)
    for action in ('reset','enable','command','autonomy_acquire'):
        with pytest.raises(ValueError,match='重新准备'):
            owner.dispatch(action,{'client':CLIENT,'bench_ready':True,'motor':'forward'})
    assert backend.writes==writes
    assert not any(row[0]=='motion' for row in backend.writes)
    # Observation and global stopping/shutdown remain available for cleanup.
    assert owner.dispatch('status',{})['mode']=='fault'
    assert owner.dispatch('shutdown',{})['mode']=='fault'


def test_failed_neutral_during_acquire_cannot_restore_manual_without_a_watcher(rig,monkeypatch):
    backend=rig[2];controller=rig[3];owner=rig[4];calls=[]
    original=backend.neutral
    def fail_second_neutral():
        calls.append(True)
        if len(calls)==2:raise OSError('simulated neutral acknowledgement failure')
        original()
    monkeypatch.setattr(backend,'neutral',fail_second_neutral)
    with pytest.raises(OSError):borrow(rig)
    assert owner.handoff_fault is not None and controller.mode=='fault'
    assert controller.stop_event.is_set() and owner.guard is None
    with pytest.raises(ValueError,match='重新准备'):owner.dispatch('reset',{})
