"""Preparation cannot start hardware or consume the active test interval."""
import threading
import pytest
from carvision.manual_drive import DeferredDrive, DriveError, ManualDrive, SimulationBackend

class Clock:
    now=0.
    def __call__(self):return self.now

class Backend(SimulationBackend):
    hardware_output=True
    ground_held_reviewed=True
    manual_session_limit_s=600
    manual_motion_hold_max_s=600

def setup(gate=None):
    clock=Clock();starts=[];delegates=[]
    def factory():
        starts.append(True)
        if gate is not None:gate.wait(timeout=2)
        worker=ManualDrive(Backend(),clock=clock,settling_s=12,session_s=600,preparation_s=120,ground_held=True)
        delegates.append(worker)
        return worker
    drive=DeferredDrive(factory,dict(ground_held_trial=True,test_session_s=600,motion_hold_max_s=600))
    return drive,clock,starts,delegates

def enable(drive):return drive.request('enable',dict(client='deferred-test',bench_ready=True))

def test_waiting_and_polling_do_not_create_worker_or_consume_driving_window():
    drive,clock,starts,_=setup()
    clock.now=9000
    for _ in range(12):
        state=drive.status()
        assert state['mode']=='disabled' and state['session_remaining_s'] is None
        assert state['pending_hardware_start'] and not state['worker_started']
    assert starts==[]
    state=enable(drive)
    drive.thread.join(timeout=2)
    assert len(starts)==1
    state=drive.status();assert state['mode']=='settling' and state['session_remaining_s']==612
    assert drive.delegate.deadline==clock.now+612
    drive.close()

def test_invalid_ready_or_pre_enable_motion_never_starts_hardware():
    drive,_,starts,_=setup()
    with pytest.raises(DriveError):drive.request('enable',{'client':'deferred-test','bench_ready':False})
    with pytest.raises(DriveError):drive.request('command',{'motor':'forward','steering':'right'})
    assert starts==[]
    drive.close()

def test_duplicate_enable_has_one_factory_and_other_page_cannot_take_owner():
    gate=threading.Event();drive,_,starts,_=setup(gate)
    enable(drive);assert enable(drive)['mode']=='starting'
    with pytest.raises(DriveError):drive.request('enable',{'client':'another-page','bench_ready':True})
    gate.set();drive.thread.join(timeout=2)
    assert len(starts)==1
    drive.close()

@pytest.mark.parametrize('action',['stop','emergency'])
def test_stop_during_startup_cancels_enable_and_never_queues_motion(action):
    gate=threading.Event();drive,_,_,delegates=setup(gate)
    enable(drive);drive.request(action,{})
    with pytest.raises(DriveError):drive.request('command',{'motor':'forward','steering':'right'})
    gate.set();drive.thread.join(timeout=2)
    assert drive.status()['mode']=='fault'
    assert delegates[0].mode=='closed' and delegates[0].session_started is False
    drive.close()

def test_active_expiry_is_terminal_and_cannot_start_a_new_worker():
    drive,clock,starts,_=setup();enable(drive);drive.thread.join(timeout=2)
    clock.now=613;drive.delegate.tick()
    assert drive.status()['mode']=='expired'
    with pytest.raises(DriveError):enable(drive)
    assert len(starts)==1
    drive.close()

def test_failed_start_is_terminal_without_automatic_restart():
    starts=[]
    def factory():
        starts.append(True)
        raise RuntimeError('startup proof rejected')
    drive=DeferredDrive(factory,{})
    enable(drive);drive.thread.join(timeout=2)
    assert drive.status()['mode']=='fault'
    assert 'startup proof rejected' in drive.status()['reason']
    with pytest.raises(DriveError):enable(drive)
    assert len(starts)==1
    drive.close()

