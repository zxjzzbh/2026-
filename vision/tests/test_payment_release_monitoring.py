"""Real owner/borrowed/console release chain, fake backend only; no vehicle I/O."""
import pytest

from carvision.cone_command import ConeCommandSender
from carvision.race_flow_console import RaceFlowDrive
from carvision.shared_executor import BorrowedActuator
from test_shared_executor import CLIENT, Worker, borrow, rig


def payment_console():
    flow=RaceFlowDrive.__new__(RaceFlowDrive)
    flow.stage,flow.reason,flow.cleanup_error='payment','parked',None
    return flow


@pytest.mark.parametrize('network_fails',[False,True])
def test_successful_neutral_release_preserves_payment_without_manual_network_check(rig,network_fails):
    now,clock,backend,controller,owner,settings=rig
    modes=[]
    def mode_change(mode):
        modes.append(mode)
        if network_fails and mode=='manual':raise RuntimeError('5G route changed; manual output stopped')
    backend.set_control_mode=mode_change
    actuator,worker=borrow(rig)
    sender=ConeCommandSender(actuator,clock=clock,threaded=False)
    sender.send_race({'action':'neutral','esc_us':1500,'steering_us':1610},run_until=10.)
    flow=payment_console()
    assert flow._close_handle(sender)
    assert flow.stage=='payment' and flow.cleanup_error is None and actuator.close_error is None
    assert modes==['autonomous'] and owner.guardian_mode=='autonomous'
    assert not owner.active and owner.guard is None and controller.mode=='emergency'
    assert owner.status()['autonomy_release_confirmed'] and owner.status()['manual_rearm_required']
    before=backend.checks
    for _ in range(5):now[0]+=.05;owner.tick()
    assert backend.checks>=before+5 and not any(row[0]=='motion' for row in backend.writes)
    with pytest.raises(ValueError):
        owner.dispatch('autonomy_intent',{'client':CLIENT,'worker_token':actuator.token,
            'intent':{'action':'search','esc_us':1600,'steering_us':1610,'sequence':1,'sent_s':clock()}})
    with pytest.raises(ValueError):borrow(rig)
    worker.request('autonomy_release',{'client':CLIENT,'worker_token':actuator.token})
    assert modes==['autonomous']


def test_only_explicit_valid_enable_restores_manual_and_the_watcher(rig):
    _,_,backend,controller,owner,_=rig
    controller.start();old_watcher=controller.thread
    modes=[]
    backend.set_control_mode=lambda mode:modes.append((mode,controller.mode))
    actuator,_=borrow(rig)
    actuator.close();assert actuator.confirm_release()
    assert not old_watcher.is_alive() and controller.stop_event.is_set()
    owner.dispatch('reset',{'client':CLIENT})
    assert controller.mode=='disabled' and [mode for mode,_ in modes]==['autonomous']
    owner.dispatch('enable',{'client':CLIENT,'bench_ready':True})
    controller.tick()
    assert modes[-1]==('manual','disabled') and controller.mode=='enabled'
    assert controller.thread is not old_watcher and controller.thread.is_alive()
    assert not owner.status()['manual_rearm_required']
    assert all(row[0]!='motion' for row in backend.writes)


@pytest.mark.parametrize('reset,payload',[(False,{'client':CLIENT,'bench_ready':True}),
    (True,{'client':CLIENT,'bench_ready':False}),(True,{'client':'bad','bench_ready':True})])
def test_invalid_enable_cannot_trigger_network_mode_change(rig,reset,payload):
    _,_,backend,_,owner,_=rig
    modes=[];backend.set_control_mode=lambda mode:modes.append(mode)
    actuator,_=borrow(rig);actuator.close();actuator.confirm_release()
    if reset:owner.dispatch('reset',{'client':CLIENT})
    with pytest.raises(ValueError):owner.dispatch('enable',payload)
    assert modes==['autonomous'] and owner.guardian_mode=='autonomous'
    assert all(row[0]!='motion' for row in backend.writes)


def test_manual_network_failure_blocks_new_manual_motion_but_not_completed_payment(rig):
    _,_,backend,controller,owner,_=rig
    modes=[]
    def mode_change(mode):
        modes.append(mode)
        if mode=='manual':raise RuntimeError('5G route changed; manual output stopped')
    backend.set_control_mode=mode_change
    actuator,_=borrow(rig);flow=payment_console()
    assert flow._close_handle(actuator) and flow.stage=='payment'
    owner.dispatch('reset',{'client':CLIENT})
    with pytest.raises(RuntimeError,match='5G route changed'):
        owner.dispatch('enable',{'client':CLIENT,'bench_ready':True})
    assert modes==['autonomous','manual'] and controller.mode=='fault'
    assert owner.handoff_fault and flow.stage=='payment'
    assert all(row[0]!='motion' for row in backend.writes)


def test_actual_neutral_write_failure_still_faults_payment(rig):
    _,_,backend,_,owner,_=rig
    actuator,_=borrow(rig)
    neutral=backend.neutral
    def fail():raise RuntimeError('neutral output ACK missing')
    backend.neutral=fail
    try:
        flow=payment_console()
        assert not flow._close_handle(actuator)
        assert flow.stage=='fault' and 'neutral output ACK missing' in flow.cleanup_error
        assert not owner.status()['autonomy_release_confirmed']
    finally:backend.neutral=neutral


class AdvancingWorker(Worker):
    def __init__(self,owner,now):super().__init__(owner);self.now=now
    def request(self,action,payload):
        if action=='autonomy_release':self.now[0]=round(self.now[0]+.1,8)
        return super().request(action,payload)


@pytest.mark.parametrize('neutral_fails',[False,True])
def test_console_waits_for_finite_brake_and_observes_delayed_stop_failure(rig,neutral_fails):
    now,clock,backend,_,owner,settings=rig
    actuator=BorrowedActuator(AdvancingWorker(owner,now),settings,CLIENT,clock=clock)
    actuator.send({'action':'search','esc_us':1600,'steering_us':1610})
    neutral=backend.neutral
    if neutral_fails:
        def fail():raise RuntimeError('late neutral write failed')
        backend.neutral=fail
    try:
        flow=payment_console()
        assert flow._close_handle(actuator) is not neutral_fails
        assert now[0]>=.45  # Close began at .1; its original .35-s brake ended.
        assert flow.stage==('fault' if neutral_fails else 'payment')
        if neutral_fails:assert 'late neutral' in flow.cleanup_error
        else:assert owner.guard is None and owner.release_confirmed
        assert len([row for row in backend.writes if row[0]=='motion' and row[2]>1500])==1
    finally:backend.neutral=neutral


def test_unconfirmed_draining_cannot_be_reported_as_successful_release(rig):
    _,_,_,_,owner,_=rig
    actuator,_=borrow(rig)
    actuator.send({'action':'search','esc_us':1600,'steering_us':1610})
    actuator.close()
    with pytest.raises(RuntimeError,match='未确认'):actuator.confirm_release(timeout_s=.02)
    assert owner.guard is not None and not owner.release_confirmed and actuator.close_error


def test_worker_shutdown_does_not_attempt_manual_network_restore(rig):
    _,_,backend,_,owner,_=rig
    modes=[];backend.set_control_mode=lambda mode:modes.append(mode)
    actuator,_=borrow(rig);actuator.close();actuator.confirm_release()
    owner.close()
    assert modes==['autonomous'] and not owner.active and owner.guard is None


def test_new_failed_acquisition_cannot_reuse_an_old_release_confirmation(rig):
    _,_,backend,_,owner,_=rig
    actuator,_=borrow(rig);actuator.close();actuator.confirm_release()
    owner.dispatch('reset',{})
    owner.dispatch('enable',{'client':CLIENT,'bench_ready':True})
    owner.controller.tick()
    original=backend.neutral
    def fail():raise RuntimeError('new neutral ACK missing')
    backend.neutral=fail
    try:
        with pytest.raises(RuntimeError):borrow(rig)
        assert not owner.status()['autonomy_release_confirmed']
    finally:backend.neutral=original
