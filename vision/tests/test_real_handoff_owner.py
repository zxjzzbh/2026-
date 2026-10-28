"""Real preparation/owner wrappers, replacing backend and RPC transport only.

No fake status dictionary supplies owner_client: it must come from the actual
ManualDrive owner through WorkerAutonomyOwner and OwnedWorkerDrive.
"""
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.application import VehicleBootDrive
from carvision.manual_drive import DeferredDrive, ManualDrive
from carvision.shared_executor import WorkerAutonomyOwner
from carvision.worker_client import OwnedWorkerDrive
from test_shared_executor import Backend

CLIENT = 'real-chain-operator'


def settings():
    return {'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':1600,
                   'creep_us':1575,'brake_us':1400},
            'brake_pulse_s':.35,'brake_release_s':.1,'actuator_lease_s':.2}


@pytest.fixture
def stack_factory(tmp_path):
    stacks = []

    def create(*, enabled=True, settling_s=0.):
        now = [0.]
        clock = lambda:now[0]
        backend = Backend(clock)
        manual = ManualDrive(backend,clock=clock,settling_s=settling_s,session_s=None)
        owner = WorkerAutonomyOwner(manual,backend,enabled=True,clock=clock)
        worker = OwnedWorkerDrive('offline-no-socket',process=SimpleNamespace(poll=lambda:None),
            worker_log=SimpleNamespace(close=lambda:None),output=tmp_path/'offline-worker',expected_boot='offline')
        rpc = []

        def transport(action,payload):
            rpc.append((action,copy.deepcopy(payload)))
            return owner.dispatch(action,payload)

        # The real worker's RPC transport is the only replaced client method.
        # Its status() implementation and all owner/preparation wrappers stay real.
        worker.request = transport
        started = []

        def worker_factory():
            started.append(worker)
            return worker

        deferred = DeferredDrive(worker_factory,{})
        boot = VehicleBootDrive(lambda cancelled:{},lambda context:None,lambda context:deferred,{})
        boot.phase,boot.context = 'esc_on',{}
        if enabled:
            # The production second preparation step creates/enables the actual
            # DeferredDrive; its real background _start forwards enable via RPC.
            boot._advance({'client':CLIENT},False)
            assert deferred.thread is not None
            deferred.thread.join(timeout=1)
            assert not deferred.thread.is_alive() and deferred.delegate is worker
            manual.tick()
            assert boot.phase=='ready'
        else:
            # A prepared worker may be observed before it is enabled. No owner
            # is invented and status must not run its factory or enable it.
            deferred.delegate,boot.delegate,boot.phase = worker,deferred,'ready'
        result = SimpleNamespace(now=now,clock=clock,backend=backend,manual=manual,
            owner=owner,worker=worker,deferred=deferred,boot=boot,rpc=rpc,started=started)
        stacks.append(result)
        return result

    yield create
    for stack in stacks:
        stack.owner.close()
        stack.manual.close()


def test_second_preparation_enable_reports_real_owner_through_all_wrappers(stack_factory):
    stack = stack_factory()
    assert stack.started==[stack.worker]
    assert stack.manual.owner==CLIENT and stack.manual.mode=='enabled'
    for node in (stack.owner,stack.worker,stack.deferred,stack.boot):
        state = node.status()
        assert state['owner_client']==CLIENT and state['mode']=='enabled'
    assert any(action=='enable' and payload['client']==CLIENT for action,payload in stack.rpc)


def test_stop_and_release_command_preserve_owner_and_enabled_state(stack_factory):
    stack = stack_factory()
    state = stack.boot.request('stop',{'client':CLIENT,'stop_source':'key_release'})
    assert state['owner_client']==CLIENT and state['mode']=='enabled'
    assert state['motor_pulse_us']==1500 and stack.manual.owner==CLIENT
    state = stack.boot.request('command',{'client':CLIENT,'setup_token':stack.boot.setup_token,
        'sequence':0,'motor':'stop','steering':'center'})
    assert state['owner_client']==CLIENT and state['mode']=='enabled'
    assert state['motor_pulse_us']==1500 and stack.manual.owner==CLIENT


def test_real_boot_borrow_succeeds_without_first_moving_or_sending_a_command(stack_factory):
    stack = stack_factory()
    assert not any(action=='command' for action,_ in stack.rpc)
    before = len(stack.backend.writes)
    handle = stack.boot.borrow_actuator(settings(),CLIENT)
    assert stack.owner.active and stack.owner.owner==CLIENT
    assert stack.deferred.delegate is stack.worker and stack.started==[stack.worker]
    assert stack.manual.mode=='emergency' and stack.manual.owner==CLIENT
    assert handle.startup_readback=={'warm_handoff':True,'physical_stop_verified':False}
    assert stack.backend.writes[before:]
    assert all(row[0] in ('neutral','steering') for row in stack.backend.writes[before:])
    assert not any(row[0]=='motion' for row in stack.backend.writes)
    handle.close()


def test_foreign_page_cannot_borrow_real_worker_or_write_output(stack_factory):
    stack = stack_factory()
    before = list(stack.backend.writes)
    rpc_start = len(stack.rpc)
    with pytest.raises(ValueError):
        stack.boot.borrow_actuator(settings(),'different-operator')
    assert stack.backend.writes==before and not stack.owner.active
    assert stack.manual.owner==CLIENT and stack.manual.mode=='enabled'
    assert not any(action=='autonomy_acquire' for action,_ in stack.rpc[rpc_start:])


@pytest.mark.parametrize('mode',['disabled','settling','emergency'])
def test_not_enabled_real_worker_is_rejected_before_acquire_without_writes(stack_factory,mode):
    stack = stack_factory(settling_s=10. if mode=='settling' else 0.)
    if mode in ('disabled','emergency'):
        stack.owner.dispatch('emergency',{})
        if mode=='disabled':stack.owner.dispatch('reset',{})
    assert stack.manual.mode==mode
    before = list(stack.backend.writes)
    rpc_start = len(stack.rpc)
    with pytest.raises(ValueError,match='启用遥控'):
        stack.boot.borrow_actuator(settings(),CLIENT)
    assert stack.backend.writes==before and not stack.owner.active
    assert not any(action=='autonomy_acquire' for action,_ in stack.rpc[rpc_start:])


@pytest.mark.parametrize('not_ready',['esc_off','esc_on','no_delegate','no_worker'])
def test_unprepared_real_wrappers_cannot_create_or_borrow_a_worker(stack_factory,not_ready):
    stack = stack_factory(enabled=False)
    if not_ready=='no_delegate':stack.boot.delegate=None
    elif not_ready=='no_worker':stack.deferred.delegate=None
    else:stack.boot.phase=not_ready
    before = list(stack.backend.writes)
    with pytest.raises(ValueError):
        stack.boot.borrow_actuator(settings(),CLIENT)
    assert stack.backend.writes==before and stack.started==[] and not stack.owner.active
    assert not any(action=='autonomy_acquire' for action,_ in stack.rpc)


def test_status_of_real_disabled_chain_does_not_enable_or_write(stack_factory):
    stack = stack_factory(enabled=False)
    before = list(stack.backend.writes)
    for _ in range(3):
        for node in (stack.owner,stack.worker,stack.deferred,stack.boot):
            state = node.status()
            assert state['mode']=='disabled' and state['owner_client'] is None
    assert stack.manual.owner is None and not stack.owner.active
    assert stack.backend.writes==before and stack.started==[]
    assert all(action=='status' for action,_ in stack.rpc)
