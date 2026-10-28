"""Cone recovery intents through real flow/worker guards with fake I/O only."""
from types import SimpleNamespace

import pytest

from carvision.cone_execution import ConeRun
from carvision.cone_low_speed import PROFILE
from carvision.cone_navigation import actuator_settings
from carvision.race_flow import RaceFlowController,load_bundle
from carvision.race_flow_console import RaceFlowDrive
from carvision.shared_executor import BorrowedActuator
from test_cone_turn_controller import ROOT,lane,started
from test_shared_executor import CLIENT,Worker,rig


@pytest.mark.parametrize('changes,pulse',[
    ({},1600),
    ({'near':.5,'far':.4},1580),
    ({'near':.5,'far':.3},1570),
    ({'state':'single'},1570),
])
def test_race_cone_intent_preserves_recovery_limit_through_shared_worker(rig,changes,pulse):
    now,clock,backend,manual,owner,_=rig
    worker=Worker(owner)
    view=SimpleNamespace(borrow_actuator=lambda settings,client:
        BorrowedActuator(worker,settings,client,clock=clock))
    bundle=load_bundle(ROOT)
    handle=RaceFlowDrive._make_actuator(SimpleNamespace(drive=view,owner=CLIENT,live=False),bundle)
    frozen=worker.calls[0][1]['settings']
    assert frozen['pwm']['search_us']==1600
    assert frozen['cone_forward_us']==[1570,1580]
    controller=RaceFlowController(bundle,0.,initialized=True)
    controller._go('cones',0.,'offline cone-stage integration')
    for index,t in enumerate((.1,.2,.3,.4),1):
        nav=lane(t,**(changes if index==4 else {}))
        output=controller.step({'sequence':index,'captured_s':t,'navigation':nav},t)
        now[0]=t;handle.send(output)
        if index<3:assert output['esc_us']==1500
    assert controller.stage=='cones' and output['esc_us']==pulse
    assert output['action']==('search' if pulse==1600 else 'cone_'+str(pulse))
    assert owner.guard.esc==pulse and not owner.guard.fault
    assert backend.writes[-1][0]=='motion' and backend.writes[-1][2]==pulse
    # The dedicated accepts may not alter other stages' frozen PWM setpoints.
    assert owner.guard.s['pwm']['search_us']==1600
    assert owner.guard.s['pwm']['creep_us']==bundle['brake']['pwm']['creep_us']
    now[0]=.41;handle.close();now[0]=.77;owner.tick()


def test_real_single_trial_caller_does_not_send_cached_moving_intent_or_renew_shared_lease(rig):
    now,clock,backend,manual,owner,_=rig
    controller=started()
    worker=Worker(owner)
    handle=BorrowedActuator(worker,actuator_settings(controller.s),CLIENT,clock=clock)
    run=ConeRun()
    run.active=True;run.profile=PROFILE;run.s=controller.s
    run.controller=controller;run.actuator=handle;run.lease_until=10.
    sample=lane(.3,near=.5,far=.3)
    now[0]=.3;run.tick(sample,.3)
    assert owner.guard.esc==1570 and run.intent['fresh_control']
    requests=len(worker.calls);deadline=owner.guard.deadline
    for t in (.32,.36,.42):
        now[0]=t;run.tick(sample,t)
        assert not run.intent['fresh_control'] and run.active
        assert len(worker.calls)==requests and owner.guard.deadline==deadline
    now[0]=.501;owner.tick()
    assert not owner.active and owner.guard.fault
    assert owner.guard.esc==1400
    assert any(row[0]=='motion' and row[2]==1400 and row[3]<=.2 for row in backend.writes)
    now[0]=.852;owner.tick()
    motor_writes=[row for row in backend.writes if row[0] in ('motion','neutral')]
    assert owner.guard is None and motor_writes[-1][0]=='neutral'
    run.cancel('offline test cleanup',now=.86)


@pytest.mark.parametrize('action,pulse',[
    ('cone_1560',1560),('cone_1570',1580),('cone_1580',1600),('search',1570),
])
def test_shared_worker_rejects_unfrozen_or_mismatched_cone_recovery_pulses(rig,action,pulse):
    now,clock,backend,manual,owner,settings=rig
    settings['cone_forward_us']=[1570,1580]
    handle=BorrowedActuator(Worker(owner),settings,CLIENT,clock=clock)
    before=len(backend.writes)
    with pytest.raises((ValueError,RuntimeError)):
        handle.send({'action':action,'esc_us':pulse,'steering_us':1610})
    assert not any(row[0]=='motion' and row[2]>1500 for row in backend.writes[before:])
