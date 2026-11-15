"""Actual standard debug session + local sender + PWM guard, without I/O."""
import pytest
import json

from carvision.cone_command import ConeCommandSender
from carvision.cone_execution import ConeRun
from carvision.cone_low_speed import PROFILE
from carvision.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard
from carvision.cone_recovery import ConeRecovery
from test_cone_turn_controller import settings, lane
from test_race_sender_cadence import GuardActuator, advance


class Session:
    def __init__(self,tmp_path):
        self.clock=[0.];self.actuator=GuardActuator(lambda:self.clock[0])
        self.actuator.guard=ConePulseGuard(actuator_settings(settings()),lambda *_:None,mode='F/B')
        self.sender=ConeCommandSender(self.actuator,clock=lambda:self.clock[0],threaded=False)
        self.run=ConeRun(lambda _:self.sender,tmp_path)
        self.run.prepare(settings(),{'mode':'disabled','boot_test_phase':'esc_off','setup_token':'boot'},
            {'profile':PROFILE,'client':'page','setup_token':'boot','esc_off':True,'fb_mode_confirmed':True},0.)
        self.heartbeat=0
        for t in (.1,.2,.3):self.step(t)
        self.run.command('cone_start',{'client':'page','run_token':self.run.token,'trial_sequence':0,
            'fb_mode_confirmed':True,'esc_on_static':True,'field_ready':True},.3)
        for i in range(1,33):self.step(.3+i*.1)
        assert self.run.active and self.run.intent['esc_us']>=1570, self.run.reason

    def step(self,t,*,nav=None,motion=None):
        advance((self.clock,self.actuator,self.sender),round(t,8))
        self.heartbeat+=1
        self.run.heartbeat({'client':'page','run_token':self.run.token,
            'heartbeat_sequence':self.heartbeat},t)
        self.run.tick(lane(t) if nav is None else nav,t,motion=motion)

    def gap(self):
        advance((self.clock,self.actuator,self.sender),self.clock[0]+.8)
        assert self.sender.race_interrupted() and self.actuator.guard.esc==1400

    def close(self):self.run.cancel('offline test completed')


def still(t,since=None,**changes):
    return {'captured_s':t,'valid':True,'moving':False,
            'continuous_still_since_s':since,**changes}


def test_calculation_gap_retains_task_and_recovers_only_after_stillness_and_new_frames(tmp_path):
    test=Session(tmp_path);run=test.run;controller=run.controller;token=run.token
    try:
        test.gap();start=test.clock[0]
        recovered=None
        for i in range(14):
            t=round(start+i*.1,8)
            test.step(t,motion=still(t,start))
            assert run.active and run.controller is controller and run.token==token
            assert test.actuator.guard.fault is None, run.reason
            if run.intent.get('control_recovered'):
                recovered=t;assert run.intent['esc_us']==1500;break
            assert run.intent['esc_us']<=1500
        assert recovered is not None,run.reason
        # The exact recovery frame cannot also become a new motion decision.
        previous=lane(recovered)
        run.tick(previous,recovered+.01,motion=still(recovered+.01,start))
        assert test.actuator.guard.esc==1500
        for i in range(1,5):test.step(recovered+i*.1,motion=still(recovered+i*.1,start))
        assert run.intent['esc_us']>=1570 and run.phase=='driving'
        assert run.first_stop is None and not test.sender.race_interrupted()
        records=[json.loads(line) for line in (run.folder/'decisions.jsonl').read_text(encoding='utf-8').splitlines()]
        assert any(row['intent'].get('control_recovered') for row in records)
        assert any(row['intent']['phase']=='recovering' and row['motion'] for row in records)
        brakes=[c for c in test.actuator.commands if c['action']=='brake']
        assert len(brakes)==1  # finite brake is not repeated by visual updates
    finally:test.close()


@pytest.mark.parametrize('kind',['missing','moving','unknown','stale','future'])
def test_fresh_lane_alone_never_recovers_without_current_stillness(tmp_path,kind):
    test=Session(tmp_path)
    try:
        test.gap();start=test.clock[0]
        for i in range(16):
            t=round(start+i*.1,8)
            motion={'missing':{},'moving':still(t,start,moving=True),
                    'unknown':still(t,start,valid=False,moving=None),
                    'stale':still(t-.5,start),'future':still(t+.1,start)}[kind]
            test.step(t,motion=motion)
            assert test.run.active and test.run.intent['esc_us']<=1500
            assert not test.run.intent.get('control_recovered')
        assert test.run.phase=='recovering'
    finally:test.close()


def test_guidance_timeout_pauses_instead_of_discarding_the_debug_run(tmp_path):
    test=Session(tmp_path)
    try:
        start=test.clock[0]
        for i in range(1,14):
            t=round(start+i*.1,8)
            test.step(t,nav=lane(t,state='missing'),motion=still(t,start,moving=True))
        assert test.run.active and test.run.phase=='recovering'
        assert test.run.intent['esc_us']<=1500 and test.run.first_stop is None
        start=test.clock[0]
        for i in range(1,17):
            t=round(start+i*.1,8);test.step(t,motion=still(t,start))
        assert test.run.active and test.run.intent['esc_us']>=1570
    finally:test.close()


def test_heartbeat_renewal_during_recovery_cannot_extend_old_forward_decision(tmp_path):
    test=Session(tmp_path)
    try:
        deadline=test.sender.race_deadline
        test.run.heartbeat({'client':'page','run_token':test.run.token,'heartbeat_sequence':500},test.clock[0])
        assert test.sender.race_deadline==deadline
        test.gap();test.step(test.clock[0],motion={})
        assert test.run.active and test.run.phase=='recovering'
        test.run.cancel('operator stopped',source='stop_button',now=test.clock[0])
        assert not test.run.active and test.run.phase=='cancelled' and test.actuator.closed
    finally:test.close()


def test_page_expiry_during_recovery_still_retires_debug_ownership(tmp_path):
    test=Session(tmp_path)
    try:
        test.gap();test.step(test.clock[0],motion={})
        t=test.run.lease_until+.01
        advance((test.clock,test.actuator,test.sender),t)
        test.run.tick(lane(t),t,motion=still(t,t-1))
        assert not test.run.active and test.run.phase=='fault'
        assert '心跳' in test.run.reason
        assert test.actuator.closed
    finally:test.close()


def test_unknown_feedback_cannot_be_bridged_by_old_continuous_still_claim():
    r=ConeRecovery();r.update(lane(1.),still(1.),1.,stale=True)
    r.update(lane(1.7),still(1.7,valid=False,moving=None),1.7)
    for t in (1.8,1.9,2.):
        out=r.update(lane(t),still(t,1.),t)
        assert not out.get('control_recovered')


@pytest.mark.parametrize('failure',['disk','serialization'])
def test_recording_failure_is_reported_without_stopping_control(tmp_path,failure):
    test=Session(tmp_path)
    try:
        t=test.clock[0]+.1;data=lane(t)
        if failure=='disk':
            test.run.log.close()
            class BrokenLog:
                closed=False
                def write(self,_):raise OSError('disk full')
                def close(self):self.closed=True
            test.run.log=BrokenLog()
        else:data['diagnostic_only']=object()
        test.step(t,nav=data)
        assert test.run.active and test.run.intent['esc_us']>=1570
        assert test.run.record_error
        assert test.run.status(settings(),{},t)['record_error']==test.run.record_error
        assert test.run.first_stop is None
    finally:test.close()
