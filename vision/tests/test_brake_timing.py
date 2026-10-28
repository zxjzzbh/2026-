"""Verify actual command edges without opening any actuator or camera."""
import copy
from types import SimpleNamespace

import pytest

from carvision.brake_timing import CROSSWALK_BRAKE_PULSE_S
from carvision.brake_output import PulseGuard
from carvision.brake_parking import BrakeParkingController,crosswalk_calibrated_settings
from carvision.race_flow import RaceFlowController,load_bundle
from carvision.race_flow_console import RaceFlowDrive
from test_crosswalk_profile_integration import profile,legacy,load_active,observation
from test_cone_short import Harness


def test_only_current_crosswalk_settings_get_the_tuned_pulse(tmp_path,legacy,profile):
    before=copy.deepcopy(legacy);active=load_active(tmp_path,legacy,profile)
    assert CROSSWALK_BRAKE_PULSE_S==.35 and active['brake_pulse_s']==.35
    assert legacy==before and legacy['brake_pulse_s']==.12
    assert active['actuator_lease_s']==legacy['actuator_lease_s']==.25
    assert active['max_brake_pulses']==legacy['max_brake_pulses']==3
    assert active['pwm']==legacy['pwm'] and active['brake_release_s']==legacy['brake_release_s']
    assert active['crosswalk_max_frame_age_s']==.5
    assert active['crosswalk_calibration']['profile_id']==profile['profile_id']


def test_legacy_without_current_calibration_keeps_its_existing_pulse(tmp_path,legacy):
    assert crosswalk_calibrated_settings(legacy,tmp_path)==legacy


def test_single_crosswalk_controller_and_output_both_brake_for_350ms(tmp_path,legacy,profile):
    active=load_active(tmp_path,legacy,profile);controller=BrakeParkingController(active,crosswalk=True)
    now=[0.];writes=[]
    guard=PulseGuard(active,lambda ch,p:writes.append((now[0],ch,p)),mode='F/B')
    for seq,t in enumerate((0.,.1,.2,.3,.349,.35)):
        now[0]=t
        sample=observation(t,near=342/359,moving=True)
        intent=controller.step(sample,t)
        assert intent['brake_pulse_s']==.35
        guard.accept({'sequence':seq,'sent_s':t,**{k:intent[k] for k in ('action','esc_us','steering_us')}},t)
        assert intent['action']==('brake' if t<.35 else 'neutral')
        assert guard.esc==(1400 if t<.35 else 1500)
    assert controller.brake_count==1 and not guard.fault
    assert guard.last_brake_end==pytest.approx(.35)


def test_full_flow_uses_the_same_350ms_cap_as_its_crosswalk_controller(monkeypatch):
    from test_race_flow import ROOT
    captured={}
    def factory(settings,**kwargs):captured.update(copy.deepcopy(settings));return SimpleNamespace(run=False)
    monkeypatch.setattr('carvision.race_flow_console.ConeTrialActuator',factory)
    flow=RaceFlowDrive(SimpleNamespace(status=lambda:{}),{},ROOT,live=False)
    assert not flow.error
    flow._make_actuator(flow.bundle)
    controller=RaceFlowController(flow.bundle,0.);controller._go('crosswalk',0.,'test')
    assert captured['brake_pulse_s']==controller.controller.s['brake_pulse_s']==.35
    assert captured['actuator_lease_s']==.2 and captured['brake_release_s']==.1
    writes=[];guard=PulseGuard(captured,lambda ch,p:writes.append((ch,p)),mode='F/B',forward_limit_us=1625)
    for seq,t in enumerate((0.,.1,.2,.3,.349,.35)):
        guard.accept({'sequence':seq,'sent_s':t,'action':'brake','esc_us':1400,'steering_us':1610},t)
        assert guard.esc==(1400 if t<.35 else 1500)
    assert guard.brake_until is None and not guard.fault


def test_first_detection_stop_gets_tuned_braking_without_changing_its_short_hold(tmp_path,legacy,profile):
    controller=BrakeParkingController(load_active(tmp_path,legacy,profile),crosswalk=True,first_detection_stop=True)
    initial=controller.step(observation(0.,near=310/359,moving=True),0.)
    assert initial['action']=='brake' and initial['stop_stage']=='first_detection'
    assert initial['first_detection_pause_s']==.5 and controller.s['hold_s']==10.
    out=controller.step(observation(.15,near=310/359,moving=True),.15)
    assert out['action']=='brake' and controller.pulse_until==.35


def test_straight_short_test_delivers_three_350ms_pulses_and_keeps_two_second_forward_time():
    h=Harness();h.send('start',mode='brake',forward_us=1600,duration_s=2.,expected_round=0)
    h.advance(8.)
    assert h.guard.phase=='complete' and h.guard.esc_us==1500
    assert 2.<=h.guard.brake_started_s-h.guard.drive_started_s<=2.026
    output=[(t,p) for t,ch,p in h.writes if ch==4]
    starts=[i for i,(t,p) in enumerate(output) if p==1400]
    assert len(starts)==3 and h.guard.snapshot(h.now)['brake_pulse_s']==.35
    for i in starts:
        end=next(t for t,p in output[i+1:] if p!=1400)
        assert .35-1e-9<=end-output[i][0]<=.376
    positive=sum(p>1500 for t,p in output)
    h.advance(12.)
    assert sum(ch==4 and p>1500 for t,ch,p in h.writes)==positive


@pytest.mark.parametrize('mode',['turn_1350','turn_1870'])
def test_fixed_turn_calibration_keeps_its_120ms_braking(mode):
    h=Harness();h.start(mode);h.advance(8.)
    assert h.guard.brake_pulse_s==.12 and h.guard.phase=='complete'


def test_frozen_brake_messages_cannot_extend_the_new_pulse(tmp_path,legacy,profile):
    active=load_active(tmp_path,legacy,profile);guard=PulseGuard(active,lambda ch,p:None,mode='F/B')
    for seq,t in enumerate((0.,.05,.1,.15,.19,.2,.25,.3,.35,.4,.45,.5,.6,.7)):
        guard.accept({'sequence':seq,'sent_s':t,'action':'brake','esc_us':1400,'steering_us':1610},t)
    assert guard.esc==1500 and guard.brake_until is None and guard.last_brake_end==.35


def test_short_test_disconnect_still_aborts_without_extending_drive_or_braking():
    h=Harness();h.send('start',mode='brake',forward_us=1600,duration_s=2.,expected_round=0);h.advance(3.5)
    h.now+=.21;h.guard.tick(h.now)
    assert h.guard.fault and h.guard.esc_us==1400
    deadline=h.guard.emergency_until
    h.now+=.30;h.guard.tick(h.now);assert h.guard.esc_us==1400 and h.guard.emergency_until==deadline
    h.now+=.051;h.guard.tick(h.now);assert h.guard.esc_us==1500 and h.guard.emergency_until is None


@pytest.mark.parametrize('trigger',[.4,.5,.65])
def test_synthetic_first_sighting_stays_before_the_saved_stop_point(tmp_path,legacy,profile,trigger):
    from flow_calibration_fixture import current_crosswalk_sample
    from carvision.crosswalk_calibration import reference_y
    active=load_active(tmp_path,legacy,profile,trigger)
    controller=SimpleNamespace(b={'brake':active})
    far=current_crosswalk_sample(controller,{'candidate':True,'far_edge_y_normalized':.5})
    stop=current_crosswalk_sample(controller,{'candidate':True,'far_edge_y_normalized':.75})
    stop_y=reference_y(profile,'secondary',trigger)['near_edge_y_normalized']
    assert far['near_edge_y_normalized']<stop_y
    assert stop['near_edge_y_normalized']==pytest.approx(stop_y)
    assert active['crosswalk_distance_trigger_m']==trigger


def test_lease_expiry_during_existing_350ms_pulse_keeps_its_original_end(tmp_path,legacy,profile):
    active=load_active(tmp_path,legacy,profile)
    guard=PulseGuard(active,lambda ch,p:None,mode='F/B')
    guard.accept({'sequence':0,'sent_s':0.,'action':'brake','esc_us':1400,'steering_us':1610},0.)
    # A lease timeout latches a fault, but cannot restart or extend this pulse.
    guard.tick(.25)
    assert guard.fault and guard.emergency_until==.35 and guard.esc==1400
    guard.tick(.349);assert guard.esc==1400 and guard.emergency_until==.35
    guard.tick(.35);assert guard.esc==1500 and guard.emergency_until is None


def test_stop_during_forward_is_bounded_to_350ms_and_never_restarts_drive(tmp_path,legacy,profile):
    active=load_active(tmp_path,legacy,profile);writes=[]
    guard=PulseGuard(active,lambda ch,p:writes.append((ch,p)),mode='F/B')
    guard.accept({'sequence':0,'sent_s':0.,'action':'search','esc_us':active['pwm']['search_us'],'steering_us':1610},0.)
    guard.abort('stop',.05)
    assert guard.esc==1400 and guard.emergency_until==pytest.approx(.40)
    guard.tick(.399);assert guard.esc==1400
    guard.tick(.401);assert guard.esc==1500 and guard.emergency_until is None
    forward_count=sum(ch==4 and p>1500 for ch,p in writes)
    guard.tick(5.)
    assert sum(ch==4 and p>1500 for ch,p in writes)==forward_count
