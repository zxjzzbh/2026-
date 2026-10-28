"""Actual output guard accepts only the frozen lower obstacle setpoints."""
import copy

import pytest

from carvision.cone_evidence import StablePairedLane
from carvision.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard
from carvision.race_flow import RaceFlowController
from test_cone_low_speed import settings, observation
from test_cone_continuous import continuous


def test_only_distinct_same_camera_frames_can_unlock_start():
    gate=StablePairedLane()
    nav=observation(1.,cones=[])
    assert not gate.update(nav,1.)
    assert not gate.update(nav,1.1)
    nav=observation(1.1,cones=[])
    assert not gate.update(nav,1.1)
    nav=observation(1.2,cones=[])
    assert gate.update(nav,1.2)
    assert not gate.ready(1.46)
    nav=observation(1.3,cones=[],line_state='single',line_held=True,boundary_observed=False)
    assert not gate.update(nav,1.3)
    assert gate.count==0


@pytest.mark.parametrize('pulse',[1570,1580])
def test_limited_commands_reach_real_guard_without_broadening_profile(settings,pulse):
    profile=actuator_settings(continuous(settings))
    writes=[]
    guard=ConePulseGuard(profile,lambda *args:writes.append(args),mode='F/B')
    guard.accept({'action':'cone_'+str(pulse),'esc_us':pulse,'steering_us':1650,
                  'sequence':0,'sent_s':1.},1.)
    assert guard.esc==pulse and (4,pulse) in writes
    guard.tick(1.201)
    assert guard.fault=='command lease expired'
    assert guard.esc<=1500


@pytest.mark.parametrize('action,pulse',[('cone_1600',1600),('cone_1570',1580),('search',1570)])
def test_unfrozen_or_mismatched_lower_commands_fail_closed(settings,action,pulse):
    guard=ConePulseGuard(actuator_settings(continuous(settings)),lambda *args:None,mode='F/B')
    with pytest.raises(ValueError):
        guard.accept({'action':action,'esc_us':pulse,'steering_us':1610,'sequence':0,'sent_s':1.},1.)
    assert guard.fault is not None


def test_limited_setpoints_cannot_exceed_selected_forward(settings):
    profile=actuator_settings(continuous(settings))
    profile['pwm']['search_us']=profile['pwm']['creep_us']=1550
    with pytest.raises(ValueError,match='frozen forward'):
        ConePulseGuard(profile,lambda *args:None,mode='F/B')


def test_flow_preserves_frozen_reduced_setpoints_for_shared_lane_guidance():
    flow=object.__new__(RaceFlowController)
    flow.stage='cones';flow.reason='test';flow.guide=None;flow.b={'brake':{'pwm':{'creep_us':1575}}}
    intent={'action':'cone_1570','esc_us':1570,'steering_us':1650,'fresh_control':False}
    result=flow._out(intent=intent)
    assert result['action']=='cone_1570' and result['fresh_control'] is False
    flow.stage='traffic'
    assert flow._out(intent=intent)['action']=='cone_1570'
