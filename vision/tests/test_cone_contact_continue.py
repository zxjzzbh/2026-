"""Contact continuation changes cone policy, never lane/lease/start evidence."""
import pytest

from carvision.cone_continuous import ContinuousConeController,DEFAULT_STOP_FOR_CONES
from carvision.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard
from test_cone_turn_controller import lane,settings,started


def close_cone(**changes):
    return {'id':'close','fraction':.62,'half_width':.03,'near':True,
            'y_fraction':.75,'image_close':True,**changes}


def test_default_policy_is_explicit_in_neutral_motion_and_cached_intents():
    assert DEFAULT_STOP_FOR_CONES is False
    controller=ContinuousConeController(settings(),0.,initialized=True)
    assert controller.progress()['cone_contact_stop_enabled'] is False
    assert controller.step(lane(0.),0.)['cone_contact_stop_enabled'] is False
    assert controller.step(lane(.1),.1)['esc_us']==1500
    output=controller.step(lane(.2),.2)
    assert output['esc_us']==1600 and output['cone_contact_stop_enabled'] is False
    cached=controller.step(lane(.2),.21)
    assert not cached['fresh_control'] and cached['cone_contact_stop_enabled'] is False


@pytest.mark.parametrize('value',[None,0,1,'false',{},[]])
def test_contact_stop_policy_requires_an_explicit_boolean(value):
    with pytest.raises(ValueError,match='布尔值'):
        ContinuousConeController(settings(),0.,stop_for_cones=value)


@pytest.mark.parametrize('uncertain',[False,True])
def test_default_continues_for_confirmed_and_uncertain_near_cones_through_output_guard(uncertain):
    controller=started();cone=close_cone()
    nav=lane(.3,cones=[] if uncertain else [cone])
    if uncertain:nav.update(uncertain=['close'],uncertain_cones=[{**cone,'provisional':True}])
    output=controller.step(nav,.3)
    assert 1500<output['esc_us']<=1600 and output['cone_contact_stop_enabled'] is False
    guard=ConePulseGuard(actuator_settings(controller.s),lambda *args:None,mode='F/B')
    guard.accept({**output,'sequence':0,'sent_s':.3},.3)
    assert not guard.fault
    assert controller.count==0 and not output['tail_clearance_verified']


@pytest.mark.parametrize('uncertain',[False,True])
def test_explicit_stop_policy_preserves_confirmed_and_uncertain_near_cone_veto(uncertain):
    controller=started(stop_for_cones=True);cone=close_cone()
    nav=lane(.3,cones=[] if uncertain else [cone])
    if uncertain:nav.update(uncertain=['close'],uncertain_cones=[{**cone,'provisional':True}])
    assert controller.progress()['cone_contact_stop_enabled'] is True
    with pytest.raises(ValueError,match='净空冲突'):controller.step(nav,.3)


def test_default_blocked_cone_keeps_lane_target_without_manufacturing_a_pass():
    controller=started(handoff=True)
    for t,y in ((.3,.55),(.4,.63),(.5,.71),(.6,.73),(.7,.75)):
        output=controller.step(lane(t,cones=[close_cone(id='wide',fraction=.5,
                                  half_width=.4,y_fraction=y)]),t)
        assert 1500<output['esc_us']<=1600 and output['best_effort_route']
        assert .23<=controller.target_fraction<=.77
        assert controller.target=='wide' and controller.count==0 and controller.pass_gate_hits==0
        assert output['phase']!='complete' and not output['lane_return_confirmed']
    assert '不视为通过' in output['reason']


def test_explicit_stop_policy_still_rejects_a_cone_with_no_confirmed_gap():
    controller=started(stop_for_cones=True)
    with pytest.raises(ValueError,match='绕行间隙'):
        controller.step(lane(.3,cones=[close_cone(fraction=.5,half_width=.4)]),.3)


def test_default_locked_side_does_not_flip_when_its_gap_disappears():
    controller=started()
    controller.step(lane(.3,cones=[close_cone(fraction=.7,y_fraction=.55,image_close=False)]),.3)
    assert controller.side=='left';old_target=controller.target_fraction
    output=controller.step(lane(.4,cones=[close_cone(fraction=.3,y_fraction=.55,image_close=False)]),.4)
    assert output['best_effort_route'] and output['esc_us']>1500
    assert controller.side=='left' and controller.target_fraction==old_target
    assert controller.count==0 and controller.target=='close'


def test_default_gap_fallback_clamps_previous_target_to_current_lane_envelope():
    controller=started()
    controller.step(lane(.3,cones=[close_cone(fraction=.45,y_fraction=.55,image_close=False)]),.3)
    assert controller.side=='right' and controller.target_fraction>.62
    output=controller.step(lane(.4,half_envelope=.35,
        cones=[close_cone(fraction=.5,half_width=.4,y_fraction=.55,image_close=False)]),.4)
    assert output['best_effort_route'] and controller.target_fraction==pytest.approx(.62)
    assert controller.side=='right' and controller.count==0


def test_gap_fallback_uses_current_near_position_when_old_target_is_unavailable():
    controller=started();controller.target_fraction=None
    output=controller.step(lane(.3,near=.4,far=.4,
        cones=[close_cone(fraction=.5,half_width=.4,y_fraction=.55,image_close=False)]),.3)
    assert output['best_effort_route'] and controller.target_fraction==pytest.approx(.4)
    assert controller.count==0


def test_disabling_contact_stop_keeps_inward_boundary_correction():
    controller=started()
    output=controller.step(lane(.3,near=.1,far=.1,
        cones=[close_cone(fraction=.35)]),.3)
    assert output['boundary_correction']=='right' and output['requested_steering_us']<1610
    assert 1500<output['esc_us']<=1570 and not output['cone_contact_stop_enabled']


def test_contact_continuation_does_not_invent_space_when_lane_envelope_cannot_fit():
    controller=started()
    with pytest.raises(ValueError,match='白线内没有可容纳'):
        controller.step(lane(.3,half_envelope=.49,cones=[close_cone(half_width=.4)]),.3)


def test_only_explicit_stop_policy_calls_both_requested_and_slewed_clearance_checks(monkeypatch):
    for enabled in (False,True):
        controller=started(stop_for_cones=enabled);seen=[]
        monkeypatch.setattr(controller,'_guard_near_clearance',lambda nav,steer:seen.append(steer))
        output=controller.step(lane(.3,near=.4,far=.4),.3)
        if enabled:
            assert len(seen)==2 and round(seen[0])==controller.requested_steering
            assert seen[1]==controller.actual_steering
        else:assert seen==[]
        assert output['cone_contact_stop_enabled'] is enabled
