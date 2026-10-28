"""Shared normal steering through race decisions and fake output guards."""
import pytest

from carvision.lane_following import LaneFollower,advance_steering
from carvision.race_flow import RaceFlowController,load_bundle
from carvision.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard
from test_cone_turn_controller import ROOT,lane,settings
from test_race_flow import Frames


@pytest.mark.parametrize('stage',['crosswalk','traffic','parking_search'])
@pytest.mark.parametrize('near',[.46,.54])
def test_all_normal_race_stages_use_small_proportional_gain_one_requests(stage,near):
    flow=RaceFlowController(load_bundle(ROOT),0.,initialized=True)
    flow._go(stage,0.,'test')
    result=flow._guided(lane(.1,near=near,far=near),.1)
    assert result['effective_steering_gain']==1.
    assert 1350<result['steering_us']<1870
    assert abs(result['steering_us']-1610)<60
    assert (result['steering_us']-1610)*(near-.5)>0


def test_multi_row_lookahead_changes_direction_before_near_error_appears():
    follower=LaneFollower(settings(gain=1.5),0.)
    result=follower.update(lane(.1,near=.5,far=.35),.1)
    assert result['lateral_error']==0 and result['lookahead_error']>0
    assert result['steering_us']<1610 and result['effective_steering_gain']==1.


def test_unknown_is_not_reported_as_an_observed_centreline():
    follower=LaneFollower(settings(),0.)
    result=follower.update(lane(.1,state='missing'),.1)
    assert result['forward_limit_us']==1500
    assert result['lane_reference_source']=='no_observed_lane' and result['lateral_error'] is None
    assert result['line_search'] and not result['boundary_observed']


def test_missing_after_current_pair_is_bounded_and_does_not_reset_budget():
    follower=LaneFollower(settings(),0.)
    follower.update(lane(.1,near=.6,far=.6),.1)
    for t in (.2,.3,.4):
        result=follower.update(lane(t,state='missing'),t)
        assert result['forward_limit_us']<=1570 and abs(result['steering_us']-1610)<=80
        assert result['lane_reference_source']=='bounded_previous_request'
    with pytest.raises(ValueError,match='0.4 秒'):
        follower.update(lane(.5,state='missing'),.5)


def test_urgent_outward_output_is_withdrawn_without_forward_power():
    follower=LaneFollower(settings(),0.)
    follower.observe_output(1870,.1)
    result=follower.update(lane(.2,near=.1,far=.1,boundary_correction='right'),.2)
    assert result['boundary_output_withdrawal'] and result['forward_limit_us']==1500
    assert result['steering_us']==1610 and result['requested_steering_us']<1610
    result=follower.update(lane(.3,near=.1,far=.1,boundary_correction='right'),.3)
    assert not result['boundary_output_withdrawal'] and result['steering_us']<1610


def test_soft_warning_retains_controlled_slew_without_a_blanket_stop():
    output,withdraw=advance_steering(1870,1350,.1,boundary_turn='right',near=.25,half=.2)
    assert output==1810 and not withdraw


def test_stage_change_preserves_output_without_a_new_arming_delay():
    flow=RaceFlowController(load_bundle(ROOT),0.,initialized=True)
    flow._go('crosswalk',0.,'test')
    first=flow._guided(lane(.1,near=.6,far=.6),.1)
    flow.last_t=.1;flow._out(intent=first)
    guide=flow.guide;old=guide.actual
    flow._go('traffic',.1,'test')
    assert flow.guide is guide and guide.actual==old
    result=flow._guided(lane(.2,near=.6,far=.6),.2)
    assert result['esc_us']>1500 and abs(result['steering_us']-old)<=60


def test_new_red_brake_is_not_filtered_by_a_repeated_lane_frame():
    flow=RaceFlowController(load_bundle(ROOT),0.,initialized=True)
    flow._go('traffic',0.,'test')
    nav=lane(.1,fid=1)
    flow._guided(nav,.1)
    assert flow._guided(nav,.1)['fresh_control'] is False
    result=Frames(flow).step(color='red')
    assert result['action']=='brake' and result['esc_us']==1400
    assert result.get('fresh_control') is not False


def test_normal_reduced_profile_and_boundary_neutral_pass_the_actual_guard():
    flow=RaceFlowController(load_bundle(ROOT),0.,initialized=True)
    flow._go('crosswalk',0.,'test')
    command=flow._guided(lane(.1,near=.68,far=.68),.1)
    flow.last_t=.1;command=flow._out(intent=command)
    assert command['action']=='cone_1580'
    guard=ConePulseGuard(actuator_settings(flow.b['cones']),lambda *args:None,mode='F/B')
    guard.accept({**command,'sequence':0,'sent_s':.1},.1)
    flow.guide.observe_output(1870,.1)
    command=flow._guided(lane(.2,near=.1,far=.1,boundary_correction='right'),.2)
    guard.accept({**command,'sequence':1,'sent_s':.2},.2)
    assert not guard.fault and guard.esc==1500 and guard.actual_steering==1610
