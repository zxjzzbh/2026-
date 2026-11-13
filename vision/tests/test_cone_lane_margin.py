"""Regressions for outward cone turns reaching the white line."""
import pytest
from carvision.cone_latest_engine.cone_lane_margin import LaneMarginGuard
from carvision.cone_latest_engine import cone_image_guidance as guidance
from test_cone_latest_recovery import settings, nav, geometry


def observed(t,q,heading=0.,camera='secondary'):
    return {**geometry(t,heading,q),'feedback_weight':1.,'camera':camera}


@pytest.mark.parametrize('sign',[-1,1])
def test_outward_heading_is_checked_before_the_line_is_reached(settings,sign):
    guard=LaneMarginGuard(settings)
    command,report=guard.constrain(sign*.9,observed(1.,.5,-sign*.06),1.)
    assert sign*command<0 and report['applied']
    assert report['left_risk']==report['right_risk']==0


def test_weak_countersteer_is_strengthened_while_heading_still_points_out(settings):
    guard=LaneMarginGuard(settings)
    command,report=guard.constrain(-.02,observed(1.,.57,-.05),1.)
    assert command==pytest.approx(-.40) and report['applied']


@pytest.mark.parametrize('sign',[-1,1])
def test_current_outward_trend_reduces_steering_early(settings,sign):
    guard=LaneMarginGuard(settings)
    q=lambda value:value if sign==1 else 1-value
    guard.constrain(sign,observed(1.,q(.60)),1.)
    command,report=guard.constrain(sign,observed(1.1,q(.69)),1.1)
    assert sign*command<0
    assert report['right_risk' if sign==1 else 'left_risk']==1
    assert abs(report['preview_fraction']-.5)>abs(q(.69)-.5)


def test_edge_guard_preserves_heading_correction_away_from_near_edge(settings):
    guard=LaneMarginGuard(settings)
    command,report=guard.constrain(-.48,observed(1.,.20,-.06),1.)
    assert command==pytest.approx(-.48) and not report['applied']


def test_two_cameras_do_not_share_position_velocity(settings):
    guard=LaneMarginGuard(settings)
    guard.constrain(0.,observed(1.,.2,camera='primary'),1.)
    _,report=guard.constrain(.5,observed(1.05,.6,camera='secondary'),1.05)
    assert report['estimated_fraction_rate_s']==0


def test_repeated_frame_does_not_add_predicted_travel(settings):
    guard=LaneMarginGuard(settings)
    guard.constrain(1.,observed(1.,.60),1.)
    _,before=guard.constrain(1.,observed(1.1,.69),1.1)
    for now in (1.11,1.15,1.2):
        _,after=guard.constrain(1.,observed(1.1,.69),now)
        assert after['preview_fraction']==before['preview_fraction']


def test_held_frame_does_not_measure_new_lateral_motion(settings):
    guard=LaneMarginGuard(settings)
    guard.constrain(1.,observed(1.,.50),1.)
    held={**observed(1.,.70),'line_held':True,'fresh_lane':False,'feedback_weight':.3}
    _,report=guard.constrain(1.,held,1.2)
    assert report['estimated_fraction_rate_s']==0 and guard.history['secondary'][1]==.50


def test_missing_feedback_does_not_fabricate_line_clearance(settings):
    guard=LaneMarginGuard(settings)
    for g in (None,{**observed(1.,.8),'feedback_weight':0.}):
        command,report=guard.constrain(.2,g,1.)
        assert command==.2 and not report['applied'] and not report['position_verified']


def test_recorded_right_cone_uses_nearer_interior_goal(monkeypatch,settings):
    # Replay values from run 1794042773021490451, before full-right steering.
    value=guidance.ImageConeGuidance(settings)
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:geometry(1.,.0198988,.459294))
    cone=dict(id='real',fraction=.444026,half_width=.03,image_fraction=.48333,
              half_image_width=.03,y_fraction=.741667,near=True)
    row={**nav(1.,[cone]),'line_missing':False,'lane_bounds_px':[[100,300],[0,400]],'ego_fraction':.459294}
    result=value.update(row,1.)
    assert result['side']=='right'
    assert .682<result['target_fraction']<.72  # Old target was .7520.
    assert result['target_fraction']<1-value.lane_margin.margin


def test_heading_alone_cannot_reverse_a_cone_manoeuvre_in_mid_lane(monkeypatch,settings):
    value=guidance.ImageConeGuidance(settings)
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:geometry(1.,-.05,.52))
    cone=dict(id='real',fraction=.44,half_width=.03,image_fraction=.44,
              half_image_width=.03,y_fraction=.75,near=True)
    row={**nav(1.,[cone]),'line_missing':False,'lane_bounds_px':[[100,300],[0,400]],'ego_fraction':.52}
    result=value.update(row,1.)
    assert result['side']=='right' and result['command']>0
    assert not result['lane_margin']['applied']
    assert result['target_fraction']>.7  # The target is retained while yaw is corrected.
