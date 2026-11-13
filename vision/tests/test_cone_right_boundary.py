"""Centimetre boundary requests use the retained mapping, not image fractions."""
import copy
import math

import numpy as np
import pytest

from carvision.cone_latest_engine.cone_right_boundary import RightBoundaryRecovery
from carvision.cone_latest_engine.cone_side_choice import ConeRouteChoice
from carvision.cone_latest_engine import cone_image_guidance as guidance
from test_cone_latest_recovery import settings,nav,geometry
from test_cone_route_lock import CHOICES,choose,CASES


def observation(settings,t,distance,*,slope=0.,single=False):
    center=(settings['geometry']['rear_axle_to_front_m']-settings['geometry']['rear_overhang_m'])/2
    h=np.asarray(settings['ground']['homography'],float)
    def stripe(d):
        y=np.linspace(.8,2.5,15)
        x=slope*(y-center)+d*math.hypot(1.,slope)
        p=np.column_stack((x,y,np.ones(len(y))))@np.linalg.inv(h).T
        return (p[:,:2]/p[:,2,None]).tolist()
    row=dict(camera='secondary',captured_s=t,right=stripe(distance),
             left=stripe(distance-settings['geometry']['lane_width_m']))
    if single:
        return {'local_course':{'single_heading':[dict(camera='secondary',captured_s=t,
            paired_observed_s=t-.2,side='right',points=row['right'])]}}
    return {'local_course':{'image_lane':[row]}}


@pytest.mark.parametrize('width',[1.17,1.25])
@pytest.mark.parametrize('slope',[-.4,0.,.4])
def test_vehicle_center_reference_and_heading_do_not_confuse_threshold(settings,width,slope):
    settings['geometry']['lane_width_m']=width
    guard=RightBoundaryRecovery(settings)
    centered=guard.observe(observation(settings,1.,width/2,slope=slope),1.)
    assert centered['available'] and not centered['active']
    result=guard.observe(observation(settings,1.1,.49,slope=slope),1.1)
    assert result['active'] and result['right_distance_m']==pytest.approx(.49)
    assert not result['metric_verified']
    assert result['reference']=='vehicle_geometric_center_to_right_inner_edge'


def test_five_centimetre_hysteresis_and_progressive_response(settings):
    guard=RightBoundaryRecovery(settings)
    for t,d,active in [(1.,.50,False),(1.1,.49,True),(1.2,.51,True),(1.3,.549,True),
                       (1.4,.55,False),(1.5,.52,False)]:
        report=guard.observe(observation(settings,t,d),t)
        assert report['active']==active
    requests=[]
    for t,d in [(2.,.49),(2.1,.4),(2.2,.3)]:
        guard.observe(observation(settings,t,d),t)
        requests.append(guard.constrain(.75,left_allowed=True))
    assert -.5<=requests[2]<requests[1]<requests[0]<0


def test_current_right_stripe_does_not_require_left_stripe_or_full_pose(settings):
    guard=RightBoundaryRecovery(settings)
    report=guard.observe(observation(settings,1.,.4,single=True),1.)
    assert report['active'] and report['left_distance_m'] is None


def test_invalid_stale_other_camera_and_image_only_data_never_become_metres(settings):
    guard=RightBoundaryRecovery(settings)
    row=observation(settings,1.,.4)
    for data,now in [({},1.),(row,1.6),({'near_ego_fraction':.9},1.)]:
        report=guard.observe(data,now)
        assert not report['available'] and guard.constrain(.3,left_allowed=True)==.3
    wrong=copy.deepcopy(row);wrong['local_course']['image_lane'][0]['camera']='primary'
    assert not guard.observe(wrong,1.)['available']
    held=copy.deepcopy(row);held['local_course']['image_lane'][0]['line_held']=True
    assert not guard.observe(held,1.)['available']
    settings['ground']['image_size']=[640,480]
    assert not guard.observe(row,1.)['available']


def test_stale_feedback_cannot_keep_left_command_or_latch_across_gap(settings):
    guard=RightBoundaryRecovery(settings)
    assert guard.observe(observation(settings,1.,.4),1.)['active']
    assert not guard.observe({},1.6)['active']
    assert guard.constrain(.2,left_allowed=True)==.2
    assert not guard.observe(observation(settings,1.7,.52),1.7)['active']


def test_cone_conflict_unwinds_right_without_inventing_a_left_route(settings):
    guard=RightBoundaryRecovery(settings)
    guard.observe(observation(settings,1.,.4),1.)
    assert guard.constrain(.75,left_allowed=False)==0.
    assert guard.report['reason']=='unwind_right_without_crossing_cone'


def test_left_boundary_correction_is_not_overridden(settings):
    guard=RightBoundaryRecovery(settings)
    guard.observe(observation(settings,1.,.4),1.)
    assert guard.constrain(.3,left_allowed=True,left_risk=.3)==.3
    assert guard.report['reason']=='left_boundary_limits_right_margin_recovery'


def test_unstable_right_fit_does_not_trigger_a_centimetre_request(settings):
    guard=RightBoundaryRecovery(settings)
    row=observation(settings,1.,.4)
    points=row['local_course']['image_lane'][0]['right']
    for i,p in enumerate(points):p[0]+=35 if i%2 else -35
    assert not guard.observe(row,1.)['available']
    assert guard.constrain(.3,left_allowed=True)==.3


def test_observed_close_left_stripe_blocks_new_left_preference(settings):
    guard=RightBoundaryRecovery(settings)
    row=observation(settings,1.,.4)
    close_left=observation(settings,1.,-.20)['local_course']['image_lane'][0]['right']
    row['local_course']['image_lane'][0]['left']=close_left
    report=guard.observe(row,1.)
    assert report['active'] and report['left_limited'] and not guard.prefer_left
    assert guard.constrain(.3,left_allowed=True)==.3


def test_boundary_preference_can_review_right_lock_but_not_blocked_left(settings):
    route=ConeRouteChoice(settings)
    choose(route,1.);choose(route,1.1)
    assert route.committed and route.side=='right'
    choose(route,1.2,prefer_left=True)
    c,r=choose(route,1.3,prefer_left=True)
    assert c[1]=='left' and r['boundary_preference']=='left'
    route.reset()
    c,r=choose(route,2.,left=.29,right=.6,prefer_left=True)
    assert c[1]=='right' and r['blocked_sides']==['left']


def test_boundary_preference_never_switches_across_near_cone(settings):
    route=ConeRouteChoice(settings)
    choose(route,1.);choose(route,1.1)
    for t in (1.2,1.3):
        c,r=choose(route,t,prefer_left=True,target=dict(x=.5,y_fraction=.95),ego=.8)
    assert c is None and r['reason']=='near_cone_crossing_not_supported'


def test_guidance_integrates_current_distance_and_recovers_gently(monkeypatch,settings):
    value=guidance.ImageConeGuidance(settings)
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:geometry(1.,0.,.5))
    row={**nav(1.,[]),**observation(settings,1.,.49)}
    result=value.update(row,1.)
    assert result['right_boundary']['active'] and -.2<result['command']<0
    assert result['power_scale']<=.65


def test_recorded_right_stripe_activates_before_second_cone(settings):
    case=next(c for c in CASES if c['name'].endswith('1794065367294961667'))
    guard=RightBoundaryRecovery(case['settings'])
    active=[];start=case['rows'][0]['now_s']
    for row in case['rows']:
        report=guard.observe(row['navigation'],row['now_s'])
        if report['active']:active.append((row['now_s']-start,report))
    assert active and active[0][0]<1.7
    assert any(r['right_distance_m']<.4 for t,r in active)
    assert all(not r['metric_verified'] for t,r in active)
