"""Recorded second-cone failures and one-sided boundary regressions."""
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from test_cone_latest_recovery import settings, geometry, nav
from carvision.cone_latest_engine import cone_image_guidance as guidance
from carvision.cone_latest_engine.cone_continuous import ContinuousConeController
from carvision.cone_latest_engine.cone_lane_margin import LaneMarginGuard
from carvision.cone_latest_engine.cone_low_speed import LowSpeedNavigator
from carvision.cone_latest_engine.cone_road import ConeRoadContext
from carvision.cone_latest_engine.cone_steering_policy import ConeHandoff

CASES=json.loads((Path(__file__).parent/'fixtures/cone-regression-20261107/second-cone-and-single-edge.json').read_text(encoding='utf8'))


def replay(case):
    start=case['rows'][0]['now_s'];controller=ContinuousConeController(case['settings'],start,initialized=True)
    return [(r['now_s']-start,controller.step(r['navigation'],r['now_s']),controller.progress()) for r in case['rows']]


@pytest.mark.parametrize('index,sign',[(0,1),(1,-1)])
def test_brief_handoff_preserves_inward_assist_and_still_observes_next_cone(index,sign):
    rows=replay(CASES[index]);handoff=[(t,i,p) for t,i,p in rows if 2.7<t<3.4]
    assert handoff
    for t,intent,progress in handoff:
        if progress['image_guidance']['handoff_recovery']:
            assert sign*(1610-intent['requested_steering_us'])>0
        assert progress['image_guidance']['target_id'] is not None


def test_recorded_single_right_stripe_retains_effective_inward_control():
    rows=replay(CASES[2]);selected=[(i,p) for t,i,p in rows if 1.8<t<3.]
    assert selected
    assert all(i['requested_steering_us']>=1700 for i,p in selected)
    margins=[p['image_guidance']['lane_margin'] for i,p in selected]
    assert any(m.get('right_risk',0)>.8 and m['reason']=='current_single_edge_margin' for m in margins)
    assert all(not m['position_verified'] for m in margins)


@pytest.mark.parametrize('case',CASES,ids=['21-59','22-00','22-01'])
def test_recorded_recovery_preserves_forward_floor_slew_and_no_pass_claim(case):
    rows=replay(case)
    for t,intent,progress in rows:
        assert intent['esc_us']>=1570
        assert progress['passed_count']==0
    for (ta,a,_),(tb,b,_) in zip(rows,rows[1:]):
        assert abs(b['steering_us']-a['steering_us'])<=600*min(.15,tb-ta)+1


def test_handoff_releases_after_two_distinct_settled_observations():
    handoff=ConeHandoff();handoff.begin()
    handoff.update(geometry(1.,-.06,.72));assert handoff.active
    handoff.update(geometry(1.1,0.,.60));assert handoff.active
    for _ in range(5):handoff.update(geometry(1.1,0.,.60))
    assert handoff.active
    handoff.update(geometry(1.2,0.,.60));assert not handoff.active


def test_next_cone_can_begin_inward_avoidance_during_recovery(settings,monkeypatch):
    current=[geometry(1.,0.,.5)]
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:current[0])
    g=guidance.ImageConeGuidance(settings)
    first=dict(id='first',image_fraction=.45,half_image_width=.03,y_fraction=.95,exit_band=True)
    g.update(nav(1.,[first]),1.)
    current[0]=geometry(1.9,-.06,.72)
    second=dict(id='second',image_fraction=.65,half_image_width=.03,y_fraction=.65)
    result=g.update(nav(1.9,[second]),1.9)
    assert result['target_id']=='second' and not result['handoff_recovery']
    assert result['handoff_reason']=='next_cone_approaching'
    assert result['command']<0  # The inward route is not delayed until exact center.


def test_next_route_is_reselected_after_yaw_recovery(settings,monkeypatch):
    current=[geometry(1.,0.,.5)]
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:current[0])
    g=guidance.ImageConeGuidance(settings)
    first=dict(id='first',image_fraction=.6,half_image_width=.03,y_fraction=.95,exit_band=True)
    g.update(nav(1.,[first]),1.)
    current[0]=geometry(1.9,.08,.3)
    second=dict(id='second',image_fraction=.6,half_image_width=.03,y_fraction=.52)
    assert g.update(nav(1.9,[second]),1.9)['side']=='left'
    for t in (2.,2.1):
        current[0]=geometry(t,0.,.5)
        result=g.update(nav(t,[{**second,'image_fraction':.35}]),t)
    assert not result['handoff_recovery']
    assert result['side']=='right' and result['command']>0
    assert result['identity_transition']=='handoff_route_reselected'


@pytest.mark.parametrize('side,sign',[('left',1),('right',-1)])
def test_current_single_edge_enforces_its_own_margin_without_inventing_other_edge(settings,side,sign):
    guard=LaneMarginGuard(settings)
    def single(t,gap):
        return dict(camera='secondary',captured_s=t,lane_observed_s=t,heading_only=True,
                    feedback_weight=.85,observed_edge_side=side,edge_clearance_fraction=gap,
                    edge_row_fraction=.9,heading_command=sign*.5)
    guard.constrain(-sign*.7,single(1.,.35),1.)
    command,report=guard.constrain(-sign*.7,single(1.1,.18),1.1)
    assert command*sign>=.5
    assert not report['position_verified'] and 'near_fraction' not in report
    for now in (1.12,1.18):
        _,duplicate=guard.constrain(-sign*.7,single(1.1,.18),now)
        assert duplicate['preview_edge_clearance_fraction']==report['preview_edge_clearance_fraction']
    unchanged,stale=guard.constrain(.2,single(1.1,.18),1.7)
    assert unchanged==.2 and not stale['applied']


def test_blue_background_at_border_cannot_seed_navigation_history(settings):
    image=np.full((360,480,3),(50,130,50),dtype=np.uint8)
    row=dict(id='board',bbox_xyxy=[418,158,450,211],foot_pixel=[434,201],
             boundary_relation='unknown',scene_supported=False,visible=True,confirmed=True,
             kind='cone_candidate',clipped=False)
    with patch('carvision.cone_latest_engine.cone_road.PaintContext.enrich',lambda *a:None):
        ConeRoadContext.enrich([row],image,{})
    assert not row['new_target_supported']  # Even with green floor below it.
    navigator=LowSpeedNavigator(settings)
    assert not navigator.image_guidance({'captured_s':1.,'tracks':[row]},480,360)['cones']
    assert 'board' not in navigator.near_history


def test_observed_single_edge_rejects_blue_object_outside_it():
    image=np.full((360,480,3),(50,130,50),dtype=np.uint8)
    row=dict(bbox_xyxy=[300,210,330,250],foot_pixel=[315,245],boundary_relation='unknown')
    edge=dict(side='right',points=[[260,180+y*15] for y in range(10)])
    with patch('carvision.cone_latest_engine.cone_road.PaintContext.enrich',lambda *a:None):
        ConeRoadContext.enrich([row],image,{'single_heading':edge})
    assert row['boundary_relation']=='outside' and not row['new_target_supported']
