"""Measured propulsion floor and fresh single-stripe direction regressions."""
import numpy as np
import pytest
import json
from pathlib import Path

from test_cone_latest_recovery import settings,nav,geometry
from test_cone_turn_corridor import track
from carvision.cone_latest_engine.cone_power import output_request
from carvision.cone_latest_engine.cone_low_speed import trial_settings
from carvision.cone_latest_engine.cone_road import ConeRoadContext
from carvision.cone_latest_engine.cone_local_flow import local_observations
from carvision.cone_latest_engine.cone_image_geometry import image_lane_geometry
from carvision.cone_latest_engine import cone_image_guidance as guidance
from carvision.cone_latest_engine.cone_edge_recovery import TurnEdgeRecovery


@pytest.mark.parametrize('forward',[1570,1575,1600,1625])
def test_every_positive_power_scale_can_move_the_motor(forward):
    for scale in np.linspace(.001,1.,101):
        action,pulse=output_request(forward,float(scale))
        assert action!='neutral' and 1570<=pulse<=forward
    assert output_request(forward,0.)==('neutral',1500)


@pytest.mark.parametrize('pulse',[1500,1501,1530,1556,1569,1626])
def test_unusable_forward_setting_is_rejected(settings,pulse):
    with pytest.raises(ValueError,match='1570'):
        trial_settings(settings,{'test_forward_us':pulse})


def single_sequence(settings):
    settings['scene']['surface']='indoor_white_line_track'
    detector=ConeRoadContext(settings)
    for index,t in enumerate(np.arange(1.,3.01,.1)):
        t=float(t)
        image=track(sides=('left','right') if index==0 else ('left',),shift=0.)
        context=detector.inspect(image,np.zeros(image.shape[:2],np.uint8),frame_id=index,captured_s=float(t))
        yield t,context,detector


def test_current_stripe_survives_width_expiry_without_inventing_center(settings):
    rows=list(single_sequence(settings))
    t,context,detector=rows[-1]
    assert not context['valid']
    assert context['single_heading']['paired_observed_s']==1.
    course=local_observations({'secondary':np.array(settings['ground']['homography'])},
        {'secondary':{'captured_s':t,'road_context':context}},t,1.2)
    g=image_lane_geometry({**nav(t),'local_course':course},settings,now=t)
    assert g and g['heading_only'] and g['single_edge_observed']
    assert not g['nav_goal_compatible'] and not g['fresh_lane']
    assert g['near_ego_fraction'] is None and g['body_ego_fraction'] is None
    assert not image_lane_geometry({**nav(t),'local_course':course},settings,now=t+.6)
    blank=np.zeros((360,480,3),np.uint8)
    missing=detector.inspect(blank,np.zeros((360,480),np.uint8),frame_id=50,captured_s=t+.1)
    assert 'single_heading' not in missing


def test_single_direction_never_confirms_center_or_cone_pass(settings,monkeypatch):
    current={**geometry(1.,.02,None),'captured_s':1.,'heading_only':True,
             'single_edge_observed':True,'fresh_lane':False,'nav_goal_compatible':False}
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:current)
    guide=guidance.ImageConeGuidance(settings)
    for t in np.arange(1.,4.,.1):
        t=float(t)
        current.update(captured_s=t,lane_observed_s=t)
        result=guide.update(nav(t),t)
        assert result['phase']=='align' and not result['stop_required']
        assert result['power_scale']>0 and guide.recenter_aligned_hits==0
        assert abs(result['command'])<=.35


def test_lost_heading_does_not_idle_for_five_seconds_in_align(settings,monkeypatch):
    current=[geometry(1.,.06,.5)]
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:current[0])
    guide=guidance.ImageConeGuidance(settings)
    cone=dict(id='one',image_fraction=.95,half_image_width=.03,y_fraction=.85)
    assert guide.update(nav(1.,[cone]),1.)['phase']=='passing'
    current[0]=None
    for t in (1.1,1.4,1.7,2.):
        result=guide.update(nav(t,[cone]),t)
        assert not result['stop_required'] and output_request(1575,result['power_scale'])[1]>=1570
    assert guide.update(nav(2.11,[cone]),2.11)['stop_required']


def test_recorded_33px_pair_can_continue_tracking_the_visible_stripe(settings):
    rows=json.loads((Path(__file__).parent/'fixtures/cone-regression-20261107/short-pair-heading.json').read_text(encoding='utf8'))
    tracker=TurnEdgeRecovery();headings=[]
    for row in rows:
        t=row['t'];model=tracker.update(row['context'],(360,480,3),t)
        if row['context']['valid']:
            assert tracker.paired[1].high-tracker.paired[1].low==33.
        if tracker.current_heading:
            n={**nav(t),'local_course':{'single_heading':[{**tracker.current_heading,'camera':'secondary'}]}}
            g=image_lane_geometry(n,settings,now=t)
            assert g and g['heading_only'] and g['near_ego_fraction'] is None
            headings.append(g)
            if t-tracker.paired[0]>1.:assert model is None
    assert len(headings)>=8
    assert any(g['paired_reference_age_s']>1.5 for g in headings)
