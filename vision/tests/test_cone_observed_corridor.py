"""Observed white edges stay valid when an unrelated target is outside them."""
import copy
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.cone_corridor import measured_corridor
from carvision.cone_low_speed import LowSpeedNavigator
from carvision.cone_planning import load_settings
from carvision.cone_road import ConeRoadContext


ROOT=Path(__file__).parents[1]


@pytest.fixture
def settings():
    return load_settings(ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json')


def context(*,ratio=False,turn_aware=True):
    ys=np.linspace(180,260 if ratio else 220,12)
    width=80+(ys-180)*270/80 if ratio else 170-.04*(ys-180)**2
    return {'valid':True,'ground_supported':True,'boundary_candidates':[],
        **({'turn_aware':True} if turn_aware else {}),
        'left_inner':np.column_stack((240-width/2,ys)).tolist(),
        'right_inner':np.column_stack((240+width/2,ys)).tolist()}


def track(t=1.,*,y=260.,identity='secondary-1',confirmed=True,**changes):
    return {'id':identity,'visible':True,'confirmed':confirmed,'seen_s':t,
        'kind':'cone_candidate','ground_contact_usable':True,'clipped':False,
        'bbox_xyxy':[230,int(y)-15,250,int(y)+5],'foot_pixel':[240.,y],**changes}


def observation(road,tracks=(),t=1.,fid=1):
    return {'road_context':road,'tracks':list(tracks),'captured_s':t,'frame_id':fid}


def test_curve_bounds_never_evaluates_outside_actual_joint_observations():
    model=measured_corridor(context(),(360,480,3))
    assert model.bounds(180)[0]<model.bounds(180)[1]
    assert model.bounds(220)[0]<model.bounds(220)[1]
    # The old unbounded polynomial really does cross outside the observations.
    assert model.left.at(260)>model.right.at(260)
    for y in (179.99,220.01,260.,float('nan')):
        with pytest.raises(ValueError,match='实际观测范围'):model.bounds(y)


@pytest.mark.parametrize('y',[160.,260.])
def test_outside_target_does_not_turn_a_valid_lane_into_missing(settings,monkeypatch,y):
    nav=LowSpeedNavigator(settings)
    def no_projection(_):raise AssertionError('Outside observations must not be projected')
    monkeypatch.setattr(nav.projection,'point',no_projection)
    item=track(y=y);road=context();image=np.zeros((360,480,3),np.uint8)
    result=nav.update(image,observation(road,[item]),1.)
    assert result['line_state']=='paired' and result['boundary_observed']
    assert not result.get('line_missing') and not result.get('boundary_failure_reason')
    assert result['cones']==[] and result['visual_cones'][0]['current_pixel']
    assert result['visual_cones'][0]['id']==item['id']
    assert result['obstacle_diagnostics']==[{'id':item['id'],'reason':'out_of_observed_band',
        'foot_y_px':y,'observed_y_range':[180.,220.]}]
    assert result['boundary_diagnostics']['model_observation_only']


@pytest.mark.parametrize('confirmed,kind',[(False,'cone_candidate'),(False,'unresolved_blue'),(True,'unresolved_blue')])
def test_background_or_unconfirmed_outside_track_cannot_invalidate_white_edges(settings,confirmed,kind):
    main=track(y=200.);outside=track(y=260.,identity='secondary-2',confirmed=confirmed,kind=kind)
    result=LowSpeedNavigator(settings).update(np.zeros((360,480,3),np.uint8),
        observation(context(),[main,outside]),1.)
    assert result['line_state']=='paired' and not result.get('line_missing')
    assert [item['id'] for item in result['cones']]==['secondary-1']
    assert result['obstacle_diagnostics'][-1]['reason']=='out_of_observed_band'


def test_obstacle_diagnostics_stay_bounded_without_changing_lane_acceptance(settings):
    items=[track(identity='secondary-'+str(i),confirmed=False) for i in range(40)]
    result=LowSpeedNavigator(settings).update(np.zeros((360,480,3),np.uint8),
        observation(context(),items),1.)
    assert result['line_state']=='paired' and result['boundary_observed']
    assert len(result['obstacle_diagnostics'])==16
    assert result['obstacle_diagnostics_omitted']==24
    assert result['cones']==result['visual_cones']==[]


def test_valid_observed_width_can_change_by_more_than_threefold(settings):
    road=context(ratio=True);model=measured_corridor(road,(360,480,3))
    widths=[model.bounds(y)[1]-model.bounds(y)[0] for y in (180,260)]
    assert max(widths)/min(widths)>4
    before=copy.deepcopy(settings)
    result=LowSpeedNavigator(settings).update(np.zeros((360,480,3),np.uint8),observation(road),1.)
    assert result['line_state']=='paired' and result['boundary_observed']
    assert not result['metric_boundary_verified'] and settings==before


def test_real_pixel_pair_with_large_perspective_width_has_supported_corridor(settings):
    settings['scene'].update(surface='indoor_white_line_track',white_v_min=255)
    image=np.full((360,480,3),60,np.uint8);ys=np.arange(180,261)
    width=80+(ys-180)*270/80
    for sign in (-1,1):
        cv2.polylines(image,[np.column_stack((240+sign*width/2,ys)).astype(np.int32)],False,(255,255,255),6)
    road=ConeRoadContext(settings,turn_aware=True).inspect(image,np.zeros((360,480),np.uint8))
    assert road['valid']
    result=LowSpeedNavigator(settings).update(image,observation(road),1.)
    assert result['line_state']=='paired' and not result.get('line_missing')


def test_crossing_within_observed_rows_is_still_rejected(settings):
    road=context();road['right_inner']=[[x-220.,y] for x,y in road['right_inner']]
    with pytest.raises(ValueError,match='白线宽度异常'):measured_corridor(road,(360,480,3))
    result=LowSpeedNavigator(settings).update(np.zeros((360,480,3),np.uint8),observation(road),1.)
    assert result['line_state']=='missing' and not result.get('lane_bounds_px')


def test_supported_near_field_history_is_not_relabelled_as_current_lane_position(settings):
    nav=LowSpeedNavigator(settings);image=np.zeros((360,480,3),np.uint8)
    road=context(ratio=True)
    for t,y in ((1.,228.),(1.1,255.)):
        result=nav.update(image,observation(road,[track(t,y=y)],t,round(t*100)),t)
        assert result['cones'] and not result['cones'][0].get('near_field_continuation')
    item=track(1.2,y=285.,clipped=True,ground_contact_usable=False)
    result=nav.update(image,observation(road,[item],1.2,120),1.2)
    assert result['line_state']=='paired'
    old=result['cones'][0]
    assert old['near_field_continuation'] and old['position_source']=='last_valid_ground_observation'
    assert result['visual_cones'][0]['y_fraction']==pytest.approx(285/360)
    assert result['obstacle_diagnostics'][0]['reason']=='historical_lane_continuation'


def test_non_turn_aware_stage_retains_its_legacy_straight_extension(settings):
    result=LowSpeedNavigator(settings).update(np.zeros((360,480,3),np.uint8),
        observation(context(turn_aware=False),[track()]),1.)
    assert result['cones']  # Its original straight extension is intentionally unchanged.
    assert 'obstacle_diagnostics' not in result and 'visual_cones' not in result
    assert 'obstacle_diagnostics_omitted' not in result
