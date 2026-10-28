"""Current cone pixels can continue a lock without inventing metric positions."""
import copy
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.cone_low_speed import LowSpeedNavigator,DualLowSpeedNavigator
from carvision.cone_planning import load_settings


ROOT=Path(__file__).parents[1]


@pytest.fixture
def settings():
    return load_settings(ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json')


def observation(t=1.,fid=1,*,camera='secondary',turn_aware=True,tracks=None):
    return {'captured_s':t,'frame_id':fid,'tracks':tracks or [],
        'road_context':{'valid':True,'ground_supported':True,
            **({'turn_aware':True} if turn_aware else {}),
            'boundary_candidates':[],
            'left_inner':[[100.,float(y)] for y in np.linspace(180,260,12)],
            'right_inner':[[380.,float(y)] for y in np.linspace(180,260,12)]}}


def cone(t=1.,*,camera='secondary',**changes):
    return {'id':camera+'-1','visible':True,'confirmed':True,'seen_s':t,
        'bbox_xyxy':[270,220,300,250],'foot_pixel':[285.,245.],
        'kind':'cone_candidate','ground_contact_usable':True,'clipped':False,**changes}


def picture(track):
    image=np.zeros((360,480,3),np.uint8)
    x1,y1,x2,y2=track['bbox_xyxy']
    cv2.rectangle(image,(x1,y1),(x2-1,y2-1),(255,0,0),-1)
    return image


def test_confirmed_current_pixels_survive_ground_roi_exit_and_clipping(settings):
    track=cone(bbox_xyxy=[290,310,340,360],foot_pixel=[315.,355.],
        clipped=True,ground_contact_usable=False,kind='unresolved_blue')
    nav=LowSpeedNavigator(settings);before=copy.deepcopy(settings)
    result=nav.update(picture(track),observation(tracks=[track]),1.)
    assert not result['cones']  # This must not become a fresh metric/lane target.
    assert len(result['visual_cones'])==1
    current=result['visual_cones'][0]
    assert current['id']=='secondary-1' and current['current_pixel'] and current['confirmed']
    assert current['clipped'] and current['image_fraction']==pytest.approx(315/480)
    assert current['y_fraction']==pytest.approx(355/360)
    assert current['half_image_width']==pytest.approx(50/960)
    assert 'fraction' not in current and 'reference_distance_m' not in current
    assert not result['metric_boundary_verified'] and settings==before


def test_round_shape_with_no_ground_contact_remains_pixel_only(settings):
    track=cone(ground_contact_usable=False,shape_evidence='round_top_view')
    result=LowSpeedNavigator(settings).update(picture(track),observation(tracks=[track]),1.)
    assert not result['cones'] and result['uncertain_cones'][0]['provisional']
    assert result['visual_cones'][0]['id']==track['id']


@pytest.mark.parametrize('changes',[
    {'visible':False}, {'confirmed':False}, {'seen_s':.9}, {'id':'primary-1'},
    {'foot_pixel':[float('nan'),230.]}, {'foot_pixel':[500.,230.]},
    {'bbox_xyxy':[10,20,5,30]}, {'foot_pixel':['invalid',230.]},
])
def test_unconfirmed_stale_other_camera_or_invalid_pixels_are_not_visual_targets(settings,changes):
    track=cone(**changes);image=np.zeros((360,480,3),np.uint8)
    # Exercise the pixel-list filter directly: the ground route has its own
    # independent acceptance and is not responsible for malformed fake tracks.
    assert LowSpeedNavigator(settings)._visual_cones(observation(tracks=[track]),480,360)==[]


def test_visual_target_uses_new_pixels_not_last_ground_fraction(settings):
    nav=LowSpeedNavigator(settings);track=cone()
    first=nav.update(picture(track),observation(tracks=[track]),1.)
    assert first['cones'] and nav.near_history
    track=cone(1.1,bbox_xyxy=[340,305,390,350],foot_pixel=[365.,345.],
        clipped=True,ground_contact_usable=False,kind='unresolved_blue')
    second=nav.update(picture(track),observation(1.1,2,tracks=[track]),1.1)
    visual=second['visual_cones'][0]
    assert visual['image_fraction']==pytest.approx(365/480)
    assert visual['bbox_xyxy']==[340,305,390,350]
    assert 'fraction' not in visual and 'near_field_continuation' not in visual


def test_reference_is_same_camera_historical_x0_at_current_observed_y(settings):
    nav=LowSpeedNavigator(settings);track=cone()
    result=nav.update(picture(track),observation(tracks=[track]),1.)
    current=result['visual_cones'][0];H=nav.projection.h
    expected=-(H[0,1]*track['foot_pixel'][1]+H[0,2])/H[0,0]/480
    assert current['image_reference_fraction']==pytest.approx(expected)
    assert current['image_reference_source']=='historical_X0_image_reference'
    assert not result['field_verified'] and not result['metric_boundary_verified']


def test_visual_provenance_matches_the_actual_camera_frame(settings):
    track=cone(1.03);row=observation(1.03,42,tracks=[track])
    result=LowSpeedNavigator(settings).update(picture(track),row,1.04)
    current=result['visual_cones'][0]
    assert current['source_camera']=='secondary'
    assert current['frame_id']==result['frame_id']==42
    assert current['captured_s']==result['captured_s']==1.03
    row.update(frame_id=43,captured_s=1.1)
    assert current['frame_id']==42 and current['captured_s']==1.03


def test_invalid_nominal_reference_is_none_instead_of_assumed_image_center(settings):
    nav=LowSpeedNavigator(settings);nav.projection.h[0,0]=0.
    track=cone();result=nav.update(picture(track),observation(tracks=[track]),1.)
    assert result['visual_cones'][0]['image_reference_fraction'] is None


def test_both_camera_views_use_their_own_reference_and_track_namespace(settings):
    nav=DualLowSpeedNavigator(settings);images={};rows={}
    for camera in ('primary','secondary'):
        track=cone(camera=camera);images[camera]=picture(track)
        rows[camera]=observation(camera=camera,tracks=[track])
    result=nav.update_views(images,rows,1.)
    references=[]
    for camera in ('primary','secondary'):
        current=nav.results[camera]['visual_cones'][0];H=nav.views[camera].projection.h
        assert current['id']==camera+'-1'
        assert current['source_camera']==camera
        assert current['frame_id']==rows[camera]['frame_id']
        assert current['captured_s']==rows[camera]['captured_s']
        expected=-(H[0,1]*245+H[0,2])/H[0,0]/480
        assert current['image_reference_fraction']==pytest.approx(expected)
        references.append(current['image_reference_fraction'])
    assert references[0]!=references[1]
    assert result['visual_cones'][0]['id'].startswith(result['source_camera']+'-')


def test_missing_lines_keeps_only_current_confirmed_pixels(settings):
    track=cone();row=observation(tracks=[track]);row['road_context']['valid']=False
    result=LowSpeedNavigator(settings).update(picture(track),row,1.)
    assert result['line_state']=='missing' and result['line_missing']
    assert result['visual_cones'][0]['current_pixel']
    assert not result.get('lane_bounds_px')


def test_stale_camera_has_no_current_pixel_targets(settings):
    track=cone();result=LowSpeedNavigator(settings).update(picture(track),observation(tracks=[track]),1.3)
    assert result['visual_cones']==[] and result['action']=='stop'


def test_same_frame_cannot_inject_changed_pixel_positions(settings):
    nav=LowSpeedNavigator(settings);track=cone()
    first=nav.update(picture(track),observation(tracks=[track]),1.)
    track=cone(bbox_xyxy=[350,220,380,250],foot_pixel=[365.,245.])
    duplicate=nav.update(picture(track),observation(tracks=[track]),1.1)
    assert duplicate['visual_cones']==first['visual_cones']
    assert duplicate['frame_id']==1 and duplicate['captured_s']==1.


def test_non_turn_aware_stage_keeps_existing_output_contract(settings):
    track=cone();result=LowSpeedNavigator(settings).update(
        picture(track),observation(tracks=[track],turn_aware=False),1.)
    assert 'visual_cones' not in result
    assert result['cones']
