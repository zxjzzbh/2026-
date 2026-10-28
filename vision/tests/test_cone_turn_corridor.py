"""Offline image guidance: actual curves, one observed side, and bounded loss."""
import copy
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.cone_corridor import measured_corridor
from carvision.cone_low_speed import LowSpeedNavigator,DualLowSpeedNavigator
from carvision.cone_planning import load_settings
from carvision.cone_road import ConeRoadContext


ROOT=Path(__file__).parents[1]


@pytest.fixture
def settings():
    value=load_settings(ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json')
    value['scene'].update(surface='indoor_white_line_track',white_v_min=255)
    return value


def track(*,curve=0.,shift=0.,sides=('left','right'),value=255,center=240.,width=180.,slope=.65):
    image=np.full((360,480,3),60,np.uint8)
    ys=np.arange(150,324);t=(ys-226)/46
    middle=center+shift+curve*t*t
    wide=width+slope*(ys-226)
    for side in sides:
        sign=-1 if side=='left' else 1
        points=np.column_stack((middle+sign*wide/2,ys)).astype(np.int32)
        cv2.polylines(image,[points],False,(value,value,value),6)
    return image


def observe(settings,image,t=1.,fid=1,camera='secondary',turn_aware=True):
    context=ConeRoadContext(settings,camera,turn_aware=turn_aware).inspect(
        image,np.zeros(image.shape[:2],np.uint8))
    return {'captured_s':t,'frame_id':fid,'road_context':context,'tracks':[]}


@pytest.mark.parametrize('curve',[0.,10.,20.])
def test_current_curved_pair_survives_both_detector_and_navigation(settings,curve):
    image=track(curve=curve)
    row=observe(settings,image)
    assert row['road_context']['valid']
    nav=LowSpeedNavigator(settings).update(image,row,1.)
    assert nav['line_state']=='paired' and nav['boundary_observed']
    assert not nav.get('line_missing') and not nav['line_held']
    assert nav['prediction_age_s']==0 and nav['boundary_uncertainty_fraction']==0
    assert nav['heading_error']==pytest.approx(nav['ego_near_fraction']-nav['ego_far_fraction'])
    assert not nav['metric_boundary_verified']


def test_old_non_cone_context_keeps_straight_only_acceptance(settings):
    image=track(curve=20.)
    row=observe(settings,image,turn_aware=False)
    assert row['road_context']['valid']
    nav=LowSpeedNavigator(settings).update(image,row,1.)
    assert nav['line_missing']
    assert '仅支持直道' in nav['boundary_failure_reason']


@pytest.mark.parametrize('center',[110.,370.])
def test_pair_does_not_need_to_straddle_image_center(settings,center):
    image=track(center=center,width=100.,slope=0.)
    context=observe(settings,image)['road_context']
    assert context['valid']
    left=np.asarray(context['left_inner'])[:,0];right=np.asarray(context['right_inner'])[:,0]
    assert (np.all(right<240) if center<240 else np.all(left>240))
    model=measured_corridor(context,image.shape)
    assert model.bounds(240)[1]-model.bounds(240)[0]>90


@pytest.mark.parametrize('side',['left','right'])
def test_single_uses_current_side_and_never_refreshes_pair_age(settings,side):
    nav=LowSpeedNavigator(settings);image=track()
    paired=nav.update(image,observe(settings,image),1.)
    assert paired['line_state']=='paired'
    original=copy.deepcopy(nav.paired_corridor[0])
    first=None
    for t,shift in ((1.10,8.),(1.20,12.),(1.25,15.)):
        image=track(sides=(side,),shift=shift)
        data=nav.update(image,observe(settings,image,t,round(t*100)),t)
        assert data['line_state']=='single' and data['line_held']
        assert not data['boundary_observed'] and not data.get('line_missing')
        assert data['prediction_age_s']==pytest.approx(t-1.)
        assert data['line_observed_s']==1. and nav.paired_corridor[0]==original
        assert data['boundary_uncertainty_fraction']>0
        assert data['half_envelope']==pytest.approx(data['base_half_envelope']+data['boundary_uncertainty_fraction'])
        if first is not None:assert data['lane_bounds_px']!=first
        first=data['lane_bounds_px']
    image=track(sides=(side,),shift=15.)
    lost=nav.update(image,observe(settings,image,1.251,126),1.251)
    assert lost['line_state']=='missing' and lost['line_missing']
    assert not lost.get('lane_bounds_px') and not lost['boundary_observed']


def test_no_current_edge_never_reuses_old_pixel_corridor(settings):
    nav=DualLowSpeedNavigator(settings);image=track()
    data=observe(settings,image)
    assert nav.update_views({'secondary':image},{'secondary':data},1.)['line_state']=='paired'
    image=np.full_like(image,60)
    data=observe(settings,image,1.05,2)
    lost=nav.update_views({'secondary':image},{'secondary':data},1.05)
    assert lost['line_state']=='missing' and lost['line_missing']
    assert not lost.get('lane_bounds_px') and not lost['line_sources']
    assert lost['boundary_failure_reason'] and not nav.memory


def test_first_single_has_no_pair_reference(settings):
    image=track(sides=('left',))
    result=LowSpeedNavigator(settings).update(image,observe(settings,image),1.)
    assert result['line_state']=='missing' and result['line_missing']
    assert not result.get('lane_bounds_px')


def test_camera_width_memory_is_not_shared(settings):
    nav=DualLowSpeedNavigator(settings);image=track()
    nav.update_views({'secondary':image},{'secondary':observe(settings,image)},1.)
    image=track(sides=('left',))
    data=observe(settings,image,1.1,2,camera='primary')
    result=nav.update_views({'primary':image},{'primary':data},1.1)
    assert result['source_camera']=='primary'
    assert result['line_state']=='missing' and not result.get('lane_bounds_px')


def test_restarted_frame_ids_clear_width_reference(settings):
    nav=LowSpeedNavigator(settings);image=track()
    nav.update(image,observe(settings,image,1.,10),1.)
    image=track(sides=('left',))
    result=nav.update(image,observe(settings,image,1.1,1),1.1)
    assert result['line_state']=='missing' and nav.paired_corridor is None


def test_turn_mode_change_rechecks_same_raw_frame_with_new_model(settings):
    nav=LowSpeedNavigator(settings);image=track(curve=20.)
    old=nav.update(image,observe(settings,image,turn_aware=False),1.)
    assert old['line_missing']
    changed=nav.update(image,observe(settings,image,turn_aware=True),1.02)
    assert changed['line_state']=='paired' and not changed.get('line_missing')
    assert changed['captured_s']==1. and changed['frame_id']==1
    back=nav.update(image,observe(settings,image,turn_aware=False),1.03)
    assert back['line_missing'] and nav.paired_corridor is None


def test_turn_mode_change_cannot_reuse_legacy_pair_as_single_anchor(settings):
    nav=LowSpeedNavigator(settings);image=track()
    assert nav.update(image,observe(settings,image,turn_aware=False),1.)['line_state']=='paired'
    image=track(sides=('left',))
    changed=nav.update(image,observe(settings,image,turn_aware=True),1.02)
    assert changed['line_state']=='missing' and not changed.get('lane_bounds_px')
    assert nav.paired_corridor is None


def test_capture_delay_counts_towards_single_prediction_age(settings):
    nav=LowSpeedNavigator(settings);image=track()
    nav.update(image,observe(settings,image),1.)
    image=track(sides=('left',))
    # This camera frame is itself fresh, but the true pair is already too old.
    result=nav.update(image,observe(settings,image,1.20,2),1.30)
    assert result['line_state']=='missing' and not result.get('lane_bounds_px')


def test_cached_single_frame_cannot_extend_prediction_budget(settings):
    nav=LowSpeedNavigator(settings);image=track()
    nav.update(image,observe(settings,image),1.)
    image=track(sides=('left',));row=observe(settings,image,1.10,2)
    assert nav.update(image,row,1.10)['line_state']=='single'
    repeated=nav.update(image,row,1.24)
    assert repeated['prediction_age_s']==pytest.approx(.24)
    assert repeated['half_envelope']==pytest.approx(repeated['base_half_envelope']+repeated['boundary_uncertainty_fraction'])
    assert nav.paired_corridor[0]==1.
    expired=nav.update(image,row,1.251)
    assert expired['line_state']=='missing' and not expired.get('lane_bounds_px')


def test_single_identity_is_not_selected_from_two_matching_stripes(settings):
    nav=LowSpeedNavigator(settings);image=track()
    nav.update(image,observe(settings,image),1.)
    row=observe(settings,image,1.1,2)
    row['road_context']['valid']=False
    # A contradictory rejected pair cannot be relabelled as one trusted side.
    result=nav.update(image,row,1.1)
    assert result['line_state']=='missing' and not result.get('lane_bounds_px')


def test_single_uncertainty_that_leaves_no_clearance_requests_stop(settings):
    settings['geometry']['lane_width_m']=.60
    nav=LowSpeedNavigator(settings);image=track()
    assert nav.update(image,observe(settings,image),1.)['line_state']=='paired'
    image=track(sides=('left',))
    result=nav.update(image,observe(settings,image,1.2,2),1.2)
    assert result['action']=='stop' and not result['ready'] and result['boundary_danger']
    assert not result.get('lane_bounds_px')


def test_navigation_keeps_independent_raw_boundary_diagnostics(settings):
    image=track(value=220);row=observe(settings,image)
    result=LowSpeedNavigator(settings).update(image,row,1.)
    diagnostics=result['boundary_diagnostics']
    assert diagnostics['effective_white_v_min']==220
    assert diagnostics['support_rows']>=8 and diagnostics['common_span_px']>36
    assert diagnostics['road_reason']==row['road_context']['reason']
    row['road_context']['boundary_diagnostics']['support_rows']=0
    assert diagnostics['support_rows']>=8


@pytest.mark.parametrize('case',['empty','white_floor','blue_board','short_posts','crossbar','wide_glare'])
def test_turn_detector_does_not_invent_a_pair(settings,case):
    image=np.full((360,480,3),60,np.uint8)
    if case=='white_floor':image[:]=230
    elif case=='blue_board':image[:]=(220,35,20)
    elif case=='short_posts':
        for x in (100,380):cv2.line(image,(x,190),(x,220),(230,230,230),5)
    elif case=='crossbar':cv2.line(image,(15,220),(465,220),(230,230,230),10)
    elif case=='wide_glare':image[:,:170]=230;image[:,310:]=230
    context=observe(settings,image)['road_context']
    assert not context['valid'] and not context['white_lines_detected']
    assert context['left_inner']==context['right_inner']==[]


@pytest.mark.parametrize('name',['cone-track-glare-secondary.jpg','cone-track-short-secondary.jpg'])
def test_real_paint_replay_stays_bounded_and_reports_observations(settings,name):
    image=cv2.imdecode(np.frombuffer((ROOT/'tests/fixtures'/name).read_bytes(),np.uint8),1)
    context=observe(settings,image)['road_context']
    assert context['valid'],context['boundary_diagnostics']
    assert context['boundary_diagnostics']['detector']=='turn_aware_curves'
    assert context['boundary_diagnostics']['candidate_count']<=12
    assert context['boundary_diagnostics']['max_threshold_attempts']==5
    assert not context['metric_boundary_verified']


def test_bad_geometry_or_unknown_ground_cannot_recover_single(settings):
    nav=LowSpeedNavigator(settings);image=track()
    nav.update(image,observe(settings,image),1.)
    image=track(sides=('left',),shift=100.)
    assert nav.update(image,observe(settings,image,1.1,2),1.1)['line_state']=='missing'
    image=track(sides=('left',))
    row=observe(settings,image,1.2,3);row['road_context']['ground_supported']=False
    assert nav.update(image,row,1.2)['line_state']=='missing'
