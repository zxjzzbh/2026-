"""Current paint stays authoritative; temporal metadata only ranks candidates."""
import copy
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision import cone_road
from carvision.cone_corridor import measured_corridor,recover_single
from carvision.cone_planning import load_settings
from carvision.cone_low_speed import LowSpeedNavigator
from carvision.cone_road import ConeRoadContext
from carvision.cone_vision import BlueConeDetector
from test_cone_turn_corridor import track


ROOT=Path(__file__).parents[1]


@pytest.fixture
def settings():
    value=load_settings(ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json')
    value['scene'].update(surface='indoor_white_line_track',white_v_min=255)
    return value


def inspect(context,image=None,fid=1,t=1.):
    image=track() if image is None else image
    return context.inspect(image,np.zeros(image.shape[:2],np.uint8),frame_id=fid,captured_s=t)


def spy_prior(monkeypatch):
    original=cone_road.curve_inner_edges;calls=[]
    def detect(image,mask,config,*,prior=None):
        calls.append(copy.deepcopy(prior))
        return original(image,mask,config,prior=prior)
    monkeypatch.setattr(cone_road,'curve_inner_edges',detect)
    return calls


def test_only_real_fresh_pair_is_available_as_next_frame_prior(settings,monkeypatch):
    before=copy.deepcopy(settings);calls=spy_prior(monkeypatch)
    context=ConeRoadContext(settings,turn_aware=True)
    first=inspect(context)
    assert first['valid'] and calls==[None]
    assert first['boundary_diagnostics']['history_pair_saved']
    original=copy.deepcopy(first['left_inner'])
    first['left_inner'][0][0]=-999
    second=inspect(context,fid=2,t=1.1)
    assert second['valid'] and calls[-1]['left_inner']==original
    assert calls[-1]['frame_id']==1 and calls[-1]['captured_s']==1.
    assert calls[-1]['shape']==(360,480) and calls[-1]['camera']=='secondary'
    assert second['boundary_diagnostics']['history_prior_available']
    assert not second['metric_boundary_verified'] and settings==before


@pytest.mark.parametrize('change',[
    {'fid':1,'t':1.1},{'fid':2,'t':1.},{'fid':0,'t':.9},
    {'fid':2,'t':1.251},{'fid':True,'t':1.1},
    {'fid':2,'t':float('nan')},{'fid':None,'t':1.1},
    {'fid':2,'t':None},
])
def test_repeated_reversed_missing_or_discontinuous_metadata_never_accumulates_prior(settings,monkeypatch,change):
    calls=spy_prior(monkeypatch);context=ConeRoadContext(settings,turn_aware=True)
    assert inspect(context)['valid']
    result=inspect(context,**change)
    assert result['valid']  # Current pixels may still be valid; history is not.
    assert calls[-1] is None and not result['boundary_diagnostics']['history_pair_saved']
    assert not result['boundary_diagnostics']['history_prior_available']
    assert context._pair_prior is None


@pytest.mark.parametrize('frames',[
    ((100,1.),(1,1.1),(2,1.2),(3,1.3)),
    ((100,1.),(101,.9),(102,1.),(103,1.1)),
    ((100,1.),(100,1.),(101,1.1),(102,1.2)),
])
def test_real_pair_history_rebuilds_after_restart_or_duplicate_without_using_reset_frame(settings,monkeypatch,frames):
    calls=spy_prior(monkeypatch);context=ConeRoadContext(settings,turn_aware=True)
    first,reset,learn,use=frames
    assert inspect(context,fid=first[0],t=first[1])['valid']
    reset_result=inspect(context,fid=reset[0],t=reset[1])
    assert reset_result['valid'] and calls[-1] is None
    assert not reset_result['boundary_diagnostics']['history_pair_saved']
    assert context._pair_prior is None
    learn_result=inspect(context,fid=learn[0],t=learn[1])
    assert learn_result['valid'] and calls[-1] is None
    assert learn_result['boundary_diagnostics']['history_pair_saved']
    use_result=inspect(context,fid=use[0],t=use[1])
    assert use_result['valid'] and calls[-1]['frame_id']==learn[0]
    assert calls[-1]['captured_s']==learn[1]


def test_shape_change_cannot_share_prior_or_turn_wrong_resolution_into_history(settings,monkeypatch):
    calls=spy_prior(monkeypatch);context=ConeRoadContext(settings,turn_aware=True)
    assert inspect(context)['valid']
    image=cv2.resize(track(),(400,300))
    result=inspect(context,image,fid=2,t=1.1)
    assert calls[-1] is None and context._pair_prior is None
    assert not result['boundary_diagnostics']['history_pair_saved']
    inspect(context,fid=3,t=1.2)
    assert calls[-1] is None


def test_metadata_free_legacy_calls_detect_current_pixels_without_learning_history(settings,monkeypatch):
    calls=spy_prior(monkeypatch);context=ConeRoadContext(settings,turn_aware=True)
    image=track();mask=np.zeros((360,480),np.uint8)
    for _ in range(3):
        result=context.inspect(image,mask)
        assert result['valid'] and not result['boundary_diagnostics']['history_pair_saved']
    assert all(prior is None for prior in calls) and context._pair_prior is None


def test_blank_current_frame_never_becomes_cached_pair_or_renews_pair_age(settings,monkeypatch):
    calls=spy_prior(monkeypatch);context=ConeRoadContext(settings,turn_aware=True)
    assert inspect(context)['valid']
    blank=np.full((360,480,3),60,np.uint8)
    for fid,t in ((2,1.1),(3,1.2)):
        result=inspect(context,blank,fid,t)
        assert not result['valid'] and result['left_inner']==result['right_inner']==[]
        assert calls[-1]['captured_s']==1. and not result['boundary_diagnostics']['history_pair_saved']
    result=inspect(context,blank,4,1.251)
    assert calls[-1] is None and context._pair_prior is None
    assert not result['valid']


def test_current_single_can_match_recent_pair_but_never_claims_new_pair(settings):
    context=ConeRoadContext(settings,turn_aware=True)
    pair=inspect(context);model=measured_corridor(pair,(360,480,3))
    image=track(sides=('left',),shift=8.)
    single=inspect(context,image,2,1.1)
    assert not single['valid'] and single['left_inner']==single['right_inner']==[]
    assert single['boundary_candidates'] and not single['boundary_diagnostics']['history_pair_saved']
    recovered=recover_single(single,image.shape,1.1,(1.,model),now_s=1.1)
    assert recovered.state=='single' and recovered.prediction_age_s==pytest.approx(.1)
    assert context._pair_prior['captured_s']==1.
    with pytest.raises(ValueError,match='0.25'):
        recover_single(single,image.shape,1.251,(1.,model),now_s=1.251)
    with pytest.raises(ValueError):recover_single(single,image.shape,1.1,None,now_s=1.1)


def test_prior_cannot_bypass_current_blue_occlusion(settings):
    context=ConeRoadContext(settings,turn_aware=True);image=track()
    assert inspect(context,image)['valid']
    result=context.inspect(image,np.full((360,480),255,np.uint8),frame_id=2,captured_s=1.1)
    assert not result['valid'] and result['left_inner']==result['right_inner']==[]
    assert not result['boundary_candidates']


def test_camera_contexts_keep_separate_ground_rois_and_history(settings,monkeypatch):
    calls=spy_prior(monkeypatch)
    secondary=ConeRoadContext(settings,'secondary',turn_aware=True)
    primary=ConeRoadContext(settings,'primary',turn_aware=True)
    assert secondary.lane.config.roi_top==.53 and primary.lane.config.roi_top==.38
    for context in (secondary,primary):
        config=context.lane.config
        assert config.roi_top<=config.sample_top<config.lookahead_y<config.sample_bottom
    assert inspect(secondary)['valid']
    assert inspect(primary,fid=100,t=1.1)['valid'] and calls[-1] is None
    assert inspect(primary,fid=101,t=1.2)['valid'] and calls[-1]['camera']=='primary'
    assert inspect(secondary,fid=2,t=1.1)['valid'] and calls[-1]['camera']=='secondary'


@pytest.mark.parametrize('camera,roi',[('primary',.60),('secondary',.38)])
def test_non_turn_indoor_legacy_keeps_its_original_roi_and_no_history(settings,camera,roi):
    context=ConeRoadContext(settings,camera,turn_aware=False,thorough_boundary_search=True)
    assert context.lane.config.roi_top==roi
    image=track();result=inspect(context,image)
    assert context._pair_prior is None and 'turn_aware' not in result
    assert 'history_prior_available' not in result['boundary_diagnostics']


def test_turn_aware_has_one_detector_and_never_uses_sampled_adaptive_fallback(settings,monkeypatch):
    def forbidden(*args,**kwargs):raise AssertionError('turn detector used legacy adaptive fallback')
    monkeypatch.setattr(cone_road,'adaptive_inner_edges',forbidden)
    context=ConeRoadContext(settings,turn_aware=True)
    assert inspect(context)['valid']
    assert not inspect(context,np.full((360,480,3),60,np.uint8),2,1.1)['valid']


def stripe(x,*,low=195.,high=310.):
    ys=np.linspace(low,high,20)
    return {'left_edge':[[x,float(y)] for y in ys],
            'right_edge':[[x+5,float(y)] for y in ys]}


def test_threshold_candidates_are_merged_and_deduplicated_not_first_background_only(settings,monkeypatch):
    current={255:[stripe(10)],220:[stripe(10),stripe(100)],
             200:[stripe(10),stripe(101),stripe(400)],180:[stripe(10),stripe(101),stripe(401)],
             160:[stripe(10),stripe(101),stripe(401)]}
    def detect(image,mask,config,*,prior=None):
        candidates=copy.deepcopy(current[config.white_v_min])
        return [],[],np.zeros(image.shape[:2],np.uint8),{'reason':'no_supported_curve_pair'},candidates
    monkeypatch.setattr(cone_road,'curve_inner_edges',detect)
    result=inspect(ConeRoadContext(settings,turn_aware=True),np.full((360,480,3),60,np.uint8))
    assert not result['valid'] and result['left_inner']==result['right_inner']==[]
    xs=sorted(row['left_edge'][0][0] for row in result['boundary_candidates'])
    assert len(xs)==3 and xs[0]==10 and 100<=xs[1]<=101 and 400<=xs[2]<=401
    assert result['boundary_diagnostics']['candidate_thresholds']==[255,220,200,180,160]
    assert result['boundary_diagnostics']['merged_candidate_count']==3


@pytest.mark.parametrize('camera',['primary','secondary'])
@pytest.mark.parametrize('case',['empty','grey_floor','glare','windows_above_roi','horizontal','blue_object'])
def test_background_without_supported_current_pair_stays_invalid_even_with_prior(settings,case,camera):
    context=ConeRoadContext(settings,camera,turn_aware=True);assert inspect(context)['valid']
    image=np.full((360,480,3),60,np.uint8);mask=np.zeros((360,480),np.uint8)
    if case=='grey_floor':image[:]=170
    elif case=='glare':image[:,:160]=230;image[:,320:]=230
    elif case=='windows_above_roi':
        for x in (90,170,310,390):cv2.line(image,(x,10),(x,175),(255,255,255),6)
    elif case=='horizontal':cv2.line(image,(10,240),(470,240),(255,255,255),8)
    elif case=='blue_object':
        cv2.rectangle(image,(90,190),(390,340),(255,30,10),-1)
        mask[190:341,90:391]=255
    result=context.inspect(image,mask,frame_id=2,captured_s=1.1)
    assert not result['valid'] and result['left_inner']==result['right_inner']==[]
    assert not result['boundary_diagnostics']['history_pair_saved']


def test_merged_single_candidates_stay_bounded(settings,monkeypatch):
    def detect(image,mask,config,*,prior=None):
        offset={255:0,220:1,200:2,180:3,160:4}[config.white_v_min]
        candidates=[stripe(8.+index*9.+offset*80.) for index in range(12)]
        return [],[],np.zeros(image.shape[:2],np.uint8),{'reason':'no_supported_curve_pair'},candidates
    monkeypatch.setattr(cone_road,'curve_inner_edges',detect)
    result=inspect(ConeRoadContext(settings,turn_aware=True))
    assert not result['valid'] and len(result['boundary_candidates'])==12
    assert result['boundary_diagnostics']['merged_candidate_count']==12


@pytest.mark.parametrize('camera,index',[('secondary',0),('secondary',1),('secondary',2)])
def test_dark_field_tape_uses_bounded_current_candidates_without_saving_threshold(settings,camera,index):
    original=copy.deepcopy(settings)
    path=ROOT/'tests/fixtures'/f'white-live-dark-20261026-{camera}-{index}.jpg'
    image=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    _,mask=BlueConeDetector(settings).detect(image)
    context=ConeRoadContext(settings,camera,turn_aware=True)
    result=context.inspect(image,mask,frame_id=1,captured_s=1.)
    assert result['valid'] and result['white_lines_detected']
    measured_corridor(result,image.shape)
    diag=result['boundary_diagnostics']
    assert diag['configured_white_v_min']==255 and diag['max_threshold_attempts']==5
    assert 160<=diag['effective_white_v_min']<=255
    assert settings==original and context.lane.config.white_v_min==255
    assert not result['metric_boundary_verified']


@pytest.mark.parametrize('index',[0,1,2])
def test_insufficient_dark_primary_views_remain_unknown_instead_of_inventing_pair(settings,index):
    # In view 1, the old left candidate was a low-contrast floor highlight
    # below the distant cone. Actual tape lacks a sufficient common band.
    path=ROOT/'tests/fixtures'/f'white-live-dark-20261026-primary-{index}.jpg'
    image=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    _,mask=BlueConeDetector(settings).detect(image)
    context=ConeRoadContext(settings,'primary',turn_aware=True)
    nav=LowSpeedNavigator(settings,camera='primary')
    for fid,t in ((1,1.),(2,1.1),(3,1.2)):
        result=context.inspect(image,mask,frame_id=fid,captured_s=t)
        assert not result['valid'] and result['left_inner']==result['right_inner']==[]
        assert not result['boundary_diagnostics']['history_pair_saved'] and context._pair_prior is None
        output=nav.update(image,{'frame_id':fid,'captured_s':t,'road_context':result,'tracks':[]},t)
        assert output['line_state']=='missing' and not output['boundary_observed']
        assert not output.get('lane_bounds_px') and not output['cones']


@pytest.mark.parametrize('turn,surface,expected',[
    (True,'indoor_white_line_track',[255,220,200,180,160]),
    (False,'indoor_white_line_track',[255,220,200,180]),
    (True,'red_running_track',[255,220,200,180]),
])
def test_extra_160_candidate_is_exclusive_to_indoor_turn_guidance(settings,turn,surface,expected):
    settings['scene']['surface']=surface
    context=ConeRoadContext(settings,turn_aware=turn)
    result=inspect(context,np.full((360,480,3),50,np.uint8))
    diag=result['boundary_diagnostics']
    assert not result['valid']
    assert [item['white_v_min'] for item in diag['threshold_attempts']]==expected


@pytest.mark.parametrize('threshold,expected',[(200,[200,180,160]),(160,[160]),(159,[159])])
def test_threshold_attempt_budget_matches_configured_candidates_and_does_not_lower_below_160(settings,threshold,expected):
    settings['scene']['white_v_min']=threshold
    result=inspect(ConeRoadContext(settings,turn_aware=True),np.full((360,480,3),50,np.uint8))
    diag=result['boundary_diagnostics']
    assert [item['white_v_min'] for item in diag['threshold_attempts']]==expected
    assert diag['max_threshold_attempts']==len(expected)


def test_budget_exhausted_threshold_cannot_contribute_single_candidates(settings,monkeypatch):
    def detect(image,mask,config,*,prior=None):
        budget=config.white_v_min==200
        # Deliberately include a stripe in a failed attempt to verify the
        # context boundary cannot leak even malformed budget-exhausted output.
        candidates=[stripe(100 if budget else 350)]
        return [],[],np.zeros(image.shape[:2],np.uint8),{
            'reason':'row_association_budget_exhausted' if budget else 'no_supported_curve_pair',
            'association_budget_exhausted':budget},candidates
    monkeypatch.setattr(cone_road,'curve_inner_edges',detect)
    result=inspect(ConeRoadContext(settings,turn_aware=True))
    assert not result['valid']
    assert len(result['boundary_candidates'])==1
    assert result['boundary_candidates'][0]['left_edge'][0][0]==350
    assert 200 not in result['boundary_diagnostics']['candidate_thresholds']


def test_all_exhausted_thresholds_return_no_single_or_pair_and_do_not_save_history(settings,monkeypatch):
    def detect(image,mask,config,*,prior=None):
        return [],[],np.zeros(image.shape[:2],np.uint8),{
            'reason':'row_association_budget_exhausted','association_budget_exhausted':True},[stripe(100)]
    monkeypatch.setattr(cone_road,'curve_inner_edges',detect)
    context=ConeRoadContext(settings,turn_aware=True);result=inspect(context)
    assert not result['valid'] and not result['boundary_candidates']
    assert not result['boundary_diagnostics']['history_pair_saved'] and context._pair_prior is None
