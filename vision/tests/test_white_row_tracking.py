"""Current-pixel row association; prior ranks accepted pairs and cannot fill gaps."""
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.config import load_config
from carvision.cone_corridor import curve_inner_edges,measured_corridor


def config(**changes):
    return replace(replace(load_config(None).lane,white_v_min=180,white_s_max=70,roi_top=.53),**changes)


def image(stripes=(),*,top=210,bottom=294):
    value=np.full((360,480,3),55,np.uint8)
    for x in stripes:cv2.line(value,(x,top),(x,bottom),(220,220,220),6)
    return value


def detect(value,*,prior=None,obstacle=None,**changes):
    return curve_inner_edges(value,np.zeros(value.shape[:2],np.uint8) if obstacle is None else obstacle,
                             config(**changes),prior=prior)


def prior_of(result):return {'left_inner':result[0],'right_inner':result[1]}


def assert_measured_pair(result):
    left,right,mask,diag,candidates=result
    assert left and right and len(left)==len(right)
    assert diag['reason']=='observed_curve_pair' and diag['association_policy']=='near_to_far_bounded_rows'
    assert not diag['association_budget_exhausted']
    assert len(candidates)<=12 and diag['pair_combinations']<=66 and diag['peak_active_paths']<=32
    assert diag['support_rows']>=8 and diag['common_span_px']>=32.4
    # Both inner edges are actual current white pixels, on identical rows.
    for l,r in zip(left,right):
        assert l[1]==r[1] and l[0]<r[0]
        for x,y in (l,r):
            # Extraction observes a three-row majority, not a fabricated
            # requirement that the centre row alone is free of JPEG holes.
            assert np.mean(mask[max(0,int(y)-1):int(y)+2,int(x)]>0)>=.5
    model=measured_corridor({'valid':True,'left_inner':left,'right_inner':right},mask.shape)
    assert model.low>=round(mask.shape[0]*.53)


@pytest.mark.parametrize('center,slope',[(110.,.35),(370.,-.35)])
def test_two_boundaries_can_lean_together_on_same_half_of_image(center,slope):
    value=image();ys=np.arange(198,298)
    middle=center+slope*(ys-248)
    for sign in (-1,1):
        points=np.column_stack((middle+sign*46,ys)).astype(np.int32)
        cv2.polylines(value,[points],False,(220,220,220),6)
    result=detect(value);assert_measured_pair(result)
    left,right=result[:2]
    assert np.all(np.asarray(right)[:,0]<240) if center<240 else np.all(np.asarray(left)[:,0]>240)


@pytest.mark.parametrize('bend',[8.,16.,24.])
def test_near_to_far_paths_retain_current_curved_tape_instead_of_a_straight_only_fit(bend):
    value=image();ys=np.arange(198,310);center=232+bend*((ys-250)/58)**2
    width=180+.5*(ys-250)
    for sign in (-1,1):
        points=np.column_stack((center+sign*width/2,ys)).astype(np.int32)
        cv2.polylines(value,[points],False,(220,220,220),7)
    assert_measured_pair(detect(value))


def test_same_pixels_are_deterministic_despite_the_global_random_state():
    value=image((100,220,340))
    outputs=[]
    for seed in (1,19,20261026):
        np.random.seed(seed);result=detect(value);assert_measured_pair(result)
        outputs.append((result[0],result[1],result[3]))
    assert outputs[0]==outputs[1]==outputs[2]


def test_prior_selects_same_pair_only_among_similarly_supported_current_pairs():
    old=detect(image((220,340)));assert_measured_pair(old)
    current=image((100,220,340));plain=detect(current);linked=detect(current,prior=prior_of(old))
    assert_measured_pair(linked)
    assert linked[3]['valid_pair_count']>=2 and linked[3]['prior_used']
    assert linked[3]['prior_reason']=='ranked_similar_current_pairs'
    assert np.median(np.asarray(linked[0])[:,0])>215
    assert np.median(np.asarray(linked[1])[:,0])>330
    assert (linked[0],linked[1])!=(plain[0],plain[1])


def test_clearly_stronger_new_pair_takes_over_instead_of_following_a_weak_prior():
    old=detect(image((100,220)));assert_measured_pair(old)
    current=image((260,380),top=198,bottom=310)
    for x in (100,220):cv2.line(current,(x,252),(x,297),(220,220,220),6)
    result=detect(current,prior=prior_of(old));assert_measured_pair(result)
    assert np.median(np.asarray(result[0])[:,0])>255
    assert result[3]['support_rows']>25


@pytest.mark.parametrize('bad_prior',[None,{}, {'left_inner':[[float('nan'),210]],'right_inner':[]}])
def test_invalid_prior_does_not_change_current_acceptance(bad_prior):
    value=image((100,340));plain=detect(value);result=detect(value,prior=bad_prior)
    assert (result[0],result[1])==(plain[0],plain[1])
    assert not result[3]['prior_used']


def test_prior_without_common_actual_observation_band_is_ignored():
    old=detect(image((100,340),top=195,bottom=245));assert_measured_pair(old)
    current=image((100,340),top=270,bottom=324)
    result=detect(current,prior=prior_of(old));assert_measured_pair(result)
    assert not result[3]['prior_used'] and result[3]['prior_reason']=='no_common_observed_band'


@pytest.mark.parametrize('missing',['left','right','both','occluded'])
def test_prior_never_generates_a_current_missing_boundary(missing):
    prior=prior_of(detect(image((100,340))))
    stripes=() if missing=='both' else (340,) if missing=='left' else (100,) if missing=='right' else (100,340)
    value=image(stripes);obstacle=np.zeros((360,480),np.uint8)
    if missing=='occluded':obstacle[:,330:350]=255
    result=detect(value,prior=prior,obstacle=obstacle)
    assert result[0]==result[1]==[] and not result[3]['prior_used']
    assert result[3]['reason']=='no_supported_curve_pair'


@pytest.mark.parametrize('kind',['empty','white_floor','blue_board','cabinet_above_ground','crossbar','short_posts','shadow'])
def test_non_corridor_shapes_cannot_become_paired_paint(kind):
    value=image()
    if kind=='white_floor':value[:]=220
    elif kind=='blue_board':cv2.rectangle(value,(80,180),(400,330),(220,40,20),-1)
    elif kind=='cabinet_above_ground':
        for x in (100,340):cv2.line(value,(x,50),(x,187),(220,220,220),7)
    elif kind=='crossbar':cv2.line(value,(40,245),(440,245),(220,220,220),12)
    elif kind=='short_posts':
        for x in (100,340):cv2.line(value,(x,230),(x,267),(220,220,220),6)
    elif kind=='shadow':cv2.fillPoly(value,[np.array([[50,359],[185,190],[295,190],[430,359]])],(10,30,10))
    result=detect(value)
    assert result[0]==result[1]==[]


@pytest.mark.parametrize('kind',['stripes','flecks'])
def test_clutter_exhaustion_is_bounded_explicit_unknown_without_partial_single_candidates(kind):
    value=image()
    if kind=='stripes':
        for x in range(8,478,14):cv2.line(value,(x,190),(x,324),(220,220,220),4)
    else:
        for x,y in np.random.default_rng(7).integers((0,191),(480,324),(600,2)):
            cv2.rectangle(value,(int(x),int(y)),(min(479,int(x)+3),min(359,int(y)+12)),(220,220,220),-1)
    result=detect(value)
    assert result[0]==result[1]==result[4]==[]
    assert result[3]['association_budget_exhausted']
    assert result[3]['reason']=='row_association_budget_exhausted'


def test_low_saturation_threshold_strips_without_a_local_bright_ridge_stay_unknown():
    value=np.full((360,480,3),(90,125,180),np.uint8)
    for x in (100,340):cv2.line(value,(x,210),(x,294),(165,165,170),6)
    # The strips pass S/V and geometric shape, but are dimmer than their floor.
    result=detect(value,white_v_min=160)
    assert result[0]==result[1]==result[4]==[]
    assert result[3]['paint_paths_rejected']>=2


def test_bright_ridge_requires_both_flanks_and_a_majority_of_current_rows():
    value=np.full((360,480,3),(90,125,180),np.uint8)
    for x in (100,340):
        cv2.line(value,(x,210),(x,294),(184,184,184),6)
        cv2.line(value,(x,210),(x,239),(225,225,225),6)
    result=detect(value,white_v_min=160)
    assert result[0]==result[1]==[] and result[3]['paint_paths_rejected']>=2


def test_dim_but_distinct_white_paint_keeps_current_pixel_evidence():
    value=image((100,340));value[value[:,:,0]>100]=170
    result=detect(value,white_v_min=160);assert_measured_pair(result)
    assert result[3]['paint_min_contrast_v']==10.
    assert all(row['supported_row_fraction']>=.70 for row in result[3]['paint_candidate_support'])


def test_current_white_tape_remains_distinct_inside_a_shadow_band():
    value=image((100,340))
    value[230:270]=(value[230:270].astype(float)*.78).astype(np.uint8)
    assert_measured_pair(detect(value,white_v_min=160))


def test_single_bright_floor_edge_cannot_substitute_for_a_white_ridge():
    value=np.full((360,480,3),(70,100,145),np.uint8)
    value[:,104:]=(100,130,195)
    cv2.line(value,(100,210),(100,294),(170,170,170),6)
    cv2.line(value,(340,210),(340,294),(230,230,230),6)
    result=detect(value,white_v_min=160)
    assert result[0]==result[1]==[] and result[3]['paint_paths_rejected']>=1


def test_image_edge_uses_only_actual_available_flank_without_zero_padding():
    good=detect(image((0,340)),white_v_min=160)
    assert_measured_pair(good)
    value=np.full((360,480,3),(100,130,180),np.uint8)
    for x in (0,340):cv2.line(value,(x,210),(x,294),(170,170,170),6)
    bad=detect(value,white_v_min=160)
    assert bad[0]==bad[1]==bad[4]==[]


def test_unobserved_flanks_are_not_infinite_or_zero_padded_positive_contrast():
    result=detect(np.full((360,480,3),220,np.uint8),max_run_fraction=1.)
    assert result[0]==result[1]==result[4]==[]
    assert result[3]['paint_rows_without_flanks']>0


def test_real_dark_primary_floor_glare_is_not_a_white_boundary():
    path=Path(__file__).parent/'fixtures/white-live-dark-20261026-primary-1.jpg'
    value=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    result=detect(value,white_v_min=160,roi_top=.38)
    # The true tape has insufficient joint support; a floor threshold fragment
    # must not substitute for the missing left edge or become a single hint.
    assert result[0]==result[1]==[]
    assert result[3]['paint_paths_rejected']>=1


@pytest.mark.parametrize('series,index,threshold',[
    *[('stability',index,180) for index in (0,4,8,12,16,20,24)],
    *[('dark',index,160) for index in (0,1,2)],
])
def test_real_secondary_tape_keeps_both_geometry_and_current_bright_ridge(series,index,threshold):
    path=Path(__file__).parent/'fixtures'/f'white-live-{series}-20261026-secondary-{index}.jpg'
    value=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    result=detect(value,white_v_min=threshold)
    assert_measured_pair(result)
    assert all(row['median_contrast_v']>=10. for row in result[3]['paint_candidate_support'])
