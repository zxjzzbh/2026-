"""Bounded brightness recovery keeps the original geometric acceptance gates."""
import copy
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.cone_planning import load_settings
from carvision.cone_road import ConeRoadContext

ROOT=Path(__file__).resolve().parents[1]


def settings(threshold=255, surface='indoor_white_line_track'):
    value=load_settings(ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json')
    value['scene'].update(white_v_min=threshold,surface=surface)
    return value


def photo(name):
    value=cv2.imdecode(np.frombuffer((ROOT/'tests/fixtures'/name).read_bytes(),np.uint8),1)
    assert value is not None and value.shape[:2]==(360,480)
    return value


@pytest.mark.parametrize('name,effective',[
    ('cone-track-glare-secondary.jpg',220),
    ('cone-track-short-secondary.jpg',200),
])
def test_real_track_recovers_from_255_without_changing_saved_settings(name,effective):
    config=settings();original=copy.deepcopy(config)
    context=ConeRoadContext(config)
    image=photo(name);mask=np.zeros(image.shape[:2],np.uint8)
    strict=context._inspect_threshold(image,mask,context.lane)
    assert not strict['valid'] and not strict['white_lines_detected']
    result=context.inspect(image,mask);diag=result['boundary_diagnostics']
    assert result['valid'] and result['white_lines_detected']
    assert diag['configured_white_v_min']==255 and diag['effective_white_v_min']==effective
    assert diag['threshold_fallback_used'] and diag['support_rows']>=10
    assert [r['white_v_min'] for r in diag['threshold_attempts']]==([255,220] if effective==220 else [255,220,200])
    assert not result['metric_boundary_verified'] and config==original
    assert context.lane.config.white_v_min==255
    # A new threshold still returns measured paint edges with joint support.
    for left,right in zip(result['left_inner'],result['right_inner']):
        assert left[0]<right[0] and left[1]==right[1]


@pytest.mark.parametrize('case',[
    'empty_green','empty_grey','white_floor','black','blue_board','shadow',
    'single','crossbar','wide_glare','flecks','short_parallel',
])
def test_fallback_does_not_invent_a_corridor_without_two_supported_edges(case):
    image=np.full((360,480,3),(35,100,35),np.uint8)
    if case=='empty_grey':image[:]=205
    elif case=='white_floor':image[:]=230
    elif case=='black':image[:]=0
    elif case=='blue_board':image[:]=(220,35,20)
    elif case=='shadow':
        cv2.fillPoly(image,[np.array([[50,359],[185,140],[295,140],[430,359]])],(15,45,15))
    elif case=='single':cv2.line(image,(30,320),(200,110),(220,220,220),9)
    elif case=='crossbar':
        cv2.line(image,(20,210),(460,210),(220,220,220),12)
        cv2.line(image,(240,210),(240,120),(220,220,220),10)
    elif case=='wide_glare':image[:,:170]=220;image[:,310:]=220
    elif case=='flecks':
        for x,y in np.random.default_rng(42).integers([0,100],[480,360],(80,2)):
            cv2.circle(image,(int(x),int(y)),2,(220,220,220),-1)
    elif case=='short_parallel':
        for x in (100,380):cv2.line(image,(x,190),(x,224),(220,220,220),5)
    result=ConeRoadContext(settings()).inspect(image,np.zeros(image.shape[:2],np.uint8))
    assert not result['valid'] and result['left_inner']==result['right_inner']==[]
    diag=result['boundary_diagnostics']
    assert not diag['threshold_fallback_used']
    assert len(diag['threshold_attempts'])<=4


def test_recovery_cannot_bypass_red_ground_requirement_or_blue_occlusion_mask():
    image=photo('cone-track-glare-secondary.jpg');mask=np.zeros(image.shape[:2],np.uint8)
    result=ConeRoadContext(settings(surface='red_running_track')).inspect(image,mask)
    assert not result['valid']
    assert any(r['reason']=='red_ground_not_confirmed' for r in result['boundary_diagnostics']['threshold_attempts'])
    result=ConeRoadContext(settings()).inspect(image,np.full(image.shape[:2],255,np.uint8))
    assert not result['valid'] and not result['white_lines_detected']


def test_good_strict_threshold_stays_authoritative_without_extra_passes():
    result=ConeRoadContext(settings(220)).inspect(photo('cone-track-glare-secondary.jpg'),np.zeros((360,480),np.uint8))
    diag=result['boundary_diagnostics']
    assert result['valid'] and not diag['threshold_fallback_used']
    assert diag['effective_white_v_min']==220 and len(diag['threshold_attempts'])==1


@pytest.mark.parametrize('name',['blue-board-close.jpg','cone-removed-secondary-a.jpg'])
def test_real_board_and_bare_floor_remain_invalid_after_all_fallback_passes(name):
    image=cv2.imdecode(np.frombuffer((ROOT/'tests/fixtures'/name).read_bytes(),np.uint8),1)
    image=cv2.resize(image,(480,360))
    result=ConeRoadContext(settings()).inspect(image,np.zeros((360,480),np.uint8))
    assert not result['valid'] and not result['white_lines_detected']
    assert len(result['boundary_diagnostics']['threshold_attempts'])==4


def test_floor_threshold_is_never_lowered_beyond_the_bounded_candidate_set():
    result=ConeRoadContext(settings(180)).inspect(np.zeros((360,480,3),np.uint8),np.zeros((360,480),np.uint8))
    assert len(result['boundary_diagnostics']['threshold_attempts'])==1
    assert result['boundary_diagnostics']['effective_white_v_min']==180
