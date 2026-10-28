"""Reference selection preserves lamp identity without changing photometry."""
import json

import cv2
import numpy as np
import pytest

from carvision.traffic_signal import TrafficSignalDetector


def fixture(image,centres,y,radius,active=None):
    for index,(x,color) in enumerate(zip(centres,((0,0,70),(0,70,70),(0,70,0)))):
        if index==active:color=tuple(255 if value else 0 for value in color)
        cv2.circle(image,(int(x),int(y)),int(radius),color,-1)


def single(active=0,*,scale=1.,shift=0):
    image=np.full((360,480,3),25,np.uint8)
    fixture(image,[240+shift+offset*scale for offset in (-110,0,110)],100,27*scale,active)
    return image


def two_fixtures():
    image=single(0)
    fixture(image,[300,365,430],260,17,2)
    return image


@pytest.mark.parametrize('active,state',[(None,'off'),(0,'red'),(1,'yellow'),(2,'green')])
def test_no_reference_keeps_existing_classification_and_none_is_compatible(active,state):
    detector=TrafficSignalDetector();image=single(active)
    default=detector.detect(image);explicit=detector.detect(image,reference_box=None)
    assert default['state']==explicit['state']==state
    for row in (default,explicit):row.pop('processing_ms')
    assert default==explicit


def test_reference_selects_only_its_fixture_with_a_different_green_fixture_visible():
    detector=TrafficSignalDetector()
    reference=detector.detect(single(0))['bbox_xyxy']
    result=detector.detect(two_fixtures(),reference_box=reference)
    assert result['state']=='red' and result['fixture_detected']
    assert result['bbox_xyxy']==reference
    diag=result['candidate_diagnostics']
    assert diag['fixture_candidate_count']>=2 and diag['reference_match_count']==1
    assert diag['selected_reference_geometry']['iou']>=.25
    json.dumps(result,allow_nan=False)


def test_missing_reference_never_selects_an_unrelated_green_fixture():
    detector=TrafficSignalDetector()
    image=np.full((360,480,3),25,np.uint8)
    fixture(image,[300,365,430],260,17,2)
    assert detector.detect(image)['state']=='green'
    result=detector.detect(image,reference_box=detector.detect(single())['bbox_xyxy'])
    assert result['state']=='unknown' and not result['fixture_detected']
    assert result['bbox_xyxy'] is None and result['reason']=='tracked_fixture_missing'
    assert result['candidate_diagnostics']['fixture_candidate_count']>=1
    assert result['candidate_diagnostics']['reference_match_count']==0


def test_clipped_foreground_red_cannot_be_replaced_by_the_background_off_lamp():
    detector=TrafficSignalDetector()
    reference=detector.detect(single())['bbox_xyxy']
    image=np.full((360,480,3),25,np.uint8)
    # The locked foreground fixture moves left, clipping its red lens. A
    # complete, distant unlit fixture must not clear the foreground incident.
    fixture(image,[-30,100,230],100,30,0)
    fixture(image,[300,365,430],260,17,None)
    assert detector.detect(image)['state']=='off'
    result=detector.detect(image,reference_box=reference)
    assert result['reason']=='tracked_fixture_missing'
    assert result['state']=='unknown' and not result['fixture_detected']


@pytest.mark.parametrize('scale,shift',[(1.2,0),(1.,30),(.8,0)])
def test_same_fixture_moderate_perspective_and_translation_remain_usable(scale,shift):
    detector=TrafficSignalDetector()
    reference=detector.detect(single())['bbox_xyxy']
    result=detector.detect(single(scale=scale,shift=shift),reference_box=reference)
    assert result['state']=='red' and result['fixture_detected']
    assert result['candidate_diagnostics']['reference_match_count']>=1


@pytest.mark.parametrize('reference_scale',[.5,2.])
def test_area_jump_is_rejected_even_when_bounding_boxes_overlap(reference_scale):
    detector=TrafficSignalDetector();image=single()
    box=detector.detect(image)['bbox_xyxy']
    x,y=(box[0]+box[2])/2,(box[1]+box[3])/2
    width,height=(box[2]-box[0])*reference_scale,(box[3]-box[1])*reference_scale
    reference=[x-width/2,y-height/2,x+width/2,y+height/2]
    # Expand the image canvas when necessary to keep a valid nonnegative
    # reference. The changed area still describes a different-sized object.
    if min(reference)<0:
        offset=[max(0,-reference[0]),max(0,-reference[1])]
        image=cv2.copyMakeBorder(image,round(offset[1]),0,round(offset[0]),0,cv2.BORDER_CONSTANT,value=(25,25,25))
        reference=[reference[i]+offset[i%2] for i in range(4)]
    result=detector.detect(image,reference_box=reference)
    assert result['reason']=='tracked_fixture_missing' and result['state']=='unknown'


def test_unknown_emission_is_not_inferred_from_a_previously_red_reference():
    detector=TrafficSignalDetector();reference=detector.detect(single())['bbox_xyxy']
    image=single(None)
    for x,color in ((130,(0,0,190)),(240,(0,240,240)),(350,(0,175,0))):
        cv2.circle(image,(x,100),27,color,-1)
    result=detector.detect(image,reference_box=reference)
    assert result['state']=='unknown' and result['reason']=='ambiguous_emission'


def test_multiple_fixture_candidates_stay_unknown_even_with_a_bounding_box():
    detector=TrafficSignalDetector();result=detector.detect(two_fixtures())
    assert result['fixture_detected'] and result['bbox_xyxy']
    assert result['state']=='unknown' and result['reason']=='multiple_fixture_candidates'


@pytest.mark.parametrize('box',[[0,0,0,10],[10,0,0,10],[0,0,float('nan'),10],
                               [0,0,float('inf'),10],[False,0,100,10],[-1,0,100,10],
                               [0,0,100],{'bbox_xyxy':[0,0,100,10]}])
def test_invalid_reference_is_rejected_instead_of_falling_back_to_global_selection(box):
    with pytest.raises(ValueError,match='reference_box'):
        TrafficSignalDetector().detect(single(),reference_box=box)
