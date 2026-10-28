"""Explicit track/obstacle colors; no field calibration or actuator I/O."""
import copy
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.cone_planning import load_settings,validate_settings
from carvision.cone_road import ConeRoadContext
from carvision.cone_vision import ConeDetector,BlueConeDetector

ROOT=Path(__file__).resolve().parents[1]


def settings(surface, color=None):
    value=load_settings(ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json')
    value['scene']['surface']=surface
    if color is not None:value['scene']['obstacle_color']=color
    return validate_settings(value)


def cone(color):
    image=np.zeros((360,480,3),np.uint8)
    pixel=cv2.cvtColor(np.array([[[color,220,220]]],np.uint8),cv2.COLOR_HSV2BGR)[0,0].tolist()
    cv2.fillPoly(image,[np.array([[240,240],[205,315],[275,315]])],pixel)
    return image


@pytest.mark.parametrize('surface,expected,hue,wrong',[
    ('red_running_track','blue',120,0),
    ('blue_running_track','red',0,120),
    ('blue_running_track','red',175,120),
    ('indoor_white_line_track','blue',120,0),
])
def test_track_selects_only_the_configured_cone_color(surface,expected,hue,wrong):
    config=settings(surface);before=copy.deepcopy(config)
    detector=ConeDetector(config)
    rows,_=detector.detect(cone(hue))
    assert len(rows)==1 and rows[0]['kind']=='cone_candidate'
    assert rows[0]['obstacle_color']==expected and detector.color==expected
    assert detector.detect(cone(wrong))[0]==[]
    assert config==before and BlueConeDetector is ConeDetector


@pytest.mark.parametrize('hue',[0,175])
def test_indoor_obstacle_color_can_be_explicitly_red(hue):
    detector=ConeDetector(settings('indoor_white_line_track','red'))
    assert detector.detect(cone(hue))[0][0]['kind']=='cone_candidate'
    assert detector.detect(cone(120))[0]==[]


@pytest.mark.parametrize('surface,color,expected',[
    ('red_running_track',(230,0,0),'unresolved_blue'),
    ('blue_running_track',(0,0,230),'unresolved_red'),
])
def test_clipped_selected_color_is_retained_as_unresolved_not_passed(surface,color,expected):
    image=np.zeros((360,480,3),np.uint8)
    cv2.fillPoly(image,[np.array([[240,240],[175,359],[305,359]])],color)
    rows,_=ConeDetector(settings(surface)).detect(image)
    assert len(rows)==1 and rows[0]['kind']==expected
    assert not rows[0]['ground_contact_usable']


def road(color):
    image=np.full((360,480,3),color,np.uint8)
    cv2.line(image,(65,355),(200,130),(220,220,220),7)
    cv2.line(image,(415,355),(280,130),(220,220,220),7)
    return image


@pytest.mark.parametrize('surface,color,ground',[
    ('red_running_track',(40,40,160),'red'),
    ('blue_running_track',(160,40,40),'blue'),
])
def test_matching_ground_is_required_after_white_threshold_recovery(surface,color,ground):
    config=settings(surface);image=road(color)
    rows,mask=ConeDetector(config).detect(image)
    result=ConeRoadContext(config).inspect(image,mask)
    assert result['valid'] and result['ground_supported'] and result['ground_color']==ground
    assert result['ground_fraction']>.9
    assert result['boundary_diagnostics']['threshold_fallback_used']
    assert not result['metric_boundary_verified']
    other=road((160,40,40) if ground=='red' else (40,40,160))
    result=ConeRoadContext(config).inspect(other,np.zeros((360,480),np.uint8))
    assert not result['valid']


def test_base_scene_support_uses_blue_floor_for_red_cones():
    config=settings('blue_running_track');image=road((160,40,40))
    foreground=cone(0);colored=np.any(foreground>0,axis=2);image[colored]=foreground[colored]
    rows,mask=ConeDetector(config).detect(image)
    context=ConeRoadContext(config).inspect(image,mask)
    assert context['valid']
    ConeRoadContext.enrich(rows,image,context)
    candidates=[r for r in rows if r['kind']=='cone_candidate']
    assert len(candidates)==1 and candidates[0]['scene_supported']
    assert candidates[0]['ground_near_base']>.5
    assert candidates[0]['red_ground_near_base']<.15


@pytest.mark.parametrize('color',['green','auto',None])
def test_invalid_explicit_color_is_rejected(color):
    config=settings('indoor_white_line_track');config['scene']['obstacle_color']=color
    with pytest.raises(ValueError,match='obstacle_color'):validate_settings(config)
