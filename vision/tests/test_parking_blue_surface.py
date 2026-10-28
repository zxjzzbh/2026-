"""Blue-floor observation stays separate from blue-board occupancy evidence."""
import copy

import cv2
import numpy as np
import pytest

from carvision.parking_bay import detect_parking,validate
from test_parking_bay import settings,calibrated,scene


def blue_scene():
    image=scene(None)
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    red=cv2.bitwise_or(cv2.inRange(hsv,(0,55,35),(15,255,255)),cv2.inRange(hsv,(165,55,35),(179,255,255)))
    image[red>0]=(170,45,35)
    return image


@pytest.mark.parametrize('surface',['blue_running_track','auto'])
def test_blue_floor_is_reported_without_becoming_a_blue_board_or_verified_empty_slot(surface):
    config=settings();config['surface']=surface;before=copy.deepcopy(config)
    view,_=detect_parking(blue_scene(),validate(config))
    assert view['layout_found'] and view['ground_surface']=='blue_running_track'
    assert all(slot['ground_fraction']>.9 and slot['ground_color_supported'] for slot in view['slots'])
    assert all(slot['availability']=='unknown' and slot['occupancy_ambiguous']
               and not slot['occupancy_verified'] for slot in view['slots'])
    assert config==before and not view['perception_verified']


def test_red_ground_does_not_support_an_explicit_blue_floor_configuration():
    config=settings();config['surface']='blue_running_track'
    view,_=detect_parking(scene(None),config)
    assert view['layout_found']
    assert all(slot['ground_fraction']<.05 and not slot['ground_color_supported'] for slot in view['slots'])
    assert all(slot['availability']=='unknown' for slot in view['slots'])


def test_existing_generic_verification_does_not_resolve_blue_floor_board_ambiguity():
    config=calibrated();config['surface']='blue_running_track'
    view,_=detect_parking(blue_scene(),config)
    assert view['metric_geometry'] and view['perception_verified']
    assert all(not slot['occupancy_verified'] and slot['availability']=='unknown' for slot in view['slots'])


def test_red_ground_auto_mode_still_detects_the_blue_obstruction_independently():
    config=settings();config['surface']='auto'
    view,_=detect_parking(scene('left'),config)
    assert view['ground_surface']=='red_running_track'
    by_id={slot['id']:slot for slot in view['slots']}
    assert by_id['left']['availability']=='blocked' and by_id['right']['availability']=='clear'
    assert not any(slot['occupancy_verified'] for slot in view['slots'])
