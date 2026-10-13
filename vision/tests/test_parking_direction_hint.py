"""Blue objects in a raised camera view are not evidence of a parking bay."""
import cv2
import numpy as np
import pytest
from carvision.parking_bay import board_direction_hint


CONFIG={'blue_lower':[90,70,40],'blue_upper':[135,255,255],'surface':'auto'}


@pytest.mark.parametrize('floor_hue',[0,65])
@pytest.mark.parametrize('side',['left','right'])
def test_board_on_visible_track_gives_direction_without_any_yellow_boundaries(floor_hue,side):
    hsv=np.full((360,480,3),(0,0,150),np.uint8)
    hsv[190:,:,:]=(floor_hue,180,160)
    x=40 if side=='left' else 270
    hsv[70:190,x:x+160]=(115,220,200)
    result=board_direction_hint(hsv,CONFIG)
    assert result['blocked_side']==side
    assert result['suggested_slot']==('right' if side=='left' else 'left')
    assert result['boards'][0]['ground_below_fraction']==1.


def test_large_blue_object_above_gray_furniture_does_not_select_a_bay():
    hsv=np.full((360,480,3),(0,0,130),np.uint8)
    hsv[72:231,254:480]=(115,170,100)
    result=board_direction_hint(hsv,CONFIG)
    assert result['suggested_slot'] is None and not result['boards']
