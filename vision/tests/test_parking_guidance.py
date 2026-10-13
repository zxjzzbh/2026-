"""Pixel regressions for the visible part of an already locked parking bay."""
import json

import cv2
import numpy as np
import pytest

from carvision.parking_guidance import visible_far_guides


def masks(*,side='both',far=True,blue_obstacle=False,dark=False):
    shape=(360,480);yellow=np.zeros(shape,np.uint8);white=np.zeros(shape,np.uint8)
    ground=np.zeros(shape,np.uint8) if dark else np.full(shape,255,np.uint8)
    blue=np.zeros(shape,np.uint8)
    if far:cv2.line(yellow,(180,145),(300,145),255,7)
    if side in ('both','left'):cv2.line(yellow,(180,145),(100,350),255,7)
    if side in ('both','right'):cv2.line(white,(300,145),(380,350),255,7)
    if blue_obstacle:cv2.rectangle(blue,(205,200),(255,260),255,-1)
    ground[(yellow>0)|(white>0)|(blue>0)]=0
    return yellow,white,ground,blue


def test_visible_far_and_two_sides_after_near_line_left_frame():
    rows=visible_far_guides(*masks())
    assert len(rows)==1
    row=rows[0];q=np.asarray(row['corners_px'])
    assert row['near_boundary_observed'] is False and row['far_boundary_observed'] is True
    assert row['geometry_source']=='image_far_corridor'
    assert q[0,0]<q[1,0] and q[3,0]<q[2,0]
    assert np.allclose(q[:2,1],288,atol=2) and np.all(q[2:,1]<160)
    assert row['far_boundary_support']>=.7 and min(row['side_boundary_support'])>=.75
    assert not any(k in row for k in ('id','metric_geometry','polygon_m','layout_found'))
    json.dumps(rows,allow_nan=False)


@pytest.mark.parametrize('side',['neither','left','right'])
def test_only_far_crossbar_or_missing_side_is_not_a_guide(side):
    assert visible_far_guides(*masks(side=side))==[]


def test_no_far_line_and_roi_top_are_not_real_boundaries():
    assert visible_far_guides(*masks(far=False))==[]


@pytest.mark.parametrize('case',['blue','dark'])
def test_blue_obstacle_or_black_ground_cannot_be_filled_as_free_floor(case):
    assert visible_far_guides(*masks(blue_obstacle=case=='blue',dark=case=='dark'))==[]


def test_image_border_cannot_substitute_for_a_side():
    y,w,g,b=masks(side='left')
    cv2.line(y,(180,145),(479,145),255,7)
    g[(y>0)|(w>0)]=0
    assert visible_far_guides(y,w,g,b)==[]


def test_white_far_crossbar_is_not_yellow_parking_end():
    y,w,g,b=masks(far=False)
    cv2.line(w,(180,145),(300,145),255,7);g[w>0]=0
    assert visible_far_guides(y,w,g,b)==[]


def test_near_line_visible_inside_roi_is_not_mislabelled_as_crop():
    y,w,g,b=masks();cv2.line(y,(126,280),(354,280),255,7);g[y>0]=0
    assert visible_far_guides(y,w,g,b)==[]


def test_finite_noise_in_floor_does_not_remove_valid_guide():
    y,w,g,b=masks();g[200:202,225:227]=0
    assert len(visible_far_guides(y,w,g,b))==1


def test_nonmatching_masks_raise_instead_of_broadcasting_geometry():
    y,w,g,b=masks()
    with pytest.raises(ValueError):visible_far_guides(y,w,g[:100],b)
