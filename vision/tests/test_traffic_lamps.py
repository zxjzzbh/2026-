from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.classic_detection import ClassicRaceDetector
from carvision.config import Config
from carvision.detection import light_candidate


def reading(image):
    detections = ClassicRaceDetector().detect(image)
    return light_candidate(image, detections, Config().traffic_light)


@pytest.mark.parametrize('name,expected', [('traffic-red-overexposed.jpg', 'red'),
                                         ('traffic-yellow-overexposed.jpg', 'yellow'),
                                         ('traffic-green-overexposed.jpg', 'green')])
@pytest.mark.parametrize('scale', [.75, 1., 4/3])
def test_real_overexposed_lamps_at_multiple_scales(name, expected, scale):
    path = Path(__file__).parent / 'fixtures' / name
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    image = cv2.resize(image, None, fx=scale, fy=scale)
    assert reading(image) == expected


@pytest.mark.parametrize('name', ['traffic-background-primary.jpg', 'crosswalk-background-primary.jpg',
                                'blue-board-close.jpg', 'crosswalk-papers-secondary.jpg'])
def test_real_non_lamp_scenes_remain_unknown(name):
    path = Path(__file__).parent / 'fixtures' / name
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    assert reading(image) == 'unknown'


def test_real_unlit_fixture_is_present_but_has_no_active_color():
    path = Path(__file__).parent / 'fixtures/traffic-all-off.jpg'
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    detections = ClassicRaceDetector().detect(image)
    assert any(det.label == 'traffic_light' for det in detections)
    assert light_candidate(image, detections, Config().traffic_light) == 'unknown'


def lamp_scene(lit=None, white_core=False, rim=True):
    image = np.full((120, 320, 3), 25, np.uint8)
    for i, (x, color) in enumerate([(60, (0, 0, 70)), (160, (0, 70, 70)), (260, (0, 70, 0))]):
        on = i in (lit if isinstance(lit, tuple) else (lit,))
        if on:
            color = tuple(255 if v else 0 for v in color)
        cv2.circle(image, (x, 60), 25, color, -1)
        if on and white_core:
            if not rim:
                cv2.circle(image, (x, 60), 25, (255, 255, 255), -1)
            cv2.circle(image, (x, 60), 20, (255, 255, 255), -1)
    return image


@pytest.mark.parametrize('index,color', [(0, 'red'), (1, 'yellow'), (2, 'green')])
@pytest.mark.parametrize('white_core', [False, True])
def test_all_colors_with_normal_or_washed_out_cores(index, color, white_core):
    assert reading(lamp_scene(index, white_core)) == color


@pytest.mark.parametrize('index', [0, 1, 2])
def test_white_patch_without_colored_rim_is_not_a_lit_lamp(index):
    assert reading(lamp_scene(index, white_core=True, rim=False)) == 'unknown'


def test_yellow_patch_on_one_side_of_white_center_is_insufficient():
    image = lamp_scene(1, white_core=True, rim=False)
    cv2.rectangle(image, (180, 40), (195, 80), (0, 255, 255), -1)
    assert reading(image) == 'unknown'


@pytest.mark.parametrize('index,color', [(0, (0, 0, 255)), (2, (0, 255, 0))])
def test_edge_white_core_with_only_lower_half_colored_rim_is_insufficient(index, color):
    image = lamp_scene(index, white_core=True, rim=False)
    center = (60 + index*100, 60)
    cv2.ellipse(image, center, (25, 25), 0, 0, 180, color, 5)
    assert reading(image) == 'unknown'


def test_unlit_lenses_and_conflicting_lamps_do_not_allow_passage():
    assert reading(lamp_scene()) == 'unknown'
    assert reading(lamp_scene((0, 2))) == 'unknown'


def test_bright_housing_does_not_hide_red_lamp_contrast():
    image = lamp_scene(0)
    image[:20] = 255
    image[100:] = 255
    assert reading(image) == 'red'
