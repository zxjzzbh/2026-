from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.classic_detection import ClassicRaceDetector


def cones(image):
    return [d for d in ClassicRaceDetector().detect(image) if d.label == 'cone']


@pytest.mark.parametrize('camera,box', [('primary', (205, 245, 273, 330)),
                                      ('secondary', (255, 278, 293, 330))])
@pytest.mark.parametrize('sample', ['a', 'b'])
@pytest.mark.parametrize('scale', [.75, 1., 4/3])
def test_real_cone_is_localized_without_background_candidates(camera, box, sample, scale):
    data = (Path(__file__).parent/'fixtures'/f'cone-{camera}-{sample}.jpg').read_bytes()
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    image = cv2.resize(image, None, fx=scale, fy=scale)
    detections = cones(image)
    assert len(detections) == 1
    x1, y1, x2, y2 = np.asarray(detections[0].bbox_xyxy) / scale
    a1, b1, a2, b2 = box
    intersection = max(0, min(x2, a2)-max(x1, a1)) * max(0, min(y2, b2)-max(y1, b1))
    union = (x2-x1)*(y2-y1) + (a2-a1)*(b2-b1) - intersection
    assert intersection / union > .55


@pytest.mark.parametrize('camera', ['primary', 'secondary'])
@pytest.mark.parametrize('sample', ['a', 'b'])
@pytest.mark.parametrize('scale', [.75, 1., 4/3])
def test_real_cone_removed_scene_has_no_cone_candidates(camera, sample, scale):
    data = (Path(__file__).parent/'fixtures'/f'cone-removed-{camera}-{sample}.jpg').read_bytes()
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    image = cv2.resize(image, None, fx=scale, fy=scale)
    assert cones(image) == []


@pytest.mark.parametrize('color', [(255, 0, 0), (0, 0, 255)])
def test_upright_cone_against_cyan_ground(color):
    image = np.full((480, 640, 3), (95, 90, 25), np.uint8)
    cv2.fillPoly(image, [np.array([(300, 270), (310, 270), (333, 335), (277, 335)])], color)
    assert len(cones(image)) == 1


@pytest.mark.parametrize('shape', ['circle', 'rectangle', 'inverted', 'speckles', 'clipped'])
@pytest.mark.parametrize('color', [(255, 0, 0), (0, 0, 255)])
def test_non_cone_colored_shapes_do_not_become_cones(shape, color):
    image = np.zeros((480, 640, 3), np.uint8)
    if shape == 'circle':
        cv2.circle(image, (310, 300), 24, color, -1)
    elif shape == 'rectangle':
        cv2.rectangle(image, (280, 260), (330, 340), color, -1)
    elif shape == 'inverted':
        cv2.fillPoly(image, [np.array([(280, 260), (330, 260), (305, 335)])], color)
    elif shape == 'speckles':
        for x in (150, 230, 370):
            cv2.fillPoly(image, [np.array([(x, 300), (x-4, 310), (x+4, 310)])], color)
    else:
        cv2.fillPoly(image, [np.array([(310, 430), (280, 479), (340, 479)])], color)
    assert cones(image) == []


@pytest.mark.parametrize('name,box', [
    ('cone-right-reflection-a.jpg', (266,272,304,330)),
    ('cone-right-reflection-b.jpg', (266,272,304,330)),
    ('cone-right-rounded-tip.jpg', (266,272,304,330)),
    ('cone-left-with-small-background-patch.jpg', (230,282,271,331)),
])
@pytest.mark.parametrize('scale', [.75, 1., 4/3])
def test_cone_localization_survives_reflection_without_inventing_small_objects(name, box, scale):
    image = cv2.imdecode(np.frombuffer((Path(__file__).parent/'fixtures'/name).read_bytes(), np.uint8), 1)
    image = cv2.resize(image, None, fx=scale, fy=scale)
    detections = cones(image)
    assert len(detections) == 1
    x1, y1, x2, y2 = np.asarray(detections[0].bbox_xyxy) / scale
    a1, b1, a2, b2 = box
    intersection = max(0, min(x2, a2)-max(x1, a1)) * max(0, min(y2, b2)-max(y1, b1))
    union = (x2-x1)*(y2-y1) + (a2-a1)*(b2-b1) - intersection
    assert intersection / union > .45


@pytest.mark.parametrize('color', [(255, 0, 0), (0, 0, 255)])
@pytest.mark.parametrize('tail', [0, 6, 12])
def test_rounded_tip_cone_with_a_narrow_reflection_tail(color, tail):
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.fillPoly(image, [np.array([(301, 280), (311, 280), (326, 318),
                                  (306, 330+tail), (286, 318)])], color)
    # This synthetic silhouette models a tapered body with a reflected lower
    # tail; its widest region, rather than its lowest pixel, defines the base.
    assert len(cones(image)) == 1
