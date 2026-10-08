import cv2
import numpy as np
import pytest

from carvision.classic_detection import ClassicRaceDetector
from carvision.config import Config
from carvision.pipeline import Pipeline
from carvision.sources import Frame


def test_blue_rectangle_is_board_candidate_and_small_triangle_is_cone():
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(image, (100, 180), (220, 330), (255, 0, 0), -1)
    cv2.fillPoly(image, [np.array([[450, 320], [435, 360], [465, 360]])], (0, 0, 255))
    candidates = ClassicRaceDetector().detect(image)
    assert any(c.label == 'blue_board' for c in candidates)
    assert any(c.label == 'cone' for c in candidates)


def test_multiple_compact_ground_patches_are_crosswalk_candidate():
    image = np.zeros((480, 640, 3), np.uint8)
    for x in [130, 230, 330, 430]:
        cv2.rectangle(image, (x, 320), (x + 60, 340), (255, 255, 255), -1)
    assert any(c.label == 'crosswalk' for c in ClassicRaceDetector().detect(image))


def test_untrained_backend_is_explicitly_experimental():
    image = np.zeros((480, 640, 3), np.uint8)
    result, _ = Pipeline(Config(), ClassicRaceDetector()).process(Frame(image, 0, 0, 0))
    assert result.detector_status == 'experimental_color_shape'


def test_empty_scene_does_not_invent_race_objects():
    assert ClassicRaceDetector().detect(np.zeros((480, 640, 3), np.uint8)) == []


def test_near_board_covering_the_view_is_not_treated_as_removed():
    image = np.full((480, 640, 3), (255, 0, 0), dtype=np.uint8)
    pipeline = Pipeline(Config(), ClassicRaceDetector())
    for timestamp in (0, 100, 200):
        result, _ = pipeline.process(Frame(image, timestamp // 100, timestamp, timestamp))
    assert result.presence['blue_board'] == 'present'
    assert result.detector_status == 'experimental_color_shape'


def test_partial_large_panel_can_be_seen_without_all_four_edges():
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(image, (0, 230), (560, 479), (255, 0, 0), -1)
    assert any(c.label == 'blue_board' for c in ClassicRaceDetector().detect(image))


def test_nearly_full_panel_with_visible_border_remains_a_board():
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(image, (5, 5), (634, 474), (255, 0, 0), -1)
    assert any(c.label == 'blue_board' for c in ClassicRaceDetector().detect(image))


@pytest.mark.parametrize('color', [(0, 0, 255), (0, 255, 0), (180, 180, 180), (0, 0, 0)])
def test_other_full_view_colors_do_not_become_blue_board(color):
    image = np.full((480, 640, 3), color, dtype=np.uint8)
    assert not any(c.label == 'blue_board' for c in ClassicRaceDetector().detect(image))


def test_thin_blue_boundary_does_not_become_near_board():
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(image, (0, 410), (639, 479), (255, 0, 0), -1)
    assert not any(c.label == 'blue_board' for c in ClassicRaceDetector().detect(image))


def test_recorded_real_near_board_is_detected():
    from pathlib import Path
    data = (Path(__file__).parent / 'fixtures/blue-board-close.jpg').read_bytes()
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert image is not None
    assert any(c.label == 'blue_board' for c in ClassicRaceDetector().detect(image))


def test_rule_paper_orientation_can_project_to_tall_patches():
    image = np.zeros((480, 640, 3), np.uint8)
    for x in (150, 250, 350, 450):
        cv2.rectangle(image, (x, 270), (x+35, 345), (255, 255, 255), -1)
    assert any(c.label == 'crosswalk' for c in ClassicRaceDetector().detect(image))


@pytest.mark.parametrize('layout', ['two', 'scattered', 'clipped_tips', 'long_lane_edges'])
def test_white_objects_without_a_visible_repeated_ground_row_are_not_crosswalk(layout):
    image = np.zeros((480, 640, 3), np.uint8)
    boxes = {'two': [(100, 270, 140, 340), (250, 270, 290, 340)],
             'scattered': [(100, 210, 140, 250), (250, 300, 290, 350), (400, 390, 440, 440)],
             'clipped_tips': [(x, 467, x+40, 479) for x in (100, 250, 400)],
             'long_lane_edges': [(100, 200, 110, 460), (350, 200, 360, 460), (550, 200, 560, 460)]}[layout]
    for box in boxes:
        cv2.rectangle(image, box[:2], box[2:], (255, 255, 255), -1)
    assert not any(c.label == 'crosswalk' for c in ClassicRaceDetector().detect(image))


@pytest.mark.parametrize('name,expected', [('crosswalk-papers-secondary.jpg', True),
                                          ('crosswalk-background-primary.jpg', False)])
def test_real_crosswalk_papers_and_background(name, expected):
    from pathlib import Path
    image = cv2.imdecode(np.frombuffer((Path(__file__).parent/'fixtures'/name).read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
    assert image is not None
    assert any(c.label == 'crosswalk' for c in ClassicRaceDetector().detect(image)) == expected
