"""Pixel and fresh-frame regressions for the team's 50% rectangular board rule.

All images are generated pixels, not labels standing in for image detection.
The 14% scene reproduces the scale of blue background seen in earlier logs;
it is a synthetic regression scene, not a claimed extracted camera frame.
"""
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.departure import (DepartureSequence, detect_board, validate_blue_percent,
    DEFAULT_MIN_BLUE_PERCENT, DEFAULT_CLEAR_CONFIRM_S)


HEIGHT, WIDTH = 360, 480
BLUE = (255, 0, 0)


def scene():
    """Healthy neutral texture with zero blue pixels and deterministic results."""
    rng = np.random.default_rng(20261022)
    gray = rng.integers(65, 185, (HEIGHT, WIDTH, 1), dtype=np.uint8)
    return np.repeat(gray, 3, axis=2)


def rectangle(width, height):
    image = scene()
    x, y = (WIDTH - width) // 2, (HEIGHT - height) // 2
    image[y:y + height, x:x + width] = BLUE
    return image


def actual_blue_fraction(image):
    mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (90, 90, 50), (135, 255, 255))
    return cv2.countNonZero(mask) / (HEIGHT * WIDTH)


def observations(t, frame_id, primary, secondary=None):
    secondary = detect_board(scene(), min_blue_percent=50) if secondary is None else secondary
    return {
        camera: {**result, 'camera_id': camera, 'frame_id': frame_id, 'captured_s': t}
        for camera, result in (('primary', primary), ('secondary', secondary))
    }


def confirmed_sequence():
    sequence = DepartureSequence(clear_s=.5)
    board = detect_board(rectangle(370, 250), min_blue_percent=50)
    assert board['state'] == 'present'
    for frame_id, t in enumerate((-.5, -.25, 0.)):
        assert not sequence.update(observations(t, frame_id, board), t)
    assert sequence.phase == 'wait_removed'
    return sequence


def test_current_defaults_match_saved_profile_without_inventing_metric_distance():
    profile=json.loads((Path(__file__).resolve().parents[1]/'tests/fixtures/configs/departure-field-20261026.json').read_text(encoding='utf-8'))
    result = detect_board(rectangle(360, 240))
    assert result['min_blue_percent'] == DEFAULT_MIN_BLUE_PERCENT == profile['min_blue_percent']
    assert DepartureSequence().clear_s == DEFAULT_CLEAR_CONFIRM_S == profile['clear_confirm_s']
    assert result['state'] == 'present'
    assert result['dimensions_cm'] == {'width': 40, 'height': 50}
    assert result['metric_distance_available'] is False
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('width,height,fraction,present', [
    (336, 252, .49, False),
    (360, 240, .50, True),
    (370, 250, 92500 / 172800, True),
])
def test_single_rectangle_obeys_exact_pixel_area_boundary(width, height, fraction, present):
    image = rectangle(width, height)
    assert actual_blue_fraction(image) == pytest.approx(fraction)
    result = detect_board(image, min_blue_percent=50)
    assert (result['state'] == 'present') is present
    assert result['blue_pixel_fraction'] == pytest.approx(fraction)


def test_large_rotated_rectangle_is_still_a_board():
    image = scene()
    corners = np.rint(cv2.boxPoints(((240., 180.), (350., 275.), 8.))).astype(np.int32)
    assert np.all(corners[:, 0] > 0) and np.all(corners[:, 0] < WIDTH - 1)
    assert np.all(corners[:, 1] > 0) and np.all(corners[:, 1] < HEIGHT - 1)
    cv2.fillConvexPoly(image, corners, BLUE)
    assert actual_blue_fraction(image) > .5
    assert detect_board(image, min_blue_percent=50)['state'] == 'present'


@pytest.mark.parametrize('shape', ['circle', 'ellipse', 'notched_rectangle'])
def test_large_nonrectangular_blue_objects_are_not_boards(shape):
    image = scene()
    if shape == 'circle':
        cv2.circle(image, (240, 180), 170, BLUE, -1)
    elif shape == 'ellipse':
        cv2.ellipse(image, (240, 180), (225, 160), 0, 0, 360, BLUE, -1)
    else:
        # A large L shape with a substantial notch, not a perspective rectangle.
        polygon = np.array([[30, 30], [280, 30], [280, 150],
                            [450, 150], [450, 330], [30, 330]], np.int32)
        cv2.fillPoly(image, [polygon], BLUE)
    assert actual_blue_fraction(image) > .5
    assert detect_board(image, min_blue_percent=50)['state'] != 'present'


def test_hollow_blue_frame_does_not_count_nonblue_interior_as_board_area():
    image = scene()
    image[25:335, 25:455] = BLUE
    image[45:315, 45:435] = scene()[45:315, 45:435]
    assert (430 * 310) / (WIDTH * HEIGHT) > .5
    assert actual_blue_fraction(image) < .2
    result = detect_board(image, min_blue_percent=50)
    assert result['state'] != 'present'
    assert result['blue_pixel_fraction'] == pytest.approx(actual_blue_fraction(image))


def test_separate_small_rectangles_are_not_combined_into_one_fifty_percent_board():
    image = scene()
    for x in (20, 260):
        for y in (35, 195):
            image[y:y + 130, x:x + 200] = BLUE
    assert actual_blue_fraction(image) > .5
    assert (200 * 130) / (WIDTH * HEIGHT) < .5
    assert detect_board(image, min_blue_percent=50)['state'] != 'present'


def test_two_pixel_gap_does_not_merge_two_subthreshold_blue_boards():
    image=scene()
    image[30:330,3:238]=BLUE
    image[30:330,240:475]=BLUE
    assert actual_blue_fraction(image)>.8
    result=detect_board(image,min_blue_percent=50)
    assert result['state']!='present'
    assert result['blue_pixel_fraction']<.5


@pytest.mark.parametrize('value', [.5, 3, 19.53125, 49.999, 50, 50.5, 75, 90, 90.001, 100])
def test_supported_threshold_range_retains_current_profile_and_explicit_fifty(value):
    assert validate_blue_percent(value) == float(value)


@pytest.mark.parametrize('value', [True, False, None, 0, .4999, 100.001, -1,
                                 float('nan'), float('inf'), '50'])
def test_invalid_area_thresholds_still_rejected(value):
    with pytest.raises(ValueError):
        validate_blue_percent(value)


def test_higher_valid_threshold_remains_effective():
    image = rectangle(370, 250)
    assert .50 < actual_blue_fraction(image) < .60
    assert detect_board(image, min_blue_percent=50)['state'] == 'present'
    assert detect_board(image, min_blue_percent=60)['state'] != 'present'


def test_never_seen_fifty_percent_board_cannot_depart_from_empty_or_small_blue_scenes():
    sequence = DepartureSequence(clear_s=.5)
    small = detect_board(rectangle(240, 100), min_blue_percent=50)  # 13.89%, like earlier background scale.
    assert small['state'] == 'unknown'  # Between explicit 50% presence and actual clear threshold.
    for frame_id in range(25):
        t = frame_id * .1
        assert not sequence.update(observations(t, frame_id, small), t)
    assert sequence.phase == 'wait_board'


def test_fourteen_percent_blue_background_does_not_supply_removal_evidence():
    sequence = confirmed_sequence()
    background = rectangle(240, 100)
    assert actual_blue_fraction(background) == pytest.approx(.1388888889)
    ambiguous = detect_board(background, min_blue_percent=50)
    assert ambiguous['state'] == 'unknown'
    for frame_id, t in enumerate((.3, .55, .8, .9), 3):
        assert not sequence.update(observations(t, frame_id, ambiguous), t)
    assert sequence.phase == 'wait_removed' and sequence.clear_frames == 0
    clear = detect_board(scene(), min_blue_percent=50)
    for frame_id, t in enumerate((1., 1.2, 1.4), 7):
        assert not sequence.update(observations(t, frame_id, clear), t)
    assert sequence.update(observations(1.6, 10, clear), 1.6)
    assert not sequence.update(observations(1.7, 11, clear), 1.7)


def test_explicit_half_second_removal_still_requires_distinct_frames():
    sequence = confirmed_sequence()
    assert sequence.clear_s == .5
    clear = detect_board(scene(), min_blue_percent=50)
    assert not sequence.update(observations(.3, 3, clear), .3)
    assert not sequence.update(observations(.55, 4, clear), .55)
    assert not sequence.update(observations(.79, 5, clear), .79)
    assert sequence.phase == 'wait_removed'
    assert sequence.update(observations(.8, 6, clear), .8)


def test_two_clear_frames_cannot_depart_even_if_half_second_has_elapsed():
    sequence = confirmed_sequence()
    clear = detect_board(scene(), min_blue_percent=50)
    assert not sequence.update(observations(.3, 3, clear), .3)
    assert not sequence.update(observations(.8, 4, clear), .8)
    assert sequence.phase == 'wait_removed'
    assert not sequence.update(observations(.81, 5, clear), .81)
    assert sequence.update(observations(.82, 6, clear), .82)


def test_many_clear_frames_cannot_depart_before_half_second():
    sequence = confirmed_sequence()
    clear = detect_board(scene(), min_blue_percent=50)
    for frame_id, t in enumerate((.3, .4, .5, .6, .79), 3):
        assert not sequence.update(observations(t, frame_id, clear), t)
    assert sequence.phase == 'wait_removed'
    assert sequence.update(observations(.8, 8, clear), .8)


def test_repeated_cached_absent_frame_cannot_supply_three_removal_observations():
    sequence = confirmed_sequence()
    cached = observations(.3, 3, detect_board(scene(), min_blue_percent=50))
    for t in (.3, .4, .5, .6, .7, .8):
        assert not sequence.update(cached, t)
    assert sequence.phase != 'complete'


@pytest.mark.parametrize('failure', ['black', 'stale', 'missing'])
def test_unavailable_tracked_camera_is_never_interpreted_as_removal(failure):
    sequence = confirmed_sequence()
    clear = detect_board(scene(), min_blue_percent=50)
    rows = observations(.3, 3, clear)
    if failure == 'black':
        black = detect_board(np.zeros((HEIGHT, WIDTH, 3), np.uint8), min_blue_percent=50)
        assert black['state'] == 'unknown'
        rows['primary'].update(black)
    elif failure == 'stale':
        rows['primary']['captured_s'] = -1.
    else:
        rows.pop('primary')
    assert not sequence.update(rows, .3)
    assert sequence.phase != 'complete'
    if failure == 'black':
        # Current quality recovery preserves prior confirmation for <1 s;
        # it never counts black as clear. Persistent black then invalidates.
        assert sequence.phase == 'wait_removed' and sequence.clear_frames == 0
        for frame_id in range(4, 16):
            t = round(.4 + (frame_id - 4) * .1, 8)
            bad = observations(t, frame_id, clear)
            bad['primary'].update(black)
            assert not sequence.update(bad, t)
        assert sequence.phase == 'wait_board'
        origin, first_id = 1.6, 16
    else:
        assert sequence.phase == 'wait_board'
        origin, first_id = .4, 4
    # Empty frames after invalidation cannot stand in for a qualifying board.
    for frame_id in range(first_id, first_id+12):
        t = origin + (frame_id-first_id)*.1
        assert not sequence.update(observations(t, frame_id, clear), t)
    assert sequence.phase == 'wait_board'


def test_board_reappearance_restarts_clear_confirmation():
    sequence = confirmed_sequence()
    clear = detect_board(scene(), min_blue_percent=50)
    board = detect_board(rectangle(370, 250), min_blue_percent=50)
    assert not sequence.update(observations(.3, 3, clear), .3)
    assert not sequence.update(observations(.55, 4, clear), .55)
    assert not sequence.update(observations(.7, 5, board), .7)
    assert not sequence.update(observations(.8, 6, clear), .8)
    assert not sequence.update(observations(1.05, 7, clear), 1.05)
    assert not sequence.update(observations(1.3, 8, clear), 1.3)
    assert sequence.update(observations(1.31, 9, clear), 1.31)


def test_perspective_solid_quad_with_small_edge_noise_keeps_real_pixel_area():
    image=scene()
    quad=np.array([[80,45],[390,70],[365,330],[110,310]],np.int32)
    cv2.fillConvexPoly(image,quad,BLUE)
    # Small edge defects may be smoothed for geometry, not counted as blue.
    image[130:133,90:94]=scene()[130:133,90:94]
    result=detect_board(image,min_blue_percent=30)
    assert result['state']=='present' and result['board_candidates']
    assert result['blue_pixel_fraction']<=actual_blue_fraction(image)


@pytest.mark.parametrize('name', ['blue-board-close.jpg', 'parking-border-clipped-left-board-20261013.jpg'])
def test_real_solid_blue_board_fixture_is_still_a_valid_detection(name):
    source=Path(__file__).parent/'fixtures'/name
    image=cv2.imdecode(np.frombuffer(source.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    result=detect_board(image,min_blue_percent=3)
    assert result['state']=='present' and result['board_candidates']
    assert result['detector_revision']=='solid_quadrilateral_pixels_v1'


@pytest.mark.parametrize('name', ['cone-removed-primary-a.jpg', 'cone-removed-secondary-a.jpg'])
def test_real_blue_background_cannot_arm_or_trigger_departure(name):
    source=Path(__file__).parent/'fixtures'/name
    image=cv2.imdecode(np.frombuffer(source.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    background=detect_board(image)
    assert background['state']!='present'
    sequence=DepartureSequence()
    for frame_id in range(15):
        t=frame_id*.1
        assert not sequence.update(observations(t,frame_id,background),t)
    for frame_id in range(15,30):
        t=frame_id*.1
        assert not sequence.update(observations(t,frame_id,detect_board(scene())),t)
    assert sequence.phase=='wait_board'
