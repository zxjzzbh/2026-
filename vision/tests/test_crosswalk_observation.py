"""Cached-camera and real-pixel tests; no camera/UART/network is opened."""
import copy
import json
import os
from pathlib import Path
import threading
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.crosswalk_calibration import DISTANCE_REFERENCE, IMAGE_FEATURE_NEAR
from carvision.crosswalk_observation import CalibratedTrialCamera


def background():
    gray = np.random.default_rng(24).integers(55, 120, (360, 480, 1), dtype=np.uint8)
    return gray.repeat(3, axis=2)


def paper_row(bottom=245, *, height=30, bottoms=None, white=245):
    image = background()
    for index, x in enumerate((100, 180, 260)):
        y = bottom if bottoms is None else bottoms[index]
        image[y-height+1:y+1, x:x+35] = white
    return image


def vertical_papers():
    image = background()
    for bottom in (195, 250, 315):
        image[bottom-19:bottom+1, 240:280] = 245
    return image


def publish(state, image, frame_id, stamp):
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    state.raw_jpeg = jpeg.tobytes()
    state.raw_frame_id, state.raw_received_ms = frame_id, stamp * 1000


@pytest.fixture
def settings():
    cameras = {}
    for camera, bottoms in [('primary', (180, 240, 300)), ('secondary', (175, 230, 290))]:
        cameras[camera] = {
            'identity': camera + ' camera | synthetic-' + camera,
            'image_size': [480, 360], 'captured_on': '2026-10-24', 'verified': True,
            'pose': {'fixed': True, 'verified': True, 'description': 'synthetic fixed test pose'},
            'references': [{'distance_m': distance, 'near_edge_y_normalized': bottom / 359,
                            'measured': True, 'evidence': 'synthetic-pixels-only'}
                           for distance, bottom in zip((.65, .5, .4), bottoms)],
        }
    return {'max_frame_age_s': .25, 'crosswalk_distance_trigger_m': .4,
            'crosswalk_calibration': {
                'schema_version': 1, 'profile_id': 'synthetic-observer-tests',
                'enabled': True, 'verified': True, 'physical_stop_verified': False,
                'distance_reference': DISTANCE_REFERENCE, 'image_feature': IMAGE_FEATURE_NEAR,
                'cameras': cameras,
                'overlap': {'verified': True, 'reference_distance_m': .5,
                            'evidence': 'synthetic-independent-two-camera-pair'},
            }}


@pytest.fixture
def states():
    result = {}
    for camera, bottom, frame in [('primary', 245, 10000), ('secondary', 235, 4)]:
        state = SimpleNamespace(condition=threading.RLock(), state='ok',
                                identity={'name': camera + ' camera', 'bus': 'synthetic-' + camera})
        publish(state, paper_row(bottom), frame, 10.)
        result[camera] = state
    return result


def advance(states, stamp, primary=245, secondary=235):
    for camera, bottom in [('primary', primary), ('secondary', secondary)]:
        image = background() if bottom is None else paper_row(bottom)
        publish(states[camera], image, states[camera].raw_frame_id + 1, stamp)


def lock_primary(settings, states):
    reader = CalibratedTrialCamera(states, settings)
    assert reader.read(10.)['source_camera'] == 'secondary'
    advance(states, 10.1)
    result = reader.read(10.1)
    assert result['source_camera'] == 'primary' and result['source_changed'] is True
    return reader


def test_first_batch_retains_each_camera_identity_native_id_and_time(settings, states):
    reader = CalibratedTrialCamera(states, settings)
    result = reader.read(10.)
    assert result['fresh'] and result['frame_id'] == 1 and result['raw_frame_id'] == 4
    assert result['source_camera'] == 'secondary' and not result['source_changed']
    assert result['captured_s'] == 10. and result['calibration_id'] == 'synthetic-observer-tests'
    assert result['camera_observations']['primary']['frame_id'] == 10000
    assert result['camera_observations']['secondary']['frame_id'] == 4
    assert reader.jpeg == states['secondary'].raw_jpeg
    assert result['candidate'] and result['metric_valid'] and result['distance_status'] == 'inside'
    assert result['physical_stop_verified'] is False
    json.dumps(result, allow_nan=False)


def test_near_edge_is_median_bottom_not_detector_maximum_bottom(settings, states):
    publish(states['secondary'], paper_row(height=40, bottoms=(240, 250, 260)), 5, 10.)
    result = CalibratedTrialCamera(states, settings).read(10.)
    strips = result['stripes_xywh']
    expected = float(np.median([y+h-1 for x, y, w, h in strips])) / 359
    assert len(strips) >= 3
    assert result['near_edge_y_normalized'] == expected
    assert expected < max(y+h-1 for x, y, w, h in strips) / 359


def test_white_thresholds_are_independent_for_each_fixed_view(settings, states):
    for camera in states:
        publish(states[camera], paper_row(245, white=185), states[camera].raw_frame_id + 1, 10.)
    reader = CalibratedTrialCamera(states, settings)
    result = reader.read(10.)
    assert reader.detectors['primary'].config.white_v_min == 200
    assert reader.detectors['secondary'].config.white_v_min == 170
    assert not result['camera_observations']['primary']['candidate']
    assert result['camera_observations']['secondary']['candidate']


def test_three_depth_wise_calibration_papers_do_not_become_a_crosswalk(settings, states):
    for camera in states:
        publish(states[camera], vertical_papers(), states[camera].raw_frame_id + 1, 10.)
    result = CalibratedTrialCamera(states, settings).read(10.)
    assert not result['candidate'] and result['distance_estimate_m'] is None
    assert result['distance_status'] == 'not_detected'
    assert all(not row['candidate'] for row in result['camera_observations'].values())


def test_empty_scene_search_requires_new_active_camera_frame(settings, states):
    advance(states, 10., primary=None, secondary=None)
    reader = CalibratedTrialCamera(states, settings)
    first = reader.read(10.)
    assert first['source_camera'] == 'secondary' and not first['candidate']
    assert first['near_edge_y_normalized'] is None and first['distance_estimate_m'] is None
    assert reader.read(10.1) is None
    publish(states['primary'], background(), states['primary'].raw_frame_id + 1, 10.1)
    assert reader.read(10.1) is None  # New inactive view cannot renew far-view lease.
    publish(states['secondary'], background(), states['secondary'].raw_frame_id + 1, 10.15)
    second = reader.read(10.15)
    assert second['frame_id'] == first['frame_id'] + 1 and second['captured_s'] == 10.15


def test_normal_handoff_needs_two_distinct_primary_frames_in_overlap(settings, states):
    reader = CalibratedTrialCamera(states, settings)
    assert reader.read(10.)['source_camera'] == 'secondary'
    publish(states['secondary'], paper_row(235), 5, 10.05)
    assert reader.read(10.05)['source_camera'] == 'secondary'
    assert reader.confirmed_primary_frames == 1
    publish(states['primary'], paper_row(245), 10001, 10.1)
    result = reader.read(10.1)
    assert result['source_camera'] == 'primary' and result['source_changed']
    assert result['raw_frame_id'] == 10001 and result['frame_id'] == 3
    assert result['captured_s'] == 10.1
    assert reader.jpeg == states['primary'].raw_jpeg


def test_after_handoff_primary_is_locked_even_if_far_view_is_newer(settings, states):
    reader = lock_primary(settings, states)
    publish(states['secondary'], paper_row(235), 6, 10.2)
    assert reader.read(10.2) is None
    assert reader.primary_locked and reader.active_source == 'primary'
    publish(states['primary'], paper_row(245), 10002, 10.21)
    result = reader.read(10.21)
    assert result['source_camera'] == 'primary' and not result['source_changed']


def test_old_handoff_timestamp_is_not_relabelled_as_current(settings, states):
    publish(states['primary'], paper_row(245), 10000, 9.8)
    reader = CalibratedTrialCamera(states, settings)
    first = reader.read(10.)
    assert first['captured_s'] == 10.
    publish(states['primary'], paper_row(245), 10001, 9.9)
    publish(states['secondary'], paper_row(235), 5, 10.1)
    assert reader.read(10.1) is None  # Lock happens, but no backwards controller time.
    assert reader.primary_locked
    publish(states['primary'], paper_row(245), 10002, 10.12)
    result = reader.read(10.12)
    assert result['captured_s'] == 10.12 and result['source_changed']
    assert result['frame_id'] == first['frame_id'] + 1


@pytest.mark.parametrize('camera', ['primary', 'secondary'])
def test_any_frozen_view_expires_without_reusing_cached_frames(settings, states, camera):
    reader = lock_primary(settings, states)
    other = 'secondary' if camera == 'primary' else 'primary'
    publish(states[other], paper_row(235 if other == 'secondary' else 245),
            states[other].raw_frame_id + 1, 10.2)
    reader.read(10.2)
    publish(states[other], paper_row(235 if other == 'secondary' else 245),
            states[other].raw_frame_id + 1, 10.36)
    with pytest.raises(ValueError, match='失效'):
        reader.read(10.36)


@pytest.mark.parametrize('fault', ['identity', 'error', 'bad_jpeg', 'size', 'old_id', 'fake_time'])
def test_one_camera_fault_cannot_fall_back_to_the_other(settings, states, fault):
    reader = CalibratedTrialCamera(states, settings)
    reader.read(10.)
    primary = states['primary']
    if fault == 'identity':
        primary.identity['bus'] = 'different-physical-usb-camera'
    elif fault == 'error':
        primary.state = 'error'
    elif fault == 'bad_jpeg':
        primary.raw_jpeg = b'not jpeg'
        primary.raw_frame_id += 1
        primary.raw_received_ms = 10100
    elif fault == 'size':
        publish(primary, np.zeros((480, 640, 3), np.uint8), primary.raw_frame_id + 1, 10.1)
    elif fault == 'old_id':
        primary.raw_frame_id -= 1
    else:
        primary.raw_received_ms = 10100  # Same native ID cannot get a new time.
    with pytest.raises(ValueError):
        reader.read(10.1)


def test_distance_disagreement_above_five_cm_is_not_averaged(settings, states):
    publish(states['primary'], paper_row(190), 10001, 10.)
    publish(states['secondary'], paper_row(280), 5, 10.)
    result = CalibratedTrialCamera(states, settings).read(10.)
    assert result['source_camera'] == 'secondary'
    assert result['distance_disagreement']['difference_m'] > .05
    assert result['distance_estimate_m'] == result['camera_observations']['secondary']['distance_estimate_m']


@pytest.mark.parametrize('value', [0, 100, 255])
def test_flat_black_or_overexposed_image_cannot_authorize_search(settings, states, value):
    publish(states['primary'], np.full((360, 480, 3), value, np.uint8), 10001, 10.)
    with pytest.raises(ValueError, match='不能作为搜索依据'):
        CalibratedTrialCamera(states, settings).read(10.)


def test_interrupted_agreement_does_not_reuse_old_primary_confirmation(settings, states):
    reader = CalibratedTrialCamera(states, settings)
    reader.read(10.)
    assert reader.confirmed_primary_frames == 1
    advance(states, 10.05, primary=None, secondary=None)
    assert reader.read(10.05)['source_camera'] == 'secondary'
    assert reader.confirmed_primary_frames == 0
    advance(states, 10.1)
    assert reader.read(10.1)['source_camera'] == 'secondary'
    advance(states, 10.15)
    assert reader.read(10.15)['source_camera'] == 'primary'


def test_default_dual_trigger_comes_from_shared_calibration_not_legacy_twenty_five_cm(settings, states):
    settings.pop('crosswalk_distance_trigger_m')
    settings['brake_trigger_distance_m'] = .25
    reader = CalibratedTrialCamera(states, settings)
    assert reader.trigger == .4
    assert reader.settings['brake_trigger_distance_m'] == .25


@pytest.mark.parametrize('bottom', [330, 359])
def test_near_limit_danger_immediately_wins_over_far_view_without_extrapolation(settings, states, bottom):
    publish(states['primary'], paper_row(bottom), 10001, 10.)
    result = CalibratedTrialCamera(states, settings).read(10.)
    assert result['source_camera'] == 'primary'
    assert result['candidate'] and result['near_limit_reached']
    assert result['distance_status'] == 'too_near'
    assert result['metric_valid'] is False and result['distance_estimate_m'] is None
    if bottom == 359:
        assert result['bottom_clipped'] and result['near_edge_y_normalized'] is None
    else:
        assert result['near_edge_y_normalized'] > max(
            row['near_edge_y_normalized'] for row in settings['crosswalk_calibration']['cameras']['primary']['references'])


def test_primary_at_trigger_wins_immediately_when_far_camera_has_no_candidate(settings, states):
    publish(states['primary'], paper_row(300), 10001, 10.)
    publish(states['secondary'], background(), 5, 10.)
    reader = CalibratedTrialCamera(states, settings)
    result = reader.read(10.)
    assert result['source_camera'] == 'primary' and reader.primary_locked
    assert result['distance_estimate_m'] == pytest.approx(.4)


def test_locked_primary_does_not_ignore_secondary_near_limit_risk(settings, states):
    reader = lock_primary(settings, states)
    publish(states['secondary'], paper_row(330), 6, 10.2)
    result = reader.read(10.2)
    assert result['source_camera'] == 'secondary' and result['near_limit_reached']
    assert result['captured_s'] == 10.2 and reader.primary_locked


def test_beyond_far_candidate_is_presence_only_not_an_extrapolated_distance(settings, states):
    publish(states['primary'], paper_row(170, height=18), 10001, 10.)
    publish(states['secondary'], paper_row(165, height=18), 5, 10.)
    result = CalibratedTrialCamera(states, settings).read(10.)
    assert result['candidate'] and result['distance_status'] == 'beyond_far'
    assert not result['metric_valid'] and not result['near_limit_reached']
    assert result['distance_estimate_m'] is None


def test_each_view_has_independent_optical_motion_and_lane_state(settings, states):
    reader = CalibratedTrialCamera(states, settings)
    assert reader.motions['primary'] is not reader.motions['secondary']
    assert reader.lanes['primary'] is not reader.lanes['secondary']
    first = reader.read(10.)
    assert first['motion']['valid'] is False
    publish(states['primary'], paper_row(245), 10001, 10.1)
    result = reader.read(10.1)
    assert result['source_camera'] == 'primary'
    assert result['motion']['valid'] is True
    assert result['camera_observations']['secondary']['motion']['valid'] is False
    assert reader.motions['primary'].previous[2] == 10001
    assert reader.motions['secondary'].previous[2] == 4


def test_settings_snapshot_does_not_follow_later_profile_edits(settings, states):
    reader = CalibratedTrialCamera(states, settings)
    settings['crosswalk_calibration']['cameras']['primary']['identity'] = 'replacement'
    assert reader.read(10.)['candidate']


@pytest.mark.parametrize('change', ['disabled', 'far_feature', 'age', 'mask'])
def test_reader_rejects_incomplete_or_incompatible_activation(settings, states, change):
    if change == 'disabled':
        settings['crosswalk_calibration']['enabled'] = False
    elif change == 'far_feature':
        settings['crosswalk_calibration']['image_feature'] = 'unreviewed-edge'
    elif change == 'age':
        settings['max_frame_age_s'] = .501
    else:
        settings['crosswalk_calibration']['cameras']['primary']['white_mask'] = {'v_min': True}
    with pytest.raises(ValueError):
        CalibratedTrialCamera(states, settings)


def test_actual_fixed_scene_with_three_depth_markers_does_not_start_a_crosswalk(settings, states):
    default = Path(__file__).resolve().parents[3] / 'crosswalk-calibration-20261024/capture-final'
    source = Path(os.environ.get('CARVISION_CROSSWALK_STATIC_FIXTURES', default))
    paths = {camera: sorted(source.glob('00-' + camera + '-*.jpg')) for camera in states}
    if not all(paths.values()):
        pytest.skip('Original stationary 40/50/65 cm capture files are not installed in this environment')
    for camera in states:
        image = cv2.imdecode(np.frombuffer(paths[camera][0].read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        assert image is not None
        publish(states[camera], image, states[camera].raw_frame_id + 1, 10.)
    result = CalibratedTrialCamera(states, settings).read(10.)
    assert not result['candidate'] and result['distance_estimate_m'] is None
    assert all(not row['candidate'] for row in result['camera_observations'].values())
