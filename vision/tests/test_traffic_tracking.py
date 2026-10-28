"""Traffic fixture identity and fake-cache integration; no cameras or outputs."""
import copy
import threading
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.traffic_tracking import FixtureTrack, TrafficApproachGuard
from carvision.traffic_trial import TrafficCamera


DT = 1 / 15
BOX = [100, 80, 250, 150]
FAR = [310, 220, 460, 270]
SETTINGS = {'cameras': ['primary', 'secondary'], 'camera': 'dual',
            'motion_camera': 'secondary', 'image_size': [480, 360],
            'max_frame_age_s': .25, 'fast_red_max_age_s': .15}


def signal(color='off', box=None, reason=None):
    return {'state': color, 'signal_for_race': color,
            'fixture_detected': color != 'unknown',
            'bbox_xyxy': list(BOX if box is None else box) if color != 'unknown' else None,
            'reason': reason or ('fixture_present_without_emission' if color == 'off' else
                                'no_supported_fixture' if color == 'unknown' else
                                'position_and_emission_agree')}


def row(camera, fid, captured, color='off', box=None):
    return {'camera_id': camera, 'frame_id': fid, 'captured_s': captured,
            'image_size': [480, 360], 'signal': signal(color, box)}


def locked_guard():
    guard = TrafficApproachGuard(SETTINGS)
    guard.update({'primary': row('primary', 0, 0)}, 0, approaching=True)
    _, info = guard.update({'primary': row('primary', 1, DT)}, DT, approaching=True)
    assert info['cameras']['primary']['locked']
    return guard


def test_lock_requires_two_distinct_continuous_matching_frames():
    track = FixtureTrack()
    _, first = track.update(signal(), 0, 0)
    assert not first['locked']
    _, duplicate = track.update(signal(), 0, 0)
    assert not duplicate['locked'] and duplicate['acquisition_frames'] == 1
    _, second = track.update(signal(), 1, DT)
    assert second['locked'] and second['bbox_xyxy'] == BOX


@pytest.mark.parametrize('interruption', ['unknown', 'far_fixture', 'time_gap'])
def test_acquisition_cannot_pool_frames_across_interruption(interruption):
    track = FixtureTrack()
    track.update(signal(), 0, 0)
    if interruption == 'time_gap':
        _, result = track.update(signal(), 1, .3)
    elif interruption == 'far_fixture':
        _, result = track.update(signal(box=FAR), 1, DT)
    else:
        track.update(signal('unknown'), 1, DT)
        _, result = track.update(signal(), 2, 2 * DT)
    assert not result['locked'] and result['acquisition_frames'] == 1


def test_ambiguous_multiple_fixture_box_never_acquires_a_lock():
    track = FixtureTrack()
    ambiguous = signal(reason='multiple_fixture_candidates')
    ambiguous.update(state='unknown', signal_for_race='unknown')
    for fid in range(4):
        _, info = track.update(ambiguous, fid, fid * DT)
        assert not info['locked'] and info['acquisition_frames'] == 0


@pytest.mark.parametrize('replacement_color', ['off', 'green'])
def test_locked_track_rejects_remote_fixture_without_automatic_reacquisition(replacement_color):
    track = FixtureTrack()
    track.update(signal(), 0, 0)
    track.update(signal(), 1, DT)
    for fid in range(2, 7):
        result, info = track.update(signal(replacement_color, FAR), fid, fid * DT)
        assert result['state'] == 'unknown' and not result['fixture_detected']
        assert result['reason'] == 'tracked_fixture_missing'
        assert info['locked'] and info['bbox_xyxy'] == BOX
        assert info['last_seen_s'] == DT


def test_camera_acquisition_counts_are_independent():
    guard = TrafficApproachGuard(SETTINGS)
    guard.update({'primary': row('primary', 0, 0)}, 0, approaching=True)
    _, info = guard.update({'primary': row('primary', 1, DT, 'unknown'),
                           'secondary': row('secondary', 0, DT)}, DT, approaching=True)
    assert not info['any_locked']
    _, info = guard.update({'primary': row('primary', 2, 2 * DT),
                           'secondary': row('secondary', 1, 2 * DT)}, 2 * DT, approaching=True)
    assert not info['cameras']['primary']['locked']
    assert info['cameras']['secondary']['locked']


def test_first_trustworthy_red_is_stop_only_even_before_identity_lock():
    guard = TrafficApproachGuard(SETTINGS)
    filtered, info = guard.update({'secondary': row('secondary', 4, .1, 'red')},
                                  .12, approaching=True)
    assert info['stop_reason'] == 'first_trustworthy_red'
    assert info['sources'] == ['secondary'] and not info['any_locked']
    assert info['first_red'] == {'camera': 'secondary', 'frame_id': 4,
                                 'captured_s': .1, 'decision_s': .12}
    assert filtered['secondary']['signal']['state'] == 'unknown'
    assert info['current_stop_color_sources'] == ['secondary']
    assert 'red_seen_this_round' not in info


def test_multiple_fixture_or_stale_red_never_becomes_trustworthy_first_red():
    for kind in ('ambiguous', 'stale'):
        guard = TrafficApproachGuard(SETTINGS)
        observation = row('primary', 0, 0, 'red')
        if kind == 'ambiguous':
            observation['signal']['reason'] = 'multiple_fixture_candidates'
        _, info = guard.update({'primary': observation}, .01 if kind == 'ambiguous' else .16,
                               approaching=True)
        assert info['first_red'] is None
        assert info['stop_reason'] != 'first_trustworthy_red'


@pytest.mark.parametrize('loss', ['unknown', 'missing_camera', 'cached'])
def test_unlit_fixture_loss_is_not_a_gate_or_a_stop_request(loss):
    guard = locked_guard()
    now = DT + .151
    observations = {'secondary': row('secondary', 2, now, 'green', FAR)}
    if loss == 'unknown':
        observations['primary'] = row('primary', 2, now, 'unknown')
    elif loss == 'cached':
        observations['primary'] = row('primary', 1, DT)
    filtered, info = guard.update(observations, now, approaching=True)
    assert info['stop_reason'] is None and info['sources'] == []
    assert info['cameras']['primary']['bbox_xyxy'] == BOX
    assert info['first_red'] is None
    assert filtered['secondary']['signal']['state'] == 'unknown'
    assert not info['cameras']['secondary']['locked']


def test_brief_loss_within_budget_does_not_immediately_stop():
    guard = locked_guard()
    _, info = guard.update({'primary': row('primary', 2, 2 * DT, 'unknown')},
                           2 * DT, approaching=True)
    assert info['stop_reason'] is None


def test_unlit_fixture_moving_towards_image_edge_does_not_request_a_stop():
    guard = TrafficApproachGuard(SETTINGS)
    for fid, left in enumerate((100, 90, 80, 70)):
        _, info = guard.update({'primary': row('primary', fid, fid * DT,
                                              box=[left, 80, left + 150, 150])},
                               fid * DT, approaching=True)
        assert info['stop_reason'] is None
    assert info['cameras']['primary']['locked']


def test_small_alternating_box_jitter_does_not_predict_an_edge_exit():
    guard = TrafficApproachGuard(SETTINGS)
    for fid, left in enumerate((100, 99, 100, 99, 100, 99, 100, 99)):
        _, info = guard.update({'primary': row('primary', fid, fid * DT,
                                              box=[left, 80, left + 150, 150])},
                               fid * DT, approaching=True)
        assert info['stop_reason'] is None
    assert info['cameras']['primary']['locked']


def test_unlit_fixture_at_image_edge_does_not_infer_gate_crossing_or_stop():
    guard = TrafficApproachGuard(SETTINGS)
    box = [8, 80, 158, 150]
    guard.update({'primary': row('primary', 0, 0, box=box)}, 0, approaching=True)
    _, info = guard.update({'primary': row('primary', 1, DT, box=box)}, DT, approaching=True)
    assert info['cameras']['primary']['locked'] and info['stop_reason'] is None
    assert not info['cameras']['primary']['metric_distance_verified']


@pytest.mark.parametrize('color', ['red', 'yellow'])
@pytest.mark.parametrize('partial', [[0, 65, 230, 150], [75, 65, 305, 150]])
def test_same_fixture_shape_change_retains_current_red_or_yellow_as_stop_only(color, partial):
    guard = locked_guard()
    now = 2 * DT
    # The same fixture still overlaps, but clipping/scale changes exceed the
    # strict formal identity envelope. Only its current red/yellow photometry
    # can stop the car; neither shape change nor the image edge supplies color.
    filtered, info = guard.update({'primary': row('primary', 2, now, color, partial)},
                                  now, approaching=True)
    assert info['stop_reason'] == 'first_trustworthy_' + color
    assert info['sources'] == ['primary']
    assert filtered['primary']['signal']['state'] == 'unknown'
    assert not filtered['primary']['signal']['fixture_detected']
    assert info['cameras']['primary']['bbox_xyxy'] == BOX
    assert info['current_stop_color_sources'] == ['primary']
    assert (info['first_red'] is not None) == (color == 'red')


@pytest.mark.parametrize('color', ['off', 'green'])
def test_partial_edge_without_red_or_yellow_does_not_stop_or_relax_formal_votes(color):
    guard = locked_guard()
    filtered, info = guard.update({'primary': row('primary', 2, 2 * DT, color,
                                                 [0, 65, 230, 150])},
                                  2 * DT, approaching=True)
    assert info['stop_reason'] is None and info['current_stop_color_sources'] == []
    assert filtered['primary']['signal']['state'] == 'unknown'
    assert not filtered['primary']['signal']['fixture_detected']
    assert info['cameras']['primary']['bbox_xyxy'] == BOX


@pytest.mark.parametrize('camera', ['primary', 'secondary'])
def test_stop_only_metadata_cannot_supply_a_remote_or_unowned_fixture(camera):
    guard = locked_guard()
    observation = row(camera, 2, 2 * DT, 'unknown')
    observation['signal'].update(stop_only_color='red',
                                 stop_only_box=FAR if camera=='primary' else BOX)
    _, info = guard.update({camera: observation}, 2 * DT, approaching=True)
    assert info['stop_reason'] is None and info['current_stop_color_sources'] == []
    assert info['first_red'] is None


@pytest.mark.parametrize('color', ['red', 'yellow'])
def test_remote_current_stop_color_cannot_replace_the_locked_fixture(color):
    guard = locked_guard()
    filtered, info = guard.update({'primary': row('primary', 2, 2 * DT, color, FAR)},
                                  2 * DT, approaching=True)
    assert info['stop_reason'] is None and info['current_stop_color_sources'] == []
    assert filtered['primary']['signal']['state'] == 'unknown'
    assert info['cameras']['primary']['bbox_xyxy'] == BOX


class Cache:
    def __init__(self, jpeg):
        self.condition = threading.Condition()
        self.state = 'ok'
        self.raw_jpeg = jpeg
        self.publish(0, 0.)

    def publish(self, fid, captured):
        self.raw_frame_id, self.raw_received_ms = fid, captured * 1000


class ScriptDetector:
    def __init__(self, responses):
        self.responses, self.calls = iter(responses), []

    def detect(self, image, *, reference_box=None):
        assert image.shape == (360, 480, 3)
        self.calls.append(copy.deepcopy(reference_box))
        return copy.deepcopy(next(self.responses))


def fake_camera(primary_responses, secondary_responses):
    success, jpeg = cv2.imencode('.jpg', np.zeros((360, 480, 3), np.uint8))
    assert success
    states = {camera: Cache(jpeg.tobytes()) for camera in ('primary', 'secondary')}
    now = [0.]
    reader = TrafficCamera(states, copy.deepcopy(SETTINGS), {}, clock=lambda: now[0])
    reader.detectors = {'primary': ScriptDetector(primary_responses),
                        'secondary': ScriptDetector(secondary_responses)}
    reader.motion = SimpleNamespace(update=lambda *_: {'valid': True, 'moving': False})
    return reader, states, now


def publish_read(reader, states, now, fid, captured):
    now[0] = captured
    for state in states.values():state.publish(fid, captured)
    result = reader.read(captured)
    assert result is not None
    return result


def test_camera_passes_reference_keyword_only_after_independent_two_frame_lock():
    reader, states, now = fake_camera([signal()] * 3, [signal('unknown')] * 3)
    first = publish_read(reader, states, now, 0, 0)
    assert not first['camera_observations']['primary']['fixture_track']['locked']
    second = publish_read(reader, states, now, 1, DT)
    assert second['camera_observations']['primary']['fixture_track']['locked']
    publish_read(reader, states, now, 2, 2 * DT)
    assert reader.detectors['primary'].calls == [None, None, BOX]
    assert reader.detectors['secondary'].calls == [None, None, None]


@pytest.mark.parametrize('replacement_color', ['off', 'green'])
def test_camera_filters_even_a_detector_that_ignores_its_reference(replacement_color):
    reader, states, now = fake_camera([signal(), signal(), signal(replacement_color, FAR)],
                                      [signal('unknown')] * 3)
    for fid in range(3):sample = publish_read(reader, states, now, fid, fid * DT)
    primary = sample['camera_observations']['primary']
    assert reader.detectors['primary'].calls[-1] == BOX
    assert primary['signal']['state'] == 'unknown'
    assert primary['signal']['reason'] == 'tracked_fixture_missing'
    assert primary['fixture_track']['bbox_xyxy'] == BOX


def test_camera_missing_unlit_source_does_not_supply_a_gate_stop():
    reader, states, now = fake_camera([signal()] * 2, [signal('unknown')] * 3)
    guard = TrafficApproachGuard(SETTINGS)
    for fid in range(2):
        sample = publish_read(reader, states, now, fid, fid * DT)
        guard.update(sample['camera_observations'], now[0], approaching=True)
    states['primary'].state = 'stopped'
    sample = publish_read(reader, states, now, 2, DT + .151)
    assert 'primary' not in sample['camera_observations']
    _, info = guard.update(sample['camera_observations'], now[0], approaching=True)
    assert info['stop_reason'] is None


def test_camera_same_fixture_red_shape_change_reaches_guard_as_stop_only():
    reader, states, now = fake_camera([signal(), signal(), signal('red', [0, 65, 230, 150])],
                                      [signal('unknown')] * 3)
    guard = TrafficApproachGuard(SETTINGS)
    for fid in range(3):
        sample = publish_read(reader, states, now, fid, fid * DT)
        filtered, info = guard.update(sample['camera_observations'], now[0], approaching=True)
    assert reader.detectors['primary'].calls[-1] == BOX
    assert sample['camera_observations']['primary']['signal']['stop_only_color'] == 'red'
    assert info['stop_reason'] == 'first_trustworthy_red'
    assert filtered['primary']['signal']['state'] == 'unknown'
    assert info['first_red']['camera'] == 'primary'


def test_begin_round_drops_old_lock_and_reclassifies_cached_pixels_without_old_reference():
    reader, states, now = fake_camera([signal(), signal(), signal('green', FAR), signal('green', FAR)],
                                      [signal('unknown')] * 4)
    for fid in range(2):publish_read(reader, states, now, fid, fid * DT)
    assert reader.tracks['primary'].box == BOX
    reader.begin_round()
    assert not reader.observations and reader.tracks['primary'].box is None
    # No new acquisition is opened: the same cached frame is deliberately
    # classified once as the first observation of a new explicit round.
    first = reader.read(now[0])
    assert first['camera_observations']['primary']['signal']['state'] == 'green'
    assert reader.detectors['primary'].calls[-1] is None
    assert not first['camera_observations']['primary']['fixture_track']['locked']
    second = publish_read(reader, states, now, 2, 2 * DT)
    assert second['camera_observations']['primary']['fixture_track']['bbox_xyxy'] == FAR


def test_new_guard_for_new_round_has_no_old_stop_latch_or_fixture():
    old = TrafficApproachGuard(SETTINGS)
    old.update({'primary': row('primary', 0, 0, 'red')}, 0, approaching=True)
    assert old.stop_reason == 'first_trustworthy_red'
    new = TrafficApproachGuard(SETTINGS)
    _, info = new.update({'primary': row('primary', 1, DT)}, DT, approaching=True)
    assert info['stop_reason'] is None and not info['any_locked'] and info['first_red'] is None

