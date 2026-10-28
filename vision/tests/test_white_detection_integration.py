"""Local saved-image/synthetic replay with real geometry and fake output only.

Repeated images below model independently delivered test frames, not a recorded
vehicle trajectory. No camera, serial port, service or network is opened.
"""
import copy
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_low_speed import DualLowSpeedNavigator, trial_settings
from carvision.cone_navigation import actuator_settings
from carvision.cone_planning import load_settings
from carvision.cone_road import ConeRoadContext
from carvision.cone_trial_output import ConePulseGuard
from carvision.cone_vision import BlueConeDetector, ImageConeTracker
from test_cone_turn_corridor import track


ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT/'vision/tests/fixtures'
FIELD_PAIRS = [('white-live-stability-20261026', i) for i in (0, 4, 8, 12, 16, 20, 24)] + [('white-live-dark-20261026', i) for i in range(3)]


def read_image(path):
    assert path.is_file(), f'local replay image missing: {path}'
    result = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    assert result is not None and result.shape == (360, 480, 3)
    return result


def replay_settings():
    return trial_settings(load_settings(ROOT/'vision/tests/fixtures/configs/cone-trial-field-20261026.json'))


class Pipeline:
    def __init__(self):
        self.s = replay_settings()
        self.detectors = {camera: BlueConeDetector(self.s) for camera in ('primary', 'secondary')}
        self.roads = {camera: ConeRoadContext(self.s, camera, turn_aware=True) for camera in self.detectors}
        self.trackers = {camera: ImageConeTracker(camera, self.s, near_field_recovery=True) for camera in self.detectors}
        self.navigator = DualLowSpeedNavigator(self.s)
        self.controller = ContinuousConeController(self.s, 0., initialized=True, handoff_after_two=True)
        self.writes = []
        self.guard = ConePulseGuard(actuator_settings(self.s), lambda *row: self.writes.append(row),
            mode='F/B', right_us=self.s['low_speed_test']['right_us'], left_us=self.s['low_speed_test']['left_us'])
        self.sequence = 0

    def frame(self, images, t, fid):
        rows = {}
        for camera, image in images.items():
            detections, obstacle = self.detectors[camera].detect(image)
            context = self.roads[camera].inspect(image, obstacle, frame_id=fid, captured_s=t)
            self.roads[camera].enrich(detections, image, context)
            tracks = self.trackers[camera].update(detections, fid, t)
            rows[camera] = {'frame_id': fid, 'captured_s': t, 'road_context': context, 'tracks': tracks}
        nav = self.navigator.update_views(images, rows, t)
        intent = self.controller.step(nav, t)
        if intent.get('fresh_control') is not False or intent['esc_us'] <= 1500:
            self.sequence += 1
            self.guard.accept({**intent, 'sequence': self.sequence, 'sent_s': t}, t)
        return rows, nav, intent


@pytest.mark.parametrize('prefix,index', FIELD_PAIRS,
                         ids=[f'{prefix}-{i}' for prefix, i in FIELD_PAIRS])
def test_field_white_tape_pairs_stabilize_through_real_detection_navigation_controller_and_guard(prefix, index):
    pipeline = Pipeline()
    images = {camera: read_image(FIXTURES/f'{prefix}-{camera}-{index}.jpg') for camera in ('primary', 'secondary')}
    recent = []
    for fid in range(1, 9):
        t = round((fid-1)*.1, 8)
        rows, nav, intent = pipeline.frame(images, t, fid)
        recent.append((rows, nav, intent))
        assert intent['physical_tail_verified'] is False and pipeline.controller.count == 0
        assert nav['frame_id'] == fid and nav['captured_s'] == t
        assert not pipeline.guard.fault
    for rows, nav, intent in recent[-4:]:
        assert rows['secondary']['road_context']['valid'], rows['secondary']['road_context']['boundary_diagnostics']
        assert nav['line_state'] == 'paired' and nav['boundary_observed'], nav
        assert nav['source_camera'] == 'secondary'
        assert nav.get('lane_bounds_px') and not nav.get('line_held') and not nav.get('line_missing')
        assert intent['phase'] != 'complete' and intent['esc_us'] > 1500
    assert pipeline.writes and any(channel == 4 and pulse > 1500 for channel, pulse in pipeline.writes)
    assert not any(channel == 4 and pulse > pipeline.s['driving']['forward_us'] for channel, pulse in pipeline.writes)


def nonroad(case, camera='secondary'):
    image = np.full((360, 480, 3), 60, np.uint8)
    if case == 'white_floor':
        image[:] = 230
    elif case == 'crossbar':
        cv2.line(image, (5, 220), (475, 220), (230, 230, 230), 10)
    elif case == 'wide_glare':
        image[:, :170] = 230
        image[:, 310:] = 230
    elif case == 'cabinet_only':
        # Actual room structures remain; current floor paint is removed.
        source = read_image(FIXTURES/f'white-live-stability-20261026-{camera}-0.jpg')
        image[:190] = source[:190]
    elif case == 'blue_cones_only':
        for x in (100, 380):
            cv2.fillPoly(image, [np.array([[x, 205], [x-25, 280], [x+25, 280]])], (220, 35, 20))
    return image


@pytest.mark.parametrize('case', ['empty', 'white_floor', 'crossbar', 'wide_glare', 'cabinet_only', 'blue_cones_only'])
@pytest.mark.parametrize('camera', ['primary', 'secondary'])
def test_nonroad_brightness_room_furniture_or_blue_cones_do_not_create_a_paired_corridor(case, camera):
    pipeline = Pipeline()
    image = nonroad(case, camera)
    for fid in range(1, 9):
        t = round((fid-1)*.1, 8)
        rows, nav, intent = pipeline.frame({camera: image}, t, fid)
        assert not rows[camera]['road_context']['white_lines_detected']
        assert nav['line_state'] != 'paired' and not nav.get('boundary_observed')
        assert intent['esc_us'] == 1500 and intent['steering_us'] == 1610
        assert not any(cone.get('pass_evidence') for cone in nav.get('cones', []))
        assert not pipeline.controller.passed_ids and pipeline.controller.count == 0
    assert not any(channel == 4 and pulse > 1500 for channel, pulse in pipeline.writes)


@pytest.mark.parametrize('curve', [0., 10., 20.])
def test_existing_synthetic_curved_paint_remains_current_paired_observation(curve):
    pipeline = Pipeline()
    image = track(curve=curve)
    for fid in range(1, 9):
        rows, nav, intent = pipeline.frame({'secondary': image}, round((fid-1)*.1, 8), fid)
    assert rows['secondary']['road_context']['valid']
    assert nav['line_state'] == 'paired' and nav['boundary_observed']
    assert intent['esc_us'] > 1500 and not pipeline.guard.fault


@pytest.mark.parametrize('change', ['single', 'missing'])
def test_current_single_or_disappeared_edges_never_reuse_an_old_pair_or_ground_pass_proof(change):
    pipeline = Pipeline()
    for fid in range(1, 9):
        _, nav, _ = pipeline.frame({'secondary': track()}, round((fid-1)*.1, 8), fid)
    assert nav['line_state'] == 'paired'
    image = track(sides=('left',)) if change == 'single' else nonroad('empty')
    observations = []
    for fid in range(9, 13):
        rows, nav, intent = pipeline.frame({'secondary': image}, round((fid-1)*.1, 8), fid)
        observations.append(copy.deepcopy(nav))
        assert nav['line_state'] != 'paired' and not nav.get('boundary_observed')
        assert not rows['secondary']['road_context']['white_lines_detected']
        assert not any(cone.get('pass_evidence') for cone in nav.get('cones', []))
        assert pipeline.controller.count == 0 and not pipeline.controller.passed_ids
        assert intent['phase'] != 'complete'
    assert observations[-1]['line_state'] == 'missing'
    assert not observations[-1].get('lane_bounds_px')
    if change == 'single':
        assert any(nav['line_state'] == 'single' for nav in observations[:-1])


@pytest.mark.parametrize('name', ['cone-track-glare-secondary.jpg', 'cone-track-short-secondary.jpg'])
def test_existing_saved_paint_fixture_still_passes_real_geometry_and_fake_guard(name):
    pipeline = Pipeline()
    image = read_image(ROOT/'vision/tests/fixtures'/name)
    for fid in range(1, 9):
        _, nav, intent = pipeline.frame({'secondary': image}, round((fid-1)*.1, 8), fid)
    assert nav['line_state'] == 'paired' and nav.get('boundary_observed')
    assert intent['esc_us'] > 1500 and not pipeline.guard.fault


@pytest.mark.parametrize('index', [0, 1, 2])
def test_dark_primary_without_measurable_two_sided_support_remains_unknown_in_actual_guard_chain(index):
    pipeline = Pipeline()
    image = read_image(FIXTURES/f'white-live-dark-20261026-primary-{index}.jpg')
    for fid in range(1, 9):
        rows, nav, intent = pipeline.frame({'primary': image}, round((fid-1)*.1, 8), fid)
        assert not rows['primary']['road_context']['valid']
        assert nav['line_state'] != 'paired' and not nav.get('boundary_observed')
        assert intent['esc_us'] == 1500 and intent['steering_us'] == 1610
        assert pipeline.controller.count == 0 and not pipeline.controller.passed_ids
    assert not any(channel == 4 and pulse > 1500 for channel, pulse in pipeline.writes)
