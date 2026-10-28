"""Controller and persistence integration for opt-in, current-camera calibration.

All observations and outputs are in memory. No camera or actuator is opened.
"""
import copy
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest

from carvision import brake_parking
from carvision.brake_trial import BrakeTrialDrive


CONFIG = Path(__file__).parents[1] / 'configs/brake-parking.json'
PROFILE_ID = 'integration-near-edge-profile'
FEATURE = 'paper_row_near_edge_y_normalized'
PIXELS = {'primary': (352, 308, 261), 'secondary': (342, 316, 289)}


@pytest.fixture
def profile():
    cameras = {}
    identities = {
        'primary': 'USB2.0 Camera: USB2.0 Camera | usb-xhci-hcd.1-2',
        'secondary': 'H65 USB CAMERA: H65 USB CAMERA | usb-xhci-hcd.0-2',
    }
    for camera, pixels in PIXELS.items():
        cameras[camera] = {
            'identity': identities[camera], 'image_size': [480, 360],
            'captured_on': '2026-10-24', 'verified': True,
            'pose': {'fixed': True, 'verified': True, 'description': 'Synthetic fixed test pose'},
            'references': [
                {'distance_m': distance, 'near_edge_y_normalized': y / 359,
                 'near_edge_y_px': y, 'measured': True, 'evidence': 'in-memory fixture',
                 'sample_count': 8, 'spread_px': 0}
                for distance, y in zip((.4, .5, .65), pixels)
            ],
            'white_mask': {'s_max': 80 if camera == 'primary' else 100,
                           'v_min': 200 if camera == 'primary' else 170},
            'runtime_detector_verified': False,
        }
    return {
        'schema_version': 1, 'profile_id': PROFILE_ID, 'enabled': True,
        'verified': True, 'motion_enabled': False,
        'distance_reference': 'vehicle_front_to_nearest_paper_edge_m',
        'image_feature': FEATURE,
        'image_edge_definition': 'Median of strip bbox y+h-1 divided by height-1.',
        'cameras': cameras,
        'overlap': {'verified': True, 'reference_distance_m': .5,
                    'evidence': 'Both fixed views measured independently'},
        'visible_measured_range_m': [.4, .65],
        'physical_stop_verified': False,
    }


@pytest.fixture
def legacy():
    settings = brake_parking.load_settings(CONFIG)
    settings['brake_trigger_distance_m'] = .25
    return settings


def write_profile(root, profile, local=None):
    folder = root / 'vision/configs'
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'crosswalk-distance-calibration.json').write_text(
        json.dumps(profile), encoding='utf-8')
    if local is not None:
        (folder / 'crosswalk-distance.local.json').write_text(json.dumps(local), encoding='utf-8')


def load_active(root, legacy, profile, distance=None):
    local = None if distance is None else {
        'schema_version': 1, 'calibration_id': PROFILE_ID,
        'brake_trigger_distance_m': distance,
    }
    write_profile(root, profile, local)
    return brake_parking.crosswalk_calibrated_settings(legacy, root)


def observation(t, camera='secondary', near=316 / 359, moving=True, **changes):
    return {
        'fresh': True, 'frame_id': round(t * 100) + 1, 'captured_s': t,
        'image_size': [480, 360], 'candidate': True,
        'source_camera': camera, 'calibration_id': PROFILE_ID,
        'image_feature': FEATURE, 'near_edge_y_normalized': near,
        'near_limit_reached': False, 'source_changed': False,
        'far_edge_y_normalized': .1, 'lane_valid': True, 'lane_offset': 0,
        'motion': {'valid': moving is not None, 'moving': moving},
        **changes,
    }


def test_profile_helpers_keep_legacy_distance_and_references_separate(tmp_path, legacy, profile):
    before = copy.deepcopy(legacy)
    settings = load_active(tmp_path, legacy, profile)
    trigger = brake_parking.crosswalk_trigger_reference(settings)
    assert legacy == before
    assert settings['references'] == before['references']
    assert settings['brake_trigger_distance_m'] == .25
    assert brake_parking.brake_trigger_reference(settings)['distance_m'] == .25
    assert settings['crosswalk_distance_trigger_m'] == .4
    assert brake_parking.crosswalk_distance_limits(settings) == [40., 65.]
    assert trigger['distance_m'] == .4
    assert trigger['calibration_id'] == PROFILE_ID and trigger['image_feature'] == FEATURE
    assert trigger['physical_stop_verified'] is False
    assert trigger['camera_references']['primary']['near_edge_y_normalized'] == pytest.approx(352 / 359)
    assert trigger['camera_references']['secondary']['near_edge_y_normalized'] == pytest.approx(342 / 359)


def test_new_saved_distance_applies_only_to_crosswalk_profile(tmp_path, legacy, profile):
    settings = load_active(tmp_path, legacy, profile, .5)
    assert brake_parking.crosswalk_trigger_reference(settings)['distance_m'] == .5
    assert brake_parking.brake_trigger_reference(settings)['distance_m'] == .25
    assert settings['references'] == legacy['references']


@pytest.mark.parametrize('camera', ['primary', 'secondary'])
def test_crosswalk_recomputes_distance_from_selected_camera_near_edge(tmp_path, legacy, profile, camera):
    settings = load_active(tmp_path, legacy, profile, .5)
    controller = brake_parking.BrakeParkingController(settings, crosswalk=True)
    y65 = PIXELS[camera][2] / 359
    result = controller.step(observation(0, camera, y65,
        far_edge_y_normalized=.99, distance_estimate_m=.01), 0)
    # Neither a large legacy far edge nor an injected metric estimate may stop
    # this branch before its independently calibrated near edge reaches 50 cm.
    assert result['action'] in ('search', 'creep')
    assert result['distance_estimate_m'] == pytest.approx(.65)
    y50 = PIXELS[camera][1] / 359
    result = controller.step(observation(.1, camera, y50,
        far_edge_y_normalized=.01, distance_estimate_m=20), .1)
    assert result['action'] == 'brake' and result['esc_us'] < 1500
    assert result['distance_estimate_m'] == pytest.approx(.5)


def test_default_parking_branch_ignores_near_profile(tmp_path, legacy, profile):
    settings = load_active(tmp_path, legacy, profile, .4)
    controller = brake_parking.BrakeParkingController(settings)
    result = controller.step(observation(0, near=.99, far_edge_y_normalized=.60), 0)
    assert result['action'] == 'search'
    assert result['brake_trigger'] == brake_parking.brake_trigger_reference(legacy)
    result = controller.step(observation(.1, near=.1, far_edge_y_normalized=.75), .1)
    assert result['action'] == 'brake'
    assert settings['pwm'] == legacy['pwm'] and settings['hold_s'] == legacy['hold_s'] == 10


@pytest.mark.parametrize('change', [
    {'calibration_id': 'earlier-camera-pose'},
    {'source_camera': 'unknown-camera'},
    {'image_feature': 'paper_row_far_edge_y_normalized'},
])
def test_profile_or_camera_identity_mismatch_fails_without_forward(tmp_path, legacy, profile, change):
    settings = load_active(tmp_path, legacy, profile)
    controller = brake_parking.BrakeParkingController(settings, crosswalk=True)
    result = controller.step(observation(0, near=289 / 359, **change), 0)
    assert result['phase'] == 'fault' and result['action'] == 'neutral'
    assert result['esc_us'] == 1500


def test_unseen_25cm_trigger_is_rejected_without_changing_legacy(tmp_path, legacy, profile):
    write_profile(tmp_path, profile, {'schema_version': 1, 'calibration_id': PROFILE_ID,
                                    'brake_trigger_distance_m': .25})
    with pytest.raises(ValueError):
        brake_parking.crosswalk_calibrated_settings(legacy, tmp_path)
    settings = load_active(tmp_path / 'valid-profile', legacy, profile)
    settings['crosswalk_distance_trigger_m'] = .25
    with pytest.raises(ValueError):
        brake_parking.crosswalk_trigger_reference(settings)
    assert legacy['brake_trigger_distance_m'] == .25


def test_saved_distance_from_different_profile_is_not_reused(tmp_path, legacy, profile):
    write_profile(tmp_path, profile, {'schema_version': 1, 'calibration_id': 'old-pose',
                                    'brake_trigger_distance_m': .5})
    with pytest.raises(ValueError):
        brake_parking.crosswalk_calibrated_settings(legacy, tmp_path)


@pytest.mark.parametrize('near', [.99, None])
def test_nearer_than_measured_range_still_brakes_without_metric_extrapolation(tmp_path, legacy, profile, near):
    settings = load_active(tmp_path, legacy, profile)
    controller = brake_parking.BrakeParkingController(settings, crosswalk=True)
    result = controller.step(observation(0, near=near, near_limit_reached=True), 0)
    assert result['distance_estimate_m'] is None
    assert result['action'] == 'brake' and result['esc_us'] < 1500


def test_camera_change_clears_metric_history_not_hold_or_once_only_speech(tmp_path, legacy, profile):
    settings = load_active(tmp_path, legacy, profile, .5)
    controller = brake_parking.BrakeParkingController(settings, crosswalk=True, first_detection_stop=True)
    events = []
    hold_started = None
    changed = False
    for step in range(125):
        t = step / 10
        camera = 'secondary' if step < 15 else 'primary'
        y = PIXELS[camera][1] / 359
        result = controller.step(observation(t, camera, y, moving=False,
            source_changed=step == 15), t)
        events += result['events']
        assert result['action'] in ('brake', 'neutral')
        assert controller.first_detection_handled and not controller.preliminary_stop
        if result['phase'] == 'hold' and hold_started is None:
            hold_started = controller.hold_since
        if step == 15:
            changed = True
            assert controller.hold_since == hold_started
            assert controller.spoken
            assert len(controller.metric_history) <= 1
            assert result['closing_speed_estimate_mps'] is None
        if result['phase'] == 'done':
            assert t - hold_started >= 10
            break
    assert changed and controller.phase == 'done'
    assert events == [{'event': 'speak', 'text': settings['speech_text']}]


def test_missing_profile_preserves_legacy_semantics(tmp_path, legacy):
    settings = brake_parking.crosswalk_calibrated_settings(legacy, tmp_path)
    assert settings == legacy
    assert brake_parking.crosswalk_distance_limits(settings) == [25, 65]
    assert brake_parking.crosswalk_trigger_reference(settings) == brake_parking.brake_trigger_reference(legacy)
    controller = brake_parking.BrakeParkingController(settings, crosswalk=True)
    result = controller.step(observation(0, far_edge_y_normalized=.75), 0)
    assert result['action'] == 'brake'


@pytest.mark.parametrize('kind', ['disabled', 'invalid', 'malformed'])
def test_present_but_unusable_profile_never_silently_falls_back(tmp_path, legacy, profile, kind):
    candidate = copy.deepcopy(profile)
    if kind == 'disabled':
        candidate['enabled'] = False
    elif kind == 'invalid':
        candidate['cameras']['primary']['references'][0]['near_edge_y_normalized'] = .1
    write_profile(tmp_path, candidate)
    if kind == 'malformed':
        (tmp_path / 'vision/configs/crosswalk-distance-calibration.json').write_text('{', encoding='utf-8')
    with pytest.raises(ValueError):
        brake_parking.crosswalk_calibrated_settings(legacy, tmp_path)
    assert 'crosswalk_calibration' not in legacy


class IdleDriver:
    def __init__(self):
        self.commands = []

    def status(self):
        return {'setup_token': 'boot', 'mode': 'disabled', 'boot_test_phase': 'esc_off',
                'hardware_output': False, 'worker_started': False}

    def request(self, action, payload):
        self.commands.append((action, payload))
        return self.status()


def test_out_of_range_saved_trigger_keeps_current_geometry_readable_but_blocks_prepare(tmp_path, profile):
    write_profile(tmp_path, profile, {'schema_version': 1, 'calibration_id': PROFILE_ID,
                                    'brake_trigger_distance_m': .25})
    driver = IdleDriver()
    opened = []
    wrapper = BrakeTrialDrive(driver, {}, CONFIG, tmp_path,
        actuator_factory=lambda: opened.append('actuator'))
    status = wrapper.status()['crosswalk_trial']
    assert status['settings_error'] and not status['can_prepare']
    assert status['calibration_id'] == PROFILE_ID
    assert status['distance_limits_cm'] == [40., 65.]
    with pytest.raises(ValueError):
        wrapper.request('crosswalk_settings', {'client': 'page',
            'settings_revision': wrapper.settings_revision, 'brake_distance_cm': 25})
    with pytest.raises(ValueError):
        wrapper.request('crosswalk_prepare', {'client': 'page', 'setup_token': 'boot',
            'esc_off': True, 'fb_mode_confirmed': True})
    assert not opened and not driver.commands


def test_saving_a_distance_cannot_clear_disabled_profile_preflight_block(tmp_path, profile):
    candidate = copy.deepcopy(profile)
    candidate['enabled'] = False
    write_profile(tmp_path, candidate)
    driver = IdleDriver()
    opened = []
    wrapper = BrakeTrialDrive(driver, {}, CONFIG, tmp_path,
        actuator_factory=lambda: opened.append('actuator'))
    assert not wrapper.status()['crosswalk_trial']['can_prepare']
    with pytest.raises(ValueError):
        wrapper.request('crosswalk_settings', {'client': 'page',
            'settings_revision': wrapper.settings_revision, 'brake_distance_cm': 50})
    assert wrapper.status()['crosswalk_trial']['settings_error']
    assert not wrapper.status()['crosswalk_trial']['can_prepare']
    assert not opened and not driver.commands


def test_active_profile_cli_refuses_before_reading_camera_or_opening_actuator(tmp_path, profile, monkeypatch):
    from carvision import brake_runtime

    write_profile(tmp_path, profile)
    config = tmp_path / 'vision/configs/brake-parking.json'
    shutil.copyfile(CONFIG, config)

    def forbidden(*args, **kwargs):
        pytest.fail('Unsupported CLI must reject an active dual profile before I/O')

    monkeypatch.setattr(brake_runtime, 'PreviewReader', forbidden)
    monkeypatch.setattr(brake_runtime, 'BrakeActuator', forbidden)
    args = SimpleNamespace(config=str(config), run=True, esc_mode='F/B',
        preview='http://127.0.0.1:8080', seconds=10, output=str(tmp_path / 'never-created'),
        path_mode='straight', port='unused')
    with pytest.raises(ValueError):
        brake_runtime.camera(args)
    assert not (tmp_path / 'never-created').exists()


def test_real_deployed_profile_loads_through_bundle_into_crosswalk_controller(tmp_path):
    from carvision.race_flow import RaceFlowController, load_bundle

    # Preserve real vehicle geometry and other deployed local settings. Only
    # omit the independent crosswalk override to exercise its measured default.
    source_configs = CONFIG.parent
    copied_configs = tmp_path / 'vision/configs'
    shutil.copytree(source_configs, copied_configs,
                    ignore=shutil.ignore_patterns('crosswalk-distance.local.json'))
    profile = json.loads((copied_configs / 'crosswalk-distance-calibration.json').read_text(encoding='utf-8-sig'))
    assert profile['enabled'] is True
    bundle = load_bundle(tmp_path)
    assert bundle['brake']['crosswalk_calibration']['profile_id'] == profile['profile_id']
    assert bundle['brake']['crosswalk_distance_trigger_m'] == .4
    flow = RaceFlowController(bundle, 0.)
    flow._go('crosswalk', 0., 'fixture stage entry')
    assert flow.controller.crosswalk_profile['profile_id'] == profile['profile_id']
    assert flow.controller.brake_reference['distance_m'] == .4
    assert flow.controller.first_detection_stop
    near = next(row['near_edge_y_normalized'] for row in profile['cameras']['secondary']['references']
                if row['distance_m'] == .4)
    result = flow.controller.step(observation(0, near=near,
        calibration_id=profile['profile_id']), 0.)
    assert result['action'] == 'brake' and result['calibration_id'] == profile['profile_id']
    assert result['distance_estimate_m'] == pytest.approx(.4)
    assert result['physical_stop_verified'] is False
