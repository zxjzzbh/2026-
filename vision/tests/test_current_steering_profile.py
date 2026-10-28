"""Cross-layer steering regressions; UART writes are captured in memory only."""
import json
import struct
from pathlib import Path

import pytest

from carvision.brake_output import write_parking_pulse
from carvision.cone_trial_output import ConePulseGuard
from carvision.cone_navigation import driving_settings, execution_readiness, validate_driving
from carvision.cone_short_output import MODES
from carvision.manual_steering import manual_endpoints
from carvision.rasadapter5 import RasAdapter
from carvision.autonomy_profile import AutonomyProfile
from test_cone_trial_output import settings, command


@pytest.mark.parametrize('pulse', [1350, 1610, 1870])
def test_cone_guard_to_actual_uart_encoding_preserves_new_targets(pulse):
    board = RasAdapter()
    packets = []
    board.send = lambda function, payload: packets.append((function, payload))
    guard = ConePulseGuard(settings(), lambda ch, p: write_parking_pulse(
        board, ch, p, forward_limit_us=1625), mode='F/B')
    guard.accept(command(0, 0, pulse), 0)
    decoded = [struct.unpack('<BHBBH', payload) for function, payload in packets if function == 4]
    assert [(row[3], row[4]) for row in decoded[-2:]] == [(3, pulse), (4, 1600)]
    guard.tick(.201)
    assert guard.fault and guard.esc == 1400
    guard.tick(.322)
    decoded = [struct.unpack('<BHBBH', payload) for _, payload in packets]
    assert [(row[3], row[4]) for row in decoded[-2:]] == [(4, 1500), (3, 1610)]


@pytest.mark.parametrize('pulse', [1349, 1871, 1900, True, 1870.0])
def test_out_of_range_pulses_never_reach_uart(pulse):
    board = RasAdapter()
    board.send = lambda *args: pytest.fail('invalid command reached UART')
    with pytest.raises(ValueError):
        board.set_position(3, pulse)


def test_only_s3_inherits_the_new_range():
    board = RasAdapter()
    board.send = lambda *args: pytest.fail('other channel inherited S3 limits')
    for channel in (1, 2, 4, 5, 6):
        for pulse in (1350, 1870):
            with pytest.raises(ValueError):
                board.set_position(channel, pulse)


def test_current_profiles_agree_without_granting_autonomy_permission():
    root = Path(__file__).parents[1]
    template = json.loads((root/'deployment/boot-test-template.json').read_text(encoding='utf-8'))
    assert manual_endpoints(template['steering_center_us'], template['manual_steering_span_us']) == (1870, 1350)
    assert template['manual_endpoints_field_verified'] is False
    profile = AutonomyProfile.load(root/'configs/autonomy-pi5.json')
    assert [profile.data['motion'][key] for key in ('steering_left_us','steering_center_us','steering_right_us')] == [1870,1610,1350]
    assert profile.missing() and profile.data['motion']['calibrated'] is False
    assert MODES == {'turn_1350':1350,'turn_1870':1870,'brake':1610}


def test_old_radius_measurements_cannot_verify_new_pulses():
    root = Path(__file__).parents[1]
    data = json.loads((root/'configs/cone-trial.json').read_text(encoding='utf-8'))
    data['driving'] = {'steering_radius_1570_cm':40., 'steering_radius_1650_cm':-40., 'steering_radius_1430_cm':50., 'steering_radius_1770_cm':-50., 'steering_verified':True}
    d = driving_settings(data)
    assert d['steering_radius_1350_cm'] is None and d['steering_radius_1870_cm'] is None
    report = execution_readiness(data)
    assert not report['ready']
    assert 'driving.steering_radius_1350_cm' in report['missing']
    assert 'driving.steering_radius_1870_cm' in report['missing']
    data['driving'].update(steering_radius_1350_cm=-40., steering_radius_1870_cm=40.)
    with pytest.raises(ValueError):
        validate_driving(data)


def test_symmetric_pulses_and_legacy_launcher_select_the_same_current_profile():
    from carvision.steering_limits import STEERING_LEFT_US, STEERING_CENTER_US, STEERING_RIGHT_US
    assert STEERING_LEFT_US - STEERING_CENTER_US == STEERING_CENTER_US - STEERING_RIGHT_US == 260
    assert manual_endpoints(1610,260) == manual_endpoints(1610,180) == (1870,1350)


def test_generic_config_and_current_cone_defaults_agree():
    import os
    from carvision.pi_control import load_pi_config
    from carvision.cone_low_speed import trial_settings
    from carvision.cone_navigation import reuse_legacy_ground
    root=Path(__file__).parents[1]
    config=load_pi_config(root/'configs/pi-control.json')
    assert (config.steering_left_us,config.steering_center_us,config.steering_right_us)==(1870,1610,1350)
    assert not config.calibration_confirmed and config.command_lease_s==.25
    data=json.loads(Path(os.environ.get('CARVISION_REPOSITORY_CONE_CONFIG',root/'configs/cone-trial.json')).read_text(encoding='utf-8'))
    # Synthetic dimensions isolate this endpoint-selection test from the
    # intentionally incomplete public geometry template; no saved data changes.
    data['geometry'].update(vehicle_length_m=.42,rear_axle_to_front_m=.335,rear_overhang_m=.085)
    reference=json.loads((root/'configs/cone-legacy-ground-candidate.json').read_text(encoding='utf-8'))
    selected=trial_settings(reuse_legacy_ground(data,reference))['low_speed_test']
    assert (selected['left_us'],selected['right_us'])==(1870,1350)
