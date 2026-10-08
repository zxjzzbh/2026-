import json
import time
from dataclasses import asdict, replace
from types import SimpleNamespace

import pytest

from carvision.pi_control import PiActuator, PiControlConfig, SpeedFeedbackController
from carvision.pi_runtime import pi_run
from carvision.race import Observation, RaceConfig
from carvision.race_workflows import demonstration


def calibrated():
    # Explicitly fake calibration, never copied into deployed settings.
    return PiControlConfig(esc_model='fake-test-controller', calibration_confirmed=True,
                           esc_neutral_us=1500, esc_forward_us=1600,
                           steering_left_us=1100, steering_center_us=1500,
                           steering_right_us=1900, speed_kp=.5)


class FakePWM:
    def __init__(self):
        self.calls = []
    def open(self):
        self.calls.append('open')
    def set_pulses(self, esc, steering):
        self.calls.append((esc, steering))
    def neutral(self):
        self.calls.append('neutral')
    def close(self):
        self.calls.append('close')
        return []


def test_unconfigured_pi_cannot_enable_pwm():
    with pytest.raises(ValueError, match='calibration incomplete'):
        PiActuator(PiControlConfig(), run=True)


def test_dry_run_does_not_open_even_injected_driver():
    backend = FakePWM()
    actuator = PiActuator(calibrated(), backend=backend)
    actuator.open()
    result = actuator.apply(.1, .2)
    actuator.close()
    assert not backend.calls
    assert result['hardware_output'] is False


def test_assigned_gpio_channels_cannot_alias_or_be_mixed_up():
    for pins in [(13, 13), (13, 12), (18, 19)]:
        with pytest.raises(ValueError):
            replace(calibrated(), steering_gpio=pins[0], esc_gpio=pins[1]).validate()


def test_pwm_mapping_clamps_throttle_and_honors_calibrated_direction():
    actuator = PiActuator(calibrated())
    assert actuator.planned_pulses(1, 1)['esc_us'] == 1515
    assert actuator.planned_pulses(0, -1)['steering_us'] == 1100
    reversed_servo = replace(calibrated(), steering_left_us=1900, steering_right_us=1100)
    assert PiActuator(reversed_servo).planned_pulses(0, -1)['steering_us'] == 1900


def test_expired_command_is_neutralized_without_new_frames():
    backend = FakePWM()
    actuator = PiActuator(replace(calibrated(), command_lease_s=.04), run=True, backend=backend)
    actuator.open()
    try:
        actuator.apply(.1, .2)
        deadline = time.monotonic() + 1
        while 'neutral' not in backend.calls and time.monotonic() < deadline:
            time.sleep(.01)
        assert 'neutral' in backend.calls
    finally:
        actuator.close()
    assert backend.calls[-1] == 'close'


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -1, True])
def test_invalid_throttle_neutralizes_instead_of_reusing_last_command(value):
    backend = FakePWM()
    actuator = PiActuator(calibrated(), run=True, backend=backend)
    actuator.open()
    try:
        actuator.apply(.1, 0)
        with pytest.raises(ValueError):
            actuator.apply(value, 0)
        assert backend.calls[-1] == 'neutral'
    finally:
        actuator.close()


def test_stop_intent_does_not_convert_speed_error_into_throttle():
    loop = SpeedFeedbackController(calibrated())
    observation = Observation(0, telemetry_valid=True, speed_mps=.1)
    assert loop.command({'action': 'stop', 'speed_mps': 0, 'steering_normalized': .5}, observation) == (0, 0)


def test_default_pi_replay_does_not_require_gpio_or_fake_calibration(tmp_path):
    race_file = tmp_path / 'race.json'
    pi_file = tmp_path / 'pi.json'
    input_file = tmp_path / 'input.jsonl'
    race_file.write_text(json.dumps(asdict(RaceConfig(school='学校', team='队名', vehicle_width_m=.19))), encoding='utf-8')
    pi_file.write_text(json.dumps(asdict(PiControlConfig())), encoding='utf-8')
    input_file.write_text('\n'.join(json.dumps(asdict(o)) for o in demonstration()), encoding='utf-8')
    args = SimpleNamespace(race_config=race_file, pi_config=pi_file, input=str(input_file), output=tmp_path / 'out',
                           run=False, acknowledge_motion=False)
    result = pi_run(args)
    assert result['phase'] == 'complete' and result['hardware_output'] is False
    assert result['stop_requested'] is False
    rows = [json.loads(line) for line in (args.output / 'pi-decisions.jsonl').read_text(encoding='utf-8').splitlines()]
    assert all(not row.get('actuator', {}).get('hardware_output', False) for row in rows)


def test_pi_trace_error_records_stop_and_summary(tmp_path):
    race_file, pi_file = tmp_path / 'race.json', tmp_path / 'pi.json'
    race_file.write_text('{}')
    pi_file.write_text('{}')
    bad_input = tmp_path / 'bad.jsonl'
    bad_input.write_text('{broken json}')
    args = SimpleNamespace(race_config=race_file, pi_config=pi_file, input=str(bad_input),
                           output=tmp_path / 'out', run=False, acknowledge_motion=False)
    with pytest.raises(ValueError, match='invalid observation'):
        pi_run(args)
    row = json.loads((args.output / 'pi-decisions.jsonl').read_text(encoding='utf-8'))
    assert row['action'] == 'stop' and row['hardware_output'] is False
    assert json.loads((args.output / 'summary.json').read_text())['error']


def test_file_replay_cannot_become_real_pwm_by_adding_run(tmp_path):
    race_file = tmp_path / 'race.json'
    pi_file = tmp_path / 'pi.json'
    race_file.write_text('{}')
    pi_file.write_text('{}')
    args = SimpleNamespace(race_config=race_file, pi_config=pi_file, input='recording.jsonl', output=tmp_path / 'out',
                           run=True, acknowledge_motion=True)
    with pytest.raises(ValueError, match='live stdin'):
        pi_run(args)
    assert not args.output.exists()
