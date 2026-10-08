import json
from dataclasses import asdict
from types import SimpleNamespace

import pytest

from carvision.gimbal import GimbalConfig, ServoCalibration
from carvision.gimbal_bench import brief_reference, daemon_command, pulse_plan, run


class Driver:
    def __init__(self):
        self.calls = []
        self.pulse = 0
        self.input = True
        self.servo_function = False
    def mode(self, gpio):
        return 0 if self.input else 1
    def set_servo(self, gpio, pulse):
        self.calls.append((gpio, pulse))
        self.pulse = pulse
        self.servo_function = True
    def servo_pulse(self, gpio):
        if not self.servo_function:
            raise RuntimeError('NOT_SERVO (-93)')
        return self.pulse
    def release(self, gpio):
        self.calls.append(('input', gpio))
        self.input = True
        self.servo_function = False


def test_reference_only_touches_one_axis_and_confirms_off():
    driver = Driver()
    result = brief_reference(driver, 17)
    assert driver.calls == [(17, 1500), (17, 0), ('input', 17)]
    assert result['pulse_off_confirmed'] and not result['calibration_completed']


def test_lost_acknowledgement_still_stops_and_releases():
    class Broken(Driver):
        def set_servo(self, gpio, pulse):
            super().set_servo(gpio, pulse)
            if pulse:
                raise ConnectionError('lost acknowledgement')
    driver = Broken()
    with pytest.raises(ConnectionError):
        brief_reference(driver, 27)
    assert driver.calls == [(27, 1500), (27, 0), ('input', 27)]


def test_off_readback_failure_still_releases_and_reports_attempted_output():
    class BrokenReadback(Driver):
        def servo_pulse(self, gpio):
            return self.pulse if self.pulse else 1500
    driver = BrokenReadback()
    progress = {}
    with pytest.raises(RuntimeError, match='cleanup failed'):
        brief_reference(driver, 17, progress)
    assert driver.calls[-1] == ('input', 17)
    assert progress['hardware_output'] is True
    assert progress['pulse_off_confirmed'] is False
    assert progress['pin_input_confirmed'] is True


def test_observation_stays_near_reference_returns_gradually_and_stops():
    driver = Driver()
    result = brief_reference(driver, 17, observe=True)
    pulses = [pulse for pin, pulse in driver.calls if pin == 17 and pulse]
    assert pulses[0] == pulses[-1] == 1500
    assert max(pulses) == 1550
    assert all(abs(a - b) <= 10 for a, b in zip(pulses, pulses[1:]))
    assert result['requested_duration_s'] == pytest.approx(.6)
    assert driver.calls[-2:] == [(17, 0), ('input', 17)]
    assert not any(pin == 27 for pin, _ in driver.calls)


def test_mid_observation_failure_disables_output_and_releases_pin():
    class BrokenStep(Driver):
        def set_servo(self, gpio, pulse):
            super().set_servo(gpio, pulse)
            if pulse == 1530:
                raise ConnectionError('lost mid-step acknowledgement')
    driver = BrokenStep()
    with pytest.raises(ConnectionError):
        brief_reference(driver, 17, observe=True)
    assert driver.calls[-2:] == [(17, 0), ('input', 17)]


def test_two_second_observation_extends_holds_without_expanding_pulse_range(tmp_path):
    plan = pulse_plan(True, 2.0)
    assert sum(period for _, period in plan) == pytest.approx(2.0)
    assert [pulse for pulse, _ in plan] == [pulse for pulse, _ in pulse_plan(True)]
    command = daemon_command(tmp_path, 17, 8890, 2.0)
    assert '5s' in command


@pytest.mark.parametrize('seconds', [0, 1, 3, float('nan'), float('inf'), True])
def test_other_observation_durations_are_rejected_before_gpio(seconds):
    driver = Driver()
    with pytest.raises(ValueError):
        brief_reference(driver, 17, observe=True, observe_seconds=seconds)
    assert not driver.calls


def test_existing_pin_owner_is_preserved():
    driver = Driver()
    driver.input = False
    with pytest.raises(RuntimeError, match='owner'):
        brief_reference(driver, 17)
    assert not driver.calls


@pytest.mark.parametrize('gpio', [12, 13, True, 17.0])
def test_invalid_axis_never_sends_output(gpio):
    driver = Driver()
    with pytest.raises(ValueError):
        brief_reference(driver, gpio)
    assert not driver.calls


def test_private_daemon_has_independent_deadline_and_only_selected_permission(tmp_path):
    command = daemon_command(tmp_path, 17, 8890)
    assert command[:7] == ['sudo', '-n', 'timeout', '--signal=TERM', '--kill-after=1s', '3s', 'env']
    assert command[command.index('-x') + 1] == '0x20000'
    assert command[command.index('-t') + 1] == '1'
    assert '-l' in command and '-f' in command


def test_dry_reference_does_not_start_any_process_or_modify_config(tmp_path, monkeypatch):
    config = GimbalConfig(pan=ServoCalibration('MG996R'))
    path = tmp_path / 'gimbal.json'
    original = json.dumps(asdict(config))
    path.write_text(original)
    def forbidden(*args, **kwargs):
        pytest.fail('dry reference accessed a process')
    monkeypatch.setattr('carvision.gimbal_bench.subprocess.run', forbidden)
    result = run(SimpleNamespace(gimbal_config=path, axis='pan', run=False))
    assert result['hardware_output'] is False and result['full_control_ready'] is False
    assert path.read_text() == original


def test_missing_wiring_confirmation_is_rejected_before_host_access(tmp_path):
    config = GimbalConfig(pan=ServoCalibration('MG996R'))
    path = tmp_path / 'gimbal.json'
    path.write_text(json.dumps(asdict(config)))
    with pytest.raises(ValueError, match='5V-pin'):
        run(SimpleNamespace(gimbal_config=path, axis='pan', run=True,
                            confirm_5v_pin_wiring=False, acknowledge_motion=True))
