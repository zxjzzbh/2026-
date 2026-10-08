from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.steering_bench import SteeringDriver, brief_reference, daemon_command, pulse_plan, run


class Driver:
    def __init__(self):
        self.calls = []
        self.pulse = 0
        self.owner = False

    def mode(self, gpio):
        return int(self.owner)

    def set_servo(self, gpio, pulse):
        self.calls.append((gpio, pulse))
        self.pulse = pulse

    def servo_pulse(self, gpio):
        return self.pulse

    def release(self, gpio):
        self.calls.append(('input', gpio))


def test_dry_run_never_accesses_host(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('dry test accessed hardware')
    monkeypatch.setattr('carvision.steering_bench.subprocess.run', forbidden)
    result = run(SimpleNamespace(observe=True, run=False))
    assert not result['hardware_output'] and not result['esc_output']
    assert result['requested_duration_s'] == pytest.approx(2)


def test_observation_moves_both_directions_only_on_12(monkeypatch):
    monkeypatch.setattr('carvision.steering_bench.time.sleep', lambda _: None)
    driver = Driver()
    result = brief_reference(driver, observe=True)
    pulses = [pulse for gpio, pulse in driver.calls if gpio == 12 and pulse]
    assert pulses[0] == pulses[-1] == 1500
    assert min(pulses) == 1450 and max(pulses) == 1550
    assert all(abs(a-b) <= 10 for a, b in zip(pulses, pulses[1:]))
    assert driver.calls[-2:] == [(12, 0), ('input', 12)]
    assert result['pulse_off_confirmed'] and not result['calibration_completed']
    assert not result['esc_output']


def test_partial_write_failure_always_attempts_stop_and_release(monkeypatch):
    monkeypatch.setattr('carvision.steering_bench.time.sleep', lambda _: None)
    class Broken(Driver):
        def set_servo(self, gpio, pulse):
            super().set_servo(gpio, pulse)
            if pulse == 1470:
                raise ConnectionError('lost acknowledgement')
    driver = Broken()
    with pytest.raises(ConnectionError):
        brief_reference(driver, observe=True)
    assert driver.calls[-2:] == [(12, 0), ('input', 12)]


def test_owner_is_preserved_without_any_output():
    driver = Driver()
    driver.owner = True
    with pytest.raises(RuntimeError, match='owner'):
        brief_reference(driver)
    assert not driver.calls


@pytest.mark.parametrize('gpio,pulse', [(13,1500),(17,1500),(27,1500),(True,1500),(12,1449),(12,1551),(12,True)])
def test_client_rejects_other_pins_and_large_pulses(gpio, pulse):
    driver = SteeringDriver(8890)
    with pytest.raises(ValueError):
        driver.set_servo(gpio, pulse)


def test_private_driver_expires_and_only_permits_12():
    command = daemon_command(Path('/tmp/driver'), True)
    assert '5s' in command and '--kill-after=1s' in command
    assert command[command.index('-x')+1] == '0x1000'
    assert command[command.index('-t')+1] == '1'
    assert '-l' in command and '-f' in command


def test_confirmation_required_before_any_host_access():
    with pytest.raises(ValueError, match='wiring'):
        run(SimpleNamespace(run=True, observe=False, confirm_5v_pin_wiring=False, wheels_raised=True))
