import json
from types import SimpleNamespace

import pytest

from carvision.steering_calibrate import CalibrationDriver, execute_plan, pulse_plan, run


@pytest.mark.parametrize('center,previous,target', [(1500,1500,1600),(1500,1500,900),(1500,True,1525),(1500,1500,1500.0)])
def test_unobserved_jumps_and_invalid_pulses_rejected(center,previous,target):
    with pytest.raises(ValueError):
        pulse_plan(center,previous,target)


def test_extension_is_small_and_returns_to_reference():
    plan = pulse_plan(1500,1450,1400)
    assert plan[0][0] == plan[-1][0] == 1500
    assert all(abs(a[0]-b[0]) <= 10 for a,b in zip(plan,plan[1:]))
    assert min(pulse for pulse,_ in plan) == 1400
    assert sum(duration for _,duration in plan) < 4.5


def test_client_rejects_esc_and_unplanned_width():
    driver = CalibrationDriver(8890,pulse_plan(1500,1500,1500))
    for gpio,pulse in ((13,1500),(17,1500),(12,1510)):
        with pytest.raises(ValueError):
            driver.set_servo(gpio,pulse)


def test_failed_step_still_turns_off_and_releases(monkeypatch):
    monkeypatch.setattr('carvision.steering_calibrate.time.sleep',lambda _:None)
    class Driver:
        def __init__(self): self.calls=[]; self.pulse=0
        def mode(self,gpio): return 0
        def set_servo(self,gpio,pulse):
            self.calls.append((gpio,pulse)); self.pulse=pulse
            if pulse == 1490: raise ConnectionError('lost reply')
        def servo_pulse(self,gpio): return self.pulse
        def release(self,gpio): self.calls.append(('input',gpio))
    driver=Driver(); result={}
    with pytest.raises(ConnectionError):
        execute_plan(driver,pulse_plan(1500,1500,1450),result)
    assert driver.calls[-2:] == [(12,0),('input',12)]
    assert result['pulse_off_confirmed'] and result['pin_input_confirmed']


def test_power_fault_during_hold_releases_pin_without_next_pulse():
    class Driver:
        def __init__(self): self.calls = []; self.pulse = 0
        def mode(self, gpio): return 0
        def set_servo(self, gpio, pulse): self.calls.append((gpio, pulse)); self.pulse = pulse
        def servo_pulse(self, gpio): return self.pulse
        def release(self, gpio): self.calls.append(('input', gpio))
    driver = Driver(); result = {}
    def health():
        if driver.pulse:
            raise RuntimeError('new undervoltage')
    with pytest.raises(RuntimeError, match='undervoltage'):
        execute_plan(driver, [(1650, 2), (1660, .02)], result, health)
    assert driver.calls == [(12, 1650), (12, 0), ('input', 12)]
    assert result['pulse_off_confirmed'] and result['pin_input_confirmed']


def test_restart_blocks_further_output_before_hardware_access(tmp_path, monkeypatch):
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'steering': {'reference_us': 1515, 'last_accepted_us': 1515},
                                 'restart_review': {'further_motion_paused': True}}))
    monkeypatch.setattr('carvision.steering_calibrate.subprocess.run',
                        lambda *a, **k: pytest.fail('hardware access after unexplained restart'))
    args = SimpleNamespace(state=str(state), target_us=1565, run=False)
    assert run(args)['motion_blocked_by_restart']
    args.run = True
    with pytest.raises(RuntimeError, match='unexplained restart'):
        run(args)
    args.center_review_retry = True
    with pytest.raises(RuntimeError, match='unexplained restart'):
        run(args)


def test_restart_retry_is_limited_to_center_and_still_requires_pi(tmp_path, monkeypatch):
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'steering': {'reference_us': 1515, 'last_accepted_us': 1515},
                                 'restart_review': {'further_motion_paused': True},
                                 'steering_5v_pin_wiring_confirmed': True,
                                 'wheels_raised_confirmed': True}))
    monkeypatch.setattr('carvision.steering_calibrate.sys.platform', 'win32')
    with pytest.raises(RuntimeError, match='requires prepared Pi'):
        run(SimpleNamespace(state=str(state), target_us=1515, run=True,
                            center_review_retry=True, bench_prepared=True))
