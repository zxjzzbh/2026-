"""Live pulse targets use the isolated S3 scope, existing lease and time window."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from carvision.manual_drive import DriveError, ManualDrive, SimulationBackend
from carvision.pi5_pwm import Pi5BenchBackend
from carvision.steering_pwm_controls import SCRIPT


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


class Steering(SimulationBackend):
    motor_available = False
    steering_pwm_available = True
    steering_center_us = 1715

    def __init__(self):
        self.calls = []
        self.steering_active = False

    def steering(self, pulse):
        self.calls.append(pulse)
        self.steering_active = True

    def motion(self, *args):
        pytest.fail('PWM tuning must not drive the motor')

    def steering_idle(self):
        self.steering_active = False


def ready(backend=None):
    backend, clock = backend or Steering(), Clock()
    drive = ManualDrive(backend, clock=clock, settling_s=0, session_s=120)
    drive.request('enable', {'client': 'pwm-test', 'bench_ready': True})
    drive.tick()
    return drive, backend, clock


def tune(drive, sequence, pulse, client='pwm-test'):
    return drive.request('steering_pwm', {'client': client, 'sequence': sequence, 'pulse_us': pulse})


def test_live_target_updates_are_ramped_and_report_actual_software_output():
    drive, backend, clock = ready()
    state = tune(drive, 1, 1700)
    assert state['steering_pwm_requested_us'] == 1700
    assert state['steering_pulse_us'] == 1705  # existing 10 us ramp
    clock.now = .03
    tune(drive, 2, 1720)
    clock.now = .06
    drive.tick()
    clock.now = .09
    drive.tick()
    # The request's initial tick also services the existing guarded target.
    assert backend.calls == [1705, 1700, 1710, 1720]
    assert drive.status()['steering_center_us'] == 1715
    assert drive.status()['motor_pulse_us'] is None


def test_center_target_is_written_even_if_the_previous_output_was_idle():
    drive, backend, _ = ready()
    tune(drive, 1, 1715)
    assert backend.calls == [1715]
    assert drive.status()['steering_pwm_active']


@pytest.mark.parametrize('pulse', [1549, 1751, True, 1650.0, '1650', None])
def test_invalid_pulse_does_not_write_to_s3(pulse):
    drive, backend, _ = ready()
    with pytest.raises(DriveError):
        tune(drive, 1, pulse)
    assert backend.calls == []


def test_default_and_combined_scopes_cannot_accept_raw_pwm():
    for backend in (SimulationBackend(), Steering()):
        backend.motor_available = True
        drive, _, _ = ready(backend)
        with pytest.raises(DriveError, match='isolated S3'):
            tune(drive, 1, 1700)


def test_wrong_page_old_sequences_and_directions_cannot_steal_pwm_control():
    drive, backend, _ = ready()
    tune(drive, 5, 1705)
    for seq, client in ((6, 'other-page'), (5, 'pwm-test'), (True, 'pwm-test')):
        with pytest.raises(DriveError):
            tune(drive, seq, 1750, client)
    with pytest.raises(DriveError):
        drive.request('command', {'client': 'pwm-test', 'sequence': 6, 'motor': 'stop', 'steering': 'left'})
    assert backend.calls == [1705]


@pytest.mark.parametrize('ending', ['stop', 'emergency', 'timeout', 'expired'])
def test_stop_faults_and_timeouts_clear_target_and_late_requests_cannot_resume(ending):
    drive, backend, clock = ready()
    tune(drive, 1, 1705)
    clock.now = .03
    if ending == 'expired':
        clock.now = 121
    elif ending == 'timeout':
        clock.now = .21
    else:
        drive.request(ending, {})
    drive.tick()
    assert not drive.status()['steering_pwm_active']
    assert drive.pulse == 1500
    with pytest.raises(DriveError):
        tune(drive, 2, 1750)
    if ending != 'expired':
        assert backend.calls[-1] == 1715


def test_changing_targets_and_heartbeats_do_not_extend_two_second_trial():
    drive, backend, clock = ready()
    for seq in range(1, 21):
        clock.now = (seq - 1) * .1
        tune(drive, seq, 1705 if seq % 2 else 1725)
    clock.now = 2.01
    drive.tick()
    assert drive.blocked and drive.pwm_target is None
    with pytest.raises(DriveError):
        tune(drive, 21, 1750)
    # A fresh explicit release precedes another finite trial.
    drive.request('command', {'client': 'pwm-test', 'sequence': 22, 'motor': 'stop', 'steering': 'center'})
    clock.now = 2.05
    assert tune(drive, 23, 1700)['steering_pwm_active']


@pytest.mark.parametrize('channel,motor,expected', [(3, False, True), (2, False, False), (3, True, False)])
def test_only_identified_uart_s3_steering_only_backend_exposes_capability(tmp_path, channel, motor, expected):
    review = dict(model='Raspberry Pi 5 Model B Rev 1.0', boot_id='boot',
                  wheels_raised_confirmed_by_user=True, esc_off_confirmed_by_user=True,
                  neutral_physically_verified=True, steering_physically_verified=True,
                  stop_physically_verified=True, steering_backend='rasadapter5a_uart',
                  steering_channel=channel, esc_backend='rasadapter5a_uart', esc_channel=4)
    backend = Pi5BenchBackend(tmp_path, tmp_path/'out', 'boot', review,
                              steering_only=not motor, combined=motor)
    assert backend.steering_pwm_available is expected


def test_browser_coalesces_targets_and_cancels_delayed_replies():
    node = os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:
        pytest.skip('Node.js is required for PWM browser checks')
    result = subprocess.run([node, str(Path(__file__).parent/'js/steering_pwm.cjs')],
                            input=json.dumps({'script': SCRIPT}), capture_output=True,
                            text=True, encoding='utf-8', timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
