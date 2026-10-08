"""Fresh key leases, release, hard hold limits and separated bench scope."""
from pathlib import Path

import pytest

from carvision.manual_drive import DriveError, ManualDrive, SimulationBackend
from carvision.pi5_pwm import Pi5RaisedHeldBackend, RasAdapterPWM


class Clock:
    now = 0.0
    def __call__(self):
        return self.now


class Hardware(SimulationBackend):
    hardware_output = True
    raised_held_reviewed = True
    combined_trial = True
    steering_center_us = 1715
    def __init__(self):
        self.calls = []
    def motion(self, pulse, seconds):
        self.calls.append(('motion', pulse, seconds))
    def neutral(self):
        self.calls.append(('neutral',))
    def steering(self, pulse):
        self.calls.append(('steering', pulse))


def ready():
    clock, backend = Clock(), Hardware()
    drive = ManualDrive(backend, clock=clock, settling_s=0, session_s=60, raised_held=True)
    drive.request('enable', dict(client='keyboard-test', bench_ready=True))
    drive.tick()
    return drive, backend, clock


def command(drive, seq, motor='forward', turn='center'):
    return drive.request('command', dict(client='keyboard-test', sequence=seq,
                         motor=motor, steering=turn, input_source='keyboard'))


def test_held_control_is_opt_in_and_cannot_reuse_simulation_or_unreviewed_hardware():
    for backend in (SimulationBackend(), Hardware()):
        if backend.hardware_output:
            backend.raised_held_reviewed = False
        with pytest.raises(ValueError, match='reviewed raised-wheel'):
            ManualDrive(backend, session_s=60, raised_held=True)
    with pytest.raises(ValueError):
        ManualDrive(Hardware(), session_s=120, raised_held=True)


def test_held_key_runs_beyond_point_eight_but_stops_at_three_even_with_fresh_messages():
    drive, backend, clock = ready()
    for seq in range(1, 66):
        clock.now = (seq-1)*.05
        command(drive, seq)
        drive.tick()
    assert any(call[0] == 'motion' for call in backend.calls)
    assert drive.blocked and drive.pulse == 1500
    assert drive.status()['motion_hold_max_s'] == 3
    assert not drive.status()['continuous_simulation']
    assert all(.02 <= call[2] <= .2 for call in backend.calls if call[0] == 'motion')
    count = len([c for c in backend.calls if c[0] == 'motion'])
    clock.now += .05
    command(drive, 66)
    drive.tick()
    assert len([c for c in backend.calls if c[0] == 'motion']) == count


def test_status_and_guardian_checks_never_replace_fresh_keyboard_messages():
    drive, backend, clock = ready()
    command(drive, 1); drive.tick()
    for instant in (.05, .1, .15, .21):
        clock.now = instant
        drive.status(); drive.tick()
    assert drive.mode == 'emergency' and drive.pulse == 1500
    count = len([c for c in backend.calls if c[0] == 'motion'])
    with pytest.raises(DriveError):
        command(drive, 2)
    assert len([c for c in backend.calls if c[0] == 'motion']) == count


def test_space_emergency_cannot_be_cleared_by_held_key_or_speed_request():
    drive, backend, clock = ready()
    command(drive, 1); drive.tick()
    drive.request('emergency', {'stop_source': 'keyboard_space'})
    with pytest.raises(DriveError):
        command(drive, 2)
    with pytest.raises(DriveError):
        drive.request('speed', {'client': 'keyboard-test', 'speed_percent': 100})
    assert drive.mode == 'emergency' and drive.pulse == 1500


def test_release_motor_key_stops_motor_while_turn_key_remains_held():
    drive, backend, clock = ready()
    command(drive, 1, 'forward', 'left'); drive.tick()
    clock.now = .05
    command(drive, 2, 'stop', 'left'); drive.tick()
    assert drive.pulse == 1500 and drive.turn == 'left'
    clock.now = .1
    command(drive, 3, 'stop', 'center'); drive.tick()
    assert not drive.blocked and drive.turn == 'center'


def test_reverse_keeps_brake_and_full_neutral_gap_without_forward_output():
    drive, backend, clock = ready()
    for seq in range(1, 43):
        clock.now = (seq-1)*.05
        command(drive, seq, 'reverse'); drive.tick()
        if clock.now < 1.5:
            assert drive.stage != 'reverse'
    assert drive.stage == 'reverse'
    assert all(c[1] == 1300 for c in backend.calls if c[0] == 'motion')
    drive.request('stop', {'stop_source': 'key_release'})
    assert drive.pulse == 1500


def review(**extra):
    return dict(model='Raspberry Pi 5 Model B Rev 1.0', boot_id='current',
                wheels_raised_confirmed_by_user=True, esc_off_confirmed_by_user=True,
                neutral_physically_verified=True, steering_physically_verified=True,
                stop_physically_verified=True, keyboard_stop_physically_verified=True,
                raised_held_requested_by_user=True, continuous_ground_driving_enabled=False,
                esc_backend='rasadapter5a_uart', esc_channel=4,
                steering_backend='rasadapter5a_uart', steering_channel=3,
                steering_center_us=1715, **extra)


def test_raised_review_cannot_enable_ground_or_use_old_boot_or_missing_stop(tmp_path):
    valid = review()
    Pi5RaisedHeldBackend(tmp_path, tmp_path/'run', 'current', valid)
    for field in ('wheels_raised_confirmed_by_user', 'keyboard_stop_physically_verified',
                  'stop_physically_verified', 'neutral_physically_verified'):
        with pytest.raises(ValueError):
            Pi5RaisedHeldBackend(tmp_path, tmp_path/'run', 'current', {**valid, field:False})
    with pytest.raises(ValueError):
        Pi5RaisedHeldBackend(tmp_path, tmp_path/'run', 'another-boot', valid)
    with pytest.raises(ValueError):
        Pi5RaisedHeldBackend(tmp_path, tmp_path/'run', 'current',
                            {**valid, 'continuous_ground_driving_enabled':True})


def test_uart_idle_preserves_trial_reference_without_touching_other_channels(monkeypatch):
    class Board:
        def __init__(self): self.calls = []
        def set_position(self, channel, pulse, *args): self.calls.append((channel, pulse))
    monkeypatch.setattr('carvision.rasadapter5.RasAdapter', Board)
    pwm = RasAdapterPWM(motor=True, channel=3, esc_channel=4, center_us=1715)
    pwm.turn(1750); pwm.steering_idle()
    assert pwm.board.calls == [(3, 1750), (3, 1715)]
    for value in (1550, 1750, 2000, True):
        with pytest.raises(ValueError):
            RasAdapterPWM(motor=True, channel=3, esc_channel=4, center_us=value)
