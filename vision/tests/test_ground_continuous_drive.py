"""Long operator-held sessions retain short output leases and explicit authorization."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import carvision.pi5_pwm as pwm_module

from carvision.manual_drive import DriveError, ManualDrive
from carvision.pi5_pwm import Pi5ContinuousBackend
from test_ground_held_drive import Clock, Hardware, command, review


def continuous_review():
    return {**review(), 'ground_continuous_requested_by_user': True,
            'disconnect_stop_physically_verified': True, 'steering_physically_verified': True,
            'drivetrain_issue_resolved_reported_by_user': True, 'manual_session_limit_s': None,
            'hold_limit_s': None, 'reverse_revalidation_prepared': True}


def ready():
    clock = Clock()
    backend = Hardware(clock)
    backend.manual_continuous_reviewed = True
    backend.manual_session_limit_s = backend.manual_motion_hold_max_s = None
    backend.reverse_available = True
    drive = ManualDrive(backend, clock=clock, settling_s=0, session_s=None,
                        preparation_s=120, ground_held=True, ground_continuous=True)
    drive.request('enable', {'client': 'ground-test', 'bench_ready': True})
    drive.tick()
    return drive, backend, clock


def test_continuous_scope_cannot_be_enabled_by_old_bounded_review(tmp_path):
    with pytest.raises(ValueError):
        Pi5ContinuousBackend(tmp_path, tmp_path/'out', 'current', review())
    for field in ('ground_continuous_requested_by_user', 'neutral_physically_verified',
                  'spotter_can_cut_power_confirmed_by_user', 'drivetrain_issue_resolved_reported_by_user'):
        with pytest.raises(ValueError):
            Pi5ContinuousBackend(tmp_path, tmp_path/'out', 'current', {**continuous_review(), field: False})
    valid = Pi5ContinuousBackend(tmp_path, tmp_path/'out', 'current', continuous_review())
    assert valid.steering_available and valid.motor_available and valid.reverse_available
    assert valid.manual_session_limit_s is valid.manual_motion_hold_max_s is None


def test_unreviewed_hardware_cannot_bypass_bounds_with_continuous_option():
    clock = Clock()
    with pytest.raises(ValueError):
        ManualDrive(Hardware(clock), clock=clock, session_s=None, ground_held=True, ground_continuous=True)


@pytest.mark.parametrize('center,boundary', [(1650, False), (1550, True)])
def test_continuous_backend_starts_only_s3_s4_guardian_with_fresh_baseline(tmp_path, monkeypatch, center, boundary):
    valid = continuous_review()
    valid.update(steering_center_us=center, steering_boundary_center_trial_requested_by_user=boundary)
    valid['baseline_record'] = 'baseline.json'
    (tmp_path/'baseline.json').write_text(json.dumps({'boot_id':'current', 'duration_s':30,
        'all_power_flags_zero':True, 'boot_unchanged':True,
        'samples':[{'boot_id':'current', 'power_flags':0} for _ in range(30)]}))
    monkeypatch.setattr(pwm_module, 'inspect_pi5', lambda:{'boot_id':'current','power_flags':0})
    commands = []
    monkeypatch.setattr(pwm_module.subprocess, 'Popen', lambda command, **kwargs:
                        commands.append(command) or SimpleNamespace())
    backend = Pi5ContinuousBackend(tmp_path, tmp_path/'out', 'current', valid)
    monkeypatch.setattr(backend, 'read_reply', lambda **kwargs:{'ready':True})
    monkeypatch.setattr(backend, 'check', lambda:None)
    backend.open()
    backend.log.close()
    command_line = commands[0]
    assert '--continuous-manual' in command_line
    assert command_line[command_line.index('--steering-channel')+1] == '3'
    assert command_line[command_line.index('--esc-channel')+1] == '4'
    assert command_line[command_line.index('--steering-center-us')+1] == str(center)
    assert ('--steering-boundary-center-trial' in command_line) is boundary


def test_fresh_input_runs_past_all_previous_session_and_hold_limits():
    drive, backend, clock = ready()
    assert drive.deadline is None
    # An idle operator can start after hours, then hold for longer than the
    # former 60 / 600 / 800 / 810-second cutoffs without recreating a worker.
    for step in range(1, 72001):
        clock.now = step*.1
        drive.tick()
    for seq in range(1, 8203):
        clock.now = 7200+(seq-1)*.1
        command(drive, seq, 'forward', 'right')
        drive.tick()
    assert drive.mode == 'enabled' and backend.pwm.pulse == 1575
    assert drive.turn == 'right' and drive.status()['test_session_s'] is None
    assert all(.02 <= seconds <= .2 for _, seconds in backend.segments)
    command(drive, 8203, 'stop', 'right')
    assert backend.pwm.pulse == 1500 and drive.turn == 'right'
    command(drive, 8204, 'stop', 'center')
    assert drive.turn == 'center'


def test_reverse_retains_brake_neutral_gap_then_continuous_reverse():
    drive, backend, clock = ready()
    for seq in range(1, 102):
        clock.now = (seq-1)*.05
        command(drive, seq, 'reverse', 'left')
        drive.tick()
        if .35 <= clock.now < 1.5:
            assert backend.pwm.pulse == 1500
    assert drive.stage == 'reverse' and backend.pwm.pulse == 1300
    assert drive.turn == 'left'
    drive.request('stop', {'stop_source': 'key_release'})
    assert backend.pwm.pulse == 1500
    assert all(seconds <= .3 for _, seconds in backend.segments)


def test_lost_input_stops_and_does_not_resume_on_late_held_keys():
    drive, backend, clock = ready()
    command(drive, 1)
    assert drive.status()['lease_ms'] == 500
    for moment in (.05, .1, .15, .2, .25, .3, .35, .4, .45, .499):
        clock.now = moment
        drive.tick()
        assert drive.status()['mode'] == 'enabled'
        assert backend.pwm.pulse == 1575
    clock.now = .5
    drive.tick()
    assert drive.mode == 'emergency' and backend.pwm.pulse == 1500
    with pytest.raises(DriveError):
        command(drive, 2)
    assert backend.pwm.pulse == 1500


def test_cellular_commands_can_arrive_400ms_apart_without_stopping():
    drive, backend, clock = ready()
    command(drive, 1)
    for step in range(1, 13):
        clock.now = step / 10
        if step % 4 == 0:
            command(drive, 1 + step // 4)
        drive.tick()
        assert drive.mode == 'enabled' and backend.pwm.pulse == 1575
    assert all(seconds <= .2 for _, seconds in backend.segments)
    assert ManualDrive().status()['lease_ms'] == 200


@pytest.mark.parametrize('action', ['stop', 'emergency'])
def test_release_and_emergency_do_not_wait_for_500ms(action):
    drive, backend, clock = ready()
    command(drive, 1)
    for moment in (.1, .2, .3):
        clock.now = moment
        drive.tick()
    clock.now = .31
    drive.request(action, {'stop_source': 'key_release' if action == 'stop' else 'keyboard_space'})
    assert backend.pwm.pulse == 1500
    if action == 'emergency':
        with pytest.raises(DriveError): command(drive, 2)
    else:
        command(drive, 2)
        drive.tick()
        assert backend.pwm.pulse == 1500 and drive.blocked


def test_independent_guardian_stops_when_control_thread_stalls():
    drive, backend, clock = ready()
    command(drive, 1)
    clock.now = .26
    backend.guard.tick()
    assert backend.pwm.pulse == 1500 and backend.guard.fault


def test_explicit_emergency_can_be_reenabled_without_a_new_round():
    drive, backend, clock = ready()
    command(drive, 1)
    drive.request('emergency', {'stop_source': 'keyboard_escape'})
    assert backend.pwm.pulse == 1500
    drive.request('reset', {})
    drive.request('enable', {'client': 'ground-test', 'bench_ready': True})
    drive.tick()
    command(drive, 2, 'stop', 'center')
    command(drive, 3)
    drive.tick()
    assert drive.mode == 'enabled' and backend.pwm.pulse == 1575 and drive.deadline is None


@pytest.mark.parametrize('continuous,expected_replies', [(False, 1), (True, 2)])
@pytest.mark.parametrize('wifi_disabled', [False, True])
def test_guardian_keeps_legacy_limit_but_accepts_long_manual_control_after_800_seconds(
        tmp_path, monkeypatch, capsys, continuous, expected_replies, wifi_disabled):
    clock = Clock()
    monkeypatch.setattr(pwm_module.time, 'monotonic', clock)
    class PWM:
        def __init__(self, **kwargs): pass
        def open(self): clock.now = 850
        def neutral(self): pass
        def steering_idle(self): pass
        def close(self): return []
    monkeypatch.setattr(pwm_module, 'RasAdapterPWM', PWM)
    monkeypatch.setattr(pwm_module.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(pwm_module.signal, 'SIGHUP', 1, raising=False)
    original_read = Path.read_text
    def read(path, *args, **kwargs):
        if path.as_posix() == '/proc/sys/kernel/random/boot_id': return 'current'
        if path.as_posix().startswith('/sys/class/net/'):
            if path.name == 'flags':
                return '0x1002' if wifi_disabled and '/wlan0/' in path.as_posix() else '0x1003'
            if wifi_disabled and '/wlan0/' in path.as_posix():
                return '1'  # stale carrier must not enable an administratively disabled radio
            return '1' if '/usb0/' in path.as_posix() else '0'
        return original_read(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', read)
    def output(command, **kwargs):
        if command[-1] == 'get_throttled': return 'throttled=0x0'
        if command[-1] == 'measure_temp': return "temp=45.0'C"
        return json.dumps([{'dev': 'usb0'}])
    monkeypatch.setattr(pwm_module.subprocess, 'check_output', output)
    monkeypatch.setattr(pwm_module.sys, 'stdin', SimpleNamespace(fileno=lambda: 42))
    monkeypatch.setattr(pwm_module.select, 'select', lambda *args: ([42], [], []))
    monkeypatch.setattr(pwm_module.os, 'read', lambda *args: b'{"action":"shutdown"}\n')
    args = SimpleNamespace(motor=True, steering=True, steering_channel=3, esc_channel=4,
        steering_center_us=1715, neutral_only=False, expected_boot_id='current',
        continuous_manual=continuous, result=tmp_path/'guardian.json')
    pwm_module.guard_main(args)
    replies = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert len(replies) == expected_replies and replies[0]['ready'] is True
    assert json.loads(args.result.read_text())['cleanup_errors'] == []
