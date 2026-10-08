"""Position holding precedes traction without overriding turns or stop leases."""
import pytest

from carvision.manual_drive import ManualDrive
from carvision.pi5_pwm import Pi5BenchBackend


class Clock:
    now = 0.

    def __call__(self):
        return self.now


def backend(tmp_path, *, motor_only=False):
    review = dict(model='Raspberry Pi 5 Model B Rev 1.0', boot_id='test-boot',
                  wheels_raised_confirmed_by_user=True, esc_off_confirmed_by_user=True,
                  neutral_physically_verified=True, steering_physically_verified=True,
                  stop_physically_verified=True, esc_backend='rasadapter5a_uart', esc_channel=4,
                  steering_backend='rasadapter5a_uart', steering_channel=3, steering_center_us=1715)
    result = Pi5BenchBackend(tmp_path, tmp_path/'out', 'test-boot', review,
                            motor_only=motor_only, combined=not motor_only)
    result.process = object()  # owned guardian; all RPCs below are simulated
    calls = []
    result.rpc = lambda action, **values: calls.append((action, values))
    return result, calls


def ready(tmp_path):
    clock = Clock()
    output, calls = backend(tmp_path)
    drive = ManualDrive(output, clock=clock, settling_s=0)
    drive.request('enable', {'client': 'hold-test', 'bench_ready': True})
    drive.tick()
    calls.clear()
    return drive, output, calls, clock


def command(drive, seq, turn='center'):
    drive.request('command', {'client': 'hold-test', 'sequence': seq,
                              'motor': 'forward', 'steering': turn})
    drive.tick()


def test_straight_from_idle_establishes_hold_before_motor_without_repeated_uart_frames(tmp_path):
    drive, output, calls, clock = ready(tmp_path)
    for seq in range(1, 14):
        clock.now = (seq-1)*.05
        command(drive, seq)
    commands = [c for c in calls if c[0] in ('steering', 'motion', 'steering_idle')]
    assert commands == [('steering', {'pulse': 1715}),
                        ('motion', {'pulse': 1575, 'seconds': .8})]
    assert output.steering_active and drive.pulse == 1575


def test_new_forward_after_idle_reestablishes_center_hold(tmp_path):
    output, calls = backend(tmp_path)
    output.motion(1575, .2)
    output.steering_idle()
    calls.clear()
    output.motion(1575, .2)
    assert calls == [('steering', {'pulse': 1715}),
                     ('motion', {'pulse': 1575, 'seconds': .2})]


@pytest.mark.parametrize('target', [1550, 1750])
def test_motion_does_not_override_an_active_left_or_right_turn(tmp_path, target):
    output, calls = backend(tmp_path)
    output.steering(target)
    calls.clear()
    output.motion(1575, .2)
    assert calls == [('motion', {'pulse': 1575, 'seconds': .2})]


def test_motor_only_scope_never_adds_steering_output(tmp_path):
    output, calls = backend(tmp_path, motor_only=True)
    output.motion(1575, .2)
    assert calls == [('motion', {'pulse': 1575, 'seconds': .2})]


def test_failure_to_establish_hold_prevents_motor_command(tmp_path):
    output, calls = backend(tmp_path)

    def failed(action, **values):
        calls.append((action, values))
        raise RuntimeError('steering acknowledgement failed')

    output.rpc = failed
    with pytest.raises(RuntimeError, match='steering acknowledgement'):
        output.motion(1575, .2)
    assert calls == [('steering', {'pulse': 1715})]
    assert not output.steering_active


@pytest.mark.parametrize('stop', ['release', 'emergency', 'lost_input', 'finite_limit'])
def test_position_hold_preserves_stop_and_timeout_protection(tmp_path, stop):
    drive, output, calls, clock = ready(tmp_path)
    command(drive, 1)
    calls.clear()
    if stop == 'release':
        drive.request('stop', {'stop_source': 'key_release'})
    elif stop == 'emergency':
        drive.request('emergency', {'stop_source': 'keyboard_space'})
    elif stop == 'lost_input':
        clock.now = .21
        drive.tick()
    else:
        for seq in range(2, 18):
            clock.now = (seq-1)*.05
            command(drive, seq)
    assert drive.pulse == 1500 and drive.motor == 'stop'
    assert any(c[0] == 'neutral' for c in calls)
    assert not any(c[0] == 'motion' for c in calls)
