"""Only an explicitly requested S3 1550-us reference can use the range boundary."""
import pytest

from carvision.pi5_pwm import Pi5BenchBackend, RasAdapterPWM


def review(**changes):
    return dict(model='Raspberry Pi 5 Model B Rev 1.0', boot_id='test-boot',
                wheels_raised_confirmed_by_user=True, esc_off_confirmed_by_user=True,
                steering_backend='rasadapter5a_uart', steering_channel=3,
                steering_center_us=1550, **changes)


def test_unrequested_boundary_reference_stays_rejected(tmp_path):
    with pytest.raises(ValueError, match='invalid observed'):
        Pi5BenchBackend(tmp_path, tmp_path/'out', 'test-boot', review(), steering_only=True)
    with pytest.raises(ValueError, match='inside the observed range'):
        RasAdapterPWM(motor=False, channel=3, center_us=1550)


def test_requested_s3_boundary_is_held_and_restored_without_opening_uart(tmp_path):
    backend = Pi5BenchBackend(tmp_path, tmp_path/'out', 'test-boot',
                             review(steering_boundary_center_trial_requested_by_user=True), steering_only=True)
    assert backend.steering_center_us == 1550 and not backend.motor_available
    pwm = RasAdapterPWM(motor=False, channel=3, center_us=1550, boundary_center_trial=True)
    calls = []
    pwm.board.set_position = lambda channel, pulse, seconds: calls.append((channel, pulse))
    pwm.turn(1560)
    pwm.steering_idle()
    assert calls == [(3, 1560), (3, 1550)]
    assert pwm.board.fd is None and backend.process is None


@pytest.mark.parametrize('center', [1549, 1551, 1750, 1751, True, 1550.0])
def test_special_trial_cannot_authorize_a_different_reference(tmp_path, center):
    data = review(steering_boundary_center_trial_requested_by_user=True)
    data['steering_center_us'] = center
    with pytest.raises(ValueError):
        Pi5BenchBackend(tmp_path, tmp_path/'out', 'test-boot', data, steering_only=True)
    with pytest.raises(ValueError):
        RasAdapterPWM(motor=False, channel=3, center_us=center, boundary_center_trial=True)


@pytest.mark.parametrize('changes', [{'steering_channel':1}, {'steering_backend':'rp1_hardware_pwm'}])
def test_boundary_trial_cannot_change_the_identified_steering_channel(tmp_path, changes):
    data = {**review(steering_boundary_center_trial_requested_by_user=True), **changes}
    with pytest.raises(ValueError):
        Pi5BenchBackend(tmp_path, tmp_path/'out', 'test-boot', data, steering_only=True)
