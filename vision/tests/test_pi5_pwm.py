import pytest

from carvision.pi5_pwm import GuardState, validate_review
from carvision.pi_control import select_pwm_chip


def test_pi5_selects_rp1_even_when_pwm_chip_number_changes():
    chips = [{'path': '/sys/class/pwm/pwmchip7', 'compatible': 'raspberrypi,rp1-pwm,', 'channels': 4},
             {'path': '/sys/class/pwm/pwmchip0', 'compatible': 'brcm,bcm2835-pwm,', 'channels': 2}]
    mux = '12: GPIO12 = PWM0_CHAN0\n13: GPIO13 = PWM0_CHAN1'
    assert select_pwm_chip(b'Raspberry Pi 5 Model B', mux, chips)['path'].endswith('pwmchip7')
    with pytest.raises(RuntimeError, match='pin mux'):
        select_pwm_chip(b'Raspberry Pi 5 Model B', 'GPIO12 = none\nGPIO13 = none', chips)
    with pytest.raises(RuntimeError, match='unique'):
        select_pwm_chip(b'Raspberry Pi 5 Model B', mux, chips, '/sys/class/pwm/pwmchip0')


class Clock:
    now = 0
    def __call__(self):
        return self.now


class PWM:
    def __init__(self):
        self.calls = []
    def neutral(self):
        self.calls.append(('neutral',))
    def steering_idle(self):
        self.calls.append(('steering_idle',))
    def motion(self, pulse):
        self.calls.append(('motion', pulse))
    def turn(self, pulse):
        self.calls.append(('turn', pulse))


def prepared(**extra):
    return dict(model='Raspberry Pi 5 Model B Rev 1.0', boot_id='current',
                wheels_raised_confirmed_by_user=True, esc_off_confirmed_by_user=True, **extra)


def test_pi4_physical_evidence_cannot_enable_pi5_motor():
    review = prepared()
    validate_review(review, 'current', neutral_only=True)
    with pytest.raises(ValueError, match='neutral must first'):
        validate_review(review, 'current', motor_only=True)
    with pytest.raises(ValueError, match='fresh Pi 5'):
        validate_review(prepared(neutral_physically_verified=True), 'new-boot', motor_only=True)


def test_combined_needs_isolated_pi5_observations():
    with pytest.raises(ValueError, match='isolated steering and stop'):
        validate_review(prepared(neutral_physically_verified=True), 'current', combined=True)
    validate_review(prepared(neutral_physically_verified=True, steering_physically_verified=True,
                            stop_physically_verified=True), 'current', combined=True)


def test_rasadapter_never_guesses_a_servo_channel():
    review = prepared(steering_backend='rasadapter5a_uart')
    with pytest.raises(ValueError, match='actual RasAdapter'):
        validate_review(review, 'current', steering_only=True)
    review['steering_channel'] = 3
    validate_review(review, 'current', steering_only=True)


def test_first_reverse_trial_requires_this_board_stop_and_neutral_evidence():
    review = prepared(reverse_revalidation_prepared=True)
    with pytest.raises(ValueError, match='current neutral and stop'):
        validate_review(review, 'current', neutral_only=True)
    review.update(neutral_physically_verified=True, stop_physically_verified=True)
    validate_review(review, 'current', motor_only=True)


def test_pi5_ground_scope_requires_ground_preparation_and_never_unlocks_continuous_drive():
    review = prepared(neutral_physically_verified=True, steering_physically_verified=True,
                      stop_physically_verified=True, disconnect_stop_physically_verified=True,
                      esc_backend='rasadapter5a_uart', esc_channel=4,
                      steering_backend='rasadapter5a_uart', steering_channel=3,
                      ground_short_requested_by_user=True, wheels_on_ground_confirmed_by_user=True,
                      clear_area_confirmed_by_user=True, power_switch_in_reach_confirmed_by_user=True,
                      continuous_ground_driving_enabled=False)
    review['wheels_raised_confirmed_by_user'] = False
    validate_review(review, 'current', ground_short=True)
    for field in ('clear_area_confirmed_by_user', 'power_switch_in_reach_confirmed_by_user',
                  'disconnect_stop_physically_verified', 'wheels_on_ground_confirmed_by_user'):
        changed = {**review, field:False}
        with pytest.raises(ValueError, match='ground preparation'):
            validate_review(changed, 'current', ground_short=True)
    with pytest.raises(ValueError, match='ground preparation'):
        validate_review({**review, 'continuous_ground_driving_enabled':True}, 'current', ground_short=True)
    with pytest.raises(ValueError, match='fresh Pi 5'):
        validate_review(review, 'other-boot', ground_short=True)


def test_neutral_only_rejects_every_movement_before_output():
    clock, pwm = Clock(), PWM()
    state = GuardState(pwm, clock=clock, neutral_only=True)
    for request in ({'action': 'motion', 'pulse': 1575, 'seconds': .8},
                    {'action': 'steering', 'pulse': 1750}):
        with pytest.raises(ValueError):
            state.request(request)
    assert not pwm.calls


def test_guardian_stops_when_control_parent_stalls_and_latches_fault():
    clock, pwm = Clock(), PWM()
    state = GuardState(pwm, clock=clock)
    state.request({'action': 'motion', 'pulse': 1575, 'seconds': .8})
    clock.now = .26
    state.tick()
    assert pwm.calls[-2:] == [('neutral',), ('steering_idle',)]
    with pytest.raises(RuntimeError, match='lease expired'):
        state.request({'action': 'motion', 'pulse': 1575, 'seconds': .8})


def test_heartbeat_never_extends_finite_motion_deadline():
    clock, pwm = Clock(), PWM()
    state = GuardState(pwm, clock=clock)
    state.request({'action': 'motion', 'pulse': 1575, 'seconds': .8})
    for now in (.2, .4, .6, .8):
        clock.now = now
        state.request({'action': 'check'})
    assert pwm.calls == [('motion', 1575), ('neutral',)]


@pytest.mark.parametrize('duration', [.81, -1, float('nan'), float('inf'), True])
def test_bad_duration_never_reaches_output(duration):
    pwm = PWM()
    state = GuardState(pwm)
    with pytest.raises(ValueError):
        state.request({'action': 'motion', 'pulse': 1575, 'seconds': duration})
    assert not pwm.calls


def test_explicit_stop_cancels_the_remaining_motion_stage():
    clock, pwm = Clock(), PWM()
    state = GuardState(pwm, clock=clock)
    state.request({'action': 'motion', 'pulse': 1575, 'seconds': .8})
    state.request({'action': 'neutral'})
    clock.now = .1
    state.request({'action': 'check'})
    assert state.motion_until is None
    assert pwm.calls == [('motion', 1575), ('neutral',)]
