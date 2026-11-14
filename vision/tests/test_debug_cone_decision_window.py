"""Debug obstacle commands use the same finite calculation window as race."""
import pytest

from carvision.cone_command import ConeCommandSender
from test_cone_command_cadence import FakeActuator, intent
from test_race_sender_cadence import advance


def test_slow_fresh_debug_results_do_not_expire_between_calculations():
    clock = [0.]
    actuator = FakeActuator(lambda: clock[0])
    sender = ConeCommandSender(actuator, clock=lambda: clock[0], threaded=False)
    try:
        for i in range(6):
            completed = .373*(i+1)
            advance((clock, actuator, sender), completed)
            assert sender.fault is None
            sender.send(intent(completed-.373, fid=i+1))
            assert actuator.guard.fault is None
            assert sender.timing()['source_valid_until_s'] == pytest.approx(completed+.127)
            assert sender.timing()['decision_valid_until_s'] == pytest.approx(completed+.75)
        assert {row['action'] for row in actuator.rows} == {'search'}
        advance((clock, actuator, sender), completed+.75)
        assert sender.fault and actuator.rows[-1]['action'] == 'brake'
    finally:
        sender.close()


def test_same_capture_with_different_frame_number_cannot_renew_or_change_output():
    clock = [1.]
    actuator = FakeActuator(lambda: clock[0])
    sender = ConeCommandSender(actuator, clock=lambda: clock[0], threaded=False)
    try:
        sender.send(intent(1., fid=1))
        deadline, count = sender.deadline, len(actuator.rows)
        clock[0] = 1.1
        result = sender.send({**intent(1., fid=2), 'steering_us': 1870})
        assert result['cached_observation_ignored']
        assert sender.deadline == deadline and len(actuator.rows) == count
    finally:
        sender.close()


def test_ignored_cached_forward_keeps_prepared_neutral_heartbeat_alive():
    clock = [0.]
    actuator = FakeActuator(lambda: clock[0])
    sender = ConeCommandSender(actuator, clock=lambda: clock[0], threaded=False)
    try:
        sender.send(intent())
        sender.send({'action':'neutral','esc_us':1500,'steering_us':1610,
                     'control_profile':'cone','page_deadline_s':2.})
        advance((clock, actuator, sender), .6)
        result=sender.send({**intent(), 'fresh_control':False})
        assert result['cached_observation_ignored'] and sender.neutral_slot
        sender.refresh_page_deadline(3.)
        advance((clock, actuator, sender), 1.2)
        assert actuator.guard.fault is None and actuator.guard.esc == 1500
        assert all(row['esc_us']==1500 for row in actuator.rows[1:])
    finally:
        sender.close()


def test_explicit_brake_is_sent_even_when_derived_from_the_same_frame():
    clock = [0.]
    actuator = FakeActuator(lambda: clock[0])
    sender = ConeCommandSender(actuator, clock=lambda: clock[0], threaded=False)
    try:
        sender.send(intent())
        clock[0]=.1
        sender.send({**intent(),'action':'brake','esc_us':1400,'fresh_control':False})
        assert actuator.rows[-1]['action']=='brake' and sender.slot is None
        actuator.guard.tick(.221)
        assert actuator.guard.esc==1500
    finally:
        sender.close()


@pytest.mark.parametrize('age', [.5, .501, 1.])
def test_debug_decision_window_never_makes_stale_input_acceptable(age):
    actuator = FakeActuator(lambda: 1.)
    sender = ConeCommandSender(actuator, clock=lambda: 1., threaded=False)
    try:
        with pytest.raises(RuntimeError, match='图像采集时间'):
            sender.send(intent(1.-age))
        assert all(row['esc_us'] <= 1500 for row in actuator.rows)
    finally:
        sender.close()
