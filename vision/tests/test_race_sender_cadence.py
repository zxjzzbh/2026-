"""Slow perception and finite race commands, using the real guard without I/O."""
import pytest

from carvision.brake_output import PulseGuard
from carvision.cone_command import COMMAND_PERIOD_S, ConeCommandSender
from carvision.runtime_timing import RACE_CONTROL_WINDOW_S


FORWARD = {'action': 'search', 'esc_us': 1600, 'steering_us': 1610}
NEUTRAL = {'action': 'neutral', 'esc_us': 1500, 'steering_us': 1610}
BRAKE = {'action': 'brake', 'esc_us': 1400, 'steering_us': 1610}


class GuardActuator:
    run = False

    def __init__(self, clock):
        self.clock = clock
        self.commands = []
        self.closed = False
        self.fail = False
        self.guard = PulseGuard(
            {'pwm': {'neutral_us': 1500, 'steering_center_us': 1610,
                     'search_us': 1600, 'creep_us': 1570, 'brake_us': 1400},
             'actuator_lease_s': .5, 'brake_pulse_s': .35, 'brake_release_s': .1},
            lambda *_: None, mode='F/B', forward_limit_us=1625)

    def send(self, intent):
        if self.fail:
            raise RuntimeError('test transport failure')
        now = self.clock()
        command = {**intent, 'sequence': len(self.commands), 'sent_s': now}
        self.guard.accept(command, now)
        self.commands.append(command)

    def close(self):
        self.closed = True
        self.guard.abort('closed', self.clock())


@pytest.fixture
def rig():
    clock = [0.]
    actuator = GuardActuator(lambda: clock[0])
    sender = ConeCommandSender(actuator, clock=lambda: clock[0], threaded=False)
    yield clock, actuator, sender
    sender.close()


def advance(rig, until):
    clock, actuator, sender = rig
    while clock[0] < until - 1e-9:
        clock[0] = min(round(clock[0] + .025, 9), until)
        actuator.guard.tick(clock[0])
        sender.tick()


def test_six_hundred_ms_vision_period_keeps_five_hundred_ms_guard_alive(rig):
    clock, actuator, sender = rig
    sender.send_race(FORWARD, run_until=10., evidence_until=.3)
    for result_time in (.6, 1.2, 1.8):
        advance(rig, result_time)
        assert not sender.race_interrupted()
        assert actuator.guard.fault is None and actuator.guard.esc == 1600
        sender.send_race(FORWARD, run_until=10., evidence_until=result_time + .3)
    gaps = [b['sent_s'] - a['sent_s'] for a, b in zip(actuator.commands, actuator.commands[1:])]
    assert max(gaps) <= COMMAND_PERIOD_S + 1e-9
    assert sender.timing()['actuator_lease_s'] == .5
    assert {row['action'] for row in actuator.commands} == {'search'}


def test_background_replay_cannot_extend_the_control_deadline_and_brake_is_finite(rig):
    clock, actuator, sender = rig
    sender.send_race(FORWARD, run_until=10., evidence_until=.3)
    deadline = sender.race_deadline
    assert deadline == pytest.approx(RACE_CONTROL_WINDOW_S)
    advance(rig, deadline)
    assert sender.race_interrupted() and actuator.guard.esc == 1400
    advance(rig, deadline + .34)
    assert actuator.guard.esc == 1400
    assert sum(row['action'] == 'brake' for row in actuator.commands) == 1
    advance(rig, deadline + .36)
    assert actuator.guard.esc == 1500 and actuator.guard.fault is None
    assert sender.send_race(FORWARD, run_until=10., evidence_until=clock[0] + .3) is None
    sender.send_race(NEUTRAL, run_until=10., recovered=True)
    assert not sender.race_interrupted()
    sender.send_race(FORWARD, run_until=10., evidence_until=clock[0] + .3)
    assert actuator.guard.esc == 1600


def test_requested_brake_is_not_renewed_by_ticks_or_repeated_decisions(rig):
    clock, actuator, sender = rig
    sender.send_race(BRAKE, run_until=10.)
    for until in (.1, .2, .3):
        advance(rig, until)
        sender.send_race(BRAKE, run_until=10.)
    advance(rig, .36)
    assert actuator.guard.esc == 1500
    assert [row['action'] for row in actuator.commands] == ['brake', 'neutral']
    assert sender.race_brake_until is None


@pytest.mark.parametrize('evidence', [None, '0.5', float('nan'), float('inf'), -.1, .1])
def test_invalid_or_expired_evidence_cannot_refresh_forward(rig, evidence):
    clock, actuator, sender = rig
    sender.send_race(FORWARD, run_until=10., evidence_until=.3)
    advance(rig, .1)
    sender.send_race(FORWARD, run_until=10., evidence_until=evidence)
    assert sender.race_interrupted() and sender.race_deadline is None
    assert actuator.guard.esc == 1400
    advance(rig, .46)
    assert actuator.guard.esc == 1500
    assert sum(row['action'] == 'brake' for row in actuator.commands) == 1


def test_cone_and_metric_parking_deadline_still_uses_captured_evidence(rig):
    _, actuator, sender = rig
    sender.send_race(FORWARD, run_until=10., evidence_until=.12, expire_at_evidence=True)
    assert sender.race_deadline == .12
    advance(rig, .12)
    assert sender.race_interrupted() and actuator.guard.esc == 1400


def test_run_budget_caps_the_longer_control_window(rig):
    _, actuator, sender = rig
    sender.send_race(FORWARD, run_until=.2, evidence_until=.3)
    advance(rig, .2)
    assert sender.race_interrupted() and actuator.guard.esc == 1400
    advance(rig, .56)
    assert actuator.guard.esc == 1500
    count = len(actuator.commands)
    advance(rig, .7)
    assert len(actuator.commands) == count


def test_explicit_neutral_cancels_forward_replay_immediately(rig):
    clock, actuator, sender = rig
    sender.send_race(FORWARD, run_until=10., evidence_until=.3)
    advance(rig, .1)
    sender.send_race(NEUTRAL, run_until=10.)
    stopped_at = len(actuator.commands) - 1
    advance(rig, 1.)
    assert sender.race_deadline is None and not sender.race_interrupted()
    assert all(row['esc_us'] == 1500 for row in actuator.commands[stopped_at:])
    sender.close()
    count = len(actuator.commands)
    advance(rig, 1.2)
    assert actuator.closed and len(actuator.commands) == count


def test_transport_failure_stops_replay_and_is_reported(rig):
    _, actuator, sender = rig
    sender.send_race(FORWARD, run_until=10., evidence_until=.3)
    actuator.fail = True
    advance(rig, .05)
    assert 'test transport failure' in sender.fault
    count = len(actuator.commands)
    advance(rig, .2)
    assert len(actuator.commands) == count
    with pytest.raises(RuntimeError, match='test transport failure'):
        sender.race_interrupted()
