"""Production race dispatch with slow fresh observations and fake actuators."""
from types import SimpleNamespace

import pytest

from carvision.cone_command import ConeCommandSender
from carvision.cone_continuous import ContinuousConeController
from carvision.cone_low_speed import trial_settings
from carvision.race_flow_console import RaceFlowDrive, start_readiness
from carvision.runtime_timing import RACE_CONTROL_WINDOW_S
from cone_mission_fixture import ground_nav, exit_nav
from test_cone_turn_controller import lane, settings
from test_race_sender_cadence import GuardActuator, advance, FORWARD, NEUTRAL
from test_race_flow import bundle


def test_obstacle_start_and_recovery_accept_three_fresh_six_hundred_ms_results():
    controller = ContinuousConeController(settings(), 0., initialized=True)
    for i, captured in enumerate((.1, .7, 1.3)):
        result = controller.step(lane(captured, fid=i), captured+.373)
        assert result['esc_us'] == (1600 if i == 2 else 1500)
    controller.recover_control(1.8)
    for i, captured in enumerate((1.9, 2.5, 3.1), 3):
        result = controller.step(lane(captured, fid=i), captured+.373)
        assert result['esc_us'] >= 1570 if i == 5 else result['esc_us'] == 1500


def test_slow_fresh_cone_evidence_can_register_and_complete_current_pixel_pass():
    controller = ContinuousConeController(settings(), 0., initialized=True)
    for t in (.1, .5, .9):
        controller.step(lane(t), t+.373)
    identifier = 'secondary-slow'
    for i, y in enumerate((.55, .63, .71, .73, .76)):
        t = 1.3 + i*.4
        controller.step(ground_nav(t, identifier, y=y), t+.373)
    assert controller.mission.registered('secondary', identifier)
    assert controller.pass_evidence['visual_pass_candidate']
    for index in range(3):
        t = 3.3 + index*.4
        controller.step(exit_nav(t, identifier, index=index), t+.373)
    assert identifier in controller.passed_ids
    assert not controller.mission.records[('secondary', identifier)]['physical_tail_verified']


def test_long_gap_and_expired_pixels_still_cannot_accumulate_lane_confirmation():
    controller = ContinuousConeController(settings(), 0., initialized=True)
    for i, t in enumerate((.1, 1.1, 2.1)):
        assert controller.step(lane(t, fid=i), t)['esc_us'] == 1500
        assert controller.lane_evidence.count == 1
    with pytest.raises(ValueError, match='画面'):
        controller.step(lane(2.2, fid=3), 2.701)


@pytest.mark.parametrize('throttle',[1501,1540,1569])
def test_debug_obstacle_configuration_rejects_nonstarting_forward_setpoints(throttle):
    with pytest.raises(ValueError,match='1570'):
        trial_settings(settings(),{'test_forward_us':throttle})


@pytest.mark.parametrize('stage, age, period, frame_limit', [
    ('cones', .373, .373, .5), ('parking_entry', .20, .60, .25),
])
def test_real_race_dispatch_does_not_brake_before_the_next_fresh_result(stage, age, period, frame_limit):
    clock = [0.]
    actuator = GuardActuator(lambda: clock[0])
    sender = ConeCommandSender(actuator, clock=lambda: clock[0], threaded=False)
    drive = RaceFlowDrive.__new__(RaceFlowDrive)
    drive.offline_competition = True
    drive.bundle = {'flow': {'max_run_s': 30.}, 'brake': {'brake_pulse_s': .35}}
    drive.controller = SimpleNamespace(stage=stage, started=0.,
        forward_evidence_until=lambda sample, previous: sample['captured_s']+frame_limit)
    rig = (clock, actuator, sender)
    try:
        for i in range(6):
            completed = age + i*period
            advance(rig, completed)
            assert not sender.race_interrupted()
            drive._send_intent(sender, FORWARD, {'captured_s': completed-age}, stage)
            assert actuator.guard.fault is None and actuator.guard.esc == 1600
            assert sender.timing()['source_valid_until_s'] == pytest.approx(completed-age+frame_limit)
            assert sender.timing()['decision_valid_until_s'] == pytest.approx(completed+RACE_CONTROL_WINDOW_S)
        assert not any(row['action'] == 'brake' for row in actuator.commands)
        advance(rig, completed+RACE_CONTROL_WINDOW_S)
        assert sender.race_interrupted() and actuator.guard.esc == 1400
    finally:
        sender.close()


def test_cached_forward_does_not_extend_an_accepted_decision():
    clock = [0.]
    actuator = GuardActuator(lambda: clock[0])
    sender = ConeCommandSender(actuator, clock=lambda: clock[0], threaded=False)
    try:
        sender.send_race(FORWARD, run_until=10., evidence_until=.5)
        original = sender.race_deadline
        advance((clock, actuator, sender), .4)
        sender.send_race({**FORWARD, 'fresh_control': False}, run_until=10., evidence_until=.5)
        assert sender.race_deadline == original
        advance((clock, actuator, sender), original)
        assert sender.race_interrupted()
    finally:
        sender.close()


def test_formal_race_checks_parking_capability_before_starting(bundle):
    parking = {'ready': False, 'missing': ['unverified motion calibration']}
    full = start_readiness(bundle, parking, full_race=True)
    assert not full['ready'] and full['missing'] == parking['missing']
    assert full['scope'] == 'through_cones' and not full['parking_ready']
    assert start_readiness(bundle, parking)['ready']  # Noncompetition prefix trials.
    assert start_readiness(bundle, {'ready': True, 'missing': []}, full_race=True)['ready']
