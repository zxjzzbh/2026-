import json
from dataclasses import asdict, replace
from types import SimpleNamespace

import pytest

from carvision.race import Observation, Phase, RaceConfig, RaceController, free_corridor
from carvision.race_workflows import demonstration, race_demo, run_trace


def config():
    return RaceConfig(school="测试大学", team="测试队", vehicle_width_m=0.19)


def obs(t, **changes):
    return replace(Observation(t, source_ok=True, telemetry_valid=True, telemetry_age_s=0,
                               speed_mps=0, lane_valid=True, lane_offset=0, lane_width_m=1.22,
                               task_monitor_valid=True, cone_monitor_valid=True), **changes)


def controller(phase):
    c = RaceController(config())
    c.phase = phase
    return c


def test_complete_synthetic_race_requires_every_stage():
    c = RaceController(config())
    phases, events = set(), []
    for row in demonstration():
        d = c.update(row)
        phases.add(d['phase'])
        events.extend(d['events'])
        assert d['hardware_output'] is False
    required = set(p.value for p in Phase) - {Phase.FAILED.value, Phase.ESTOP.value}
    assert phases == required
    assert c.summary()['payment_bonus_points'] == 5
    assert c.summary()['parking_slot_id'] == 'right'
    assert len([e for e in events if e['event'] == 'play_announcement']) == 1


@pytest.mark.parametrize('setting', [{'crosswalk_hold_s': 3}, {'crosswalk_stop_distance_m': .3},
                                    {'payment_window_s': 31}, {'max_stationary_s': 21},
                                    {'cruise_speed_mps': float('nan')}])
def test_rules_cannot_silently_use_invalid_draft_values(setting):
    with pytest.raises(ValueError):
        replace(config(), **setting).validate()


def test_removed_board_without_seen_board_does_not_launch():
    c = controller(Phase.BOARD)
    for t in [0, .4, .8]:
        assert c.update(obs(t, board_monitor_valid=True, start_board_state='removed'))['speed_mps'] == 0
    assert c.phase == Phase.BOARD
    c.update(obs(1, board_monitor_valid=True, start_board_state='present'))
    c.update(obs(1.1, board_monitor_valid=True, start_board_state='removed'))
    c.update(obs(1.5, board_monitor_valid=True, start_board_state='removed'))
    assert c.phase == Phase.CROSSWALK_APPROACH


def test_unknown_or_gap_resets_blue_board_removal_confirmation():
    c = controller(Phase.BOARD)
    c.update(obs(0, board_monitor_valid=True, start_board_state='present'))
    c.update(obs(.1, board_monitor_valid=True, start_board_state='removed'))
    c.update(obs(.2, board_monitor_valid=False))
    c.update(obs(.4, board_monitor_valid=True, start_board_state='removed'))
    assert c.phase == Phase.BOARD
    assert c.update(obs(1.5, board_monitor_valid=True, start_board_state='removed'))['reason'] == 'observation_gap'
    assert c.phase == Phase.BOARD


def hold_crosswalk(c, start=0, duration=10, ack=True):
    result = None
    for tick in range(round(duration * 10) + 1):
        result = c.update(obs(round(start + tick / 10, 6), crosswalk_distance_m=.2, announcement_done=ack))
    return result


def test_crosswalk_waits_ten_actual_stationary_seconds_and_audio_ack():
    c = controller(Phase.CROSSWALK_HOLD)
    hold_crosswalk(c, duration=9.9)
    assert c.phase == Phase.CROSSWALK_HOLD
    c.update(obs(10, crosswalk_distance_m=.2, announcement_done=True))
    assert c.phase == Phase.LIGHT_APPROACH
    c = controller(Phase.CROSSWALK_HOLD)
    hold_crosswalk(c, duration=10, ack=False)
    assert c.phase == Phase.CROSSWALK_HOLD
    c.update(obs(10.1, crosswalk_distance_m=.2, announcement_done=True))
    assert c.phase == Phase.LIGHT_APPROACH


def test_moving_feedback_resets_crosswalk_timer():
    c = controller(Phase.CROSSWALK_HOLD)
    hold_crosswalk(c, duration=5)
    c.update(obs(5.1, speed_mps=.08, crosswalk_distance_m=.2, announcement_done=True))
    hold_crosswalk(c, start=5.2, duration=9.9)
    assert c.phase == Phase.CROSSWALK_HOLD


def test_ten_second_camera_gap_does_not_count_as_crosswalk_stop():
    c = controller(Phase.CROSSWALK_HOLD)
    c.update(obs(0, crosswalk_distance_m=.2, announcement_done=True))
    assert c.update(obs(10, crosswalk_distance_m=.2, announcement_done=True))['speed_mps'] == 0
    assert c.phase == Phase.CROSSWALK_HOLD


def test_green_without_stop_zone_cannot_bypass_traffic_stop():
    c = controller(Phase.LIGHT_APPROACH)
    c.update(obs(0, traffic_light_state='green'))
    assert c.phase == Phase.LIGHT_APPROACH
    for t in [.1, .2, .4, .6]:
        c.update(obs(t, speed_mps=.1, traffic_zone_entered=True, traffic_stop_distance_m=.5, traffic_light_state='green'))
    assert c.phase == Phase.LIGHT_HOLD
    c.update(obs(.7, traffic_zone_entered=True, traffic_stop_distance_m=.5, traffic_light_state='green'))
    c.update(obs(1.1, traffic_zone_entered=True, traffic_stop_distance_m=.5, traffic_light_state='green'))
    assert c.phase == Phase.CONES


def test_unknown_light_never_becomes_green_after_countdown():
    c = controller(Phase.LIGHT_HOLD)
    for i in range(121):
        assert c.update(obs(i / 10, traffic_zone_entered=True, traffic_stop_distance_m=.6))['speed_mps'] == 0
    assert c.phase == Phase.LIGHT_HOLD


@pytest.mark.parametrize('position', [-.3, 0, .3])
def test_cone_clearance_uses_observed_position(position):
    cone = {'lateral_m': position, 'forward_m': 1, 'radius_m': .04}
    goal = free_corridor(1.22, [cone], config())
    assert goal is not None
    assert abs(goal - position) > .04 + .19 / 2 + .08
    assert abs(goal) < 1.22 / 2 - .19 / 2 - .08


def test_unknown_or_narrow_corridor_stops():
    assert free_corridor(None, [], config()) is None
    assert free_corridor(.25, [], config()) is None
    assert free_corridor(1.22, [{'lateral_m': 0, 'forward_m': 1, 'radius_m': 1}], config()) is None
    c = controller(Phase.CONES)
    assert c.update(obs(0, cone_monitor_valid=False))['speed_mps'] == 0


def parking(t, clear='right', **changes):
    slots = [{'id': 'left', 'center_lateral_m': -.3, 'availability': 'clear' if clear == 'left' else 'blocked'},
             {'id': 'right', 'center_lateral_m': .3, 'availability': 'clear' if clear == 'right' else 'blocked'}]
    return obs(t, parking_zone_visible=True, parking_geometry_valid=True,
               parking_slots=slots, parking_remaining_m=.4, **changes)


def test_duplicate_cone_ids_do_not_count_as_two_cones():
    c = controller(Phase.CONES)
    d = c.update(replace(parking(0), passed_cone_ids=['cone-1', 'cone-1']))
    assert d['speed_mps'] == 0
    assert c.phase == Phase.CONES


@pytest.mark.parametrize('clear', ['left', 'right'])
def test_parking_chooses_random_clear_slot_and_requires_four_wheels(clear):
    c = controller(Phase.PARKING)
    c.update(parking(0, clear=clear))
    assert c.parking_slot == clear
    c.update(parking(.1, clear=clear, wheels_inside=3, parked_slot_id=clear))
    assert c.phase == Phase.PARKING
    c.update(parking(.2, clear=clear, wheels_inside=4, parked_slot_id=clear))
    assert c.phase == Phase.PAYMENT


def test_parking_does_not_change_to_newly_blocked_slot():
    c = controller(Phase.PARKING)
    c.update(parking(0, clear='left'))
    d = c.update(parking(.1, clear='right'))
    assert d['speed_mps'] == 0 and d['reason'] == 'selected_slot_became_blocked'


def test_parking_requires_clear_corridor_at_selected_slot_not_elsewhere():
    c = controller(Phase.PARKING)
    observation = parking(0, clear='right')
    observation.cones = [{'lateral_m': .3, 'forward_m': .6, 'radius_m': .039}]
    assert free_corridor(observation.lane_width_m, observation.cones, c.config) is not None
    decision = c.update(observation)
    assert decision['action'] == 'stop' and decision['reason'] == 'parking_approach_obstructed'


def test_parking_cannot_target_a_slot_outside_measured_lane():
    c = controller(Phase.PARKING)
    observation = parking(0, clear='right')
    for slot in observation.parking_slots:
        if slot['id'] == 'right':
            slot['center_lateral_m'] = 1
    assert c.update(observation)['action'] == 'stop'


def test_payment_window_is_thirty_seconds_after_four_wheel_stop():
    c = controller(Phase.PARKING)
    c.update(parking(0, wheels_inside=4, parked_slot_id='right'))
    c.update(obs(30.1, payment_confirmed=True, payment_amount_cents=1))
    assert c.phase == Phase.COMPLETE
    assert c.summary()['payment_bonus_points'] == 0


@pytest.mark.parametrize('amount', [0, 2, 100, True])
def test_payment_requires_one_cent(amount):
    c = controller(Phase.PARKING)
    c.update(parking(0, wheels_inside=4, parked_slot_id='right'))
    c.update(obs(.1, payment_confirmed=True, payment_amount_cents=amount))
    assert c.phase == Phase.PAYMENT


def test_remote_commands_require_fresh_5g_gateway_and_deadman():
    c = RaceController(config())
    cmd = {'authenticated': True, 'verified_5g': False, 'deadman': True, 'command_age_s': 0,
           'speed_mps': .2, 'steering_normalized': .1}
    assert c.update(obs(0, remote=cmd))['speed_mps'] == 0
    cmd = dict(cmd, verified_5g=True, command_age_s=1)
    assert c.update(obs(.1, remote=cmd))['speed_mps'] == 0
    cmd = dict(cmd, command_age_s=0)
    assert c.update(obs(.2, remote=cmd))['speed_mps'] == .2


def test_autonomous_stage_cannot_be_overridden_by_remote_command():
    c = controller(Phase.CROSSWALK_HOLD)
    d = c.update(obs(0, crosswalk_distance_m=.2, remote={'speed_mps': 1}))
    assert d['speed_mps'] == 0
    assert any(e['event'] == 'remote_driving_ignored_in_autonomous_phase' for e in d['events'])


def test_emergency_stop_stays_latched():
    c = RaceController(config())
    c.update(obs(0, emergency_stop=True))
    assert c.update(obs(.1, enable_autonomy=True, in_switch_zone=True))['speed_mps'] == 0
    assert c.phase == Phase.ESTOP


def test_stationary_over_twenty_seconds_fails():
    c = controller(Phase.BOARD)
    c.race_started = 0
    for i in range(202):
        c.update(obs(i / 10))
    assert c.phase == Phase.FAILED


@pytest.mark.parametrize('change', [{'source_ok': False}, {'telemetry_age_s': 1}, {'telemetry_valid': False}])
def test_missing_source_or_stale_speed_feedback_stops(change):
    c = controller(Phase.CROSSWALK_APPROACH)
    assert c.update(obs(0, **change))['speed_mps'] == 0


def test_nonfinite_or_forged_boolean_observation_is_rejected():
    for changes in [{'speed_mps': float('nan')}, {'source_ok': 'yes'}, {'wheels_inside': True}]:
        with pytest.raises(ValueError):
            Observation.from_dict(asdict(obs(0)) | changes)


def test_demo_marks_simulation_and_refuses_output_overwrite(tmp_path):
    args = SimpleNamespace(output=tmp_path / 'demo', race_config=None)
    summary = race_demo(args)
    assert summary['phase'] == 'complete' and summary['simulated'] is True
    log = [json.loads(line) for line in (args.output / 'decisions.jsonl').read_text(encoding='utf-8').splitlines()]
    assert all(row['simulated'] and row['hardware_output'] is False for row in log)
    assert log[-1]['action'] == 'stop'
    with pytest.raises(FileExistsError):
        race_demo(args)


def test_invalid_trace_leaves_explicit_stop(tmp_path):
    output = tmp_path / 'broken'
    with pytest.raises(TypeError):
        run_trace([{'t_s': 0, 'unknown_field': True}], config(), output)
    row = json.loads((output / 'decisions.jsonl').read_text())
    assert row['action'] == 'stop' and row['speed_mps'] == 0
