"""Competition-only recovery with a clock and actuator that never drive hardware."""
import pytest

from carvision.cone_command import ConeCommandSender
from carvision.race_flow import RaceFlowController
from test_race_flow import Frames, bundle
from test_cone_turn_controller import lane
from flow_calibration_fixture import current_crosswalk_sample


def test_control_gap_after_zebra_brake_recovers_without_restarting_stage(bundle):
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('crosswalk', 0., 'test')
    if race.controller.crosswalk_profile is not None:
        for reference in race.controller.brake_reference['camera_references'].values():
            reference['near_edge_y_normalized'] = .99
    else:
        race.controller.brake_reference['far_edge_y_normalized'] = .90
    frames = Frames(race)
    assert frames.step(moving=True)['action'] == 'brake'
    frames.t += .56
    paused = race.step(None, frames.t, interrupted=True)
    assert paused['control_recovering'] and paused['esc_us'] == 1500
    for _ in range(8):
        result = frames.step(moving=False)
        assert result['esc_us'] == 1500
        if result.get('control_recovered'):
            break
    assert result['control_recovered'] and race.stage == 'crosswalk'
    assert race.controller.first_detection_handled and race.controller.hold_since is None
    assert frames.step(moving=False)['phase'] == 'crosswalk'


@pytest.mark.parametrize('stage', ['traffic', 'cones', 'parking_search', 'parking_entry'])
def test_feedback_gap_keeps_current_stage_beyond_old_recovery_window(bundle, stage):
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go(stage, 0., 'test')
    if stage == 'parking_entry':
        race.parking_wait = 0.
    for now in (.51, 1., 2., 3., 4.):
        result = race.step(None, now)
        assert result['control_recovering'] and result['esc_us'] == 1500
        assert race.stage == stage


def test_old_frame_does_not_resume_and_the_race_budget_still_applies(bundle):
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('traffic', 0., 'test')
    assert race.step(None, .6)['control_recovering']
    sample = {'sequence': 1, 'captured_s': .5, 'traffic': {'frame_id': 1, 'captured_s': .5}}
    result = race.step(sample, .7)
    assert result['control_recovering'] and result['esc_us'] == 1500
    result = race.step(None, race.s['max_run_s']+1)
    assert result['phase'] == 'fault' and result['esc_us'] == 1500


def test_slow_vision_can_use_continuous_stillness_from_the_local_camera_cache(bundle):
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('traffic', 0., 'test')
    race.step(None, .51)
    for fid, now in enumerate((.9, 1.32), 1):
        sample = {'sequence': fid, 'captured_s': now-.1,
                  'traffic': {'frame_id': fid, 'captured_s': now-.1},
                  'motion': {'valid': True, 'moving': False, 'captured_s': now-.04,
                             'continuous_still_since_s': .55}}
        result = race.step(sample, now)
    assert result['control_recovered'] and result['esc_us'] == 1500
    assert race.step(sample, 1.33)['esc_us'] == 1500


def test_board_already_seen_remains_confirmed_but_removal_is_rechecked(bundle):
    race = RaceFlowController(bundle, 0., initialized=True, recover_gaps=True)
    frames = Frames(race)
    for _ in range(9):
        frames.step(board='present')
    assert race.departure.phase == 'wait_removed'
    frames.t += .7
    assert race.step(None,frames.t)['control_recovering']
    for _ in range(4):
        frames.step(board='absent')
    assert race.stage == 'wait_board' and race.departure.phase == 'wait_removed'
    frames.until('crosswalk', board='absent')


def test_background_gap_withdraws_forward_and_only_resumes_after_acknowledgment():
    class Actuator:
        run = False

        def send(self, intent):
            outputs.append((clock[0], intent['action']))

    clock, outputs = [1.], []
    sender = ConeCommandSender(Actuator(), clock=lambda: clock[0], threaded=False)
    forward = {'action':'search', 'esc_us':1600, 'steering_us':1610}
    neutral = {'action':'neutral', 'esc_us':1500, 'steering_us':1610}
    sender.send_race(forward, run_until=10., evidence_until=1.45)
    clock[0] = 1.75
    sender.tick()
    assert sender.race_interrupted() and outputs[-1][1] == 'brake'
    clock[0] = 2.11
    sender.tick()
    assert outputs[-1][1] == 'neutral'
    assert sender.send_race(forward, run_until=10., evidence_until=2.2) is None
    sender.send_race(neutral, run_until=10., recovered=True)
    assert not sender.race_interrupted()
    sender.send_race(forward, run_until=10., evidence_until=2.2)
    assert [action for _, action in outputs].count('brake') == 1
    assert outputs[-1][1] == 'search'


def test_metric_and_cone_forward_deadline_stays_anchored_to_capture():
    class Actuator:
        run = False

        def send(self, intent):
            outputs.append(intent['action'])

    clock, outputs = [2.], []
    sender = ConeCommandSender(Actuator(), clock=lambda: clock[0], threaded=False)
    forward = {'action':'search', 'esc_us':1600, 'steering_us':1610}
    sender.send_race(forward, run_until=10., evidence_until=2.09, expire_at_evidence=True)
    clock[0] = 2.1
    sender.tick()
    assert sender.race_interrupted() and outputs == ['search', 'brake']


def test_line_timeout_pauses_and_checks_both_edges_before_new_forward(bundle):
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('parking_search', 0., 'test')
    frames = Frames(race)

    def observed(t, fid):
        return lane(t, fid=fid, lane_rows=[180, 300])

    def missing(t, fid):
        return lane(t, fid=fid, state='missing')

    def shifted(t, fid):
        return lane(t, fid=fid, lane_rows=[180, 300], lane_bounds_px=[[200, 480], [150, 530]])

    assert frames.step(moving=True, nav_override=observed)['esc_us'] >= 1570
    for _ in range(4):
        paused = frames.step(moving=True, nav_override=missing)
    assert paused['control_recovering'] and paused['action'] == 'brake'
    for _ in range(10):
        result = frames.step(nav_override=shifted)
        assert result['esc_us'] <= 1500 and not result.get('control_recovered')
    # A different frame number on the same camera is needed for each vote.
    duplicated = observed(frames.t+.1, frames.i+1)
    for _ in range(3):
        assert not frames.step(nav_override=lambda *_: duplicated).get('control_recovered')
    for _ in range(3):
        result = frames.step(nav_override=observed)
        if result.get('control_recovered'):
            break
    assert result['control_recovered'] and result['esc_us'] == 1500
    confirmed_frame, confirmed_time = race.lane_resume_after
    repeated = observed(confirmed_time, confirmed_frame)
    assert frames.step(nav_override=lambda *_: repeated)['esc_us'] == 1500
    assert frames.step(moving=True, nav_override=observed)['esc_us'] >= 1570
    assert race.stage == 'parking_search'


def test_real_boundary_alarm_still_ends_lane_recovery(bundle):
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('parking_search', 0., 'test')
    frames = Frames(race)
    frames.step(moving=True)
    for _ in range(4):
        result = frames.step(lines=False, moving=True)
    assert result['control_recovering']
    result = frames.step(nav_override=lambda t, i: lane(t, fid=i, boundary_danger=True, reason='越界'))
    assert result['phase'] == 'fault' and result['esc_us'] == 1500 and result['reason'] == '越界'


def ground_still(stamp, since):
    return {'valid': True, 'moving': False, 'captured_s': stamp, 'continuous_still_since_s': since}


def at_formal_hold(bundle):
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('crosswalk', 0., 'test')
    frames = Frames(race)
    for _ in range(30):
        result = frames.step()
        if race.controller.phase == 'hold':
            break
    assert race.controller.phase == 'hold' and not race.controller.preliminary_stop
    assert any(event['event'] == 'speak' for event in result['events'])
    return race, frames


def test_zebra_stillness_continues_without_detector_and_next_stage_needs_a_live_lane(bundle):
    race, frames = at_formal_hold(bundle)
    started = race.controller.hold_since
    for elapsed in (.7, 2.5, 6., 9.99, 10.):
        captured = started+elapsed
        result = race.step(None, captured+.05, motion=ground_still(captured, started-.4))
        assert result['esc_us'] == 1500 and result['events'] == []
        assert result['hold_remaining_s'] == pytest.approx(10-elapsed)
    assert race.controller.phase == 'done' and race.stage == 'crosswalk'
    t = started+10.2
    sample = {'sequence':frames.i+1,'captured_s':t,'navigation':lane(t, state='missing')}
    assert race.step(sample, t, {'crosswalk':'command_sent'})['esc_us'] == 1500
    t += .1
    sample = {'sequence':frames.i+2,'captured_s':t,'navigation':lane(t)}
    assert race.step(sample, t, {'crosswalk':'command_sent'})['esc_us'] >= 1570
    assert race.stage == 'traffic'


def test_zebra_motion_gap_requires_new_stop_position_and_a_full_new_hold(bundle):
    race, frames = at_formal_hold(bundle)
    started = race.controller.hold_since
    race.step(None, started+.8, motion=ground_still(started+.8, started-.4))
    assert race.step(None, started+2., motion=ground_still(started+.8, started-.4))['esc_us'] == 1500
    recovered = started+2.2
    assert race.step(None, recovered+.6, motion=ground_still(recovered+.6, recovered))['esc_us'] == 1500
    assert race.controller.hold_since == started
    t = recovered+.8
    zebra = current_crosswalk_sample(race, {'fresh':True, 'frame_id':frames.i+1, 'captured_s':t,
        'image_size':[480,360], 'candidate':True, 'far_edge_y_normalized':.75})
    assert race.step({'crosswalk':zebra}, t, motion=ground_still(t, recovered))['hold_remaining_s'] == 10
    result = race.step(None, started+10.1, motion=ground_still(started+10.1, recovered))
    assert result['hold_remaining_s'] > 0 and race.controller.phase == 'hold'
    assert race.step(None, t+10., motion=ground_still(t+10., recovered))['hold_remaining_s'] == 0


def test_confirmed_movement_during_zebra_hold_never_credits_the_old_stop(bundle):
    race, _ = at_formal_hold(bundle)
    t = race.controller.hold_since + 1.
    moving = lambda stamp: {'valid': True, 'moving': True, 'captured_s': stamp}
    assert race.step(None, t, motion=moving(t))['action'] == 'brake'
    result = race.step(None, t+.1, motion=moving(t+.1))
    assert result['esc_us'] == 1500 and result['phase'] == 'crosswalk'
    end = t + race.controller.s['brake_timeout_s'] + .1
    result = race.step(None, end, motion=moving(end))
    assert result['phase'] == 'fault' and result['esc_us'] == 1500


def test_stillness_limit_counts_only_continuous_current_evidence_across_stages(bundle):
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('crosswalk', 1., 'test')
    race._go('traffic', 10., 'test')
    for now, motion in ((19.8, ground_still(19.8, .2)), (20.5, ground_still(19.8, .2)),
                        (21., ground_still(21., .2))):
        assert race.step(None, now, motion=motion)['phase'] == 'traffic'
    result = race.step(None, 21.01, motion=ground_still(21.01, .2))
    assert result['phase'] == 'fault' and result['esc_us'] == 1500 and '超过 20 秒' in result['reason']
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('traffic', 0., 'test')
    race.step(None, 18., motion=ground_still(18., 1.))
    race.step(None, 19., motion={'valid': True, 'moving': True, 'captured_s':19.})
    assert race.step(None, 24., motion=ground_still(24., 19.2))['phase'] == 'traffic'
