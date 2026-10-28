"""Real guidance controller/guards with synthetic observations and fake writes.

The inputs prove software contracts only, never wheel angles, vehicle speed,
physical cone clearance or the actual duration of a real turn.
"""
import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_execution import ConeRun
from carvision.cone_low_speed import PROFILE
from carvision.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard
from carvision.shared_executor import BorrowedActuator
from cone_mission_fixture import ground_nav
from test_cone_passage import Scene
from test_cone_turn_controller import lane, settings
from test_shared_executor import CLIENT, Worker, rig


IDENTIFIER = 'secondary-A'


def pixels_only(t, *, visible=True):
    """Current measured pixel contract, without invented lane/ground history."""
    visual = ground_nav(t, IDENTIFIER, y=.55)['visual_cones'] if visible else []
    return lane(t, state='missing', visual_cones=visual)


def locked_controller():
    controller = ContinuousConeController(settings(gain=1.5), 0., initialized=True,
                                          handoff_after_two=True)
    for t in (0., .1, .2):
        output = controller.step(lane(t), t)
    assert output['esc_us'] > 1500 and output['effective_steering_gain'] == 1.
    for t in (.3, .4, .5, .6):
        output = controller.step(ground_nav(t, IDENTIFIER, y=.55), t)
    assert controller.target == IDENTIFIER and controller.side == 'left'
    assert output['target_observation']['target_stable']
    assert controller.mission.registered('secondary', IDENTIFIER)
    assert controller.count == 0
    return controller


def test_current_same_target_pixels_sustain_selected_side_past_one_second_through_actual_guard():
    controller = locked_controller()
    writes = []
    guard = ConePulseGuard(actuator_settings(controller.s), lambda *row: writes.append(row),
                           mode='F/B', right_us=controller.options['right_us'],
                           left_us=controller.options['left_us'])
    outputs = []
    for index in range(7, 48):
        t = round(index*.1, 8)
        output = controller.step(pixels_only(t), t)
        outputs.append(output)
        guard.accept({**output, 'sequence': index, 'sent_s': t}, t)
        assert output['fresh_control'] and output['target_id'] == IDENTIFIER
        assert controller.side == 'left' and output['target_space'] == 'image'
        assert output['guidance_mode'] == 'target_only' and output['target_feedback_steering']
        assert output['effective_steering_gain'] == 1.5
        assert 1500 < output['esc_us'] <= 1570
        assert output['guidance_budget']['target_only_deadline_s'] == pytest.approx(5.7)
        assert not output['guidance_budget']['expired']
        assert not output['boundary_observed'] and not output['physical_tail_verified']
        assert output['phase'] != 'complete' and controller.count == 0
    assert any(abs(row['steering_us']-1610) > 80 for row in outputs)
    assert guard.esc == 1570 and not guard.fault
    assert any(channel == 3 and abs(pulse-1610) > 80 for channel, pulse in writes)
    assert not any(channel == 4 and pulse > 1570 for channel, pulse in writes)


def test_target_only_deadline_cannot_be_washed_by_new_pixels_or_late_paired_road():
    controller = locked_controller()
    for index in range(7, 57):
        t = round(index*.1, 8)
        controller.step(pixels_only(t), t)
    with pytest.raises(ValueError, match='5.*试验上限'):
        controller.step(pixels_only(5.7), 5.7)
    assert controller.guidance_budget['reason'] == 'target_only_episode_deadline'
    # A new good road after the expired interval does not restart this run.
    with pytest.raises(ValueError, match='5.*试验上限'):
        controller.step(ground_nav(5.8, IDENTIFIER, y=.55), 5.8)


@pytest.mark.parametrize('late_input', ['pixels', 'paired'])
def test_no_road_no_target_has_one_second_deadline_and_late_input_cannot_clear_it(late_input):
    controller = locked_controller()
    for index in range(7, 17):
        t = round(index*.1, 8)
        output = controller.step(pixels_only(t, visible=False), t)
        assert output['guidance_budget']['none_deadline_s'] == pytest.approx(1.7)
        assert abs(output['steering_us']-1610) <= 80
        assert output['effective_steering_gain'] == 1.
        assert controller.count == 0 and output['phase'] != 'complete'
    late = pixels_only(1.7) if late_input == 'pixels' else ground_nav(1.7, IDENTIFIER, y=.55)
    with pytest.raises(ValueError, match='均无可靠引导超过 1 秒'):
        controller.step(late, 1.7)
    assert controller.guidance_budget['reason'] == 'no_reliable_guidance_deadline'


@pytest.mark.parametrize('forgery', ['old_frame', 'wrong_camera', 'bad_bbox', 'wrong_foot', 'no_revision'])
def test_same_id_with_invalid_current_pixel_metadata_cannot_release_small_recovery_steering(forgery):
    controller = locked_controller()
    for index in range(7, 12):
        t = round(index*.1, 8)
        nav = pixels_only(t)
        entry = nav['visual_cones'][0]
        if forgery == 'old_frame':
            entry['frame_id'] -= 1
        elif forgery == 'wrong_camera':
            entry['source_camera'] = 'primary'
        elif forgery == 'bad_bbox':
            entry['bbox_xyxy'] = [0., 0., 1., 1.]
        elif forgery == 'wrong_foot':
            entry['y_fraction'] = .99
        else:
            entry.pop('identity_revision')
        output = controller.step(nav, t)
        assert not output['target_observation']['current_valid']
        assert not output['target_observation']['target_stable']
        assert not output['target_feedback_steering']
        assert abs(output['requested_steering_us']-1610) <= 80
        assert abs(output['steering_us']-1610) <= 80
        assert output['effective_steering_gain'] == 1.
        assert controller.count == 0


def test_real_single_edge_model_expires_from_last_observed_pair_and_cannot_extend_itself():
    scene = Scene()
    controller = ContinuousConeController(scene.s, 0., initialized=True)
    for t in (0., .1, .2):
        _, nav = scene.row(t)
        output = controller.step(nav, t)
    assert output['esc_us'] > 1500
    for t in (.3, .4):
        _, nav = scene.row(t, single=True)
        assert nav['line_state'] == 'single'
        assert nav['line_observed_s'] == pytest.approx(.2)
        output = controller.step(nav, t)
        assert output['guidance_mode'] == 'single'
        assert 1500 < output['esc_us'] <= 1570
    _, nav = scene.row(.451, single=True)
    assert nav['line_state'] == 'missing' and not nav.get('lane_bounds_px')
    output = controller.step(nav, .451)
    assert output['guidance_mode'] == 'none'
    assert output['guidance_budget']['none_since_s'] == pytest.approx(.451)
    for t in (.6, .8, 1., 1.2, 1.4):
        _, nav = scene.row(t, single=True)
        assert nav['line_state'] == 'missing'
        controller.step(nav, t)
    _, nav = scene.row(1.451)
    with pytest.raises(ValueError, match='均无可靠引导超过 1 秒'):
        controller.step(nav, 1.451)


def test_reacquired_paired_road_with_current_target_keeps_avoidance_until_real_exit_evidence():
    controller = locked_controller()
    for index in range(7, 18):
        t = round(index*.1, 8)
        controller.step(pixels_only(t), t)
    for t in (1.8, 1.9, 2.):
        output = controller.step(ground_nav(t, IDENTIFIER, y=.55), t)
        assert controller.target == IDENTIFIER and controller.side == 'left'
        assert output['target_space'] == 'lane' and output['maneuver_phase'] == 'avoid'
        assert output['effective_steering_gain'] == 1.5
        assert not output['lane_return_confirmed'] and not controller.recentering
        assert controller.count == 0 and output['phase'] != 'complete'
    assert output['paired_stable'] and output['guidance_mode'] == 'paired'
    assert output['guidance_budget']['target_only_deadline_s'] is None


def test_disappearing_target_pixels_cannot_turn_elapsed_time_into_a_pass():
    controller = locked_controller()
    for t, y in ((.7, .63), (.8, .71), (.9, .73), (1., .76)):
        controller.step(ground_nav(t, IDENTIFIER, y=y), t)
    assert controller.pass_gate_hits == 3
    assert controller.pass_evidence['visual_pass_candidate']
    assert not controller.pass_evidence['visual_estimate_ready']
    for index in range(11, 21):
        t = round(index*.1, 8)
        output = controller.step(pixels_only(t, visible=False), t)
        assert controller.count == 0 and not controller.passed_ids
        assert not output['physical_tail_verified'] and output['phase'] != 'complete'
        assert output['effective_steering_gain'] == 1.
    assert output['guidance_budget']['none_deadline_s'] == pytest.approx(2.1)
    with pytest.raises(ValueError, match='均无可靠引导超过 1 秒'):
        controller.step(pixels_only(2.1, visible=False), 2.1)


def test_ordinary_lane_gain_one_and_active_same_target_gain_one_point_five_remain_separate():
    controller = ContinuousConeController(settings(gain=1.5), 0., initialized=True)
    for t in (0., .1, .2, .3):
        output = controller.step(lane(t, near=.55, far=.55), t)
    assert output['target_space'] == 'lane_follow'
    assert output['effective_steering_gain'] == 1.
    for t in (.4, .5, .6, .7):
        output = controller.step(ground_nav(t, IDENTIFIER, y=.55), t)
    assert output['target_space'] == 'lane'
    assert output['effective_steering_gain'] == 1.5


def test_cached_target_navigation_does_not_renew_real_shared_worker_lease(rig):
    now, clock, backend, manual, owner, _ = rig
    controller = locked_controller()
    worker = Worker(owner)
    handle = BorrowedActuator(worker, actuator_settings(controller.s), CLIENT, clock=clock)
    run = ConeRun()
    run.active = True
    run.profile = PROFILE
    run.s = controller.s
    run.controller = controller
    run.actuator = handle
    run.lease_until = 10.
    nav = pixels_only(.7)
    now[0] = .7
    run.tick(nav, .7)
    assert run.intent['target_feedback_steering'] and owner.guard.esc == 1570
    requests, deadline = len(worker.calls), owner.guard.deadline
    for t in (.72, .76, .82):
        now[0] = t
        run.tick(nav, t)
        assert run.active and run.intent['fresh_control'] is False
        assert len(worker.calls) == requests and owner.guard.deadline == deadline
    now[0] = deadline+.001
    owner.tick()
    assert not owner.active and owner.guard.fault and owner.guard.esc == 1400
    assert any(row[0] == 'motion' and row[2] == 1400 for row in backend.writes)
    now[0] += .4
    owner.tick()
    motor_writes = [row for row in backend.writes if row[0] in ('motion', 'neutral')]
    assert owner.guard is None and motor_writes[-1][0] == 'neutral'
    run.cancel('offline cleanup', now=now[0])


def test_real_dual_navigation_identity_revision_cannot_borrow_old_large_lane_recovery_demand():
    scene = Scene(reference=(.55, .55, .55), fraction=.7)
    controller = ContinuousConeController(scene.s, 0., initialized=True)
    for t in (0., .1, .2, .3, .4, .5, .6):
        _, nav = scene.row(t, .55)
        output = controller.step(nav, t)
    assert controller.target == 'secondary-1' and controller.side == 'left'
    assert output['paired_stable'] and output['target_observation']['target_stable']
    assert abs(output['steering_us']-1610) > 80
    assert controller.mission.registered('secondary', 'secondary-1')

    # Keep the real DualNav's measured current pixels; remove road observations
    # explicitly to model a turn with segmentation loss, without inventing a
    # current ground coordinate or a physical trajectory.
    for t in (.7, .8, .9):
        _, paired = scene.row(t, .55)
        nav = lane(t, state='missing', visual_cones=paired['visual_cones'])
        output = controller.step(nav, t)
        assert output['target_feedback_steering']
        assert abs(output['steering_us']-1610) > 80

    for index, t in enumerate((1., 1.1), 1):
        _, nav = scene.row(t, .55, identity_revision=1)
        output = controller.step(nav, t)
        assert output['target_observation']['frames'] == index
        assert not output['paired_stable']
        assert not output['target_observation']['target_stable']
        assert not output['paired_recovery_continuation']
        assert abs(output['requested_steering_us']-1610) <= 80
        assert abs(output['steering_us']-1610) <= 80
        assert controller.count == 0 and controller.target == 'secondary-1'
        assert controller.mission.registered('secondary', 'secondary-1')

    _, nav = scene.row(1.2, .55, identity_revision=1)
    output = controller.step(nav, 1.2)
    assert output['paired_stable'] and output['target_observation']['target_stable']
    assert output['target_observation']['frames'] == 3
    assert abs(output['requested_steering_us']-1610) > 80
    assert abs(output['steering_us']-1610) > 80
    assert controller.side == 'left' and controller.count == 0
