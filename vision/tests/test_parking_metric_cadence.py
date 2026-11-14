"""Freshness, slow computation and real motor-floor regressions; no hardware."""
import copy

import cv2
import numpy as np
import pytest

from carvision.parking_control import calibration_tables, parking_intent, curvature_for_pwm
from carvision.parking_mission import ParkingMission, plan_entry
from carvision.parking_task import ParkingTask
from test_parking_bay import sample, slots
from test_parking_path import measured_settings, measured_cones, selected_mission
from test_cone_navigation import measured, image, context
from carvision.cone_navigation import ConeOdometry


@pytest.mark.parametrize('period', [.3, .6, .75])
def test_fresh_slow_results_can_confirm_a_bay_and_begin_entry(period):
    task = ParkingTask(measured_settings(), measured_cones())
    for i in range(3):
        now = i * period
        out = task.step(sample(i, now), now)
    assert out['action'].startswith('parking_')
    assert out['esc_us'] >= 1570
    assert task.selected == 'right'
    assert task.mission.s['max_frame_age_s'] == .25


def test_control_gap_reconfirms_same_bay_and_replans_before_forward():
    mission = selected_mission()
    previous_path = mission.path
    first = mission.step(sample(7, 1.5, speed=.05), 1.5)
    assert first['action'] == 'brake' and mission.replan_required
    for i, now in enumerate((1.8, 2.1), 8):
        assert mission.step(sample(i, now), now)['action'] == 'hold'
        assert mission.path is previous_path
    resumed = mission.step(sample(10, 2.4), 2.4)
    assert resumed['action'] == 'follow_path' and not mission.replan_required
    assert mission.path is not previous_path and mission.selected == 'right'


@pytest.mark.parametrize('kind', ['frame', 'pose'])
def test_slow_computation_budget_never_makes_stale_evidence_current(kind):
    mission = selected_mission()
    world = sample(7, 1.2)
    if kind == 'frame':
        world['captured_s'] = .9
    else:
        world['pose']['captured_s'] = .9
    out = mission.step(world, 1.2)
    assert out['phase'] == 'fault'
    assert out['action'] in ('stop', 'brake')


@pytest.mark.parametrize('mutation', ['blocked', 'boundary_jump', 'pose_jump'])
def test_recovery_cannot_resume_old_path_when_current_world_disagrees(mutation):
    mission = selected_mission()
    mission.recover_control()
    world = sample(7, .9)
    if mutation == 'blocked':
        world = sample(7, .9, blocked='right')
    elif mutation == 'boundary_jump':
        for row in world['slots']:
            row['polygon_m'] = [[x+.06, y] for x, y in row['polygon_m']]
    else:
        world['pose']['x_m'] = .2
    out = mission.step(world, .9)
    assert out['phase'] == 'fault' and out['action'] == 'brake'


@pytest.mark.parametrize('bad_pwm', [1501, 1540, 1569])
def test_low_pwm_calibration_is_rejected_without_clamping_or_relabelling_speed(bad_pwm):
    settings = measured_settings()
    settings['motion_calibration']['speed_pwm'][0]['pwm_us'] = bad_pwm
    original = copy.deepcopy(settings)
    with pytest.raises(ValueError, match='1570'):
        calibration_tables(settings)
    with pytest.raises(ValueError, match='1570'):
        parking_intent({'action': 'follow_path', 'reason': 'test',
                        'target_speed_mps': settings['final_speed_mps'],
                        'target_curvature_1pm': 0.}, settings)
    assert settings == original


def test_route_reserves_motion_until_the_next_supported_control_result():
    settings = measured_settings()
    settings['braking_distance_m'] = .15
    pose = {'x_m': .3075, 'y_m': 0., 'yaw_rad': 0.}
    # This is a synthetic geometric comparison, not a measured braking value.
    assert plan_entry(slots('left'), 'right', pose, settings, max_control_gap_s=0.)
    with pytest.raises(ValueError, match='制动距离'):
        plan_entry(slots('left'), 'right', pose, settings, max_control_gap_s=.75)


def test_mission_uses_combined_evidence_age_and_command_reaction_budget():
    mission = ParkingMission(measured_settings(), max_control_gap_s=.75)
    assert mission.reaction_s == 1.
    with pytest.raises(ValueError):
        ParkingMission(measured_settings(), max_control_gap_s=.751)


@pytest.mark.parametrize('period', [.3, .6, .75])
def test_parking_optical_flow_accepts_supported_sample_cadence_with_real_lk(measured, period):
    odometry = ConeOdometry(measured, max_sample_gap_s=.75)
    first = image()
    assert not odometry.update(first, 1, 0., [], context(measured, first))['valid']
    moved = cv2.warpAffine(first, np.float32([[1,0,0],[0,1,1]]), (480,360),
                           borderMode=cv2.BORDER_REFLECT)
    result = odometry.update(moved, 2, period, [], context(measured, moved))
    assert result['valid']
    assert result['pose']['y_m'] == pytest.approx(.02, abs=.003)
    assert result['speed_mps'] == pytest.approx(.02 / period, abs=.01)
    assert not odometry.update(moved, 3, period+.751, [], context(measured, moved))['valid']
    assert odometry.failed


def test_longer_sampling_budget_keeps_physical_motion_and_feature_guards(measured):
    measured['driving']['max_speed_mps'] = .02
    odometry = ConeOdometry(measured, max_sample_gap_s=.75)
    first = image()
    odometry.update(first, 1, 0., [], context(measured, first))
    moved = cv2.warpAffine(first, np.float32([[1,0,0],[0,1,4]]), (480,360),
                           borderMode=cv2.BORDER_REFLECT)
    result = odometry.update(moved, 2, .3, [], context(measured, moved))
    assert not result['valid'] and odometry.failed


def test_parking_feed_requests_new_sampling_budget_without_promoting_calibration(monkeypatch):
    from carvision import parking_feedback
    calls = []
    monkeypatch.setattr(parking_feedback, 'ConeOdometry',
                        lambda settings, **kwargs: calls.append((settings, kwargs)))
    bundle = {'parking': measured_settings(), 'cones': measured_cones()}
    original = copy.deepcopy(bundle)
    parking_feedback.ParkingWorldFeed(bundle)
    assert calls[0][1]['max_sample_gap_s'] == .75
    assert bundle == original


def selected_task_decision(x=0., blocked='left'):
    task = ParkingTask(measured_settings(), measured_cones())
    for i in range(7):
        now = i*.1
        decision = task.decide(sample(i,now,{'x_m':x,'y_m':0.,'yaw_rad':0.},blocked),now)
    assert decision['action'] == 'follow_path'
    return task, decision, now


def test_final_slew_pwm_reports_its_own_calibrated_curvature_not_target():
    task, decision, now = selected_task_decision()
    task.steering = 1350.
    task.control_at = now-.1
    output = task.intent(decision,now,now)
    assert output['action'].startswith('parking_')
    assert output['steering_us'] == 1410
    assert output['command_curvature_1pm'] == curvature_for_pwm(1410,task.s)
    assert output['command_curvature_1pm'] != output['target_curvature_1pm']
    assert output['steering_policy'] == 'calibrated_path_slew'


@pytest.mark.parametrize('x,blocked,old_pwm',[(.4,'left',1350),(-.4,'right',1870)])
def test_final_slew_vetoes_unsafe_actual_arc_even_when_target_arc_was_clear(x,blocked,old_pwm):
    task, decision, now = selected_task_decision(x,blocked)
    task.steering = float(old_pwm)
    task.control_at = now-.1
    output = task.intent(decision,now,now)
    assert decision['action'] == 'follow_path'  # Planner's target arc was clear.
    assert output['action'] == 'brake' and output['phase'] == 'fault'
    assert output['esc_us'] == 1400 and output['steering_us'] == 1610
    assert output['command_curvature_1pm'] == 0.
    assert '最终转向PWM' in output['reason']
    assert output['rejected_command_curvature_1pm'] != output['rejected_target_curvature_1pm']


def test_final_output_cannot_reuse_an_expired_pose_after_a_slow_planning_step():
    task, decision, now = selected_task_decision()
    output = task.intent(decision,now+.3,now)
    assert output['action'] == 'brake' and output['phase'] == 'fault'
    assert '新鲜位姿' in output['reason']
