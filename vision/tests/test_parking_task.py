"""Shared production parking core; synthetic calibration is test-only."""
import copy
import math

import numpy as np
import pytest

from carvision.parking_control import MetricParkingController, parking_blockers, parking_readiness, calibration_tables
from carvision.parking_task import ParkingTask
from carvision.parking_entry_run import ParkingEntryRun
from test_parking_path import measured_settings, measured_cones
from test_parking_bay import sample
from test_parking_entry_run import configurations, base, preparation
from test_parking_visual_fixes import moving_controller, step, frame


def test_grouped_readiness_preserves_all_gates_and_does_not_edit_configuration():
    parking, _, cones = configurations()
    bundle = {'parking': parking, 'cones': cones}
    original = copy.deepcopy(bundle)
    report = parking_readiness(bundle)
    assert report['missing'] == parking_blockers(bundle)
    assert not report['ready'] and not report['calibration_modified']
    assert {x['reason'] for x in report['items']} == set(report['missing'])
    assert all(x['required_evidence'] for x in report['items'])
    assert {'camera','perception','motion','localization'} <= {x['group'] for x in report['items']}
    assert bundle == original
    with pytest.raises(ValueError, match='路径入库缺少'):
        ParkingTask(parking, cones)
    pending = ParkingTask(parking, cones, require_ready=False)
    output = pending.step(None, 0.)
    assert output['esc_us'] == 1500 and output['phase'] == 'not_ready' and not output['payment_ready']


def test_independent_metric_adapter_and_competition_core_produce_the_same_commands():
    parking, cones = measured_settings(), measured_cones()
    task = ParkingTask(parking, cones)
    independent = MetricParkingController(parking, cones)
    for i in range(30):
        output = independent.step({'parking_world': sample(i,i*.1)}, i*.1)
        assert output['phase'] == 'arming' and output['esc_us'] == 1500
    for i in range(30, 42):
        now = i*.1
        world = sample(i,now)
        direct = task.step(world,now)
        wrapped = independent.step({'parking_world':world},now)
        for key in ('action','esc_us','steering_us','phase','parking_decision','task_core'):
            assert wrapped[key] == direct[key]
    assert independent.mission is independent.task.mission
    assert direct['task_core'] == 'metric_parking_v1'


@pytest.mark.parametrize('blocked',['left','right'])
def test_shared_task_parks_from_integrated_command_feedback(blocked):
    parking, cones = measured_settings(), measured_cones()
    task = ParkingTask(parking, cones)
    pose = {'x_m':0., 'y_m':0., 'yaw_rad':0.}
    speed = 0.
    _, steering = calibration_tables(parking)
    for i in range(601):
        now = i*.1
        output = task.step(sample(i,now,dict(pose),blocked,speed),now)
        decision = output['parking_decision']
        assert decision['phase'] != 'fault', decision
        if decision['phase'] == 'parked':break
        target = output.get('command_speed_mps',0.)
        speed += max(-.04,min(.03,target-speed))
        curvature = float(np.interp(output['steering_us'],steering[1][::-1],steering[0][::-1]))
        mid = pose['yaw_rad']+.05*speed*curvature
        pose['x_m'] += speed*math.sin(mid)*.1
        pose['y_m'] += speed*math.cos(mid)*.1
        pose['yaw_rad'] += speed*curvature*.1
    assert decision['phase'] == 'parked' and decision['four_wheels_inside']
    assert output['payment_ready'] and output['formal_task'] and not output['diagnostic_only']


def test_missing_mode_uses_metric_gate_and_never_opens_hardware():
    made = []
    run = ParkingEntryRun(lambda settings:made.append(settings))
    payload = preparation()
    payload.pop('controller_mode')
    with pytest.raises(ValueError,match='路径入库缺少'):
        run.prepare(*configurations(),base(),payload,0.)
    assert not made and not run.active


def test_explicit_legacy_trial_never_authorizes_formal_completion_or_payment():
    run = ParkingEntryRun(lambda settings:None)
    run.prepare(*configurations(),base(),preparation(controller_mode='visual_trial'),0.)
    report = run.status(base(),0.)
    assert report['diagnostic_only'] and not report['formal_task']
    assert not report['completion_authorizes_payment']
    controller = ParkingEntryRun._controller(run.s)
    output = controller._result()
    assert output['diagnostic_only'] and not output['formal_task'] and not output['payment_ready']


def test_recovery_rejects_stale_stationary_feedback_and_requires_three_fresh_frames():
    controller = moving_controller()
    assert step(controller,36,white=False,empty=True)['esc_us'] == 1400
    for i in range(37,44):
        row = frame(i,moving=False)
        row['motion']['captured_s'] = 3.0
        output = controller.step(row,i*.1)
        assert output['esc_us'] <= 1500 and output['boundary_wait_cause'] == 'stop_feedback_invalid'
    for i in (44,45):
        output = step(controller,i,moving=False)
        assert output['phase'] == 'boundary_wait' and output['esc_us'] == 1500
    output = step(controller,46,moving=False)
    assert output['phase'] == 'approach' and output['esc_us'] == 1550
