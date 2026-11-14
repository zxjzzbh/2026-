"""Metric parking target -> final PWM/sweep integration, synthetic data only."""
import pytest
from carvision.race_flow import RaceFlowController
from carvision.parking_control import curvature_for_pwm
from test_race_flow import bundle, measured_bundle, parking_sample

@pytest.mark.parametrize('curvature,expected_pwm',[(1.,1510),(-1.,1730)])
def test_flow_slew_retains_measured_speed_final_curve_and_brake(bundle,monkeypatch,curvature,expected_pwm):
    controller=RaceFlowController(measured_bundle(bundle),0.)
    controller._go('parking_entry',0.,'test')
    controller.parking_wait=-1.
    task=controller.parking
    for frame in range(7):
        task.decide(parking_sample(frame*.1,frame),frame*.1)
    assert task.selected=='right'
    decide=task.decide
    def fixed_target(world,now):
        decision=decide(world,now)
        if decision['action']=='follow_path':
            decision={**decision,'target_speed_mps':.075,'target_curvature_1pm':curvature}
        return decision
    # Isolate two target curvatures while retaining real pose, bay validation
    # and final-output swept-body checks. No replacement mission/guard stub.
    monkeypatch.setattr(task,'decide',fixed_target)
    previous=1610
    for frame in range(7,13):
        now=frame*.1
        out=controller._step_stage({'parking':parking_sample(now,frame)},now,{})
        assert out['action']=='parking_1580' and out['esc_us']==1580
        assert out['target_speed_mps']==.075 and out['command_speed_mps']==.075
        assert abs(out['steering_us']-previous)<=61
        assert 1350<=out['steering_us']<=1870
        assert out['steering_policy']=='calibrated_path_slew'
        assert out['command_curvature_1pm']==pytest.approx(curvature_for_pwm(out['steering_us'],task.s))
        previous=out['steering_us']
    assert previous==expected_pwm
    task.mission.phase='braking';task.mission.brake_since=1.3
    out=controller._step_stage({'parking':parking_sample(1.3,13,speed=.05)},1.3,{})
    assert out['action']=='brake' and out['esc_us']==1400 and out['steering_us']==1610
    assert out['command_curvature_1pm']==0. and task.steering==1610
