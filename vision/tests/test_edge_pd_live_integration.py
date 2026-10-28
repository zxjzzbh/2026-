"""Integration with the installed measured-PWM parking adapter, no hardware."""
from types import SimpleNamespace
import pytest
from carvision.race_flow import RaceFlowController
from test_race_flow import bundle, measured_bundle

@pytest.mark.parametrize('curvature,expected_pwm',[(1.,1510),(-1.,1730)])
def test_flow_pd_retains_measured_speed_special_action_and_finite_brake(bundle,curvature,expected_pwm):
    controller=RaceFlowController(measured_bundle(bundle),0.)
    controller._go('parking_entry',0.,'test')
    controller.parking_wait=-1.
    decision={'phase':'enter','action':'follow_path','reason':'synthetic metric target',
              'target_speed_mps':.075,'target_curvature_1pm':curvature}
    controller.parking.mission=SimpleNamespace(selected='right',path=[],step=lambda sample,now:dict(decision))
    previous=1610
    for frame in range(1,7):
        now=frame*.1
        out=controller._step_stage({'parking':{'frame_id':frame,'captured_s':now}},now,{})
        assert out['action']=='parking_1550' and out['esc_us']==1550
        assert out['target_speed_mps']==.075 and out['command_speed_mps']==.075
        assert abs(out['steering_us']-previous)<=61
        assert 1350<=out['steering_us']<=1870
        assert out['steering_policy']=='verified_path_pd'
        previous=out['steering_us']
    assert previous==expected_pwm
    decision.update(action='brake',phase='braking')
    out=controller._step_stage({'parking':{'frame_id':7,'captured_s':.7}},.7,{})
    assert out['action']=='brake' and out['esc_us']==1400 and out['steering_us']==1610
    assert controller.parking.pd.error is None
