"""Slow but fresh results must not be confused with lost actuator cadence."""
import pytest
from carvision.race_flow import RaceFlowController
from carvision.traffic_driving import TrafficDrivingController
from carvision.brake_parking import BrakeParkingController
from test_race_flow import Frames, bundle
from test_cone_turn_controller import lane
from test_race_control_recovery import ground_still


@pytest.mark.parametrize('stage', ['crosswalk','traffic','parking_search'])
def test_fresh_results_at_six_hundred_ms_do_not_restart_recovery(bundle,stage):
    race=RaceFlowController(bundle,0.,recover_gaps=True)
    race._go(stage,0.,'test')
    frames=Frames(race)
    for _ in range(8):
        result=frames.step(dt=.6,color='off',moving=stage!='crosswalk')
        assert result['phase']==stage,result
        assert not result.get('control_recovering'),result
        assert not result.get('control_recovered'),result


def test_recovered_frame_then_slow_fresh_frame_does_not_pause_again(bundle):
    race=RaceFlowController(bundle,0.,recover_gaps=True)
    race._go('parking_search',0.,'test')
    assert race.step(None,.6)['control_recovering']
    for fid,t in enumerate((1.2,1.8),1):
        sample={'sequence':fid,'captured_s':t,'navigation':lane(t,fid=fid),
                'motion':ground_still(t,.61)}
        result=race.step(sample,t)
    assert result['control_recovered'],result
    t=2.4
    sample={'sequence':3,'captured_s':t,'navigation':lane(t,fid=3),
            'motion':ground_still(t,.61)}
    result=race.step(sample,t)
    assert not result.get('control_recovering') and result['esc_us']>=1570,result


def test_old_lane_does_not_gain_authority_from_fresh_task_result(bundle):
    race=RaceFlowController(bundle,0.,recover_gaps=True)
    race._go('parking_search',0.,'test')
    result=race.step({'sequence':1,'captured_s':1.,'navigation':lane(.1)},1.)
    assert result['control_recovering'] and result['esc_us']==1500
    assert result['recovery_diagnostics']['cause']=='task_frame_stale'


def test_initial_traffic_result_with_old_lane_waits_instead_of_faulting(bundle):
    race=RaceFlowController(bundle,0.,recover_gaps=True)
    race._go('traffic',0.,'test')
    frames=Frames(race)
    result=frames.step(dt=1.,color='off',nav_override=lambda t,i:lane(t-.6,fid=i))
    assert result['phase']=='traffic' and result['control_recovering']
    assert result['esc_us']==1500


@pytest.mark.parametrize('stage', ['crosswalk','traffic'])
def test_duplicate_time_and_stale_inputs_still_fail_in_external_cadence(bundle,stage):
    race=RaceFlowController(bundle,0.,recover_gaps=True)
    race._go(stage,0.,'test')
    frames=Frames(race);frames.step(color='off')
    assert race.controller.control_interval_s is None
    result=race.controller.step({},frames.t)
    assert result['phase']=='fault'


def test_standalone_traffic_keeps_its_original_processing_guard(bundle):
    controller=TrafficDrivingController(bundle['traffic'])
    controller.last_t=0.
    assert controller.step({},.6)['reason']=='控制处理间隔失效，停止本轮'
    controller=BrakeParkingController(bundle['brake'])
    controller.last_t=0.
    assert controller.step({},.6)['reason']=='控制间隔失效'
