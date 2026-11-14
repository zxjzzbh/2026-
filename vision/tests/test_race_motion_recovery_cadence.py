"""Race recovery consumes independently sampled motion, never elapsed time alone."""
import pytest

from carvision.race_flow import FlowTrafficController
from carvision.traffic_driving import load_settings
from test_traffic_driving import CONFIG, Run


def race_run():
    run=Run('off')
    run.c=FlowTrafficController(load_settings(CONFIG),0.,120.,external_cadence=True)
    assert run.step(moving=None)['phase']=='motion_recovery'
    return run


def still(t,since=.45):
    return {'valid':True,'moving':False,'captured_s':t,'continuous_still_since_s':since}


def test_slow_race_processing_can_use_new_continuously_sampled_ground_stillness():
    run=race_run()
    assert run.step(dt=.6,motion=still(.7))['phase']=='motion_recovery'
    # The second current observation proves .85 seconds of continuous rest,
    # although the decision loop has not sampled every optical-flow frame.
    out=run.step(dt=.6,motion=still(1.3))
    assert out['phase']=='approach' and out['esc_us']==1500
    assert run.step(dt=.6,moving=True)['esc_us']>=1570


@pytest.mark.parametrize('motion', [
    still(1.3,.0),  # A window from before this recovery is insufficient.
    still(1.3,1.2),  # New, but not enough continuous rest yet.
    still(.9,.45),  # Stale even though its duration would otherwise suffice.
    still(1.4,.45),  # Future evidence is not current evidence.
    {'valid':False,'moving':False,'captured_s':1.3,'continuous_still_since_s':.45},
    {'valid':True,'moving':True,'captured_s':1.3,'continuous_still_since_s':.45},
])
def test_old_short_stale_future_unknown_and_moving_proofs_do_not_resume(motion):
    run=race_run()
    run.step(dt=.6,motion=still(.7))
    out=run.step(dt=.6,motion=motion)
    assert out['phase']=='motion_recovery' and out['esc_us']==1500


@pytest.mark.parametrize('moving',[None,True])
def test_movement_or_unknown_retires_earlier_stillness_claim(moving):
    run=race_run()
    run.step(dt=.6,moving=moving)
    out=run.step(dt=.6,motion=still(1.3))
    assert out['phase']=='motion_recovery' and out['esc_us']==1500
    # A newly established interval after the intervening observation is valid.
    out=run.step(dt=.1,motion=still(1.4,.8))
    assert out['phase']=='approach' and out['esc_us']==1500


def test_independent_trial_does_not_opt_into_the_race_continuity_override():
    run=Run('off');run.until('approach',moving=True)
    run.step(moving=None)
    start=run.t
    for _ in range(3):
        t=round(run.t+.4,8)
        out=run.step(dt=.4,motion=still(t,start+.36))
        assert out['phase']=='motion_recovery' and out['esc_us']==1500
