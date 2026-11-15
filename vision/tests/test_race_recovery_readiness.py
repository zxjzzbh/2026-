"""Sparse stationary observations must not cause endless stop/start cycles."""
from carvision.race_flow import RaceFlowController
from test_race_flow import bundle


def sample(fid, now, *, still=True):
    return {'sequence':fid,'captured_s':now,
        'traffic':{'frame_id':fid,'captured_s':now,'source_camera':'secondary'},
        'motion':{'valid':True,'moving':False,'captured_s':now,
                  **({'continuous_still_since_s':.55} if still else {})}}


def test_confirmed_rest_is_preserved_but_slow_planner_does_not_restart_forward(bundle):
    race=RaceFlowController(bundle,0.,recover_gaps=True)
    race._go('traffic',0.,'test')
    race.step(None,.51)
    for fid,now in enumerate((1.,2.,3.),1):
        out=race.step(sample(fid,now),now)
        assert out['esc_us']==1500 and out['control_recovering']
    assert out['recovery_diagnostics']['hits']==3
    assert out['recovery_diagnostics']['cadence_hits']==1
    assert '处理间隔' in out['reason']
    out=race.step(sample(4,3.4),3.4)
    assert out['control_recovered'] and out['esc_us']==1500
    assert race.stage=='traffic'
    assert race.step(sample(4,3.4),3.41)['esc_us']==1500


def test_sparse_isolated_still_images_cannot_claim_continuous_recovery(bundle):
    race=RaceFlowController(bundle,0.,recover_gaps=True)
    race._go('traffic',0.,'test');race.step(None,.51)
    for fid,now in enumerate((1.,2.,3.),1):out=race.step(sample(fid,now,still=False),now)
    assert out['control_recovering'] and out['recovery_diagnostics']['hits']==1
