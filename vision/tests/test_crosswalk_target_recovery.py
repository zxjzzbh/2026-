"""Short missed detections stop/reconfirm instead of retiring the whole flow."""
import copy
from pathlib import Path

import pytest

from carvision.brake_parking import BrakeParkingController,load_settings
from carvision.race_flow import RaceFlowController,load_bundle
from flow_calibration_fixture import current_crosswalk_sample
from test_cone_turn_controller import lane


ROOT=Path(__file__).parents[2]


def controller(crosswalk=True):
    settings=load_settings(ROOT/'vision/configs/brake-parking.json')
    return BrakeParkingController(settings,crosswalk=crosswalk,control_interval_s=None)


def sample(t,fid,y=.61,*,moving=False,since=None):
    motion={'valid':moving is not None,'moving':moving,'captured_s':t,'frame_id':fid}
    if since is not None:motion['continuous_still_since_s']=since
    return {'fresh':True,'frame_id':fid,'captured_s':t,'image_size':[480,360],
        'candidate':y is not None,'far_edge_y_normalized':y,'motion':motion}


def lost_controller():
    c=controller()
    assert c.step(sample(0.,1,moving=True),0.)['action']=='search'
    row=c.step(sample(.4,2,None,moving=True),.4)
    assert row['phase']=='target_recovery' and row['action']=='brake'
    return c


def test_legacy_shared_parking_behavior_unchanged():
    c=controller(crosswalk=False)
    c.step(sample(0.,1,moving=True),0.)
    assert c.step(sample(.4,2,None,moving=True),.4)['phase']=='fault'


def test_target_loss_brake_is_finite_and_reconfirmation_never_drives_same_frame():
    c=lost_controller();pulse=c.pulse_until
    assert c.step(sample(.45,3,None),.45)['action']=='brake' and c.pulse_until==pulse
    assert c.step(sample(.65,4,None,since=.6),.65)['action']=='neutral'
    assert c.step(sample(1.2,5,since=.6),1.2)['phase']=='target_recovery'
    row=c.step(sample(1.3,6,since=.6),1.3)
    assert row['phase']=='search' and row['action']=='neutral'
    assert c.step(sample(1.4,7,moving=True),1.4)['esc_us']>=1570
    assert c.brake_count==1


@pytest.mark.parametrize('kind',['unknown','moving','stale','future','inconsistent','out_of_range'])
def test_recovery_does_not_resume_on_missing_or_conflicting_evidence(kind):
    c=lost_controller()
    for fid,t in enumerate((1.2,1.3,1.4),3):
        row=sample(t,fid,since=.6)
        if kind=='unknown':row['motion']['valid']=False
        elif kind=='moving':row['motion']['moving']=True
        elif kind=='stale':row['motion']['captured_s']=t-.3
        elif kind=='future':row['motion']['captured_s']=t+.1
        elif kind=='inconsistent':row['far_edge_y_normalized']=.61 if fid%2 else .73
        elif kind=='out_of_range':row['far_edge_y_normalized']=.4
        result=c.step(row,t)
        assert result['phase']=='target_recovery' and result['esc_us']<=1500


def test_control_pause_discards_already_collected_target_and_stillness_proof():
    c=lost_controller()
    c.step(sample(1.2,3,since=.6),1.2)
    assert c.target_recovery['target_hits']==1
    c.recover_target_control(1.25)
    row=c.step(sample(1.3,4,since=.6),1.3)
    assert row['phase']=='target_recovery' and c.target_recovery['target_hits']==0


def test_target_reappearing_at_stop_reference_runs_full_ten_second_hold():
    c=lost_controller()
    c.step(sample(1.2,3,.70,since=.6),1.2)
    row=c.step(sample(1.3,4,.70,since=.6),1.3)
    assert row['phase']=='settle' and row['action']=='neutral'
    for fid in range(5,125):
        t=round(1.3+(fid-4)*.1,8)
        row=c.step(sample(t,fid,.70,since=.6),t)
        assert row['esc_us']==1500
        if row['phase']=='done':break
    assert row['phase']=='done' and c.last_t-c.hold_since>=10.


def test_formal_flow_one_400ms_missed_target_no_longer_ends_round():
    flow=RaceFlowController(load_bundle(ROOT),0.,recover_gaps=True)
    flow._go('crosswalk',0.,'offline-test')
    now=0.;fid=0
    def step(dt=.1,candidate=True,moving=False):
        nonlocal now,fid
        now=round(now+dt,8);fid+=1
        module=current_crosswalk_sample(flow,sample(now,fid,.53 if candidate else None,
            moving=moving,since=.2 if not moving else None))
        return flow.step({'sequence':fid,'captured_s':now,'motion':module['motion'],
            'navigation':lane(now,fid=fid),'crosswalk':module},now,motion=module['motion'])
    assert step(moving=True)['action']=='brake'
    for _ in range(30):
        step()
        if flow.controller.phase=='search' and not flow.controller.preliminary_stop:break
    assert step(moving=True)['esc_us']>=1570
    row=step(.4,candidate=False,moving=True)
    assert row['phase']=='crosswalk' and flow.controller.phase=='target_recovery'
    assert row['esc_us']==1400
