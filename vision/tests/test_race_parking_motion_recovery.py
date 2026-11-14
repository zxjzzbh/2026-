"""Parking recovery uses independent stillness, without inventing pose history."""
import copy

import pytest

from carvision.race_flow import RaceFlowController
from test_race_flow import bundle
from test_race_sender_cadence import FORWARD, advance, rig


def parking_sample(now, frame):
    stamp = now-.05
    return {'sequence':frame, 'captured_s':stamp,
            'parking':{'frame_id':frame, 'captured_s':stamp,
                'coordinate_frame':'parking_road_world', 'obstacle_region_verified':True,
                'speed_mps':0., 'pose':{'verified':True, 'captured_s':stamp,
                    'x_m':0., 'y_m':0., 'yaw_rad':0., 'error_bound_m':0.}}}


def still(now, since=.9):
    return {'valid':True, 'moving':False, 'captured_s':now-.01,
            'continuous_still_since_s':since}


def paused_parking(bundle):
    bundle=copy.deepcopy(bundle)
    bundle['parking'].update(maximum_pose_error_m=.03,max_speed_mps=.15)
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('parking_entry', 0., 'test')
    race.parking_wait = 0.
    race.last_drive = True
    assert race.step(None, .8, interrupted=True)['control_recovering']
    return race


def test_six_hundred_ms_parking_recovery_acknowledges_the_real_paused_sender(bundle, rig):
    clock, actuator, sender = rig
    sender.send_race(FORWARD, run_until=10., evidence_until=.5)
    advance(rig, .8)
    assert sender.race_interrupted()
    race = paused_parking(bundle)
    for frame, now in enumerate((1.4, 2.0), 1):
        advance(rig, now)
        sample, motion = parking_sample(now, frame), still(now, 1.11)
        original_sample, original_motion = copy.deepcopy(sample), dict(motion)
        result = race.step(sample, now, interrupted=sender.race_interrupted(), motion=motion)
        sender.send_race(result, run_until=10., recovering=result.get('control_recovering',False),
                         recovered=result.get('control_recovered',False))
        assert sample == original_sample and motion == original_motion
    assert result['control_recovered'] and result['esc_us'] == 1500
    assert race.control_pause is None and race.stage == 'parking_entry'
    assert not sender.race_interrupted() and not actuator.guard.fault
    repeated = race.step(sample, 2.01, motion=still(2.01,1.11))
    assert repeated['esc_us'] == 1500 and not repeated.get('control_recovered')
    assert len([row for row in actuator.commands if row['action']=='brake']) == 1
    assert all(row['esc_us']<=1500 for row in actuator.commands if row['sent_s']>=.75)


@pytest.mark.parametrize('invalid', ['unverified_pose','pose_stale','pose_future','pose_value',
    'pose_error','wrong_coordinate','unverified_obstacles','speed_nonfinite',
    'independent_stale','independent_future','independent_unknown','independent_missing_interval'])
def test_parking_recovery_rejects_invalid_pose_or_independent_stillness(bundle, invalid):
    race = paused_parking(bundle)
    for frame, now in enumerate((1.4,2.0,2.6), 1):
        sample, motion = parking_sample(now,frame), still(now)
        parking, pose = sample['parking'], sample['parking']['pose']
        if invalid=='unverified_pose': pose['verified']=False
        elif invalid=='pose_stale': pose['captured_s']=now-.6
        elif invalid=='pose_future': pose['captured_s']=now+.01
        elif invalid=='pose_value': pose['x_m']=float('nan')
        elif invalid=='pose_error': pose['error_bound_m']=1.
        elif invalid=='wrong_coordinate': parking['coordinate_frame']='image'
        elif invalid=='unverified_obstacles': parking['obstacle_region_verified']=False
        elif invalid=='speed_nonfinite': parking['speed_mps']=float('nan')
        elif invalid=='independent_stale': motion['captured_s']=now-.3
        elif invalid=='independent_future': motion['captured_s']=now+.01
        elif invalid=='independent_unknown': motion.update(valid=False,moving=None)
        elif invalid=='independent_missing_interval': motion.pop('continuous_still_since_s')
        result = race.step(sample,now,motion=motion)
        assert result['control_recovering'] and result['esc_us'] == 1500
    assert race.control_pause is not None


@pytest.mark.parametrize('moving_source', ['pose','pose_overspeed','independent'])
def test_a_moving_conflict_cannot_be_erased_by_reusing_an_old_stillness_interval(bundle,moving_source):
    race = paused_parking(bundle)
    race.step(parking_sample(1.4,1),1.4,motion=still(1.4))
    sample,motion=parking_sample(2.,2),still(2.)
    if moving_source=='pose': sample['parking']['speed_mps']=.03
    elif moving_source=='pose_overspeed': sample['parking']['speed_mps']=.3
    else: motion['moving']=True
    assert race.step(sample,2.,motion=motion)['control_recovering']
    for frame,now in ((3,2.6),(4,3.2)):
        result=race.step(parking_sample(now,frame),now,motion=still(now))
        assert result['control_recovering'] and not result.get('control_recovered')
    result=race.step(parking_sample(3.8,5),3.8,motion=still(3.8,2.01))
    assert result['control_recovered'] and result['esc_us']==1500


def test_duplicate_parking_pose_does_not_add_recovery_votes(bundle):
    race=paused_parking(bundle)
    sample=parking_sample(1.4,1)
    for now in (1.4,1.45,1.5):
        result=race.step(sample,now,motion=still(now))
        assert result['control_recovering'] and race.control_pause['hits']==1
    result=race.step(parking_sample(2.,2),2.,motion=still(2.))
    assert result['control_recovered']
