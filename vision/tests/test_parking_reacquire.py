"""Current-pixel recovery, broken motion chains and the 17:43 camera run."""
from pathlib import Path
import copy,json
import cv2
import numpy as np
import pytest
from carvision.parking_far_tracker import ParkingFarTracker
from carvision.parking_ground_view import ParkingGroundMotion
from carvision.parking_stop_evidence import ParkingStopEvidence
from carvision.parking_bay import detect_parking
from parking_replay_support import replay
from test_parking_continuity import camera
from test_parking_stages import ground,route,quad,sample,drive

DATA=Path(__file__).parent/'fixtures/parking-reacquire-20261108'


def test_partial_track_retries_real_entrance_without_full_detector_box():
    image,config=camera();tracker=ParkingFarTracker();obs=detect_parking(image,config)[0]
    tracker.update(image,obs,0,0.,config)
    track=next(iter(tracker.boundaries.tracks.values()))
    track['near_boundary_observed']=False
    track['quad']=track['quad'].copy();track['quad'][:2,1]-=8
    current={'slots':[],'entry_candidates':[]}
    tracker.update(image,current,1,.1,config)
    row=current['tracked_bay_boundaries'][0]
    assert row['near_boundary_observed'] and row['entrance_reacquired']
    assert min(row['edge_support'])>=.72


def test_current_obstruction_still_rejects_reacquired_track():
    image,config=camera();tracker=ParkingFarTracker();obs=detect_parking(image,config)[0]
    tracker.update(image,obs,0,0.,config)
    side=next(iter(tracker.boundaries.tracks))
    tracker.boundaries.tracks[side]['near_boundary_observed']=False
    current={'slots':[{'id':side,'availability':'blocked'}]}
    tracker.update(image,current,1,.1,config)
    assert current['tracked_bay_boundaries']==[]


def test_ground_motion_can_match_current_frame_directly_over_a_blank_frame():
    rng=np.random.default_rng(4);image=rng.integers(0,255,(360,480,3),dtype=np.uint8)
    mask=np.full((360,480),255,np.uint8);flow=ParkingGroundMotion(ground())
    flow.update(image,0,0.,mask);assert flow.update(image,1,.1,mask)['valid']
    assert not flow.update(np.zeros_like(image),2,.2,np.zeros_like(mask))['valid']
    row=flow.update(image,3,.3,mask)
    assert not row['valid']
    matching=[v for v in row['relocalizations'] if v['previous_frame_id']==1]
    assert matching and matching[0]['valid'] and matching[0]['captured_s']==.3


def test_retained_reference_is_not_current_position_until_direct_match_succeeds():
    r=route();drive(r,0)
    assert drive(r,1,near=False,valid=False)['action']=='brake'
    assert r.anchor is None and r.reference['frame']==0
    s=sample(2,dy=.05)
    recovered={**s['parking_ground_motion'],'previous_frame_id':0,'previous_captured_s':0.,
        'transform':[[1,0,0],[0,1,-.12]],'translation_m':.12}
    s['parking_ground_motion']['relocalizations']=[recovered]
    out=r.step(s,quad(),False,{'availability':'clear'},{},.2,was_driving=True)
    assert r.anchor is not None and r.report['entry_distance_m']==pytest.approx(.98)
    assert out['action']!='creep'  # Brake and stop confirmation are not bypassed.


@pytest.mark.parametrize('bad',['wrong_reference','stale_capture','expired','reflection'])
def test_direct_recovery_rejects_wrong_or_expired_frame_link(bad):
    r=route();drive(r,0);drive(r,1,near=False,valid=False)
    fid=6 if bad=='expired' else 2;s=sample(fid,valid=False)
    alt={**sample(2,dy=.1)['parking_ground_motion'],'previous_frame_id':0,'previous_captured_s':0.}
    if bad=='wrong_reference':alt['previous_frame_id']=77
    if bad=='stale_capture':alt['captured_s']=.1
    if bad=='expired':alt.update(frame_id=6,captured_s=.6)
    if bad=='reflection':alt['transform']=[[-1,0,0],[0,1,0]]
    s['parking_ground_motion']['relocalizations']=[alt]
    r.step(s,quad(),False,{'availability':'clear'},{},s['captured_s'],was_driving=True)
    assert r.anchor is None and not r.report['entrance_position_available']


def stop_sample(fid,pixel,*,dy=0.,valid=True):
    s=sample(fid,dy=dy,valid=valid,moving=pixel>.4)
    s['motion']['pixel_motion_per_100ms']=pixel
    return s


def test_stationary_pixel_jitter_needs_current_ground_corroboration():
    detector=ParkingStopEvidence()
    assert not detector.update(stop_sample(1,.35),enabled=True)['moving']
    held=detector.update(stop_sample(2,.43),enabled=True)
    assert not held['moving'] and held['parking_stop_hysteresis']
    assert detector.update(stop_sample(3,.43,valid=False),enabled=True)['moving']


@pytest.mark.parametrize('kind',['translation','pixel','disabled','stale_ground'])
def test_stop_hysteresis_never_hides_real_motion_or_uses_old_ground(kind):
    detector=ParkingStopEvidence();detector.update(stop_sample(1,.35),enabled=True)
    s=stop_sample(2,.43)
    if kind=='translation':s=stop_sample(2,.35,dy=.02)
    if kind=='pixel':s=stop_sample(2,1.)
    if kind=='stale_ground':s['parking_ground_motion']['frame_id']=1
    result=detector.update(s,enabled=kind!='disabled')
    assert result['moving']


def test_position_failure_messages_separate_line_range_and_motion():
    r=route();drive(r,0,quad(.5))
    assert r.position_problem=='entrance_outside_calibration'
    assert '标定有效范围' in r.last_plan['reason']
    r=route();r.step(sample(0),None,False,{}, {},0.)
    assert r.position_problem=='entrance_line_missing'
    r=route();drive(r,0);drive(r,1,near=False,valid=False)
    assert r.position_problem=='ground_motion_invalid'
    assert '地面运动匹配失败' in r.last_plan['reason']


def test_lost_metric_position_cannot_fall_back_to_initial_white_line_search():
    r=route();drive(r,0)
    out=r.step(sample(1,valid=False),None,False,{}, {'observed':True},.1,was_driving=True)
    assert out['action']=='brake' and not r.last_plan['allowed']


@pytest.fixture(scope='module')
def recorded():return replay(DATA,staged=True)


def test_original_stop_frame_reacquires_measured_entrance_without_trusting_bad_flow(recorded):
    indexed={r['frame_id']:r for r in recorded}
    for fid in (3292,3293,3294):
        row=indexed[fid];out=row['intent'];p=out['entry_progress']
        assert out['esc_us']==1570 and p['current_entrance_measurement']
        assert p['entrance_position_available'] and p['position_failure'] is None
        assert not row['sample']['parking_ground_motion']['valid']
    edge=indexed[3292]['sample']['parking']['tracked_bay_boundaries'][0]
    assert edge['search_radius_px']==24 and edge['near_boundary_observed']


def test_replay_does_not_invent_position_after_all_measured_links_fail(recorded):
    after=[r for r in recorded if r['frame_id']>=3295]
    assert after and all(r['intent']['esc_us']<=1500 for r in after)
    assert all(not r['intent']['entry_progress']['entrance_position_available'] for r in after)
    assert all(not r['intent']['done'] and not r['intent']['whole_vehicle_inside_verified'] for r in recorded)
    assert all(r['intent']['esc_us']<=1500 or r['intent']['esc_us']>=1570 for r in recorded)
    assert after[-1]['intent']['boundary_wait_elapsed_s']>0
    assert after[-1]['intent']['recovery_timing']['boundary_missing_s'] is not None
    assert after[-1]['intent']['brake_trigger']['far_edge_y_normalized'] is None
