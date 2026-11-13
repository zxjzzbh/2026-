"""Hidden entrance continuity: current landmarks, stage memory and raw replay."""
from pathlib import Path
import cv2
import numpy as np
import pytest
from carvision.parking_bay_pose import ParkingBayPose
from carvision.parking_ground_view import ParkingGroundView
from carvision.parking_landmarks import ParkingLandmarks
from carvision.parking_entry import ParkingEntryController
from carvision.parking_progress import ParkingProgress
from parking_replay_support import replay
from test_parking_stages import ground,route,quad,sample,drive
from test_parking_entry import settings


def landmarks(fid,t,distance=.5,side='both',x=0.,angle=0.):
    normal=np.array([np.sin(angle),np.cos(angle)])
    tangent=np.array([normal[1],-normal[0]])
    def pixel(along,lateral):
        p=normal*along+tangent*(x+lateral)
        return [240+p[0]/.005,(4.1-p[1])/.01]
    edges={name:[pixel(distance+.02,offset),pixel(distance+1.,offset)]
           for name,offset in [('left',-.3125),('right',.3125)] if side in ('both',name)}
    return {'id':'left','source':'current_bay_landmarks','frame_id':fid,'captured_s':t,
        'far_observed':True,'far_edge_px':[pixel(distance+1.,-.3125),pixel(distance+1.,.3125)],
        'side_edges_px':edges,'side_support':{k:1. for k in edges}}


@pytest.mark.parametrize('side',['both','left','right'])
@pytest.mark.parametrize('angle',[-.12,0.,.12])
def test_current_far_and_visible_side_fit_bay_without_entrance(side,angle):
    pose=ParkingBayPose(ParkingGroundView(ground()),.625,1.)
    row=pose.measure(landmarks(2,.2,side=side,angle=angle),2,.2)
    assert row is not None,pose.reason
    assert row['distance_m']==pytest.approx(.5,abs=.002)
    assert row['normal']==pytest.approx([np.sin(angle),np.cos(angle)],abs=.002)
    assert not row['entrance_observed'] and not row['physical_pose_verified']
    assert not row['geometry_assumption_verified']


@pytest.mark.parametrize('bad',['stale','far_missing','far_outside','side_missing','side_outside','support','width','bent','nan'])
def test_invalid_or_outside_landmarks_do_not_become_current_position(bad):
    pose=ParkingBayPose(ParkingGroundView(ground()),.625,1.)
    row=landmarks(2,.2)
    if bad=='stale':row['frame_id']=1
    if bad=='far_missing':row['far_observed']=False
    if bad=='far_outside':row['far_edge_px']=[[150,200],[330,200]]
    if bad=='side_missing':row['side_edges_px']={}
    if bad=='side_outside':row['side_edges_px']={'left':[[15,250],[20,320]]}
    if bad=='support':row['side_support']={'left':.5,'right':.5}
    if bad=='width':
        for p in row['side_edges_px']['right']:p[0]+=40
    if bad=='bent':row['far_edge_px'][1][1]+=22
    if bad=='nan':row['far_edge_px'][0][0]=float('nan')
    assert pose.measure(row,2,.2) is None


def test_acquired_bay_recovers_from_current_landmarks_after_motion_chain_break():
    r=route();drive(r,0,quad(.95))
    t=3*.1
    out=r.step(sample(3,valid=False),None,False,{}, {},t,
               was_driving=True,landmarks=landmarks(3,t))
    assert r.report['pose_source']=='current_far_side_bay_pose'
    assert r.report['entry_distance_m']==pytest.approx(.5)
    assert not r.report['current_entrance_measurement']
    assert out['action']=='creep'
    assert not r.report['front_crossing_confirmed']


def test_unacquired_target_cannot_be_created_by_inferred_hidden_entrance():
    r=route();out=r.step(sample(0),None,False,{}, {},0.,landmarks=landmarks(0,0.))
    assert r.anchor is None and out['action']=='neutral'


@pytest.mark.parametrize('dt,x',[(1.3,0.),(.1,.5)])
def test_far_side_reacquisition_rejects_expired_or_jumping_target(dt,x):
    r=route();drive(r,0,quad(.95))
    out=r.step(sample(1,t=dt,valid=False),None,False,{}, {},dt,
               was_driving=True,landmarks=landmarks(1,dt,x=x))
    assert r.anchor is None and out['action']=='brake'


def test_inconsistent_bay_length_disables_hidden_entrance_inference_for_this_target():
    r=route();drive(r,0,quad(.95))
    # Far+side geometry puts the entrance at .5m, incompatible with the
    # currently measured .95m entrance. No later frame may ignore this.
    drive_sample=sample(1,valid=False)
    r.step(drive_sample,quad(.95),True,{'availability':'clear'}, {},.1,
           landmarks=landmarks(1,.1),was_driving=True)
    assert r.anchor['source']=='current_calibrated_entrance'
    assert not r.bay_model_consistent
    out=r.step(sample(2,valid=False),None,False,{}, {},.2,
               landmarks=landmarks(2,.2),was_driving=True)
    assert r.anchor is None and out['action']=='brake'
    r.retarget();assert r.bay_model_consistent and not r.position_acquired


def test_model_uncertainty_is_kept_when_ground_flow_takes_over():
    r=route();drive(r,0,quad(.95))
    t=3*.1
    r.step(sample(3,valid=False),None,False,{}, {},t,landmarks=landmarks(3,t,side='left'))
    initial=r.error
    r.step(sample(4,dy=.01),None,False,{'far_edge_y_normalized':.8}, {},.4)
    assert r.anchor['source']=='bounded_ground_motion_estimate'
    assert r.error>=initial and r.error>r.view.error


def test_progress_confirmation_uses_distinct_current_frames_and_survives_loss():
    p=ParkingProgress();assert p.observe('align_entry',0.)=='align_entry'
    assert p.observe('cross_entry',.1)=='align_entry'
    assert p.observe('cross_entry',.1)=='align_entry'
    p.missing();assert not p.entered
    assert p.observe('cross_entry',.2)=='align_entry'
    assert p.observe('cross_entry',.3)=='cross_entry' and p.entered
    p.missing();assert p.stage=='cross_entry'
    assert p.observe('approach',.4)=='cross_entry'
    assert p.observe('inside_estimate',.5)=='cross_entry'
    assert p.observe('inside_estimate',.6)=='inside_estimate'


def test_lost_position_retains_crossing_stage_but_cannot_finish_or_keep_driving():
    r=route();drive(r,0,quad(.95))
    for fid in range(1,10):drive(r,fid,near=False,dy=.09)
    assert r.progress.entered
    out=drive(r,10,near=False,valid=False)
    assert r.report['stage']=='cross_entry' and r.report['front_crossing_confirmed']
    assert not r.report['entrance_position_available'] and out['action']=='brake'
    assert not r.report['rear_entry_confirmed']


def test_latched_progress_never_overrides_a_conflicting_current_position():
    r=route();drive(r,0,quad(.95))
    r.progress.observe('cross_entry',.1);r.progress.observe('cross_entry',.2)
    out=drive(r,3,quad(.95),valid=False)
    assert out['action']=='brake' and '阶段冲突' in r.last_plan['reason']


def test_braking_can_finish_with_fresh_bounded_position_after_far_edge_leaves_view():
    r=route();drive(r,0,quad(.85))
    for fid in range(1,12):out=drive(r,fid,near=False,dy=.1)
    assert out['action']=='brake' and r.stopping is not None
    for fid in range(12,26):
        s=sample(fid,dy=.05 if fid<17 else 0.,moving=fid<17)
        out=r.step(s,None,False,{}, {},s['captured_s'])
        assert out['action']!='creep'
    assert out['phase']=='done' and r.report['rear_inside_estimate']
    assert r.report['side_clearance_estimate_m']>0 and r.report['far_clearance_estimate_m']>0
    assert r.report['pose_source']=='bounded_ground_motion_estimate'
    assert not r.report['physical_pose_verified']


def test_crossed_bay_cannot_retarget_side_after_current_report_is_lost():
    controller=ParkingEntryController(settings(),selected_side='left',ground=ground(),
                                     visual_options={'ground_pose_confirmed':True})
    controller.route.progress.observe('cross_entry',0.)
    controller.route.progress.observe('cross_entry',.1)
    controller.route.report={}  # A missing measurement must not permit a switch.
    reason=controller._retarget_if_blocked({'slots':[{'id':'left','availability':'blocked'}]},.2)
    assert reason and controller.side=='left' and controller.retarget_count==0


def test_landmarks_need_current_pixels_and_reject_explicit_obstruction():
    tracker=ParkingLandmarks();marks=np.zeros((360,480),np.uint8)
    q=np.array([[100,350],[300,350],[300,250],[100,250]],np.int32)
    cv2.polylines(marks,[q],True,255,8)
    floor=cv2.bitwise_not(marks);zero=np.zeros_like(marks)
    obs={'entry_candidates':[{'id':'left','availability':'clear','corners_px':q.tolist()}]}
    assert tracker.update(obs,0,0.,marks,floor,zero)
    blocked={**obs,'slots':[{'id':'left','availability':'blocked'}]}
    assert not tracker.update(blocked,1,.1,marks,floor,zero)
    assert not tracker.update(obs,2,.2,zero,floor,zero)


def test_stale_side_reference_without_current_far_is_not_an_observation():
    tracker=ParkingLandmarks();zero=np.zeros((360,480),np.uint8)
    tracker.references['left']={'time':0.,'frame':0,'far':np.array([[150,240],[300,240]]),
        'sides':{'left':np.array([[150,240],[100,350]])}}
    assert tracker.update({},1,.1,zero,zero,zero)==[]


@pytest.fixture(scope='module')
def recorded():return replay(Path(__file__).parent/'fixtures/parking-bay-pose-20261108',staged=True)


def test_actual_last_run_recovers_position_without_requiring_entrance_to_reappear(recorded):
    rows={r['frame_id']:r for r in recorded}
    assert rows[2351]['intent']['esc_us']==1400
    for fid in (2359,2361,2362,2363):
        p=rows[fid]['intent']['entry_progress']
        assert p['pose_source']=='current_far_side_bay_pose'
        assert p['entrance_position_available'] and not p['current_entrance_measurement']
        assert not p['bay_pose']['geometry_assumption_verified']
    last=rows[2363]['intent']
    assert last['phase']=='boundary_wait' and last['entry_progress']['plan_confirmed']
    assert last['entry_progress']['recovery_failure']=='stop_confirmation_pending'
    # This recording ends before continuous stop evidence. No false restart,
    # physical crossing or completed new trajectory is claimed from replay.
    assert not last['done'] and not last['entry_progress']['front_crossing_confirmed']
    assert all(r['intent']['esc_us']<=1500 or r['intent']['esc_us']>=1570 for r in recorded)
    assert all(not r['intent']['whole_vehicle_inside_verified'] for r in recorded)
