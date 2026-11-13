"""Stage and route checks, calibrated pixels, real replay; no physical output."""
from pathlib import Path
import copy,math
import cv2
import numpy as np
import pytest
from carvision.parking_ground_view import ParkingGroundView,ParkingGroundMotion
from carvision.parking_route import ParkingRoute
from carvision.parking_entry import ParkingEntryController
from carvision.parking_entry_run import ParkingEntryRun
from parking_replay_support import replay
from test_parking_entry import settings
from test_parking_entry_run import configurations,base,preparation

DATA=Path(__file__).parent/'fixtures/parking-stages-20261108'


def ground():
    # Synthetic calibration with a deliberately limited 0.8-1.8m range.
    return {'image_size':[480,360],'homography':[[.005,0,-1.2],[0,-.01,4.1],[0,0,1]],
        'valid_polygon_px':[[70,330],[410,330],[410,230],[70,230]],
        'coordinate_frame':'rear_axle_x_right_y_forward','verified':True,'max_error_m':.03}


def route():return ParkingRoute(ground(),width=.3,front=.335,rear=.085,bay_width=.625,bay_length=1.)


def quad(distance=1.1,x=0.):
    return np.array([[240+(x-.3125)/.005,(4.1-distance)/.01],
                     [240+(x+.3125)/.005,(4.1-distance)/.01],
                     [290,180],[190,180]],np.float32)


def sample(fid,t=None,*,dy=0.,moving=True,valid=True):
    t=fid*.1 if t is None else t
    return {'frame_id':fid,'captured_s':t,'motion':{'valid':True,'moving':moving},
        'parking_ground_motion':{'valid':valid,'frame_id':fid,'captured_s':t,
            'previous_frame_id':fid-1,'previous_captured_s':(fid-1)*.1,
            'transform':[[1.,0.,0.],[0.,1.,-dy]],'translation_m':abs(dy),'rms_m':0.}}


def drive(r,fid,q=None,near=True,**kw):
    s=sample(fid,**kw)
    return r.step(s,quad() if q is None else q,near,{'availability':'clear'},
                  {'observed':False},s['captured_s'],was_driving=True)


def test_projection_rejects_pixels_outside_existing_range():
    view=ParkingGroundView(ground())
    assert view.point([240,220]) is None and view.point([240,340]) is None
    assert view.point([240,300])==pytest.approx([0,1.1])
    assert view.entrance(quad())['distance_m']==pytest.approx(1.1)
    assert view.entrance(quad(2.)) is None


def test_existing_mapping_is_not_promoted_to_verified_current_pose():
    g=ground();g['verified']=False;g['historical_verified']=True
    view=ParkingGroundView(g);entry=view.entrance(quad())
    assert not entry['physical_pose_verified'] and g.get('current_pose_verified') is None
    g['historical_verified']=False
    with pytest.raises(ValueError):ParkingGroundView(g)


def test_forward_bay_image_is_not_a_current_vehicle_footprint():
    r=route();q=quad(1.5,x=-.4)
    out=drive(r,0,q)
    assert out['action']=='creep' and r.report['stage']=='approach'
    assert r.report['side_clearance_estimate_m']<0
    assert not r.report['front_crossed_estimate']
    assert not r.report['image_corridor_is_vehicle_body']


def test_metric_stage_changes_need_entrance_crossing_evidence():
    r=route();drive(r,0,quad(.95))
    stages=[]
    for fid in range(1,15):
        # The same currently visible far/side corridor does not invent an
        # entrance; only continuous ground motion advances the measured anchor.
        out=drive(r,fid,quad(),near=False,dy=.09)
        stages.append(r.report['stage'])
    assert 'approach' in stages and 'align_entry' in stages and 'cross_entry' in stages
    # Rear crossing is confirmed on two new frames, independently of the
    # earlier front crossing, rather than accepting a single noisy estimate.
    assert stages[-2]=='cross_entry' and stages[-1]=='inside_estimate'
    assert not r.report['physical_pose_verified']


@pytest.mark.parametrize('failure',['gap','out_of_order','transform','travel','error','nan','negative','reflection'])
def test_invalid_motion_never_carries_an_old_entrance_into_a_new_pose(failure):
    r=route();drive(r,0);s=sample(1,dy=.1)
    if failure=='gap':s['parking_ground_motion']['valid']=False
    if failure=='out_of_order':s['parking_ground_motion']['previous_frame_id']=-5
    if failure=='transform':s['parking_ground_motion']['transform']=[[2,0,0],[0,2,0]]
    if failure=='travel':s['parking_ground_motion']['translation_m']=1.6
    if failure=='error':s['parking_ground_motion']['rms_m']=.1
    if failure=='nan':s['parking_ground_motion']['rms_m']=float('nan')
    if failure=='negative':s['parking_ground_motion']['translation_m']=-.1
    if failure=='reflection':s['parking_ground_motion']['transform']=[[-1,0,0],[0,1,0]]
    out=r.step(s,quad(),False,{'availability':'clear'},{'observed':False},.1,was_driving=True)
    assert r.anchor is None and out['action']=='brake'
    assert not r.report['entrance_position_available']


def test_steering_moves_inward_when_heading_competes_with_side_margin():
    r=route()
    angle=.12;n=np.array([math.sin(angle),math.cos(angle)]);t=np.array([n[1],-n[0]])
    entry={'center_m':(.7*n-.075*t).tolist(),'normal':n.tolist(),'distance_m':.7,'source':'test'}
    r.report={};plan=r._plan(quad(),True,{'availability':'clear'},{},entry,0.)
    assert plan['allowed'] and plan['error']<0
    assert r.report['side_clearance_estimate_m']<.07


def test_inside_margin_failure_is_not_masked_by_an_approach_stage():
    r=route();entry={'center_m':[.3,.1],'normal':[0,1],'distance_m':.1,'source':'test'}
    plan=r._plan(quad(),False,{'availability':'clear'},{},entry,0.)
    assert not plan['allowed'] and plan['stage']=='cross_entry'


def test_stationary_prepare_does_not_send_reverse_braking_for_missing_position():
    r=route();out=r.step(sample(0,moving=False),None,False,{}, {},0.)
    assert out['action']=='neutral'


def test_resume_requires_new_route_confirmation_and_stop_feedback():
    r=route();drive(r,0)
    out=r.step(sample(1),None,False,{}, {},.1,was_driving=True)
    assert out['action']=='brake'
    for fid in range(2,7):
        out=drive(r,fid,moving=True)
        assert out['action']!='creep'
    for fid in range(7,10):out=drive(r,fid,moving=False)
    assert out['action']=='creep' and r.plan_revision==1
    # Losing evidence again cannot reuse the old stop or plan confirmation.
    out=r.step(sample(10,moving=False),None,False,{}, {},1.,was_driving=True)
    assert out['action']=='brake' and not r.report['plan_confirmed']


def test_finite_final_brake_needs_position_and_stationary_frames_to_finish():
    r=route();drive(r,0,quad(.85))
    out=None
    # Stay below the bounded travel budget and seed a nearer last measured line.
    drive(r,1,quad(.85))
    for fid in range(2,13):out=drive(r,fid,quad(),near=False,dy=.1)
    assert out['action']=='brake' and r.report['rear_inside_estimate']
    for fid in range(13,24):out=drive(r,fid,quad(),near=False,dy=0.,moving=False)
    assert out['phase']=='done' and out['action']=='neutral'
    assert not r.report['physical_pose_verified']


def test_missing_mapping_fails_preparation_without_hardware():
    p,b,c=configurations();p.setdefault('visual_entry',{})['ground_pose_confirmed']=True
    opened=[];run=ParkingEntryRun(lambda _:opened.append(True))
    with pytest.raises(ValueError):run.prepare(p,b,c,base(),preparation(),0.)
    assert not opened and not run.active


def test_raw_failure_sequence_does_not_repeat_virtual_body_fault_or_rightward_restart():
    rows=replay(DATA,staged=True);by_frame={r['frame_id']:r for r in rows}
    for fid in (2384,2397,2404,2417):
        intent=by_frame[fid]['intent']
        assert intent['phase']=='approach' and intent['esc_us']==1570
        assert intent['entry_progress']['stage']=='approach'
        assert not intent['entry_progress']['image_corridor_is_vehicle_body']
        assert not intent['done'] and not intent['whole_vehicle_inside_verified']
    assert by_frame[2397]['intent']['steering_us']>1610
    assert by_frame[2404]['intent']['steering_us']>1610
    assert by_frame[2417]['intent']['entry_progress']['entry_distance_m']>.8
    assert all(r['intent']['esc_us']<=1500 or r['intent']['esc_us']>=1570 for r in rows)


def test_ground_motion_uses_current_floor_and_rejects_blank_frames():
    rng=np.random.default_rng(3);image=rng.integers(0,255,(360,480,3),dtype=np.uint8)
    flow=ParkingGroundMotion(ground());mask=np.full((360,480),255,np.uint8)
    assert not flow.update(image,0,0.,mask)['valid']
    result=flow.update(image,1,.1,mask)
    assert result['valid'] and abs(result['forward_speed_mps'])<.001
    assert not flow.update(np.zeros_like(image),2,.2,np.zeros_like(mask))['valid']


def test_current_far_edge_and_continuous_pose_bridge_hidden_entrance():
    r=route();drive(r,0)
    s=sample(1,dy=.1)
    out=r.step(s,None,False,{'far_edge_y_normalized':.6},{},.1,was_driving=True)
    assert out['action']=='creep' and r.report['entry_distance_m']==pytest.approx(1.)
    out=r.step(sample(2,valid=False),None,False,{'far_edge_y_normalized':.6},{},.2,was_driving=True)
    assert out['action']=='brake' and not r.report['entrance_position_available']


def test_stop_never_completes_after_current_position_is_lost():
    r=route();drive(r,0,quad(.85))
    for fid in range(1,12):out=drive(r,fid,near=False,dy=.1)
    assert out['action']=='brake'
    for fid in range(12,39):out=drive(r,fid,near=False,valid=False,moving=False)
    assert out['phase']=='fault' and out['action']=='neutral'


def test_current_ground_translation_has_correct_direction_and_scale():
    rng=np.random.default_rng(9);image=rng.integers(0,255,(360,480,3),dtype=np.uint8)
    flow=ParkingGroundMotion(ground());mask=np.full((360,480),255,np.uint8)
    flow.update(image,0,0.,mask)
    shifted=cv2.warpAffine(image,np.float32([[1,0,0],[0,1,3]]),(480,360))
    result=flow.update(shifted,1,.1,mask)
    assert result['valid'] and result['forward_speed_mps']==pytest.approx(.3,abs=.02)
    assert result['transform'][1][2]==pytest.approx(-.03,abs=.002)


def test_final_brake_allows_reported_quarter_metre_coast_without_false_arrival():
    r=route();drive(r,0,quad(.85))
    for fid in range(1,12):out=drive(r,fid,near=False,dy=.1)
    assert out['action']=='brake'
    for fid in range(12,17):out=drive(r,fid,near=False,dy=.05)
    assert out['phase']!='done' and r.anchor is not None
    for fid in range(17,25):out=drive(r,fid,near=False,dy=0,moving=False)
    assert out['phase']=='done' and r.report['far_clearance_estimate_m']>0
