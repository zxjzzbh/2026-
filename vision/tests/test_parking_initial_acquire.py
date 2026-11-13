"""Partial calibrated entrances, bounded initial alignment and recorded failures."""
from pathlib import Path
import copy
import numpy as np
import pytest
from carvision.parking_ground_view import ParkingGroundView
from carvision.parking_bay_pose import ParkingBayPose
from carvision.parking_approach import ParkingApproach
from carvision.parking_route import ParkingRoute
from parking_replay_support import replay
from test_parking_stages import ground,quad,sample,route


def partial():
    g=ground();g['valid_polygon_px']=[[210,330],[410,330],[410,230],[210,230]]
    q=np.array([[117.5,300],[242.5,300],[242.5,240],[117.5,240]],np.float32)
    return g,q


def geometry():
    return {'availability':'clear','far_edge_y_normalized':.5,'envelope_checks':[
        {'left_px':110.,'right_px':230.,'center_px':240.},
        {'left_px':90.,'right_px':240.,'center_px':240.}]}


def test_partial_entrance_keeps_distance_without_projecting_outside_pixels():
    g,q=partial();v=ParkingGroundView(g)
    assert v.point(q[:2].mean(axis=0)) is None and v.entrance(q) is None
    line=v.entrance_line(q)
    assert line['distance_m']==pytest.approx(1.1) and line['normal']==pytest.approx([0,1])
    before=copy.deepcopy(g)
    pose=ParkingBayPose(v,.625,1.).entrance(q,line)
    assert pose['center_m']==pytest.approx([-.3,1.1])
    assert pose['error_estimate_m']==pytest.approx(.07)
    assert pose['visible_sides']==['right']
    assert pose['centre_inferred'] and not pose['geometry_assumption_verified']
    assert not pose['physical_pose_verified'] and g==before


@pytest.mark.parametrize('failure',['short_side','angled_side','outside_corner','short_entrance'])
def test_partial_pose_requires_current_usable_entrance_and_side(failure):
    g,q=partial();v=ParkingGroundView(g)
    if failure=='short_side':q[2]=[242.5,296]
    if failure=='angled_side':q[2]=[320,240]
    if failure=='outside_corner':q[1]=[205,300]
    if failure=='short_entrance':q[1]=[220,300]
    assert ParkingBayPose(v,.625,1.).entrance(q,v.entrance_line(q)) is None


def test_partial_pose_establishes_alignment_with_uncertainty_and_left_correction():
    g,q=partial();r=ParkingRoute(g,width=.3,front=.335,rear=.085,bay_width=.625,bay_length=1.)
    out=r.step(sample(0),q,True,geometry(),{},0.,was_driving=True)
    assert out['action']=='creep' and out['steering_error']<0
    assert r.position_acquired and r.report['current_entrance_measurement']
    assert r.report['position_error_estimate_m']==pytest.approx(.07)
    assert not r.report['front_crossing_confirmed']
    out=r.step(sample(1,dy=.01),q,False,geometry(),{},.1,was_driving=True)
    assert out['action']=='creep' and r.error>.07
    assert r.anchor['source']=='bounded_ground_motion_estimate'


def test_far_classification_uses_sloped_polygon_edge_not_minimum_row():
    g=ground();g['valid_polygon_px']=[[70,330],[410,330],[410,230],[70,280]]
    q=np.array([[100,270],[170,260],[160,200],[120,200]],np.float32)
    v=ParkingGroundView(g);initial=ParkingApproach(v,.335,5.)
    assert q[:2,1].mean()>v.poly[:,1].min()+8
    assert initial.region(q,None)=='far'
    out=initial.plan(q,True,geometry(),{},None,0.,sample(0))
    assert out['allowed']


@pytest.mark.parametrize('q',[
    np.array([[10,300],[60,300],[60,240],[10,240]],np.float32),
    np.array([[180,345],[300,345],[300,335],[180,335]],np.float32)])
def test_lateral_or_near_blind_region_cannot_authorize_forward_search(q):
    a=ParkingApproach(ParkingGroundView(ground()),.335,5.)
    assert not a.plan(q,True,geometry(),{},None,0.,sample(0))['allowed']


def test_initial_wait_does_not_use_running_position_loss_timeout():
    r=route()
    for fid in range(50):
        out=r.step(sample(fid,moving=False),None,False,{}, {},fid*.1)
        assert out['phase']=='boundary_wait' and out['action']=='neutral'
    out=r.step(sample(50,moving=False),None,False,{}, {},5.)
    assert out['phase']=='fault' and '初次车位定位' in out['reason']


def test_initial_deadline_brakes_before_terminal_neutral():
    r=route();q=quad(2.)
    for fid in range(50):
        out=r.step(sample(fid),q,True,geometry(),{},fid*.1,was_driving=True)
        assert out['action']=='creep'
    out=r.step(sample(50),q,True,geometry(),{},5.,was_driving=True)
    assert out['action']=='brake'
    for fid in range(51,54):
        assert r.step(sample(fid),q,True,geometry(),{},fid*.1)['action']=='brake'
    assert r.step(sample(54),q,True,geometry(),{},5.4)['phase']=='fault'


def test_far_only_blink_needs_fresh_link_and_cannot_extend_full_boundary_age():
    a=ParkingApproach(ParkingGroundView(ground()),.335,5.);q=quad(2.)
    assert a.plan(q,True,geometry(),{},None,0.,sample(0))['allowed']
    far={'source':'current_tracked_far_edge','far_edge_px':q[[3,2]].tolist()}
    for fid in (1,2):
        assert a.plan(None,False,far,{},None,fid*.1,sample(fid))['allowed']
        assert a.report['bridge_used']
    assert not a.plan(None,False,far,{},None,.3,sample(3))['allowed']


@pytest.mark.parametrize('bad',['no_seed','invalid_flow','wrong_frame','far_jump','old_capture','blocked'])
def test_blink_does_not_mask_missing_or_conflicting_current_evidence(bad):
    a=ParkingApproach(ParkingGroundView(ground()),.335,5.);q=quad(2.)
    if bad!='no_seed':a.plan(q,True,geometry(),{},None,0.,sample(0))
    s=sample(1);far={'source':'current_tracked_far_edge','far_edge_px':q[[3,2]].tolist()}
    if bad=='invalid_flow':s['parking_ground_motion']['valid']=False
    if bad=='wrong_frame':s['parking_ground_motion']['previous_frame_id']=42
    if bad=='old_capture':s['parking_ground_motion']['captured_s']=0.
    if bad=='far_jump':far['far_edge_px']=(q[[3,2]]+[90,0]).tolist()
    if bad=='blocked':
        assert not a.plan(q,False,{'availability':'blocked'}, {},None,.1,s)['allowed']
    else:assert not a.plan(None,False,far,{},None,.1,s)['allowed']


def test_partial_three_edge_observation_bridges_once_without_inventing_entrance():
    a=ParkingApproach(ParkingGroundView(ground()),.335,5.);q=quad(2.)
    a.plan(q,True,geometry(),{},None,0.,sample(0))
    assert a.plan(q,False,geometry(),{},None,.1,sample(1))['allowed']
    assert a.report['bridge_used'] and a.full_at==0.
    assert not a.plan(q,False,geometry(),{},None,.3,sample(3))['allowed']
    a.reset()
    assert not a.plan(q,False,geometry(),{},None,.4,sample(4))['allowed']


def test_current_road_corridor_preserves_short_far_edge_bridge_when_flow_is_invalid():
    a=ParkingApproach(ParkingGroundView(ground()),.335,5.);q=quad(2.)
    a.plan(q,True,geometry(),{},None,0.,sample(0))
    far={'source':'current_tracked_far_edge','far_edge_px':q[[3,2]].tolist()}
    assert a.plan(None,False,far,{'observed':True},None,.1,sample(1,valid=False))['allowed']
    assert not a.plan(None,False,far,{'observed':True},None,.3,sample(3,valid=False))['allowed']


@pytest.fixture(scope='module')
def recorded():
    return replay(Path(__file__).parent/'fixtures/parking-acquire-offset-20261109',staged=True)


def test_recorded_blinks_and_partial_entrance_no_longer_cause_repeated_stops(recorded):
    rows={r['frame_id']:r['intent'] for r in recorded}
    for fid in (7911,7926,7945,7957):
        assert rows[fid]['action']=='creep', (fid,rows[fid]['reason'])
    # Frame 7945 has only a few side pixels in the calibration polygon.
    # Subpixel platform differences may legitimately leave it unmeasured;
    # the later frame must acquire position once sufficient side is in range.
    for fid in (7957,):
        progress=rows[fid]['entry_progress']
        assert progress['entrance_position_available']
        assert progress['pose_source']=='current_entrance_side_estimate'
        assert not progress['entrance_centre_estimate']['geometry_assumption_verified']
    assert all(r['intent']['esc_us']<=1500 or r['intent']['esc_us']>=1570 for r in recorded)
    assert all(not r['intent']['done'] and not r['intent']['whole_vehicle_inside_verified'] for r in recorded)


def test_stationary_recording_does_not_mask_failure_to_move():
    rows=replay(Path(__file__).parent/'fixtures/parking-acquire-stationary-20261109',staged=True)
    assert rows[-1]['intent']['phase']=='fault'
    assert '未观察到移动' in rows[-1]['intent']['reason']
