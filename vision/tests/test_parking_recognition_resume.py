"""Current board/tape evidence and restart guards, including Nov 9 recordings."""
from pathlib import Path
import json
import cv2
import numpy as np
import pytest
from carvision.parking_bay import detect_parking,board_direction_hint,blue_board_mask
from carvision.parking_inner_edges import inner_line
from carvision.parking_ground_view import ParkingGroundView
from carvision.parking_approach import ParkingApproach
from carvision.parking_bay_pose import ParkingBayPose
from parking_replay_support import replay
from test_parking_initial_acquire import partial,geometry
from test_parking_stages import route,quad,sample

DATA=Path(__file__).parent/'fixtures'
FIRST=DATA/'parking-recognition-first-20261109'
SECOND=DATA/'parking-recognition-second-20261109'
PI_CASES=json.loads((DATA/'parking-pi-edge-cases-20261110.json').read_text(encoding='utf8'))


def raw(folder,fid):
    frame=next(r for r in map(json.loads,(folder/'frames.jsonl').read_text().splitlines())
               if r['frame_id']==fid)
    with (folder/'frames.mjpg').open('rb') as stream:
        stream.seek(frame['offset'])
        image=cv2.imdecode(np.frombuffer(stream.read(frame['length']),np.uint8),1)
    return image,json.loads((folder/'settings.json').read_text(encoding='utf8'))['parking']


@pytest.mark.parametrize('folder,fid',[(FIRST,1026),(SECOND,1264)])
@pytest.mark.parametrize('mirror',[False,True])
def test_background_connection_does_not_discard_solid_board(folder,fid,mirror):
    image,settings=raw(folder,fid)
    if mirror:image=cv2.flip(image,1)
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    original=blue_board_mask(hsv,settings).copy()
    hint=board_direction_hint(hsv,settings)
    assert hint['suggested_slot']==('right' if mirror else 'left')
    board=max(hint['boards'],key=lambda b:b['area_fraction'])
    assert 100<board['bbox_px'][1]<180 and 230<board['bbox_px'][3]<270
    # Shape cleanup must not erase original blue pixels used for occupancy.
    assert np.array_equal(original,blue_board_mask(hsv,settings))
    assert np.count_nonzero(original[:100])>0


@pytest.mark.parametrize('folder,fid',[(FIRST,1038),(SECOND,1263),(SECOND,1313)])
@pytest.mark.parametrize('mirror',[False,True])
def test_current_tape_refit_replaces_bad_contour_far_chord(folder,fid,mirror):
    image,settings=raw(folder,fid)
    if mirror:image=cv2.flip(image,1)
    obs,_=detect_parking(image,settings)
    assert len(obs['entry_candidates'])==1
    row=obs['entry_candidates'][0];q=np.array(row['corners_px'])
    assert row['id']==('right' if mirror else 'left')
    assert min(row['edge_support'])>=.7
    assert 208<q[2:,1].mean()<216
    assert obs['diagnostics']['entry_search']['refitted_boundaries']==1
    assert row['ground_fraction']>=.9 and not row['occupancy_verified']


@pytest.mark.parametrize('kind',['dark','white','blue','thin_blue'])
def test_shape_cleanup_and_refit_do_not_declare_an_occupied_bay_clear(kind):
    image,settings=raw(SECOND,1264)
    color={'dark':(0,0,0),'white':(245,245,245),'blue':(230,90,40),'thin_blue':(230,90,40)}[kind]
    a,b=((181,219),(184,241)) if kind=='thin_blue' else ((165,218),(209,239))
    cv2.rectangle(image,a,b,color,-1)
    obs,_=detect_parking(image,settings)
    assert not obs['entry_candidates']
    assert not any(s['availability']=='clear' for s in obs['slots'])


def test_scattered_transition_outliers_need_current_support_in_all_edge_sections():
    marks=np.zeros((360,480),np.uint8);floor=np.zeros_like(marks)
    marks[197:200,100:301]=255;floor[200:]=255
    for x in [139,180,221,258]:marks[203:206,x:x+8]=255
    result=inner_line(marks,floor,np.array([100.,204.]),np.array([300.,204.]),np.array([200.,240.]))
    assert result is not None
    n,d=result
    assert (d-n[0]*200)/n[1]==pytest.approx(200,abs=1.)
    # Evidence localized to one section cannot create the full far edge.
    marks[:,180:]=0
    assert inner_line(marks,floor,np.array([100.,204.]),np.array([300.,204.]),np.array([200.,240.])) is None


def test_expiring_bridge_cannot_confirm_restart_but_can_continue_motion():
    r=route();q=quad(2.)
    out=r.step(sample(0,moving=False),None,False,{}, {},0.)
    assert out['action']=='neutral'
    r.step(sample(1,moving=False),q,True,geometry(),{},.1)
    far={'source':'current_tracked_far_edge','far_edge_px':q[[3,2]].tolist(),
         'far_edge_y_normalized':float(q[2:,1].mean()/359.)}
    for fid in [2,3,4]:
        out=r.step(sample(fid,moving=False),None,False,far,{},fid*.1)
        assert out['action']=='neutral' and not r.report['plan_confirmed']
        assert not r.report['restart_evidence_current']
    for fid in [5,6,7]:
        out=r.step(sample(fid,moving=False),q,True,geometry(),{},fid*.1)
    assert out['action']=='creep' and r.report['restart_evidence_current']
    out=r.step(sample(8),None,False,far,{},.8,was_driving=True)
    assert out['action']=='creep' and r.report['initial_position']['bridge_used']


@pytest.mark.parametrize('distance,allowed',[(1.1,True),(.8,False)])
def test_partial_calibration_bridge_keeps_conservative_range_budget(distance,allowed):
    g,q=partial();q[:2,1]=(4.1-distance)/.01
    view=ParkingGroundView(g);a=ParkingApproach(view,.335,5.)
    assert a.plan(q,True,geometry(),{},view.entrance_line(q),0.,sample(0))['allowed']
    far={'source':'current_tracked_far_edge','far_edge_px':q[[3,2]].tolist()}
    out=a.plan(None,False,far,{},None,.1,sample(1))
    assert out['allowed']==allowed
    assert not a.plan(None,False,far,{},None,.3,sample(3))['allowed']


def test_far_entrance_ignores_endpoint_beyond_calibration_lateral_extent():
    case=next(c for c in PI_CASES if c['frame_id']==3283)
    view=ParkingGroundView(case['ground']);q=np.array(case['quad'])
    initial=ParkingApproach(view,.335,5.)
    assert view.entrance_line(q) is None
    assert initial.region(q,None)=='far'
    assert initial.plan(q,True,geometry(),{},None,0.,sample(0))['allowed']
    # A wholly lateral entrance is still not a far approach target.
    shifted=q+[-300,0]
    assert initial.region(shifted,None)=='lateral_or_near_outside'


def test_short_visible_side_does_not_force_early_metric_position():
    case=next(c for c in PI_CASES if c['frame_id']==7945)
    view=ParkingGroundView(case['ground']);q=np.array(case['quad'])
    assert view.entrance_line(q) is not None
    assert view.segment(q[1],q[2]) is None
    assert ParkingBayPose(view,.625,1.).entrance(q,view.entrance_line(q)) is None
    assert ParkingApproach(view,.335,5.).region(q,view.entrance_line(q))=='partial_with_distance'


@pytest.fixture(scope='module',params=[FIRST,SECOND])
def recorded(request):return request.param,replay(request.param,staged=True)


def test_latest_recordings_keep_board_and_bay_without_repeated_restart(recorded):
    folder,rows=recorded
    active=[r for r in rows if r['intent']['phase']!='arming']
    assert len(active)==51
    assert all(r['sample']['parking']['direction_hint']['suggested_slot']=='left' for r in active)
    assert sum(bool(r['sample']['parking']['entry_candidates']) for r in active)>=48
    assert sum(bool(r['sample']['parking']['tracked_bay_boundaries']) for r in active)>=46
    assert all(r['intent']['phase']!='boundary_wait' for r in active)
    assert sum(r['intent']['esc_us']>=1570 for r in active)>=(40 if folder==FIRST else 19)
    # The original trajectory stopped here. Counterfactual forward commands
    # must not turn a stationary recording into claimed physical progress.
    assert rows[-1]['intent']['phase']=='fault'
    assert '未观察到移动' in rows[-1]['intent']['reason']
    assert all(not r['intent']['done'] and not r['intent']['whole_vehicle_inside_verified'] for r in rows)
    assert all(r['intent']['esc_us']<=1500 or r['intent']['esc_us']>=1570 for r in rows)
