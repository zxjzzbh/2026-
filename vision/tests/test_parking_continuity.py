"""Camera replay and loss/obstruction checks; never acquires vehicle hardware."""
from pathlib import Path
import json

import cv2
import numpy as np
import pytest

from carvision.parking_bay import detect_parking
from carvision.parking_far_tracker import ParkingFarTracker
from carvision.parking_entry import ParkingEntryController
from parking_replay_support import replay
from test_parking_entry import settings,ALIGNED
from test_parking_visual_fixes import frame

DATA=Path(__file__).parent/'fixtures/parking-continuity-20261108'
RUN='parking-entry-1794077501475811189'


def camera(fid=1505):
    folder=DATA/RUN
    row=next(r for r in map(json.loads,(folder/'frames.jsonl').read_text().splitlines()) if r['frame_id']==fid)
    with (folder/'frames.mjpg').open('rb') as stream:
        stream.seek(row['offset']);image=cv2.imdecode(np.frombuffer(stream.read(row['length']),np.uint8),1)
    config=json.loads((folder/'settings.json').read_text(encoding='utf8'))['parking']
    return image,config


@pytest.mark.parametrize('name',[RUN,'parking-entry-1794077522964177017'])
def test_recorded_intermittent_contours_keep_current_boundary_control(name):
    rows=replay(DATA/name)
    assert rows and not any(r['intent']['phase'] in ('fault','safety_brake') for r in rows)
    moving=[r for r in rows if r['intent']['esc_us']>1500]
    assert moving and all(r['intent']['esc_us']>=1570 for r in moving)
    if name==RUN:
        first=moving[0]['now_s']
        assert all(r['intent']['esc_us']>=1570 for r in rows if r['now_s']>=first)
        former_stop=next(r for r in rows if r['frame_id']==1508)
        assert former_stop['intent']['geometry_source']=='tracked_bay_edges'
        assert former_stop['intent']['white_boundary']['support_source']=='current_bay_envelope'
    else:
        assert moving[0]['frame_id']<=1712
    # Original frames end early; replay cannot claim a new trajectory or arrival.
    assert all(not r['intent']['done'] and not r['intent']['whole_vehicle_inside_verified'] for r in rows)


@pytest.mark.parametrize('mirror',[False,True])
def test_current_markings_support_tracking_when_the_contour_disappears(mirror):
    image,config=camera()
    if mirror:image=cv2.flip(image,1)
    tracker=ParkingFarTracker()
    obs=detect_parking(image,config)[0]
    assert obs['entry_candidates']
    tracker.update(image,obs,0,0.,config)
    for fid in range(1,6):
        current={'slots':[],'entry_candidates':[]}
        tracker.update(image,current,fid,fid*.1,config)
        records=current['tracked_bay_boundaries']
        assert len(records)==1 and min(records[0]['edge_support'])>=.72
        assert records[0]['frame_id']==fid and records[0]['captured_s']==fid*.1
        assert not records[0]['occupancy_verified']


@pytest.mark.parametrize('failure',['blank','gap','same_frame','blue_block','missing_side','explicit_blocked'])
def test_tracking_cannot_fill_in_missing_or_obstructed_boundaries(failure):
    image,config=camera();tracker=ParkingFarTracker();obs=detect_parking(image,config)[0]
    tracker.update(image,obs,0,0.,config)
    q=np.asarray(obs['entry_candidates'][0]['corners_px'],np.int32)
    changed=image.copy();current={'slots':[],'entry_candidates':[]}
    fid,t=1,.1
    if failure=='blank':changed[:]=0
    elif failure=='gap':t=.6
    elif failure=='same_frame':fid=0
    elif failure=='blue_block':
        center=tuple(np.rint(q.mean(axis=0)).astype(int));cv2.circle(changed,center,12,(230,70,15),-1)
    elif failure=='missing_side':
        cv2.line(changed,tuple(q[0]),tuple(q[3]),tuple(int(x) for x in image[225,230]),24)
    elif failure=='explicit_blocked':current['slots']=[{'id':'left','availability':'blocked'}]
    tracker.update(changed,current,fid,t,config)
    assert current['tracked_bay_boundaries']==[]


def test_unseeded_tracker_never_invents_a_bay():
    image,config=camera();tracker=ParkingFarTracker()
    for i in range(5):
        obs={};tracker.update(image,obs,i,i*.1,config)
        assert obs['tracked_bay_boundaries']==[]


def test_recent_bay_observations_survive_a_blink_but_expire():
    c=ParkingEntryController(settings(),'left')
    for t in (1.,1.2,1.4):
        c.last_far_at=t;c.geometry_source='visible_candidate'
        c.geometry={'availability':'clear','lateral_error':0.,'heading_error':0.}
        c._bay_evidence(t)
        c.geometry={};c._bay_evidence(t+.05)
    assert c._bay_stable() and c.bay_frames==3
    c._bay_evidence(2.)
    assert not c._bay_stable() and c.bay_frames==0


@pytest.mark.parametrize('field,value',[('frame_id',99),('captured_s',.5),('edge_support',[1.,1.,.2,1.]),('blue_inside_fraction',.1)])
def test_controller_rejects_stale_or_unsupported_boundary_tracking(field,value):
    c=ParkingEntryController(settings(),'left');c.last_frame=10;c.last_capture=1.
    record={'id':'left','source':'current_tracked_bay_edges','frame_id':10,'captured_s':1.,
            'corners_px':ALIGNED,'availability':'clear','tracked_points':20,'edge_support':[1.]*4,
            'ground_fraction':1.,'blue_inside_fraction':0.}
    record[field]=value
    assert c._polygon({'tracked_bay_boundaries':[record],'direction_hint':{'suggested_slot':'left'}},1.)[0] is None
