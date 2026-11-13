"""Reproduce the recorded 1.316 s stop and verify bounded recovery."""
from pathlib import Path
import copy
import json

import cv2
import numpy as np
import pytest

from carvision.parking_recovery import ParkingRecovery
from carvision.parking_bay import parking_road_context
from carvision.cone_road import ConeRoadContext
from carvision.parking_entry import ParkingEntryController
from carvision.parking_entry_run import ParkingEntryRun
from test_parking_entry import settings, ALIGNED
from test_parking_entry_run import prepared, command
from parking_replay_support import replay

DATA=Path(__file__).parent/'fixtures/parking-recovery-20261108'


def update(clock,t,*,boundary=True,moving=False,valid=True):
    return clock.update(boundary,{'valid':valid,'moving':moving},t,t,.5)


def test_recorded_first_stopped_frame_gets_its_own_confirmation_time():
    clock=ParkingRecovery(1.2);clock.start(0.,.45,True)
    for t in (.083,.166,.334,.416,.498,.579,.702,.743,.906,.987,1.110,1.152):
        resumed,cause,expired=update(clock,t,moving=True)
        assert not resumed and not expired and cause=='still_moving'
    resumed,cause,expired=update(clock,1.316)
    assert not resumed and not expired and clock.stop_frames==1
    assert cause=='stop_confirmation_pending'
    assert update(clock,1.416,boundary=False)[0] is False
    assert clock.stop_frames==2
    resumed,_,expired=update(clock,1.516)
    assert resumed and not expired and clock.stop_frames==3


@pytest.mark.parametrize('moving,valid,cause',[(True,True,'still_moving'),(False,False,'stop_feedback_invalid')])
def test_returning_boundary_does_not_authorize_motion_without_stop(moving,valid,cause):
    clock=ParkingRecovery(1.2);clock.start(0.,.45,True)
    for t in np.arange(.1,2.45,.1):
        resumed,why,expired=update(clock,float(t),moving=moving,valid=valid)
        assert not resumed and not expired and why==cause
    resumed,why,expired=update(clock,2.46,moving=moving,valid=valid)
    assert not resumed and expired and why==cause


def test_no_boundary_and_flickering_boundary_have_finite_deadlines():
    clock=ParkingRecovery(1.2);clock.start(0.,.45,True)
    assert update(clock,1.21,boundary=False)[2]
    clock.start(0.,.45,True)
    for t in np.arange(.1,4.,.1):
        resumed,_,expired=update(clock,float(t),boundary=round(t*10)%4==0,moving=True)
        assert not resumed
        if expired:break
    assert expired and t<=3.7


def test_new_motion_or_invalid_feedback_resets_stop_evidence():
    clock=ParkingRecovery(1.2);clock.start(0.,.45,True)
    update(clock,.5);update(clock,.6)
    assert clock.stop_frames==2
    update(clock,.7,moving=True);assert clock.stop_frames==0
    update(clock,.8);update(clock,.9,valid=False);assert clock.stop_frames==0


def test_late_stop_confirmation_cannot_override_recovery_deadline():
    clock=ParkingRecovery(1.2);clock.start(0.,.45,True)
    update(clock,.1,moving=True)
    update(clock,2.3);update(clock,2.4)
    resumed,_,expired=update(clock,2.5)
    assert expired and not resumed and clock.stop_frames==3


def test_changing_target_preserves_an_active_recovery_clock():
    controller=ParkingEntryController(settings(),'left')
    controller.boundary_wait_since=1.
    controller.boundary_brake_until=1.35
    controller.recovery.start(1.,1.45,True)
    deadline=controller.recovery.hard_deadline
    assert controller._retarget_if_blocked({'slots':[{'id':'left','availability':'blocked'}]},1.1) is None
    assert controller.side=='right'
    assert controller.boundary_wait_since==1. and controller.boundary_brake_until==1.35
    assert controller.recovery.started==1. and controller.recovery.hard_deadline==deadline


def test_current_far_line_reacquires_a_guide_after_the_full_quad_is_old():
    c=ParkingEntryController(settings(),'left')
    c.has_full_quad=True;c.last_poly=np.asarray(ALIGNED,dtype=np.float32);c.last_poly_at=1.
    c.last_poly_near_observed=True
    c.last_far_edge=np.asarray([[160,180],[320,180]],dtype=np.float32);c.last_far_at=2.
    guide={'corners_px':[[110,288],[370,288],[320,180],[160,180]],
           'near_boundary_observed':False,'far_boundary_observed':True,'availability':'clear'}
    q,blocked,source,near=c._polygon({'entry_guides':[guide]},2.)
    assert q is not None and not blocked and source=='tracked_far_guide' and not near
    changed=copy.deepcopy(guide)
    changed['corners_px']=[[x+100,y] for x,y in guide['corners_px']]
    assert c._polygon({'entry_guides':[changed]},2.)[0] is None
    assert c._polygon({'entry_guides':[guide]},2.51)[0] is None


def test_yellow_bay_sides_are_not_reported_as_white_road_edges():
    stored=json.loads((DATA/'settings.json').read_text(encoding='utf8'))
    index=next(r for r in map(json.loads,(DATA/'frames.jsonl').read_text().splitlines()) if r['frame_id']==2403)
    with (DATA/'frames.mjpg').open('rb') as stream:
        stream.seek(index['offset']);image=cv2.imdecode(np.frombuffer(stream.read(index['length']),np.uint8),1)
    road=ConeRoadContext(stored['cone'],'secondary',thorough_boundary_search=True)
    result=parking_road_context(road,image,stored['parking'])
    assert not result['white_lines_detected']
    assert result['boundary_diagnostics']['parking_color_separation']


def test_recorded_camera_sequence_recovers_bay_geometry_without_old_fault():
    rows=replay(DATA)
    assert rows and all(r['intent']['phase']!='fault' for r in rows)
    assert any(r['intent']['geometry_source']=='tracked_bay_edges'
               and not r['intent']['near_boundary_observed'] for r in rows)
    last=rows[-1]['intent']
    assert last['phase']=='boundary_wait' and last['esc_us']==1500
    assert last['white_boundary']['support_source']=='current_bay_envelope'
    assert last['boundary_wait_cause']=='stop_confirmation_pending'
    assert last['recovery_stop_frames']==1
    # Current edges now recover earlier, so the same stop-confirmation clock
    # has run longer by the end of the unchanged recording.
    assert 0<last['recovery_timing']['settle_remaining_s']<=2.
    assert not last['done'] and not last['whole_vehicle_inside_verified']
    assert all(r['intent']['esc_us']<=1500 or r['intent']['esc_us']>=1570 for r in rows)


def test_visual_brake_setting_is_frozen_before_hardware_and_cannot_be_payload_extended():
    run,actuator=prepared()
    assert run.s['brake']['brake_pulse_s']==.35
    run.command('parking_trial_keepalive',command(run,0,brake_pulse_s=9),.2)
    assert run.s['brake']['brake_pulse_s']==.35
    run.cancel('test complete')
