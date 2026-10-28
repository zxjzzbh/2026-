"""Actual tracking/navigation/control chain with synthetic observations only.

The transform/white edges below describe a unit-test world. They do not replace
vehicle calibration or reconstruct an accident frame. No hardware is opened.
"""
import copy
import threading
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_low_speed import DualLowSpeedNavigator,LowSpeedNavigator
from carvision.cone_navigation import actuator_settings
from carvision.cone_observer import ConeCamera
from carvision.cone_vision import ImageConeTracker
from carvision.shared_executor import BorrowedActuator
from test_cone_identity_recovery import detection,first_clip,second_clip
from test_cone_turn_controller import settings
from test_shared_executor import CLIENT,Worker,rig


IMAGE=np.zeros((360,480,3),np.uint8)


def synthetic_settings():
    value=settings(gain=1.5)
    # Nominal reference X=0 at pixel140. All calibration flags stay false.
    value['ground']['homography']=[[.005,0,-.7],[0,-.01,3.6],[0,0,1]]
    value['scene']['surface']='indoor_white_line_track'
    assert not value['ground']['verified'] and not value['motion']['verified']
    return value


def context(high=300.):
    ys=np.linspace(180.,high,16)
    return {'valid':True,'turn_aware':True,'ground_supported':True,
            'left_inner':[[52.,float(y)] for y in ys],
            'right_inner':[[52.+880/3,float(y)] for y in ys],
            'boundary_candidates':[]}


def navigation(navigator,tracker,detections,fid,t,*,high=300.):
    tracks=tracker.update(copy.deepcopy(detections),fid,t)
    row={'captured_s':t,'frame_id':fid,'tracks':tracks,'road_context':context(high)}
    nav=navigator.update(IMAGE,row,t)
    nav['source_camera']='secondary'
    return tracks,nav


def test_actual_recovered_identity_freezes_earned_candidate_and_resets_exit_before_worker_output(rig):
    now,clock,backend,manual,owner,_=rig
    value=synthetic_settings()
    tracker=ImageConeTracker('secondary',value,near_field_recovery=True)
    navigator=LowSpeedNavigator(value)
    controller=ContinuousConeController(value,0.,initialized=True,handoff_after_two=True)
    handle=BorrowedActuator(Worker(owner),actuator_settings(value),CLIENT,clock=clock)
    for fid,y in enumerate((240.,246.,252.,260.,270.,285.,291.,297.)):
        t=fid/10;tracks,nav=navigation(navigator,tracker,[detection(228.,y)],fid,t)
        now[0]=t;output=controller.step(nav,t);handle.send(output)
        assert owner.active and not owner.guard.fault and controller.count==0
    assert controller.target=='secondary-1' and controller.side=='left'
    assert controller.near_seen and controller.pass_gate_hits==3
    assert nav['cones'][0]['identity_revision']==0
    earned=copy.deepcopy(controller.pass_evidence)
    assert earned['visual_pass_candidate'] and earned['exit_frames']==0
    assert not earned['visual_estimate_ready']

    tracks,nav=navigation(navigator,tracker,[first_clip()],8,.8)
    now[0]=.8;output=controller.step(nav,.8);handle.send(output)
    assert not any(track['id']=='secondary-1' and track['visible'] for track in tracks)
    assert controller.review_pending and controller.count==0
    # Earlier same-row ground observations earned a candidate. An old elapsed
    # tail timer is not the later three-frame current-pixel exit sequence.
    controller.tail_powered=1.05

    tracks,nav=navigation(navigator,tracker,[second_clip()],9,.9)
    recovered=next(track for track in tracks if track['visible'])
    assert recovered['id']=='secondary-1' and recovered['identity_recovered']
    assert recovered['identity_revision']==1 and not recovered['ground_contact_usable']
    pixel=next(item for item in nav['visual_cones'] if item['id']=='secondary-1')
    assert pixel['identity_revision']==1 and pixel['identity_recovered']
    assert pixel['captured_s']==.9 and pixel['frame_id']==9 and pixel['source_camera']=='secondary'
    now[0]=.9;output=controller.step(nav,.9);handle.send(output)
    assert output['target_id']=='secondary-1' and output['target_space']=='image'
    assert output['target_identity_revision']==1
    assert earned['gate_hits']==3 and controller.pass_gate_hits==0 and controller.near_seen
    assert controller.tail_powered==0
    assert output['passage_evidence']['recovery_candidate_frozen']
    assert output['passage_evidence']['visual_pass_candidate']
    assert output['passage_evidence']['anchor_valid']
    assert output['passage_evidence']['anchor_hits']==earned['anchor_hits']
    assert output['passage_evidence']['anchor_frame_id']==earned['anchor_frame_id']
    assert output['passage_evidence']['exit_frames']==0
    assert output['passage_evidence']['observation_deadline_s']==earned['observation_deadline_s']
    assert not output['passage_evidence']['visual_estimate_ready']
    assert controller.count==controller.auto_count==0 and not controller.passed_ids
    assert not output['lane_return_confirmed'] and output['phase']!='complete'
    assert owner.guard.esc==output['esc_us'] and owner.active

    # A subsequent actual detector row may restore ground use. Its persistent
    # revision must also reach current lane entries, without repeating recovery.
    actual=detection(263.,340.,box=[236,290,300,355])
    tracks,nav=navigation(navigator,tracker,[actual],10,1.,high=350.)
    entry=next(item for item in nav['cones'] if item['id']=='secondary-1')
    assert not entry.get('near_field_continuation')
    assert entry['identity_revision']==1 and not entry['identity_recovered']
    now[0]=1.;output=controller.step(nav,1.);handle.send(output)
    # Current ground can revalidate the anchor, but neither identity recovery
    # nor the old pre-gap gates supply a new current passage sequence.
    assert controller.pass_gate_hits==0 and controller.near_seen
    assert controller.pass_evidence['anchor_valid']
    assert controller.pass_evidence['observation_deadline_s']==earned['observation_deadline_s']
    assert controller.pass_evidence['exit_frames']==0 and not controller.pass_evidence['visual_estimate_ready']
    assert controller.count==0 and not controller.passed_ids
    assert not owner.guard.fault
    handle.close();now[0]=1.351;owner.tick();assert owner.guard is None
    assert all(record[3]<=.2 for record in backend.writes if record[0]=='motion')


@pytest.mark.parametrize('enabled',[False,True])
def test_camera_explicitly_enables_recovery_and_frame_restart_keeps_that_mode(enabled):
    value=synthetic_settings();now=[1.]
    _,jpeg=cv2.imencode('.jpg',IMAGE)
    states={camera:SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpeg.tobytes(),
             raw_frame_id=10,raw_received_ms=1000.,state='ok') for camera in ('primary','secondary')}
    camera=ConeCamera(states,value,lambda:now[0],turn_aware=enabled)
    camera.navigator=DualLowSpeedNavigator(value)
    assert all(tracker.near_field_recovery is enabled for tracker in camera.trackers.values())
    result=camera.read()
    assert result is not None and camera.ids=={'primary':10,'secondary':10}
    previous=dict(camera.trackers)
    now[0]=1.1
    states['primary'].raw_frame_id=1;states['primary'].raw_received_ms=1100.
    result=camera.read()
    assert result is not None and camera.ids['primary']==1
    assert camera.trackers['primary'] is not previous['primary']
    assert camera.trackers['primary'].last_id==1
    assert camera.trackers['primary'].near_field_recovery is enabled
    assert camera.trackers['secondary'] is previous['secondary']
    assert camera.trackers['secondary'].near_field_recovery is enabled
    if enabled:assert camera.trackers['primary'].latest_diagnostics['enabled'] is True
    else:assert 'tracking_diagnostics' not in camera.rows['primary']


def test_camera_mode_switch_updates_existing_trackers_without_fabricating_new_frames():
    value=synthetic_settings();camera=ConeCamera({},value,turn_aware=False)
    trackers=dict(camera.trackers)
    for tracker in trackers.values():assert tracker.near_field_recovery is False
    camera.set_turn_aware(True)
    assert camera.ids=={} and camera.rows=={}
    for name,tracker in trackers.items():
        assert camera.trackers[name] is tracker and tracker.near_field_recovery is True
    camera.set_turn_aware(False)
    assert camera.ids=={} and camera.rows=={}
    for name,tracker in trackers.items():
        assert camera.trackers[name] is tracker and tracker.near_field_recovery is False
        assert tracker.latest_diagnostics=={'enabled':False}
