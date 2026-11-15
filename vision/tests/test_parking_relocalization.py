"""Actual pixels/optical-flow recovery; fixtures are not field calibration."""
import copy
import math

import cv2
import numpy as np
import pytest

from carvision.parking_bay import detect_parking
from carvision.parking_feedback import ParkingWorldFeed
from carvision.parking_localization import road_pose, transform_slots
from carvision.parking_mission import layout
from carvision.parking_task import ParkingTask
from carvision.parking_control import curvature_for_pwm, MetricParkingController
from carvision.parking_mission import contained
from test_cone_navigation import measured, image, context
from test_parking_bay import calibrated, scene, slots, sample
from test_parking_path import measured_settings, measured_cones


@pytest.mark.parametrize('degrees', [0,2,4,6,8,10,-4,-10,15])
def test_heading_compensated_detection_keeps_real_vehicle_coordinates(degrees):
    settings=calibrated();homography=np.asarray(settings['ground']['homography'])
    angle=math.radians(degrees)
    rotation=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
    shifted=cv2.warpPerspective(scene('left',True),np.linalg.inv(homography)@rotation@homography,
                               (480,360),borderValue=(45,45,170))
    view,_=detect_parking(shifted,settings)
    assert view['metric_geometry'] and [s['availability'] for s in view['slots']]==['blocked','clear']
    pose=road_pose(view['slots'],settings)
    assert pose['yaw_rad']==pytest.approx(angle,abs=.015)
    for slot in view['slots']:
        pixels=np.c_[slot['corners_px'],np.ones(4)]@homography.T
        assert np.asarray(slot['polygon_m'])==pytest.approx(pixels[:,:2]/pixels[:,2:],abs=1e-6)
    _,boxes=layout(transform_slots(view['slots'],pose),settings)
    assert boxes['right'][3]-boxes['right'][1]==pytest.approx(1.,abs=.03)


def test_horizontal_only_and_unverified_mapping_do_not_create_metric_layout():
    settings=calibrated();im=scene('left',True)
    im[:,115:122]=(45,45,170);im[:,360:367]=(45,45,170);im[:170,237:244]=(45,45,170)
    assert not detect_parking(im,settings)[0]['metric_geometry']
    settings['ground']['verified']=False
    assert not detect_parking(scene(),settings)[0]['metric_geometry']


@pytest.fixture
def recovery_rig(measured):
    cones,parking=measured_cones(),measured_settings()
    parking['ground']=copy.deepcopy(measured['ground']);parking['maximum_pose_error_m']=.015
    cones['ground']=copy.deepcopy(measured['ground']);cones['driving']['max_speed_mps']=.5
    bundle={'parking':parking,'cones':cones};im=image();road=context(measured,im)
    view={'metric_geometry':True,'layout_found':True,'slots':slots()}
    feed=ParkingWorldFeed(bundle)
    def step(now,frame,*,motion=True,road_valid=True,current=None,since=1.11):
        row={'captured_s':now,'frame_id':frame,'tracks':[],'road_context':{**road,'valid':road_valid}}
        proof={'valid':True,'moving':False,'captured_s':now,'continuous_still_since_s':since} if motion is True else motion
        return feed.update(im,row,copy.deepcopy(view if current is None else current),now,motion=proof)
    assert step(0.,0) is None
    assert step(.1,1) is not None
    return feed,step,bundle,view


@pytest.mark.parametrize('loss',['gap','white_line'])
def test_real_failed_optical_flow_relocalizes_same_world_after_independent_rest(recovery_rig,loss):
    feed,step,_,_=recovery_rig
    original=copy.deepcopy(feed.anchor)
    failed_at=1.1 if loss=='gap' else .2
    assert step(failed_at,2,road_valid=loss!='white_line') is None
    old=feed.odo
    assert old.failed
    for fid,now in enumerate((1.8,1.9,2.),3):
        world=step(now,fid)
        if fid<5:assert world is None
    assert world['relocalized'] and world['localization_epoch']==1 and world['speed_mps']==0.
    assert not feed.odo.failed and feed.odo is not old and feed.anchor==original
    assert world['pose']['x_m']==pytest.approx(0.,abs=1e-9)
    assert world['pose']['yaw_rad']==pytest.approx(0.,abs=1e-9)
    following=step(2.1,6)
    assert following and not following['relocalized'] and following['localization_epoch']==1


@pytest.mark.parametrize('invalid',['moving','unknown','stale','future','missing_interval','wrong_bay',
    'changed_occupancy','shape','partial','unverified','repeated_time','repeated_frame'])
def test_relocalization_rejects_invalid_or_contradictory_evidence(recovery_rig,invalid):
    feed,step,_,view=recovery_rig
    assert step(1.1,2) is None
    for i,now in enumerate((1.8,1.9,2.),3):
        proof={'valid':True,'moving':False,'captured_s':now,'continuous_still_since_s':1.11}
        current=copy.deepcopy(view)
        if invalid=='moving':proof['moving']=True
        elif invalid=='unknown':proof['valid']=False
        elif invalid=='stale':proof['captured_s']=now-.3
        elif invalid=='future':proof['captured_s']=now+.01
        elif invalid=='missing_interval':proof.pop('continuous_still_since_s')
        elif invalid=='wrong_bay':
            for slot in current['slots']:slot['polygon_m']=[[x,y+1.] for x,y in slot['polygon_m']]
        elif invalid=='shape':current['slots'][0]['polygon_m'][0][0]-=.2
        elif invalid=='changed_occupancy':
            for slot in current['slots']:slot['availability']='clear' if slot['availability']=='blocked' else 'blocked'
        elif invalid=='partial':current['metric_geometry']=False
        elif invalid=='unverified':current['slots'][0]['occupancy_verified']=False
        elif invalid=='repeated_time':now=1.8
        elif invalid=='repeated_frame':i=3
        assert step(now,i,motion=proof,current=current) is None
    assert feed.odo.failed and feed.epoch==0


def test_seen_motion_retires_old_stillness_interval(recovery_rig):
    feed,step,_,_=recovery_rig
    step(1.1,2)
    step(1.8,3,motion={'valid':True,'moving':True,'captured_s':1.8})
    for fid,t in enumerate((2.3,2.4,2.5),4):assert step(t,fid,since=1.11) is None
    for fid,t in enumerate((2.6,2.7,2.8),7):world=step(t,fid,since=1.81)
    assert world and world['relocalized']


def test_new_localization_epoch_reconfirms_and_replans_before_forward():
    task=ParkingTask(measured_settings(),measured_cones())
    for i in range(7):
        world=sample(i,i*.1);world['localization_epoch']=0
        out=task.step(world,i*.1)
    assert out['esc_us']>=1570
    old_path=task.path
    for i,t in enumerate((.7,1.,1.3),7):
        world=sample(i,t);world['localization_epoch']=1
        out=task.step(world,t)
        if i<9:assert out['esc_us']==1500 and task.path is old_path
    assert out['esc_us']>=1570 and task.path is not old_path
    assert task.selected=='right'


def test_failed_feed_recovers_through_actual_debug_controller(recovery_rig):
    feed,step,bundle,_=recovery_rig
    controller=MetricParkingController(bundle['parking'],bundle['cones'])
    controller.started=-3.
    for frame in range(2,10):
        now=round(frame*.1,8)
        world=step(now,frame)
        out=controller.step({'parking_world':world},now)
    assert out['esc_us']>=1570 and controller.mission.selected=='right'
    old_path=controller.task.path
    assert step(1.8,10) is None  # Real LK gap, not a mock failed flag.
    out=controller.step({'parking_world':None},1.8)
    assert out['action']=='brake' and out['phase']=='recovering'
    localized_at=None
    for frame,now in enumerate((2.2,2.5,2.6,2.7,2.8,2.9,3.,3.1,3.2,3.3,3.4),11):
        world=step(now,frame,since=1.81)
        if world and world['relocalized']:localized_at=now
        out=controller.step({'parking_world':world},now)
        assert out['phase']!='fault'
        if localized_at is None or now-localized_at<bundle['parking']['selection_confirm_s']-1e-9:
            assert out['esc_us']<=1500
    assert feed.epoch==1 and out['esc_us']>=1570
    assert controller.task.path is not old_path and controller.mission.selected=='right'


def test_debug_localization_wait_uses_one_finite_brake_and_bounded_neutral():
    controller=MetricParkingController(measured_settings(),measured_cones())
    controller.started=-3.
    for i in range(7):controller.step({'parking_world':sample(i,i*.1)},i*.1)
    assert controller.last_forward
    for now in (.7,.8,1.):
        out=controller.step({'parking_world':None},now)
        assert out['action']=='brake' and out['phase']=='recovering'
    for now in (1.1,10.,20.6):
        out=controller.step({'parking_world':None},now)
        assert out['action']=='neutral' and out['phase']=='recovering'
    out=controller.step({'parking_world':None},20.7)
    assert out['action']=='neutral' and out['phase']=='fault'
    assert controller.step({'parking_world':sample(20,20.8)},20.8)['phase']=='fault'


@pytest.mark.parametrize('blocked',['left','right'])
def test_actual_task_parks_after_stopped_relocalization_using_final_pwm(blocked):
    settings=measured_settings();task=ParkingTask(settings,measured_cones())
    pose={'x_m':0.,'y_m':0.,'yaw_rad':.08};speed=0.;epoch=0
    pause_start=None;now=0.
    for frame in range(500):
        now=round(now+.1,8)
        if epoch==0 and now>=3.:
            if pause_start is None:pause_start=now
            speed=max(0.,speed-.04)
            pose['x_m']+=speed*math.sin(pose['yaw_rad'])*.1
            pose['y_m']+=speed*math.cos(pose['yaw_rad'])*.1
            if now-pause_start<.9:continue
            assert speed==0.
            epoch=1  # Feed's independent stopped registration was proved above.
        world=sample(frame,now,dict(pose),blocked,speed)
        world['localization_epoch']=epoch
        output=task.step(world,now)
        assert output['phase']!='fault',output
        if output['payment_ready']:break
        target=output.get('command_speed_mps',0.)
        speed+=max(-.04,min(.03,target-speed))
        curvature=curvature_for_pwm(output['steering_us'],settings)
        assert output['command_curvature_1pm']==pytest.approx(curvature)
        assert output['esc_us']<=1500 or output['esc_us']>=1570
        middle=pose['yaw_rad']+.05*speed*curvature
        pose['x_m']+=.1*speed*math.sin(middle)
        pose['y_m']+=.1*speed*math.cos(middle)
        pose['yaw_rad']+=.1*speed*curvature
    assert epoch==1 and output['payment_ready'] and abs(speed)<=settings['stopped_speed_mps']
    _,boxes=layout(slots(blocked),settings)
    assert contained(pose,boxes[task.selected],settings,.03)
