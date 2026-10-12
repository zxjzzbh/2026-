"""Measured synthetic coordinate worlds exercise integration; no vehicle I/O."""
import copy
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.cone_planning import load_settings,path_clear,ConeRoutePlanner
from carvision.cone_navigation import (driving_settings,execution_readiness,ConeNavigator,ConeOdometry,
    ConeDrivingController,steering_for_path,metric_corridor,import_ground_calibration)
from carvision.cone_execution import ConeRun
from carvision.cone_observer import ConeCamera,ConeObserverDrive
from carvision.cone_road import ConeRoadContext

ROOT=Path(__file__).parents[1];CONFIG=ROOT/'configs/cone-trial.json'


@pytest.fixture
def measured():
    s=load_settings(CONFIG)
    s['geometry'].update(measured=True,vehicle_length_m=.41,rear_axle_to_front_m=.3,rear_overhang_m=.11,clearance_m=.02)
    scale=1.23/308
    s['ground'].update(verified=True,homography=[[scale,0,-240*scale],[0,-.02,7.2],[0,0,1]],
        valid_polygon_px=[[0,0],[479,0],[479,359],[0,359]],max_error_m=.01)
    s['motion'].update(verified=True,pose_feedback_verified=True,minimum_turn_radius_m=.5,
        maximum_tracking_error_m=.03,braking_distance_m=.1)
    s['driving']={**driving_settings(s),'steering_radius_1570_cm':-50.,'steering_radius_1650_cm':50.,
        'visual_error_bound_m':.01,'visual_verified_distance_m':10.,'fixed_camera_confirmed':True,
        'steering_verified':True,'odometry_verified':True,'max_speed_mps':.5}
    assert execution_readiness(s)['ready']
    return s


def image():
    rng=np.random.default_rng(4)
    noise=rng.integers(-20,21,(360,480,1))
    im=np.clip(np.array([30,35,150])[None,None,:]+noise,0,255).astype(np.uint8)
    cv2.line(im,(80,0),(80,359),(255,255,255),10);cv2.line(im,(400,0),(400,359),(255,255,255),10)
    return im


def context(s,im):return ConeRoadContext(s).inspect(im,np.zeros(im.shape[:2],np.uint8))


def good_motion(y=0,speed=0):
    return {'valid':True,'speed_mps':speed,'travel_m':y,'pose':{'valid':True,'x_m':0.,'y_m':y,'yaw_rad':0.,'error_bound_m':.01}}


def test_defaults_cannot_drive_even_with_recognized_cone():
    s=load_settings(CONFIG);nav=ConeNavigator(s)
    result=nav.update(None,{},1.)
    assert result['phase']=='needs_calibration' and result['action']=='stop'
    assert not execution_readiness(s)['ready']


def test_metric_white_inner_edges_agree_with_lane_width(measured):
    im=image();road=context(measured,im)
    bounds,heading,coverage=metric_corridor(road,measured)
    assert bounds==pytest.approx([-.615,.615],abs=.002)
    assert abs(heading)<.001 and coverage>3
    measured['geometry']['lane_width_m']=2
    with pytest.raises(ValueError,match='宽度'):metric_corridor(road,measured)


def test_real_optical_flow_measures_forward_displacement_and_latches_gap(measured):
    im=image();odom=ConeOdometry(measured);road=context(measured,im)
    assert not odom.update(im,1,1.,[],road)['valid']
    shifted=cv2.warpAffine(im,np.float32([[1,0,0],[0,1,1]]),(480,360),borderMode=cv2.BORDER_REFLECT)
    result=odom.update(shifted,2,1.1,[],context(measured,shifted))
    assert result['valid'] and result['speed_mps']==pytest.approx(.2,abs=.025)
    assert result['pose']['y_m']==pytest.approx(.02,abs=.003)
    assert not odom.update(shifted,3,1.5,[],road)['valid']
    assert odom.update(shifted,4,1.6,[],road)['reason']=='pose_lost_reprepare_required'


def test_flow_calibration_preview_does_not_enable_driving(measured,monkeypatch):
    measured['driving']['odometry_verified']=False;measured['motion']['pose_feedback_verified']=False
    nav=ConeNavigator(measured);im=image()
    monkeypatch.setattr(nav.odometry,'update',lambda *args:good_motion(.1,.1))
    result=nav.update(im,{'frame_id':1,'captured_s':1.,'road_context':context(measured,im),'tracks':[]},1.)
    assert result['phase']=='calibration_only' and result['action']=='stop'
    assert result['motion']['pose']['y_m']==.1 and not execution_readiness(measured)['ready']


@pytest.mark.parametrize('two_cones',[False,True])
def test_actual_camera_detection_flows_into_plan_and_steering(measured,monkeypatch,two_cones):
    monkeypatch.setattr(ConeOdometry,'update',lambda *args:good_motion())
    if two_cones:measured['ground']['homography'][1]=[0,-.08,28.8]  # Synthetic longer visible corridor.
    im=image();cv2.fillPoly(im,[np.array([[235,229],[245,229],[255,270],[225,270]])],(255,0,0))
    if two_cones:cv2.fillPoly(im,[np.array([[235,183],[245,183],[255,211],[225,211]])],(255,0,0))
    _,jpg=cv2.imencode('.jpg',im)
    state=SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),raw_frame_id=0,raw_received_ms=1000.,state='ok')
    timer=[1.];reader=ConeCamera({'secondary':state},measured,lambda:timer[0])
    for i in range(3):
        timer[0]=1+i*.1;state.raw_frame_id=i;state.raw_received_ms=timer[0]*1000;result=reader.read()
    decision=reader.navigation
    assert any(t['confirmed'] for t in result['cameras']['secondary']['tracks'])
    if not two_cones:
        assert decision['action']=='stop' and decision['phase']=='waiting_for_two_cones'
        return
    assert decision['action']=='follow_candidate_path' and decision['phase']=='pass_first_cone'
    assert len({c['side'] for c in decision['slalom']['order']})==2
    assert 1570<=steering_for_path(decision['path'],measured)<=1650
    assert min(p['x_m'] for p in decision['path'])<0<max(p['x_m'] for p in decision['path'])
    assert not decision['hardware_output']


def test_rotated_corridor_checks_all_vehicle_corners(measured):
    planner=ConeRoutePlanner(measured)
    result=planner.plan([],lane_bounds=[-.615,.615],lane_heading_rad=.12,coverage_forward_m=4)
    assert result['action']=='follow_candidate_path'
    assert path_clear(result['path'],[],measured,[-.615,.615],.12)
    assert result['path'][-1]['yaw_rad']==pytest.approx(.12)


def nav(now,phase='follow_lane',speed=0):
    return {'action':'follow_candidate_path','phase':phase,'captured_s':now,'motion':good_motion(0,speed),
            'path':[{'x_m':0.,'y_m':.4,'yaw_rad':0.,'curvature':0.}]}


def test_controller_neutral_start_route_follow_and_braking_stop(measured):
    ctl=ConeDrivingController(measured,0.)
    assert ctl.step(nav(1.),1.)['action']=='neutral'
    assert ctl.step(nav(3.1),3.1)['action']=='search'
    assert ctl.step(nav(3.2,'complete'),3.2)['action']=='brake'
    assert ctl.step(nav(3.34,'complete'),3.34)['action']=='neutral'
    assert ctl.step(nav(3.45,'complete'),3.45)['action']=='brake'
    assert ctl.step(nav(3.7,'complete'),3.7)['phase']=='complete'


def test_new_unconfirmed_cone_brakes_then_resumes_after_confirmation_and_release(measured):
    ctl=ConeDrivingController(measured,0.)
    uncertain={**nav(3.1,phase='waiting_for_feedback'),'action':'stop'}
    assert ctl.step(uncertain,3.1)['action']=='brake'
    assert ctl.step(nav(3.24),3.24)['action']=='neutral'
    assert ctl.step(nav(3.36),3.36)['action']=='search'


def test_unresolved_obstacle_timeout_does_not_search_blindly(measured):
    ctl=ConeDrivingController(measured,0.)
    def blocked(t):return {**nav(t,phase='waiting_for_clear_route'),'action':'stop'}
    assert ctl.step(blocked(3.1),3.1)['action']=='brake'
    assert ctl.step(blocked(4.1),4.1)['action']=='neutral'
    with pytest.raises(ValueError):ctl.step(blocked(6.2),6.2)


@pytest.mark.parametrize('case',['stale','lost_pose','no_route','speed','arming_motion'])
def test_controller_rejects_unusable_or_excessive_motion(measured,case):
    ctl=ConeDrivingController(measured,0.);now=3.1;sample=nav(now)
    if case=='stale':sample['captured_s']=1.
    if case=='lost_pose':sample['motion']['valid']=False
    if case=='no_route':sample['action']='stop'
    if case=='speed':sample['motion']['speed_mps']=1
    if case=='arming_motion':now=1.;sample=nav(now,speed=.1)
    with pytest.raises(ValueError):ctl.step(sample,now)


class Actuator:
    run=False
    def __init__(self):self.commands=[];self.closed=False
    def send(self,intent):self.commands.append(intent);return {'hardware_output':False}
    def close(self):self.closed=True


def base():
    return {'mode':'disabled','boot_test_phase':'esc_off','setup_token':'boot','hardware_output':False,
            'worker_started':False,'pending_hardware_start':False}


def preparation():return {'setup_token':'boot','client':'page','esc_off':True,'fb_mode_confirmed':True}


def command(run,n,**kw):return {'client':'page','run_token':run.token,'trial_sequence':n,**kw}


def test_explicit_preparation_and_start_are_both_required(measured):
    opened=[]
    def factory(s):a=Actuator();opened.append(a);return a
    run=ConeRun(factory)
    assert not opened and not run.active
    run.prepare(measured,base(),preparation(),0.)
    assert not opened
    run.tick(nav(.1),.1)
    assert len(opened)==1 and opened[0].commands[-1]['action']=='neutral'
    run.command('cone_start',command(run,0,fb_mode_confirmed=True,esc_on_static=True,field_ready=True),.2)
    for i in range(1,36):
        now=.2+i*.1
        run.command('cone_keepalive',command(run,i),now);run.tick(nav(now),now)
    assert any(c['action']=='search' for c in opened[0].commands)
    run.cancel('operator stopped')
    assert opened[0].closed and not run.active


def test_no_steering_or_motor_open_when_calibration_or_control_ownership_missing(measured):
    calls=[];run=ConeRun(lambda s:calls.append(s))
    with pytest.raises(ValueError):run.prepare(load_settings(CONFIG),base(),preparation(),0.)
    with pytest.raises(ValueError):run.prepare(measured,{**base(),'traffic_trial':{'active':True}},preparation(),0.)
    assert not calls


@pytest.mark.parametrize('failure',['lease','camera','stop_during_startup'])
def test_failsafe_closes_the_output_session(measured,failure):
    actuator=Actuator();run=ConeRun(lambda s:actuator)
    run.prepare(measured,base(),preparation(),0.)
    if failure=='stop_during_startup':
        def factory(s):run.cancel('cancelled during startup');return actuator
        run.factory=factory;run.tick(nav(.1),.1)
    else:
        run.tick(nav(.1),.1)
        run.tick(None,.2 if failure=='camera' else 1.)
    assert not run.active and actuator.closed


def test_foreign_repeated_or_expired_page_command_cannot_drive(measured):
    run=ConeRun(lambda s:Actuator());run.prepare(measured,base(),preparation(),0.);run.tick(nav(.1),.1)
    with pytest.raises(ValueError):run.command('cone_keepalive',command(run,0,client='other'),.2)
    run.command('cone_keepalive',command(run,0),.2)
    with pytest.raises(ValueError):run.command('cone_keepalive',command(run,0),.25)
    with pytest.raises(ValueError):run.command('cone_keepalive',command(run,1),1.)
    assert not run.active


def test_calibration_import_requires_measured_independent_checks():
    data={'camera':'secondary','image_size':[480,360],'measured_fixed_camera':True,'accepted_error_m':.01,
        'valid_polygon_px':[[0,0],[479,0],[479,359],[0,359]],
        'points':[{'pixel':p,'ground_m':[p[0]/100,p[1]/100]} for p in [[0,0],[400,0],[400,300],[0,300]]],
        'check_points':[{'pixel':p,'ground_m':[p[0]/100,p[1]/100]} for p in [[100,100],[200,200]]]}
    result=import_ground_calibration(load_settings(CONFIG),data)
    assert result['ground']['verified'] and not execution_readiness(result)['ready']
    bad=copy.deepcopy(data);bad['check_points'][0]['ground_m'][0]+=1
    with pytest.raises(ValueError):import_ground_calibration(load_settings(CONFIG),bad)
    with pytest.raises(ValueError):import_ground_calibration(load_settings(CONFIG),{'measured_fixed_camera':True,'camera':'secondary','image_size':[480,360]})


def test_wrapper_preserves_manual_commands_and_blocks_competing_control(tmp_path,measured):
    class Driver:
        def __init__(self):self.commands=[]
        def status(self):return base()
        def request(self,a,p):self.commands.append(a)
        def close(self):pass
    driver=Driver();w=ConeObserverDrive(driver,{},CONFIG,tmp_path,clock=lambda:0.,actuator_factory=lambda s:Actuator())
    w.request('command',{});assert driver.commands==['command']
    w.settings=measured;w.run.prepare(measured,base(),preparation(),0.)
    assert not w.status()['traffic_trial']['can_prepare']
    with pytest.raises(ValueError):w.request('traffic_prepare',{})
    with pytest.raises(ValueError):w.request('command',{})
    w.request('emergency',{})
    assert not w.run.active and driver.commands==['command']
