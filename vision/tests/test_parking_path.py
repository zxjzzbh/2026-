"""Synthetic geometry/vehicle tests. None of these records are field calibration."""
import copy
import math
import os
import shutil
import subprocess

import numpy as np
import pytest

from carvision.parking_bay import validate
from carvision.parking_mission import ParkingMission, plan_entry, contained
from carvision.parking_path import pursuit
from carvision.parking_control import calibration_tables, parking_blockers, parking_intent
from carvision.parking_entry_run import ParkingEntryRun
from carvision.cone_trial_output import ConePulseGuard
from test_parking_bay import calibrated, sample, slots
from test_parking_entry_run import configurations, base, preparation, command, Actuator, sample as camera_sample


def measured_settings():
    s=calibrated()
    s['motion_calibration']={'verified':True,'source':'synthetic test fixture, NOT field data',
        'speed_pwm':[{'speed_mps':v,'pwm_us':p} for v,p in
                     ((.05,1540),(.075,1550),(.1,1560),(.125,1570),(.15,1575))],
        'curvature_pwm':[{'curvature_1pm':k,'pwm_us':p} for k,p in
                         ((-2.5,1870),(-1.,1730),(0.,1610),(1.,1510),(2.5,1350))]}
    return validate(s)


def measured_cones():
    c=configurations()[2]
    c['driving']={'fixed_camera_confirmed':True,'steering_verified':True,
        'odometry_verified':True,'steering_radius_right_cm':40.,'steering_radius_left_cm':-40.,
        'visual_error_bound_m':.01,'visual_verified_distance_m':6.}
    return c


def selected_mission(blocked='left'):
    m=ParkingMission(measured_settings())
    for i in range(7):
        r=m.step(sample(i,i*.1,blocked=blocked),i*.1)
    assert r['action']=='follow_path',r
    return m


@pytest.mark.parametrize('blocked',['left','right'])
@pytest.mark.parametrize('yaw',[0.,-.08,.08])
def test_closed_loop_bicycle_follows_actual_pwm_and_parks_inside(blocked,yaw):
    """Feed back integrated vehicle motion, not poses copied from planned path."""
    s=measured_settings();m=ParkingMission(s);pose={'x_m':0.,'y_m':0.,'yaw_rad':yaw}
    speed=0.;dt=.1;commands=[];phases=set()
    speed_rows,steer_rows=calibration_tables(s)
    guard=ConePulseGuard({'pwm':{'neutral_us':1500,'steering_center_us':1610,
        'search_us':1600,'creep_us':1575,'brake_us':1400},
        'parking_forward_us':speed_rows[1],'brake_pulse_s':.12,'brake_release_s':.1,'actuator_lease_s':.2},
        lambda ch,v:None,mode='F/B')
    for i in range(601):
        t=i*dt
        decision=m.step(sample(i,t,dict(pose),blocked,speed),t)
        assert decision['phase']!='fault',decision
        phases.add(decision['phase'])
        output=parking_intent(decision,s)
        guard.accept({**output,'sequence':i,'sent_s':t},t)
        if decision['phase']=='parked':
            break
        commands.append(output)
        goal_speed=output.get('command_speed_mps',0.)
        # Finite acceleration and braking, independently integrated from PWM.
        speed+=max(-.4*dt,min(.3*dt,goal_speed-speed))
        curvature=float(np.interp(output['steering_us'],steer_rows[1][::-1],steer_rows[0][::-1]))
        middle=pose['yaw_rad']+.5*speed*curvature*dt
        pose['x_m']+=speed*math.sin(middle)*dt
        pose['y_m']+=speed*math.cos(middle)*dt
        pose['yaw_rad']+=speed*curvature*dt
    assert decision['phase']=='parked',decision
    assert decision['payment_ready'] and decision['four_wheels_inside']
    assert abs(speed)<=s['stopped_speed_mps']
    box=(-.615,2.,0.,3.) if blocked=='right' else (0.,2.,.615,3.)
    assert contained(pose,box,s,.03)
    assert 'braking' in phases and not guard.fault
    moving=[r for r in commands if r['esc_us']>1500]
    assert min(r['command_speed_mps'] for r in moving[-10:]) < max(r['command_speed_mps'] for r in moving)
    assert all(1350<=r['steering_us']<=1870 and r['esc_us']<=1625 for r in commands)


@pytest.mark.parametrize('side,x,blocked',[('right',.3075,'left'),('left',-.3075,'right')])
def test_already_aligned_close_start_uses_straight_entry(side,x,blocked):
    s=measured_settings()
    path=plan_entry(slots(blocked),side,{'x_m':x,'y_m':1.8,'yaw_rad':0.},s)
    assert all(p['x_m']==x and p['curvature']==0 for p in path)
    assert contained(path[-1],(x-.3075,2.,x+.3075,3.),s,.03)


def test_insufficient_distance_still_rejects_lateral_entry():
    with pytest.raises(ValueError,match='距离不足'):
        plan_entry(slots(),'right',{'x_m':0.,'y_m':1.8,'yaw_rad':0.},measured_settings())


def test_arc_length_lookahead_and_monotone_progress():
    path=[{'x_m':0.,'y_m':0.},{'x_m':.3,'y_m':.4},{'x_m':.3,'y_m':1.}]
    r=pursuit(path,{'x_m':0.,'y_m':0.,'yaw_rad':0.},0.,.25)
    assert r['target_x_m']==pytest.approx(.15) and r['target_y_m']==pytest.approx(.2)
    ahead=pursuit(path,{'x_m':.3,'y_m':.6,'yaw_rad':0.},0.,.25)
    back=pursuit(path,{'x_m':.3,'y_m':.59,'yaw_rad':0.},ahead['progress_m'],.25)
    assert back['progress_m']>=ahead['progress_m']


@pytest.mark.parametrize('mutation',[
    lambda x:x.update(obstacle_region_verified=False),
    lambda x:x.update(pose=None),
    lambda x:x.update(pose=['bad']),
    lambda x:x['pose'].update(verified=False),
    lambda x:x['pose'].update(captured_s=0.),
    lambda x:x['pose'].update(x_m=float('nan')),
    lambda x:x.update(speed_mps=.16),
    lambda x:x.update(speed_mps=-.02),
    lambda x:x['slots'][1].update(availability='unknown'),
    lambda x:x['slots'][1].update(occupancy_verified=False),
])
def test_feedback_loss_after_selection_latches_stop_and_never_resumes(mutation):
    m=selected_mission();x=sample(7,.7);mutation(x)
    r=m.step(x,.7)
    assert r['phase']=='fault' and r['action']=='brake'
    assert m.step(sample(8,.8),.8)['action']=='stop'


def test_delayed_control_with_fresh_images_cannot_resume():
    m=selected_mission()
    assert m.step(sample(7,1.),1.)['phase']=='fault'


def test_actual_command_arc_is_checked_in_addition_to_reference_path():
    s=measured_settings();m=selected_mission()
    m.previous_pose=None
    # Rear axle lies on the clear-bay path, but a left-facing actual body would
    # cross the divider before it can brake. Ideal route remains valid.
    pose={'x_m':.3075,'y_m':1.58,'yaw_rad':-.33}
    r=m.step(sample(7,.7,pose,speed=.1),.7)
    assert r['action']=='brake' and r['phase']=='fault'


def test_inside_bay_but_still_moving_cannot_complete():
    m=selected_mission();m.previous_pose=None;m.phase='braking';m.brake_since=.7
    end=m.path[-1]
    for i in range(7,15):
        r=m.step(sample(i,i*.1,end,speed=.02),i*.1)
        assert not r['payment_ready']


@pytest.mark.parametrize('bad',[
    {'verified':False}, {'source':''}, {'speed_pwm':[]},
    {'curvature_pwm':[{'curvature_1pm':0.,'pwm_us':1610}]*3},
    {'curvature_pwm':[{'curvature_1pm':-1.,'pwm_us':2000},{'curvature_1pm':0.,'pwm_us':1610},
                      {'curvature_1pm':1.,'pwm_us':1300}]},
])
def test_missing_or_invalid_motion_tables_block_output(bad):
    s=measured_settings();s['motion_calibration'].update(bad)
    assert parking_blockers({'parking':s,'cones':measured_cones()})
    with pytest.raises(ValueError):calibration_tables(s)


def test_speed_is_not_old_fixed_throttle_and_no_curve_extrapolation():
    s=measured_settings()
    d={'action':'follow_path','reason':'test','target_speed_mps':.08,'target_curvature_1pm':.5}
    out=parking_intent(d,s)
    assert out['esc_us']==1550 and out['command_speed_mps']==.075 and out['steering_us']==1560
    with pytest.raises(ValueError,match='禁止外推'):
        parking_intent({**d,'target_curvature_1pm':3.},s)


def test_default_unverified_profile_does_not_reserve_session_or_open_hardware():
    made=[]
    run=ParkingEntryRun(lambda s:made.append(s))
    with pytest.raises(ValueError,match='路径入库缺少'):
        run.prepare(*configurations(),base(),preparation(controller_mode='metric_path'),0.)
    assert not run.active and not made


def test_metric_session_uses_world_feedback_not_browser_selected_side():
    actor=Actuator();run=ParkingEntryRun(lambda s:actor)
    p=measured_settings();_,b,_=configurations()
    run.prepare(p,b,measured_cones(),base(),preparation(controller_mode='metric_path',selected_slot='left'),0.)
    run.tick(camera_sample(.1,parking_world=sample(1,.1)),.1)
    run.command('parking_trial_start',command(run,0,fb_mode_confirmed=True,esc_on_static=True,field_ready=True),.2)
    for i in range(1,41):
        t=.2+i*.1
        run.command('parking_trial_keepalive',command(run,i,selected_slot='left'),t)
        run.tick(camera_sample(t,parking_world=sample(i,t)),t)
    assert run.active and run.controller.mission.selected=='right'
    assert actor.commands[-1]['action'].startswith('parking_')
    # No world feedback: never fall back to pixel seeking or a stale direction.
    run.command('parking_trial_keepalive',command(run,41),4.3)
    run.tick(camera_sample(4.3),4.3)
    assert not run.active and actor.closed and run.phase=='fault'


def test_parking_pwm_whitelist_does_not_open_the_original_guard_profile():
    s={'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':1600,'creep_us':1575,'brake_us':1400},
       'brake_pulse_s':.12,'brake_release_s':.1,'actuator_lease_s':.2}
    guard=ConePulseGuard(s,lambda ch,v:None,mode='F/B')
    with pytest.raises(ValueError):
        guard.accept({'sequence':1,'sent_s':1.,'action':'parking_1550','esc_us':1550,'steering_us':1610},1.)
    with pytest.raises(ValueError):
        ConePulseGuard({**s,'parking_forward_us':[1700]},lambda ch,v:None,mode='F/B')


def test_parking_whitelist_keeps_original_profile_and_brake_deadline():
    s={'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':1600,'creep_us':1575,'brake_us':1400},
       'parking_forward_us':[1540,1560],'brake_pulse_s':.12,'brake_release_s':.1,'actuator_lease_s':.2}
    original=copy.deepcopy(s);writes=[]
    guard=ConePulseGuard(s,lambda ch,v:writes.append((ch,v)),mode='F/B')
    def send(seq,t,action,esc):
        guard.accept({'sequence':seq,'sent_s':t,'action':action,'esc_us':esc,'steering_us':1610},t)
    send(1,1.,'parking_1540',1540)
    assert guard.esc==1540 and guard.s==original and s==original
    send(2,1.1,'search',1600)
    assert guard.esc==1600
    send(3,1.2,'parking_1560',1560)
    send(4,1.3,'brake',1400)
    send(5,1.4,'brake',1400)
    guard.tick(1.43)
    assert guard.esc==1500 and guard.s==original and not guard.fault


def test_metric_page_displays_missing_measurements_and_sends_explicit_mode():
    from carvision.parking_bay_controls import SCRIPT
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node.js is required for the browser contract check')
    render=SCRIPT[SCRIPT.index('function renderParkingTrial'):SCRIPT.index('function parkingTrialRunSend')]
    prepare=SCRIPT[SCRIPT.index('async function parkingTrialPrepare'):SCRIPT.index('async function parkingTrialStart')]
    script="""
const assert=require('assert/strict'),nodes=new Map();
function el(id){if(!nodes.has(id))nodes.set(id,{value:id==='parking-trial-controller'?'metric_path':'1575',disabled:false,checkValidity:()=>true});return nodes.get(id);}
let parkingTrialToken=null,parkingTrialSequence=0,parkingTrialGeneration=0,parkingTrialActive=false,parkingTrialOwned=false;
let parkingTrialStopping=false,parkingTrialBusy=false,parkingTrialError='',stale=false,client='page',lastStatus=null;
const sent=[];async function send(action,payload){sent.push([action,payload]);return lastStatus;}
function render(s){renderParkingTrial(s);}
""" + render + prepare + """
(async()=>{
 lastStatus={parking_trial:{active:false,can_prepare:true,metric_missing:['need measured speed']}};
 render(lastStatus);
 assert.equal(el('parking-trial-prepare').disabled,true);
 assert.equal(el('parking-trial-forward').disabled,true);
 assert.match(el('parking-trial-missing').textContent,/need measured speed/);
 lastStatus.parking_trial.metric_missing=[];render(lastStatus);
 assert.equal(el('parking-trial-prepare').disabled,false);
 await parkingTrialPrepare();
 assert.equal(sent[0][1].controller_mode,'metric_path');
 el('parking-trial-controller').value='visual_trial';render(lastStatus);
 assert.equal(el('parking-trial-forward').disabled,false);
 await parkingTrialPrepare();
 assert.equal(sent[1][1].controller_mode,'visual_trial');
})().catch(e=>{console.error(e);process.exitCode=1;});
"""
    result=subprocess.run([node,'-'],input=script,text=True,encoding='utf-8',capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr
