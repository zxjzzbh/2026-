"""Synthetic slalom acceptance and motion replay. Never opens an actuator."""
import json
import math
import os
import shutil
import subprocess

import pytest

from carvision.cone_planning import ConeMission, ConeRoutePlanner, path_clear, load_settings, distance_to_path
from carvision.cone_slalom import SlalomProgress
from carvision.cone_navigation import steering_for_path, driving_settings, execution_readiness
from pathlib import Path


@pytest.fixture
def measured():
    # These coordinates / radii are simulation fixtures, NOT vehicle calibration.
    s = load_settings(Path(__file__).parents[1] / 'configs/cone-trial.json')
    s['geometry'].update(measured=True, lane_width_m=2., vehicle_width_m=.24,
        vehicle_length_m=.42, rear_axle_to_front_m=.32, rear_overhang_m=.1,
        cone_height_m=.2, cone_base_width_m=.16, clearance_m=.05)
    s['ground'].update(verified=True, homography=[[.01,0,-2.4],[0,-.04,14.4],[0,0,1]],
        valid_polygon_px=[[0,0],[479,0],[479,359],[0,359]], max_error_m=.01)
    s['motion'].update(verified=True, minimum_turn_radius_m=.4,
        maximum_tracking_error_m=.02, braking_distance_m=.15, pose_feedback_verified=True)
    s['driving'] = {**driving_settings(s), 'steering_radius_right_cm':40.,
        'steering_radius_left_cm':-40., 'visual_error_bound_m':.01,
        'visual_verified_distance_m':15., 'fixed_camera_confirmed':True,
        'steering_verified':True, 'odometry_verified':True, 'max_run_s':60.}
    return s


def cone(key, x, y):
    return {'id':key, 'x_m':x, 'y_m':y, 'radius_m':.08}


def pose(x=0., y=0., yaw=0.):
    return {'valid':True, 'x_m':x, 'y_m':y, 'yaw_rad':yaw, 'error_bound_m':.01}


def sample(i, cones, p=None, lane=2., coverage=12.):
    p = p or pose()
    return {'frame_id':i+1, 'captured_s':1.+i*.1, 'pose':p,
        'observed_world':cones, 'unresolved_obstacles':[], 'lane_valid':True,
        'lane_bounds_m':[-lane/2-p['x_m'], lane/2-p['x_m']],
        'lane_offset_m':p['x_m'], 'lane_heading_rad':-p['yaw_rad'],
        'coverage_forward_m':coverage}


def step(mission, data):
    return mission.step(data, data['captured_s'])


def test_centered_cones_require_opposite_sides_and_lane_recovery(measured):
    cones = [cone('near',0,2.5), cone('far',0,7.)]
    result = step(ConeMission(measured), sample(0,cones))
    assert result['action'] == 'follow_candidate_path'
    order = result['slalom']['order']
    assert [c['id'] for c in order] == ['near','far']
    assert [c['side'] for c in order] in [['left','right'], ['right','left']]
    offsets = [min(result['path'],key=lambda p:abs(p['y_m']-c['y_m']))['x_m'] for c in cones]
    assert offsets[0] * offsets[1] < 0
    assert path_clear(result['path'],cones,measured,[-1,1])
    assert abs(result['path'][-1]['x_m']) < .001
    assert not result['hardware_output']


@pytest.mark.parametrize('count',[0,1])
def test_no_drive_before_both_cones_are_confirmed(measured,count):
    result = step(ConeMission(measured),sample(0,[cone('near',0,2.5)][:count]))
    assert result['action'] == 'stop' and result['phase'] == 'waiting_for_two_cones'


def test_same_side_escape_is_rejected_when_weave_cannot_fit(measured):
    cones = [cone('near',0,1.8),cone('far',0,3.)]
    assert ConeRoutePlanner(measured).plan(cones,lane_bounds=(-1,1),coverage_forward_m=6)['action']=='follow_candidate_path'
    result = step(ConeMission(measured),sample(0,cones,coverage=6))
    assert result['action']=='stop' and result['reason']=='no_feasible_alternating_route'


def test_side_by_side_cones_cannot_be_counted_as_ordered_slalom(measured):
    result=step(ConeMission(measured),sample(0,[cone('one',-.5,2.5),cone('two',.5,2.5)]))
    assert result['action']=='stop' and result['reason']=='cones_too_close_for_alternation'


def test_side_assignment_survives_reacquisition_and_detection_order(measured):
    cones = [cone('near',0,2.5),cone('far',0,7.)]
    mission = ConeMission(measured)
    first = step(mission,sample(0,cones))
    changed = [cone('new-far',0,7.),cone('new-near',0,2.5)]
    second = step(mission,sample(1,changed))
    assert second['slalom']['order']==first['slalom']['order']
    assert mission.counter.aliases['new-near']=='near'
    assert second['action']=='follow_candidate_path'


def test_obstacle_moves_after_commit_latches_fault(measured):
    mission = ConeMission(measured)
    cones = [cone('near',0,2.5),cone('far',0,7.)]
    step(mission,sample(0,cones))
    result = step(mission,sample(1,[cone('near',.2,2.5),cones[1]]))
    assert result['phase']=='fault' and result['reason']=='slalom_cone_position_changed'
    assert step(mission,sample(2,cones))['action']=='stop'


def test_brief_uncertain_obstacle_pause_keeps_fresh_pose_history(measured):
    cones=[cone('near',0,2.5),cone('far',0,7.)]
    mission=ConeMission(measured)
    first=step(mission,sample(0,cones))
    for i in range(1,6):
        data=sample(i,cones)
        data['unresolved_obstacles']=['temporary-blue-shape']
        result=step(mission,data)
        assert result['action']=='stop'
    resumed=step(mission,sample(6,cones))
    assert resumed['action']=='follow_candidate_path'
    assert resumed['slalom']['order']==first['slalom']['order']
    assert resumed['slalom']['passed_ids']==[]


def test_uncertain_obstacle_does_not_hide_pose_jump(measured):
    cones=[cone('near',0,2.5),cone('far',0,7.)]
    mission=ConeMission(measured);step(mission,sample(0,cones))
    data=sample(1,cones,pose(y=1.))
    data['unresolved_obstacles']=['temporary-blue-shape']
    result=step(mission,data)
    assert result['phase']=='fault'
    assert result['reason']=='slalom_pose_discontinuity'


def test_between_reference_samples_is_not_tracking_deviation(measured):
    measured['motion']['maximum_tracking_error_m']=.006
    cones=[cone('near',0,2.5),cone('far',0,7.)]
    mission=ConeMission(measured)
    initial=sample(0,cones);initial['pose']['error_bound_m']=.001
    result=step(mission,initial)
    a,b=result['path'][:2]
    data=sample(1,cones,pose((a['x_m']+b['x_m'])/2,(a['y_m']+b['y_m'])/2))
    data['pose']['error_bound_m']=.001
    result=step(mission,data)
    assert result['action']=='follow_candidate_path'


@pytest.mark.parametrize('x,y,expected',[(.03,.01,.03),(0.,-.02,.02),(0.,.05,.03)])
def test_path_projection_keeps_lateral_and_endpoint_error(x,y,expected):
    path=[{'x_m':0.,'y_m':0.},{'x_m':0.,'y_m':.02}]
    error,_=distance_to_path(path,x,y)
    assert error==pytest.approx(expected)


def test_path_projection_cannot_rewind_completed_segments():
    path=[{'x_m':0.,'y_m':0.},{'x_m':0.,'y_m':1.},{'x_m':0.,'y_m':2.}]
    error,index=distance_to_path(path,0.,.1,start_index=1)
    assert error==pytest.approx(.9) and index==1


def test_deployment_respects_other_live_tasks_and_calibration():
    # Execute the actual read-only predicates without importing SSH/hardware
    # libraries into the offline suite.
    import ast
    tools=Path(os.environ.get('CARVISION_DEPLOY_SOURCE_DIR',Path(__file__).parents[1]/'tools'))
    namespace={}
    for filename,names in [('deploy_traffic_trial.py',{'idle'}),
                           ('deploy_cone_trial.py',{'idle','verify_protected'})]:
        tree=ast.parse((tools/filename).read_text(encoding='utf-8'))
        functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names]
        exec(compile(ast.Module(body=functions,type_ignores=[]),str(tools/filename),'exec'),namespace)
        if filename=='deploy_traffic_trial.py':namespace['original_idle']=namespace['idle']
    base={'mode':'disabled','boot_test_phase':'esc_off'}
    assert namespace['idle'](base)
    for key,value in [('departure_trial',{'active':True}),('parking_payment',{'motion_locked':True}),
                      ('parking_mission',{'active':True}),('parking_bay',{'motor_output_enabled':True}),
                      ('cone_trial',{'short_test':{'active':True}})]:
        assert not namespace['idle']({**base,key:value})
    namespace.update(REMOTE='/car',read=lambda _sftp,path:b'original')
    namespace['verify_protected'](None,{'config.local.json':b'original'})
    namespace['read']=lambda _sftp,path:b'changed-during-deployment'
    with pytest.raises(RuntimeError,match='Protected control file changed'):
        namespace['verify_protected'](None,{'config.local.json':b'original'})


@pytest.mark.parametrize('case',['jump','gap','deviation'])
def test_pose_errors_cannot_skip_cones_or_restart_route(measured,case):
    mission = ConeMission(measured);cones = [cone('near',0,2.5),cone('far',0,7.)]
    step(mission,sample(0,cones))
    data=sample(1,cones,pose(y=8.) if case=='jump' else pose(x=.03) if case=='deviation' else pose())
    if case=='gap':data['captured_s']+=1.
    result=step(mission,data)
    assert result['action']=='stop' and result['phase']=='fault'
    assert not result['slalom']['complete']


def test_two_same_side_passes_never_count_as_slalom(measured):
    cones=[cone('near',0,2.5),cone('far',0,7.)]
    progress=SlalomProgress(measured,cones,{'near':'right','far':'left'},pose(x=.4),0.,1.)
    for i in range(1,450):
        progress.update(pose(.4,i*.02),1+i*.1,{c['id']:c for c in cones})
        if progress.fault:break
    assert progress.passed==['near'] and progress.fault=='slalom_wrong_side_or_clearance'
    assert not progress.status()['complete']


@pytest.mark.parametrize('heading',[0.,.25,-.25])
def test_pass_evidence_uses_fixed_road_frame_not_camera_yaw(measured,heading):
    co,si=math.cos(heading),math.sin(heading)
    def rotate(p):return pose(co*p['x_m']+si*p['y_m'], -si*p['x_m']+co*p['y_m'],heading)
    raw=[cone('near',0,2.5),cone('far',0,7.)]
    cones=[{**c,'x_m':si*c['y_m'],'y_m':co*c['y_m']} for c in raw]
    start=rotate(pose(.4,0))
    progress=SlalomProgress(measured,cones,{'near':'right','far':'left'},start,heading,1.)
    for i in range(1,180):progress.update(rotate(pose(.4,i*.02)),1+i*.1,{c['id']:c for c in cones})
    assert progress.passed==['near'] and progress.fault is None


def test_asymmetric_measured_curvature_is_checked(measured):
    measured['driving']['steering_radius_right_cm']=1000.
    assert not path_clear([{'x_m':0.,'y_m':0.,'yaw_rad':0.,'curvature':.2}],[],measured,[-1,1])


@pytest.mark.parametrize('mirror',[1,-1])
@pytest.mark.parametrize('coverage',[9.,12.])
def test_closed_loop_two_cones_with_calibrated_pwm_model_then_lane_recovery(measured,mirror,coverage):
    measured['geometry']['lane_width_m']=1.23
    cones=[cone('near',mirror*.2,2.5),cone('far',-mirror*.2,7.)]
    mission=ConeMission(measured);x=y=yaw=0.;crossings=[];dt=.1;speed=.2
    for i in range(650):
        result=step(mission,sample(i,cones,pose(x,y,yaw),lane=1.23,coverage=coverage))
        if result['phase']=='complete':break
        assert result['action']=='follow_candidate_path', (i,x,y,yaw,result['reason'])
        pulse=steering_for_path(result['path'],measured)
        assert 1350<=pulse<=1870
        # Fixture-only kinematics for the signed calibrated radius/PWM map.
        curvature=(1610-pulse)/260 * 2.5
        yaw+=speed*curvature*dt
        x+=speed*math.sin(yaw)*dt;y+=speed*math.cos(yaw)*dt
        crossings.append((x,y))
    else:pytest.fail('Slalom never reached lane recovery completion')
    assert result['slalom']['passed_ids']==['near','far']
    assert result['slalom']['gate_verified_ids']==['near','far']
    assert result['slalom']['order'][0]['side']==('left' if mirror==1 else 'right')
    assert abs(x)<=measured['motion']['maximum_tracking_error_m'] and abs(yaw)<=.05
    assert min(p[0] for p in crossings)<0<max(p[0] for p in crossings)


def test_repository_configuration_is_still_unverified():
    s=load_settings(Path(__file__).parents[1]/'configs/cone-trial.json')
    assert not execution_readiness(s)['ready']
    assert not s['ground']['verified'] and not s['motion']['verified']


@pytest.mark.parametrize('case',['missing','widened','lost_view'])
def test_fresh_corridor_still_limits_the_locked_world_route(measured,case):
    cones=[cone('near',0,2.5),cone('far',0,7.)];mission=ConeMission(measured)
    step(mission,sample(0,cones))
    data=sample(1,cones)
    if case=='missing':data['lane_valid']=False
    if case=='widened':data['lane_bounds_m']=[-5.,5.]
    if case=='lost_view':data['coverage_forward_m']=.3
    result=step(mission,data)
    assert result['action']=='stop' and result['slalom']['order']


def test_web_slalom_order_progress_and_existing_start_ownership():
    from carvision.cone_controls import SCRIPT, STATE
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node.js required for actual console render replay')
    source=STATE+SCRIPT[:SCRIPT.index('async function coneAction')]
    script=r'''
const vm=require('node:vm'),assert=require('node:assert/strict'),fs=require('node:fs');
const source=JSON.parse(fs.readFileSync(0,'utf8')).source,nodes=new Map();
const env={stale:false,client:'page',held:new Set(),gamepadControl:null,
 document:{querySelectorAll(){return [];}},el(id){
  if(!nodes.has(id))nodes.set(id,{disabled:false,textContent:'',dataset:{},addEventListener(){}});
  return nodes.get(id);
 }};
vm.createContext(env);vm.runInContext(source,env);
function render(can,owner,count,reason){env.renderCone({cone_trial:{
 avoidance:{active:true,profile:'legacy_low_speed',owner_client:owner,can_request_start:can,reason,
 low_speed_test:{passed_count:count}},latest:{navigation:{}},readiness:{missing:[]}}});}
render(true,'page',0,'等待开始');assert(!env.el('cone-test-start').disabled);
render(true,'page',1,'第二桶避让');assert(env.el('cone-test-progress').textContent.includes('已过 1 个'));
assert(env.el('cone-test-progress').textContent.includes('持续运行，由你停止'));
render(true,'other',1,'第二桶避让');assert(env.el('cone-test-start').disabled);
render(false,'page',1,'白线无可通行空间');assert(env.el('cone-test-start').disabled);
assert(env.el('cone-test-progress').textContent.includes('白线无可通行空间'));
console.log('slalom DOM replay passed; no network or hardware calls');
'''
    result=subprocess.run([node,'-e',script],input=json.dumps({'source':source}),
        encoding='utf-8',text=True,capture_output=True,timeout=15)
    assert result.returncode==0,result.stdout+result.stderr
