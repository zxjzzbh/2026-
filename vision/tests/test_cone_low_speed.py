"""Bounded supervised trial regressions; no vehicle or actuator is opened."""
import copy
import json
import os
from pathlib import Path

import numpy as np
import pytest

from carvision.cone_low_speed import PROFILE,trial_settings,LowSpeedNavigator,DualLowSpeedNavigator,LowSpeedController
from carvision.cone_navigation import reuse_legacy_ground,execution_readiness
from carvision.cone_planning import load_settings
from carvision.cone_execution import ConeRun

ROOT=Path(__file__).parents[1]


@pytest.fixture
def settings():
    s=load_settings(Path(os.environ.get('CARVISION_REPOSITORY_CONE_CONFIG',ROOT/'configs/cone-trial.json')))
    reference=json.loads((ROOT/'configs/cone-legacy-ground-candidate.json').read_text(encoding='utf-8'))
    s=reuse_legacy_ground(s,reference);s['geometry']['lane_width_m']=1.25
    return s


def observation(t,cones=None,**kwargs):
    cone={'id':'one','fraction':.66,'half_width':.03,'near':True,'y_fraction':.60,
          'reference_distance_m':None,'bbox_xyxy':[290,190,305,217]}
    return {'experimental':True,'ready':True,'captured_s':t,'frame_id':round(t*100),
            'ego_fraction':.5,'half_envelope':.20,'cones':[cone] if cones is None else cones,
            'lane_bounds_px':[[100,380],[50,430]],'uncertain':[],**kwargs}


def row(t=1.,fid=1):
    return {'captured_s':t,'frame_id':fid,'road_context':{'valid':True,
        'left_inner':[[100.,y] for y in np.linspace(180,260,12)],
        'right_inner':[[380.,y] for y in np.linspace(180,260,12)]},
        'tracks':[{'id':'one','visible':True,'confirmed':True,'ground_contact_usable':True,
                   'clipped':False,'foot_pixel':[290,210],'bbox_xyxy':[281,181,300,217]}]}


def test_profile_uses_reported_limits_without_changing_verified_flags(settings):
    before=copy.deepcopy(settings);s=trial_settings(settings)
    assert settings==before and not execution_readiness(s)['ready']
    assert s['driving']['forward_us']==1600 and s['low_speed_test']['stopping_budget_m']==.27
    assert not s['motion']['verified'] and not s['ground']['verified']


@pytest.mark.parametrize('values',[{'test_forward_us':1601},{'test_forward_us':True},
    {'test_run_s':121},{'test_run_s':float('nan')},{'test_run_s':True},{'test_burst_s':.25},
    {'test_left_us':1755},{'test_left_us':1645},{'test_left_us':True},
    {'test_right_us':1545},{'test_right_us':1575}])
def test_trial_cannot_expand_actuator_envelope(settings,values):
    with pytest.raises(ValueError):trial_settings(settings,values)


def test_no_history_does_not_silently_use_pixel_defaults(settings):
    settings['ground']['historical_verified']=False
    with pytest.raises(ValueError):trial_settings(settings)


def test_real_image_pipeline_uses_observed_white_lines_without_metric_extrapolation(settings):
    nav=LowSpeedNavigator(trial_settings(settings));data=row()
    result=nav.update(np.zeros((360,480,3),np.uint8),data,1.)
    assert result['ready'] and len(result['cones'])==1
    assert result['cones'][0]['reference_distance_m'] is None
    assert not result['field_verified'] and not result['metric_boundary_verified']
    assert json.loads(json.dumps(result,allow_nan=False))['cones'][0]['near'] is False


@pytest.mark.parametrize('case',['lost','near_edge','crossed','stale'])
def test_white_line_or_frame_guard_stops_before_any_drive(settings,case):
    data=row();now=1.
    if case=='lost':data['road_context']['valid']=False
    if case=='near_edge':data['road_context']['left_inner']=[[225.,y] for y in np.linspace(180,260,12)]
    if case=='crossed':data['road_context']['left_inner']=[[400.,y] for y in np.linspace(180,260,12)]
    if case=='stale':now=1.4
    result=LowSpeedNavigator(trial_settings(settings)).update(np.zeros((360,480,3),np.uint8),data,now)
    assert not result['ready'] and result['action']=='stop'


def test_ambiguous_blue_in_corridor_cannot_be_ignored(settings):
    data=row();data['tracks'][0]['ground_contact_usable']=False
    result=LowSpeedNavigator(trial_settings(settings)).update(np.zeros((360,480,3),np.uint8),data,1.)
    assert not result['ready'] and result['uncertain']==['one']


@pytest.mark.parametrize('with_cone',[True,False])
def test_continuous_forward_has_no_periodic_braking_or_four_second_cutoff(settings,with_cone):
    ctl=LowSpeedController(trial_settings(settings),0.)
    assert ctl.step(observation(1),1)['action']=='neutral'
    for t in np.arange(3.01,12.,.05):
        intent=ctl.step(observation(float(t),cones=None if with_cone else []),float(t))
        assert intent['action']=='search' and intent['esc_us']==1600
        assert intent['steering_us']==(1690 if with_cone else 1610)
    assert ctl.powered_s>8.


def test_cruising_updates_steering_when_cone_appears_without_braking(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    assert ctl.step(observation(3.1,cones=[]),3.1)['steering_us']==1610
    result=ctl.step(observation(3.2),3.2)
    assert result['action']=='search' and result['steering_us']==1690


def test_cone_turning_is_tunable_but_empty_lane_uses_old_small_corrections(settings):
    ctl=LowSpeedController(trial_settings(settings,{'test_left_us':1730,'test_right_us':1560}),0.)
    assert ctl.step(observation(3.1,cones=[],ego_fraction=.55),3.1)['steering_us']==1650
    assert ctl.step(observation(3.2),3.2)['steering_us']==1730
    ctl=LowSpeedController(trial_settings(settings,{'test_left_us':1730,'test_right_us':1560}),0.)
    data=observation(3.1);data['cones'][0]['fraction']=.34
    assert ctl.step(data,3.1)['steering_us']==1560


def test_near_cone_inside_turn_buffer_can_move_away_when_body_clear(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    data=observation(3.1,ego_fraction=.438,half_envelope=.203)
    data['cones'][0].update(fraction=.663,half_width=.034,y_fraction=.714,image_close=True)
    intent=ctl.step(data,3.1)
    assert intent['action']=='search' and intent['steering_us']==1690


def test_near_cone_inside_body_clearance_still_stops(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    data=observation(3.1,ego_fraction=.438,half_envelope=.203)
    data['cones'][0].update(fraction=.61,half_width=.034,y_fraction=.714,image_close=True)
    with pytest.raises(ValueError,match='车身净空'):ctl.step(data,3.1)


@pytest.mark.parametrize('case',['freshness','white','early_loss','close_cone','other_cone'])
def test_runtime_hazards_interrupt_powered_segment(settings,case):
    ctl=LowSpeedController(trial_settings(settings),0.)
    first=observation(3.01)
    if case=='early_loss':first['cones'][0]['near']=False
    ctl.step(first,3.01);data=observation(3.1)
    if case=='freshness':data['captured_s']=2.
    if case=='white':data.pop('lane_bounds_px')
    if case=='early_loss':data['cones']=[]
    if case=='close_cone':data['cones'][0].update(y_fraction=.72,fraction=.55)
    if case=='other_cone':data['cones'].append({**data['cones'][0],'id':'two','y_fraction':.7,'fraction':.5})
    if case=='white':
        assert ctl.step(data,3.1)['action']=='brake'
        assert ctl.count==0
    else:
        with pytest.raises(ValueError):ctl.step(data,3.1)


@pytest.mark.parametrize('available',['primary','secondary'])
def test_either_camera_can_supply_white_boundaries(settings,available):
    data=row();data['camera_id']=available
    images={available:np.zeros((360,480,3),np.uint8)}
    result=DualLowSpeedNavigator(trial_settings(settings)).update_views(images,{available:data},1.)
    assert result['ready'] and result['source_camera']==available
    assert result['line_sources']==[available] and not result['line_held']
    json.dumps(result,allow_nan=False)


def test_lane_memory_expires_from_real_observation_not_reused_frame(settings):
    nav=DualLowSpeedNavigator(trial_settings(settings));image=np.zeros((360,480,3),np.uint8)
    assert nav.update_views({'secondary':image},{'secondary':row()},1.)['ready']
    for i,t in enumerate((1.1,1.3,1.6),2):
        data=row(t,i);data['road_context']['valid']=False
        result=nav.update_views({'secondary':image},{'secondary':data},t)
        assert result['line_held'] and result['line_observed_s']==1.
        assert result['line_age_s']==pytest.approx(t-1.)
    data=row(1.9,5);data['road_context']['valid']=False
    result=nav.update_views({'secondary':image},{'secondary':data},1.9)
    assert not result['ready'] and not result.get('lane_bounds_px')


def test_stale_camera_is_not_rescued_by_line_memory(settings):
    nav=DualLowSpeedNavigator(trial_settings(settings));images={'secondary':np.zeros((360,480,3),np.uint8)}
    nav.update_views(images,{'secondary':row()},1.)
    result=nav.update_views(images,{'secondary':row()},1.4)
    assert not result['ready'] and result['captured_s']==1.


def test_visible_boundary_danger_overrides_another_camera(settings):
    nav=DualLowSpeedNavigator(trial_settings(settings));a=row();b=row()
    b['road_context']['left_inner']=[[225.,y] for y in np.linspace(180,260,12)]
    image=np.zeros((360,480,3),np.uint8)
    result=nav.update_views({'primary':image,'secondary':image},{'primary':a,'secondary':b},1.)
    assert result['boundary_danger'] and not result['ready']


def test_arming_survives_single_frame_line_miss_without_driving(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    lost=observation(1.);lost.pop('lane_bounds_px')
    assert ctl.step(lost,1.)['action']=='neutral'
    assert ctl.step(observation(1.1),1.1)['action']=='neutral'
    assert ctl.step(observation(3.1),3.1)['action']=='search'


def test_line_hold_has_independent_power_budget_and_cannot_extend_burst(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    ctl.step(observation(3.01),3.01)
    assert ctl.step(observation(3.06,line_held=True),3.06)['action']=='search'
    assert ctl.step(observation(3.18,line_held=True),3.18)['action']=='brake'
    assert ctl.step(observation(3.4,line_held=True),3.4)['action']=='neutral'
    assert ctl.step(observation(4.,line_held=True),4.)['action']=='neutral'
    assert ctl.step(observation(4.1,line_held=False),4.1)['action']=='search'


def test_both_lines_missing_pause_then_end_without_extra_pass(settings):
    ctl=LowSpeedController(trial_settings(settings),0.);ctl.step(observation(3.01),3.01)
    def lost(t):
        data=observation(t);data.pop('lane_bounds_px');return data
    assert ctl.step(lost(3.1),3.1)['action']=='brake'
    assert ctl.step(lost(3.3),3.3)['action']=='neutral'
    assert ctl.step(lost(4.),4.)['action']=='neutral'
    assert ctl.count==0
    with pytest.raises(ValueError,match='持续未恢复'):ctl.step(lost(6.2),6.2)


def test_camera_switch_reassociates_unique_cone_without_double_count(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    ctl.step(observation(3.01,source_camera='secondary'),3.01)
    next_view=observation(3.1,source_camera='primary');next_view['cones'][0]['id']='primary-one'
    assert ctl.step(next_view,3.1)['action']=='search'
    assert ctl.target=='primary-one' and ctl.count==0


def test_disappearance_never_automatically_counts_pass_and_next_side_alternates(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    ctl.step(observation(3.01),3.01)
    for t in np.arange(3.06,15,.05):
        ctl.step(observation(float(t),cones=[]),float(t))
        if ctl.phase=='review':break
    assert ctl.phase=='review' and ctl.count==0
    assert ctl.tail_powered<1.1
    assert ctl.step(observation(15.,cones=[]),15.)['action']=='neutral'
    ctl.confirm_pass();assert ctl.count==1
    next_cone={'id':'two','fraction':.34,'half_width':.03,'near':False,
               'y_fraction':.55,'reference_distance_m':None}
    intent=ctl.step(observation(15.1,cones=[next_cone]),15.1)
    assert intent['action']=='search' and ctl.side=='right'
    with pytest.raises(ValueError):ctl.confirm_pass()


def test_configured_wall_clock_limit_stops_continuous_trial(settings):
    ctl=LowSpeedController(trial_settings(settings,{'test_run_s':10}),0.)
    assert ctl.step(observation(3.1,cones=[]),3.1)['action']=='search'
    with pytest.raises(ValueError,match='10 秒运行时限'):ctl.step(observation(10.),10.)


@pytest.mark.parametrize('phase',['review','complete'])
def test_confirmation_and_completion_cannot_resume_after_line_loss(settings,phase):
    ctl=LowSpeedController(trial_settings(settings),0.);ctl.phase=phase
    lost=observation(3.1);lost.pop('lane_bounds_px')
    result=ctl.step(lost,3.1)
    assert result['action']=='neutral' and result['phase']==phase
    result=ctl.step(observation(3.2,cones=[]),3.2)
    assert result['action']=='neutral' and result['phase']==phase


class Actuator:
    run=True
    def __init__(self):self.intents=[];self.closed=False
    def send(self,intent):self.intents.append(intent);return {'hardware_output':True}
    def close(self):self.closed=True


def test_experimental_session_is_exclusive_leased_and_never_implicitly_starts(settings,tmp_path):
    created=[]
    def factory(s):
        a=Actuator();created.append(a);return a
    run=ConeRun(factory,tmp_path)
    base={'mode':'disabled','boot_test_phase':'esc_off','setup_token':'test-setup'}
    payload={'profile':PROFILE,'client':'page','setup_token':'test-setup','esc_off':True,'fb_mode_confirmed':True}
    run.prepare(settings,base,payload,1.)
    assert not created
    run.tick(observation(1.1,cones=[]),1.1)
    assert created[0].intents[-1]['action']=='neutral'
    assert run.status(settings,base,1.1)['can_start']
    with pytest.raises(ValueError):run.command('cone_start',{'client':'other','run_token':run.token,'trial_sequence':1},1.2)
    run.command('cone_start',{'client':'page','run_token':run.token,'trial_sequence':1,
        'fb_mode_confirmed':True,'esc_on_static':True,'field_ready':True},1.2)
    run.tick(observation(1.3),1.3)
    assert all(i['action']=='neutral' for i in created[0].intents)
    run.tick(observation(1.8),1.8)
    assert not run.active and created[0].closed and run.phase=='fault'
    with pytest.raises(ValueError):run.command('cone_start',{},2.)


def test_normal_mode_still_requires_calibration(settings,tmp_path):
    run=ConeRun(lambda _:pytest.fail('must not open actuator'),tmp_path)
    base={'mode':'disabled','boot_test_phase':'esc_off','setup_token':'x'}
    with pytest.raises(ValueError,match='标定未完成'):
        run.prepare(settings,base,{'client':'page','setup_token':'x','esc_off':True,'fb_mode_confirmed':True},1.)


def test_low_speed_buttons_use_own_prepared_session_and_manual_pass_confirmation():
    import os,shutil,subprocess
    from carvision.cone_controls import STATE,SCRIPT
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node is required for actual page render logic')
    source=STATE+SCRIPT[:SCRIPT.index('async function coneAction')]
    js=r'''
const vm=require('node:vm'),fs=require('node:fs'),assert=require('node:assert/strict'),nodes=new Map();
const env={stale:false,client:'page',held:new Set(),gamepadControl:null,document:{querySelectorAll(){return []}},el(id){
 if(!nodes.has(id))nodes.set(id,{disabled:false,dataset:{},textContent:'',addEventListener(){}});return nodes.get(id)}};
vm.createContext(env);vm.runInContext(JSON.parse(fs.readFileSync(0,'utf8')).source,env);
function render(owner,canStart,review){env.renderCone({cone_trial:{low_speed_trial_available:true,avoidance:{
 active:true,can_prepare:false,profile:'legacy_low_speed',owner_client:owner,can_request_start:canStart,
 low_speed_test:{can_confirm_pass:review,operator_confirmed_count:0}},readiness:{ready:false,missing:['ground.verified_mapping']}}});}
render('page',true,false);assert(!env.el('cone-test-start').disabled);assert(env.el('cone-drive-start').disabled);
assert(env.el('cone-test-confirm').disabled);render('page',false,true);assert(!env.el('cone-test-confirm').disabled);
render('other',true,true);assert(env.el('cone-test-start').disabled);assert(env.el('cone-test-confirm').disabled);
console.log('Experimental controls preserve ownership; no motion sent');
'''
    result=subprocess.run([node,'-e',js],input=json.dumps({'source':source}),text=True,encoding='utf-8',capture_output=True,timeout=10)
    assert result.returncode==0,result.stdout+result.stderr
