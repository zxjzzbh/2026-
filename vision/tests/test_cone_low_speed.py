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
    {'test_left_us':1875},{'test_left_us':1645},{'test_left_us':True},
    {'test_right_us':1345},{'test_right_us':1575}])
def test_trial_cannot_expand_actuator_envelope(settings,values):
    with pytest.raises(ValueError):trial_settings(settings,values)


def test_no_history_does_not_silently_use_pixel_defaults(settings):
    settings['ground']['historical_verified']=False
    with pytest.raises(ValueError):trial_settings(settings)


def test_old_radius_data_is_preserved_but_does_not_authorize_new_endpoints(settings):
    from carvision.cone_navigation import steering_for_path
    settings['driving']={'steering_radius_1570_cm':1203.,'steering_radius_1650_cm':-1203.,'steering_verified':True}
    report=execution_readiness(settings)
    assert 'driving.steering_radius_right_cm' in report['missing']
    assert settings['driving']['steering_radius_1570_cm']==1203.
    with pytest.raises(ValueError,match='旧档位'):
        steering_for_path([{'x_m':.1,'y_m':.5}],settings)


def test_simplified_panel_has_no_dangling_calibration_or_short_test_nodes():
    import re,shutil,subprocess
    from carvision.cone_controls import HTML,STATE,SCRIPT
    for removed in ('cone-short-section','cone-short-1570','cone-prepare','cone-drive-start','cone-calibration-json','cone-save-calibration','data-cone-geometry'):
        assert removed not in HTML
    ids=re.findall(r'id="([^"]+)"',HTML)
    assert len(ids)==len(set(ids))
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node required to check real panel initialization')
    source=r'''
const vm=require('node:vm'),fs=require('node:fs'),assert=require('node:assert/strict');
const input=JSON.parse(fs.readFileSync(0,'utf8')),nodes=new Map(input.ids.map(id=>[id,{disabled:false,dataset:{},value:'',textContent:'',addEventListener(){}}]));
const env={stale:false,client:'page',lastStatus:null,held:new Set(),gamepadControl:null,setInterval(){},
 document:{hidden:false,querySelectorAll(){return []}},el(id){if(!nodes.has(id))throw new Error('Missing DOM node: '+id);return nodes.get(id)}};
vm.createContext(env);vm.runInContext(input.source,env);env.renderCone({cone_trial:{observing:false,avoidance:{active:false},short_test:{active:false}}});
assert(nodes.has('cone-test-start')&&nodes.has('cone-white-save'));console.log('Concise task UI has no removed-element callbacks');
'''
    result=subprocess.run([node,'-e',source],input=json.dumps({'source':STATE+SCRIPT,'ids':ids}),text=True,encoding='utf-8',capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr


def test_real_image_pipeline_uses_observed_white_lines_without_metric_extrapolation(settings):
    nav=LowSpeedNavigator(trial_settings(settings));data=row()
    result=nav.update(np.zeros((360,480,3),np.uint8),data,1.)
    assert result['ready'] and len(result['cones'])==1
    assert result['cones'][0]['reference_distance_m'] is None
    assert not result['field_verified'] and not result['metric_boundary_verified']
    assert json.loads(json.dumps(result,allow_nan=False))['cones'][0]['near'] is False


def test_higher_visible_white_band_does_not_make_far_cone_a_pass_event(settings):
    nav=LowSpeedNavigator(trial_settings(settings));data=row()
    data['road_context']['left_inner']=[[100.,y] for y in np.linspace(140,215,12)]
    data['road_context']['right_inner']=[[380.,y] for y in np.linspace(140,215,12)]
    data['tracks'][0]['foot_pixel']=[290.,210.]
    result=nav.update(np.zeros((360,480,3),np.uint8),data,1.)
    assert result['ready'] and not result['cones'][0]['near']


def test_live_glare_pair_reaches_continuous_forward_without_false_primary_veto(settings):
    import cv2
    from carvision.cone_road import ConeRoadContext
    from carvision.cone_vision import BlueConeDetector,ImageConeTracker
    s=trial_settings(settings);s['scene']['surface']='indoor_white_line_track'
    navigator=DualLowSpeedNavigator(s);images={};rows={}
    for camera in ('primary','secondary'):
        image=cv2.imdecode(np.frombuffer((ROOT/f'tests/fixtures/cone-track-glare-{camera}.jpg').read_bytes(),np.uint8),1)
        images[camera]=image
        detections,mask=BlueConeDetector(s).detect(image)
        road=ConeRoadContext(s,camera);context=road.inspect(image,mask)
        road.enrich(detections,image,context);tracker=ImageConeTracker(camera,s)
        for i in range(4):tracks=tracker.update(detections,i,3+i*.1)
        rows[camera]={'frame_id':4,'captured_s':3.3,'road_context':context,'tracks':tracks}
    nav=navigator.update_views(images,rows,3.3)
    assert nav['ready'] and nav['source_camera']=='secondary',nav
    assert nav['line_sources']==['secondary'] and not nav['line_held']
    controller=LowSpeedController(s,0.)
    intent=controller.step(nav,3.3)
    assert intent['action']=='search' and intent['esc_us']==1600
    assert intent['steering_us']==1870


def test_live_short_visible_pair_reaches_driving_pipeline(settings):
    import cv2
    from carvision.cone_road import ConeRoadContext
    from carvision.cone_vision import BlueConeDetector,ImageConeTracker
    s=trial_settings(settings);s['scene']['surface']='indoor_white_line_track'
    image=cv2.imdecode(np.frombuffer((ROOT/'tests/fixtures/cone-track-short-secondary.jpg').read_bytes(),np.uint8),1)
    detections,mask=BlueConeDetector(s).detect(image);road=ConeRoadContext(s)
    context=road.inspect(image,mask);road.enrich(detections,image,context);tracker=ImageConeTracker('secondary',s)
    for i in range(4):tracks=tracker.update(detections,i,3+i*.1)
    data={'frame_id':4,'captured_s':3.3,'road_context':context,'tracks':tracks}
    nav=DualLowSpeedNavigator(s).update_views({'secondary':image},{'secondary':data},3.3)
    assert nav['ready'] and not nav.get('boundary_danger'),nav
    result=LowSpeedController(s,0.).step(nav,3.3)
    assert result['action']=='search' and result['esc_us']==1600


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
        assert intent['steering_us']==(1870 if with_cone else 1610)
    assert ctl.powered_s>8.


def test_cruising_updates_steering_when_cone_appears_without_braking(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    assert ctl.step(observation(3.1,cones=[]),3.1)['steering_us']==1610
    result=ctl.step(observation(3.2),3.2)
    assert result['action']=='search' and result['steering_us']==1870


def test_cone_turning_and_empty_lane_use_the_same_selected_endpoints(settings):
    ctl=LowSpeedController(trial_settings(settings,{'test_left_us':1730,'test_right_us':1560}),0.)
    assert ctl.step(observation(3.1,cones=[],ego_fraction=.55),3.1)['steering_us']==1730
    assert ctl.step(observation(3.2),3.2)['steering_us']==1730
    ctl=LowSpeedController(trial_settings(settings,{'test_left_us':1730,'test_right_us':1560}),0.)
    data=observation(3.1);data['cones'][0]['fraction']=.34
    assert ctl.step(data,3.1)['steering_us']==1560


def test_near_cone_inside_turn_buffer_can_move_away_when_body_clear(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    data=observation(3.1,ego_fraction=.438,half_envelope=.203)
    data['cones'][0].update(fraction=.663,half_width=.034,y_fraction=.714,image_close=True)
    intent=ctl.step(data,3.1)
    assert intent['action']=='search' and intent['steering_us']==1870


def test_near_cone_inside_body_clearance_still_stops(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    data=observation(3.1,ego_fraction=.438,half_envelope=.203)
    data['cones'][0].update(fraction=.61,half_width=.034,y_fraction=.714,image_close=True)
    with pytest.raises(ValueError,match='净空'):ctl.step(data,3.1)


def test_tracked_approaching_cone_uses_forward_escape_instead_of_near_stop(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    for i,y in enumerate((.55,.63,.71,.74)):
        t=3.1+i*.1;data=observation(t)
        data['cones'][0].update(y_fraction=y,near=y>=.60,image_close=y>=.70,exit_band=y>=.70)
        intent=ctl.step(data,t)
        assert intent['action']=='search' and intent['esc_us']==1600 and intent['steering_us']==1870
    assert ctl.possible_contact and ctl.escape_turn=='left'
    assert not ctl.progress()['field_verified']


def test_same_cone_clipped_below_ground_roi_keeps_moving_until_estimated_pass(settings):
    s=trial_settings(settings);navigator=LowSpeedNavigator(s);controller=LowSpeedController(s,0.)
    image=np.zeros((360,480,3),np.uint8)
    for i,y in enumerate((210,244,266,279)):
        t=3.1+i*.1;data=row(t,i);data['tracks'][0].update(foot_pixel=[290,y],bbox_xyxy=[280,y-30,300,y+5])
        nav=navigator.update(image,data,t)
        assert controller.step(nav,t)['action']=='search'
    for i in range(4,17):
        t=3.1+i*.1;data=row(t,i)
        data['tracks'][0].update(foot_pixel=[298,344],bbox_xyxy=[280,300,320,360],clipped=True,ground_contact_usable=False)
        nav=navigator.update(image,data,t)
        assert nav['ready'] and not nav['uncertain']
        assert nav['cones'][0]['near_field_continuation'] and nav['cones'][0]['reference_distance_m'] is None
        assert controller.step(nav,t)['action']=='search'
    assert controller.count==1 and controller.auto_count==1


def test_static_bonnet_blob_does_not_gain_near_field_continuation(settings):
    nav=LowSpeedNavigator(trial_settings(settings));image=np.zeros((360,480,3),np.uint8)
    for i in range(4):
        data=row(1.+i*.1,i);data['tracks'][0].update(foot_pixel=[290,270],bbox_xyxy=[280,240,300,275])
        nav.update(image,data,1.+i*.1)
    data=row(1.4,4);data['tracks'][0].update(foot_pixel=[290,344],bbox_xyxy=[280,300,320,360],clipped=True,ground_contact_usable=False)
    result=nav.update(image,data,1.4)
    assert not any(c.get('near_field_continuation') for c in result['cones'])


def test_escape_does_not_override_white_boundary_hard_stop(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    for i,y in enumerate((.55,.63,.71)):
        t=3.1+i*.1;data=observation(t);data['cones'][0].update(y_fraction=y,image_close=y>=.7,exit_band=y>=.7)
        ctl.step(data,t)
    data=observation(3.5,boundary_danger=True,reason='车身白线净空不足')
    with pytest.raises(ValueError,match='白线'):ctl.step(data,3.5)


def test_second_cone_can_use_same_side_when_alternation_has_no_lane_space(settings):
    ctl=LowSpeedController(trial_settings(settings),0.);ctl.count=1;ctl.first_side='left'
    data=observation(3.1);data['cones'][0].update(id='second',fraction=.90,near=False,y_fraction=.55)
    intent=ctl.step(data,3.1)
    assert intent['action']=='search' and ctl.side=='left'
    assert .23<=ctl.target_fraction<=.77


@pytest.mark.parametrize('ego,pulse,direction',[(.22,1350,'right'),(.78,1870,'left')])
def test_soft_white_boundary_steers_inward_without_braking(settings,ego,pulse,direction):
    ctl=LowSpeedController(trial_settings(settings),0.)
    for t in (3.1,3.2,3.3):
        intent=ctl.step(observation(t,cones=[],ego_fraction=ego,ego_range=[ego-.01,ego+.01]),t)
        assert intent['action']=='search' and intent['esc_us']==1600
        assert intent['steering_us']==pulse and intent['boundary_correction']==direction


def test_white_boundary_correction_hysteresis_avoids_alternating_at_threshold(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    for t,ego in ((3.1,.25),(3.2,.27),(3.3,.28)):
        assert ctl.step(observation(t,cones=[],ego_fraction=ego),t)['steering_us']==1350
    assert ctl.step(observation(3.4,cones=[],ego_fraction=.5),3.4)['steering_us']==1610
    assert ctl.boundary_turn is None


@pytest.mark.parametrize('ego',[.13,.87])
def test_white_boundary_body_clearance_still_stops(settings,ego):
    ctl=LowSpeedController(trial_settings(settings),0.)
    with pytest.raises(ValueError,match='净空不足'):
        ctl.step(observation(3.1,cones=[],ego_fraction=ego),3.1)


def test_inward_correction_cannot_turn_into_nearby_cone(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    data=observation(3.1,ego_fraction=.23)
    data['cones'][0].update(fraction=.43,y_fraction=.71,image_close=True)
    with pytest.raises(ValueError,match='净空'):ctl.step(data,3.1)


def test_soft_boundary_navigator_keeps_full_corridor_and_cone_data(settings):
    data=row();data['road_context']['left_inner']=[[190.,y] for y in np.linspace(180,260,12)]
    nav=LowSpeedNavigator(trial_settings(settings)).update(np.zeros((360,480,3),np.uint8),data,1.)
    assert nav['ready'] and nav['boundary_correction']=='right'
    assert nav['lane_bounds_px'] and nav['cones'] and not nav.get('boundary_danger')


def test_soft_boundary_seen_by_either_camera_requests_correction_not_veto(settings):
    data=row();data['road_context']['left_inner']=[[190.,y] for y in np.linspace(180,260,12)]
    image=np.zeros((360,480,3),np.uint8)
    nav=DualLowSpeedNavigator(trial_settings(settings)).update_views(
        {'primary':image,'secondary':image},{'primary':row(),'secondary':data},1.)
    assert nav['ready'] and nav['boundary_correction']=='right' and not nav.get('boundary_danger')


def test_two_cones_automatically_link_only_after_ordered_exit_evidence(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    def seen(t,key,x,y,ego):
        cone={'id':key,'fraction':x,'half_width':.03,'near':y>=.60,
              'y_fraction':y,'image_close':y>=.70,'exit_band':y>=.70,'reference_distance_m':None}
        return observation(t,cones=[cone],ego_fraction=ego,source_camera='secondary')
    # First cone: approach, move to its left, then three distinct exit frames.
    for i,y in enumerate((.55,.63,.71,.73,.75)):
        t=3.1+i*.1
        assert ctl.step(seen(t,'first',.70,y,.38),t)['action']=='search'
    for t in np.arange(3.6,4.9,.1):
        assert ctl.step(observation(float(t),cones=[],ego_fraction=.38,source_camera='secondary'),float(t))['action']=='search'
    assert ctl.count==1 and ctl.auto_count==1 and ctl.manual_count==0
    assert ctl.phase=='driving' and 'first' in ctl.passed_ids
    # Second cone: alternate side, keep moving, then clear its rear.
    for i,y in enumerate((.55,.63,.71,.73,.75)):
        t=5.+i*.1
        assert ctl.step(seen(t,'second',.30,y,.62),t)['action']=='search'
    assert ctl.side=='right'
    for t in np.arange(5.5,6.8,.1):
        ctl.step(observation(float(t),cones=[],ego_fraction=.62,source_camera='secondary'),float(t))
    assert ctl.count==2 and ctl.auto_count==2 and ctl.phase=='driving'
    intents=[]
    for t in np.arange(6.8,8.2,.1):
        intents.append(ctl.step(observation(float(t),cones=[],ego_fraction=.5,source_camera='secondary'),float(t)))
    assert any(i['action']=='brake' for i in intents)
    assert ctl.phase=='complete' and intents[-1]['action']=='neutral'
    assert ctl.progress()['pass_estimate_unverified'] and not ctl.progress()['field_verified']


def test_repeated_frame_cannot_build_automatic_pass_evidence(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    data=observation(3.1,ego_fraction=.38);data['cones'][0].update(fraction=.70,y_fraction=.55,exit_band=False)
    ctl.step(data,3.1)
    data=observation(3.2,ego_fraction=.38);data['cones'][0].update(fraction=.70,y_fraction=.74,exit_band=True)
    for t in (3.2,3.21,3.22):ctl.step(data,t)
    assert ctl.pass_gate_hits==1


def test_reappearing_target_resets_tail_continuation(settings):
    ctl=LowSpeedController(trial_settings(settings),0.)
    ctl.step(observation(3.1),3.1)
    ctl.step(observation(3.2,cones=[]),3.2)
    ctl.step(observation(3.4,cones=[]),3.4)
    assert ctl.tail_powered>0
    ctl.step(observation(3.5),3.5)
    assert ctl.tail_powered==0 and not ctl.review_pending and ctl.count==0


@pytest.mark.parametrize('after',['review','complete'])
def test_end_brake_cannot_resume_driving_when_lines_disappear(settings,after):
    ctl=LowSpeedController(trial_settings(settings),0.)
    ctl.step(observation(3.1,cones=[]),3.1);ctl._brake(3.2,'结束本段',after=after)
    for t in (3.25,3.4,4.,4.2):
        lost=observation(t,cones=[]);lost.pop('lane_bounds_px')
        assert ctl.step(lost,t)['action']!='search'
    assert ctl.phase==after
    assert ctl.step(observation(4.3),4.3)['action']=='neutral'


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
render('page',true,false);assert(!env.el('cone-test-start').disabled);
assert(env.el('cone-test-confirm').disabled);render('page',false,true);assert(!env.el('cone-test-confirm').disabled);
render('other',true,true);assert(env.el('cone-test-start').disabled);assert(env.el('cone-test-confirm').disabled);
assert(env.el('cone-white-save').disabled);
const thresholdSettings={scene:{white_v_min:210},driving:{},blue_hsv_lower:[100,110,50],blue_hsv_upper:[135,255,255],confirm_frames:3};
env.renderCone({cone_trial:{observing:true,settings:thresholdSettings,avoidance:{active:false},short_test:{active:false},readiness:{missing:[]}}});
assert(!env.el('cone-white-save').disabled);assert.equal(env.el('cone-white-v').value,210);
vm.runInContext('coneWhiteDirty=true',env);env.el('cone-white-v').value=195;
env.renderCone({cone_trial:{observing:true,settings:thresholdSettings,avoidance:{active:false},readiness:{missing:[]}}});
assert.equal(env.el('cone-white-v').value,195);
console.log('Experimental controls preserve ownership; no motion sent');
'''
    result=subprocess.run([node,'-e',js],input=json.dumps({'source':source}),text=True,encoding='utf-8',capture_output=True,timeout=10)
    assert result.returncode==0,result.stdout+result.stderr
