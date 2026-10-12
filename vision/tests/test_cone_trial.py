"""Observation and geometry regression tests; numeric worlds are test fixtures only."""
import copy
import json
import os
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.cone_vision import BlueConeDetector, ImageConeTracker
from carvision.cone_planning import (load_settings, readiness, ConeRoutePlanner,
    ConePassCounter, ConeMission, path_clear, smooth_path)
from carvision.cone_observer import ConeCamera, ConeObserverDrive
from carvision.cone_controls import extend_controls, add_preview_endpoint

ROOT=Path(__file__).parents[1]
CONFIG=ROOT/'configs/cone-trial.json'


@pytest.fixture
def settings():return load_settings(CONFIG)


@pytest.fixture
def measured(settings):
    # These numbers define a synthetic unit-test world, not this vehicle.
    s=copy.deepcopy(settings)
    s['geometry'].update(measured=True,lane_width_m=2.,vehicle_width_m=.24,vehicle_length_m=.42,
        rear_axle_to_front_m=.32,rear_overhang_m=.1,cone_height_m=.2,cone_base_width_m=.16,clearance_m=.05)
    s['ground'].update(verified=True,homography=[[.01,0,-2.4],[0,-.01,3.6],[0,0,1]],
        valid_polygon_px=[[0,0],[479,0],[479,359],[0,359]],max_error_m=.01)
    s['motion'].update(verified=True,minimum_turn_radius_m=.4,maximum_tracking_error_m=.02,
        braking_distance_m=.15,pose_feedback_verified=True)
    return s


def picture(name):return cv2.imdecode(np.frombuffer((ROOT/'tests/fixtures'/name).read_bytes(),np.uint8),1)


@pytest.mark.parametrize('name',['cone-primary-a.jpg','cone-primary-b.jpg','cone-secondary-a.jpg',
    'cone-secondary-b.jpg','cone-right-reflection-a.jpg','cone-right-reflection-b.jpg',
    'cone-right-rounded-tip.jpg','cone-left-with-small-background-patch.jpg'])
def test_real_blue_cone_fixtures_have_one_cone_and_blue_walls_are_unresolved(settings,name):
    rows,_=BlueConeDetector(settings).detect(picture(name))
    cones=[r for r in rows if r['kind']=='cone_candidate']
    assert len(cones)==1
    assert cones[0]['foot_pixel'][1]>280
    assert any(r['kind']=='unresolved_blue' for r in rows)


@pytest.mark.parametrize('camera',['primary','secondary'])
@pytest.mark.parametrize('sample',['a','b'])
def test_real_empty_scene_does_not_promote_wall_or_small_background_patch(settings,camera,sample):
    rows,_=BlueConeDetector(settings).detect(picture(f'cone-removed-{camera}-{sample}.jpg'))
    assert all(r['kind']!='cone_candidate' for r in rows)


@pytest.mark.parametrize('clipped',[False,True])
def test_close_cone_is_preserved_and_red_is_excluded(settings,clipped):
    image=np.zeros((360,480,3),np.uint8);bottom=359 if clipped else 345
    points=np.array([[240,100],[150,bottom],[330,bottom]])
    cv2.fillPoly(image,[points],(255,0,0))
    rows,_=BlueConeDetector(settings).detect(image)
    assert len(rows)==1 and rows[0]['area_fraction']>.025
    assert rows[0]['kind']==('unresolved_blue' if clipped else 'cone_candidate')
    assert rows[0]['ground_contact_usable'] is (not clipped)
    image[:]=0;cv2.fillPoly(image,[points],(0,0,255))
    assert BlueConeDetector(settings).detect(image)[0]==[]


def detection():
    return {'kind':'cone_candidate','bbox_xyxy':[200,200,240,260],'foot_pixel':[220,255],
        'ground_contact_usable':True,'clipped':False}


def test_tracker_does_not_count_duplicates_disappearance_or_camera_twice(settings):
    tracker=ImageConeTracker('secondary',settings)
    for i in range(3):rows=tracker.update([detection()],i,1+i*.1)
    assert rows[0]['confirmed'] and rows[0]['id']=='secondary-1'
    with pytest.raises(ValueError):tracker.update([detection()],2,1.2)
    clipped={**detection(),'kind':'unresolved_blue','clipped':True,'ground_contact_usable':False}
    row=tracker.update([clipped],3,1.3)[0]
    assert row['id']=='secondary-1' and row['confirmed'] and not row['ground_contact_usable']
    missing=tracker.update([],4,1.4)[0]
    assert not missing['visible'] and 'passed' not in missing
    assert ImageConeTracker('primary',settings).update([detection()],0,1)[0]['id']=='primary-1'
    rows=tracker.update([detection()],5,2.)
    assert not rows[0]['confirmed'] and rows[0]['id']!='secondary-1'


def cone(id,x,y):return {'id':id,'x_m':x,'y_m':y,'radius_m':.08}


def test_missing_measurements_cannot_create_route(settings):
    assert not readiness(settings)['metric_planning_ready']
    assert ConeRoutePlanner(settings).plan([],lane_bounds=(-1,1),coverage_forward_m=5)['action']=='stop'
    assert ConeMission(settings).step({},1)['reason']=='needs_measured_configuration'


@pytest.mark.parametrize('positions',[(0.,0.),(-.5,-.5),(.5,.5),(-.4,.4)])
def test_two_cones_checked_by_whole_vehicle_and_path_curvature(measured,positions):
    obstacles=[cone('1',positions[0],1.8),cone('2',positions[1],3.)]
    result=ConeRoutePlanner(measured).plan(obstacles,lane_bounds=(-1,1),coverage_forward_m=6)
    assert result['action']=='follow_candidate_path'
    assert not result['hardware_output'] and not result['field_collision_clearance_verified']
    assert path_clear(result['path'],obstacles,measured,(-1,1))


@pytest.mark.parametrize('case',['narrow','near','unseen_exit','unknown_lane','bad_cone','repeated_id'])
def test_unreachable_or_uncertain_geometry_has_no_forward_path(measured,case):
    obstacles=[cone('1',0,1.8)];bounds=(-1,1);coverage=6;fresh=True
    if case=='narrow':bounds=(-.2,.2)
    if case=='near':obstacles=[cone('1',0,.3)]
    if case=='unseen_exit':coverage=2
    if case=='unknown_lane':fresh=False
    if case=='bad_cone':obstacles[0]['x_m']=float('nan')
    if case=='repeated_id':obstacles.append(cone('1',.5,3))
    assert ConeRoutePlanner(measured).plan(obstacles,lane_bounds=bounds,lane_fresh=fresh,coverage_forward_m=coverage)['action']=='stop'


def test_front_corner_and_turning_radius_are_checked(measured):
    assert not path_clear([{'x_m':.65,'y_m':0,'yaw_rad':.8,'curvature':0}],[],measured,(-1,1))
    assert not path_clear(smooth_path([(0,0),(.1,.4)]),[],measured,(-1,1))


def pose(y=0):return {'valid':True,'x_m':0.,'y_m':y,'yaw_rad':0.,'error_bound_m':.02}


def test_pass_counter_requires_whole_vehicle_after_initially_seen_ahead(measured):
    counter=ConePassCounter(measured)
    assert not counter.update([cone('1',.5,1),cone('2',.5,2)],pose())['complete']
    assert counter.update([],pose())['passed_ids']==[]  # disappearance alone proves nothing
    assert counter.update([],pose(1.1))['passed_ids']==[]  # rear of car has not passed
    assert counter.update([],pose(1.4))['passed_ids']==['1']
    assert counter.update([],pose(2.4))['complete']
    assert not ConePassCounter(measured).update([cone('fake',.5,-1)],pose())['complete']


def test_reacquired_camera_id_cannot_count_same_world_cone_twice(measured):
    counter=ConePassCounter(measured)
    counter.update([cone('secondary-1',.5,1),cone('primary-7',.5,1.01)],pose())
    result=counter.update([],pose(2))
    assert len(result['passed_ids'])==1 and not result['complete']


def test_uncertain_pose_and_extra_obstacles_cannot_finish_stage(measured):
    counter=ConePassCounter(measured)
    result=counter.update([cone('1',0,1)],{**pose(),'error_bound_m':.3})
    assert result['reason']=='pose_unavailable' and not counter.world
    counter.update([cone('1',0,1),cone('2',.5,2),cone('3',-.5,2)],pose())
    result=counter.update([],pose(4))
    assert result['reason']=='ambiguous_obstacle_count' and not result['complete']


def test_mission_stale_camera_fault_and_missing_pose_do_not_advance(measured):
    mission=ConeMission(measured)
    sample={'frame_id':1,'captured_s':1,'lane_bounds_m':[-1,1],'lane_valid':True,'coverage_forward_m':6,
        'observed_world':[],'pose':pose()}
    assert mission.step(sample,1)['phase']=='waiting_for_two_cones'
    assert mission.step(sample,1.01)['phase']=='fault'
    sample.update(frame_id=2,captured_s=1.1)
    assert mission.step(sample,1.1)['action']=='stop'  # fault latches
    assert ConeMission(measured).step({**sample,'pose':None},1.1)['action']=='stop'


def test_return_targets_lane_center_in_vehicle_coordinates(measured):
    route=ConeRoutePlanner(measured).plan([],lane_bounds=(-1.3,.7),coverage_forward_m=6)
    assert route['action']=='follow_candidate_path'
    assert route['path'][-1]['x_m']==pytest.approx(-.3)


def camera_state(now=1,fid=1):
    image=picture('cone-secondary-a.jpg');_,jpg=cv2.imencode('.jpg',image)
    return SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),raw_frame_id=fid,raw_received_ms=now*1000,state='ok')


def test_real_cache_reader_keeps_sources_distinct_and_expires(settings):
    now=[1.];states={'primary':camera_state(fid=42),'secondary':camera_state(fid=3)}
    reader=ConeCamera(states,settings,lambda:now[0])
    result=reader.read()
    assert result['cameras']['primary']['frame_id']==42 and result['cameras']['secondary']['frame_id']==3
    assert cv2.imdecode(np.frombuffer(reader.preview,np.uint8),1).shape==(360,960,3)
    assert result['physical_cones_passed'] is None and not result['hardware_output']
    assert reader.read() is None
    now[0]=1.3
    with pytest.raises(ValueError,match='新鲜'):reader.read()


class Driver:
    def __init__(self):self.calls=[]
    def status(self):return {'mode':'disabled','hardware_output':False}
    def request(self,action,payload):self.calls.append((action,payload))
    def close(self):pass


def test_observation_has_no_actuator_and_settings_cannot_change_motion(tmp_path):
    config=tmp_path/'cone.json';config.write_bytes(CONFIG.read_bytes())
    base=Driver();drive=ConeObserverDrive(base,{},config,tmp_path)
    assert drive.status()['cone_trial']['can_observe'] and not drive.status()['cone_trial']['can_drive']
    drive.request('cone_settings',{'blue_hsv_lower':[101,110,50],'motion':{'verified':True}})
    assert drive.settings['motion']['verified'] is False and not base.calls
    assert ConeObserverDrive(base,{},config,tmp_path).settings['blue_hsv_lower'][0]==101
    with pytest.raises(ValueError):drive.request('cone_start',{})
    with pytest.raises(ValueError):drive.request('cone_settings',{'blue_hsv_lower':[170,110,50]})
    drive.request('command',{'motor':'forward'})
    assert base.calls==[('command',{'motor':'forward'})]
    drive.close()


def test_observer_initialization_failure_exits_cleanly(tmp_path,monkeypatch):
    drive=ConeObserverDrive(Driver(),{},CONFIG,tmp_path)
    def fail(*a):raise ValueError('reader unavailable')
    monkeypatch.setattr('carvision.cone_observer.ConeCamera',fail)
    drive.observing=True;drive._work()
    assert not drive.observing and drive.reason=='reader unavailable'


def test_field_dimensions_save_in_centimeters_without_enabling_drive(tmp_path):
    config=tmp_path/'cone.json';config.write_bytes(CONFIG.read_bytes())
    drive=ConeObserverDrive(Driver(),{},config,tmp_path)
    dimensions={'lane_width':200,'vehicle_width':24,'vehicle_length':42,
        'rear_axle_to_front':32,'rear_overhang':10,'cone_height':20,'cone_base_width':16,'clearance':5}
    result=drive.request('cone_settings',{'geometry_cm':dimensions})['cone_trial']
    assert result['settings']['geometry']['vehicle_width_m']==.24
    assert result['settings']['geometry']['measured'] and not result['can_drive']
    assert not result['settings']['ground']['verified'] and not result['settings']['motion']['verified']
    assert not result['readiness']['metric_planning_ready']
    restored=ConeObserverDrive(Driver(),{},config,tmp_path)
    assert restored.settings['geometry']==drive.settings['geometry']
    restored.request('cone_settings',{'geometry_cm':{'cone_height':None}})
    assert restored.settings['geometry']['cone_height_m'] is None
    assert not restored.settings['geometry']['measured']


def test_legacy_reference_and_lane_width_persist_without_actuator(tmp_path):
    config=tmp_path/'cone.json';config.write_bytes(CONFIG.read_bytes())
    (tmp_path/'cone-legacy-ground-candidate.json').write_bytes((CONFIG.parent/'cone-legacy-ground-candidate.json').read_bytes())
    base=Driver();drive=ConeObserverDrive(base,{},config,tmp_path)
    drive.request('cone_settings',{'geometry_cm':{'lane_width':125}})
    result=drive.request('cone_reuse_legacy',{})['cone_trial']
    assert result['settings']['geometry']['lane_width_m']==1.25
    assert result['settings']['ground']['homography'] is not None
    assert not result['can_drive'] and not result['hardware_output'] and not base.calls
    restored=ConeObserverDrive(base,{},config,tmp_path)
    assert restored.settings['ground']==drive.settings['ground']
    drive.observing=True
    with pytest.raises(ValueError):drive.request('cone_reuse_legacy',{})
    drive.observing=False;drive.close();restored.close()


@pytest.mark.parametrize('geometry',[{'vehicle_width':-5},{'vehicle_width':float('nan')},
    {'vehicle_width':True},{'vehicle_length':40,'rear_axle_to_front':35,'rear_overhang':20},
    {'verified':True}])
def test_invalid_dimensions_are_not_written(tmp_path,geometry):
    config=tmp_path/'cone.json';config.write_bytes(CONFIG.read_bytes())
    drive=ConeObserverDrive(Driver(),{},config,tmp_path)
    with pytest.raises(ValueError):drive.request('cone_settings',{'geometry_cm':geometry})
    assert not drive.local_config.exists()


def test_ui_composition_preserves_original_handlers():
    from carvision.brake_controls import extend_controls as brake
    from carvision.traffic_controls import extend_controls as traffic
    control=SimpleNamespace(HTML='existing',SCRIPT='''<script>(()=>{let gamepadControl=null,pwmControl=null;
function render(s){legacy(s);}async function heartbeat(){original();}})();</script>''')
    brake(control);traffic(control);extend_controls(control)
    assert control.HTML.index('id="crosswalk-test"')<control.HTML.index('id="traffic-test"')<control.HTML.index('id="cone-test"')
    assert 'original();' in control.SCRIPT and 'legacy(s);' in control.SCRIPT
    assert 'cone_observe' in control.SCRIPT and 'traffic_start' in control.SCRIPT and 'crosswalk_start' in control.SCRIPT


def test_cone_preview_expiry_and_delegation(tmp_path):
    now=[1.];drive=ConeObserverDrive(Driver(),{},CONFIG,tmp_path,lambda:now[0])
    class Base:
        def send_body(self,body,mime,status=200):self.reply=(body,mime,status)
        def do_GET(self):self.reply='existing'
    server=SimpleNamespace(RequestHandlerClass=Base);add_preview_endpoint(server,drive)
    handler=server.RequestHandlerClass();handler.path='/api/cone/frame.jpg'
    drive.preview=b'jpeg';drive.updated=1
    handler.do_GET();assert handler.reply[2]==503
    drive.observing=True;handler.do_GET();assert handler.reply[2]==200
    now[0]=1.3;handler.do_GET();assert handler.reply[2]==503
    handler.path='/api/traffic/frame.jpg';handler.do_GET();assert handler.reply=='existing'


def test_pc_observer_saves_real_images_and_deduplicates_without_control(tmp_path):
    import importlib.util
    spec=importlib.util.spec_from_file_location('cone_cli_test',ROOT/'tools/cone_trial.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    jpg=(ROOT/'tests/fixtures/cone-secondary-a.jpg').read_bytes()
    with (tmp_path/'log.jsonl').open('w',encoding='utf-8') as log:
        sink=module.LiveSink(load_settings(CONFIG),log)
        result=sink.publish('secondary',jpg,{'status_is_exact_jpeg_timestamp':False})
        assert not result['usable_for_vehicle_control'] and not result['hardware_output']
        assert result['physical_cones_passed'] is None
        assert sink.publish('secondary',jpg,{}) is None and sink.count==1
        sink.save(tmp_path)
        assert len(list(tmp_path.glob('*.jpg')))==2
        sink.closed=True
    # An outstanding reader callback after close must not touch the log.
    assert sink.publish('primary',jpg,{}) is None
    sink.error('primary','late reader error')


@pytest.mark.parametrize('view',['front','side','top','oblique'])
def test_user_actual_8cm_cone_all_views(settings,view):
    rows,_=BlueConeDetector(settings).detect(picture('cone-user-'+view+'.jpg'))
    cones=[r for r in rows if r['kind']=='cone_candidate']
    assert len(cones)==1
    if view=='top':
        assert cones[0]['shape_evidence']=='round_top_view'
        assert not cones[0]['ground_contact_usable']


def red_track_image(color=(30,35,150),right=True):
    image=np.full((360,480,3),color,np.uint8)
    cv2.line(image,(150,150),(70,359),(255,255,255),10)
    if right:cv2.line(image,(330,150),(410,359),(255,255,255),10)
    return image


def test_white_inner_edges_exclude_paint_and_red_floor_supports_scene(settings):
    from carvision.cone_road import ConeRoadContext
    image=red_track_image();mask=np.zeros(image.shape[:2],np.uint8)
    context=ConeRoadContext(settings).inspect(image,mask)
    assert context['valid'] and context['red_ground_fraction']>.9
    for x,y in context['left_inner']:
        center=150+(70-150)*(y-150)/(359-150)
        assert x>center+4
    for x,y in context['right_inner']:
        center=330+(410-330)*(y-150)/(359-150)
        assert x<center-4
    rows=[{**detection(),'foot_pixel':[240,280]}, {**detection(),'foot_pixel':[20,280]}]
    ConeRoadContext.enrich(rows,image,context)
    assert rows[0]['boundary_relation']=='inside' and rows[0]['scene_supported']
    assert rows[1]['boundary_relation']=='outside' and not rows[1]['scene_supported']


@pytest.mark.parametrize('case',['one_line','green_floor'])
def test_missing_white_edge_or_wrong_surface_does_not_claim_valid_track(settings,case):
    from carvision.cone_road import ConeRoadContext
    image=red_track_image(color=(25,140,30) if case=='green_floor' else (30,35,150),right=case!='one_line')
    context=ConeRoadContext(settings).inspect(image,np.zeros(image.shape[:2],np.uint8))
    assert not context['valid']


def test_actual_known_dimensions_are_preloaded_and_boundary_width_is_enforced(settings,measured):
    # Deployment preserves the car's base/local configs. Validate repository
    # defaults against the supplied source sample, not an older live base file.
    source=os.environ.get('CARVISION_REPOSITORY_CONE_CONFIG')
    g=(load_settings(source) if source else settings)['geometry']
    assert (g['lane_width_m'],g['vehicle_width_m'],g['vehicle_length_m'],g['cone_height_m'],g['cone_base_width_m'])==(1.23,.295,.42,.08,.075)
    assert g['rear_axle_to_front_m']==.335 and g['rear_overhang_m']==.085 and not g['measured']
    measured['geometry']['lane_width_m']=1.23
    result=ConeRoutePlanner(measured).plan([],lane_bounds=(-1,1),coverage_forward_m=5)
    assert result['action']=='stop' and result['reason']=='observed_corridor_exceeds_measured_lane_width'


@pytest.mark.parametrize('surface',['red_running_track','indoor_white_line_track'])
@pytest.mark.parametrize('right',[False,True])
def test_indoor_surface_is_explicit_and_still_needs_both_white_edges(settings,surface,right):
    from carvision.cone_road import ConeRoadContext
    settings['scene']['surface']=surface
    im=red_track_image(color=(25,140,30),right=right)
    context=ConeRoadContext(settings).inspect(im,np.zeros(im.shape[:2],np.uint8))
    assert context['valid'] is (surface=='indoor_white_line_track' and right)
    assert not context['red_ground_supported']
    if context['valid']:
        rows=[{**detection(),'foot_pixel':[240,240]}]
        ConeRoadContext.enrich(rows,im,context)
        assert rows[0]['scene_supported'] and rows[0]['boundary_relation']=='inside'


def test_indoor_fit_uses_visible_ground_above_bonnet(settings):
    from carvision.cone_road import ConeRoadContext
    settings['scene']['surface']='indoor_white_line_track'
    im=np.full((360,480,3),(45,105,45),np.uint8)
    cv2.line(im,(190,160),(-40,290),(255,255,255),8)
    cv2.line(im,(290,160),(520,290),(255,255,255),8)
    context=ConeRoadContext(settings).inspect(im,np.zeros(im.shape[:2],np.uint8))
    assert context['valid'] and len(context['left_inner'])>=6
    assert max(p[1] for p in context['left_inner'])<290


def test_surface_setting_does_not_verify_calibration_or_open_actuator(tmp_path):
    config=tmp_path/'cone-trial.json';config.write_bytes(CONFIG.read_bytes())
    base=Driver();drive=ConeObserverDrive(base,{},config,tmp_path)
    before=copy.deepcopy(drive.settings)
    drive.request('cone_settings',{'scene_surface':'indoor_white_line_track'})
    for key in ('geometry','ground','motion','driving'):
        assert drive.settings[key]==before[key]
    assert not base.calls and not drive.status()['cone_trial']['can_drive']
    assert ConeObserverDrive(base,{},config,tmp_path).settings['scene']['surface']=='indoor_white_line_track'
    with pytest.raises(ValueError):
        drive.request('cone_settings',{'scene_surface':'ignore_boundaries'})
    drive.observing=True
    with pytest.raises(ValueError):
        drive.request('cone_settings',{'scene_surface':'red_running_track'})
