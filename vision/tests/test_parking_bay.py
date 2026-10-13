import copy
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.parking_bay import load_settings,validate,readiness,detect_parking
from carvision.parking_bay_console import BaySelection,ParkingBayDrive
from carvision.parking_bay_controls import extend_controls
from carvision.parking_mission import ParkingMission,plan_entry,contained

ROOT=Path(__file__).resolve().parents[2]


def settings():return load_settings(ROOT/'vision/configs/parking-bay.json')


def calibrated():
    s=settings();s.update(perception_verified=True,minimum_turn_radius_m=.4,braking_distance_m=.05,maximum_pose_error_m=.01)
    s['ground']={'verified':True,'image_size':[480,360],'homography':[[1/200,0,-1.2],[0,-1/140,2.5],[0,0,1]],
                 'valid_polygon_px':[[3,3],[476,3],[476,356],[3,356]]}
    return s


def scene(blocked='left',occlude_line=False):
    im=np.full((360,480,3),(45,45,170),np.uint8)
    cv2.line(im,(117,5),(117,354),(220,220,220),5);cv2.line(im,(363,5),(363,354),(220,220,220),5)
    for y in (70,210):cv2.line(im,(117,y),(363,y),(0,240,240),5)
    cv2.line(im,(240,70),(240,210),(0,240,240),5)
    if blocked:
        x=124 if blocked=='left' else 247
        cv2.rectangle(im,(x,173),(x+108,228 if occlude_line else 206),(230,40,20),-1)
    return im


@pytest.mark.parametrize('blocked',['left','right'])
def test_real_pixels_find_two_side_by_side_regions_on_red_ground(blocked):
    obs,_=detect_parking(scene(blocked),settings())
    assert obs['layout_found'] and len(obs['slots'])==2
    states={r['id']:r['availability'] for r in obs['slots']}
    assert states[blocked]=='blocked' and states['right' if blocked=='left' else 'left']=='clear'
    assert not obs['metric_geometry'] and all(not r['occupancy_verified'] for r in obs['slots'])
    json.dumps(obs,allow_nan=False)


def test_metric_template_can_recover_entry_line_occluded_by_board():
    obs,_=detect_parking(scene('left',True),calibrated())
    assert obs['metric_geometry'] and obs['layout_found']
    assert [r['availability'] for r in obs['slots']]==['blocked','clear']
    assert all(r['occupancy_verified'] for r in obs['slots'])


def test_plain_qr_like_white_patch_or_blue_object_is_not_a_parking_layout():
    im=np.full((360,480,3),160,np.uint8);cv2.rectangle(im,(150,110),(280,220),(255,255,255),-1)
    cv2.rectangle(im,(40,100),(100,220),(255,0,0),-1)
    assert not detect_parking(im,settings())[0]['layout_found']


def test_no_board_still_gives_explicit_tentative_default_without_confirmation():
    obs,_=detect_parking(scene(None),settings());sel=BaySelection()
    for i in range(8):result=sel.update(obs,i,i*.1,i*.1)
    assert not result['confirmed'] and result['selected_slot']=='right'
    assert result['tentative'] and result['suggestion_source']=='default_right'


def test_dark_interior_is_unknown_not_clear():
    im=scene('left');cv2.rectangle(im,(247,80),(355,203),(0,0,0),-1)
    obs,_=detect_parking(im,settings())
    assert obs['layout_found'] and obs['slots'][1]['availability']=='unknown'


@pytest.mark.parametrize('flip',[False,True])
def test_actual_pale_tape_green_floor_and_occluded_half(flip):
    path=ROOT/'vision/tests/fixtures/parking-pale-green-left-board-20261013.jpg'
    image=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),1)
    if flip:image=cv2.flip(image,1)
    obs,_=detect_parking(image,settings())
    assert obs['layout_found'] and not obs['layout_complete']
    assert obs['diagnostics']['original_yellow_pixels']<100
    assert obs['diagnostics']['yellow_pixels']>5000
    assert obs['layout_source']=='one_visible_half_with_adjacent_blue_board'
    states={r['id']:r for r in obs['slots']};blocked='right' if flip else 'left';clear='left' if flip else 'right'
    assert states[blocked]['availability']=='blocked' and states[blocked]['corners_px'] is None
    assert states[blocked]['blue_entrance_fraction'] is None
    assert states[clear]['availability']=='clear' and states[clear]['ground_surface']=='indoor_green'
    assert not obs['metric_geometry'] and all(not r['occupancy_verified'] for r in obs['slots'])
    selector=BaySelection()
    for i in range(8):selection=selector.update(obs,i,i*.1,i*.1)
    assert selection['confirmed'] and selection['selected_slot']==clear


def test_actual_green_frame_in_explicit_red_mode_stays_unknown():
    image=cv2.imdecode(np.frombuffer((ROOT/'vision/tests/fixtures/parking-pale-green-left-board-20261013.jpg').read_bytes(),np.uint8),1)
    s=settings();s['surface']='red_running_track';obs,_=detect_parking(image,s)
    assert obs['layout_found'] and obs['slots'][1]['availability']=='unknown'


def test_green_ground_is_supported_without_weakening_dark_obstacle_check():
    im=scene('left');red=(im[:,:,2]>im[:,:,1]*1.5)&(im[:,:,2]>im[:,:,0]*1.5)
    im[red]=(45,170,45)
    obs,_=detect_parking(im,settings())
    assert [s['availability'] for s in obs['slots']]==['blocked','clear']
    cv2.rectangle(im,(247,80),(355,203),(0,0,0),-1)
    obs,_=detect_parking(im,settings())
    assert obs['slots'][1]['availability']=='unknown'


def test_single_yellow_box_without_adjacent_entrance_board_is_not_two_bays():
    im=np.full((360,480,3),(45,170,45),np.uint8)
    cv2.rectangle(im,(180,100),(370,250),(0,240,240),7)
    assert not detect_parking(im,settings())[0]['layout_found']
    # Blue object is behind the far edge, not aligned with the entrance.
    cv2.rectangle(im,(60,20),(150,85),(230,40,20),-1)
    assert not detect_parking(im,settings())[0]['layout_found']


def test_surface_validation_and_layout_reason_reaches_selector():
    assert validate(settings())['surface']=='auto'
    for value in ['auto','red_running_track','indoor_green']:
        assert validate({**settings(),'surface':value})['surface']==value
    with pytest.raises(ValueError):validate({**settings(),'surface':'anything'})
    out=BaySelection().update({'layout_found':False,'slots':[],'reason':'yellow support insufficient'},1,1.,1.)
    assert out['reason']=='yellow support insufficient' and not out['confirmed']


@pytest.mark.parametrize('flip',[False,True])
def test_blue_board_cue_selects_opposite_side_when_layout_matching_drops(flip):
    image=cv2.imdecode(np.frombuffer((ROOT/'vision/tests/fixtures/parking-pale-green-left-board-20261013.jpg').read_bytes(),np.uint8),1)
    if flip:image=cv2.flip(image,1)
    observation,_=detect_parking(image,settings())
    observation.update(layout_found=False,slots=[])
    selector=BaySelection();expected='left' if flip else 'right'
    for i in range(8):
        result=selector.update(observation,i,i*.1,i*.1)
        assert result['selected_slot']==expected and result['tentative'] and not result['confirmed']
        assert not result['autonomous_entry_allowed'] and not result['hardware_output']
    assert result['suggestion_source']=='blue_board_position'


def test_selection_never_disappears_through_layout_flicker_or_stale_data():
    full,_=detect_parking(scene('right'),settings());selector=BaySelection()
    missing={'layout_found':False,'slots':[],'reason':'missing lines'}
    for i in range(8):selector.update(full,i,i*.1,i*.1)
    assert selector.selected=='left'
    result=selector.update(missing,8,.8,.8)
    assert result['selected_slot']=='left' and result['tentative'] and not result['confirmed']
    stale=selector.update(missing,9,.9,2.)
    assert stale['selected_slot']=='left' and stale['stale'] and not stale['confirmed']
    assert selector.snapshot()['selected_slot']=='left'


def test_changed_board_side_can_be_recommended_without_restarting_observer():
    selector=BaySelection();left,_=detect_parking(scene('left'),settings());right,_=detect_parking(scene('right'),settings())
    for i in range(8):selector.update(left,i,i*.1,i*.1)
    result=selector.update(right,8,.8,.8)
    assert result['selected_slot']=='left' and not result['confirmed']
    for i in range(9,16):result=selector.update(right,i,i*.1,i*.1)
    assert result['selected_slot']=='left' and result['confirmed']


def test_blank_and_both_blocked_return_tentative_choice_not_clear_space():
    selector=BaySelection()
    result=selector.update({'layout_found':False,'slots':[]},1,1.,1.)
    assert result['selected_slot']=='right' and result['tentative'] and not result['confirmed']
    both={'layout_found':True,'slots':[{'id':side,'availability':'blocked'} for side in ('left','right')]}
    result=selector.update(both,2,1.1,1.1)
    assert result['selected_slot'] in ('left','right') and not result['confirmed']


def test_clipped_actual_scene_has_direction_and_observed_entry_candidate_without_fake_full_layout():
    path=ROOT/'vision/tests/fixtures/parking-border-clipped-left-board-20261013.jpg'
    image=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),1);obs,_=detect_parking(image,settings())
    assert not obs['layout_found'] and not obs['metric_geometry']
    assert obs['direction_hint']['suggested_slot']=='right'
    assert len(obs['entry_candidates'])==1
    candidate=obs['entry_candidates'][0]
    assert candidate['id']=='right' and candidate['candidate_only'] and candidate['availability']=='clear'
    assert candidate['polygon_m'] is None and not candidate['occupancy_verified']
    result=BaySelection().update(obs,1,1.,1.)
    assert result['selected_slot']=='right' and result['tentative']


def test_observer_status_retains_advisory_when_current_frame_is_hidden_as_stale(tmp_path):
    folder=tmp_path/'vision/configs';folder.mkdir(parents=True)
    (folder/'parking-bay.json').write_text(json.dumps(settings()),encoding='utf-8')
    now=[1.];drive=ParkingBayDrive(Base(),{},tmp_path,clock=lambda:now[0])
    observation,_=detect_parking(scene('right'),settings())
    drive.active=True;drive.updated=.1;drive.latest=observation
    drive.recommendation=drive.selector.update(observation,1,.1,.1)
    p=drive.status()['parking_bay']
    assert p['observation'] is None and p['selection']['selected_slot']=='left'
    assert p['selection']['stale'] and not p['selection']['confirmed']
    assert not p['motor_output_enabled'] and not p['payment_ready']


def test_selection_requires_fresh_sequence_and_never_switches_after_lock():
    left,_=detect_parking(scene('left'),settings());right,_=detect_parking(scene('right'),settings());sel=BaySelection()
    for i in range(7):result=sel.update(left,i,i*.1,i*.1)
    assert result['confirmed'] and result['selected_slot']=='right'
    assert not sel.update(right,8,.8,.8)['confirmed']
    assert not sel.update(left,8,.8,2.)['confirmed']


def test_duplicate_images_do_not_confirm_a_bay():
    obs,_=detect_parking(scene('left'),settings());sel=BaySelection()
    for i in range(8):result=sel.update(obs,1,0.,i*.1)
    assert not result['confirmed'] and sel.selected is None


def slots(blocked='left'):
    rows=[]
    for name,a,b in [('left',-.615,0.),('right',0.,.615)]:
        rows.append({'id':name,'polygon_m':[[a,2.],[b,2.],[b,3.],[a,3.]],
                     'availability':'blocked' if name==blocked else 'clear','occupancy_verified':True})
    return rows


def sample(fid,t,pose=None,blocked='left',speed=0.):
    pose=pose or {'x_m':0.,'y_m':0.,'yaw_rad':0.}
    return {'frame_id':fid,'captured_s':t,'coordinate_frame':'parking_road_world','obstacle_region_verified':True,
            'slots':slots(blocked),'speed_mps':speed,'pose':{**pose,'captured_s':t,'error_bound_m':.01,'verified':True}}


@pytest.mark.parametrize('blocked',['left','right'])
def test_measured_mission_plans_enters_brakes_then_confirms_parked(blocked):
    s=calibrated();m=ParkingMission(s)
    for i in range(7):r=m.step(sample(i,i*.1,blocked=blocked),i*.1)
    assert r['action']=='follow_path' and m.selected==('right' if blocked=='left' else 'left')
    assert not r['hardware_output'] and not r['payment_ready']
    t=.6;fid=6;seen_brake=False
    for pose in m.path[1:]:
        t+=.15;fid+=1;r=m.step(sample(fid,t,pose,blocked,.10),t)
        assert r['phase']!='fault',r
        seen_brake|=r['action']=='brake'
    for _ in range(8):
        t+=.1;fid+=1;r=m.step(sample(fid,t,m.path[-1],blocked,0.),t)
    assert seen_brake and r['phase']=='parked' and r['four_wheels_inside']
    assert r['payment_ready'] and r['payment_amount_cents']==1 and not r['payment_provider_verified']


def test_unknown_measurements_never_generate_motion_intents():
    m=ParkingMission(settings());r=m.step(sample(0,0.),0.)
    assert r['action']=='stop' and r['missing'] and not r['payment_ready']


def test_goal_contains_whole_vehicle_not_just_front_axle():
    s=calibrated();path=plan_entry(slots(),'right',{'x_m':0.,'y_m':0.,'yaw_rad':0.},s)
    assert contained(path[-1],(0.,2.,.615,3.),s,.03)
    assert not contained({'x_m':.3075,'y_m':2.,'yaw_rad':0.},(0.,2.,.615,3.),s,.03)


def test_close_entrance_or_large_braking_distance_rejects_path():
    s=calibrated()
    with pytest.raises(ValueError):plan_entry(slots(),'right',{'x_m':0.,'y_m':1.8,'yaw_rad':0.},s)
    s['braking_distance_m']=.5
    with pytest.raises(ValueError):plan_entry(slots(),'right',{'x_m':0.,'y_m':0.,'yaw_rad':0.},s)


def test_stale_pose_conflicting_bay_and_replayed_frame_stop():
    s=calibrated();m=ParkingMission(s)
    for i in range(7):r=m.step(sample(i,i*.1),i*.1)
    r=m.step(sample(7,.7,blocked='right'),.7);assert r['phase']=='fault' and r['action']=='brake'
    m=ParkingMission(s);x=sample(1,1.);x['pose']['captured_s']=0.;assert m.step(x,1.)['action']=='stop'
    m=ParkingMission(s);m.step(sample(1,1.),1.);assert m.step(sample(1,1.),1.)['phase']=='fault'


def test_two_clear_or_two_blocked_regions_cannot_start():
    for state in ('clear','blocked'):
        m=ParkingMission(calibrated())
        for i in range(7):
            x=sample(i,i*.1)
            for r in x['slots']:r['availability']=state
            r=m.step(x,i*.1)
        assert r['action']=='stop' and m.selected is None


class Base:
    def __init__(self):self.calls=[]
    def status(self):return {'mode':'disabled','setup_token':'boot'}
    def request(self,*args):self.calls.append(args)


def test_parking_entry_console_reserves_output_and_blocks_other_tasks(tmp_path,monkeypatch):
    from carvision.race_flow_console import other_task_active
    from test_parking_entry_run import Actuator,sample as entry_sample
    configs=tmp_path/'vision/configs';configs.mkdir(parents=True)
    for name in ('parking-bay','brake-parking','cone-trial','cone-legacy-ground-candidate'):
        (configs/(name+'.json')).write_bytes((ROOT/'vision/configs'/(name+'.json')).read_bytes())
    base=Base();base.status=lambda:{'mode':'disabled','boot_test_phase':'esc_off','setup_token':'boot',
        'hardware_output':False,'worker_started':False,'pending_hardware_start':False}
    now=[0.];opened=[]
    def factory(s):
        a=Actuator();opened.append(a);return a
    w=ParkingBayDrive(base,{},tmp_path,clock=lambda:now[0],entry_factory=factory)
    monkeypatch.setattr(w,'_start_observation',lambda:None)
    result=w.request('parking_trial_prepare',{'client':'page','setup_token':'boot','esc_off':True,
        'fb_mode_confirmed':True,'selected_slot':'right'})
    assert result['parking_trial']['active'] and not opened
    assert other_task_active(result)
    for action in ('command','cone_prepare','traffic_prepare','parking_payment_begin'):
        with pytest.raises(ValueError,match='占用'):w.request(action,{})
    assert not base.calls
    now[0]=.1;w.entry.tick(entry_sample(.1),.1)
    assert opened and opened[0].commands[-1]['action']=='neutral'
    w.request('emergency',{})
    assert not w.entry.active and opened[0].closed and not base.calls
    assert not other_task_active(w.status())


def test_console_injects_own_fresh_start_selection_not_client_fields(tmp_path,monkeypatch):
    from test_parking_entry_run import Actuator,configurations,sample as entry_sample
    configs=tmp_path/'vision/configs';configs.mkdir(parents=True)
    (configs/'parking-bay.json').write_text(json.dumps(settings()),encoding='utf-8')
    base=Base();base.status=lambda:{'mode':'disabled','boot_test_phase':'esc_off','setup_token':'boot',
        'hardware_output':False,'worker_started':False,'pending_hardware_start':False}
    now=[0.];w=ParkingBayDrive(base,{},tmp_path,clock=lambda:now[0],entry_factory=lambda s:Actuator())
    monkeypatch.setattr(w,'_entry_settings',configurations)
    monkeypatch.setattr(w,'_start_observation',lambda:setattr(w,'active',True))
    w.request('parking_trial_prepare',{'client':'page','setup_token':'boot','esc_off':True,
        'fb_mode_confirmed':True,'selected_slot':'right','suggestion_source':'layout'})
    assert w.entry.s['selection_source']=='default_right'
    now[0]=.1;w.entry.tick(entry_sample(.1),.1)
    w.updated=.15;w.latest={'frame_id':2};w.recommendation={
        'selected_slot':'left','suggestion_source':'blue_board_position',
        'suggestion_basis':'right board observed','confirmed':False}
    now[0]=.2
    status=w.request('parking_trial_start',{'client':'page','run_token':w.entry.token,'trial_sequence':0,
        'esc_on_static':True,'field_ready':True,'fb_mode_confirmed':True,'selected_slot':'right',
        'start_selection':{'selected_slot':'right','suggestion_source':'layout','captured_s':.2}})
    assert status['parking_trial']['selected_slot']=='left' and w.entry.controller.side=='left'
    now[0]=.21;w.recommendation['selected_slot']='right'
    w.request('parking_trial_keepalive',{'client':'page','run_token':w.entry.token,'trial_sequence':1,'selected_slot':'right'})
    assert w.entry.controller.side=='left'
    w.entry.cancel('test complete')


def test_observation_loop_feeds_real_decoded_frame_and_motion_to_entry(tmp_path):
    import threading
    configs=tmp_path/'vision/configs';configs.mkdir(parents=True)
    (configs/'parking-bay.json').write_text(json.dumps(settings()),encoding='utf-8')
    ok,jpg=cv2.imencode('.jpg',scene('left'));assert ok
    state=SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),raw_frame_id=1,raw_received_ms=1000.,state='ok')
    w=ParkingBayDrive(Base(),{'secondary':state},tmp_path,clock=lambda:1.05)
    samples=[]
    class Entry:
        active=True
        actuator=object()
        def tick(self,sample,now):
            if sample is not None:samples.append(sample);w.stop.set()
        def cancel(self,*a,**kw):self.active=False
    w.entry=Entry();w.active=True;w.folder=tmp_path/'observation';w.folder.mkdir()
    w._run()
    assert len(samples)==1 and samples[0]['fresh'] and samples[0]['image_size']==[480,360]
    assert samples[0]['parking']['layout_found'] and samples[0]['motion']['source']=='ground_optical_estimate'
    assert samples[0]['processing_ms']==0 and samples[0]['frame_age_ms']==pytest.approx(50.)
    assert not w.entry.active


def test_slow_motion_processing_cannot_confirm_an_expired_selection(tmp_path):
    import threading
    configs=tmp_path/'vision/configs';configs.mkdir(parents=True)
    (configs/'parking-bay.json').write_text(json.dumps(settings()),encoding='utf-8')
    ok,jpg=cv2.imencode('.jpg',scene('left'));assert ok
    state=SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),raw_frame_id=1,raw_received_ms=1000.,state='ok')
    now=[1.05];w=ParkingBayDrive(Base(),{'secondary':state},tmp_path,clock=lambda:now[0])
    def slow_motion(*args):now[0]=1.4;return {'valid':True,'moving':False}
    w.motion=SimpleNamespace(update=slow_motion);samples=[]
    class Entry:
        active=True
        actuator=object()
        def tick(self,sample,t):
            if sample is not None:samples.append((sample,t));w.stop.set()
        def cancel(self,*a,**kw):self.active=False
    w.entry=Entry();w.active=True;w.folder=tmp_path/'observation';w.folder.mkdir()
    w._run()
    assert w.recommendation['stale'] and not w.recommendation['confirmed']
    sample,t=samples[0]
    assert sample['fresh'] is False
    assert t==1.4 and sample['processing_ms']==pytest.approx(350.) and sample['frame_age_ms']==pytest.approx(400.)
    record=json.loads((w.folder/'observations.jsonl').read_text(encoding='utf-8'))
    assert record['observation']['frame_age_ms']==pytest.approx(400.)


def test_web_observer_save_and_status_have_no_motor_or_payment_requests(tmp_path):
    folder=tmp_path/'vision/configs';folder.mkdir(parents=True)
    (folder/'parking-bay.json').write_text(json.dumps(settings()),encoding='utf-8')
    base=Base();w=ParkingBayDrive(base,{},tmp_path)
    w.request('parking_bay_save',{'setup_token':'boot','settings_revision':w.revision,'blocked_blue_fraction':.05})
    restored=ParkingBayDrive(base,{},tmp_path)
    assert restored.settings['blocked_blue_fraction']==.05 and not base.calls
    assert not w.status()['parking_bay']['payment_ready'] and not w.status()['parking_bay']['motor_output_enabled']
    with pytest.raises(ValueError):w.request('parking_bay_drive',{})


def test_bad_local_settings_do_not_take_down_the_existing_console(tmp_path):
    folder=tmp_path/'vision/configs';folder.mkdir(parents=True)
    (folder/'parking-bay.json').write_text(json.dumps(settings()),encoding='utf-8')
    damaged=folder/'parking-bay.local.json';damaged.write_text('{broken',encoding='utf-8')
    base=Base();w=ParkingBayDrive(base,{},tmp_path);state=w.status()
    assert state['mode']=='disabled' and state['parking_bay']['settings_error']
    assert not state['parking_bay']['can_observe'] and damaged.read_text()=='{broken'


def test_verification_is_a_boolean_not_a_truthy_string():
    s=calibrated();s['ground']['verified']='false'
    with pytest.raises(ValueError):validate(s)
    assert not readiness(s)['ready']


def test_corrupted_polygon_or_wrong_coordinate_frame_does_not_start():
    s=calibrated();m=ParkingMission(s);x=sample(0,0.)
    x['slots'][0]['polygon_m']=[[0,2],[0,2],[0,3],[0,3]]
    assert m.step(x,0.)['action']=='stop'
    m=ParkingMission(s);x=sample(0,0.);x['coordinate_frame']='camera_relative'
    assert m.step(x,0.)['action']=='stop'


def test_invalid_pose_revokes_current_parking_handoff_claim():
    m=ParkingMission(calibrated());m.phase='parked';m.selected='right';x=sample(1,1.);x['pose']['verified']=False
    r=m.step(x,1.);assert not r['payment_ready'] and r['action']=='brake'


def test_mission_freezes_settings_and_stops_if_vehicle_never_moves():
    s=calibrated();m=ParkingMission(s);s['max_run_s']=1.
    assert m.s['max_run_s']==30.
    for i in range(35):r=m.step(sample(i,i*.1),i*.1)
    assert m.phase=='fault' and r['action']=='stop'


def test_actual_rotated_body_crossing_divider_stops_even_if_axle_is_in_right_bay():
    s=calibrated();m=ParkingMission(s);m.selected='right';m.selected_bounds=(0.,2.,.615,3.);m.phase='entering'
    m.path=plan_entry(slots(),'right',{'x_m':0.,'y_m':0.,'yaw_rad':0.},s)
    r=m.step(sample(1,1.,{'x_m':.3075,'y_m':2.1,'yaw_rad':-.6},speed=.1),1.)
    assert r['phase']=='fault' and r['action']=='brake' and not r['payment_ready']


def test_entry_timeout_and_impossible_heading_jump_stop():
    s=calibrated();m=ParkingMission(s);m.enter_started=0.;m.phase='entering'
    assert m.step(sample(1,31.),31.)['phase']=='fault'
    m=ParkingMission(s);m.step(sample(1,1.),1.)
    r=m.step(sample(2,1.1,{'x_m':0.,'y_m':0.,'yaw_rad':3.14}),1.1)
    assert r['phase']=='fault' and r['action']=='brake'


def test_parking_ui_is_before_payment_and_preserves_other_renderers():
    c=SimpleNamespace(HTML='<section id="parking-payment"></section>',STYLE='',SCRIPT='<script>(()=>{let gamepadControl=null,pwmControl=null;function render(s){existing(s);} })();</script>')
    extend_controls(c)
    assert c.HTML.index('id="parking-bay-test"')<c.HTML.index('id="parking-payment"')
    assert 'renderBeforeParkingBay(s);renderParkingBay(s)' in c.SCRIPT and 'existing(s)' in c.SCRIPT
