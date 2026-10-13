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


def test_no_board_does_not_select_arbitrary_half():
    obs,_=detect_parking(scene(None),settings());sel=BaySelection()
    for i in range(8):result=sel.update(obs,i,i*.1,i*.1)
    assert not result['confirmed'] and result['selected_slot'] is None


def test_dark_interior_is_unknown_not_clear():
    im=scene('left');cv2.rectangle(im,(247,80),(355,203),(0,0,0),-1)
    obs,_=detect_parking(im,settings())
    assert obs['layout_found'] and obs['slots'][1]['availability']=='unknown'


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
