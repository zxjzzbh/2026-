"""Navigation-log regressions and real shared output with fake hardware only.

699/701 contain no raw tracks/visual_cones. Optional pixel proxies below use
only a cone emitted in that same nav and its current image fractions/bbox. The
proxy is a contract-level what-if, not a replay proving raw detector confirmation.
The historical same-camera X=0 reference is nominal and remains unverified.
"""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.race_flow import RaceFlowController,load_bundle
from carvision.race_flow_console import RaceFlowDrive
from carvision.shared_executor import BorrowedActuator
from test_cone_turn_controller import lane,started
from test_shared_executor import CLIENT,Worker,rig

ROOT=Path(__file__).resolve().parents[2]
FIXTURE=Path(__file__).parent/'fixtures/cone-response-699701-navigation.json'


def session(identifier):
    return json.loads(FIXTURE.read_text(encoding='utf-8'))['sessions'][identifier]


def with_current_pixel_proxy(navigation,settings):
    """Never persist/propagate a pixel from a different nav, frame, or camera."""
    nav=copy.deepcopy(navigation)
    assert nav['source_camera']==settings['camera']=='secondary'
    width,height=settings['ground']['image_size'];H=settings['ground']['homography']
    nav['visual_cones']=[]
    for cone in nav.get('cones',[]):
        box=cone.get('bbox_xyxy')
        if (not isinstance(box,list) or len(box)!=4 or cone.get('provisional')
                or not cone['id'].startswith(nav['source_camera']+'-')
                or not all(key in cone for key in ('image_fraction','y_fraction'))):continue
        reference=-(H[0][1]*cone['y_fraction']*height+H[0][2])/H[0][0]/width
        nav['visual_cones'].append({'id':cone['id'],'current_pixel':True,'confirmed':True,
            # A revision-zero identity is an explicit proxy assumption, not
            # identity/confirmation evidence recovered from the old logs.
            'identity_revision':0,
            'image_fraction':cone['image_fraction'],'y_fraction':cone['y_fraction'],
            'half_image_width':(box[2]-box[0])/(2*width),'bbox_xyxy':copy.deepcopy(box),
            'image_reference_fraction':reference,
            'image_reference_source':'historical_X0_image_reference',
            'source_camera':nav['source_camera'],'frame_id':nav['frame_id'],'captured_s':nav['captured_s']})
    return nav


def replay(identifier,*,pixels=False,controller_type=ContinuousConeController):
    data=session(identifier);rows=data['records']
    controller=controller_type(copy.deepcopy(data['settings']),rows[0]['now_s'],initialized=True)
    # The actual start command was not recorded. The first logged driving
    # decision supplies the arming boundary; pre-drive real frames stay neutral.
    controller.arm_until=data['first_logged_driving_s']
    outputs={}
    for row in rows:
        nav=with_current_pixel_proxy(row['navigation'],data['settings']) if pixels else copy.deepcopy(row['navigation'])
        result=controller.step(nav,row['now_s'])
        outputs[nav['frame_id']]={'intent':result,'progress':copy.deepcopy(controller.progress())}
    return controller,outputs


def test_699_recorded_lane_error_no_longer_disappears_into_heading_deadband_at_original_gain():
    data=session('699');controller,outputs=replay('699')
    assert data['settings']['low_speed_test']['steering_gain']==1.
    for fid in (2321,2322):
        original=next(row['original_intent'] for row in data['records'] if row['navigation']['frame_id']==fid)
        assert original['requested_steering_us']==1610
        assert outputs[fid]['intent']['requested_steering_us']>1610
        assert outputs[fid]['intent']['target_id']=='secondary-215'
    assert controller.count==0 and not controller.passed_ids


def test_701_same_frame_pixel_proxy_prevents_single_uncertainty_reversal_and_hands_off_new_blocker():
    data=session('701');controller,outputs=replay('701',pixels=True)
    original=next(row['original_intent'] for row in data['records'] if row['navigation']['frame_id']==2451)
    assert original['requested_steering_us']<1610
    single=outputs[2451]['intent']
    assert single['target_id']=='secondary-255' and single['target_space']=='image'
    assert single['requested_steering_us']>=1610 and single['steering_us']>=1610
    assert single['soft_boundary_override']
    new=outputs[2455]
    assert new['intent']['target_id']=='secondary-265'
    assert new['intent']['requested_steering_us']<1610
    assert 'secondary-255' in new['progress']['retired_unverified_ids']
    assert new['progress']['passed_count']==0
    # Each has a current pixel proxy; changing paired/missing does not replace
    # that current image demand with an unconditional centered target.
    for fid in (2456,2458,2459,2461,2462,2463):
        assert outputs[fid]['intent']['target_id']=='secondary-265'
        assert outputs[fid]['intent']['requested_steering_us']<1610
    assert controller.count==0 and not controller.passed_ids


def test_proxy_does_not_fill_a_log_gap_or_relabel_historical_lane_coordinates_as_current_pixels():
    data=session('701')
    for row in data['records']:
        nav=with_current_pixel_proxy(row['navigation'],data['settings'])
        assert nav['cones']==row['navigation']['cones']
        assert len(nav['visual_cones'])==len(nav['cones'])
        for pixel in nav['visual_cones']:
            assert 'fraction' not in pixel and 'reference_distance_m' not in pixel
            assert pixel['captured_s']==nav['captured_s'] and pixel['frame_id']==nav['frame_id']
        if not row['navigation']['cones']:assert not nav['visual_cones']
    assert data['raw_tracks_recorded'] is False
    assert data['settings']['ground']['verified'] is False


@pytest.mark.parametrize('identifier',['699','701'])
def test_log_navigation_and_pixel_proxy_cross_actual_race_shared_worker_without_false_pass(identifier,rig):
    now,clock,backend,manual,owner,_=rig
    data=session(identifier);rows=data['records'];now[0]=rows[0]['now_s']
    worker=Worker(owner)
    view=SimpleNamespace(borrow_actuator=lambda settings,client:BorrowedActuator(worker,settings,client,clock=clock))
    bundle=load_bundle(ROOT)
    handle=RaceFlowDrive._make_actuator(SimpleNamespace(drive=view,owner=CLIENT,live=False),bundle)
    flow=RaceFlowController(bundle,now[0],initialized=True);flow._go('cones',now[0],'offline logged-navigation integration')
    flow.controller.arm_until=data['first_logged_driving_s']
    for sequence,row in enumerate(rows):
        now[0]=row['now_s'];nav=with_current_pixel_proxy(row['navigation'],data['settings'])
        output=flow.step({'sequence':sequence,'captured_s':nav['captured_s'],'navigation':nav},now[0])
        assert flow.stage=='cones' and not owner.guard.fault
        if output.get('fresh_control') is not False:handle.send(output)
        assert owner.guard.esc==output['esc_us']
        assert 1350<=owner.guard.actual_steering<=1870
        assert flow.controller.count==0 and not flow.controller.passed_ids
    assert owner.active
    assert all(row[3]<=.2 for row in backend.writes if row[0]=='motion')
    handle.close();now[0]+=.351;owner.tick();assert owner.guard is None


def cone(identifier,fraction,y=.55):
    return {'id':identifier,'fraction':fraction,'half_width':.03,'y_fraction':y,'near':False,
            'bbox_xyxy':[250,210,270,235],'confirmed':True}


def replaced():
    controller=started(gain=1.)
    controller.step(lane(.3,cones=[cone('A',.7)]),.3)
    controller.step(lane(.4,cones=[cone('A',.7),cone('B',.5,.7)]),.4)
    assert controller.target=='B'
    return controller


def test_retired_target_cannot_ping_pong_or_extend_its_reacquire_window_on_duplicates():
    controller=replaced()
    for t in (.5,.6):
        output=controller.step(lane(t,cones=[cone('A',.25,.85),cone('B',.5,.7)]),t)
        assert output['target_id']=='B' and 'A' in output['target_reacquire_blocked_ids']
    held=controller.step(lane(.6,cones=[cone('A',.25,.85),cone('B',.5,.7)]),.69)
    assert held['fresh_control'] is False and controller.target=='B'
    output=controller.step(lane(.701,cones=[cone('A',.25,.9),cone('B',.5,.7)]),.701)
    assert output['target_id']=='A' and controller.count==0 and not controller.passed_ids


def test_short_reacquire_hold_does_not_hide_a_different_current_blocker():
    controller=replaced()
    output=controller.step(lane(.5,cones=[cone('A',.25,.85),cone('C',.3,.9),cone('B',.5,.7)]),.5)
    assert output['target_id']=='C' and controller.count==0 and not controller.passed_ids


@pytest.mark.parametrize('mismatch',[{'captured_s':.2},{'captured_s':.31},
                                     {'frame_id':1},{'source_camera':'primary'}])
def test_copying_old_or_other_camera_visual_pixels_cannot_extend_an_existing_target(mismatch):
    controller=started(gain=1.)
    controller.step(lane(.3,cones=[cone('secondary-A',.7)]),.3)
    nav=lane(.4,cones=[{**cone('secondary-A',.7,.8),'near_field_continuation':True}])
    nav['visual_cones']=[{'id':'secondary-A','current_pixel':True,'confirmed':True,
        'image_fraction':.7,'half_image_width':.03,'image_reference_fraction':.5,
        'source_camera':'secondary','captured_s':.4,'frame_id':nav['frame_id'],**mismatch}]
    output=controller.step(nav,.4)
    assert output['target_space']!='image' and controller.count==0
