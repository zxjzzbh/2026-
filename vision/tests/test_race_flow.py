"""Whole-flow regressions with sensor fixtures and fake output only."""
import copy
import json
import os
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.race_flow import load_bundle,prepare_bundle,RaceFlowController,parking_blockers
from carvision.race_flow_console import RaceFlowDrive
from carvision.cone_trial_output import ConePulseGuard

ROOT=Path(__file__).resolve().parents[2]


@pytest.fixture
def bundle():return load_bundle(Path(os.environ.get('CARVISION_FLOW_TEST_ROOT',ROOT)))


class Frames:
    def __init__(self,controller):self.c=controller;self.t=0.;self.i=0;self.outputs=[]
    def step(self,dt=.1,board='absent',color='red',moving=False,cones=None,ego=.5,parking=None,layout=False,audio=True):
        self.t=round(self.t+dt,8);self.i+=1;t=self.t;i=self.i
        motion={'valid':True,'moving':moving,'captured_s':t}
        nav={'experimental':True,'ready':True,'captured_s':t,'frame_id':i,'ego_fraction':ego,'half_envelope':.20,
             'lane_bounds_px':[[100,380],[50,430]],'cones':cones or [],'uncertain':[],'source_camera':'secondary'}
        signal={'state':color,'fixture_detected':True,'bbox_xyxy':[100,70,250,125]}
        sample={'sequence':i,'captured_s':t,'motion':motion,'navigation':nav,
            'board':{cam:{'state':board,'scene_valid':True,'frame_id':i,'captured_s':t} for cam in ('primary','secondary')},
            'crosswalk':{'fresh':True,'image_size':[480,360],'frame_id':i,'captured_s':t,
                         'candidate':True,'far_edge_y_normalized':.75,'motion':motion},
            'traffic':{'fresh':True,'camera_id':'dual','image_size':[480,360],'frame_id':i,'captured_s':t,'motion':motion,
                'camera_observations':{'secondary':{'camera_id':'secondary','frame_id':i,'captured_s':t,'image_size':[480,360],'signal':signal}}},
            'parking_view':{'layout_found':layout},'parking':parking(t,i) if callable(parking) else parking}
        out=self.c.step(sample,t,{'crosswalk':'command_sent'} if audio else {})
        if out:self.outputs.append((t,out))
        return out
    def until(self,stage,limit=250,**kw):
        for _ in range(limit):
            out=self.step(**kw)
            if self.c.stage==stage:return out
            assert self.c.stage!='fault',out
        pytest.fail(f'never reached {stage}: {out}')


def test_snapshot_does_not_change_individual_trial_behavior(bundle):
    original=copy.deepcopy(bundle);prepared=prepare_bundle(bundle)
    assert bundle==original and prepared['traffic']['green_action']=='speech_only'
    assert prepared['brake']['hold_s']==10 and prepared['flow']['payment_window_s']==30
    assert parking_blockers(bundle) and not prepared['parking']['perception_verified']


def test_board_must_be_seen_then_removed_and_zebra_holds_ten_seconds(bundle):
    c=RaceFlowController(bundle,0.);r=Frames(c);r.until('wait_board')
    for _ in range(20):assert r.step()['esc_us']==1500
    assert c.stage=='wait_board'
    for _ in range(8):r.step(board='present')
    assert c.departure.phase=='wait_removed'
    r.until('crosswalk',board='absent');start=r.t
    r.until('traffic')
    assert r.t-start>=10.5
    speech=[e for _,o in r.outputs for e in o['events'] if e.get('id')=='crosswalk']
    assert len(speech)==1 and speech[0]['text']==bundle['flow']['school_team_speech']


def test_only_actual_red_then_green_advances_traffic_and_single_trial_stays_neutral(bundle):
    c=RaceFlowController(bundle,0.);c._go('traffic',0.,'test');r=Frames(c)
    for _ in range(70):r.step(color='green',moving=True)
    assert c.stage=='traffic' and not c.controller.red_seen
    for _ in range(35):
        r.step(color='red')
        if c.controller.phase=='wait_green':break
    assert c.controller.phase=='wait_green'
    for _ in range(20):r.step(color='red')
    assert c.stage=='traffic'
    r.until('cones',color='green')
    out=r.step(moving=True)
    assert out['esc_us']>1500 and out['phase']=='cones'


def measured_bundle(bundle):
    b=copy.deepcopy(bundle)
    b['parking'].update(perception_verified=True,minimum_turn_radius_m=.4,braking_distance_m=.05,maximum_pose_error_m=.01)
    b['parking']['ground']={'verified':True,'image_size':[480,360],
        'homography':[[.005,0,-1.2],[0,-1/140,2.5],[0,0,1]],'valid_polygon_px':[[3,3],[476,3],[476,356],[3,356]]}
    b['cones']['driving'].update(steering_verified=True,odometry_verified=True,steering_radius_1570_cm=40,
        steering_radius_1650_cm=-40,visual_error_bound_m=.01,visual_verified_distance_m=6.)
    return b


def parking_sample(t,i,pose=None,speed=0.):
    position=pose or {'x_m':0.,'y_m':0.,'yaw_rad':0.}
    return {'frame_id':i,'captured_s':t,'coordinate_frame':'parking_road_world','obstacle_region_verified':True,
        'pose':{**position,'captured_s':t,'error_bound_m':.01,'verified':True},'speed_mps':speed,
        'slots':[{'id':'left','polygon_m':[[-.615,2],[0,2],[0,3],[-.615,3]],'availability':'blocked','occupancy_verified':True},
                 {'id':'right','polygon_m':[[0,2],[.615,2],[.615,3],[0,3]],'availability':'clear','occupancy_verified':True}]}


def test_complete_task_chain_and_every_pwm_through_real_guard(bundle):
    b=measured_bundle(bundle);c=RaceFlowController(b,0.);r=Frames(c)
    r.until('wait_board')
    for _ in range(8):r.step(board='present')
    r.until('crosswalk');r.until('traffic')
    for _ in range(50):
        r.step(color='red')
        if c.controller.phase=='wait_green':break
    assert c.controller.phase=='wait_green'
    r.until('cones',color='green')
    for number,x,ego in ((1,.70,.38),(2,.30,.62)):
        for y in (.55,.63,.71,.73,.75):
            cone={'id':f'bucket-{number}','fraction':x,'half_width':.03,'near':y>=.60,'exit_band':y>=.70,
                  'y_fraction':y,'image_close':y>=.70,'reference_distance_m':None}
            assert r.step(cones=[cone],ego=ego,moving=True)['esc_us']>1500
        for _ in range(14):r.step(ego=ego,moving=True)
    r.until('parking_search',moving=True)
    r.step(layout=True);assert c.stage=='parking_entry'
    for _ in range(10):r.step(parking=parking_sample)
    assert c.parking.selected=='right' and c.parking.path
    for pose in c.parking.path[1:]:r.step(dt=.15,parking=lambda t,i:parking_sample(t,i,pose,.10),moving=True)
    r.until('payment',parking=lambda t,i:parking_sample(t,i,c.parking.path[-1],0.))
    assert c.payment_started==r.t and r.outputs[-1][1]['esc_us']==1500
    c.report_paid(r.t+2);assert c.stage=='complete' and c.paid_in_time and not c.status(r.t+2)['provider_verified']
    assert [t['to'] for t in c.transitions]==['wait_board','crosswalk','traffic','cones','parking_search','parking_entry','payment','complete']
    writes=[];act={'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':1600,'creep_us':1575,'brake_us':1400},
                   'brake_pulse_s':.12,'brake_release_s':.1,'actuator_lease_s':.2}
    guard=ConePulseGuard(act,lambda ch,v:writes.append((ch,v)),mode='F/B',right_us=1550,left_us=1690)
    for sequence,(t,intent) in enumerate(r.outputs):guard.accept({**intent,'sequence':sequence,'sent_s':t},t)
    assert not guard.fault and all(ch in (3,4) for ch,_ in writes)


def test_unverified_parking_stops_before_entry_and_never_starts_payment(bundle):
    c=RaceFlowController(bundle,0.);c._go('parking_search',0.,'test');r=Frames(c)
    r.step(layout=True)
    for _ in range(30):assert r.step(parking=parking_sample)['esc_us']<=1500
    assert c.stage=='parking_entry' and c.payment_started is None and c.status(r.t)['parking_missing']


def test_stale_input_and_repeated_batches_cannot_keep_driving(bundle):
    c=RaceFlowController(bundle,0.)
    assert c.step({'sequence':1,'captured_s':0.},1.)['phase']=='fault'
    c=RaceFlowController(bundle,0.);s={'sequence':1,'captured_s':.1}
    assert c.step(s,.1)['esc_us']==1500 and c.step(s,.2) is None
    assert c.step(s,.4)['phase']=='fault'
    c=RaceFlowController(bundle,0.);c.step({'sequence':2,'captured_s':.1},.1)
    assert c.step({'sequence':1,'captured_s':.2},.2)['phase']=='fault'


def test_payment_timer_does_not_need_camera_or_network_and_never_claims_bank_verification(bundle):
    c=RaceFlowController(bundle,0.);c.stage='payment';c.payment_started=10.
    assert c.step(None,25.)['esc_us']==1500 and c.status(25.)['payment_remaining_s']==15.
    assert c.step(None,40.)['phase']=='complete' and not c.paid
    with pytest.raises(ValueError):c.report_paid(40.)


class Base:
    def __init__(self,manual=False):self.calls=[];self.manual=manual
    def status(self):return {'mode':'enabled' if self.manual else 'disabled','hardware_output':self.manual,'worker_started':self.manual,
                            'pending_hardware_start':False,'setup_token':'boot','trial_token':'manual-token','boot_test_phase':'ready' if self.manual else 'esc_off'}
    def request(self,action,payload):
        self.calls.append(action)
        if action=='finish_test':self.manual=False


def test_explicit_handoff_releases_manual_and_blocks_parallel_trials(bundle,tmp_path,monkeypatch):
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    class Thread:
        def __init__(self,**kw):pass
        def start(self):pass
        def join(self,**kw):pass
    monkeypatch.setattr('carvision.race_flow_console.threading.Thread',Thread)
    now=[1.];base=Base(manual=True);made=[]
    flow=RaceFlowDrive(base,{},tmp_path,live=True,clock=lambda:now[0],actuator_factory=lambda b:made.append(b))
    assert not made and flow.status()['race_flow']['can_enter']
    payload={'client':'operator','setup_token':'boot','trial_token':'manual-token','stationary_confirmed':True,'in_switch_zone':True,'fb_mode_confirmed':True}
    state=flow.request('race_enter',payload)
    assert base.calls[0]=='finish_test' and not base.manual and state['race_flow']['active'] and not made
    for action in ('command','traffic_prepare','crosswalk_prepare','cone_prepare','parking_bay_observe'):
        with pytest.raises(ValueError,match='占用'):flow.request(action,{'client':'operator'})
    with pytest.raises(ValueError,match='未持有'):flow.request('race_keepalive',{'client':'other','run_token':flow.token,'race_sequence':1})
    now[0]=2.
    with pytest.raises(ValueError,match='超时'):flow.request('race_keepalive',{'client':'operator','run_token':flow.token,'race_sequence':1})
    flow.request('race_exit',{});assert not flow.active
    flow.request('traffic_prepare',{});assert base.calls[-1]=='traffic_prepare'


def test_enter_requires_explicit_field_switch_and_never_on_status(bundle,tmp_path,monkeypatch):
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    base=Base();flow=RaceFlowDrive(base,{},tmp_path,live=True,actuator_factory=lambda _:pytest.fail('must not open'))
    flow.status();assert not base.calls
    with pytest.raises(ValueError):flow.request('race_enter',{'client':'operator','setup_token':'boot'})


def test_worker_closes_sole_actuator_when_camera_stops_updating(bundle,tmp_path,monkeypatch):
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    now=[1.];output=[]
    class Actuator:
        run=True
        def __init__(self):self.closed=False
        def send(self,intent):assert not self.closed;output.append(intent)
        def close(self):self.closed=True
    actuator=Actuator()
    class Reader:
        preview=b'actual-test-cache'
        def __init__(self,*args):self.count=0
        def read(self,*args):
            self.count+=1
            if self.count==1:return {'sequence':1,'captured_s':1.}
            now[0]=1.3;return None
    flow=RaceFlowDrive(Base(),{},tmp_path,live=True,clock=lambda:now[0],actuator_factory=lambda _:actuator,reader_factory=Reader)
    flow.active=True;flow.stage='arming';flow.lease_until=2.;flow.controller=RaceFlowController(bundle,1.)
    flow._work()
    assert actuator.closed and not flow.active and flow.stage=='fault'
    assert output and all(i['esc_us']==1500 for i in output)
    assert '相机' in flow.reason


def test_slow_actuator_startup_does_not_consume_neutral_arming_time(bundle,tmp_path,monkeypatch):
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    now=[1.];sent=[]
    class Actuator:
        run=True
        def send(self,intent):sent.append(intent)
        def close(self):pass
    def factory(b):now[0]=3.9;return Actuator()
    class Reader:
        preview=None
        def __init__(self,*a):self.count=0
        def read(self,*a):
            self.count+=1
            if self.count==1:now[0]=4.;return {'sequence':1,'captured_s':4.}
            flow.cancel_event.set();return None
    flow=RaceFlowDrive(Base(),{},tmp_path,live=True,clock=lambda:now[0],actuator_factory=factory,reader_factory=Reader)
    flow.active=True;flow.stage='arming';flow.lease_until=5.;flow.controller=RaceFlowController(bundle,1.)
    flow._work()
    assert sent[0]['phase']=='arming' and sent[0]['esc_us']==1500
    assert flow.controller.started==3.9


def test_page_hidden_stop_does_not_restart_payment_window(bundle,tmp_path,monkeypatch):
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    flow=RaceFlowDrive(Base(),{},tmp_path,live=True,clock=lambda:15.)
    flow.active=True;flow.stage='payment';flow.controller=RaceFlowController(bundle,0.)
    flow.controller.stage='payment';flow.controller.payment_started=10.
    result=flow.request('stop',{'stop_source':'page_hidden'})['race_flow']
    assert result['active'] and result['stage']=='payment' and result['payment_remaining_s']==25.


@pytest.mark.skipif(os.environ.get('CARVISION_SKIP_PC_DEPLOY_TESTS')=='1',reason='PC deployment helper is not installed on the car')
def test_deployment_check_never_opens_ssh_and_preserves_launcher_arguments(monkeypatch):
    import importlib.util,sys
    sys.path.insert(0,str(ROOT/'vision/tools'))
    spec=importlib.util.spec_from_file_location('deploy_full_flow_test',ROOT/'vision/tools/deploy_race_flow.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    assert all((ROOT/path).is_file() for path in module.FILES)
    monkeypatch.setattr(module.paramiko,'SSHClient',lambda:pytest.fail('local check must not connect'))
    monkeypatch.setattr(module.subprocess,'call',lambda command:0)
    assert module.main([])==0
    before=f'[Service]\nExecStart=\nExecStart={module.PYTHON} {module.REMOTE}/vision/tools/serve_parking_payment.py --port 8080 --camera /dev/video0\n'.encode()
    after=module.extend_unit(before).decode()
    assert '--port 8080 --camera /dev/video0 --race-live' in after
    assert 'serve_race_flow.py' in after and after.count('--race-live')==1
    assert module.extend_unit(after.encode()).decode()==after


def test_cached_reader_rejects_duplicate_and_stale_frames_without_opening_camera(bundle):
    import cv2,numpy as np,threading
    from carvision.race_flow_camera import RaceFlowCamera
    now=[1.];data=(ROOT/'vision/tests/fixtures/cone-track-short-secondary.jpg').read_bytes()
    states={k:SimpleNamespace(condition=threading.RLock(),raw_jpeg=data,raw_frame_id=1,
                             raw_received_ms=1000.,state='ok') for k in ('primary','secondary')}
    reader=RaceFlowCamera(states,bundle,ROOT,lambda:now[0])
    first=reader.read('arming',1.)
    assert first['sequence']==1 and first['captured_s']==1. and reader.preview
    assert reader.read('arming',1.) is None
    now[0]=1.3
    with pytest.raises(ValueError):reader.read('arming',1.3)


def test_console_js_and_all_independent_panels_are_composed():
    from carvision import manual_controls as original
    from carvision.brake_controls import extend_controls as brake
    from carvision.traffic_controls import extend_controls as traffic
    from carvision.cone_controls import extend_controls as cones
    from carvision.payment_controls import extend_controls as payment
    from carvision.departure_controls import extend_controls as departure
    from carvision.parking_bay_controls import extend_controls as bay
    from carvision.race_flow_controls import extend_controls as race
    # Match the actual launcher prerequisite without importing a hardware driver.
    controls=SimpleNamespace(HTML='existing',SCRIPT='''<script>(()=>{let gamepadControl=null,pwmControl=null;
function render(s){legacy(s);}async function heartbeat(){original();}})();</script>''',STYLE='')
    for extend in (brake,traffic,cones,payment,departure,bay,race):extend(controls)
    for panel in ('race-flow-panel','crosswalk-test','traffic-test','cone-test','departure-test','parking-bay-test','parking-payment'):
        assert f'id="{panel}"' in controls.HTML
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if node:
        result=subprocess.run([node,'--check'],input=controls.SCRIPT.removeprefix('<script>').removesuffix('</script>'),
                              text=True,encoding='utf-8',capture_output=True,timeout=10)
        assert result.returncode==0,result.stderr
