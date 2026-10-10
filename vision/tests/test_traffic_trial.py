import json
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.brake_controls import extend_controls as extend_brake
from carvision.traffic_controls import extend_controls, add_preview_endpoint
from carvision.traffic_trial import TrafficCamera, TrafficTrialDrive
from carvision.traffic_driving import load_settings

ROOT = Path(__file__).parents[1]
CONFIG, PROFILE = ROOT/'configs/traffic-driving.json', ROOT/'configs/traffic-signal.json'


class Driver:
    def __init__(self):
        self.commands = []
        self.state = {'setup_token': 'boot', 'mode': 'disabled', 'boot_test_phase': 'esc_off',
                      'hardware_output': False, 'worker_started': False, 'crosswalk_trial': {'active': False, 'can_prepare': True}}
    def status(self): return dict(self.state)
    def request(self, action, payload):
        self.commands.append((action, payload))
        if action == 'finish_test': self.state.update(mode='disabled', boot_test_phase='esc_off', hardware_output=False)
    def close(self): pass


class DeferredThread:
    def __init__(self, **kwargs): pass
    def start(self): pass
    def join(self, **kwargs): pass


class Speech:
    def __init__(self): self.count = 0
    def start(self): self.count += 1
    def status(self): return {'state': 'command_sent' if self.count else 'not_requested'}
    def close(self): pass


@pytest.fixture
def trial(tmp_path, monkeypatch):
    monkeypatch.setattr('carvision.traffic_trial.threading.Thread', DeferredThread)
    now, opened, base = [1.], [], Driver()
    w = TrafficTrialDrive(base, {}, CONFIG, PROFILE, tmp_path, clock=lambda: now[0],
                          actuator_factory=lambda: opened.append('opened'), speech_factory=Speech)
    return w, base, now, opened


def prepare(w, **kw):
    return w.request('traffic_prepare', {'client': 'page', 'setup_token': 'boot', 'esc_off': True, 'fb_mode_confirmed': True, **kw})


def owned(w, n=0, **kw):
    return {'client': 'page', 'run_token': w.token, 'trial_sequence': n, **kw}


def ready(w, t):
    w.phase = 'ready'
    w.observation = {'camera_id':'dual','captured_s': t, 'signal': {'state': 'red', 'fixture_detected': True}}


def test_status_settings_and_manual_forward_do_not_open_new_hardware(trial):
    w, base, _, opened = trial
    assert w.status()['traffic_trial']['can_prepare']
    w.request('command', {'motor': 'forward'})
    assert base.commands == [('command', {'motor': 'forward'})]
    w.request('traffic_settings', {'client': 'page', 'settings_revision': w.settings_revision,
                                  'forward_us': 1565, 'max_approach_s': 2})
    assert not opened
    restored = TrafficTrialDrive(base, {}, CONFIG, PROFILE, w.root)
    assert restored.settings['pwm']['search_us'] == 1565 and restored.settings['max_approach_s'] == 2


def test_higher_forward_is_saved_without_driving_and_used_by_next_round(trial):
    w, base, now, opened=trial
    assert w.settings['pwm']['search_us']==1600
    w.request('traffic_settings',{'client':'page','settings_revision':w.settings_revision,'forward_us':1605})
    restored=TrafficTrialDrive(base,{},CONFIG,PROFILE,w.root)
    assert restored.settings['pwm']['search_us']==1605
    prepare(w);ready(w,now[0])
    w.request('traffic_start',owned(w,fb_mode_confirmed=True,esc_on_static=True,field_ready=True))
    assert w.controller.s['pwm']['search_us']==1605 and not opened
    with pytest.raises(ValueError):
        w.request('traffic_settings',{'client':'page','settings_revision':w.settings_revision,'forward_us':1610})
    assert w.active_settings['pwm']['search_us']==1605


def test_only_traffic_factory_requests_the_higher_output_envelope(trial,monkeypatch):
    w, base, _, _=trial;calls=[]
    monkeypatch.setattr('carvision.traffic_trial.BrakeActuator',lambda *a,**kw:calls.append((a,kw)))
    actual=TrafficTrialDrive(base,{},CONFIG,PROFILE,w.root)
    actual.actuator_factory()
    assert calls[0][1]['forward_limit_us']==1625
    assert calls[0][0][0]['pwm']['search_us']==1600


@pytest.mark.parametrize('pulse',[1500,1626,1700,True,float('nan')])
def test_throttle_api_rejects_invalid_values_before_persisting(trial,pulse):
    w,_,_,opened=trial
    with pytest.raises(ValueError):
        w.request('traffic_settings',{'client':'page','settings_revision':w.settings_revision,'forward_us':pulse})
    assert not w.settings_path.exists() and not opened


def test_red_percentage_persists_and_is_frozen_while_prepared(trial):
    w, base, now, opened = trial
    assert w.settings['red_confirm_percent'] == 50
    w.request('traffic_settings', {'client': 'page', 'settings_revision': w.settings_revision,
                                   'red_confirm_percent': 70})
    restored = TrafficTrialDrive(base, {}, CONFIG, PROFILE, w.root)
    assert restored.settings['red_confirm_percent'] == 70 and restored.settings['red_window_s'] == 2
    prepare(w); ready(w,now[0])
    w.request('traffic_start', owned(w, fb_mode_confirmed=True, esc_on_static=True, field_ready=True))
    assert w.controller.votes.threshold == 70
    with pytest.raises(ValueError):
        w.request('traffic_settings', {'client': 'page','settings_revision':w.settings_revision,'red_confirm_percent':20})
    assert w.active_settings['red_confirm_percent'] == 70 and not opened


def test_old_persisted_parameters_pick_up_fifty_percent_default(trial):
    w, base, _, opened = trial
    w.settings_path.parent.mkdir(parents=True, exist_ok=True)
    w.settings_path.write_text(json.dumps({'forward_us':1575,'max_approach_s':10}),encoding='utf-8')
    restored = TrafficTrialDrive(base,{},CONFIG,PROFILE,w.root)
    assert restored.settings['red_confirm_percent'] == 50 and restored.settings['max_approach_s'] == 10
    assert not restored.settings_error and not opened


@pytest.mark.parametrize('percent',[0,101,True,'50',float('nan')])
def test_api_rejects_invalid_red_percentage_without_writing(trial,percent):
    w,_,_,opened=trial
    with pytest.raises(ValueError):
        w.request('traffic_settings', {'client':'page','settings_revision':w.settings_revision,'red_confirm_percent':percent})
    assert not w.settings_path.exists() and not opened


def test_crosswalk_and_traffic_control_are_mutually_exclusive(trial):
    w, base, _, opened = trial
    base.state['crosswalk_trial']['active'] = True
    assert not w.status()['traffic_trial']['can_prepare']
    with pytest.raises(ValueError): prepare(w)
    base.state['crosswalk_trial']['active'] = False
    prepare(w)
    assert not w.status()['crosswalk_trial']['can_prepare']
    for action in ('crosswalk_prepare', 'command', 'prepare'):
        with pytest.raises(ValueError): w.request(action, {'client': 'page'})
    assert not opened


def test_manual_owner_is_released_before_traffic_worker(trial):
    w, base, _, opened = trial
    base.state.update(mode='enabled', boot_test_phase='ready', hardware_output=True)
    prepare(w)
    assert base.commands[0][0] == 'finish_test' and not opened


@pytest.mark.parametrize('changes', [{'client': None}, {'setup_token': 'old'}, {'esc_off': False}, {'fb_mode_confirmed': False}])
def test_prepare_checks_current_page_and_operator_declarations(trial, changes):
    w, _, _, opened = trial
    with pytest.raises(ValueError): prepare(w, **changes)
    assert not w.busy and not opened


def test_start_requires_fresh_camera_and_owner_but_not_visible_lamp(trial):
    w, _, now, opened = trial; prepare(w); w.phase = 'ready'
    flags = dict(fb_mode_confirmed=True, esc_on_static=True, field_ready=True)
    with pytest.raises(ValueError): w.request('traffic_start', owned(w, **flags))
    ready(w, now[0])
    w.observation['signal'].update(fixture_detected=False,state='unknown')
    assert w.status()['traffic_trial']['can_start'] and not w.status()['traffic_trial']['vision_ready']
    with pytest.raises(ValueError): w.request('traffic_start', owned(w, client='other', **flags))
    w.request('traffic_start', owned(w, **flags))
    assert w.phase == 'arming' and w.round_number == 1 and not opened
    before = w.lease_until; now[0] += .1
    with pytest.raises(ValueError): w.request('traffic_keepalive', owned(w))
    assert w.lease_until == before
    w.request('traffic_keepalive', owned(w, 1))
    assert w.lease_until == now[0]+.5


def test_start_accepts_detected_unlit_fixture_before_external_sensor_trigger(trial):
    w, _, now, opened = trial; prepare(w); ready(w, now[0])
    w.observation['signal']['state'] = 'off'
    status = w.status()['traffic_trial']
    assert status['can_start'] and status['start_blocked_reason'] == ''
    w.request('traffic_start', owned(w, fb_mode_confirmed=True, esc_on_static=True, field_ready=True))
    assert w.phase == 'arming' and not opened


def test_start_explains_preparation_and_stale_video_without_lamp_gate(trial):
    w, _, now, _ = trial
    assert w.status()['traffic_trial']['start_blocked_code'] == 'prepare_required'
    prepare(w)
    assert w.status()['traffic_trial']['start_blocked_code'] == 'preparing'
    ready(w, now[0]); w.observation['signal']['fixture_detected'] = False
    assert w.status()['traffic_trial']['start_blocked_code'] == ''
    w.observation['signal'].update(fixture_detected=True, state='unknown')
    assert w.status()['traffic_trial']['start_blocked_code'] == ''
    w.observation['signal']['state'] = 'off'; now[0] += .3
    assert w.status()['traffic_trial']['start_blocked_code'] == 'frame_stale'
    w.request('stop', {'stop_source': 'window_blur'})
    assert w.status()['traffic_trial']['start_blocked_code'] == 'prepare_required'
    assert '失去焦点' in w.reason


def test_start_button_is_stable_and_only_frame_freshness_blocks_the_request(trial):
    w, _, now, opened = trial; prepare(w); ready(w, now[0])
    flags = dict(fb_mode_confirmed=True, esc_on_static=True, field_ready=True)
    for fixture, state, age in [(True,'off',0), (True,'unknown',0), (False,'unknown',0),
                                (True,'off',.3), (True,'red',0)]:
        w.observation = {'camera_id':'dual','captured_s':now[0]-age,'signal':{'state':state,'fixture_detected':fixture}}
        t = w.status()['traffic_trial']
        assert t['can_request_start']  # The button does not follow single-frame flicker.
        assert t['can_start'] == (age<=.25)
        if not t['can_start']:
            with pytest.raises(ValueError): w.request('traffic_start',owned(w,**flags))
            assert w.phase == 'ready' and w.controller is None
    assert not opened
    w.request('traffic_start',owned(w,**flags))
    assert w.phase=='arming' and not w.status()['traffic_trial']['can_request_start']


def test_snapshot_time_orders_status_without_renewing_control_lease(trial):
    w, _, now, _ = trial; prepare(w)
    first = w.status(); until = w.lease_until
    now[0] += .1
    second = w.status()
    assert second['traffic_status_s'] > first['traffic_status_s']
    assert w.lease_until == until
    w.request('stop',{})
    assert not w.status()['traffic_trial']['can_request_start']


def test_settings_cannot_change_mid_run_or_from_stale_revision(trial):
    w, _, _, _ = trial
    with pytest.raises(ValueError): w.request('traffic_settings', {'client': 'page', 'settings_revision': 'old'})
    prepare(w)
    with pytest.raises(ValueError): w.request('traffic_settings', {'client': 'page', 'settings_revision': w.settings_revision})


def test_emergency_stop_from_any_page_is_allowed(trial):
    w, base, _, _ = trial; prepare(w)
    w.request('emergency', {'client': 'other'})
    assert w.stop.is_set() and w.phase == 'cancelled' and not base.commands


def test_invalid_camera_is_caught_before_actuator_open(trial):
    w, _, _, opened = trial; prepare(w); w._work()
    assert w.phase == 'fault' and not w.busy and not opened


@pytest.mark.parametrize('lose_page', [False, True])
def test_real_worker_records_red_brake_green_speech_and_closes_on_loss(trial, lose_page):
    w, _, now, _ = trial
    writes = []
    class Actuator:
        def send(self, row):
            writes.append(dict(row)); return {'hardware_output': False}
        def close(self): writes.append({'action': 'close'})
    class Reader:
        jpeg = preview = None
        def __init__(self, *_): self.n = 0
        def read(self, _):
            self.n += 1; now[0] = round(self.n/10, 8)
            if w.phase == 'complete': w.stop.set(); return None
            if not lose_page or self.n < 48: w.lease_until = now[0]+.5
            if w.phase == 'ready' and self.n >= 3:
                w.request('traffic_start', owned(w, fb_mode_confirmed=True, esc_on_static=True, field_ready=True))
            color = 'green' if self.n < 42 or self.n > 80 else 'red'
            return {'fresh': True, 'camera_id':'dual','frame_id': self.n, 'captured_s': now[0], 'image_size': [480, 360],
                    'signal': {'state': color, 'fixture_detected': True, 'bbox_xyxy': [100, 70, 250, 125]},
                    'motion': {'valid': True, 'moving': w.phase == 'approach', 'captured_s': now[0]},
                    'camera_observations': {'secondary': {'camera_id':'secondary','frame_id':self.n,
                        'captured_s':now[0],'image_size':[480,360],
                        'signal':{'state':color,'fixture_detected':True,'bbox_xyxy':[100,70,250,125]}}}}
    w.reader_factory, w.actuator_factory = Reader, Actuator
    prepare(w); w._work()
    assert writes[-1]['action'] == 'close' and not w.busy
    assert w.phase == ('fault' if lose_page else 'complete')
    assert any(r.get('action') == 'search' for r in writes)
    assert any(r.get('action') == 'brake' for r in writes)
    if not lose_page:
        event_index = next(i for i, r in enumerate(writes) if r.get('events') == [{'event': 'speak', 'text': '红绿灯结束，开始前行'}])
        assert all(r.get('action') in ('neutral', 'close') for r in writes[event_index:])
        assert w.speech.count == 1
    assert json.loads((w.folder/'summary.json').read_text(encoding='utf-8'))['physical_stop_verified'] is False


def test_camera_uses_existing_states_and_fails_on_stale_secondary():
    _, jpg = cv2.imencode('.jpg', np.zeros((360, 480, 3), np.uint8))
    def state():
        return SimpleNamespace(condition=threading.RLock(), raw_jpeg=jpg.tobytes(), raw_frame_id=1, raw_received_ms=1000, state='ok')
    p, s = state(), state()
    reader = TrafficCamera({'primary': p, 'secondary': s}, load_settings(CONFIG), None)
    sample = reader.read(1)
    assert sample['frame_id'] == 1 and sample['signal']['state'] == 'unknown'
    assert sample['camera_id']=='dual'
    assert reader.preview and reader.read(1.1) is None
    p.raw_frame_id, p.raw_received_ms = 2, 1300
    assert set(reader.read(1.3)['camera_observations'])=={'primary'}


def test_decision_and_motion_use_same_secondary_frame_even_if_primary_is_broken():
    def state(value,fid):
        _,jpg=cv2.imencode('.jpg',np.full((360,480,3),value,np.uint8))
        return SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),raw_frame_id=fid,
                               raw_received_ms=1000,state='ok')
    primary,secondary=state(20,90),state(180,7)
    primary.state='error'
    reader=TrafficCamera({'primary':primary,'secondary':secondary},load_settings(CONFIG),None)
    reader.detectors['secondary']=SimpleNamespace(detect=lambda image:{'state':'red' if image.mean()>100 else 'green','bbox_xyxy':None})
    sample=reader.read(1)
    assert sample['camera_id']=='dual' and sample['camera_observations']['secondary']['frame_id']==7 and sample['signal']['state']=='red'
    assert reader.jpegs['secondary']==secondary.raw_jpeg and sample['motion']['captured_s']==sample['captured_s']
    assert reader.read(1.1) is None  # A changing primary cannot renew the decision.
    secondary.raw_frame_id=8;secondary.raw_received_ms=1100
    assert reader.read(1.1)['camera_observations']['secondary']['frame_id']==8
    secondary.state='error'
    with pytest.raises(ValueError,match='两路'):reader.read(1.2)


def test_either_camera_can_supply_frames_and_only_secondary_supplies_motion():
    _,jpg=cv2.imencode('.jpg',np.zeros((360,480,3),np.uint8))
    s=SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),raw_frame_id=1,raw_received_ms=1000,state='ok')
    reader=TrafficCamera({'secondary':s},load_settings(CONFIG),None)
    assert set(reader.read(1)['camera_observations'])=={'secondary'}
    wrong=TrafficCamera({'primary':s},load_settings(CONFIG),None)
    sample=wrong.read(1)
    assert set(sample['camera_observations'])=={'primary'} and sample['motion']['valid'] is False


def test_both_streams_use_original_classifier_profile_and_keep_independent_ids():
    from carvision.traffic_signal import TrafficSignalDetector
    def state(value,fid):
        _,jpg=cv2.imencode('.jpg',np.full((360,480,3),value,np.uint8))
        return SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),raw_frame_id=fid,raw_received_ms=1000,state='ok')
    primary,secondary=state(20,90),state(180,7)
    reader=TrafficCamera({'primary':primary,'secondary':secondary},load_settings(CONFIG),None)
    assert all(type(d) is TrafficSignalDetector for d in reader.detectors.values())
    assert reader.detectors['primary'].profile==reader.detectors['secondary'].profile
    seen=[]
    def detect(image):
        seen.append(round(float(image.mean())))
        return {'state':'red' if image.mean()>100 else 'green','fixture_detected':True,'bbox_xyxy':[20,20,200,90]}
    for c in reader.detectors:reader.detectors[c]=SimpleNamespace(detect=detect)
    sample=reader.read(1)
    assert seen==[20,180]
    assert sample['camera_observations']['primary']['signal']['state']=='green'
    assert sample['camera_observations']['secondary']['signal']['state']=='red'
    assert sample['camera_observations']['primary']['frame_id']==90
    assert sample['camera_observations']['secondary']['frame_id']==7
    assert cv2.imdecode(np.frombuffer(reader.preview,np.uint8),cv2.IMREAD_COLOR).shape[:2]==(360,960)
    assert reader.read(1.01) is None
    primary.raw_frame_id=91;primary.raw_received_ms=1100
    sample=reader.read(1.1)
    assert seen==[20,180,20] and sample['camera_observations']['secondary']['frame_id']==7


def test_secondary_published_during_primary_processing_is_not_mistaken_for_future_frame(monkeypatch):
    _,jpg=cv2.imencode('.jpg',np.zeros((360,480,3),np.uint8))
    def state():return SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),raw_frame_id=1,raw_received_ms=1000,state='ok')
    p,s=state(),state();timer=[10.]
    monkeypatch.setattr('carvision.traffic_trial.time.perf_counter',lambda:timer[0])
    reader=TrafficCamera({'primary':p,'secondary':s},load_settings(CONFIG),None)
    def first(image):
        timer[0]+=.1;s.raw_received_ms=1100
        return {'state':'unknown','bbox_xyxy':None}
    reader.detectors['primary']=SimpleNamespace(detect=first)
    reader.detectors['secondary']=SimpleNamespace(detect=lambda image:{'state':'unknown','bbox_xyxy':None})
    sample=reader.read(1.)
    assert set(sample['camera_observations'])=={'primary','secondary'}
    assert sample['camera_observations']['secondary']['captured_s']==1.1


def test_ui_extension_keeps_both_original_workflows_and_simple_controls():
    original = '''<script>(()=>{let gamepadControl=null,pwmControl=null;
function render(s){renderLegacy(s);}
async function heartbeat(){originalHeartbeat();}
})();</script>'''
    c = SimpleNamespace(SCRIPT=original, HTML='<p>existing cameras</p>')
    extend_brake(c); extend_controls(c)
    assert 'crosswalk_prepare' in c.SCRIPT and 'traffic_prepare' in c.SCRIPT
    assert 'originalHeartbeat();' in c.SCRIPT and 'renderLegacy(s)' in c.SCRIPT
    assert 'type="checkbox"' not in c.HTML and 'traffic-continue' not in c.HTML
    assert '红绿灯结束，开始前行' in c.HTML
    assert 'id="traffic-red-percent"' in c.HTML and 'value="50"' in c.HTML
    assert 'red_confirm_percent:Number' in c.SCRIPT and "v.red_frames" in c.SCRIPT
    assert 'if(trafficActive)' in c.SCRIPT


def test_preview_endpoint_never_serves_stale_or_inactive_images(trial):
    w, _, now, opened = trial
    class Base:
        def send_body(self, body, mime, status=200): self.reply = (body, mime, status)
        def do_GET(self): self.reply = 'original'
    server = SimpleNamespace(RequestHandlerClass=Base)
    add_preview_endpoint(server, w)
    h = server.RequestHandlerClass(); h.path = '/api/traffic/frame.jpg'
    w.preview_jpeg = b'jpeg'; w.observation = {'captured_s': now[0]}
    h.do_GET(); assert h.reply[2] == 503
    w.busy = True; h.do_GET(); assert h.reply == (b'jpeg', 'image/jpeg', 200)
    now[0] += .3; h.do_GET(); assert h.reply[2] == 503
    h.path = '/api/status'; h.do_GET(); assert h.reply == 'original'
    assert not opened
