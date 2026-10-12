"""Supervised traffic stage on the existing console; one UART owner at a time."""
import copy
import json
import secrets
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .brake_output import BrakeActuator
from .brake_parking import GroundMotionEstimate, numeric
from .parking_speech import ParkingSpeech
from .traffic_driving import (TrafficDrivingController, load_settings, validate_settings,
                              TRAFFIC_FORWARD_MIN_US, TRAFFIC_FORWARD_MAX_US)
from .traffic_signal import TrafficSignalDetector, load_profile


class TrafficCamera:
    """Run the same classifier independently on both cached camera streams."""
    def __init__(self, states, settings, profile, clock=None):
        self.states, self.s = states, settings
        self.clock = clock
        self.detectors = {c:TrafficSignalDetector(profile) for c in settings['cameras']}
        self.motion = GroundMotionEstimate()
        self.last_ids, self.last_times, self.observations = {}, {}, {}
        self.images, self.jpegs, self.errors = {}, {}, {}
        self.motion_result = {'valid':False,'moving':None}
        self.sequence = 0
        self.jpeg = self.preview = None

    def _frame(self, camera, now, batch_started):
        state = self.states.get(camera)
        if state is None: raise ValueError(camera+' 摄像头未就绪')
        with state.condition:
            jpeg, fid, ms = state.raw_jpeg, state.raw_frame_id, state.raw_received_ms
            failed = state.state in ('error','stopped')
        observed_now = self.clock() if self.clock else now + (time.perf_counter()-batch_started)
        if (failed or not jpeg or type(fid) is not int or not numeric(ms)
                or not -1e-9 <= observed_now-ms/1000 <= self.s['max_frame_age_s']):
            raise ValueError(camera+' 摄像头画面失效')
        return jpeg, fid, ms/1000

    def read(self, now):
        changed = False
        batch_started = time.perf_counter()
        for camera in self.s['cameras']:
            try:
                jpeg, fid, captured = self._frame(camera, now, batch_started)
                if fid == self.last_ids.get(camera) and captured == self.last_times.get(camera):
                    continue
                image = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                if image is None or [image.shape[1],image.shape[0]] != self.s['image_size']:
                    raise ValueError(camera+' 图像尺寸与480×360配置不一致')
                observation = self.detectors[camera].detect(image)
                if camera == self.s['motion_camera']:
                    self.motion_result = {**self.motion.update(image,fid,captured),'captured_s':captured}
                self.observations[camera] = {'camera_id':camera,'frame_id':fid,'captured_s':captured,
                    'image_size':self.s['image_size'],'signal':observation}
                self.last_ids[camera], self.last_times[camera] = fid, captured
                self.jpegs[camera], self.images[camera] = jpeg, image
                self.errors.pop(camera,None)
                changed = True
            except (ValueError, cv2.error) as exc:
                self.errors[camera] = str(exc)
                self.observations.pop(camera,None);self.images.pop(camera,None);self.jpegs.pop(camera,None)
                self.last_ids.pop(camera,None);self.last_times.pop(camera,None)
        if not self.observations: raise ValueError('两路均无新鲜图像：'+'; '.join(self.errors.values()))
        if not changed: return None  # Cached copies cannot renew an output lease.
        self.sequence += 1
        panes = []
        for camera in self.s['cameras']:
            row = self.observations.get(camera)
            if row is None:
                display = np.zeros((360,480,3),np.uint8)
                cv2.putText(display,camera.upper()+' NO FRESH FRAME',(10,28),cv2.FONT_HERSHEY_SIMPLEX,.55,(180,180,180),1)
            else:
                display = self.images[camera].copy();obs = row['signal']
                color = {'red':(40,40,255),'yellow':(0,220,255),'green':(70,220,80)}.get(obs['state'],(180,180,180))
                box=obs.get('bbox_xyxy')
                if box:
                    x1,y1,x2,y2=map(int,box);cv2.rectangle(display,(x1,y1),(x2,y2),color,2)
                cv2.putText(display,f"{camera.upper()} #{row['frame_id']} {obs['state'].upper()}",(8,23),cv2.FONT_HERSHEY_SIMPLEX,.55,color,2)
            panes.append(display)
        ok, jpg = cv2.imencode('.jpg',np.concatenate(panes,axis=1))
        self.preview = jpg.tobytes() if ok else None
        self.jpeg = self.preview
        raw = {r['signal']['state'] for r in self.observations.values()}
        merged = {'state':next((c for c in ('red','yellow','green','off') if c in raw),'unknown'),
                  'fixture_detected':any(r['signal'].get('fixture_detected') for r in self.observations.values()),
                  'processing_ms':sum(r['signal'].get('processing_ms',0) for r in self.observations.values())}
        motion = dict(self.motion_result) if self.s['motion_camera'] in self.observations else {'valid':False,'moving':None}
        return {'fresh':True,'camera_id':'dual','frame_id':self.sequence,
                'captured_s':max(r['captured_s'] for r in self.observations.values()),
                'image_size':self.s['image_size'],'signal':merged,'motion':motion,
                'camera_observations':copy.deepcopy(self.observations),'camera_errors':dict(self.errors)}


class TrafficTrialDrive:
    """Wrap BrakeTrialDrive without changing original manual/crosswalk commands.

Constructor/status/preview/settings are inert. Only prepare opens a neutral
actuator, and only start can subsequently request forward/braking commands.
"""
    def __init__(self, drive, states, config, profile, root, *, actuator_factory=None,
                 reader_factory=TrafficCamera, speech_factory=None, clock=time.monotonic):
        self.drive, self.states, self.root, self.clock = drive, states, Path(root), clock
        self.settings, self.profile = load_settings(config), load_profile(profile)
        self.settings_path = self.root/'vision/configs/traffic-driving.local.json'
        self.settings_revision = secrets.token_hex(8)
        self.settings_error = None
        if self.settings_path.exists():
            try:
                self.settings = self._updated(json.loads(self.settings_path.read_text(encoding='utf-8')))
            except (OSError, ValueError, TypeError, KeyError) as exc:
                self.settings_error = str(exc)
        self.active_settings = copy.deepcopy(self.settings)
        self.actuator_factory = actuator_factory or (lambda: BrakeActuator(self.active_settings, run=True, esc_mode='F/B',
                                                                         forward_limit_us=TRAFFIC_FORWARD_MAX_US))
        self.reader_factory = reader_factory
        self.speech_factory = speech_factory or (lambda: ParkingSpeech(self.root, text=self.active_settings['speech_text']))
        self.speech = None
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.thread = None
        self.busy = self.closed = False
        self.phase, self.reason = 'idle', '红绿灯场地测试就绪；先关闭电调并准备'
        self.client = self.token = self.folder = None
        self.last_sequence, self.round_number = -1, 0
        self.lease_until = 0
        self.controller = self.latest = self.observation = self.preview_jpeg = None
        self.execution = None

    def _updated(self, payload):
        s = copy.deepcopy(self.settings)
        p = payload.get('forward_us', s['pwm']['search_us'])
        s['pwm']['search_us'] = s['pwm']['creep_us'] = p
        for name in ('max_approach_s', 'red_confirm_percent'):
            if name in payload:
                s[name] = payload[name]
        return validate_settings(s)

    def status(self):
        with self.lock:
            snapshot_s = self.clock()
            base = self.drive.status()
            zebra = base.get('crosswalk_trial', {})
            idle = (base.get('boot_test_phase') == 'esc_off' and base.get('mode') == 'disabled'
                    and not base.get('hardware_output') and not base.get('worker_started')
                    and not base.get('pending_hardware_start'))
            can_end = base.get('boot_test_phase') == 'ready' and not zebra.get('active')
            sample = self.observation or {}
            age = snapshot_s-sample.get('captured_s', -1e9)
            raw = sample.get('signal', {})
            frame_ok = (0 <= age <= self.settings['max_frame_age_s']
                        and sample.get('camera_id') == self.settings['camera'])
            vision_ok = (frame_ok and raw.get('fixture_detected') is True
                         and raw.get('state') in ('red', 'yellow', 'green', 'off'))
            code, blocked = '', ''
            if not self.busy or self.closed or self.stop.is_set():
                code, blocked = 'prepare_required', '本轮尚未准备或已停止；先关闭电调，再点红绿灯面板的准备'
            elif self.phase == 'preparing':
                code, blocked = 'preparing', '正在检查相机和建立中位输出，请等待准备完成'
            elif self.phase not in ('ready', 'complete'):
                code, blocked = 'round_running', '本轮已经开始，无需重复点击开始'
            elif not frame_ok:
                code, blocked = 'frame_stale', '两路均没有新鲜画面；等待摄像头恢复，无需看见灯具'
            trial = {'program': 'traffic-fb-v1', 'active': self.busy, 'phase': self.phase, 'reason': self.reason,
                     'owner_client': self.client, 'run_token': self.token, 'round_number': self.round_number,
                     'can_prepare': not self.closed and not self.busy and not zebra.get('active') and (idle or can_end) and not self.settings_error,
                     'can_start': not code,
                     # Stable presentation gate. The actual request still
                     # checks camera freshness, not lamp visibility, before arming.
                     'can_request_start': self.busy and not self.closed and not self.stop.is_set()
                        and self.phase in ('ready', 'complete'),
                     'start_blocked_code': code, 'start_blocked_reason': blocked,
                     'can_end_manual': can_end, 'settings': self.settings, 'running_settings': self.active_settings,
                     'forward_limits_us': [TRAFFIC_FORWARD_MIN_US, TRAFFIC_FORWARD_MAX_US], 'forward_step_us': 5,
                     'settings_revision': self.settings_revision, 'settings_error': self.settings_error,
                     'intent': self.latest, 'observation': self.observation,
                     'frame_age_s': age if sample else None, 'vision_ready': vision_ok, 'camera_ready': frame_ok,
                     'camera': self.settings['camera'], 'camera_label': '双摄独立识别，任一路达标', 'motion_camera': self.settings['motion_camera'],
                     'start_requires_fixture': False, 'search_without_visible_fixture': True,
                     'vote_window_s': self.settings['red_window_s'], 'signal_confirm_percent': self.settings['red_confirm_percent'],
                     'cameras': self.settings['cameras'], 'vote_rule': 'independent_any_camera_red_priority',
                     'green_action': 'speech_only', 'speech_text': self.settings['speech_text'],
                     'speech': self.speech.status() if self.speech else None,
                     'hardware_output': bool(self.busy and self.execution and self.execution.get('hardware_output')),
                     'log_directory': str(self.folder) if self.folder else None,
                     'physical_stop_verified': False, 'parking_zone_verified': False,
                     'external_sensor_connected': False, 'field_verified': False}
            if self.busy:
                zebra = {**zebra, 'can_prepare': False, 'can_start': False,
                         'prepare_blocked_reason': '红绿灯测试占用控制权；停止后可使用斑马线和遥控'}
            return {**base, 'crosswalk_trial': zebra, 'traffic_trial': trial, 'traffic_status_s': snapshot_s}

    def _cancel(self, reason, fault=False):
        self.phase, self.reason = ('fault' if fault else 'cancelled'), reason
        self.stop.set()

    def request(self, action, payload):
        if not isinstance(payload, dict):
            raise ValueError('JSON object required')
        with self.lock:
            if action == 'status':
                return self.status()
            if not action.startswith('traffic_'):
                if self.busy:
                    if action in ('stop', 'emergency', 'finish_test', 'crosswalk_cancel'):
                        reason = {'window_blur': '控制页失去焦点，已结束准备／测试；返回后请重新准备',
                                  'page_hidden': '控制页已切到后台，已结束准备／测试；返回后请重新准备',
                                  'connection_recovery': '控制页连接中断，已结束本轮；连接恢复后请重新准备'}.get(
                                      payload.get('stop_source'), '人工停止红绿灯测试')
                        self._cancel(reason)
                        return self.status()
                    raise ValueError('红绿灯测试占用控制权；先停止再使用遥控或斑马线')
                self.drive.request(action, payload)
                return self.status()
            if action == 'traffic_cancel':
                if self.busy:
                    self._cancel('用户停止红绿灯测试')
                return self.status()
            if self.closed:
                raise ValueError('服务已关闭')
            client = payload.get('client')
            if not isinstance(client, str) or not 1 <= len(client) <= 128:
                raise ValueError('控制页面身份无效')
            if action == 'traffic_settings':
                if payload.get('settings_revision') != self.settings_revision:
                    raise ValueError('参数已变化，请刷新后再保存')
                if self.busy:
                    raise ValueError('请停止红绿灯测试后再调整行驶参数')
                settings = self._updated(payload)
                saved = {k: settings[k] for k in ('max_approach_s', 'red_confirm_percent')}
                saved['forward_us'] = settings['pwm']['search_us']
                self.settings_path.parent.mkdir(parents=True, exist_ok=True)
                tmp = self.settings_path.with_suffix('.json.tmp')
                tmp.write_text(json.dumps(saved, indent=2), encoding='utf-8')
                tmp.replace(self.settings_path)
                self.settings, self.settings_error = settings, None
                self.settings_revision = secrets.token_hex(8)
                return self.status()
            if action == 'traffic_prepare':
                t = self.status()['traffic_trial']
                if not t['can_prepare']:
                    raise ValueError(self.settings_error or '请先停止当前测试，再准备红绿灯测试')
                if payload.get('setup_token') != self.drive.status().get('setup_token'):
                    raise ValueError('控制页面已变化，请刷新')
                if payload.get('esc_off') is not True or payload.get('fb_mode_confirmed') is not True:
                    raise ValueError('准备时电调需关闭，并保持 F/B 档')
                if t['can_end_manual']:
                    self.drive.request('finish_test', payload)
                self.client, self.token = client, secrets.token_hex(16)
                self.last_sequence, self.round_number = -1, 0
                self.active_settings = copy.deepcopy(self.settings)
                self.phase, self.reason = 'preparing', '正在检查双路实时画面并建立中位输出，无需看见灯具'
                self.controller = self.latest = self.observation = self.preview_jpeg = self.execution = None
                self.stop, self.busy = threading.Event(), True
                self.lease_until = self.clock()+.5
                self.folder = self.root/'run'/('traffic-console-'+str(time.time_ns()))
                self.thread = threading.Thread(target=self._work, name='traffic-console', daemon=True)
                self.thread.start()
                return self.status()
            if not self.busy or client != self.client or payload.get('run_token') != self.token:
                raise ValueError('本轮由另一页面持有，或本轮已结束')
            sequence = payload.get('trial_sequence')
            if type(sequence) is not int or sequence <= self.last_sequence:
                raise ValueError('old or invalid traffic sequence')
            if action == 'traffic_start':
                if not self.status()['traffic_trial']['can_start']:
                    raise ValueError(self.status()['traffic_trial']['start_blocked_reason'])
                if any(payload.get(k) is not True for k in ('fb_mode_confirmed', 'esc_on_static', 'field_ready')):
                    raise ValueError('开始需 F/B 档、电调打开且静止、场地就绪')
                self.controller = TrafficDrivingController(self.active_settings, self.profile)
                if self.speech:
                    self.speech.close()
                self.speech = self.speech_factory()
                self.round_number += 1
                self.phase, self.reason = 'arming', '3 秒中位初始化，随后执行红灯停车；绿灯只播报'
                self.latest = None
            elif action != 'traffic_keepalive':
                raise ValueError('未知红绿灯动作')
            self.last_sequence, self.lease_until = sequence, self.clock()+.5
            return self.status()

    def _work(self):
        actuator = None
        try:
            self.folder.mkdir(parents=True, exist_ok=False)
            (self.folder/'settings.json').write_text(json.dumps({'driving': self.active_settings, 'vision': self.profile,
                'camera_selection': {'decision_camera': self.active_settings['camera'],
                    'profile_recommended_camera': self.profile['recommended_camera'],
                    'reason': 'both use original classifier; independent 2s red/green votes; any camera qualifies; red priority'}}, ensure_ascii=False, indent=2), encoding='utf-8')
            deadline = self.clock()+5
            while True:
                s = self.drive.status()
                if (s.get('mode') == 'disabled' and s.get('boot_test_phase') == 'esc_off'
                        and not s.get('hardware_output') and not s.get('worker_started')
                        and not s.get('crosswalk_trial', {}).get('active')):
                    break
                if self.stop.is_set() or self.clock() > self.lease_until or self.clock() >= deadline:
                    raise ValueError('原控制尚未释放或页面心跳中断')
                self.stop.wait(.01)
            reader = self.reader_factory(self.states, self.active_settings, self.profile, self.clock)
            first = reader.read(self.clock())
            if first is None:
                raise ValueError('等待至少一路新鲜图像')
            if self.stop.is_set() or self.clock() > self.lease_until:
                raise ValueError('准备已取消或页面心跳中断')
            actuator = self.actuator_factory()
            prepared = self.clock()
            active_since = None
            with (self.folder/'decisions.jsonl').open('w', encoding='utf-8') as log:
                while not self.stop.is_set():
                    now = self.clock()
                    if now > self.lease_until:
                        raise ValueError('页面心跳中断，结束本轮并制动回中位')
                    if self.phase in ('preparing', 'ready', 'complete') and now-prepared > 120:
                        raise ValueError('等待开始超时，请重新准备')
                    sample = reader.read(now)
                    if sample is None:
                        self.stop.wait(.005)
                        continue
                    now = self.clock()
                    with self.lock:
                        if self.stop.is_set():
                            break
                        if now > self.lease_until or not 0 <= now-sample['captured_s'] <= self.active_settings['max_frame_age_s']:
                            raise ValueError('处理后画面或控制心跳已过期')
                        self.observation, self.preview_jpeg = sample, reader.preview
                        if self.controller is None or self.phase == 'complete':
                            if self.phase != 'complete':
                                self.phase, self.reason = 'ready', '已准备；打开电调并确认静止后开始，无需看见灯具'
                            intent = {'phase': self.phase, 'action': 'neutral', 'esc_us': 1500,
                                      'steering_us': 1610, 'events': []}
                            active_since = None
                        else:
                            if active_since is None:
                                active_since = now
                            intent = self.controller.step(sample, now)
                            if now-active_since > 90:
                                intent = self.controller.fault('本轮总时限已到，停止本轮')
                            self.phase, self.reason = intent['phase'], intent['reason']
                        self.latest = intent
                        if self.phase == 'fault':
                            self.stop.set()
                            self.execution = {'failsafe_close_requested': True}
                        else:
                            self.execution = actuator.send(intent)
                        if any(e.get('event') == 'speak' for e in intent['events']) and self.speech:
                            self.speech.start()
                        log.write(json.dumps({'now_s': now, 'round': self.round_number, 'sample': sample,
                                              'intent': intent, 'execution': self.execution,
                                              'speech': self.speech.status() if self.speech else None}, ensure_ascii=False, allow_nan=False)+'\n')
                        log.flush()
                        for event in intent['events']:
                            evidence_frames = getattr(reader,'jpegs',{})
                            for camera, jpeg in evidence_frames.items():
                                evidence = self.folder/f"round-{self.round_number:03d}-{event['event']}.{camera}.jpg"
                                if jpeg and not evidence.exists(): evidence.write_bytes(jpeg)
                        if self.phase == 'done':
                            self.phase, self.reason = 'complete', '已确认绿灯并请求播报“红绿灯结束，开始前行”；保持停车'
                            prepared = now
                        if self.phase == 'fault':
                            break
                    self.stop.wait(.005)
        except Exception as exc:
            with self.lock:
                self._cancel(str(exc), fault=True)
        finally:
            try:
                if actuator:
                    actuator.close()
            finally:
                if self.speech:
                    self.speech.close()
                with self.lock:
                    self.busy = False
                    if self.folder and self.folder.is_dir():
                        try:
                            (self.folder/'summary.json').write_text(json.dumps({'phase': self.phase, 'reason': self.reason,
                                'speech': self.speech.status() if self.speech else None,
                                'physical_stop_verified': False, 'parking_zone_verified': False}, ensure_ascii=False, indent=2), encoding='utf-8')
                        except OSError:
                            pass

    def close(self):
        with self.lock:
            self.closed = True
            if self.busy:
                self._cancel('服务停止')
            thread = self.thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=4)
        self.drive.close()
