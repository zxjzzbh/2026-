"""Real-camera F/B parking trial for the existing car console.

The preview service remains the sole camera owner. Parking and the original
manual driver take UART ownership separately; mode changes never start motion.
"""
import json
import copy
import secrets
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .brake_output import BrakeActuator
from .brake_parking import BrakeParkingController, GroundMotionEstimate, load_settings, numeric, brake_trigger_reference
from .config import load_config
from .crosswalk import CrosswalkDetector
from .lane import LaneDetector
from .parking_speech import ParkingSpeech


class TrialCamera:
    def __init__(self, states, settings):
        self.state = states[settings['camera']]
        self.settings = settings
        self.last_id = None
        self.detector = CrosswalkDetector()
        self.lane = LaneDetector(load_config(None).lane)
        self.motion = GroundMotionEstimate()
        self.jpeg = None

    def read(self, now):
        state = self.state
        with state.condition:
            jpeg, frame_id, captured = state.raw_jpeg, state.raw_frame_id, state.raw_received_ms
            failed = state.state in ('error', 'stopped')
        if (failed or jpeg is None or not numeric(captured)
                or not 0 <= now - captured / 1000 <= self.settings['max_frame_age_s']):
            raise ValueError('第二路摄像头画面失效')
        if frame_id == self.last_id:
            return None  # Do not refresh the actuator lease with an old image.
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None or [image.shape[1], image.shape[0]] != self.settings['image_size']:
            raise ValueError('第二路画面尺寸与原标定不一致')
        zebra, _ = self.detector.detect(image, captured)
        clean = image.copy()
        for x, y, w, h in zebra['stripes_xywh']:
            clean[y:y+h, x:x+w] = 0
        lane, _ = self.lane.detect(clean)
        self.last_id, self.jpeg = frame_id, jpeg
        return {'fresh': True, 'frame_id': frame_id, 'captured_s': captured / 1000,
                'image_size': self.settings['image_size'], 'candidate': zebra['candidate'],
                'far_edge_y_normalized': zebra['far_edge_y_normalized'],
                'lane_valid': lane.valid, 'lane_offset': lane.offset_normalized,
                'motion': self.motion.update(image, frame_id, captured / 1000)}


class BrakeTrialDrive:
    """Explicit prepare/start; 500-ms page lease and independent PWM lease."""
    def __init__(self, drive, states, config, root, *, actuator_factory=None,
                 reader_factory=TrialCamera, speech_factory=None, clock=time.monotonic):
        self.drive, self.states = drive, states
        self.settings = load_settings(config)
        self.root, self.clock = Path(root), clock
        self.settings_path = self.root/'vision/configs/brake-parking.local.json'
        self.settings_error = None
        self.settings_revision = secrets.token_hex(8)
        if self.settings_path.exists():
            try:
                saved = json.loads(self.settings_path.read_text(encoding='utf-8'))
                candidate = {**self.settings, 'brake_trigger_distance_m': saved['brake_trigger_distance_m']}
                brake_trigger_reference(candidate)
                self.settings = candidate
            except (OSError, ValueError, KeyError, TypeError) as exc:
                self.settings_error = '已保存距离无效，请在页面重新保存：'+str(exc)
        self.brake_reference = brake_trigger_reference(self.settings)
        self.active_settings = copy.deepcopy(self.settings)
        self.round_number = 0
        self.actuator_factory = actuator_factory or (
            lambda: BrakeActuator(self.active_settings, run=True, esc_mode='F/B'))
        self.reader_factory = reader_factory
        self.speech_factory = speech_factory or (lambda: ParkingSpeech(self.root, text=self.active_settings['speech_text']))
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.thread = None
        self.closed = self.busy = False
        self.phase, self.reason = 'idle', '电调保持关闭，先准备 F/B 斑马线测试'
        self.client = self.token = self.folder = None
        self.last_sequence = -1
        self.lease_until = 0
        self.arm_until = self.started = None
        self.path_mode = 'straight'
        self.latest = self.speech = self.controller = None

    def status(self):
        with self.lock:
            base = self.drive.status()
            manual_idle = (base.get('boot_test_phase') == 'esc_off' and base.get('mode') == 'disabled'
                           and not base.get('hardware_output') and not base.get('worker_started'))
            can_end_manual = not self.busy and base.get('boot_test_phase') == 'ready'
            blocked_reason = ('' if self.busy or manual_idle or can_end_manual else
                              '原遥控仍在准备或故障状态；先结束准备或关闭电调后重置')
            blocked_reason = self.settings_error or blocked_reason
            trial = {'program': 'fb-brake-v1', 'active': self.busy,
                     'phase': self.phase, 'reason': self.reason, 'owner_client': self.client,
                     'run_token': self.token, 'can_prepare': not self.busy and (manual_idle or can_end_manual) and not self.closed and not self.settings_error,
                     'can_end_manual': can_end_manual, 'prepare_blocked_reason': blocked_reason,
                     'can_start': self.busy and self.phase in ('ready', 'complete'), 'esc_mode': 'F/B',
                     'camera': 'secondary', 'brake_trigger': self.brake_reference,
                     'running_brake_trigger': brake_trigger_reference(self.active_settings),
                     'settings_revision': self.settings_revision, 'settings_error': self.settings_error,
                     'round_number': self.round_number, 'hold_s': self.settings['hold_s'],
                     'speech_text': self.settings['speech_text'],
                     'distance_limits_cm': [25,65],
                     'reference_summary': f"H65 第二路：65 / 50 / 25 cm 标定保留；约 {self.brake_reference['distance_m']*100:g} cm 触发制动（原标定插值）",
                     'arming_remaining_s': max(0, self.arm_until-self.clock()) if self.phase == 'arming' else 0,
                     'intent': self.latest, 'speech': self.speech.status() if self.speech else None,
                     'log_directory': str(self.folder) if self.folder else None,
                     'tuning_verified': False, 'physical_stop_verified': False}
            # F/B is a hardware mode: expose its capabilities to every manual
            # input path, and also reject reverse server-side below.
            return {**base, 'reverse_available': False, 'esc_mode': 'F/B',
                    'manual_reverse_disabled_reason': 'F/B 档没有倒车功能', 'crosswalk_trial': trial}

    def _cancel(self, reason, fault=False):
        self.phase, self.reason = ('fault' if fault else 'cancelled'), reason
        self.stop.set()

    def request(self, action, payload):
        if not isinstance(payload, dict):
            raise ValueError('JSON object required')
        with self.lock:
            if action == 'status':
                return self.status()
            if not action.startswith('crosswalk_'):
                if self.busy:
                    if action in ('stop', 'emergency', 'finish_test'):
                        self._cancel('人工停止斑马线测试')
                        return self.status()
                    raise ValueError('斑马线测试占用控制权；先停止本轮再遥控')
                if action == 'command' and payload.get('motor') == 'reverse':
                    self.drive.request('stop', {'stop_source': 'fb_reverse_rejected'})
                    raise ValueError('当前 F/B 档禁止倒车；未发送倒车或刹车替代指令')
                self.drive.request(action, payload)
                return self.status()
            if action == 'crosswalk_cancel':
                if self.busy:
                    self._cancel('用户取消斑马线测试')
                return self.status()
            if self.closed:
                raise ValueError('服务已停止')
            client = payload.get('client')
            if not isinstance(client, str) or not 1 <= len(client) <= 128:
                raise ValueError('控制页面身份无效')
            if action == 'crosswalk_settings':
                if payload.get('settings_revision') != self.settings_revision:
                    raise ValueError('距离已被其他操作更新，请读取当前值后再保存')
                if self.busy and client != self.client:
                    raise ValueError('请在持有本轮控制权的页面调整距离')
                cm = payload.get('brake_distance_cm')
                if not numeric(cm) or not 25 <= cm <= 65:
                    raise ValueError('现有标定支持 25–65 cm；超出范围需要补充实测标定')
                updated = {**self.settings, 'brake_trigger_distance_m': cm/100}
                trigger = brake_trigger_reference(updated)
                self.settings_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.settings_path.with_suffix('.json.tmp')
                temporary.write_text(json.dumps({'schema_version':1,'brake_trigger_distance_m':cm/100},indent=2),encoding='utf-8')
                temporary.replace(self.settings_path)
                self.settings, self.brake_reference = updated, trigger
                self.settings_error = None
                self.settings_revision = secrets.token_hex(8)
                return self.status()
            if action == 'crosswalk_prepare':
                if not self.status()['crosswalk_trial']['can_prepare']:
                    raise ValueError('先结束原遥控控制，并等待电调关闭准备状态')
                if payload.get('setup_token') != self.drive.status().get('setup_token'):
                    raise ValueError('控制页面已变化，请刷新')
                if payload.get('esc_off') is not True or payload.get('fb_mode_confirmed') is not True:
                    raise ValueError('准备需要电调关闭并确认实物 F/B 档')
                if self.status()['crosswalk_trial']['can_end_manual']:
                    self.drive.request('finish_test',payload)
                self.client, self.token = client, secrets.token_hex(16)
                self.last_sequence = -1
                self.phase, self.reason = 'preparing', '正在建立中位输出，请保持电调关闭'
                self.stop = threading.Event()
                self.busy = True
                self.lease_until = self.clock() + .5
                self.arm_until = self.started = self.latest = self.controller = None
                self.active_settings = copy.deepcopy(self.settings)
                self.round_number = 0
                self.folder = self.root/'run'/('brake-console-'+str(time.time_ns()))
                self.thread = threading.Thread(target=self._work, name='brake-console', daemon=True)
                self.thread.start()
                return self.status()
            if (not self.busy or client != self.client or payload.get('run_token') != self.token):
                raise ValueError('本轮由另一页面持有，或本轮已经结束')
            sequence = payload.get('trial_sequence')
            if type(sequence) is not int or sequence <= self.last_sequence:
                raise ValueError('old or invalid brake sequence')
            if action == 'crosswalk_start':
                if self.phase not in ('ready', 'complete'):
                    raise ValueError('先等待准备完成或上一轮停车结束')
                if (payload.get('fb_mode_confirmed') is not True or payload.get('esc_on_static') is not True
                        or payload.get('field_ready') is not True):
                    raise ValueError('需要 F/B 档、电调已开且静止、场地就绪')
                mode = payload.get('path_mode', 'straight')
                if mode not in ('straight', 'lane'):
                    raise ValueError('路径模式无效')
                self.path_mode = mode
                self.active_settings = copy.deepcopy(self.settings)
                self.controller = None
                self.round_number += 1
                if self.speech:
                    self.speech.close()
                self.speech = self.speech_factory()
                self.arm_until = self.clock() + self.settings['arming_s']
                self.phase, self.reason = 'arming', '保持中位初始化，倒计时结束后自动前进'
            elif action != 'crosswalk_keepalive':
                raise ValueError('旧固定回零测试动作已停用')
            self.last_sequence = sequence
            self.lease_until = self.clock() + .5
            return self.status()

    def _work(self):
        actuator = None
        try:
            self.folder.mkdir(parents=True, exist_ok=False)
            (self.folder/'settings.json').write_text(json.dumps(self.settings, ensure_ascii=False, indent=2), encoding='utf-8')
            release_deadline = self.clock()+5
            while True:
                state = self.drive.status()
                if (state.get('boot_test_phase') == 'esc_off' and state.get('mode') == 'disabled'
                        and not state.get('hardware_output') and not state.get('worker_started')):
                    break
                if self.stop.is_set() or self.clock() > self.lease_until:
                    raise ValueError('准备已取消或页面心跳中断')
                if self.clock() >= release_deadline:
                    raise ValueError('原遥控尚未释放串口，请结束控制后重试')
                self.stop.wait(.01)
            reader = self.reader_factory(self.states, self.settings)
            # A broken/mismatched camera must be caught before UART acquisition.
            first = reader.read(self.clock())
            if first is None:
                raise ValueError('尚无新的第二路图像')
            actuator = self.actuator_factory()
            self.speech = self.speech_factory()
            prepared_at = self.clock()
            logged_round = -1
            with (self.folder/'observations.jsonl').open('w', encoding='utf-8') as observations, \
                    (self.folder/'decisions.jsonl').open('w', encoding='utf-8') as log:
                # Worker startup/readback can outlast a frame's lease. Keep
                # the preflight check, but consume a new frame after startup.
                sample = None
                while not self.stop.is_set():
                    now = self.clock()
                    with self.lock:
                        if now > self.lease_until:
                            raise ValueError('控制页心跳中断，已结束本轮')
                        if self.phase in ('preparing', 'ready', 'complete') and now-prepared_at > 120:
                            raise ValueError('等待开始超时，请重新准备')
                    if sample is None:
                        sample = reader.read(now)
                    if sample is None:
                        self.stop.wait(.005)
                        continue
                    now = self.clock()  # Include vision processing time.
                    with self.lock:
                        if self.stop.is_set():
                            break
                        if now > self.lease_until:
                            raise ValueError('控制页心跳中断，已结束本轮')
                        if not 0 <= now-sample['captured_s'] <= self.settings['max_frame_age_s']:
                            raise ValueError('视觉处理后图像已过期')
                        if self.phase in ('preparing', 'ready'):
                            self.phase, self.reason = 'ready', '中位已准备；打开电调并确认静止后点击开始'
                            intent = {'phase': 'ready', 'action': 'neutral', 'esc_us': 1500,
                                      'steering_us': 1610, 'events': []}
                        elif self.phase == 'complete':
                            intent = {'phase': 'complete', 'action': 'neutral', 'esc_us': 1500,
                                      'steering_us': 1610, 'events': []}
                        elif self.phase == 'arming' and now < self.arm_until:
                            intent = {'phase': 'arming', 'action': 'neutral', 'esc_us': 1500,
                                      'steering_us': 1610, 'events': []}
                        else:
                            if self.controller is None:
                                self.controller = BrakeParkingController(self.active_settings, self.path_mode)
                                self.started = now
                            intent = self.controller.step(sample, now)
                            self.phase, self.reason = intent['phase'], intent['reason']
                        self.latest = intent
                        if self.round_number != logged_round:
                            (self.folder/f'round-{self.round_number:03d}.settings.json').write_text(
                                json.dumps(self.active_settings,ensure_ascii=False,indent=2),encoding='utf-8')
                            logged_round = self.round_number
                        if intent['phase'] == 'fault':
                            self.stop.set()
                            execution = {'failsafe_close_requested': True}
                        else:
                            execution = actuator.send(intent)
                        if any(e.get('event') == 'speak' for e in intent['events']):
                            self.speech.start()
                        for stream, row in [(observations, {'now_s': now, 'sample': sample, 'round': self.round_number}),
                                            (log, {'now_s': now, 'intent': intent, 'execution': execution, 'round': self.round_number,
                                                   'speech': self.speech.status()})]:
                            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
                            stream.flush()
                        if intent['phase'] in ('brake_pulse', 'hold') and reader.jpeg:
                            evidence = self.folder/(f'round-{self.round_number:03d}-'+intent['phase']+'.jpg')
                            if not evidence.exists():
                                evidence.write_bytes(reader.jpeg)
                        if self.phase == 'done':
                            self.phase, self.reason = 'complete', '10 秒停车结束，保持中位；可调整距离并直接开始下一轮'
                            prepared_at = now
                        if self.phase == 'fault':
                            break
                    sample = None
                    self.stop.wait(.005)
        except Exception as exc:
            with self.lock:
                self._cancel(str(exc), fault=True)
        finally:
            if actuator:
                actuator.close()  # Finite brake if driving, then neutral.
            if self.speech:
                self.speech.close()
            with self.lock:
                self.busy = False
                if self.folder and self.folder.is_dir():
                    result = {'phase': self.phase, 'reason': self.reason, 'tuning_verified': False,
                              'physical_stop_verified': False, 'parking_zone_verified': False,
                              'speech': self.speech.status() if self.speech else None}
                    try:
                        (self.folder/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
                    except OSError:
                        pass  # Output was closed before attempting the log.

    def close(self):
        with self.lock:
            self.closed = True
            if self.busy:
                self._cancel('服务停止')
            thread = self.thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=4)
