"""Exclusive experimental parking-entry session with independent output leases.

Preparation opens no hardware. A fresh secondary-camera tick may acquire S3/S4
and maintain neutral; only an authenticated explicit start creates the entry
controller. Completion is an image-based estimate, never four-wheel validation.
"""
import copy
import json
import math
import secrets
import time
from pathlib import Path

from .cone_trial_output import ConeTrialActuator
from .parking_entry import ParkingEntryController
from .steering import LEFT_US, RIGHT_US


EVIDENCE_SELECTION_SOURCES = frozenset(('layout', 'blocked_side', 'visible_ground', 'blue_board_position'))


def parking_base_idle(base):
    """Observer-only sessions are allowed; every motor owner must be released."""
    return (isinstance(base, dict) and base.get('mode') == 'disabled'
            and base.get('boot_test_phase') == 'esc_off'
            and not any(base.get(k) for k in ('hardware_output', 'worker_started', 'pending_hardware_start'))
            and not any((base.get(k) or {}).get('active') for k in (
                'traffic_trial', 'crosswalk_trial', 'departure_trial', 'race_flow', 'parking_trial'))
            and not any(((base.get('cone_trial') or {}).get(k) or {}).get('active')
                        for k in ('avoidance', 'short_test'))
            and not (base.get('parking_payment') or {}).get('motion_locked'))


def make_actuator(settings):
    return ConeTrialActuator(settings, right_us=RIGHT_US, left_us=LEFT_US,
                             run=True, esc_mode='F/B')


def _clean(value):
    """Diagnostics contain no page authority or non-finite JSON values."""
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()
                if k not in ('client', 'owner_client', 'run_token', 'setup_token', 'token')}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class ParkingEntryRun:
    def __init__(self, factory=None, log_root=None):
        self.factory = factory or make_actuator
        self.log_root = Path(log_root) if log_root else None
        self.active = False
        self.phase = 'idle'
        self.reason = '尚未准备实验停车入位'
        self.owner = self.token = None
        self.sequence = -1
        self.lease_until = self.prepared_s = 0.
        self.s = self.controller = self.sample = self.intent = self.execution = None
        self.actuator = self.folder = self.log = None
        self.last_record = None
        self.last_control_frame = None

    @staticmethod
    def _fresh(sample, now):
        if not isinstance(sample, dict) or sample.get('fresh') is not True:
            return False
        t = sample.get('captured_s')
        return (type(t) in (int, float) and math.isfinite(t) and 0 <= now - t <= .25
                and type(sample.get('frame_id')) is int
                and sample.get('image_size') == [480, 360]
                and sample.get('camera_id', 'secondary') == 'secondary')

    def status(self, base, now):
        ready = self.active and self.phase == 'ready'
        return {
            'program': 'experimental-parking-entry-v1',
            'active': self.active, 'phase': self.phase, 'reason': self.reason,
            'owner_client': self.owner, 'run_token': self.token,
            'can_prepare': not self.active and parking_base_idle(base),
            'can_request_start': ready,
            'can_start': ready and self._fresh(self.sample, now),
            'start_blocked_reason': '' if ready and self._fresh(self.sample, now)
                else '需要本页完成准备并取得第二路新鲜画面；不要求选区已连续确认',
            'selected_slot': (self.s or {}).get('selected_slot'),
            'prepared_slot': (self.s or {}).get('prepared_slot'),
            'selection_source': (self.s or {}).get('selection_source'),
            'selection_basis': (self.s or {}).get('selection_basis'),
            'selection_locked': self.controller is not None,
            'forward_us': (self.s or {}).get('forward_us', 1575),
            'steering_limits_us': [RIGHT_US, LEFT_US],
            'hardware_output': bool(self.actuator and getattr(self.actuator, 'run', False)),
            'intent': copy.deepcopy(self.intent),
            'field_verified': False, 'actual_pose_not_verified': True,
            'parking_zone_verified': False,
            'completion_basis': '视觉入位与光流停稳估计；不代表四轮物理入位验证',
            'log_directory': str(self.folder) if self.folder else None,
        }

    def prepare(self, parking_settings, brake_settings, cone_settings, base, payload, now, *, selection=None):
        if self.active or not parking_base_idle(base):
            raise ValueError('先结束遥控、斑马线、红绿灯、锥桶或完整流程，再准备停车入位')
        if not isinstance(payload, dict) or not base.get('setup_token') or payload.get('setup_token') != base['setup_token']:
            raise ValueError('页面已变化，请刷新')
        if payload.get('esc_off') is not True or payload.get('fb_mode_confirmed') is not True:
            raise ValueError('准备时电调关闭，并保持 F/B 挡位')
        client = payload.get('client')
        if not isinstance(client, str) or not 1 <= len(client) <= 128:
            raise ValueError('页面身份无效')
        side = payload.get('selected_slot')
        if side not in ('left', 'right'):
            raise ValueError('停车侧应为当前选出的 left 或 right')
        forward = payload.get('forward_us', 1575)
        if type(forward) is not int or not 1501 <= forward <= 1625:
            raise ValueError('停车前进油门仅允许 1501–1625 μs 整数')
        gain = payload.get('steering_gain', 1.)
        if type(gain) not in (int, float) or not math.isfinite(gain) or not 1 <= gain <= 3:
            raise ValueError('停车转向增益仅允许 1.0–3.0')
        parking, brake, cone = map(copy.deepcopy, (parking_settings, brake_settings, cone_settings))
        if parking.get('camera') != 'secondary' or parking.get('image_size') != [480, 360]:
            raise ValueError('停车使用第二路 480×360 实际画面')
        brake.update(arming_s=3., brake_pulse_s=.12, brake_release_s=.1, actuator_lease_s=.2)
        brake['pwm'] = {'neutral_us': 1500, 'steering_center_us': 1610,
                        'search_us': forward, 'creep_us': forward, 'brake_us': 1400}
        candidate = {'parking': parking, 'brake': brake, 'cone': cone,
                     'selected_slot': side, 'prepared_slot': side, 'forward_us': forward, 'steering_gain': gain,
                     'selection_source': 'prepared_side', 'selection_basis': '准备时选定方向'}
        # Metadata is supplied by the observer, never accepted from page JSON.
        if isinstance(selection, dict) and selection.get('selected_slot') == side:
            source = selection.get('suggestion_source')
            if source in EVIDENCE_SELECTION_SOURCES | {'history', 'default_right'}:
                candidate['selection_source'] = source
                candidate['selection_basis'] = selection.get('suggestion_basis') or '准备时选定方向'
        candidate['prepared_selection_source'] = candidate['selection_source']
        # Validate controller inputs before reserving a session; its constructor
        # is pure and does not start arming until the first step after start.
        self._controller(candidate)
        self.s = candidate
        self.owner, self.token = client, secrets.token_hex(16)
        self.sequence, self.lease_until, self.prepared_s = -1, now + .5, now
        self.active, self.phase = True, 'preparing'
        self.reason = '等待第二路新鲜停车画面；准备期间仅保持中位'
        self.controller = self.sample = self.intent = self.execution = None
        self.last_control_frame = None
        self.folder = self.log = self.last_record = None
        if self.log_root:
            try:
                self.folder = self.log_root / ('parking-entry-' + str(time.time_ns()))
                self.folder.mkdir(parents=True, exist_ok=False)
                (self.folder / 'settings.json').write_text(
                    json.dumps(_clean(self.s), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
                self.log = (self.folder / 'decisions.jsonl').open('w', encoding='utf-8')
            except Exception:
                self.cancel('无法创建停车入位日志', phase='fault')
                raise

    @staticmethod
    def _controller(settings):
        p = settings['parking']
        ground = settings['cone'].get('ground') or p.get('ground')
        return ParkingEntryController(
            settings['brake'], selected_side=settings['selected_slot'],
            forward_us=settings['forward_us'], steering_gain=settings['steering_gain'],
            vehicle_width_m=p['vehicle_width_m'], front_extent_m=p['front_extent_m'],
            rear_extent_m=p['rear_extent_m'], zone_length_m=p['zone_length_m'],
            half_width_m=p['lane_width_m'] / 2, ground=ground)

    @staticmethod
    def _fresh_selection(selection, now):
        """Only server observer evidence may revise a not-yet-started target."""
        if not isinstance(selection, dict) or selection.get('stale') is True:
            return None
        t = selection.get('captured_s')
        if (selection.get('selected_slot') not in ('left', 'right')
                or selection.get('suggestion_source') not in EVIDENCE_SELECTION_SOURCES
                or type(t) not in (int, float) or not math.isfinite(t) or not 0 <= now-t <= .25):
            return None
        return selection

    def command(self, action, payload, now, *, start_selection=None):
        if not isinstance(payload, dict) or not self.active or payload.get('client') != self.owner or payload.get('run_token') != self.token:
            raise ValueError('本页未持有当前停车入位控制权')
        if now > self.lease_until:
            self.cancel('页面心跳超时，重新准备停车入位', phase='fault')
            raise ValueError(self.reason)
        sequence = payload.get('trial_sequence')
        if type(sequence) is not int or sequence <= self.sequence:
            raise ValueError('停车请求序号无效或重复')
        if action == 'parking_trial_start':
            if self.phase != 'ready' or not self._fresh(self.sample, now):
                raise ValueError('准备尚未完成或第二路画面已过期')
            if any(payload.get(k) is not True for k in ('fb_mode_confirmed', 'esc_on_static', 'field_ready')):
                raise ValueError('开始前需 F/B、电调打开且静止、场地就绪')
            candidate = copy.deepcopy(self.s)
            selection = self._fresh_selection(start_selection, now)
            if selection is not None:
                candidate.update(selected_slot=selection['selected_slot'],
                    selection_source=selection['suggestion_source'],
                    selection_basis=selection.get('suggestion_basis') or '开始前最新视觉方向证据',
                    selection_captured_s=selection['captured_s'])
            controller = self._controller(candidate)
            if self.folder:
                (self.folder / 'settings.json').write_text(
                    json.dumps(_clean(candidate), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
                (self.folder / 'start-selection.json').write_text(json.dumps(_clean({
                    'now_s': now, 'prepared_slot': candidate['prepared_slot'],
                    'selected_slot': candidate['selected_slot'], 'selection_source': candidate['selection_source'],
                    'selection_basis': candidate['selection_basis'],
                    'selection_captured_s': candidate.get('selection_captured_s'),
                    'selection_locked': True, 'occupancy_verified': False}),
                    ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
            self.s, self.controller = candidate, controller
            self.last_control_frame = None
            self.phase, self.reason = 'arming', '3 秒中位后低速驶向选定停车半区'
        elif action == 'parking_trial_cancel':
            self.sequence = sequence
            self.cancel('已手动停止停车入位')
            return
        elif action != 'parking_trial_keepalive':
            raise ValueError('未知停车入位请求')
        self.sequence, self.lease_until = sequence, now + .5

    def tick(self, sample, now):
        if not self.active:
            return
        self.sample = sample
        try:
            if now > self.lease_until:
                raise ValueError('控制页心跳中断，已结束停车入位')
            if self.controller is None and now - self.prepared_s > 120:
                raise ValueError('准备等待超时')
            fresh = self._fresh(sample, now)
            if self.actuator is None:
                if not fresh:
                    if now - self.prepared_s > 5:
                        raise ValueError('等待第二路新鲜画面超时，停车准备未完成')
                    return
                token = self.token
                handle = self.factory(self.s['brake'])
                if not self.active or self.token != token:
                    handle.close()
                    return
                self.actuator = handle
            if not fresh:
                raise ValueError('第二路画面中断或过期，已结束停车入位')
            if self.controller is None:
                self.phase, self.reason = 'ready', '准备完成；打开电调且保持静止后点击开始入位'
                intent = {'action': 'neutral', 'esc_us': 1500, 'steering_us': 1610,
                          'phase': self.phase, 'reason': self.reason}
            else:
                key = (sample['frame_id'], sample['captured_s'])
                # At up to 250 ms camera intervals the output child still
                # requires commands every <200 ms. Repeat only the last intent
                # while that exact frame remains fresh; do not accumulate
                # stationary/entry evidence or extend PulseGuard's brake pulse.
                if key == self.last_control_frame:
                    intent = self.intent
                else:
                    intent = self.controller.step(sample, now)
                    self.last_control_frame = key
                self.phase, self.reason = intent['phase'], intent['reason']
            self.intent = intent
            handle = self.actuator
            if not self.active or handle is None:
                return
            self.execution = handle.send(intent)
            key = (sample.get('frame_id'), sample.get('captured_s'), self.phase, intent['action'])
            if self.log and not self.log.closed and key != self.last_record:
                self.log.write(json.dumps(_clean({'now_s': now, 'intent': intent, 'sample': sample,
                                                 'execution': self.execution}), ensure_ascii=False, allow_nan=False) + '\n')
                self.log.flush()
                self.last_record = key
            if self.phase in ('done', 'fault'):
                self.cancel(self.reason, phase=self.phase)
        except Exception as exc:
            if self.active:
                self.cancel(str(exc), phase='fault')

    def cancel(self, reason, phase='cancelled'):
        try:
            if self.actuator:
                # The existing child process applies a bounded F/B failsafe if
                # it was driving; repeated parent calls cannot extend it.
                self.actuator.close()
        finally:
            self.actuator = None
            self.active, self.phase, self.reason = False, phase, reason
            self.token = None
            if self.log and not self.log.closed:
                self.log.close()
            if self.folder:
                try:
                    summary = {'phase': phase, 'reason': reason, 'field_verified': False,
                               'actual_pose_not_verified': True, 'parking_zone_verified': False,
                               'failure_sample': self.sample if phase == 'fault' else None,
                               'last_requested_intent': self.intent}
                    (self.folder / 'summary.json').write_text(
                        json.dumps(_clean(summary), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
                except (OSError, TypeError, ValueError):
                    pass
