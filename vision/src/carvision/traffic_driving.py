"""Traffic stage: bounded forward -> F/B brake -> actual green -> speech only.

This supervised field-test controller does not pretend the external light
trigger or a front-wheel position sensor is connected. Ground optical flow is
only a stationary estimate. Competition-zone compliance remains unverified.
No hardware, network, sleeping or wall-clock countdown exists in this core.
"""
import copy
from .runtime_timing import VISION_MAX_AGE_S, MOTION_MAX_AGE_S, MOTION_RECOVERY_S
import json
from pathlib import Path

from .brake_parking import numeric
from .traffic_voting import SignalFrameVote, DualSignalVotes, FAST_RED_DEFAULTS
from .traffic_tracking import TrafficApproachGuard
from .traffic_stop_motion import TrafficStopMotion

# Compatibility for existing red-only replay scripts.
RedFrameVote = SignalFrameVote

TRAFFIC_FORWARD_MIN_US = 1570
TRAFFIC_STORED_MIN_US = 1501  # Read historical tuning files; effective output is floored below.
TRAFFIC_FORWARD_MAX_US = 1625


def approach_settings(settings):
    """One traffic-only speed ceiling for standalone and whole-route use."""
    s = copy.deepcopy(settings)
    s['max_frame_age_s']=VISION_MAX_AGE_S
    s['actuator_lease_s']=VISION_MAX_AGE_S
    limit = s.get('approach_speed_limit_us', 1570)
    if type(limit) is not int or not TRAFFIC_FORWARD_MIN_US <= limit <= 1575:
        raise ValueError('红绿灯低速接近上限必须为 1501–1575 μs')
    s['pwm']['search_us'] = s['pwm']['creep_us'] = min(max(TRAFFIC_FORWARD_MIN_US,s['pwm']['search_us']), limit)
    return s


def validate_settings(s):
    if (s.get('schema_version') != 1 or s.get('required_esc_mode') != 'F/B'
            or s.get('camera') != 'dual' or s.get('cameras') != ['primary', 'secondary'] or s.get('motion_camera') != 'secondary'
            or s.get('image_size') != [480, 360]):
        raise ValueError('红绿灯使用双路 480×360、第二路地面反馈、F/B 电调模式')
    p = s.get('pwm', {})
    if (p.get('neutral_us') != 1500 or p.get('steering_center_us') != 1610
            or any(type(p.get(k)) is not int for k in ('search_us', 'creep_us', 'brake_us'))
            or not TRAFFIC_STORED_MIN_US <= p['search_us'] <= TRAFFIC_FORWARD_MAX_US or p['creep_us'] != p['search_us']
            or p['brake_us'] != 1400):
        raise ValueError('使用已有中位和制动档；红绿灯接近 PWM 必须为 1501–1625 μs')
    limits = {'arming_s': (3, 3), 'max_frame_age_s': (.1, .5), 'actuator_lease_s': (.1, .5),
              'brake_pulse_s': (.35, .35), 'brake_release_s': (.1, .3),
              'brake_timeout_s': (1, 3), 'stop_confirm_s': (.5, 2),
              'max_verify_red_s': (1, 5),
              'max_approach_s': (.5, 10), 'max_wait_green_s': (15, 60), 'no_progress_s': (.5, 3),
              'red_window_s': (1, 1), 'green_window_s': (2, 2), 'red_confirm_percent': (1, 100)}
    for key, (low, high) in limits.items():
        if not numeric(s.get(key)) or not low <= s[key] <= high:
            raise ValueError('红绿灯参数范围错误：'+key)
    if type(s.get('max_brake_pulses')) is not int or not 1 <= s['max_brake_pulses'] <= 6:
        raise ValueError('制动脉冲最多 6 次')
    if type(s.get('fast_red_frames',3)) is not int or s.get('fast_red_frames',3) not in (2,3):
        raise ValueError('快速红灯停车需要连续 2 或 3 个有效新帧')
    for key in ('fast_red_max_span_s','fast_red_max_gap_s','fast_red_max_age_s'):
        value = s.get(key,FAST_RED_DEFAULTS[key])
        if not numeric(value) or not 0 < value <= .25:
            raise ValueError('快速红灯时间预算错误：'+key)
    approach_settings(s)
    if s.get('green_action') != 'speech_only' or s.get('speech_text') != '红绿灯结束，开始前行':
        raise ValueError('本轮绿灯只播报“红绿灯结束，开始前行”，不实际续行')
    return s


def load_settings(path):
    return validate_settings(json.loads(Path(path).read_text(encoding='utf-8-sig')))


class TrafficDrivingController:
    def __init__(self, settings, profile=None, *, control_interval_s=VISION_MAX_AGE_S):
        if control_interval_s is not None and (not numeric(control_interval_s) or not 0<control_interval_s<=VISION_MAX_AGE_S):
            raise ValueError('invalid control interval')
        self.control_interval_s = control_interval_s
        self.s = approach_settings(validate_settings(settings))
        self.approach_guard = TrafficApproachGuard(self.s)
        self.approach_evidence = None
        self.votes = DualSignalVotes(self.s['red_window_s'], self.s['red_confirm_percent'],
                                    self.s['max_frame_age_s'], green_window_s=self.s['green_window_s'],
                                    **{k:self.s.get(k,v) for k,v in FAST_RED_DEFAULTS.items()})
        self.signal_votes = None
        self.red_statistics = None
        self.phase, self.reason = 'arming', '中位初始化 3 秒'
        self.started = self.last_t = self.last_capture = self.last_frame = None
        self.approach_at = self.brake_at = self.pulse_until = self.release_until = None
        self.stop_since = self.wait_at = self.no_progress_since = None
        self.motion_unknown_since = None
        self.pending_fault_reason = None
        self.red_verification_deadline = None
        self.red_seen = False
        self.red_source_cameras = set()
        self.stop_signal_seen = False
        self.brake_count = 0
        self.raw_state = 'unknown'
        self.stable = {}
        self.fast_red_sources = self.fast_red_timeout_sources = []
        self.fast_red_trigger_sources = self.fast_red_timeout_trigger_sources = []
        self.fast_red_triggered = self.fast_red_timeout_triggered = False
        self.fast_red_timeout_reasons = {}
        self.early_stop_reason = None
        self.neutral_hold_since = None
        self.stop_sequence_complete = False
        self.stop_motion = TrafficStopMotion(self.s['stop_confirm_s'])
        self.motion_state = 'unknown'
        self.motion_reason = None

    def output(self, action='neutral', events=None):
        p = self.s['pwm']
        return {'phase': self.phase, 'reason': self.reason, 'action': action,
                'esc_us': {'neutral': 1500, 'search': p['search_us'], 'brake': p['brake_us']}[action],
                'steering_us': p['steering_center_us'], 'events': events or [],
                'signal_state': self.raw_state, 'stable': self.stable,
                'approach_guard':self.approach_evidence,
                'approach_forward_us':p['search_us'],
                'red_vote': self.red_statistics, 'signal_votes': self.signal_votes,
                'fast_red': {'required_frames':self.s.get('fast_red_frames',3),
                             'sources':list(self.fast_red_sources),
                             'triggered':self.fast_red_triggered,
                             'trigger_sources':list(self.fast_red_trigger_sources),
                             'timeout_triggered':self.fast_red_timeout_triggered,
                             'timeout_sources':list(self.fast_red_timeout_sources),
                             'timeout_trigger_sources':list(self.fast_red_timeout_trigger_sources),
                             'timeout_reasons':dict(self.fast_red_timeout_reasons)},
                'red_seen_this_round': self.red_seen, 'brake_pulses': self.brake_count,
                'max_brake_pulses': self.s['max_brake_pulses'],
                'red_sources_this_round':sorted(self.red_source_cameras),
                'brake_pulse_s': self.s['brake_pulse_s'],
                'green_action': 'speech_only',
                'stationary_estimated': self.stop_motion.stationary,
                'stop_sequence_complete': self.stop_sequence_complete,
                'stop_basis': ('visual_still' if self.stop_motion.stationary else 'brake_neutral_timing')
                              if self.stop_sequence_complete else None,
                'neutral_hold_required_s': self.s['stop_confirm_s'],
                'ground_motion_state': self.motion_state, 'ground_motion_reason': self.motion_reason,
                'ground_feedback_warning': self.motion_state == 'unknown' and self.phase in ('settle','verify_red','wait_green','done'),
                'sliding_unresolved': self.stop_motion.sliding,
                'supplementary_brake_evidence': self.stop_motion.status(),
                'red_verification_deadline_s':self.red_verification_deadline,
                'pending_fault_reason':self.pending_fault_reason,
                'physical_stop_verified': False, 'parking_zone_verified': False,
                'external_sensor_connected': False, 'field_verified': False}

    def fault(self, reason):
        self.phase, self.reason = 'fault', reason
        return self.output()

    def _brake(self, now, reason, *, repeat=False):
        if self.brake_count >= self.s['max_brake_pulses']:
            return self.fault('已达到本轮制动次数上限，保持中位并结束本轮')
        event = {'event': 'brake_requested', 'reason': reason}
        if repeat:
            event['motion_evidence'] = self.stop_motion.status()
        self.phase, self.reason = 'brake_pulse', reason
        self.brake_at = now
        self.stop_motion.begin_brake(preserve_slide=repeat)
        self.pulse_until = now + self.s['brake_pulse_s']
        self.release_until = None
        self.stop_since = self.neutral_hold_since = None
        self.stop_sequence_complete = False
        self.brake_count += 1
        if self.red_verification_deadline is None and not self.pending_fault_reason:
            self.red_verification_deadline = now+self.s['max_verify_red_s']
        self.votes.reset_green()
        self.stable = {}
        return self.output('brake', [event])

    def _finish_neutral_hold(self, now):
        if self.stop_motion.blocking:
            return False
        if self.neutral_hold_since is None:
            self.neutral_hold_since = now
        if now-self.neutral_hold_since >= self.s['stop_confirm_s']-1e-9:
            self.stop_sequence_complete = True
        return self.stop_sequence_complete

    def _stop_fault(self, now, reason):
        """Transmit a bounded brake before fault closes the trial actuator."""
        self.pending_fault_reason = self.pending_fault_reason or reason
        return self._brake(now, reason)

    def _begin_motion_recovery(self,now):
        if self.brake_count >= self.s['max_brake_pulses']:
            return self.fault('已达到本轮制动次数上限，保持中位并结束本轮')
        self.brake_count += 1
        self.phase='motion_recovery';self.motion_unknown_since=now
        self.motion_recovery_evidence_after=now
        self.motion_recovery_brake_until=now+self.s['brake_pulse_s']
        self.stop_since=None;self.recovery_motion_capture=None
        self.reason='运动反馈短暂失效，有限制动后等待恢复'
        self.votes.reset_green()
        return self.output('brake',[{'event':'motion_recovery_started'}])

    def _wait_for_green(self, now, events=None):
        # A real red cycle and completed brake/neutral sequence are required.
        # Timing completion is never labelled a physical stationary measurement.
        self.votes.begin_green_wait()
        self.stable = {}
        self.phase = 'wait_green'
        if self.wait_at is None:
            self.wait_at = now
        self.reason = '本轮红灯已确认，制动与中位保持已完成，等待同一灯具实际转绿'
        return self.output(events=events)

    def step(self, sample, now):
        if self.phase in ('done', 'fault'):
            return self.output()
        if (not numeric(now) or self.last_t is not None and
                (now <= self.last_t or self.control_interval_s is not None
                 and now-self.last_t > self.control_interval_s)):
            return self.fault('控制处理间隔失效，停止本轮')
        t, fid = sample.get('captured_s'), sample.get('frame_id')
        if (sample.get('fresh') is not True or not numeric(t) or not 0 <= now-t <= self.s['max_frame_age_s']
                or type(fid) is not int or self.last_frame is not None and fid <= self.last_frame
                or sample.get('image_size') != self.s['image_size']
                or sample.get('camera_id') != self.s['camera']):
            return self.fault('双摄批次重复、过期、尺寸或来源变化，停止本轮')
        if self.started is None:
            self.started = now
        self.last_t, self.last_capture, self.last_frame = now, t, fid
        # Decide the deadline before accepting this batch's evidence. A late
        # formal red may be logged, but cannot retroactively qualify the stop.
        if (self.red_verification_deadline is not None and now >= self.red_verification_deadline
                and not self.red_seen):
            self.pending_fault_reason = (self.pending_fault_reason or
                '制动后红灯复核超时，停止本轮；不把未知灯色当成等绿灯')
        observations = sample.get('camera_observations')
        if not isinstance(observations, dict):
            return self.fault('缺少带来源的双摄观测')
        observations, self.approach_evidence = self.approach_guard.update(
            observations, now, approaching=self.phase in ('arming','approach','motion_recovery'))
        self.signal_votes = self.votes.update(observations, now)
        if not self.signal_votes['any_fresh']:
            return self.fault('两路摄像头均无新鲜图像')
        self.raw_state = self.signal_votes['raw_state']
        self.fast_red_sources = self.signal_votes['fast_red_sources']
        self.fast_red_timeout_sources = self.signal_votes['fast_red_timeout_sources']
        if self.phase in ('arming','approach','motion_recovery'):
            guard_reason = self.approach_evidence['stop_reason']
            if guard_reason:
                self.stop_signal_seen = True
                self.early_stop_reason = {
                    'first_trustworthy_red':'当前新鲜灯具已可靠识别为红灯，首帧立即制动；停车后继续正式红灯复核',
                    'first_trustworthy_yellow':'当前新鲜灯具已可靠识别为黄灯，立即制动等待',
                }.get(guard_reason, '灯具接近保护请求停车')
            if self.fast_red_sources:
                self.fast_red_triggered = True
                self.fast_red_trigger_sources = list(self.fast_red_sources)
                self.stop_signal_seen = True
                if self.early_stop_reason is None:
                    self.early_stop_reason = f"同一灯具连续 {self.s.get('fast_red_frames',3)} 个新鲜红帧，快速制动；本轮红灯仍由 1 秒投票确认"
            if self.fast_red_timeout_sources:
                self.fast_red_timeout_triggered = True
                self.fast_red_timeout_trigger_sources = list(self.fast_red_timeout_sources)
                self.fast_red_timeout_reasons.update(self.signal_votes['fast_red_timeout_reasons'])
                self.stop_signal_seen = True
                if self.early_stop_reason is None:
                    self.early_stop_reason = '红灯候选超过快速确认时间预算，保守制动；不视为本轮红灯已确认'
        cycle_green = self.red_source_cameras.intersection(self.signal_votes['current_green_sources'])
        self.stable = {'state':self.signal_votes['state'],
                       'green_confirmed':(self.signal_votes['green_confirmed'] and bool(cycle_green)
                                          and not self.approach_evidence['current_stop_color_sources']),
                       'cycle_green_sources':sorted(cycle_green)}
        qualified = [self.signal_votes['cameras'][c] for c in self.signal_votes['red_sources']]
        eligible = qualified or [v for v in self.signal_votes['cameras'].values() if v['valid']]
        self.red_statistics = max(eligible, key=lambda row:row['red_percent'])
        if self.signal_votes['red_confirmed']:
            self.red_seen = True
            self.red_source_cameras.update(self.signal_votes['red_sources'])
            self.stop_signal_seen = True
        if self.signal_votes['yellow_present']:
            self.stop_signal_seen = True
        motion = sample.get('motion') or {}
        mt = motion.get('captured_s')
        motion_fresh = (motion.get('valid') is True and numeric(mt)
                        and 0 <= now-mt <= MOTION_MAX_AGE_S)
        moving = motion.get('moving') if motion_fresh else None
        if type(moving) is not bool:
            moving = None
        self.motion_state = 'unknown' if moving is None else 'moving' if moving else 'still'
        self.motion_reason = motion.get('reason') if moving is None else None
        if self.phase in ('brake_pulse','settle','verify_red','wait_green'):
            self.stop_motion.update(moving, mt, motion.get('frame_id'), now)
            if self.stop_motion.sliding:
                self.stop_sequence_complete = False
                self.neutral_hold_since = None

        if self.phase == 'arming':
            if now-self.started < self.s['arming_s']:
                return self.output()
            self.approach_at = now
            if self.stop_signal_seen:
                return self._brake(now, self.early_stop_reason or '红灯比例达标或已见黄灯，先制动等待')
            self.phase = 'approach'

        if self.phase == 'approach':
            if self.early_stop_reason is not None:
                return self._brake(now,self.early_stop_reason)
            if self.red_seen:
                v = self.red_statistics
                return self._brake(now, f"最近 {self.s['red_window_s']:g} 秒红灯 {v['red_frames']}/{v['total_frames']} 帧（{v['red_percent']:.1f}%），达到 {v['threshold_percent']:g}% 阈值，制动")
            if self.signal_votes['yellow_present']:
                return self._brake(now, '识别到黄灯，立即请求制动')
            if moving is None:
                return self._begin_motion_recovery(now)
            if now-self.approach_at >= self.s['max_approach_s']:
                return self._stop_fault(now, '接近超时仍未观察到触发后的红灯，制动后停止本轮；不以计时放行')
            if moving is False:
                if self.no_progress_since is None:
                    self.no_progress_since = now
                if now-self.no_progress_since >= self.s['no_progress_s']:
                    return self._stop_fault(now, '持续前进指令但未观察到移动，制动后停止；请检查电调与起步档')
            else:
                self.no_progress_since = None
            self.reason = ('锁定灯具未亮，按红绿灯独立低速上限接近，等待传感器触发' if self.raw_state == 'off' else
                           '尚未观察到触发红灯，在接近总时限内低速巡线并继续寻找灯具' if not self.approach_evidence['any_locked'] else
                           '保持同一灯具跟踪，低速接近；可信红灯即制动')
            return self.output('search')

        if self.phase == 'motion_recovery':
            if self.early_stop_reason is not None or self.stop_signal_seen:
                return self._brake(now,self.early_stop_reason or '恢复中识别到红黄灯，保持停车复核')
            if now-self.motion_unknown_since>=MOTION_RECOVERY_S:
                return self.fault('地面运动反馈恢复超时，保持停车')
            if now<self.motion_recovery_brake_until:return self.output('brake')
            if moving is False:
                previous=getattr(self,'recovery_motion_capture',None)
                if previous is not None and mt<=previous:return self.output()
                if previous is None or mt-previous>MOTION_MAX_AGE_S:self.stop_since=None
                self.recovery_motion_capture=mt
                if self.stop_since is None:self.stop_since=mt
                since=motion.get('continuous_still_since_s')
                continuous=(self.control_interval_s is None and numeric(since)
                            and self.motion_recovery_evidence_after <= since <= mt
                            and mt-max(since,self.motion_recovery_brake_until)>=.3-1e-9)
                if mt-self.stop_since>=.3-1e-9 or continuous:
                    self.phase='approach';self.motion_unknown_since=None;self.no_progress_since=None
                    self.reason='连续静止反馈已恢复，下一帧重新核对红灯后低速接近'
                    return self.output()
            else:
                self.stop_since=None;self.recovery_motion_capture=None
                # An older stillness interval cannot erase movement or an
                # intervening unknown result observed during this recovery.
                self.motion_recovery_evidence_after=now
            self.reason='短暂反馈丢失，保持中位等待连续有效地面反馈'
            return self.output()

        if self.phase in ('brake_pulse', 'settle'):
            if self.phase == 'brake_pulse':
                if now < self.pulse_until:
                    return self.output('brake')
                self.phase, self.release_until = 'settle', now+self.s['brake_release_s']+.02
                self.neutral_hold_since = now
                self.stop_motion.begin_neutral(now)
            if self.pending_fault_reason:
                return self.fault(self.pending_fault_reason)
            if self.stop_motion.sliding:
                if self.stop_motion.rebrake_ready and now >= self.release_until:
                    return self._brake(now, '仍检测到滑动，再请求一个有限制动脉冲', repeat=True)
                if now-self.brake_at > self.s['brake_timeout_s']:
                    return self.fault('制动后仍未排除已检测到的滑动，保持中位')
                self.reason = '已检测到滑动，保持中位等待有效反馈；未知不能当作停稳'
                return self.output()
            if self._finish_neutral_hold(now):
                events=[{'event':'stop_sequence_complete','physical_stop_verified':False}]
                if self.stop_motion.stationary:
                    events.append({'event':'stationary_estimate'})
                if self.red_seen:
                    return self._wait_for_green(now, events)
                self.phase = 'verify_red'
                self.reason = '制动与中位保持已完成，继续复核本轮红灯'
                return self.output(events=events)
            self.reason = ('制动已完成，保持中位；地面反馈暂不可用，继续计时并检测灯色'
                           if moving is None else '制动已完成，继续中位保持')
            return self.output()

        if self.phase == 'verify_red':
            if self.pending_fault_reason:
                if self.stop_motion.rebrake_ready:
                    return self._brake(now, self.pending_fault_reason, repeat=True)
                return self.fault(self.pending_fault_reason)
            if self.stop_motion.rebrake_ready:
                return self._brake(now, '红灯复核期间检测到滑动，重新有限制动；复核截止时间不延长', repeat=True)
            self.motion_unknown_since = None
            if self.red_seen and self.stop_sequence_complete and not self.stop_motion.blocking:
                return self._wait_for_green(now)
            self.reason = ('保持中位复核红灯；地面反馈暂不可用，继续取样'
                           if moving is None else '保持中位等待正式红灯证据；未确认红灯不会放行')
            return self.output()

        if self.phase == 'wait_green':
            if now-self.wait_at >= self.s['max_wait_green_s']:
                return self.fault('等待绿灯超时，保持停车；计时到期不能放行')
            if self.stop_motion.rebrake_ready:
                return self._brake(now, '等待时检测到滑动，重新有限制动', repeat=True)
            self.motion_unknown_since = None
            if (self.red_seen and self.stop_sequence_complete and not self.stop_motion.blocking
                    and self.stable.get('green_confirmed') is True):
                self.phase = 'done'
                self.reason = '停车后本轮确认过红灯的同一灯具绿灯占比达标，请求播报；本轮保持中位'
                return self.output(events=[{'event': 'speak', 'text': self.s['speech_text']}])
            self.reason = ('红绿票同时达标，红灯优先，继续停车' if self.signal_votes['conflict'] else
                           '保持中位，等待连续新地面帧复核移动候选' if self.stop_motion.blocking else
                           '中位保持已完成，地面反馈暂不可用；继续等待同一灯具绿灯'
                           if moving is None else
                           f"等待本轮红灯来源的同一灯具在 {self.s['green_window_s']:g} 秒绿灯窗口达标" if self.red_seen else
                           '尚未确认本轮红灯，保持停止；不会用初始绿灯代替完整周期')
            return self.output()
        return self.fault('未知阶段，停止本轮')
