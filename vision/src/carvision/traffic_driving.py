"""Traffic stage: bounded forward -> F/B brake -> actual green -> speech only.

This supervised field-test controller does not pretend the external light
trigger or a front-wheel position sensor is connected. Ground optical flow is
only a stationary estimate. Competition-zone compliance remains unverified.
No hardware, network, sleeping or wall-clock countdown exists in this core.
"""
import json
from pathlib import Path

from .brake_parking import numeric
from .traffic_voting import SignalFrameVote, DualSignalVotes

# Compatibility for existing red-only replay scripts.
RedFrameVote = SignalFrameVote

TRAFFIC_FORWARD_MIN_US = 1501
TRAFFIC_FORWARD_MAX_US = 1625


def validate_settings(s):
    if (s.get('schema_version') != 1 or s.get('required_esc_mode') != 'F/B'
            or s.get('camera') != 'dual' or s.get('cameras') != ['primary', 'secondary'] or s.get('motion_camera') != 'secondary'
            or s.get('image_size') != [480, 360]):
        raise ValueError('红绿灯使用双路 480×360、第二路地面反馈、F/B 电调模式')
    p = s.get('pwm', {})
    if (p.get('neutral_us') != 1500 or p.get('steering_center_us') != 1610
            or any(type(p.get(k)) is not int for k in ('search_us', 'creep_us', 'brake_us'))
            or not TRAFFIC_FORWARD_MIN_US <= p['search_us'] <= TRAFFIC_FORWARD_MAX_US or p['creep_us'] != p['search_us']
            or p['brake_us'] != 1400):
        raise ValueError('使用已有中位和制动档；红绿灯接近 PWM 必须为 1501–1625 μs')
    limits = {'arming_s': (3, 3), 'max_frame_age_s': (.1, .25), 'actuator_lease_s': (.1, .25),
              'brake_pulse_s': (.12, .12), 'brake_release_s': (.1, .3),
              'brake_timeout_s': (1, 3), 'stop_confirm_s': (.5, 2),
              'max_approach_s': (.5, 10), 'max_wait_green_s': (15, 60), 'no_progress_s': (.5, 3),
              'red_window_s': (2, 2), 'red_confirm_percent': (1, 100)}
    for key, (low, high) in limits.items():
        if not numeric(s.get(key)) or not low <= s[key] <= high:
            raise ValueError('红绿灯参数范围错误：'+key)
    if type(s.get('max_brake_pulses')) is not int or not 1 <= s['max_brake_pulses'] <= 3:
        raise ValueError('制动脉冲最多 3 次')
    if s.get('green_action') != 'speech_only' or s.get('speech_text') != '红绿灯结束，开始前行':
        raise ValueError('本轮绿灯只播报“红绿灯结束，开始前行”，不实际续行')
    return s


def load_settings(path):
    return validate_settings(json.loads(Path(path).read_text(encoding='utf-8-sig')))


class TrafficDrivingController:
    def __init__(self, settings, profile=None):
        self.s = validate_settings(settings)
        self.votes = DualSignalVotes(self.s['red_window_s'], self.s['red_confirm_percent'], self.s['max_frame_age_s'])
        self.signal_votes = None
        self.red_statistics = None
        self.phase, self.reason = 'arming', '中位初始化 3 秒'
        self.started = self.last_t = self.last_capture = self.last_frame = None
        self.approach_at = self.brake_at = self.pulse_until = self.release_until = None
        self.stop_since = self.wait_at = self.no_progress_since = None
        self.motion_unknown_since = None
        self.red_seen = False
        self.stop_signal_seen = False
        self.brake_count = 0
        self.raw_state = 'unknown'
        self.stable = {}

    def output(self, action='neutral', events=None):
        p = self.s['pwm']
        return {'phase': self.phase, 'reason': self.reason, 'action': action,
                'esc_us': {'neutral': 1500, 'search': p['search_us'], 'brake': p['brake_us']}[action],
                'steering_us': p['steering_center_us'], 'events': events or [],
                'signal_state': self.raw_state, 'stable': self.stable,
                'red_vote': self.red_statistics, 'signal_votes': self.signal_votes,
                'red_seen_this_round': self.red_seen, 'brake_pulses': self.brake_count,
                'green_action': 'speech_only',
                'stationary_estimated': self.phase in ('wait_green', 'done'),
                'physical_stop_verified': False, 'parking_zone_verified': False,
                'external_sensor_connected': False, 'field_verified': False}

    def fault(self, reason):
        self.phase, self.reason = 'fault', reason
        return self.output()

    def _brake(self, now, reason):
        self.phase, self.reason = 'brake_pulse', reason
        self.brake_at, self.pulse_until = now, now + self.s['brake_pulse_s']
        self.release_until = None
        self.stop_since = None
        self.brake_count = 1
        self.votes.reset()
        self.stable = {}
        return self.output('brake', [{'event': 'brake_requested', 'reason': reason}])

    def step(self, sample, now):
        if self.phase in ('done', 'fault'):
            return self.output()
        if not numeric(now) or self.last_t is not None and not 0 < now - self.last_t <= .25:
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
        observations = sample.get('camera_observations')
        if not isinstance(observations, dict):
            return self.fault('缺少带来源的双摄观测')
        self.signal_votes = self.votes.update(observations, now)
        if not self.signal_votes['any_fresh']:
            return self.fault('两路摄像头均无新鲜图像')
        self.raw_state = self.signal_votes['raw_state']
        self.stable = {'state':self.signal_votes['state'], 'green_confirmed':self.signal_votes['green_confirmed']}
        qualified = [self.signal_votes['cameras'][c] for c in self.signal_votes['red_sources']]
        eligible = qualified or [v for v in self.signal_votes['cameras'].values() if v['valid']]
        self.red_statistics = max(eligible, key=lambda row:row['red_percent'])
        if self.signal_votes['red_confirmed']:
            self.red_seen = True
            self.stop_signal_seen = True
        if self.signal_votes['yellow_present']:
            self.stop_signal_seen = True
        motion = sample.get('motion') or {}
        mt = motion.get('captured_s')
        motion_fresh = (motion.get('valid') is True and numeric(mt)
                        and 0 <= now-mt <= self.s['max_frame_age_s'])
        moving = motion.get('moving') if motion_fresh else None
        if type(moving) is not bool:
            moving = None

        if self.phase == 'arming':
            if now-self.started < self.s['arming_s']:
                return self.output()
            self.approach_at = now
            if self.stop_signal_seen:
                return self._brake(now, '红灯比例达标或已见黄灯，先制动等待')
            self.phase = 'approach'

        if self.phase == 'approach':
            # User-requested bounded search does not require a visible lamp.
            # Unknown/missing detection remains non-red in the vote; it never
            # authorizes completion or restarting after a stop.
            if self.red_seen:
                v = self.red_statistics
                return self._brake(now, f"最近 2 秒红灯 {v['red_frames']}/{v['total_frames']} 帧（{v['red_percent']:.1f}%），达到 {v['threshold_percent']:g}% 阈值，制动")
            if self.signal_votes['yellow_present']:
                return self._brake(now, '识别到黄灯，立即请求制动')
            if moving is None:
                return self._brake(now, '地面运动反馈失效，制动等待')
            if now-self.approach_at >= self.s['max_approach_s']:
                return self.fault('接近超时仍未观察到触发后的红灯，停止本轮；不以计时放行')
            if moving is False:
                if self.no_progress_since is None:
                    self.no_progress_since = now
                if now-self.no_progress_since >= self.s['no_progress_s']:
                    return self.fault('持续前进指令但未观察到移动，请检查电调与起步档')
            else:
                self.no_progress_since = None
            self.reason = ('红灯候选尚未达到 2 秒窗口占比阈值，保持低档接近' if self.raw_state == 'red' else
                           '灯具已识别但未亮，低档接近等待传感器触发' if self.raw_state == 'off' else
                           '两路暂未确认灯具，按已保存油门限时前进搜索' if self.raw_state == 'unknown' else
                           '低档接近，等待红灯占比达标')
            return self.output('search')

        if self.phase in ('brake_pulse', 'settle'):
            if now-self.brake_at > self.s['brake_timeout_s']:
                return self.fault('制动后未取得连续停稳估计，请检查刹车与地面画面')
            if self.phase == 'brake_pulse':
                if now < self.pulse_until:
                    return self.output('brake')
                self.phase, self.release_until = 'settle', now+self.s['brake_release_s']+.02
            if moving is False:
                if self.stop_since is None:
                    self.stop_since = now
                if now-self.stop_since >= self.s['stop_confirm_s']:
                    self.votes.reset()
                    self.stable = {}
                    self.phase = 'wait_green'
                    self.reason = '视觉停稳候选成立，等待实际红灯转绿'
                    self.wait_at = now
                    return self.output(events=[{'event': 'stationary_estimate'}])
            else:
                self.stop_since = None
                if moving is True and now >= self.release_until and self.brake_count < self.s['max_brake_pulses']:
                    self.brake_count += 1
                    self.phase, self.pulse_until = 'brake_pulse', now+self.s['brake_pulse_s']
                    self.reason = '仍检测到移动，再请求一个有限制动脉冲'
                    return self.output('brake')
            self.reason = '保持中位，等待连续停稳估计'
            return self.output()

        if self.phase == 'wait_green':
            if now-self.wait_at >= self.s['max_wait_green_s']:
                return self.fault('等待绿灯超时，保持停车；计时到期不能放行')
            if moving is not False:
                self.votes.reset()
                self.stable = {}
                if moving is True:
                    return self._brake(now, '等待时检测到滑动，重新制动')
                if self.motion_unknown_since is None:
                    self.motion_unknown_since = now
                if now-self.motion_unknown_since >= self.s['brake_timeout_s']:
                    return self.fault('等待中地面运动反馈持续失效，停止本轮')
                self.reason = '地面反馈暂不可用，保持中位且不累计绿灯'
                return self.output()
            self.motion_unknown_since = None
            if self.red_seen and self.stable.get('green_confirmed') is True:
                self.phase = 'done'
                self.reason = '停车后任一路绿灯占比达标，请求播报；本轮保持中位，不实际前行'
                return self.output(events=[{'event': 'speak', 'text': self.s['speech_text']}])
            self.reason = ('红绿票同时达标，红灯优先，继续停车' if self.signal_votes['conflict'] else
                           '等待任一路绿灯在 2 秒窗口内达到共用阈值' if self.red_seen else
                           '尚未确认本轮红灯，保持停止；不会用初始绿灯代替完整周期')
            return self.output()
        return self.fault('未知阶段，停止本轮')
