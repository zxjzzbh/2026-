#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""识别和跟踪锥桶，按一次一个目标的顺序给出低速绕行建议。

默认检测画面下方 70%，支持蓝桶和红桶。动作依次是避让、确认侧向离开、
继续通过并修正朝向；当前目标确认通过后，才能恢复巡线或交给下一个桶。
操场和比赛融合入口会提供同一张图的白线位置，限制向边界外转向，
并根据当前车道确认回正。独立绕桶入口没有白线反馈，仍采用原定时方式。
图像中的空隙和位置不代表实测距离。持续失去目标或画面异常时请求停车。
本文件只计算结果，不操作摄像头、电机或舵机。"""
from collections import deque
from dataclasses import dataclass, field
import math
import time

import cv2
import numpy as np


@dataclass
class Params:
    """保存识别颜色、目标确认次数、转向大小和各阶段时间等绕桶参数。"""

    color: str = 'blue'
    # 限制蓝色的鲜艳程度和亮度，减少把偏蓝的黑色车身误认成锥桶。
    blue_low: tuple = (90, 80, 60)
    blue_high: tuple = (124, 255, 255)
    roi_top: float = 0.30  # 检测画面下方70%
    min_area: float = 12.0  # 以352x288画面为准，保留很小的锥桶色块
    front_x: float = 0.47  # 来自居中样本的临时方向参考；非车道标定
    padding: float = 0.17  # 图像宽度比例；不是实测车身投影
    trigger_height: float = 6.0  # 按288像素图高折算的候选高度；不是距离
    lost_hold_seconds: float = 0.30  # 短时丢检保留同侧转向；超时不盲走
    confirm_window: int = 5
    confirm_hits: int = 3
    lost_frames: int = 3
    clear_frames: int = 3
    max_age: float = 0.25
    avoid_timeout: float = 2.5  # 持续这么久没有横向进展时请求停车
    max_turn_us: float = 90.0  # 相对原转向中位的偏移；正数=向右
    min_turn_us: float = 25.0
    hold_peak_turn: bool = False  # 启用后保持已达到的转向量，确认横向让开才释放
    kp: float = 650.0
    slew_us_s: float = 600.0
    pass_seconds: float = 0.60  # 目标侧向离开视野后，再直行保持的时间
    align_seconds: float = 0.30  # 定时反打初值，不是已验证的航向恢复时间
    lane_guidance: bool = False  # 融合入口提供本帧车道；单独绕桶仍沿用原方式
    lane_margin_ratio: float = 0.22  # 在车道宽度内给两侧白线保留的比例；不是厘米
    align_timeout: float = 2.0  # 回正超过此时长标记继续恢复，不因时间到而停车
    recovery_turn_us: float = 130.0  # 让开后修正朝向可比初始避让更有力
    protection_stop: bool = True  # 操场可关闭丢桶、无进展及未确认通过的停车

    def __post_init__(self):
        """检查参数是否为有限数值且在允许范围内，不合格时说明原因并拒绝使用。"""
        nums = [v for v in vars(self).values() if isinstance(v, (int, float))]
        if not all(math.isfinite(v) for v in nums):
            raise ValueError('参数必须为有限数值')
        if self.color not in ('blue', 'red'):
            raise ValueError('color 必须为 blue 或 red')
        if not (0.15 <= self.front_x <= 0.85 and 0 <= self.roi_top < 0.8):
            raise ValueError('front_x/roi_top 超出范围')
        if not (0 < self.padding < 0.35 and 0 <= self.pass_seconds <= 3):
            raise ValueError('padding 或 pass_seconds 超出范围')
        if not (
            1 <= self.confirm_hits <= self.confirm_window and self.lost_frames >= 1
        ):
            raise ValueError('多帧确认参数无效')
        if not (0 < self.max_age <= 1 and 0 < self.avoid_timeout <= 5):
            raise ValueError('超时参数无效')
        if not (0 < self.min_turn_us <= self.max_turn_us <= 130 and self.slew_us_s > 0):
            raise ValueError('转向参数无效')
        if not (0 < self.align_seconds <= 1 and self.clear_frames >= 1 and self.kp > 0):
            raise ValueError('反打或控制参数无效')
        if not (
            0 < self.trigger_height <= 50
            and self.min_area > 0
            and 0 < self.lost_hold_seconds <= 0.40
        ):
            raise ValueError('候选高度或短时跟踪保持参数无效')
        if not (
            0.05 <= self.lane_margin_ratio <= 0.35
            and self.align_seconds <= self.align_timeout <= 3
            and 0 < self.recovery_turn_us <= 130
        ):
            raise ValueError('白线预留比例或车道回正时间无效')


class Detector:
    """根据指定颜色从画面中找出可能的锥桶，保留位置和大小。"""

    def __init__(self, params):
        """保存颜色和检测区域等参数，供后续每张画面共用。"""
        self.p = params

    def detect(self, frame):
        """检查彩色画面，返回颜色黑白图和按横向位置排序的候选方框。"""
        if (
            not isinstance(frame, np.ndarray)
            or frame.ndim != 3
            or frame.shape[2] != 3
            or min(frame.shape[:2]) < 4
            or frame.dtype != np.uint8
        ):
            raise ValueError('需要非空 uint8 BGR 图像')
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if self.p.color == 'blue':
            mask = cv2.inRange(
                hsv,
                np.array(self.p.blue_low, np.uint8),
                np.array(self.p.blue_high, np.uint8),
            )
        else:
            mask = cv2.inRange(hsv, (0, 75, 50), (10, 255, 255))
            mask |= cv2.inRange(hsv, (170, 75, 50), (179, 255, 255))
        mask[: int(h * self.p.roi_top)] = 0
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[
            -2
        ]
        boxes = []
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            area = float(cv2.contourArea(contour))
            if area < self.p.min_area * (h * w) / (352 * 288) or min(bw, bh) < 3:
                continue
            boxes.append(
                dict(
                    x=x,
                    y=y,
                    w=bw,
                    h=bh,
                    area=area,
                    cx=(x + bw / 2) / w,
                    bottom=(y + bh - 1) / (h - 1),
                    left=x / w,
                    right=(x + bw) / w,
                    nw=bw / w,
                    nh=bh / h,
                    clipped=(y + bh >= h - 1),
                )
            )
        return mask, sorted(boxes, key=lambda b: (b['x'], b['y']))


class Tracker:
    """对比前后画面的物体位置与大小，为同一个目标保留编号并累计确认次数。"""

    def __init__(self, params):
        """保存确认参数，建立空的目标记录，并从编号一开始分配新目标。"""
        self.p, self.tracks, self.next_id = params, {}, 1

    def update(self, boxes, ts):
        """用本帧方框和采集时间更新目标记录，给方框补上编号及确认状态后返回。"""
        for t in self.tracks.values():
            t['miss'] += 1
            t['hits'].append(0)
        pairs = []
        for i, b in enumerate(boxes):
            for tid, t in self.tracks.items():
                dx, dy = abs(b['cx'] - t['cx']), abs(b['bottom'] - t['bottom'])
                rw = max(b['nw'] / t['nw'], t['nw'] / b['nw'])
                rh = max(b['nh'] / t['nh'], t['nh'] / b['nh'])
                cropped = b['clipped'] or t['clipped_seen']
                if (
                    ts - t['seen'] > 0.40
                    or dx > 0.10
                    or dy > 0.12
                    or rw > (6 if cropped else 3)
                    or (not cropped and rh > 3)
                ):
                    continue
                cost = dx / 0.10 + dy / 0.12 + 0.25 * math.log(rw)
                if not cropped:
                    cost += 0.15 * math.log(rh)
                pairs.append((cost, i, tid))
        used_i, used_t = set(), set()
        for _, i, tid in sorted(pairs):
            if i in used_i or tid in used_t:
                continue
            t = self.tracks[tid]
            t.update(boxes[i])
            t['seen'], t['miss'], t['hits'][-1] = ts, 0, 1
            t['confirmed'] |= sum(t['hits']) >= self.p.confirm_hits
            t['clipped_seen'] |= boxes[i]['clipped']
            boxes[i]['id'] = tid
            used_i.add(i)
            used_t.add(tid)
        for i, b in enumerate(boxes):
            if i not in used_i:
                tid = self.next_id
                self.next_id += 1
                b['id'] = tid
                self.tracks[tid] = dict(
                    b,
                    seen=ts,
                    miss=0,
                    hits=deque([1], maxlen=self.p.confirm_window),
                    confirmed=(self.p.confirm_hits == 1),
                    clipped_seen=b['clipped'],
                )
            t = self.tracks[b['id']]
            b.update(confirmed=t['confirmed'], clipped_seen=t['clipped_seen'])
        # 保留最近丢失的轨迹供决策层检查；不把删除当作通过证据。
        for tid in list(self.tracks):
            if ts - self.tracks[tid]['seen'] > 2.0:
                del self.tracks[tid]
        return boxes


def free_intervals(blocked, low=0.08, high=0.92):
    """从指定横向范围扣除所有占用区间，返回剩余的空隙；数值均为画面宽度比例。"""
    free = [(low, high)]
    for lo, hi in sorted(blocked):
        nxt = []
        for a, b in free:
            if hi <= a or lo >= b:
                nxt.append((a, b))
            else:
                if lo > a:
                    nxt.append((a, lo))
                if hi < b:
                    nxt.append((hi, b))
        free = nxt
    return free


@dataclass
class Decision:
    """保存当前绕桶阶段、转向偏移、停车原因、锁定目标和检查画面数据。"""

    state: str
    turn_us: float  # 正=右，负=左；与pigpio脉宽方向相反
    stop: bool
    reason: str
    side: int
    boxes: list = field(default_factory=list)
    mask: object = None
    target_id: object = None  # 本次锁定的单个目标，完成回正或满足已通过的交接条件才释放
    handoff_turn_us: object = None  # 完成交接这一帧延续回正方向，避免先停车或突然反向


class Avoider:
    """连续处理画面，锁定单个锥桶，完成整次绕行动作并在异常时请求停车。"""

    def __init__(self, params=None):
        """创建检测与跟踪工具，并清空目标、计时、转向和故障记录。"""
        self.p = params or Params()
        self.detector, self.tracker = Detector(self.p), Tracker(self.p)
        self.state, self.fault, self.side = 'STRAIGHT', '', 0
        self.active, self.clear_count = set(), 0
        self.target_id = None
        self.last_id, self.last_ts, self.last_now, self.shape = None, None, None, None
        self.started, self.phase_time, self.turn, self.turn_area = 0.0, 0.0, 0.0, 0.0
        self.align_turn = 0.0
        self.avoid_peak = 0.0
        self.progress_time, self.best_deficit = 0.0, None
        self.clear_hits = {}
        self.exit_evidence, self.pass_targets = set(), set()
        self.exit_since = None
        self.pass_confirmed = False
        self.last = Decision('STRAIGHT', 0, False, '', 0)
        self.lane_hint = {'valid': False}
        self.align_hits = 0
        self.align_last_valid = None
        self.edge_limited = False
        self.edge_caps = {-1: self.p.max_turn_us, 1: self.p.max_turn_us}
        self.lane_missing_since = None
        self.recovery_turn = 0.0
        self.recovery_seen_at = None

    def set_lane_hint(self, **hint):
        """接收同一张图的车道结果；只使用本帧可信的边界，失效时不沿用旧绿线。"""
        self.lane_hint = hint

    def _lane_turn(self):
        """优先修正车身朝向；没有可靠朝向时才采用巡线方向，使用独立的回正上限。"""
        h = self.lane_hint
        requested = h.get('turn_us', 0.0)
        if h.get('heading_valid'):
            requested = 3.0 * h['heading_error_px']
            left, right = h.get('left_x'), h.get('right_x')
            if left is not None and right is not None and right > left:
                position = (h['front_x'] - left) / (right - left)
                if (
                    not self.p.lane_margin_ratio
                    <= position
                    <= 1 - self.p.lane_margin_ratio
                ):
                    requested += h.get('error_px', 0.0)
        return max(
            -self.p.recovery_turn_us,
            min(self.p.recovery_turn_us, requested),
        )

    def _pass_turn(self):
        """横向让开就开始修正朝向；漏一帧白线时保留刚确认的修正，不突然收回中位。"""
        return self.recovery_turn if self.p.lane_guidance else 0.0

    def _lane_edge_caps(self, maximum):
        """由本帧可信白线计算两侧转向上限，供选边和实际限幅共同使用。"""
        h = self.lane_hint
        left, right = h.get('left_x'), h.get('right_x')
        if not (self.p.lane_guidance and h.get('valid') and
                left is not None and right is not None and right > left):
            return None
        front = h['front_x']
        margin = self.p.lane_margin_ratio
        caps = {}
        for direction, distance in ((1, right - front), (-1, front - left)):
            clearance = distance / (right - left)
            fraction = max(0.0, min(1.0, (clearance - margin) / margin))
            caps[direction] = maximum * fraction
        return caps

    def _edge_limit(self, requested):
        """越靠近一侧白线，越少允许继续向那一侧转；向车道内回正不受此限制。"""
        self.edge_limited = False
        if not self.p.lane_guidance or not requested:
            return requested
        maximum = (self.p.recovery_turn_us if self.state in ('PASS', 'ALIGN')
                   else self.p.max_turn_us)
        caps = self._lane_edge_caps(maximum)
        if caps is not None:
            self.edge_caps.update(caps)
        # 短时漏检只能沿用原有的限制，不能借漏检突然恢复向白线外满打。
        cap = self.edge_caps[1 if requested > 0 else -1]
        limited = math.copysign(min(abs(requested), cap), requested)
        self.edge_limited = abs(limited) + 0.5 < abs(requested)
        return limited

    def _finish_avoid(self, boxes, mask, dt):
        """结束已完成的桶并保留交接方向；下一张新图才允许挑选第二个桶。"""
        handoff_turn = None
        if self.p.lane_guidance and self.lane_hint.get('valid'):
            handoff_turn = self._edge_limit(self._lane_turn())
        self.turn = 0.0
        self.state, self.side = 'STRAIGHT', 0
        self.target_id = None
        self.active.clear()
        self.pass_targets.clear()
        self.clear_hits.clear()
        self.exit_evidence.clear()
        self.turn_area = 0.0
        self.align_hits = 0
        self.pass_confirmed = False
        out = self._out(boxes, mask, reason='RESUME_STRAIGHT', dt=dt)
        out.handoff_turn_us = handoff_turn
        return out

    def _inside_lane(self):
        """本帧白线有效且车头位于预留范围内时，允许把已通过的桶交接出去。"""
        h = self.lane_hint
        left, right = h.get('left_x'), h.get('right_x')
        if not h.get('valid') or left is None or right is None or right <= left:
            return False
        position = (h['front_x'] - left) / (right - left)
        return self.p.lane_margin_ratio <= position <= 1 - self.p.lane_margin_ratio

    def _confirmed_exit(self, track, captured_at):
        """桶已让到侧边并从近处退出，两张新图看不见它才承认退出；无证据丢失不算。"""
        if track is None:
            return False
        return (
            track['id'] in self.exit_evidence
            and track['miss'] > 0
            and (
                captured_at - track['seen'] > self.p.lost_hold_seconds
                or (
                    self.p.lane_guidance
                    and track['bottom'] >= 0.90
                    and track['miss'] >= 2
                )
            )
        )

    def _align_with_lane(self, boxes, mask, now, dt):
        """回正期间不断前进并重检白线；连续确认恢复才交接，不因跳变或超时停车。"""
        h = self.lane_hint
        if h.get('valid'):
            self.align_last_valid = now
            aligned = (
                abs(h.get('error_px', float('inf'))) <= 12
                and abs(self._lane_turn()) <= 35
            )
            if h.get('heading_valid'):
                left, right = h.get('left_x'), h.get('right_x')
                aligned = False
                if left is not None and right is not None and right > left:
                    position = (h['front_x'] - left) / (right - left)
                    aligned = (
                        abs(h['heading_error_px']) <= 10
                        and self.p.lane_margin_ratio
                        <= position
                        <= 1 - self.p.lane_margin_ratio
                    )
            self.align_hits = self.align_hits + 1 if aligned else 0
            requested = self._lane_turn()
        else:
            self.align_hits = 0
            # 暂时沿用刚确认的回正方向；持续跳变时收回中位，避免一直沿旧方向转。
            since = (
                self.phase_time
                if self.recovery_seen_at is None
                else self.recovery_seen_at
            )
            age = max(0.0, now - since)
            fraction = max(0.0, min(1.0, (0.80 - age) / 0.50))
            requested = self._pass_turn() * fraction
        elapsed = now - self.phase_time
        if self.align_hits >= 3 and elapsed >= self.p.align_seconds:
            out = self._out(boxes, mask, dt=dt, reason='CENTER_AFTER_ALIGN')
            if abs(out.turn_us) <= 0.5:
                return self._finish_avoid(boxes, mask, dt)
            return out
        reason = ('ALIGN_REDETECT' if not h.get('valid') else
                  'ALIGN_CONTINUE' if elapsed > self.p.align_timeout else 'ALIGN_WITH_LANE')
        return self._out(boxes, mask, requested, reason=reason, dt=dt)

    def _interval(self, b):
        """在目标左右边缘各加上预留宽度，返回需要避开的横向区间。"""
        return b['left'] - self.p.padding, b['right'] + self.p.padding

    def _blocks(self, b):
        """检查目标加上预留宽度后，是否挡住车头的前进参考位置。"""
        lo, hi = self._interval(b)
        return lo <= self.p.front_x <= hi

    def _out(self, boxes, mask, requested=0.0, stop=False, reason='', dt=0.0):
        """限制本次转向变化幅度，保存并返回绕行结果；停车时把转向偏移归零。"""
        if stop:
            self.turn = 0.0
        else:
            requested = self._edge_limit(requested)
            if self.edge_limited:
                reason = 'LANE_EDGE_GUARD'
            step = self.p.slew_us_s * dt
            self.turn += max(-step, min(step, requested - self.turn))
        self.last = Decision(
            self.state, self.turn, stop, reason, self.side, boxes, mask, self.target_id
        )
        return self.last

    def halt(self, reason, boxes=None, mask=None):
        """锥桶停车关闭时改为车道回正；其他故障记住首次原因并锁定停车。"""
        if not self.p.protection_stop and reason in (
            'TARGET_UNCERTAIN', 'PASS_UNCONFIRMED', 'NO_AVOID_PROGRESS'
        ):
            # 放弃不确定的桶并按本帧白线回正，不把丢失记作已通过。
            # 保留当前转向和斜率限制，避免在交接帧突然跳到另一个方向。
            self.fault = ''
            self.state, self.side = 'ALIGN', 0
            self.phase_time = self.last_now if self.last_now is not None else 0.0
            self.target_id = None
            self.active.clear()
            self.pass_targets.clear()
            self.clear_hits.clear()
            self.exit_evidence.clear()
            self.pass_confirmed = False
            self.align_hits = 0
            self.align_last_valid = None
            self.align_turn = 0.0
            self.turn_area = 0.0
            return self._out(boxes or [], mask, self._lane_turn(),
                             reason=reason + '_LANE_RECOVERY', dt=0.0)
        self.fault = self.fault or reason
        self.state = 'HALT'
        return self._out(boxes or [], mask, stop=True, reason=self.fault)

    def _choose_side(self, ready, prefer=0):
        """比较目标两侧所需横移，返回左侧负一或右侧正一；已有方向时尽量保持。"""
        free = [
            ab
            for ab in free_intervals([self._interval(t) for t in ready])
            if ab[1] - ab[0] >= 0.04
        ]
        if not prefer:
            caps = self._lane_edge_caps(self.p.max_turn_us)
            if caps is not None:
                usable = [ab for ab in free if caps[
                    1 if (ab[0] + ab[1]) / 2 > self.p.front_x else -1
                ] >= self.p.min_turn_us]
                # 开始绕行时优先选择白线允许最低避让量的一侧；
                # 两侧都受限则仍按原空隙选择，实际输出继续接受边界限幅。
                # 已锁定的动作不在途中换边。
                if usable:
                    free = usable
        if prefer:
            same = [
                ab for ab in free if ((ab[0] + ab[1]) / 2 - self.p.front_x) * prefer > 0
            ]
            if same:
                return prefer
        if not free:
            # 图像中的预留宽度覆盖两侧，并不能证明实际道路已被封死。
            # 已经绕行时保持方向，避免空隙短暂消失造成左右反复切换。
            if prefer:
                return prefer
            left_room = min(t['left'] for t in ready) - self.p.padding - 0.08
            right_room = 0.92 - max(t['right'] for t in ready) - self.p.padding
            # 比较带符号余量：即使两侧都为负，也能选覆盖较少的一侧。
            # 完全相同时固定向右，保证选择可重复。
            return 1 if right_room >= left_room - 1e-9 else -1

        # 比较进入空隙所需的最小横移，避免相机偏向一侧时选错方向。
        def cost(ab):
            """给一个空隙计算排序依据：先比横移距离，再比宽度，最后固定并列时的选择。"""
            target = max(ab[0] + 0.02, min(ab[1] - 0.02, self.p.front_x))
            return abs(target - self.p.front_x), -(ab[1] - ab[0]), -target

        a, b = min(free, key=cost)
        return 1 if (a + b) / 2 > self.p.front_x else -1

    def _begin_avoid(self, side, target, now):
        """锁定目标编号和绕行方向，从给定时间开始避让，并清空上一次动作记录。"""
        # 只从直行状态进入；整个动作锁定一个目标编号和一个方向。
        self.turn_area = 0.0
        self.state, self.side, self.started = 'AVOID', side, now
        self.target_id = target['id']
        self.active = {self.target_id}
        self.clear_hits = {}
        self.exit_evidence = set()
        self.avoid_peak = 0.0
        self.pass_targets = set()
        self.exit_since = None
        self.pass_confirmed = False
        self.align_hits = 0
        self.align_last_valid = None
        self.progress_time, self.best_deficit = now, None

    def _deficit(self, t):
        """计算当前方向还需让开的横向距离；负数表示图像中已经让出预留宽度。"""
        lo, hi = self._interval(t)
        return hi - self.p.front_x if self.side > 0 else self.p.front_x - lo

    def _visible_side_exit(self, t):
        """检查目标是否已横向让开并靠近画面底部或侧边，作为离开视野的辅助证据。"""
        edge = t['left'] <= 0.02 if self.side > 0 else t['right'] >= 0.98
        return self._deficit(t) < -0.015 and (t['bottom'] >= 0.90 or edge)

    def _start_pass(self, now):
        """记录通过阶段的开始时间，保留当前目标，并等待确认它已离开视野。"""
        self.state, self.phase_time = 'PASS', now
        self.pass_targets = self.active.copy()
        self.exit_since = None
        self.pass_confirmed = False

    def _after_pass_confirmed(self, boxes, mask, now, dt):
        """Original mode starts alignment; optional modes can hand off here."""
        self.state, self.phase_time = 'ALIGN', now
        self.align_hits = 0
        self.align_last_valid = None
        self.align_turn = max(
            -self.p.max_turn_us,
            min(self.p.max_turn_us, -self.turn_area / self.p.align_seconds),
        )
        return None

    def _after_tracking(self, boxes, captured_at):
        """Optional target continuation; original mode leaves tracking unchanged."""

    def update(self, frame, frame_id, captured_at, now=None):
        """输入画面、帧编号和采集时间，推进绕桶动作并返回当前控制建议。

        now 是当前时间，未传入时使用不会随系统校时倒退的时钟；
        所有时间必须来自同一条时间轴。重复帧不重复确认，过期或倒序画面请求停车。"""
        now = time.monotonic() if now is None else float(now)
        p = self.p
        # 必须在重复帧分支之前判断超时；完全停帧也能触发停车。
        if (
            not math.isfinite(now)
            or not math.isfinite(captured_at)
            or now - captured_at > p.max_age
            or captured_at - now > 0.03
        ):
            return self.halt('STALE_FRAME')
        if self.last_now is not None and now < self.last_now:
            return self.halt('CLOCK_REVERSED')
        if frame_id == self.last_id:
            return self.last
        if self.last_id is not None and (
            frame_id < self.last_id or captured_at <= self.last_ts
        ):
            return self.halt('FRAME_ORDER')
        dt = 1 / 30 if self.last_now is None else min(0.10, now - self.last_now)
        self.last_id, self.last_ts, self.last_now = frame_id, captured_at, now
        if frame is None:
            return self.halt('NO_FRAME')
        if self.shape is not None and frame.shape != self.shape:
            return self.halt('CAMERA_SIZE_CHANGED')
        try:
            mask, boxes = self.detector.detect(frame)
        except (ValueError, cv2.error):
            return self.halt('BAD_FRAME')
        self.shape = frame.shape
        boxes = self.tracker.update(boxes, captured_at)
        if self.fault:
            return self.halt(self.fault, boxes, mask)
        self._after_tracking(boxes, captured_at)
        if p.lane_guidance:
            if self.lane_hint.get('valid'):
                self.lane_missing_since = None
                self.recovery_turn = self._lane_turn()
                self.recovery_seen_at = now
            elif self.lane_missing_since is None:
                self.lane_missing_since = now
            elif (
                self.state in ('AVOID', 'PASS') and now - self.lane_missing_since > 0.40
            ):
                return self.halt('LANE_BOUNDARY_LOST', boxes, mask)
        # 不使用底边位置推断距离；触底只保留CLIP标志。
        ready = [
            t
            for t in self.tracker.tracks.values()
            if t['confirmed']
            and t['miss'] == 0
            and (
                t['nh'] * 288 >= p.trigger_height
                or t['clipped_seen']
                or t['id'] in self.active
            )
        ]
        # 其他目标可以继续被检测/跟踪，但不能改变当前目标、方向或阶段。
        for tid in self.active:
            t = self.tracker.tracks.get(tid)
            if t is not None and t['miss'] == 0 and not self._visible_side_exit(t):
                self.exit_evidence.discard(tid)
        threats = [t for t in ready if self._blocks(t)]
        if (
            self.state == 'PASS' or (p.lane_guidance and self.state == 'ALIGN')
        ) and any(t['id'] == self.target_id for t in threats):
            # 当前桶重新挡住前进位置时继续避让它，仍不改方向或换目标。
            self.state = 'AVOID'
            self.clear_hits.clear()
            self.exit_evidence.clear()
            self.pass_targets.clear()
            self.exit_since = None
            self.pass_confirmed = False
            self.progress_time, self.best_deficit = now, None
        if self.state in ('ALIGN', 'PASS'):
            pass_requested = self._pass_turn()
            if self.state == 'PASS':
                gone = True
                for tid in self.pass_targets:
                    t = self.tracker.tracks.get(tid)
                    if t is not None and t['miss'] == 0:
                        gone = False
                        if self._visible_side_exit(t):
                            self.exit_evidence.add(tid)
                        else:
                            self.exit_evidence.discard(tid)
                    elif (
                        t is not None and captured_at - t['seen'] <= p.lost_hold_seconds
                    ):
                        # 只有近处、已横向让开并连续两帧消失的桶，才允许提早确认退出。
                        if not self._confirmed_exit(t, captured_at):
                            gone = False
                    elif tid not in self.exit_evidence:
                        return self.halt('TARGET_UNCERTAIN', boxes, mask)
                if gone:
                    # 这里仅决定何时确认通过；修正朝向已在横向让开时开始。
                    # 通过保持期间仍锁定原桶，第二个桶不能抢走控制。
                    if self.exit_since is None:
                        self.exit_since = now
                    if now - self.exit_since >= p.pass_seconds:
                        self.pass_confirmed = True
                        handoff = self._after_pass_confirmed(boxes, mask, now, dt)
                        if handoff is not None:
                            return handoff
                else:
                    self.exit_since = None
                if (
                    self.state == 'PASS'
                    and now - self.phase_time > p.avoid_timeout + p.pass_seconds
                ):
                    return self.halt('PASS_UNCONFIRMED', boxes, mask)
            if self.state == 'ALIGN' and p.lane_guidance:
                # 已从侧边退出并完成通过保持，才算第一个桶结束。
                # 此时第二个桶需要处理，就连续交接；不等车身回到完美朝向，更不刹车。
                if (
                    self.pass_confirmed
                    and self._inside_lane()
                    and any(t['id'] != self.target_id for t in threats)
                ):
                    return self._finish_avoid(boxes, mask, dt)
                return self._align_with_lane(boxes, mask, now, dt)
            if self.state == 'ALIGN' and now - self.phase_time >= p.align_seconds:
                out = self._out(boxes, mask, reason='CENTER_AFTER_ALIGN', dt=dt)
                if abs(out.turn_us) <= 0.5:
                    return self._finish_avoid(boxes, mask, dt)
                return out
            out = self._out(
                boxes,
                mask,
                self.align_turn if self.state == 'ALIGN' else pass_requested,
                reason=(
                    'EXIT_CONFIRMED_ALIGN'
                    if self.state == 'ALIGN'
                    else 'PASS_LANE_RECOVERY' if pass_requested else 'WAIT_SIDE_EXIT'
                ),
                dt=dt,
            )
            self.turn_area += out.turn_us * dt
            return out
        if self.state == 'STRAIGHT':
            if not threats:
                return self._out(boxes, mask, dt=dt)
            # 底边更靠下者优先，仅用于处理顺序，不换算真实距离。
            target = min(
                threats,
                key=lambda t: (
                    -t['bottom'],
                    -t['nh'],
                    abs(t['cx'] - p.front_x),
                    t['id'],
                ),
            )
            side = self._choose_side([target])
            self._begin_avoid(side, target, now)

        # 当前桶未确认通过时，不能把第二个桶加入正在处理的目标。
        for tid in list(self.active):
            t = self.tracker.tracks.get(tid)
            if t is None or captured_at - t['seen'] > p.lost_hold_seconds:
                if tid not in self.exit_evidence:
                    return self.halt('TARGET_UNCERTAIN', boxes, mask)
                self.active.remove(tid)
        tracks = [self.tracker.tracks[tid] for tid in sorted(self.active)]
        if tracks and all(self._confirmed_exit(t, captured_at) for t in tracks):
            self._start_pass(now)
            self.exit_since = now
            return self._out(
                boxes, mask, self._pass_turn(), reason='SIDE_EXIT_PASS', dt=dt
            )
        if not tracks:
            self._start_pass(now)
            out = self._out(
                boxes, mask, self._pass_turn(), reason='SIDE_EXIT_PASS', dt=dt
            )
            self.turn_area += out.turn_us * dt
            return out
        if any(t['miss'] > 0 for t in tracks):
            for t in tracks:
                if t['miss'] > 0:
                    self.clear_hits[t['id']] = 0
            requested = self.turn
            if p.hold_peak_turn:
                requested = self.side * self.avoid_peak
            reason = 'TRACK_HOLD'
            out = self._out(boxes, mask, requested, reason=reason, dt=dt)
            self.turn_area += out.turn_us * dt
            return out

        # 横向让开仅允许开始通过，不能证明车头已经越过锥桶。
        for t in tracks:
            tid = t['id']
            self.clear_hits[tid] = (
                self.clear_hits.get(tid, 0) + 1 if self._deficit(t) < -0.015 else 0
            )
            # 退出证据与“连续三帧横向让开”分开：临近画面底部时可能只剩一两帧。
            # 这里仅记下可见的退出位置；后续仍需连续消失和通过保持，不能直接换桶。
            if self._visible_side_exit(t):
                self.exit_evidence.add(tid)
            else:
                self.exit_evidence.discard(tid)
        if tracks and all(self.clear_hits[t['id']] >= p.clear_frames for t in tracks):
            self._start_pass(now)
            out = self._out(
                boxes, mask, self._pass_turn(), reason='SIDE_CLEAR_PASS', dt=dt
            )
            self.turn_area += out.turn_us * dt
            return out
        deficit = max(self._deficit(t) for t in tracks)
        if self.best_deficit is None or self.best_deficit - deficit >= 0.004:
            self.best_deficit, self.progress_time = deficit, now
        if now - self.progress_time > p.avoid_timeout:
            return self.halt('NO_AVOID_PROGRESS', boxes, mask)
        requested = (
            self.side * min(p.max_turn_us, max(p.min_turn_us, p.kp * deficit))
            if deficit >= 0
            else 0.0
        )
        if p.hold_peak_turn:
            # 方框变小或剩余偏差变小，不能证明车身已经让开锥桶。
            # 连续确认横向让开、进入通过阶段后，才释放保持的转向量。
            self.avoid_peak = min(
                p.max_turn_us, max(self.avoid_peak, abs(requested), p.min_turn_us)
            )
            requested = self.side * self.avoid_peak
        out = self._out(boxes, mask, requested, dt=dt)
        self.turn_area += out.turn_us * dt
        return out


def draw_debug(frame, result, params):
    """复制原图，画出前进参考位置、目标方框、锁定编号和动作状态，返回新图。"""
    out = frame.copy()
    h, w = out.shape[:2]
    for q in (
        params.front_x - params.padding,
        params.front_x,
        params.front_x + params.padding,
    ):
        x = int(q * w)
        cv2.line(out, (x, int(params.roi_top * h)), (x, h - 1), (180, 180, 180), 1)
    for b in result.boxes:
        color = (
            (0, 0, 255)
            if b['clipped_seen']
            else (0, 220, 0) if b['confirmed'] else (0, 200, 255)
        )
        cv2.rectangle(
            out, (b['x'], b['y']), (b['x'] + b['w'] - 1, b['y'] + b['h'] - 1), color, 1
        )
        cv2.putText(
            out,
            '#%s%s' % (b['id'], ' CLIP' if b['clipped_seen'] else ''),
            (b['x'], max(10, b['y'] - 3)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            color,
            1,
        )
    for i, text in enumerate(
        (
            result.state + ' target=' + str(result.target_id) + ' ' + result.reason,
            'turn=%+.0fus  stop=%d' % (result.turn_us, result.stop),
            'IMAGE HEURISTIC - NOT A LANE',
        )
    ):
        cv2.putText(
            out,
            text,
            (4, 15 + 15 * i),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (0, 255, 255),
            1,
        )
    return out
