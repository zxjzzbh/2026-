#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把巡线、绕桶和斑马线的结果合成一条行驶命令。

FusionController负责处理同一张图片，MissionPolicy决定谁来控制方向、
何时停车，Command装好结果，最后由fusion_run下发给车。
这里不直接操作硬件；绕桶时由锥桶控制方向，不把两种转向指令相加。
"""
from dataclasses import dataclass, replace
import math
import time

import config as C
from cone_solo_logic import Avoider, Detector, Params, Decision
from crosswalk import CrosswalkDetector
from lane_follow import LaneFollower


@dataclass
class Settings:
    """保存各阶段动力、等待时间和是否启用斑马线；仅绕桶测试可以关闭白条任务。"""

    cruise_duty: int = C.CRUISE_DUTY
    cone_duty: int = C.LOW_FORWARD_DUTY
    approach_duty: int = C.CROSSWALK_APPROACH_DUTY
    hold_seconds: float = 10.0
    resume: bool = True
    max_age: float = C.MAX_FRAME_AGE_SECONDS
    reacquire_frames: int = 3
    reacquire_timeout: float = 3.0
    cone_max_seconds: float = 8.0
    steer_slew_us_s: float = 600.0
    cone_steer_slew_us_s: float = 600.0
    cross_enable_delay: float = C.CROSSWALK_ENABLE_DELAY_SECONDS
    crosswalk_enabled: bool = True
    cones_enabled: bool = True
    brake_on_cone_fault: bool = False
    cone_protection_enabled: bool = True

    def __post_init__(self):
        """创建参数后立即检查范围，避免错误动力或负数时间进入控制循环。"""
        if not all(math.isfinite(x) for x in vars(self).values()):
            raise ValueError('融合参数必须为有限数值')
        if not all(
            C.STOP_DUTY < x <= 16400
            for x in (self.cruise_duty, self.cone_duty, self.approach_duty)
        ):
            raise ValueError('前进 duty 必须大于 STOP_DUTY 且不超过 16400')
        if not (
            0 < self.max_age <= 0.25
            and self.hold_seconds >= 0
            and self.reacquire_frames >= 1
            and self.reacquire_timeout > 0
            and self.cone_max_seconds > 0
            and self.steer_slew_us_s > 0
            and self.cone_steer_slew_us_s > 0
            and self.cross_enable_delay >= 0
        ):
            raise ValueError('时序、帧数或转向参数无效')


@dataclass
class Command:
    """一条最终指令：当前状态、谁控方向、转向值、动力值、原因和是否播报。"""

    state: str
    owner: str
    steer_us: int
    duty: int
    reason: str = ''
    announce: bool = False


class MissionPolicy:
    """统一处理巡线、绕桶和斑马线停车，决定本帧的最终方向和动力。"""

    STOPPED = ('BRAKE', 'FAULT_BRAKE', 'HOLD', 'HALT', 'DONE')

    def __init__(self, settings=None):
        """保存行驶参数，从停车等待可靠车道开始，并清空各项计时。"""
        self.s = settings or Settings()
        self.state = 'REACQUIRE'
        self.owner = 'none'
        self.steer = float(C.STEER_CENTER_US)
        self.last_now = None
        self.reacquire_since = None
        self.lane_hits = 0
        self.cone_since = None
        self.brake_end = self.hold_end = None
        self.cross_done = False
        self.reason = ''

    def halt(self, reason):
        """锁定故障停车并返回停止命令；之后的画面不能自动解除，保留最早原因。"""
        self.state, self.owner = 'HALT', 'none'
        self.reason = self.reason or reason
        return Command('HALT', 'none', int(self.steer), C.STOP_DUTY, self.reason)

    def _command(self, duty, reason='', announce=False):
        """把当前状态和方向与指定动力装成Command，自己不写电机。"""
        return Command(
            self.state, self.owner, int(round(self.steer)), int(duty), reason, announce
        )

    def update(self, now, lane, cone, cross):
        """输入当前时间和三种检测结果，返回本帧应该怎样开。

        先处理已经开始的刹车/停车，再检查新的停止请求，最后选择
        由锥桶还是车道负责方向。斑马线接近时还能限制动力。
        """
        if self.state == 'HALT':
            return self.halt(self.reason)
        if self.state == 'DONE':
            return self._command(C.STOP_DUTY, 'CROSS_COMPLETE')
        dt = 1 / 30 if self.last_now is None else max(0, min(0.1, now - self.last_now))
        self.last_now = now

        if self.state == 'FAULT_BRAKE':
            if now < self.brake_end:
                return self._command(C.BRAKE_DUTY, self.reason)
            return self.halt(self.reason)

        # 刹车和保持不阻塞主循环，相机仍逐帧检查。
        if self.state == 'BRAKE':
            if now < self.brake_end:
                return self._command(C.CROSS_FR_BRAKE_DUTY, 'CROSS_FR_BRAKE')
            self.state, self.owner = 'HOLD', 'none'
            self.hold_end = now + self.s.hold_seconds
            return self._command(C.STOP_DUTY, 'CROSS_HOLD', announce=True)
        if self.state == 'HOLD':
            if now < self.hold_end:
                return self._command(C.STOP_DUTY, 'CROSS_HOLD')
            self.cross_done = True
            if not self.s.resume:
                self.state = 'DONE'
                return self._command(C.STOP_DUTY, 'CROSS_COMPLETE')
            self.state, self.reacquire_since = 'REACQUIRE', now
            self.lane_hits = 0
            return self._command(C.STOP_DUTY, 'RESUME_CHECK')

        if cone.stop:
            if self.s.brake_on_cone_fault and self.owner == 'cone':
                self.reason = 'CONE_' + (cone.reason or 'STOP')
                self.state, self.owner = 'FAULT_BRAKE', 'none'
                self.brake_end = now + C.BRAKE_TIME
                return self._command(C.BRAKE_DUTY, self.reason)
            return self.halt('CONE_' + (cone.reason or 'STOP'))
        if cross is not None and cross.brake_requested and not self.cross_done:
            self.state, self.owner = 'BRAKE', 'none'
            self.brake_end = now + C.CROSS_FR_BRAKE_TIME
            # 保留刹车前方向；不在刹车时突然叠加另一模块的转向。
            return self._command(C.CROSS_FR_BRAKE_DUTY, cross.trigger_reason)

        self.lane_hits = self.lane_hits + 1 if lane.valid else 0
        handoff_turn = getattr(cone, 'handoff_turn_us', None)
        continuous_handoff = (
            cone.state == 'STRAIGHT'
            and cone.reason == 'RESUME_STRAIGHT'
            and handoff_turn is not None
            and lane.valid
        )
        active = cone.state in ('AVOID', 'ALIGN', 'PASS') or abs(cone.turn_us) > 0.5
        lane_fallback = bool(getattr(cone, 'lane_follow_fallback', False)) and not active
        if lane_fallback and not lane.valid and self.state != 'REACQUIRE':
            self.state, self.owner = 'REACQUIRE', 'none'
            self.reacquire_since, self.cone_since = now, None
        if self.owner == 'cone' and not active:
            self.cone_since = None
            if (self.lane_hits < self.s.reacquire_frames and not continuous_handoff
                    and not (lane_fallback and lane.valid)):
                self.state, self.reacquire_since = 'REACQUIRE', now
        if self.state == 'REACQUIRE':
            self.owner = 'none'
            if self.reacquire_since is None:
                self.reacquire_since = now
            if now - self.reacquire_since > self.s.reacquire_timeout:
                return self.halt('LANE_REACQUIRE_TIMEOUT')
            if self.lane_hits < self.s.reacquire_frames:
                return self._command(C.STOP_DUTY, 'WAIT_RELIABLE_LANE')
            self.state, self.reacquire_since = 'CRUISE', None

        if active:
            # 回正仍在逐帧寻找白线，不能再被外层8秒总超时变成停车。
            # 如果当前桶重新挡路、再次避让，则从重新避让时计时。
            if self.cone_since is None or cone.state == 'ALIGN' or self.state == 'CONE_ALIGN':
                self.cone_since = now
            if (self.s.cone_protection_enabled
                    and now - self.cone_since > self.s.cone_max_seconds):
                return self.halt('CONE_TIMEOUT')
            self.state, self.owner = 'CONE_' + cone.state, 'cone'
            desired = C.STEER_CENTER_US + C.STEER_SIGN * cone.turn_us
            duty = min(self.s.cruise_duty, self.s.cone_duty)
        else:
            self.cone_since = None
            if lane.lost_count >= C.LANE_LOST_STOP_FRAMES:
                return self.halt('LANE_LOST')
            self.state, self.owner = 'CRUISE', 'lane'
            desired, duty = lane.steer_us, self.s.cruise_duty
            if continuous_handoff:
                # 已通过的桶带着本帧可靠白线交接，不插入停车等待或反方向巡线脉冲。
                desired = C.STEER_CENTER_US + C.STEER_SIGN * handoff_turn
                duty = min(duty, self.s.cone_duty)

        # 方向所有者与速度条件独立：绕桶时看到斑马线同样要降速。
        if (
            cross is not None
            and not self.cross_done
            and cross.state in ('ARMED', 'APPROACH')
        ):
            duty = min(duty, self.s.approach_duty)
            if not active:
                self.state = 'CROSS_APPROACH'
        rate = (
            self.s.cone_steer_slew_us_s
            if active or continuous_handoff
            else self.s.steer_slew_us_s
        )
        step = rate * dt
        self.steer += max(-step, min(step, desired - self.steer))
        self.steer = max(C.STEER_MIN_US, min(C.STEER_MAX_US, self.steer))
        return self._command(duty, cone.reason if active or lane_fallback else '')


class FusionController:
    """每张新图只处理一次，把检测结果交给统一的行驶判断。"""

    def __init__(
        self, settings=None, lane=None, cone=None, cross=None, cone_params=None
    ):
        """准备三种检测器；也允许测试时传入模拟检测器，不需要连接车。"""
        self.s = settings or Settings()
        self.policy = MissionPolicy(self.s)
        self.lane = lane if lane is not None else LaneFollower()
        params = replace(cone_params or Params(),
                         protection_stop=self.s.cone_protection_enabled)
        self.cone = cone if cone is not None else Avoider(params)
        self.mask_detector = Detector(params)
        self.cross = cross if cross is not None else CrosswalkDetector()
        self.lane_result = self.cone_result = self.cross_result = None
        self.last_id = self.last_ts = self.last_now = self.started = None
        self.motion_time = 0.0

    def cones_enabled(self):
        """普通联调允许全程检查锥桶；比赛版会按阶段改写这个判断。"""
        return self.s.cones_enabled

    def cross_enabled(self):
        """按本次选项决定是否检查斑马线；只巡线绕桶时返回关闭。"""
        return self.s.crosswalk_enabled

    def update_extra(self, frame, frame_id, captured_at, now):
        """普通联调无额外检测；比赛版用此处补上红绿灯和停车区检测。"""

    def extra_event(self):
        """普通联调没有额外日志字段；比赛版返回灯色、车位和比赛进度。"""
        return {}

    def draw_extra(self, frame):
        """普通联调直接返回图片；比赛版在这里补画红绿灯和车位。"""
        return frame

    def frame_for_cones(self, frame):
        """普通联调用原图找桶；比赛版可先遮掉已识别的停车挡板。"""
        return frame

    def step(self, frame, frame_id, captured_at, now):
        """输入图片、帧号、拍摄时间和当前时间，返回一条行驶命令。

        重复帧返回None，过期或顺序不对的图片返回停止命令。
        检查通过后先找桶的颜色区域，再更新斑马线和巡线，
        把本帧白线结果交给绕桶，最后统一判断；不会拿旧边界约束新图的动作。
        停车期间暂停绕桶计时，避免把停车10秒当成已经向前走了10秒。
        """
        p = self.policy
        self.perception_skipped = None
        if p.state == 'HALT':
            return p.halt(p.reason)
        if not (math.isfinite(now) and math.isfinite(captured_at)):
            return p.halt('BAD_TIMESTAMP')
        if not 0 <= now - captured_at <= self.s.max_age:
            return p.halt('STALE_FRAME')
        if self.last_now is not None and now < self.last_now:
            return p.halt('CLOCK_REVERSED')
        if frame is None:
            return p.halt('NO_FRAME')
        if (
            frame.shape != (C.CAMERA_HEIGHT, C.CAMERA_WIDTH, 3)
            or str(frame.dtype) != 'uint8'
        ):
            return p.halt('CAMERA_SIZE_OR_FORMAT')
        if frame_id == self.last_id:
            return None
        if self.last_id is not None and (
            frame_id < self.last_id or captured_at <= self.last_ts
        ):
            return p.halt('FRAME_ORDER')
        self.update_extra(frame, frame_id, captured_at, now)
        if p.state == 'HALT':
            return p.halt(p.reason)
        dt = 0 if self.last_now is None else now - self.last_now
        stopped = p.state in p.STOPPED or p.state == 'REACQUIRE'
        # 用暂停过的同一时间轴给锥桶跟踪器、反打及通过计时。
        # REACQUIRE 中也暂停计时，并暂缓锥桶更新，避免原地投票成“已通过”。
        if not stopped:
            self.motion_time += dt
        elif p.cone_since is not None:
            p.cone_since += dt
        self.last_id, self.last_ts, self.last_now = frame_id, captured_at, now
        if self.started is None:
            self.started = now

        # A completed crosswalk brake already holds neutral. Keep all frame
        # validation above, but do not run lane vision during the timed wait.
        # At the deadline the ordinary path runs again on a fresh frame before
        # handing off; its lane-reacquisition checks still gate forward motion.
        if (getattr(p, 'stage', None) == 'CROSSWALK' and p.state == 'HOLD'
                and p.hold_end is not None and now < p.hold_end):
            self.perception_skipped = 'CROSSWALK_HOLD'
            return p.update(now, self.lane_result, self.cone_result, self.cross_result)

        enabled = self.cones_enabled()
        cone_frame = self.frame_for_cones(frame) if enabled else frame
        cone_updated = enabled and (not stopped or self.cone_result is None)
        if not enabled:
            self.cone_result = Decision('STRAIGHT', 0.0, False, '', 0)
        if self.cone_result is None:
            self.cone_result = Decision('STRAIGHT', 0.0, False, '', 0)
        # 巡线去掉彩色候选的边缘，避免把桶边识别为白线；斑马线仍用原图。
        lane_frame = cone_frame
        mask = None
        if enabled:
            # 先取本帧颜色区域给巡线使用；只检测，不推进绕桶阶段或计时。
            # 后面把同一张图的车道结果交给绕桶，不能用上一帧边界限制转向。
            mask, _ = self.mask_detector.detect(cone_frame)
        if mask is not None:
            import cv2
            import numpy as np

            lane_frame = cone_frame.copy()
            lane_frame[cv2.dilate(mask, np.ones((3, 3), np.uint8)) > 0] = 0
        if not self.s.crosswalk_enabled:
            # 明确关闭白条任务时也清掉旧结果，避免旧停车请求参与判断。
            self.cross_result = None
        if (
            self.cross_enabled()
            and not p.cross_done
            and p.state not in p.STOPPED
            and now - self.started >= self.s.cross_enable_delay
        ):
            timed_update = getattr(self.cross, 'update_timed', None)
            if timed_update is None:
                self.cross_result = self.cross.update(frame)
            else:
                self.cross_result = timed_update(frame, frame_id, captured_at, now)
        slow = (
            self.cone_result.state != 'STRAIGHT'
            or p.state == 'REACQUIRE'
            or (
                not p.cross_done
                and self.cross_result is not None
                and self.cross_result.state in ('ARMED', 'APPROACH')
            )
        )
        if isinstance(self.lane, LaneFollower):
            self.lane.maneuver_active = enabled and self.cone_result.state in (
                'AVOID',
                'PASS',
                'ALIGN',
            )
        traffic_timing = getattr(self, 'traffic_timing_ms', None)
        if traffic_timing is not None:
            lane_started = time.perf_counter()
        self.lane_result = self.lane.update(
            lane_frame, profile_name='approach' if slow else 'cruise'
        )
        if traffic_timing is not None:
            traffic_timing['lane_ms'] = round((time.perf_counter() - lane_started) * 1000, 3)
        if isinstance(self.cone, Avoider):
            lr = self.lane_result
            left, right = getattr(lr, 'left_x', None), getattr(lr, 'right_x', None)
            measured = getattr(lr, 'measured_center_x', None)
            error = (
                measured - lr.target_x
                if measured is not None
                else getattr(lr, 'error_px', float('inf'))
            )
            self.cone.set_lane_hint(
                valid=bool(lr.valid),
                turn_us=(lr.steer_us - C.STEER_CENTER_US) / C.STEER_SIGN,
                front_x=self.cone.p.front_x,
                left_x=None if left is None else left / C.CAMERA_WIDTH,
                right_x=None if right is None else right / C.CAMERA_WIDTH,
                error_px=error,
                heading_valid=getattr(lr, 'heading_valid', False),
                heading_error_px=getattr(lr, 'heading_error_px', None),
            )
        if cone_updated:
            # 内部使用暂停后的运动时钟，停车十秒不会计入绕桶动作。
            self.cone_result = self.cone.update(
                cone_frame, frame_id, self.motion_time, now=self.motion_time
            )
        if (
            enabled
            and p.state == 'REACQUIRE'
            and self.lane_result.valid
            and p.lane_hits + 1 >= self.s.reacquire_frames
            and not cone_updated
        ):
            # 放行前消费当前帧，让新出现的无路/故障请求可以否决前进。
            # 只增加1微秒以满足跟踪器严格递增要求，不计入实际停车时长。
            # 兼容不提供 math.nextafter 的树莓派 Python。
            self.motion_time = self.motion_time + 1e-6
            self.cone_result = self.cone.update(
                cone_frame, frame_id, self.motion_time, now=self.motion_time
            )
        was_done = p.cross_done
        cmd = p.update(now, self.lane_result, self.cone_result, self.cross_result)
        if p.cross_done and not was_done:
            self.cross.mark_done()
            # 清掉停车前 PD 的差分记忆，首帧按新的误差重新建立。
            if hasattr(self.lane, 'last_error'):
                self.lane.last_error = 0.0
        return cmd
