#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""从车头画面识别左右车道线，计算下一次转向，并提供检查画面。

使用白色边界检测提取候选，再用本车几何和近期位置排除错误线。
只有双线积累足够后，才允许用已知车道宽度短时补足缺失的一侧。
本文件只给出转向建议，不操作摄像头、电机或舵机。"""

from dataclasses import dataclass
from collections import deque
import json
import math

import cv2
import numpy as np

import config as C
import wisecar_vision as W

LANE_VERSION = W.VERSION + '_TURN_TRACK_V1'


@dataclass
class LaneResult:
    """保存一帧的巡线结果：建议转向、车道位置、是否可信和检查用图像。"""

    steer_us: int
    center_x: float
    target_x: float
    error_px: float
    valid: bool
    degraded: bool
    lost_count: int
    mode: str
    left_line: object
    right_line: object
    lane_width: float
    binary: np.ndarray
    measured_center_x: object = None  # 本帧测量值，不是旧中心或平滑后的值
    left_x: object = None  # 参考高度处的横向像素位置；单边时另一边按已知宽度补足
    right_x: object = None
    heading_error_px: object = None  # 左右白线延长交点相对起步方向的横向变化
    heading_valid: bool = False
    left_points: object = None
    right_points: object = None


class LaneFollower:
    """连续读取画面中的车道线，并根据车道中心的偏移量计算转向。

    会记住最近可信的中心和宽度；画面不可信时保留上一次转向，
    由调用方结合连续丢线次数决定是否停车。"""

    def __init__(self):
        """读取车头对准位置，并清空车道宽度、历史中心和丢线次数。"""
        # Original follows runtime config; backup uses an instance-local copy.
        self.limits = C
        self.vp_height_tolerance_ratio = .06
        self.target_x_ratio = self._load_target_ratio()
        self.target_x = C.CAMERA_WIDTH * self.target_x_ratio

        self.last_center_x = self.target_x
        self.last_error = 0.0
        self.last_steer_us = C.STEER_CENTER_US

        self.last_lane_width = None
        self.dual_seen_count = 0
        self.lost_count = 0

        # 保存最近可靠的中心，避免某一帧选错直线就突然打方向。
        self.center_history = deque(maxlen=C.LANE_CENTER_HISTORY)
        self.maneuver_active = False
        self.relocation_samples = deque(maxlen=3)
        self.geometry_reference = None
        self.geometry_samples = deque(maxlen=3)

    def _confirm_relocated(self, center, width, mode):
        """双线连续三帧稳定或绕桶中平滑移动时更新旧位置；单帧跳变仍拒绝。

        双线进入这里前已经通过宽度与透视检查。普通巡线只允许双线重新确认；
        只见单边时另一边靠旧宽度推算，证据较弱，仍只在绕桶期间允许重建。
        """
        if not self.maneuver_active and mode != 'DUAL':
            self.relocation_samples.clear()
            return False
        if (
            self.center_history
            and abs(center - float(np.median(self.center_history)))
            > 2.5 * self.limits.LANE_CENTER_MAX_JUMP_PX
        ):
            # 连续三帧也不能把画面另一边的背景重新认成原来的赛道。
            self.relocation_samples.clear()
            return False
        samples = self.relocation_samples
        # During a turn, a real pair keeps moving instead of settling at one
        # pixel. Require three geometrically valid, smooth measurements.
        moving_pair = self.maneuver_active and mode == 'DUAL'
        step_limit = 24 if moving_pair else 12
        if samples and (
            samples[-1][2] != mode
            or abs(samples[-1][0] - center) > step_limit
            or abs(samples[-1][1] - width) > 0.15 * max(width, 1)
        ):
            samples.clear()
        samples.append((center, width, mode))
        if len(samples) < 3:
            return False
        spread = max(s[0] for s in samples) - min(s[0] for s in samples)
        if spread > 12:
            steps = [samples[i+1][0] - samples[i][0] for i in range(2)]
            if not (moving_pair and steps[0]*steps[1] > 0
                    and max(abs(v) for v in steps) <= step_limit
                    and abs(steps[1]-steps[0]) <= 8):
                return False
        self.center_history.clear()
        # 新位置已经连续确认，补足有效历史，下一帧仍执行跳变检查。
        self.center_history.extend([center] * C.LANE_TEMPORAL_WARMUP_DUAL)
        self.last_center_x = center
        self.last_error = 0.0
        self.last_lane_width = width
        samples.clear()
        return True

    def _heading_measurement(self, pair):
        """起步时用三张稳定双线图记住朝向；以后只测变化，不追着错误画面重设基准。"""
        if pair is None or 'vp_x' not in pair or 'vp_y' not in pair:
            return None
        point = (pair['vp_x'], pair['vp_y'])
        if self.geometry_reference is None:
            if self.maneuver_active:
                return None
            if (
                self.geometry_samples
                and max(abs(a - b) for a, b in zip(point, self.geometry_samples[-1]))
                > 10
            ):
                self.geometry_samples.clear()
            self.geometry_samples.append(point)
            if len(self.geometry_samples) < 3:
                return None
            self.geometry_reference = tuple(np.median(self.geometry_samples, axis=0))
        return float(point[0] - self.geometry_reference[0])

    @staticmethod
    def _load_target_ratio():
        """读取保存的车头对准位置；文件缺失、损坏或数值越界时使用默认值。"""
        if C.LANE_TARGET_FILE.exists():
            try:
                data = json.loads(C.LANE_TARGET_FILE.read_text(encoding="utf-8"))
                ratio = float(data["target_x_ratio"])

                if 0.15 <= ratio <= 0.85:
                    return ratio
            except Exception:
                pass

        return float(C.LANE_TARGET_X_RATIO_DEFAULT)

    @staticmethod
    def _acute_angle_deg(dx, dy):
        """根据线段两端的横纵差，返回它与水平线之间的较小夹角。"""
        angle = abs(math.degrees(math.atan2(dy, dx)))

        if angle > 90.0:
            angle = 180.0 - angle

        return angle

    @staticmethod
    def _line_x(line, y):
        """给定一条直线和画面中的高度，算出这条线在该高度的横向位置。"""
        return line["a"] * y + line["b"]

    @staticmethod
    def _candidate_to_draw_line(line, roi_y0, roi_h, image_w):
        """沿实际直线截取画面内的部分；不能只压缩端点横坐标，那样会把绿线画歪。"""
        if line is None:
            return None

        observed = line.get('observed_points')
        if observed and len(observed) >= 2:
            return (tuple(map(lambda v: int(round(v)), observed[-1])),
                    tuple(map(lambda v: int(round(v)), observed[0])))

        y_top = 0.0
        y_bottom = float(roi_h - 1)

        x_top = int(round(LaneFollower._line_x(line, y_top)))
        x_bottom = int(round(LaneFollower._line_x(line, y_bottom)))

        visible, start, end = cv2.clipLine(
            (0, int(roi_y0), int(image_w), int(roi_h)),
            (x_bottom, roi_y0 + int(y_bottom)),
            (x_top, roi_y0 + int(y_top)),
        )
        return (start, end) if visible else None

    def _get_candidates(self, roi, frame):
        """提取独立白线候选；输入整帧，输出仍用本车像素坐标。"""
        return W.candidates(frame, C.LANE_ROI_Y0_RATIO, C.LANE_ROI_Y1_RATIO,
                            C.LANE_PAIR_REF_RATIO)

    @staticmethod
    def _best_single(candidates):
        """从一侧候选线中选出最长的一条；没有候选时返回空值。"""
        if not candidates:
            return None

        return max(candidates, key=lambda c: c["length"])

    def _best_pair(self, left, right, roi_h, image_w, ref_y, bottom_y):
        """结合线长、车道宽度和两线交点，选出最可信的一对左右线。

        各高度均相对选定区域计算；bottom_y 保留在原调用接口中，
        底部位置已由候选线记录。找不到合适组合时返回空值。"""
        best = None
        best_rank = (-float('inf'), -float('inf'))

        for l in left:
            for r in right:
                width_ref = r["x_ref"] - l["x_ref"]
                width_bottom = r["x_bottom"] - l["x_bottom"]

                if not (
                    self.limits.LANE_PAIR_MIN_WIDTH_RATIO * image_w
                    <= width_ref
                    <= C.LANE_PAIR_MAX_WIDTH_RATIO * image_w
                ):
                    continue

                if not (
                    self.limits.LANE_PAIR_MIN_BOTTOM_WIDTH_RATIO * image_w
                    <= width_bottom
                    <= C.LANE_PAIR_MAX_BOTTOM_WIDTH_RATIO * image_w
                ):
                    continue

                # 透视下靠近车的位置应该更宽。
                if width_bottom < 0.95 * width_ref:
                    continue

                denom = l["a"] - r["a"]

                if abs(denom) < 1e-6:
                    continue

                # 算出两条线延长后的交点。
                vp_y = (r["b"] - l["b"]) / denom
                vp_x = l["a"] * vp_y + l["b"]

                if self.geometry_reference is not None and abs(
                    vp_y - self.geometry_reference[1]
                ) > max(12, C.CAMERA_HEIGHT * self.vp_height_tolerance_ratio):
                    # 云台固定时，路面白线延长后的交会高度不应突然换到墙面或近地面。
                    continue

                if not (
                    C.LANE_VP_MIN_Y_RATIO * roi_h
                    <= vp_y
                    <= self.limits.LANE_VP_MAX_Y_RATIO * ref_y
                ):
                    continue

                # 交点可以偏左或偏右，但不能离图像太远。
                if not (-0.30 * image_w <= vp_x <= 1.30 * image_w):
                    continue

                # 主奖励：两条线段越长越可靠。
                score = l["length"] + r["length"]

                # 交点离画面中部越远，得分略微降低。
                score -= 0.12 * abs(vp_x - image_w / 2.0)

                # 测量高度处的车道宽度偏离参考值越多，得分略微降低。
                score -= 0.03 * abs(width_ref - 0.45 * image_w)

                # Prefer the already confirmed tape when several plausible
                # fragments exist. A longer floor crease must not win every
                # other frame. This ranks evidence; jump/relocation guards below
                # still decide whether the selected measurement is usable.
                if len(self.center_history) >= C.LANE_TEMPORAL_WARMUP_DUAL:
                    center = (l['x_ref'] + r['x_ref']) / 2.0
                    score -= 2.0 * abs(center - float(np.median(self.center_history)))
                    if self.last_lane_width is not None:
                        score -= 2.0 * abs(width_ref - self.last_lane_width)

                # On a multi-lane athletics track choose the immediate pair
                # surrounding the calibrated vehicle target, not a longer line
                # from the next lane. Blue-track ranking remains unchanged.
                playground = l.get('surface') == r.get('surface') == 'red_track'
                if playground and not l['x_ref'] < self.target_x < r['x_ref']:
                    # A turn can put both observed boundaries on one side of
                    # the calibrated vehicle point. Keep the original lane
                    # only when it remains near the established pair and width.
                    tracked_pair = (
                        self.maneuver_active
                        and len(self.center_history) >= C.LANE_TEMPORAL_WARMUP_DUAL
                        and self.last_lane_width is not None
                        and abs((l['x_ref']+r['x_ref'])/2.
                                - float(np.median(self.center_history)))
                            <= 2.5*self.limits.LANE_CENTER_MAX_JUMP_PX
                        and abs(width_ref-self.last_lane_width)
                            <= self.limits.LANE_WIDTH_MAX_REL_JUMP*self.last_lane_width
                    )
                    if not tracked_pair:
                        continue
                rank = (-width_ref if playground else 0., score)
                if rank > best_rank:
                    best_rank = rank
                    best = {
                        "left": l,
                        "right": r,
                        "width_ref": float(width_ref),
                        "width_bottom": float(width_bottom),
                        "vp_x": float(vp_x),
                        "vp_y": float(vp_y),
                        "score": float(score),
                    }

        return best

    def _pd(self, center_x, profile_name):
        """按指定速度档的参数计算转向，保存本次偏差，并返回舵机脉宽和偏差。"""
        profile = C.PID_PROFILES[profile_name]

        center_filtered = (
            C.LANE_CENTER_FILTER * center_x
            + (1.0 - C.LANE_CENTER_FILTER) * self.last_center_x
        )

        self.last_center_x = center_filtered

        error = center_filtered - self.target_x

        if abs(error) < C.LANE_ERROR_DEADBAND_PX:
            error = 0.0

        correction = profile["kp_us_per_px"] * error + profile["kd_us_per_px"] * (
            error - self.last_error
        )

        self.last_error = error

        if 0 < abs(correction) < profile["min_effective_us"]:
            correction = math.copysign(profile["min_effective_us"], correction)

        correction = max(
            -profile["max_correction_us"], min(profile["max_correction_us"], correction)
        )

        steer_us = C.STEER_CENTER_US + C.STEER_SIGN * correction

        steer_us = int(max(C.STEER_MIN_US, min(C.STEER_MAX_US, steer_us)))

        self.last_steer_us = steer_us

        return steer_us, error

    def update(self, frame, profile_name="cruise"):
        """处理一张彩色画面，更新车道历史，返回本帧的巡线和转向结果。"""
        h, w = frame.shape[:2]

        roi_y0 = int(h * C.LANE_ROI_Y0_RATIO)
        roi_y1 = int(h * C.LANE_ROI_Y1_RATIO)

        roi_y0 = max(0, min(h - 2, roi_y0))
        roi_y1 = max(roi_y0 + 2, min(h, roi_y1))

        roi = frame[roi_y0:roi_y1, :]

        (binary, left_candidates, right_candidates, ref_y, bottom_y) = (
            self._get_candidates(roi, frame)
        )

        pair = self._best_pair(
            left_candidates, right_candidates, roi.shape[0], w, ref_y, bottom_y
        )

        left_line_model = None
        right_line_model = None

        center_x = self.last_center_x
        lane_width = self.last_lane_width if self.last_lane_width is not None else 0.0

        valid = False
        degraded = False
        mode = "LOST"
        measured_center = None
        left_x = right_x = None

        # ----------------------------------------------------
        # 最优情况：真正找到一对左右车道线
        # ----------------------------------------------------

        if pair is not None:
            left_line_model = pair["left"]
            right_line_model = pair["right"]

            left_x = left_line_model["x_ref"]
            right_x = right_line_model["x_ref"]

            measured_width = right_x - left_x
            measured_center = (left_x + right_x) / 2.0

            temporal_ok = True

            # 1) 中心跳变检查：已有历史后，不接受单帧大跳。
            if len(self.center_history) >= C.LANE_TEMPORAL_WARMUP_DUAL:
                median_center = float(np.median(self.center_history))

                if abs(measured_center - median_center) > self.limits.LANE_CENTER_MAX_JUMP_PX:
                    temporal_ok = False
                    mode = "CENTER_JUMP"

            # 2) 车道宽度跳变检查。
            if (
                temporal_ok
                and self.last_lane_width is not None
                and self.dual_seen_count >= C.LANE_TEMPORAL_WARMUP_DUAL
            ):
                rel_jump = abs(measured_width - self.last_lane_width) / max(
                    self.last_lane_width, 1.0
                )

                if rel_jump > self.limits.LANE_WIDTH_MAX_REL_JUMP:
                    temporal_ok = False
                    mode = "WIDTH_JUMP"

            if not temporal_ok:
                temporal_ok = self._confirm_relocated(
                    measured_center, measured_width, 'DUAL'
                )

            if temporal_ok:
                self.center_history.append(measured_center)

                # 控制使用最近中心的中位数，比单帧值稳定。
                center_x = float(np.median(self.center_history))

                if self.last_lane_width is None:
                    self.last_lane_width = measured_width
                else:
                    self.last_lane_width = (
                        C.LANE_WIDTH_FILTER * measured_width
                        + (1.0 - C.LANE_WIDTH_FILTER) * self.last_lane_width
                    )

                lane_width = self.last_lane_width

                self.dual_seen_count += 1
                self.lost_count = 0

                valid = True
                degraded = False
                mode = "DUAL"
            else:
                # 线画出来用于诊断，但该帧不参与控制。
                valid = False
                degraded = True
                self.lost_count += 1

        else:
            # ------------------------------------------------
            # 没配成双线时，尝试用已确认的车道宽度补足单边。
            # 启动阶段还没有可靠宽度，因此不能直接补线。
            # ------------------------------------------------

            best_left = self._best_single(left_candidates)
            best_right = self._best_single(right_candidates)

            can_single_fallback = (
                self.last_lane_width is not None
                and self.dual_seen_count >= C.LANE_SINGLE_FALLBACK_MIN_DUAL_FRAMES
            )

            if can_single_fallback and best_left is not None and best_right is None:
                left_line_model = best_left

                proposed_center = best_left["x_ref"] + self.last_lane_width / 2.0

                if self.center_history:
                    median_center = float(np.median(self.center_history))

                    if abs(
                        proposed_center - median_center
                    ) > self.limits.LANE_CENTER_MAX_JUMP_PX and not self._confirm_relocated(
                        proposed_center, self.last_lane_width, 'LEFT'
                    ):
                        valid = False
                        degraded = True
                        mode = "LEFT_JUMP"
                        self.lost_count += 1
                    else:
                        center_x = proposed_center
                        lane_width = self.last_lane_width
                        valid = True
                        degraded = True
                        mode = "LEFT_ONLY"
                        self.lost_count = 0
                else:
                    valid = False
                    degraded = True
                    mode = "LEFT_SEEN"
                    self.lost_count += 1

            elif can_single_fallback and best_right is not None and best_left is None:
                right_line_model = best_right

                proposed_center = best_right["x_ref"] - self.last_lane_width / 2.0

                if self.center_history:
                    median_center = float(np.median(self.center_history))

                    if abs(
                        proposed_center - median_center
                    ) > self.limits.LANE_CENTER_MAX_JUMP_PX and not self._confirm_relocated(
                        proposed_center, self.last_lane_width, 'RIGHT'
                    ):
                        valid = False
                        degraded = True
                        mode = "RIGHT_JUMP"
                        self.lost_count += 1
                    else:
                        center_x = proposed_center
                        lane_width = self.last_lane_width
                        valid = True
                        degraded = True
                        mode = "RIGHT_ONLY"
                        self.lost_count = 0
                else:
                    valid = False
                    degraded = True
                    mode = "RIGHT_SEEN"
                    self.lost_count += 1

            else:
                # 启动阶段看到单边，只用于诊断，不参与控制。
                if best_left is not None and best_right is not None:
                    mode = "PAIR_REJECTED"
                    left_line_model = best_left
                    right_line_model = best_right

                elif best_left is not None:
                    mode = "LEFT_SEEN"
                    left_line_model = best_left

                elif best_right is not None:
                    mode = "RIGHT_SEEN"
                    right_line_model = best_right

                else:
                    mode = "LOST"

                valid = False
                degraded = True
                self.lost_count += 1

        # ----------------------------------------------------
        # 控制
        # ----------------------------------------------------

        if valid:
            self.relocation_samples.clear()
            if mode == 'LEFT_ONLY':
                measured_center = proposed_center
                left_x = best_left['x_ref']
                right_x = left_x + lane_width
            elif mode == 'RIGHT_ONLY':
                measured_center = proposed_center
                right_x = best_right['x_ref']
                left_x = right_x - lane_width
            steer_us, error = self._pd(center_x, profile_name)
        else:
            if mode not in ('CENTER_JUMP', 'WIDTH_JUMP', 'LEFT_JUMP', 'RIGHT_JUMP'):
                self.relocation_samples.clear()
            # 本帧视觉不可信时，不根据错误的单线中心打方向。
            # 保持上一个舵机命令；尚未得到有效结果时保持配置中的转向中位。
            steer_us = self.last_steer_us
            error = self.last_error

        heading_error = (
            self._heading_measurement(pair) if valid and mode == 'DUAL' else None
        )
        if self.geometry_reference is None and (not valid or mode != 'DUAL'):
            self.geometry_samples.clear()

        left_draw = self._candidate_to_draw_line(
            left_line_model, roi_y0, roi.shape[0], w
        )

        right_draw = self._candidate_to_draw_line(
            right_line_model, roi_y0, roi.shape[0], w
        )

        return LaneResult(
            steer_us=int(steer_us),
            center_x=float(center_x),
            target_x=float(self.target_x),
            error_px=float(error),
            valid=bool(valid),
            degraded=bool(degraded),
            lost_count=int(self.lost_count),
            mode=mode,
            left_line=left_draw,
            right_line=right_draw,
            lane_width=float(lane_width),
            binary=binary,
            measured_center_x=measured_center,
            left_x=left_x if valid else None,
            right_x=right_x if valid else None,
            heading_error_px=heading_error,
            heading_valid=heading_error is not None,
            left_points=left_line_model.get('observed_points') if left_line_model else None,
            right_points=right_line_model.get('observed_points') if right_line_model else None,
        )

    @staticmethod
    def draw_debug(frame, result):
        """复制原图：绿线为有效车道，橙线为未通过的候选；红点为车道中心，紫线为车头对准位置。"""
        out = frame.copy()

        h, w = out.shape[:2]

        roi_y0 = int(h * C.LANE_ROI_Y0_RATIO)
        roi_y1 = int(h * C.LANE_ROI_Y1_RATIO)

        roi_y0 = max(0, min(h - 2, roi_y0))
        roi_y1 = max(roi_y0 + 2, min(h, roi_y1))

        # 未通过检查的候选不能也画成绿色，以免看图时误认为已经放行。
        line_color = (0, 255, 0) if result.valid else (0, 165, 255)
        for points, line in ((result.left_points, result.left_line),
                             (result.right_points, result.right_line)):
            if points:
                for p, q in zip(points, points[1:]):
                    if q[1]-p[1] <= 3:
                        cv2.line(out, tuple(int(round(v)) for v in p),
                                 tuple(int(round(v)) for v in q), line_color, 1)
            elif line is not None:
                cv2.line(out, line[0], line[1], line_color, 1)

        ref_y = int(roi_y0 + C.LANE_PAIR_REF_RATIO * (roi_y1 - roi_y0 - 1))

        cx = int(round(result.center_x))
        tx = int(round(result.target_x))

        cv2.circle(out, (max(0, min(w - 1, cx)), ref_y), 5, (0, 0, 255), -1)

        text = (
            f"lane={result.mode} "
            f"center={result.center_x:.1f} "
            f"target={result.target_x:.1f} "
            f"width={result.lane_width:.1f} "
            f"err={result.error_px:.1f} "
            f"steer={result.steer_us}"
        )

        cv2.putText(
            out,
            text,
            (5, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 255, 0) if result.valid else (0, 0, 255),
            1,
        )

        return out
