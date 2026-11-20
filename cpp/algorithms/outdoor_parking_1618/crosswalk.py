#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""识别斑马线白条，连续确认位置，并向主程序提出停车请求。

远处用横向排列的完整白条确认斑马线，近处另查较大的白块。
根据配置跟踪白条上沿或近处白块下沿；位置是图像高度比例，不是米数。
本文件只给出识别结果和停车请求，实际刹车与停车计时由主程序负责。"""

from collections import deque
from dataclasses import dataclass
import statistics

import cv2
import numpy as np

import config as C


@dataclass
class CrosswalkResult:
    """保存斑马线识别状态、图像中的位置、停车请求和检查画面所需的数据。"""

    state: str

    # 完整白条或近处白块任一被识别时，这里记录总体结果。
    detected: bool
    bottom: float

    # 分别保留两种检测结果，方便回看原因。
    far_detected: bool
    near_detected: bool
    far_bottom: float
    near_bottom: float
    near_bottom_tracked: float

    candidate_count: int
    valid_count: int
    near_valid_count: int

    brake_requested: bool
    trigger_reason: str
    miss_streak: int

    binary: np.ndarray
    valid_rects: list
    near_rects: list
    roi_box: tuple
    distance_metric: str = "near_bottom"
    distance_raw: float = 0.0
    distance_tracked: float = 0.0


class CrosswalkDetector:
    """逐帧寻找斑马线，并按连续出现的次数确认接近和停车条件。

    SEARCH 表示寻找，ARMED 表示已确认白条，APPROACH 表示进入接近阶段，
    DONE 表示本次任务已完成。位置阈值和所需次数均从配置读取。"""

    SEARCH = "SEARCH"
    ARMED = "ARMED"
    APPROACH = "APPROACH"
    DONE = "DONE"

    def __init__(self):
        """从寻找状态开始，清空各阶段的确认记录、位置记录和丢失次数。"""
        self.state = self.SEARCH

        self.arm_hist = deque(maxlen=C.CROSS_ARM_HISTORY)

        self.approach_hist = deque(maxlen=C.CROSS_APPROACH_HISTORY)

        self.brake_hist = deque(maxlen=C.CROSS_BRAKE_HISTORY)

        self.miss_streak = 0

        # 连续记录图像中的位置，排除只出现一帧的大幅跳动。
        self.near_track = None
        self.near_jump_candidate = None
        self.near_jump_count = 0
        self.idle_miss_streak = 0
        self.close_hist = deque(maxlen=3)
        self.previous_seen_position = None

    # ========================================================
    # 提取白色部分
    # ========================================================

    @staticmethod
    def _build_binary(roi, v_min=None):
        """从选定区域提取白色部分，返回黑白图；可指定更低亮度以查找阴影白条。"""
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        white = cv2.inRange(
            hsv,
            np.array([0, 0, C.CROSS_V_MIN if v_min is None else v_min], dtype=np.uint8),
            np.array([180, C.CROSS_S_MAX, 255], dtype=np.uint8),
        )

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        gray2 = clahe.apply(gray)

        gray2 = cv2.GaussianBlur(gray2, (5, 5), 0)

        _, otsu = cv2.threshold(gray2, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        mask = cv2.bitwise_and(white, otsu)

        # 按配置保留只有几像素高的远处白条。
        if getattr(C, "CROSS_PRESERVE_THIN_STRIPES", False):
            return mask

        k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3, iterations=1)

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=1)

        return mask

    # ========================================================
    # 寻找横向排列的完整白条
    # ========================================================

    @staticmethod
    def _far_candidate(cnt, allow_square=False):
        """检查一个轮廓是否像完整白条，返回位置和尺寸；不合适时返回空值。"""
        area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        # Distant paint may be only four pixels high. Admit these small shapes
        # provisionally; a row using any of them must have four separate stripes.
        small = area < C.CROSS_MIN_AREA
        thin_distant = (getattr(C, 'CROSS_USE_STRIPE_TOP', False)
                        and 18 <= area and w >= 9 and 4 <= h <= 7)
        if (small and not thin_distant) or area > C.CROSS_MAX_AREA:
            return None

        if w < 5 or h < 4:
            return None

        rr = cv2.minAreaRect(cnt)
        rw, rh = rr[1]

        if rw < 1 or rh < 1:
            return None

        long_side = max(rw, rh)
        short_side = min(rw, rh)

        ratio = long_side / short_side

        # 靠近时，白条在画面中也可能接近正方形。
        min_ratio = 1.0 if allow_square else C.CROSS_MIN_LONG_SHORT
        if ratio < min_ratio or ratio > C.CROSS_MAX_LONG_SHORT:
            return None

        fill = area / max(float(w * h), 1.0)

        if fill < C.CROSS_MIN_FILL:
            return None

        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)

        solidity = area / max(hull_area, 1.0)

        if solidity < C.CROSS_MIN_SOLIDITY:
            return None

        return {
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "cx": x + w / 2.0,
            "cy": y + h / 2.0,
            "bottom": y + h,
            "fill": fill,
            "small": small,
        }

    @staticmethod
    def _choose_far_row(candidates, roi_w, roi_h):
        """从候选白条中选出横向排列且宽度足够的一排，没有合适排列时返回空列表。"""
        # RETR_LIST may yield an outer contour and holes from the same stripe.
        # Count distinct horizontal paint locations, never contour multiplicity.
        distinct = []
        for c in sorted(candidates, key=lambda c: c['w'] * c['h'], reverse=True):
            if any(c['x'] < s['x'] + s['w'] and s['x'] < c['x'] + c['w']
                   and c['y'] < s['y'] + s['h'] and s['y'] < c['y'] + c['h']
                   for s in distinct):
                continue
            distinct.append(c)
        candidates = distinct
        if len(candidates) < C.CROSS_MIN_RECTS:
            return []

        # 用上沿判断位置时，排列检查也比较上沿，避免白条高矮不同造成误差。
        row_key = "y" if getattr(C, "CROSS_USE_STRIPE_TOP", False) else "cy"
        best = []
        best_span = 0.0

        def independent(row):
            # Global XY deduplication keeps objects on other rows. Inside a
            # proposed row, stacked fragments still belong to one stripe.
            kept = []
            for c in sorted(row, key=lambda v: v['w']*v['h'], reverse=True):
                if not any(c['x'] < s['x']+s['w'] and s['x'] < c['x']+c['w'] for s in kept):
                    kept.append(c)
            return sorted(kept, key=lambda c: c['cx'])

        def enough(row):
            minimum = max(C.CROSS_MIN_RECTS, 4 if any(c.get('small', False) for c in row) else 0)
            return len(row) >= minimum

        for anchor in candidates:
            group = [
                c
                for c in candidates
                if abs(c[row_key] - anchor[row_key])
                <= (C.CROSS_MAX_Y_SPAN_RATIO * roi_h / 2.0)
            ]

            if len(group) < C.CROSS_MIN_RECTS:
                continue

            group = independent(group)

            for i in range(len(group)):
                for j in range(i + C.CROSS_MIN_RECTS, len(group) + 1):
                    g = group[i:j]
                    if not enough(g):
                        continue

                    ys = [c[row_key] for c in g]

                    if max(ys) - min(ys) > (C.CROSS_MAX_Y_SPAN_RATIO * roi_h):
                        continue

                    xs = [c["cx"] for c in g]

                    x_span = max(xs) - min(xs)

                    # Four distinct thin distant stripes can span the required
                    # width by their actual paint edges while their centers do
                    # not. Three-component hypotheses keep the stricter rule.
                    supported_span = x_span
                    if len(g) >= 4:
                        supported_span = max(c['x']+c['w'] for c in g) - min(c['x'] for c in g)
                    if supported_span < (C.CROSS_MIN_X_SPAN_RATIO * roi_w):
                        continue

                    if len(g) > len(best) or (
                        len(g) == len(best) and x_span > best_span
                    ):
                        best = g
                        best_span = x_span

        if best or not getattr(C, 'CROSS_USE_STRIPE_TOP', False):
            return best

        # Perspective and a modest vehicle yaw make real stripe tops slant.
        # Fit a bounded row through independent components, not any three blocks.
        tolerance = max(3.0, C.CROSS_MAX_Y_SPAN_RATIO * roi_h / 2.0)
        ordered = sorted(candidates, key=lambda c: c['cx'])
        for i, a in enumerate(ordered):
            for b in ordered[i+1:]:
                dx = b['cx'] - a['cx']
                if dx < C.CROSS_MIN_X_SPAN_RATIO * roi_w:
                    continue
                slope = (b['y'] - a['y']) / dx
                if abs(slope) > .20:
                    continue
                row = independent([c for c in ordered
                       if abs(c['y'] - a['y'] - slope*(c['cx']-a['cx'])) <= tolerance])
                if not enough(row):
                    continue
                span = row[-1]['cx'] - row[0]['cx']
                if span < C.CROSS_MIN_X_SPAN_RATIO * roi_w:
                    continue
                if len(row) > len(best) or (len(row) == len(best) and span > best_span):
                    best, best_span = row, span
        return best

    def _recover_shadow_row(self, roi, candidates, y0, frame_h):
        """在至少两条清楚白条旁寻找较暗的白条，返回符合排列要求的一排。

        先确认两条亮白条位于不同位置，再降低亮度要求；恢复出的白条必须
        仍与原来的两条对应，且不会把同一白块的内外轮廓重复计算。"""
        roi_h, roi_w = roi.shape[:2]
        tolerance = C.CROSS_MAX_Y_SPAN_RATIO * roi_h / 2.0
        seeds = []
        for c in sorted(candidates, key=lambda c: c['w'] * c['h'], reverse=True):
            if (y0 + c['y']) / float(frame_h) < getattr(
                C, 'CROSS_SHADOW_MIN_TOP', 0.50
            ):
                continue
            # 同一白块可能有内外两条轮廓；两条亮白条必须分处不同的横向位置。
            if any(
                c['x'] < s['x'] + s['w'] and s['x'] < c['x'] + c['w'] for s in seeds
            ):
                continue
            seeds.append(c)
        seeds = [
            c
            for c in seeds
            if any(s is not c and abs(c['y'] - s['y']) <= tolerance for s in seeds)
        ]
        if len(seeds) < 2:
            return []
        mask = self._build_binary(roi, v_min=min(C.CROSS_V_MIN, 120))
        recovered = []
        for cnt in cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]:
            _, y, _, _ = cv2.boundingRect(cnt)
            if not any(abs(y - s['y']) <= tolerance for s in seeds):
                continue
            candidate = self._far_candidate(
                cnt,
                allow_square=(y0 + y) / float(frame_h)
                >= getattr(C, 'CROSS_CLOSE_SHAPE_TOP', 0.68),
            )
            if candidate is not None:
                recovered.append(candidate)
        # 降低亮度要求后，同一白块可能出现多层轮廓；同一横向位置只算一条。
        distinct = []
        for c in sorted(recovered, key=lambda c: c['w'] * c['h'], reverse=True):
            if any(
                c['x'] < s['x'] + s['w'] and s['x'] < c['x'] + c['w'] for s in distinct
            ):
                continue
            distinct.append(c)
        row = self._choose_far_row(distinct, roi_w, roi_h)
        # 恢复结果必须仍包含原来的亮白条，不能用另外两个亮块代替。
        matched = set()
        for seed in seeds:
            for i, c in enumerate(row):
                if i in matched:
                    continue
                overlap_w = max(
                    0,
                    min(seed['x'] + seed['w'], c['x'] + c['w'])
                    - max(seed['x'], c['x']),
                )
                overlap_h = max(
                    0,
                    min(seed['y'] + seed['h'], c['y'] + c['h'])
                    - max(seed['y'], c['y']),
                )
                if (
                    overlap_w * overlap_h >= 0.7 * seed['w'] * seed['h']
                    and abs(c['y'] - seed['y']) <= tolerance
                ):
                    matched.add(i)
                    break
        return row if len(matched) >= 2 else []

    # ========================================================
    # 寻找近处的大白块
    # ========================================================

    @staticmethod
    def _near_candidate(cnt, x_offset, y_offset):
        """检查近处白块的大小和完整程度，并把合格白块的位置换回整个检测区域。"""
        area = cv2.contourArea(cnt)

        if area < C.CROSS_NEAR_MIN_AREA:
            return None

        x, y, w, h = cv2.boundingRect(cnt)

        if w < C.CROSS_NEAR_MIN_W or h < C.CROSS_NEAR_MIN_H:
            return None

        fill = area / max(float(w * h), 1.0)

        if fill < C.CROSS_NEAR_MIN_FILL:
            return None

        # 换回整个斑马线检测区域中的位置，方便统一画框。
        return {
            "x": x + x_offset,
            "y": y + y_offset,
            "w": w,
            "h": h,
            "cx": (x + x_offset + w / 2.0),
            "cy": (y + y_offset + h / 2.0),
            "bottom": (y + y_offset + h),
            "fill": fill,
        }

    @staticmethod
    def _choose_near(near_candidates, roi_w):
        """优先从较低、较大的近处白块中选出横向跨度足够的一组。"""
        if len(near_candidates) < C.CROSS_NEAR_MIN_RECTS:
            return []

        # 优先检查画面中较低、较大的白块。
        cand = sorted(
            near_candidates, key=lambda c: (c["bottom"], c["w"] * c["h"]), reverse=True
        )

        for i in range(len(cand)):
            for j in range(i + C.CROSS_NEAR_MIN_RECTS, len(cand) + 1):
                g = cand[i:j]

                xs = [c["cx"] for c in g]

                x_span = max(xs) - min(xs)

                if x_span >= (C.CROSS_NEAR_MIN_X_SPAN_RATIO * roi_w):
                    return g

        return []

    # ========================================================
    # 同时检测完整白条和近处白块
    # ========================================================

    def _detect(self, frame):
        """从一张原图同时寻找完整白条和近处白块，返回位置、候选列表和黑白图。"""
        h, w = frame.shape[:2]

        x0 = int(w * C.CROSS_ROI_X0)
        x1 = int(w * C.CROSS_ROI_X1)
        y0 = int(h * C.CROSS_ROI_Y0)
        y1 = int(h * C.CROSS_ROI_Y1)

        roi = frame[y0:y1, x0:x1]

        roi_h, roi_w = roi.shape[:2]

        binary = self._build_binary(roi)

        # 完整白条。

        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        far_candidates = []

        for cnt in contours:
            # 截取区域时，远处灯光可能被切成矩形；只有上沿完整可见才可用于位置判断。
            if (
                getattr(C, "CROSS_REJECT_TOP_CLIPPED", False)
                and cv2.boundingRect(cnt)[1] <= 1
            ):
                continue
            c = self._far_candidate(
                cnt,
                allow_square=(
                    getattr(C, "CROSS_USE_STRIPE_TOP", False)
                    and (y0 + cv2.boundingRect(cnt)[1]) / float(h)
                    >= getattr(C, "CROSS_CLOSE_SHAPE_TOP", 0.68)
                ),
            )

            if c is not None:
                far_candidates.append(c)

        far_valid = self._choose_far_row(far_candidates, roi_w, roi_h)

        if (
            not far_valid
            and getattr(C, 'CROSS_SHADOW_ROW_RECOVERY', False)
            and getattr(C, 'CROSS_USE_STRIPE_TOP', False)
        ):
            far_valid = self._recover_shadow_row(roi, far_candidates, y0, h)

        far_detected = len(far_valid) >= C.CROSS_MIN_RECTS

        far_bottom = 0.0

        if far_detected:
            bottoms = [y0 + c["bottom"] for c in far_valid]

            far_bottom = statistics.median(bottoms) / float(h)

            far_bottom = max(0.0, min(1.0, far_bottom))

        # 近处白块。

        global_near_y0 = int(h * C.CROSS_NEAR_Y0)

        near_y0 = max(0, global_near_y0 - y0)

        near_x0 = int(roi_w * C.CROSS_NEAR_X0)

        near_x1 = int(roi_w * C.CROSS_NEAR_X1)

        near_mask = binary[near_y0:roi_h, near_x0:near_x1]

        near_candidates = []

        if near_mask.size > 0:
            near_contours, _ = cv2.findContours(
                near_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in near_contours:
                c = self._near_candidate(cnt, x_offset=near_x0, y_offset=near_y0)

                if c is not None:
                    near_candidates.append(c)

        near_valid = self._choose_near(near_candidates, roi_w)

        near_detected = len(near_valid) >= C.CROSS_NEAR_MIN_RECTS

        near_bottom = 0.0

        if near_detected:
            bottoms = [y0 + c["bottom"] for c in near_valid]

            near_bottom = statistics.median(bottoms) / float(h)

            near_bottom = max(0.0, min(1.0, near_bottom))

        detected = far_detected or near_detected

        bottom = max(far_bottom, near_bottom)

        return {
            "detected": detected,
            "far_detected": far_detected,
            "near_detected": near_detected,
            "bottom": bottom,
            "far_bottom": far_bottom,
            "near_bottom": near_bottom,
            "far_candidates": far_candidates,
            "far_valid": far_valid,
            "near_candidates": near_candidates,
            "near_valid": near_valid,
            "binary": binary,
            "roi_box": (x0, y0, x1, y1),
        }

    # ========================================================
    # 连续确认白条位置
    # ========================================================

    def _track_near_bottom(self, near_detected, raw_bottom):
        """根据是否看到白条和当前测量值，返回连续确认后的图像位置。

        小幅变化逐渐跟随；大幅跳动必须连续出现才接受。没有看到白条时
        保留上次位置。原方法名保持不变，跟踪值也可以是配置指定的白条上沿。"""

        if not near_detected:
            self.previous_seen_position = None
            self.near_jump_candidate = None
            self.near_jump_count = 0
            return 0.0 if self.near_track is None else float(self.near_track)

        raw_bottom = float(raw_bottom)
        previous = self.previous_seen_position
        self.previous_seen_position = raw_bottom

        if self.near_track is None:
            self.near_track = raw_bottom
            self.near_jump_candidate = None
            self.near_jump_count = 0
            return float(self.near_track)

        delta = raw_bottom - self.near_track

        # A confirmed row moving smoothly towards the car must not wait until
        # its image position stays still for four frames. Two fresh, bounded
        # observations corroborate forward movement. Large isolated jumps still
        # use the original confirmation below; a missed observation breaks it.
        if (getattr(C, 'CROSS_USE_STRIPE_TOP', False)
                and self.state in (self.ARMED, self.APPROACH)
                and previous is not None and delta > 0
                and 0 <= raw_bottom - previous <= .10
                and abs(previous - self.near_track) <= .10):
            self.near_track = raw_bottom
            self.near_jump_candidate = None
            self.near_jump_count = 0
            return float(self.near_track)

        # 位置只小幅变化时，把新旧位置按比例合并，减少抖动。
        if abs(delta) <= C.CROSS_NEAR_MAX_JUMP:
            self.near_track = (
                1.0 - C.CROSS_NEAR_EMA_ALPHA
            ) * self.near_track + C.CROSS_NEAR_EMA_ALPHA * raw_bottom

            self.near_jump_candidate = None
            self.near_jump_count = 0

            return float(self.near_track)

        # 位置突然跳动时，等待连续几帧确认。
        if (
            self.near_jump_candidate is None
            or abs(raw_bottom - self.near_jump_candidate) > 0.03
        ):
            self.near_jump_candidate = raw_bottom
            self.near_jump_count = 1
        else:
            self.near_jump_count += 1

        if self.near_jump_count >= C.CROSS_NEAR_JUMP_CONFIRM_FRAMES:
            self.near_track = float(self.near_jump_candidate)

            self.near_jump_candidate = None
            self.near_jump_count = 0

        return float(self.near_track)

    # ========================================================
    # 逐步确认斑马线、接近位置和停车条件
    # ========================================================

    def update(self, frame):
        """处理新画面，更新确认次数和接近状态，返回本帧是否应当停车。"""
        det = self._detect(frame)

        far_detected = det["far_detected"]

        near_detected = det["near_detected"]

        near_bottom = det["near_bottom"]

        use_top = getattr(C, "CROSS_USE_STRIPE_TOP", False)
        seen = far_detected if use_top else near_detected
        rects = det["far_valid"]
        top = (
            (
                statistics.median(det["roi_box"][1] + r["y"] for r in rects)
                / float(frame.shape[0])
            )
            if rects
            else 0.0
        )
        raw = top if use_top else near_bottom
        # 寻找或刚确认阶段的旧位置不能永久保留。
        # 已进入接近阶段时保留位置，以便持续丢失后按规则停车。
        if self.state in (self.SEARCH, self.ARMED):
            self.idle_miss_streak = 0 if seen else self.idle_miss_streak + 1
            expire = getattr(C, 'CROSS_IDLE_TRACK_EXPIRE_FRAMES', 10)
            if expire > 0 and self.idle_miss_streak >= expire:
                self.near_track = None
                self.near_jump_candidate = None
                self.near_jump_count = 0
                self.previous_seen_position = None
                self.arm_hist.clear()
                self.approach_hist.clear()
                self.brake_hist.clear()
                self.close_hist.clear()
                self.state = self.SEARCH
        else:
            self.idle_miss_streak = 0
        near_tracked = self._track_near_bottom(seen, raw)
        det["near_bottom_tracked"] = 0.0 if use_top else near_tracked
        det["distance_metric"] = "stripe_top" if use_top else "near_bottom"
        det["distance_raw"] = raw
        det["distance_tracked"] = near_tracked
        approach_limit = (
            C.CROSS_APPROACH_STRIPE_TOP if use_top else C.CROSS_APPROACH_NEAR_BOTTOM
        )
        brake_limit = C.CROSS_BRAKE_STRIPE_TOP if use_top else C.CROSS_BRAKE_NEAR_BOTTOM
        blind_limit = brake_limit if use_top else C.CROSS_BLINDZONE_ARM_NEAR_BOTTOM

        brake_requested = False
        trigger_reason = ""

        if self.state == self.DONE:
            return self._result(det, brake_requested=False, trigger_reason="")

        # ----------------------------------------------------
        # 从寻找进入确认阶段：先连续看到横向排列的白条。
        # ----------------------------------------------------

        self.arm_hist.append(1 if far_detected else 0)

        if (
            self.state == self.SEARCH
            and len(self.arm_hist) == C.CROSS_ARM_HISTORY
            and sum(self.arm_hist) >= C.CROSS_ARM_HITS
        ):
            self.state = self.ARMED
            # 保留寻找阶段的有效接近观察，避免晚识别时重复确认而错过停车。
            # 超时回到寻找和任务完成时，仍按原规则清空历史。
            self.miss_streak = 0

        # ----------------------------------------------------
        # 从确认进入接近阶段：使用连续确认的位置，不能只凭一帧近处白块。
        # ----------------------------------------------------

        # 启用极早停车时，仍需连续确认完整白条，车身附近的白色噪点不能触发。
        if getattr(C, "CROSS_EARLY_STOP_ON_ARM", False) and self.state in (
            self.ARMED,
            self.APPROACH,
        ):
            return self._result(
                det, brake_requested=True, trigger_reason="EARLY_FAR_CONFIRMED"
            )

        # 同时积累近处白条的确认次数；启用此配置时，较晚看见白条也能及时停车。
        close_hit = bool(
            seen
            and far_detected
            and rects
            and raw >= brake_limit
            and near_tracked >= brake_limit
        )
        self.close_hist.append(1 if close_hit else 0)
        if (
            getattr(C, 'CROSS_CONFIRMED_CLOSE_BRAKE', False)
            and use_top
            and self.state in (self.ARMED, self.APPROACH)
            and close_hit
            and len(self.close_hist) == 3
            and sum(self.close_hist) >= 2
        ):
            self.state = self.APPROACH
            return self._result(
                det, brake_requested=True, trigger_reason='TRACKED_STRIPE_TOP'
            )

        approach_hit = (seen and near_tracked >= approach_limit
                        and (not use_top or raw >= approach_limit))

        self.approach_hist.append(1 if approach_hit else 0)

        if (
            self.state == self.ARMED
            and len(self.approach_hist) == C.CROSS_APPROACH_HISTORY
            and sum(self.approach_hist) >= C.CROSS_APPROACH_HITS
        ):
            self.state = self.APPROACH
            self.miss_streak = 0

        # ----------------------------------------------------
        # 接近后请求刹车：位置连续达到配置阈值，或符合条件后持续丢失目标。
        # ----------------------------------------------------

        if self.state == self.APPROACH:
            # A high retained filter value cannot turn a fresh far-away row into
            # a second close vote after an isolated false near observation.
            brake_hit = (seen and near_tracked >= brake_limit
                         and (not use_top or raw >= brake_limit))

            self.brake_hist.append(1 if brake_hit else 0)

            if (
                len(self.brake_hist) == C.CROSS_BRAKE_HISTORY
                and sum(self.brake_hist) >= C.CROSS_BRAKE_HITS
            ):
                brake_requested = True
                trigger_reason = (
                    "TRACKED_STRIPE_TOP" if use_top else "TRACKED_NEAR_BOTTOM"
                )

            if not brake_requested:
                if seen:
                    self.miss_streak = 0
                else:
                    if use_top or near_tracked >= blind_limit:
                        self.miss_streak += 1
                    else:
                        self.miss_streak = 0

                    if self.miss_streak >= C.CROSS_MISS_STREAK_TO_BRAKE:
                        brake_requested = True
                        trigger_reason = (
                            "STRIPE_TRACK_LOST"
                            if use_top
                            else "NEAR_BLINDZONE_FALLBACK"
                        )

        return self._result(
            det, brake_requested=brake_requested, trigger_reason=trigger_reason
        )

    def _result(self, det, brake_requested, trigger_reason):
        """把本帧检测数据与当前状态、停车原因合成统一的返回结果。"""
        return CrosswalkResult(
            state=self.state,
            distance_metric=det.get("distance_metric", "near_bottom"),
            distance_raw=det.get("distance_raw", 0.0),
            distance_tracked=det.get("distance_tracked", 0.0),
            detected=det["detected"],
            bottom=det["bottom"],
            far_detected=det["far_detected"],
            near_detected=det["near_detected"],
            far_bottom=det["far_bottom"],
            near_bottom=det["near_bottom"],
            near_bottom_tracked=det.get("near_bottom_tracked", 0.0),
            candidate_count=len(det["far_candidates"]),
            valid_count=len(det["far_valid"]),
            near_valid_count=len(det["near_valid"]),
            brake_requested=(brake_requested),
            trigger_reason=(trigger_reason),
            miss_streak=(self.miss_streak),
            binary=det["binary"],
            valid_rects=det["far_valid"],
            near_rects=det["near_valid"],
            roi_box=det["roi_box"],
        )

    def mark_done(self):
        """标记本次斑马线任务完成，并清空确认和位置记录以停止继续请求刹车。"""
        self.state = self.DONE
        self.arm_hist.clear()
        self.approach_hist.clear()
        self.brake_hist.clear()
        self.miss_streak = 0
        self.near_track = None
        self.near_jump_candidate = None
        self.near_jump_count = 0
        self.idle_miss_streak = 0
        self.close_hist.clear()
        self.previous_seen_position = None

    # ========================================================
    # 绘制检查画面
    # ========================================================

    @staticmethod
    def draw_debug(frame, result):
        """复制原图并画出检测区域、白条方框和位置数值，返回带标记的新图。"""
        out = frame.copy()

        x0, y0, x1, y1 = result.roi_box

        # 完整白条画绿框。
        for c in result.valid_rects:
            p1 = (x0 + c["x"], y0 + c["y"])

            p2 = (x0 + c["x"] + c["w"], y0 + c["y"] + c["h"])

            cv2.rectangle(out, p1, p2, (0, 165, 255), 1)

        # 近处白块画紫框。
        for c in result.near_rects:
            p1 = (x0 + c["x"], y0 + c["y"])

            p2 = (x0 + c["x"] + c["w"], y0 + c["y"] + c["h"])

            cv2.rectangle(out, p1, p2, (0, 165, 255), 1)

        if result.detected:
            yy = int(result.bottom * out.shape[0])

            cv2.circle(out, (out.shape[1]-6, yy), 3, (0, 165, 255), -1)

        cv2.putText(
            out,
            (
                f"cross={result.state} "
                f"F={int(result.far_detected)} "
                f"N={int(result.near_detected)} "
                f"Fb={result.far_bottom:.3f} "
                f"Nb={result.near_bottom:.3f} "
                f"Nt={result.near_bottom_tracked:.3f}"
            ),
            (5, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 255, 0),
            1,
        )

        cv2.putText(
            out,
            "%s raw=%.3f pos=%.3f"
            % (result.distance_metric, result.distance_raw, result.distance_tracked),
            (5, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 255, 255),
            1,
        )
        return out
