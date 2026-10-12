"""Inspectable paper-row candidates. Pixels are never silently converted to metres.

This detector is shared by the existing preview and the crosswalk lab. Scores
are heuristic; it is not a field-verified autonomous measurement source.
"""
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

from .temporal import TimedConfirmation


@dataclass
class CrosswalkVisionConfig:
    roi_top: float = .4
    white_s_max: int = 100
    white_v_min: int = 170
    min_stripes: int = 3
    min_area_fraction: float = .0004
    max_area_fraction: float = .06
    min_row_width_fraction: float = .20
    confirm_ms: float = 200
    max_gap_ms: float = 250
    min_clipped_height_fraction: float = .06

    def validate(self):
        for key, value in asdict(self).items():
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"{key} must be finite numeric")
        if not 0 <= self.roi_top < .9:
            raise ValueError("roi_top must be in [0, .9)")
        if any(type(v) is not int or not 0 <= v <= 255 for v in (self.white_s_max, self.white_v_min)):
            raise ValueError("HSV S/V limits must be integer 0..255")
        if type(self.min_stripes) is not int or not 3 <= self.min_stripes <= 20:
            raise ValueError("min_stripes must be integer 3..20")
        if not 0 < self.min_area_fraction < self.max_area_fraction < 1:
            raise ValueError("invalid contour area limits")
        if not 0 < self.min_row_width_fraction < 1 or self.confirm_ms <= 0 or self.max_gap_ms <= 0:
            raise ValueError("invalid row width or confirmation timing")
        if not .04 <= self.min_clipped_height_fraction <= .3:
            raise ValueError("invalid clipped stripe height")
        return self


def load_crosswalk_config(path=None):
    data = {} if path is None else json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("crosswalk vision config must be an object")
    return CrosswalkVisionConfig(**data).validate()


class CrosswalkDetector:
    status = "experimental_color_shape"

    def __init__(self, config=None):
        self.config = (config or CrosswalkVisionConfig()).validate()
        self.confirm = TimedConfirmation(self.config.confirm_ms, self.config.max_gap_ms)

    def detect(self, image, timestamp_ms=None):
        h, w = image.shape[:2]
        c = self.config
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, c.white_v_min), (179, c.white_s_max, 255))
        mask[:round(h * c.roi_top)] = 0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        patches = []
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            vertices = len(cv2.approxPolyDP(contour, .04 * cv2.arcLength(contour, True), True))
            a, b = cv2.minAreaRect(contour)[1]
            clipped_bottom = y + bh >= h - 1
            if (bw >= 8 and bh >= max(6, h * .02) and .3 <= bw / bh <= 6
                    and c.min_area_fraction <= area / (w * h) <= c.max_area_fraction
                    and area >= .30 * bw * bh and a * b > 0 and area >= .5 * a * b
                    and 4 <= vertices <= 6 and y > h * c.roi_top
                    and (not clipped_bottom or bh >= c.min_clipped_height_fraction * h)):
                patches.append((x, y, bw, bh))
        rows = []
        for base in patches:
            cy = base[1] + base[3] / 2
            row = sorted([p for p in patches
                          if abs(p[1] + p[3] / 2 - cy) < max(base[3], p[3]) * .6
                          and max(base[3], p[3]) / min(base[3], p[3]) < 3], key=lambda p: p[0])
            if len(row) < c.min_stripes:
                continue
            spacing = [(r[0] + r[2] / 2) - (l[0] + l[2] / 2) for l, r in zip(row, row[1:])]
            if not all(.6 * (l[2] + r[2]) / 2 <= gap <= 4 * (l[2] + r[2]) / 2
                       for l, r, gap in zip(row, row[1:], spacing)):
                continue
            x1, y1 = min(p[0] for p in row), min(p[1] for p in row)
            x2, y2 = max(p[0] + p[2] for p in row), max(p[1] + p[3] for p in row)
            if x2 - x1 > w * c.min_row_width_fraction:
                rows.append(([x1, y1, x2, y2], row))
        # Clipped, substantial stripes can support presence, not a near-edge
        # distance. Tiny tips remain rejected by minimum clipped height.
        selected = max(rows, key=lambda item: item[0][3]) if rows else None
        candidate = selected is not None
        clipped = candidate and any(p[1] + p[3] >= h - 1 for p in selected[1])
        presence = self.confirm.update("present" if candidate else "absent", timestamp_ms)
        return {"candidate": candidate, "presence": presence,
                "detector_status": self.status, "score": .6 if candidate else 0,
                "bbox_xyxy": selected[0] if candidate else None,
                "stripes_xywh": [list(p) for p in selected[1]] if candidate else [],
                "bottom_clipped": bool(clipped),
                "far_edge_y_normalized": float(np.median([p[1] for p in selected[1]])) / (h - 1) if candidate else None,
                "near_edge_y_normalized": (selected[0][3] - 1) / (h - 1) if candidate and not clipped else None,
                "distance_m": None, "metric_valid": False,
                "reason": ("clipped_paper_row_presence_only" if clipped else "paper_row_candidate")
                          if candidate else "no_paper_row"}, mask
