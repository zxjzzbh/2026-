"""Conservative white-line baseline; missing boundaries produce invalid results.

This is an image-space prototype, not a calibrated vehicle path planner.
"""

import cv2
import numpy as np

from .config import LaneConfig
from .results import LaneResult


def _runs(row, min_pixels, max_pixels):
    padded = np.pad(row.astype(np.int8), (1, 1))
    changes = np.diff(padded)
    starts, ends = np.where(changes == 1)[0], np.where(changes == -1)[0]
    return [(s + e - 1) / 2 for s, e in zip(starts, ends) if min_pixels <= e - s <= max_pixels]


class LaneDetector:
    def __init__(self, config: LaneConfig):
        self.config = config

    def detect(self, image):
        c = self.config
        h, w = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, c.white_v_min), (179, c.white_s_max, 255))
        mask[:int(c.roi_top * h)] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        samples = []
        previous = None
        for y in np.linspace(c.sample_bottom * (h-1), c.sample_top * (h-1), c.sample_rows):
            yi = int(round(y))
            row = (mask[max(0, yi-2):min(h, yi+3)] > 0).mean(axis=0) >= 0.5
            xs = _runs(row, max(2, int(w * c.min_run_fraction)), int(w * c.max_run_fraction))
            center = w / 2 if previous is None else sum(previous) / 2
            lefts, rights = [x for x in xs if x < center], [x for x in xs if x > center]
            pairs = [(l, r) for l in lefts for r in rights
                     if c.min_width_fraction * w < r-l < c.max_width_fraction * w]
            if not pairs:
                continue
            if previous is None:
                pair = min(pairs, key=lambda p: p[1]-p[0])
            else:
                pair = min(pairs, key=lambda p: abs(p[0]-previous[0])+abs(p[1]-previous[1]))
                if max(abs(pair[0]-previous[0]), abs(pair[1]-previous[1])) > w * 0.18:
                    continue
            samples.append((y, *pair))
            previous = pair
        if len(samples) < c.min_pairs:
            return LaneResult(reason="insufficient_boundary_pairs"), mask
        ys, left, right = np.asarray(samples, dtype=float).T
        coverage = np.ptp(ys) / ((c.sample_bottom-c.sample_top) * (h-1))
        if coverage < c.min_coverage:
            return LaneResult(reason="insufficient_vertical_coverage"), mask
        yn = ys / (h-1)
        lp, rp = np.polyfit(yn, left, 2), np.polyfit(yn, right, 2)
        residual = max(np.sqrt(np.mean((np.polyval(lp, yn)-left)**2)),
                       np.sqrt(np.mean((np.polyval(rp, yn)-right)**2))) / w
        if residual > c.max_fit_residual_fraction:
            return LaneResult(reason="inconsistent_boundaries"), mask
        if not min(yn) <= c.lookahead_y <= max(yn):
            return LaneResult(reason="lookahead_not_observed"), mask
        out_y = np.linspace(min(yn), max(yn), 20)
        lx, rx = np.polyval(lp, out_y), np.polyval(rp, out_y)
        if np.any(lx < 0) or np.any(rx >= w) or np.any(rx-lx < c.min_width_fraction*w):
            return LaneResult(reason="invalid_fitted_geometry"), mask
        tx = float((np.polyval(lp, c.lookahead_y)+np.polyval(rp, c.lookahead_y))/2)
        center = (lx+rx)/2
        quality = float(min(1, len(samples)/c.sample_rows) * max(0, 1-residual/c.max_fit_residual_fraction))
        return LaneResult(True, "both_boundaries", quality, (tx-w/2)/(w/2),
                          np.column_stack((lx, out_y*(h-1))).tolist(),
                          np.column_stack((rx, out_y*(h-1))).tolist(),
                          np.column_stack((center, out_y*(h-1))).tolist(),
                          [tx, c.lookahead_y*(h-1)]), mask
