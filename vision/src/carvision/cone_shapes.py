"""Conservative color/shape candidates for small upright red or blue cones.

The tapered silhouette needs enough pixels to be inspected. These candidates
are observation evidence, not calibrated positions or proof of clear ground.
"""

import cv2
import numpy as np

from .results import Detection


def cone_candidates(hsv):
    h, w = hsv.shape[:2]
    # Cone blue must be separated from cyan/green ground. Board detection keeps
    # its own broader color range, including nearby occluding panels.
    masks = (
        cv2.inRange(hsv, (100, 110, 50), (135, 255, 255)),
        cv2.bitwise_or(cv2.inRange(hsv, (0, 110, 50), (10, 255, 255)),
                      cv2.inRange(hsv, (170, 110, 50), (179, 255, 255))),
    )
    found = []
    for mask in masks:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1:
                continue
            # Tiny patches cannot support a reliable taper measurement.
            if bw < max(8, w * .02) or bh < max(12, h * .035):
                continue
            if not (.00015 <= area / (w * h) <= .025 and y + bh >= h * .4
                    and .9 <= bh / bw <= 3.5 and .35 <= area / (bw * bh) <= .8):
                continue
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            if hull_area <= 0 or area / hull_area < .82:
                continue
            silhouette = np.zeros((bh, bw), np.uint8)
            local_contour = contour - np.array([[[x, y]]], dtype=contour.dtype)
            cv2.drawContours(silhouette, [local_contour], -1, 1, -1)
            widths = np.count_nonzero(silhouette, axis=1)
            # Ground reflections can join the base and extend a narrow tail.
            # Measure taper relative to the broad base, not that unstable tail.
            # A circle's widest band is midway down, unlike an upright cone.
            broad_rows = np.flatnonzero(widths >= .9 * widths.max())
            base_row = int(np.median(broad_rows))
            if base_row < bh * .55:
                continue
            body_height = base_row + 1
            upper = float(np.mean(widths[round(body_height * .1):round(body_height * .3)]))
            lower = float(np.mean(widths[round(body_height * .8):body_height]))
            # Rounded or slightly blurred tips need not narrow to a point.
            if lower < upper * 1.5:
                continue
            found.append(Detection('cone', .6, [x, y, x + bw, y + bh]))
    return found
