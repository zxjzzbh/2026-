"""Conservative image cues for a horizontal red/amber/green fixture.

Two colored circular lenses anchor the layout. A washed-out third lens still
needs its own colored rim; a white patch at an expected position is not a lamp.
These experimental cues do not authorize vehicle motion.
"""

from itertools import combinations

import cv2
import numpy as np


def _color(hue, index):
    # Warm flare shifts dark red toward orange and green toward yellow-green.
    # These ranges intentionally overlap; the three-lens layout, colored rims
    # and emission contrast must agree before assigning any active signal.
    if index == 0:
        return (hue <= 15) | (hue >= 170)
    if index == 1:
        # Unlit amber plastic can look orange, especially under a red lamp.
        return (hue >= 8) & (hue <= 38)
    return (hue >= 32) & (hue <= 90)


def _anchors(hsv, index):
    hue, sat, val = cv2.split(hsv)
    mask = (_color(hue, index) & (sat >= 90) & (val >= 50)).astype('uint8') * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    anchors = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if (min(w, h) >= 6 and max(w, h) <= min(hsv.shape[0] * .9, hsv.shape[1] / 3)
                and .65 <= w / h <= 1.55 and area >= .5 * w * h):
            anchors.append((x + (w-1)/2, y + (h-1)/2, (w+h)/4, area))
    return sorted(anchors, key=lambda a: a[3], reverse=True)[:12]


def _lens(hsv, center, radius, index):
    cx, cy = center
    pad = radius * 1.7
    x1, y1 = int(np.floor(cx-pad)), int(np.floor(cy-pad))
    x2, y2 = int(np.ceil(cx+pad+1)), int(np.ceil(cy+pad+1))
    if x1 < 0 or y1 < 0 or x2 > hsv.shape[1] or y2 > hsv.shape[0]:
        return None
    roi = hsv[y1:y2, x1:x2]
    yy, xx = np.ogrid[y1:y2, x1:x2]
    distance2 = (xx-cx)**2 + (yy-cy)**2
    core = distance2 <= (radius*.7)**2
    ring = (distance2 >= (radius*.7)**2) & (distance2 <= pad**2)
    hue, sat, val = cv2.split(roi)
    colored = _color(hue, index) & (sat >= 90) & (val >= 50)
    rim = _color(hue, index) & (sat >= 25) & (val >= 70)
    white = (sat <= 35) & (val >= 230)
    quadrants = [ring & (xx < cx if left else xx >= cx) & (yy < cy if top else yy >= cy)
                 for left, top in ((True, True), (True, False), (False, True), (False, False))]
    white_core = float(white[core].mean())
    quadrant_support = [float(rim[q].mean()) for q in quadrants]
    distributed = sum(value >= .08 for value in quadrant_support) >= 3
    # Resampling can shift a thin rim across a quadrant boundary. Permit weaker
    # support in its third quadrant only when the total colored rim is strong;
    # a half-ring with no third direction remains insufficient for an edge lamp.
    distributed = distributed or (rim[ring].mean() >= .25
                                  and sum(value >= .04 for value in quadrant_support) >= 3)
    # A blooming center lamp can merge into the bright housing above it. When
    # both neighboring lenses support its position, accept strong rim color on
    # BOTH lateral sides. An edge lamp still needs the stricter surrounding rim.
    bracketed_rim = (index == 1 and rim[ring].mean() >= .18
                     and np.mean(quadrant_support[:2]) >= .10
                     and np.mean(quadrant_support[2:]) >= .10)
    # Require distributed color around the white core, not a neighboring sign.
    washed_out = (white_core >= .6 and rim[ring].mean() >= .10
                  and (distributed or bracketed_rim))
    if colored[core].mean() < .18 and not washed_out:
        return None
    return {'values': val[core], 'colored': colored[core], 'washed_out': washed_out}


def horizontal_lamp_groups(hsv):
    """Return geometrically supported groups without assuming which lamp is on."""
    anchors = [_anchors(hsv, i) for i in range(3)]
    groups = []
    for i, j in combinations(range(3), 2):
        for first in anchors[i]:
            for second in anchors[j]:
                dx, dy = (second[0]-first[0])/(j-i), (second[1]-first[1])/(j-i)
                radius = (first[2]+second[2])/2
                if (max(first[2], second[2])/min(first[2], second[2]) > 1.7
                        or not 1.8*radius <= dx <= 6*radius or abs(dy) > radius*.8):
                    continue
                centers = [(first[0]+(k-i)*dx, first[1]+(k-i)*dy) for k in range(3)]
                if any(max(np.hypot(x-u, y-v) for (x, y), (u, v) in zip(centers, g['centers']))
                       < radius*.6 for g in groups):
                    continue
                lenses = [_lens(hsv, center, radius, k) for k, center in enumerate(centers)]
                if any(lens is None for lens in lenses):
                    continue
                pad = radius * 1.7
                bbox = [int(np.floor(min(x for x, _ in centers)-pad)),
                        int(np.floor(min(y for _, y in centers)-pad)),
                        int(np.ceil(max(x for x, _ in centers)+pad+1)),
                        int(np.ceil(max(y for _, y in centers)+pad+1))]
                groups.append({'centers': centers, 'radius': radius, 'bbox': bbox, 'lenses': lenses})
    return groups


def group_light_state(group, config):
    """Compare lens interiors, excluding bright housing and background pixels."""
    lenses = group['lenses']
    brightness = [float(np.median(lens['values'])) for lens in lenses]
    active = []
    for i, lens in enumerate(lenses):
        values = lens['values']
        # Ambient reflections on unlit colored plastic are not enough. Keep a
        # strong emission threshold until real dim-lamp footage validates more.
        bright = values >= max(230, config.min_value)
        emission = (bright & (lens['colored'] | lens['washed_out'])).mean()
        if emission >= max(.25, config.min_bright_fraction) and brightness[i] > max(
                brightness[j] for j in range(3) if j != i) + 25:
            active.append(i)
    return ('red', 'yellow', 'green')[active[0]] if len(active) == 1 else 'unknown'
