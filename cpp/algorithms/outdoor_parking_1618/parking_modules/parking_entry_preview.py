"""Parking-entry observation using existing blocker contours and color geometry.

This module produces image references only. It has no actuator interface and
never declares a path safe for the car.
"""
from copy import deepcopy

import cv2
import numpy as np

from parking_entry_geometry import detect_entry_geometry


def _x_at_y(line, y):
    vx, vy = line['direction']
    if abs(vy) < 1e-5:
        return None
    x, y0 = line['center']
    return x + (y - y0) * vx / vy


def _for_side(frame, side):
    """Use the same color-geometry checks on either side of the divider."""
    if side == 'LEFT':
        result = detect_entry_geometry(frame, 'LEFT')
    else:
        result = detect_entry_geometry(cv2.flip(frame, 1), 'LEFT')
        width = frame.shape[1]
        def flip(point):
            return [width - 1 - point[0], point[1]]
        for key in ('entrance', 'divider', 'left_boundary'):
            line = result.get(key)
            if line:
                line['center'] = flip(line['center'])
                line['endpoints'] = [flip(p) for p in line['endpoints']]
                line['direction'][0] = -line['direction'][0]
                if 'support_x_range' in line:
                    left, right = line['support_x_range']
                    line['support_x_range'] = [width - 1 - right, width - 1 - left]
        for key in ('corridor', 'centerline'):
            if result.get(key):
                result[key] = [flip(p) for p in result[key]]
        for key in ('entrance_left_intersection', 'entrance_divider_intersection'):
            if result.get(key):
                result[key] = flip(result[key])
    result['outer_boundary'] = result.pop('left_boundary', None)
    if 'entrance_left_intersection' in result:
        result['entrance_outer_intersection'] = result.pop('entrance_left_intersection')
    result['open_side'] = side
    return result


class EntryPreview:
    def __init__(self, parking_cfg, confirm_frames=3):
        config = deepcopy(parking_cfg)
        # Observation profile for the current blue cloth: floor saturation is
        # lower than the upright board. Keep this separate from drive settings.
        config['blue_low'] = [90, 130, 45]
        config['blue_high'] = [130, 255, 255]
        self.blue_low = np.asarray(config['blue_low'], dtype=np.uint8)
        self.blue_high = np.asarray(config['blue_high'], dtype=np.uint8)
        self.confirm_frames = confirm_frames
        self.last_key = None
        self.hits = 0
        self.previous_points = None

    def _boards(self, frame, entrance):
        """Keep the original color/shape checks, allowing minor edge notches.

        An exact four-vertex test flickers with compression and reflections.
        High rectangle fill and convex-hull solidity tolerate those notches;
        ground contact and divider checks still run in update().
        """
        h, w = frame.shape[:2]
        y0 = int(.30 * h)
        cut_y = max(y0 + 1, min(h, int(entrance['center'][1])))
        hsv = cv2.cvtColor(frame[y0:cut_y], cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.blue_low, self.blue_high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        boards = []
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            if (area < .004 * w * h or bh < .10 * h or bh > .35 * h
                    or bw > .34 * w or not .55 <= bh / float(bw) <= 3.0
                    or area / max(1.0, bw * bh) < .80
                    or area / max(1.0, hull_area) < .90):
                continue
            boards.append((area, x + bw / 2.0, (x, y + y0, bw, bh)))
        return boards

    def update(self, frame):
        h, w = frame.shape[:2]
        geometry = _for_side(frame, 'LEFT')
        result = dict(mode='OBSERVE_ONLY', motion_ready=False,
                      blocked='UNKNOWN', open_side=None, board_boxes=[],
                      geometry=geometry, stable_frames=0,
                      observation_ready=False, reasons=[])
        entrance, divider = geometry.get('entrance'), geometry.get('divider')
        if entrance is None or divider is None:
            result['reasons'] = list(geometry['reasons'])
            self._reset()
            return result
        # Crop upright-board contours at the color-supported entrance.
        boards = self._boards(frame, entrance)
        sides = set()
        ambiguous = False
        for _, _, box in boards:
            x, y, bw, bh = box
            ground_y = y + bh
            vx, vy = entrance['direction']
            ex, ey = entrance['center']
            if abs(vx) < 1e-5:
                continue
            line_y = ey + (x + bw / 2.0 - ex) * vy / vx
            if abs(ground_y - line_y) > .07 * h:
                continue
            split_x = _x_at_y(divider, ground_y)
            if split_x is None:
                continue
            result['board_boxes'].append(list(box))
            if x + bw < split_x - .01 * w:
                sides.add('LEFT')
            elif x > split_x + .01 * w:
                sides.add('RIGHT')
            else:
                ambiguous = True
        if ambiguous or len(sides) != 1:
            result['blocked'] = 'BOTH' if len(sides) > 1 else 'UNKNOWN'
            result['reasons'] = ['Board side is missing or ambiguous']
            self._reset()
            return result
        blocked = next(iter(sides))
        side = 'LEFT' if blocked == 'RIGHT' else 'RIGHT'
        if side == 'RIGHT':
            geometry = _for_side(frame, side)
        result.update(blocked=blocked, open_side=side, geometry=geometry)
        if not geometry['valid']:
            result['reasons'] = list(geometry['reasons'])
            self._reset()
            return result
        points = np.asarray(geometry['centerline'], dtype=float) / [w, h]
        if not np.isfinite(points).all():
            result['geometry']['valid'] = False
            result['reasons'] = ['Non-finite image reference']
            self._reset()
            return result
        consistent = (self.last_key == side and self.previous_points is not None
                      and np.max(np.abs(points - self.previous_points)) <= .04)
        self.hits = self.hits + 1 if consistent else 1
        self.previous_points = points
        self.last_key = side
        result['stable_frames'] = self.hits
        result['observation_ready'] = self.hits >= self.confirm_frames
        result['reasons'] = list(geometry['reasons'])
        return result

    def _reset(self):
        self.hits = 0
        self.last_key = None
        self.previous_points = None


def draw_preview(frame, result, scale=3):
    out = frame.copy()
    geometry = result['geometry']
    if result['observation_ready'] and geometry.get('corridor'):
        tint = out.copy()
        cv2.fillPoly(tint, [np.rint(geometry['corridor']).astype(np.int32)], (60, 180, 60))
        out = cv2.addWeighted(tint, .25, out, .75, 0)
    for key, color in [('entrance', (0, 255, 255)), ('divider', (0, 140, 255)),
                       ('outer_boundary', (255, 255, 0))]:
        line = geometry.get(key)
        if line:
            a, b = [tuple(np.rint(p).astype(int)) for p in line['endpoints']]
            cv2.line(out, a, b, color, 1, cv2.LINE_AA)
    for x, y, width, height in result['board_boxes']:
        cv2.rectangle(out, (x, y), (x + width, y + height), (255, 0, 255), 1)
    if result['observation_ready'] and geometry.get('centerline'):
        a, b = [tuple(np.rint(p).astype(int)) for p in geometry['centerline']]
        cv2.line(out, a, b, (0, 255, 0), 2)
    out = cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    labels = ['OBSERVE ONLY - NO MOTION',
              'Board: %s | Open: %s | Stable: %d' % (
                  result['blocked'], result['open_side'], result['stable_frames']),
              'Yellow: entrance | Orange: divider | Cyan: outer edge',
              'Green: IMAGE REFERENCE ONLY' if result['observation_ready']
              else 'Geometry not confirmed - no center reference']
    for i, label in enumerate(labels):
        y = 24 + i * 24
        cv2.putText(out, label, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, label, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (255, 255, 255), 1, cv2.LINE_AA)
    return out
