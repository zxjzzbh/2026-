#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conservative image observations for BypassTrialPolicy, Python 3.7+.

No camera acquisition, networking, GPIO or actuator access. The initial
vanishing-point baseline assumes a stopped car placed parallel to the bay and
an unchanged camera pose. It is a weak pixel reference, not calibrated yaw.
"""
from copy import deepcopy
import math

import cv2
import numpy as np

from parking_entry_preview import EntryPreview, _for_side, _x_at_y


def _finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


class BypassPerception:
    def __init__(self, parking_cfg=None, confirm_frames=3, front_x=.47,
                 front_half_width=.13):
        if not isinstance(confirm_frames, int) or confirm_frames < 3:
            raise ValueError('confirm_frames must be an integer >= 3')
        if not (0 < front_half_width < min(front_x, 1-front_x)):
            raise ValueError('Front safety window must be inside the image')
        self.preview_detector = EntryPreview(
            {} if parking_cfg is None else parking_cfg, confirm_frames)
        self.confirm_frames = confirm_frames
        self.front_x = float(front_x)
        self.front_half_width = float(front_half_width)
        self.selected_side = None
        self.baseline_side = None
        self.baseline_samples = []
        self.baseline = None
        self.last_entrance_y = None
        self.last_frame_id = None
        self.last_captured_at = None
        self.last_observation = None
        self.seen_frame_ids = set()
        self._baseline_sample_added = False

    @staticmethod
    def _empty(fid, captured_at):
        return dict(frame_id=fid, captured_at=captured_at, valid=False,
                    observation_ready=False, blocked='UNKNOWN', open_side=None,
                    target_x=None, heading_error=None, entrance_y=None,
                    clear_of_board=False, board_visible=False,
                    board_boxes=[], geometry=None, preview=None, reasons=[],
                    board_bbox_semantics='ABOVE_ENTRANCE_SILHOUETTE_PLUS_HORIZONTAL_MARGIN',
                    clearance_is_image_only=True,
                    heading_confidence='UNAVAILABLE', heading_baseline=None,
                    baseline_frames=0, vanishing_point=None,
                    assumptions=['Initial car is placed parallel to the bay',
                                 'Camera pan, tilt, zoom and mounting stay fixed'])

    @staticmethod
    def _vanishing_point(geometry, width, height):
        divider, outer = geometry.get('divider'), geometry.get('outer_boundary')
        if not divider or not outer:
            return None, 'Heading requires divider and outer boundary'
        if (divider.get('observed_length_px', 0) < .045*height
                or outer.get('observed_length_px', 0) < .08*height):
            return None, 'Visible heading line segment is too short'
        p, q = np.asarray(divider['center']), np.asarray(outer['center'])
        v, u = np.asarray(divider['direction']), np.asarray(outer['direction'])
        matrix = np.column_stack((v, -u))
        if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix)) < .15:
            return None, 'Heading-line intersection is ill-conditioned'
        point = p + np.linalg.solve(matrix, q-p)[0]*v
        entry_y = geometry['entrance']['center'][1]
        if (not np.isfinite(point).all()
                or not (-.10*width <= point[0] <= 1.10*width)
                or not (.05*height <= point[1] < entry_y-.02*height)):
            return None, 'Vanishing point lies outside the plausible visible region'
        return [float(point[0]/width), float(point[1]/height)], None

    def _full_boards(self, frame, geometry):
        """Upright silhouette above measured ground line, widened for clearance.

        Blue floor may touch the board. Remove only the measured entrance's
        lower half-plane before contour extraction. These are image-silhouette
        proxies with horizontal margin, not complete physical-board bounds.
        """
        height, width = frame.shape[:2]
        entrance, divider = geometry['entrance'], geometry['divider']
        evx, evy = entrance['direction']
        ex, ey = entrance['center']
        if abs(evx) < 1e-5:
            return [], 'UNKNOWN', ['Entrance direction is degenerate']
        cut_y = ey+(np.arange(width)-ex)*evy/evx-max(1., .005*height)
        below_ground = np.arange(height)[:, None] >= cut_y[None, :]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.preview_detector.blue_low,
                           self.preview_detector.blue_high)
        mask[below_ground] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                               np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                               np.ones((5, 5), np.uint8))
        mask[below_ground] = 0
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        boxes, sides, problems = [], set(), []
        horizontal_margin = max(3, int(math.ceil(.015*width)))
        for contour in contours:
            x, y, bw, bh = map(int, cv2.boundingRect(contour))
            area = cv2.contourArea(contour)
            if area < .004*width*height or bh < .10*height:
                continue
            evx, evy = entrance['direction']
            ex, ey = entrance['center']
            if abs(evx) < 1e-5:
                problems.append('Entrance direction is degenerate')
                continue
            entrance_at_box = ey+(x+bw/2.-ex)*evy/evx
            # A region entirely below the measured ground entrance is blue
            # floor. Filter whole components; do not truncate upright boxes.
            if y >= entrance_at_box-.035*height:
                continue
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            boxes.append([x-horizontal_margin, y, bw+2*horizontal_margin, bh])
            if (not .45 <= bh/float(max(1, bw)) <= 4.0
                    or area/max(1., bw*bh) < .65
                    or area/max(1., hull_area) < .85):
                problems.append('Substantial blue region has ambiguous board shape')
                continue
            if (x-horizontal_margin <= 0 or y <= 0
                    or x+bw+horizontal_margin >= width or y+bh >= height):
                problems.append('Board silhouette or its margin reaches the image border')
                continue
            vx, vy = entrance['direction']
            ex, ey = entrance['center']
            if abs(vx) < 1e-5:
                problems.append('Entrance direction is degenerate')
                continue
            ground_y = y+bh
            line_y = ey+(x+bw/2.-ex)*vy/vx
            split_x = _x_at_y(divider, ground_y)
            if (split_x is None or not math.isfinite(split_x)
                    or abs(ground_y-line_y) > .08*height):
                problems.append('Board ground contact does not match the entrance')
                continue
            if x+bw+horizontal_margin < split_x-.01*width:
                sides.add('LEFT')
            elif x-horizontal_margin > split_x+.01*width:
                sides.add('RIGHT')
            else:
                problems.append('Board overlaps the divider or its side is ambiguous')
        if len(sides) > 1:
            problems.append('Boards occupy both sides')
        blocked = 'BOTH' if len(sides) > 1 else (
            next(iter(sides)) if sides else 'UNKNOWN')
        return boxes, blocked, problems

    def _finish(self, result):
        if self.baseline is None and not self._baseline_sample_added:
            self.baseline_samples = []
        self.last_frame_id = result['frame_id']
        self.last_captured_at = result['captured_at']
        self.seen_frame_ids.add(result['frame_id'])
        self.last_observation = deepcopy(result)
        return result

    def update(self, frame, fid, captured_at, selected_side=None):
        result = self._empty(fid, captured_at)
        self._baseline_sample_added = False
        if (not _finite(captured_at) or isinstance(fid, bool)
                or not isinstance(fid, (int, str)) or fid == ''):
            result['reasons'].append('Invalid frame identity or capture timestamp')
            return result
        if fid == self.last_frame_id:
            if captured_at == self.last_captured_at:
                return deepcopy(self.last_observation)
            result['reasons'].append('Repeated frame changed its capture timestamp')
            return result
        if (fid in self.seen_frame_ids
                or (self.last_captured_at is not None
                    and captured_at <= self.last_captured_at)):
            result['reasons'].append('Frame replay or non-advancing capture time')
            return result
        if selected_side is not None:
            if (selected_side not in ('LEFT', 'RIGHT')
                    or (self.selected_side is not None
                        and selected_side != self.selected_side)
                    or self.baseline is None or selected_side != self.baseline_side):
                result['reasons'].append('Selected side conflicts with initial confirmation')
                return self._finish(result)
            self.selected_side = selected_side
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            result['reasons'].append('Missing or malformed BGR frame')
            return self._finish(result)
        try:
            return self._observe(frame, result)
        except (cv2.error, ValueError, KeyError, TypeError, IndexError,
                OverflowError, np.linalg.LinAlgError) as error:
            result['valid'] = False
            result['observation_ready'] = False
            result['clear_of_board'] = False
            result['reasons'].append('Perception error: '+str(error))
            return self._finish(result)

    def _observe(self, frame, result):
        height, width = frame.shape[:2]
        if self.selected_side is None:
            preview = self.preview_detector.update(frame)
            geometry = preview['geometry']
            side = preview.get('open_side')
        else:
            side = self.selected_side
            geometry = _for_side(frame, side)
            preview = dict(mode='LOCKED_SIDE_OBSERVATION', geometry=geometry,
                           open_side=side, observation_ready=False,
                           motion_ready=False, blocked='UNKNOWN', board_boxes=[],
                           stable_frames=0, reasons=[])
        result.update(geometry=geometry, preview=preview, open_side=side)
        if not geometry.get('valid') or side not in ('LEFT', 'RIGHT'):
            result['reasons'].extend(geometry.get('reasons', []))
            result['reasons'].append('Selected empty-side geometry is unavailable')
            if self.baseline is None:
                self.baseline_samples = []
            return self._finish(result)
        entry_y = float(geometry['entrance']['center'][1]/height)
        result['entrance_y'] = entry_y
        if (not math.isfinite(entry_y) or entry_y > .90
                or (self.last_entrance_y is not None
                    and abs(entry_y-self.last_entrance_y) > .06)):
            result['reasons'].append('Entrance jumped or reached the lower image limit')
            return self._finish(result)
        self.last_entrance_y = entry_y
        target = float(geometry['centerline'][-1][0]/width)
        if not math.isfinite(target) or not 0 <= target <= 1:
            result['reasons'].append('Image target is invalid')
            return self._finish(result)
        result['target_x'] = target
        boxes, blocked, problems = self._full_boards(frame, geometry)
        result.update(board_boxes=boxes, blocked=blocked, board_visible=bool(boxes))
        if self.selected_side is not None:
            preview.update(board_boxes=boxes, blocked=blocked)
        if problems or blocked == side or blocked == 'BOTH':
            result['reasons'].extend(problems)
            if blocked == side:
                result['reasons'].append('Board occupies the selected empty side')
            return self._finish(result)
        if self.selected_side is None and (
                blocked == 'UNKNOWN' or blocked != preview.get('blocked')):
            result['reasons'].append('Initial board silhouette and preview confirmation disagree')
            self.baseline_samples = []
            return self._finish(result)
        vp, problem = self._vanishing_point(geometry, width, height)
        result['vanishing_point'] = vp
        if problem:
            result['reasons'].append(problem)
            if self.baseline is None:
                self.baseline_samples = []
            return self._finish(result)
        if self.baseline is None:
            if self.baseline_side != side:
                self.baseline_side = side
                self.baseline_samples = []
            self.baseline_samples.append(vp)
            self._baseline_sample_added = True
            recent = np.asarray(self.baseline_samples[-self.confirm_frames:])
            if np.ptp(recent[:, 0]) > .025 or np.ptp(recent[:, 1]) > .05:
                self.baseline_samples = [vp]
            elif len(recent) >= self.confirm_frames:
                self.baseline = float(np.median(recent[:, 0]))
        elif self.baseline_side != side:
            result['reasons'].append('Empty side changed after heading baseline was fixed')
            return self._finish(result)
        result['baseline_frames'] = min(len(self.baseline_samples), self.confirm_frames)
        result['heading_baseline'] = self.baseline
        if self.baseline is None:
            result['reasons'].append('Waiting for at least three consistent initial VP frames')
            return self._finish(result)
        result['heading_error'] = float(vp[0]-self.baseline)
        result['heading_confidence'] = 'WEAK_UNCALIBRATED_INITIAL_REFERENCE'
        if self.selected_side is None and not preview.get('observation_ready'):
            result['reasons'].append('Waiting for EntryPreview initial confirmation')
            return self._finish(result)
        low = (self.front_x-self.front_half_width)*width
        high = (self.front_x+self.front_half_width)*width
        result['clear_of_board'] = bool(boxes) and all(
            x+bw < low or x > high for x, y, bw, bh in boxes)
        result['valid'] = True
        result['observation_ready'] = True
        if self.selected_side is not None:
            preview['observation_ready'] = True
        result['reasons'].append('Current selected-side image geometry is valid')
        result['reasons'].append('Heading uses a weak fixed-camera initial VP baseline')
        if not boxes:
            result['reasons'].append('Board lost: geometry continues; clear_of_board stays false')
        elif not result['clear_of_board']:
            result['reasons'].append('Board silhouette plus margin still overlaps the front safety window')
        else:
            result['reasons'].append('Current board silhouette plus margin lies outside the front safety window')
        result['reasons'].append('Board bounds use above-entrance pixels plus 1.5% image-width margin per side')
        result['reasons'].append('Image clearance does not verify rear-wheel passage or parking')
        return self._finish(result)
