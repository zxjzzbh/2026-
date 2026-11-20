#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Track a previously confirmed upright blue board through a side exit.

Pure image adapter, Python 3.7+. No hardware access. Disappearance is accepted
only after an observed edge trajectory; image exit is not rear-wheel clearance.
"""
from copy import deepcopy
import math

import cv2
import numpy as np

from parking_bypass_perception import BypassPerception
from parking_entry_preview import _for_side, _x_at_y
from parking_corridor_tracker import ParkingCorridorTracker


class SideExitPerception(BypassPerception):
    def __init__(self, parking_cfg=None, confirm_frames=3, front_x=.47,
                 front_half_width=.13):
        super().__init__(parking_cfg, confirm_frames, front_x, front_half_width)
        self._track_box = None
        self._track_height = None
        self._trusted_geometry = None
        self._last_target = None
        self._edge_frames = 0
        self._missing_frames = 0
        self._exit_armed = False
        self._exit_confirmed = False
        self._last_edge_near = None
        self._exit_last_fid = None
        self._exit_last_ts = None
        self._exit_cache = None
        self._exit_seen = set()
        self._initial_side = None
        self._initial_hits = 0
        self.corridor_tracker = ParkingCorridorTracker()

    def update(self, frame, fid, ts, selected_side=None):
        obs = self._update_board(frame, fid, ts, selected_side)
        # Track the longitudinal tapes before the board departs. Entrance
        # geometry seeds their identities; later tracking uses fresh pixels.
        obs['lane_guide'] = self.corridor_tracker.update(
            frame, fid, ts, obs.get('open_side'),
            obs.get('geometry') if obs.get('valid') else None,
            self.baseline)
        return obs

    def _initial_update(self, frame, fid, ts):
        """Confirm geometry and the visible upper board, allowing a far-edge crop."""
        obs = self._empty(fid, ts)
        obs.update(side_exit_confirmed=False, exit_edge_frames=0,
                   exit_missing_frames=0, exit_armed=False, front_clear=False,
                   tracking_valid=False, geometry_valid=False,
                   board_conflict=False, target_is_cached=False)
        h, w = frame.shape[:2]
        geometry = _for_side(frame, 'LEFT')
        obs['geometry'] = geometry
        entrance, divider = geometry.get('entrance'), geometry.get('divider')
        if not entrance or not divider:
            self._initial_hits = 0
            obs['reasons'] = ['Initial entrance/divider unavailable']
            return self._remember(obs, fid, ts)
        ey = entrance['center'][1]
        if not 0 < ey/h <= .90:
            self._initial_hits = 0
            obs['reasons'] = ['Initial entrance is outside the supported view']
            return self._remember(obs, fid, ts)
        split = _x_at_y(divider, ey)
        y0, y1 = int(.30*h), min(h, int(ey-.08*h))
        if split is None or y1 <= y0:
            obs['reasons'] = ['Initial upper-board search is unavailable']
            return self._remember(obs, fid, ts)
        hsv = cv2.cvtColor(frame[y0:y1], cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.preview_detector.blue_low,
                           self.preview_detector.blue_high)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        boxes, sides = [], set()
        ambiguous = False
        for x, y, bw, bh, area in stats[1:]:
            if area < .003*w*h or bh < .08*h or area/float(max(1,bw*bh)) < .65:
                continue
            if self._is_qr_placard(hsv[y:y+bh, x:x+bw]):
                continue
            box = [int(x), int(y+y0), int(bw), int(bh)]
            boxes.append(box)
            if x+bw < split-.01*w:
                sides.add('LEFT')
            elif x > split+.01*w:
                sides.add('RIGHT')
            else:
                ambiguous = True
        if ambiguous or len(sides) != 1:
            self._initial_hits = 0
            obs.update(blocked='BOTH' if len(sides)>1 else 'UNKNOWN', board_boxes=boxes,
                       board_conflict=bool(ambiguous or len(sides)>1))
            obs['reasons'] = ['Initial upper-board side is missing or ambiguous']
            return self._remember(obs, fid, ts)
        blocked = next(iter(sides))
        side = 'LEFT' if blocked == 'RIGHT' else 'RIGHT'
        if side == 'RIGHT':
            geometry = _for_side(frame, side)
        obs.update(geometry=geometry, blocked=blocked, open_side=side,
                   board_visible=True, board_boxes=boxes,
                   geometry_valid=bool(geometry.get('valid')))
        if not geometry.get('valid'):
            self._initial_hits = 0
            obs['reasons'] = list(geometry.get('reasons', []))
            return self._remember(obs, fid, ts)
        vp, problem = self._vanishing_point(geometry,w,h)
        if problem:
            self._initial_hits = 0
            obs['reasons'] = [problem]
            return self._remember(obs, fid, ts)
        if side != self._initial_side:
            self._initial_hits = 0
            self.baseline_samples = []
            self.baseline = None
            self._initial_side = side
        self.baseline_samples.append(vp)
        self.baseline_samples = self.baseline_samples[-self.confirm_frames:]
        recent = np.asarray(self.baseline_samples)
        if np.ptp(recent[:,0])>.025 or np.ptp(recent[:,1])>.05:
            self._initial_hits = 1
            self.baseline_samples = [vp]
        else:
            self._initial_hits += 1
        ready = self._initial_hits >= self.confirm_frames and len(self.baseline_samples)>=self.confirm_frames
        if ready:
            if self.baseline is None:
                self.baseline = float(np.median(recent[:,0]))
            self.baseline_side = side
            self._trusted_geometry = deepcopy(geometry)
            self._last_target = float(geometry['centerline'][-1][0]/w)
            self._track_box = max(boxes,key=lambda b:b[2]*b[3])[:]
            self._track_height = max(float(self._track_box[3]),ey-self._track_box[1])
            self.last_entrance_y = float(ey/h)
        obs.update(valid=ready, observation_ready=ready, tracking_valid=ready,
                   target_x=float(geometry['centerline'][-1][0]/w),
                   entrance_y=float(ey/h), vanishing_point=vp,
                   heading_baseline=self.baseline, baseline_frames=len(self.baseline_samples),
                   heading_error=float(vp[0]-self.baseline) if self.baseline is not None else None,
                   heading_confidence='WEAK_UNCALIBRATED_INITIAL_REFERENCE')
        obs['preview'] = dict(mode='UPPER_BOARD_INITIAL_CONFIRMATION',geometry=geometry,
                              blocked=blocked,open_side=side,board_boxes=boxes,
                              observation_ready=ready,stable_frames=self._initial_hits)
        obs['reasons'] = ['Initial upper-board side and geometry confirmed' if ready
                          else 'Waiting for three consistent initial upper-board observations']
        return self._remember(obs,fid,ts)

    @staticmethod
    def _is_qr_placard(hsv):
        """Recognize the course's dark backing with an inset light square."""
        height, width = hsv.shape[:2]
        if min(height, width) < 10:
            return False
        sample = hsv[::2, ::2]
        if np.median(sample[:, :, 1]) > 140:
            return False
        background = float(np.median(sample[:, :, 2]))
        if background > 135:
            return False
        light = cv2.inRange(hsv, (0, 0, int(max(105, background + 45))), (179, 95, 255))
        contours = cv2.findContours(light, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if not (.12 * width <= w <= .55 * width and .10 * height <= h <= .55 * height):
                continue
            if not (.55 <= w / h <= 1.65 and cv2.contourArea(contour) >= .50 * w * h):
                continue
            if (.22 * width <= x + .5*w <= .78 * width
                    and .18 * height <= y + .5*h <= .82 * height
                    and 0 < x and x + w < width and 0 < y and y + h < height):
                return True
        return False

    def _track_upper_board(self, frame, geometry):
        """Search the last upright board's upper body, above the blue floor.

        The vertical crop follows the measured board top and size. Its bottom
        is capped by the last credible entrance plane. Whole upper-body blue
        components are examined across the image so an opposite board can
        still be reported. Cropping at the far side is intentionally allowed.
        """
        h, w = frame.shape[:2]
        x, y, bw, bh = self._track_box
        body_height = max(float(bh), float(self._track_height or bh))
        y0 = max(0, int(math.floor(y-.07*h)))
        y1 = min(h, int(math.ceil(y+min(.60*body_height, .27*h))))
        entrance = geometry.get('entrance') if geometry else None
        if entrance:
            y1 = min(y1, int(entrance['center'][1]-.02*h))
        if y1-y0 < .05*h:
            return None, [], False, 'Upper-board search band is unavailable'
        # Keep context below the tracking band to identify partially cropped placards.
        hsv = cv2.cvtColor(frame[y0:], cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv[:y1-y0], self.preview_detector.blue_low,
                           self.preview_detector.blue_high)
        # Preserve skinny edge remnants: no opening that could erase them.
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        candidates = []
        for i in range(1, count):
            bx, by, bwidth, bheight, area = map(int, stats[i])
            if area < 8 or bheight < max(5, int(.02*h)):
                continue
            fill = area/float(max(1, bwidth*bheight))
            if fill < .35:
                continue
            patch = hsv[by:by+bheight, bx:bx+bwidth]
            if self._is_qr_placard(patch):
                continue
            if by + bheight == y1-y0 and 10 <= bwidth and bheight < 1.6*bwidth:
                # Recover the same blue component below the cut before classifying it.
                extended = hsv[by:by+int(1.6*bwidth)+1, bx:bx+bwidth]
                blue = cv2.inRange(extended, self.preview_detector.blue_low,
                                   self.preview_detector.blue_high)
                _, extended_labels, components, _ = cv2.connectedComponentsWithStats(blue, 8)
                ys, xs = np.nonzero(labels[by:by+bheight, bx:bx+bwidth] == i)
                label = extended_labels[ys[0], xs[0]]
                cx, cy, cw, ch, _ = components[label]
                if self._is_qr_placard(extended[cy:cy+ch, cx:cx+cw]):
                    continue
            candidates.append([bx, by+y0, bwidth, bheight, area])
        expected = 'RIGHT' if self.selected_side == 'LEFT' else 'LEFT'
        previous_center = x+bw/2.
        possible = []
        for box in candidates:
            bx, by, bwidth, bheight, area = box
            gap = max(0., bx-(x+bw), x-(bx+bwidth))
            if gap <= .12*w and abs((by+bheight/2.)-(y+min(bh,body_height*.60)/2.)) < .18*h:
                possible.append((area-.2*abs(bx+bwidth/2.-previous_center), box))
        matched = max(possible, key=lambda item: item[0])[1] if possible else None
        conflicts = []
        divider = geometry.get('divider') if geometry else None
        split = None
        if divider and entrance:
            split = _x_at_y(divider, entrance['center'][1])
        if split is None or not math.isfinite(split):
            split = self.front_x*w
        for box in candidates:
            if box is matched:
                continue
            bx, by, bwidth, bheight, area = box
            if area < .0015*w*h or bheight < .06*h:
                continue
            on_opposite = ((expected == 'RIGHT' and bx+bwidth < split-.01*w)
                           or (expected == 'LEFT' and bx > split+.01*w))
            if on_opposite:
                conflicts.append(box[:4])
        if matched is None:
            # A substantial blue region near the expected side that cannot be
            # associated is ambiguous, not evidence of disappearance.
            nearby_unmatched = any(
                box[4] >= 8 and ((expected == 'RIGHT' and box[0]+box[2] > .60*w)
                                or (expected == 'LEFT' and box[0] < .34*w))
                for box in candidates)
            if nearby_unmatched:
                return None, conflicts, False, 'Blue edge region is present but cannot be associated'
        return matched, conflicts, True, 'Upper-board search completed'

    def _update_board(self, frame, fid, ts, selected_side=None):
        if fid == self._exit_last_fid and ts == self._exit_last_ts:
            return deepcopy(self._exit_cache)
        identity_ok = (isinstance(fid, (int, str)) and not isinstance(fid, bool)
                       and fid != '' and isinstance(ts, (int, float))
                       and not isinstance(ts, bool) and math.isfinite(ts)
                       and fid not in self._exit_seen
                       and (self._exit_last_ts is None or ts > self._exit_last_ts))
        if (identity_ok and self.selected_side is None and selected_side is None
                and frame is not None and frame.ndim == 3):
            return self._initial_update(frame, fid, ts)
        obs = super().update(frame, fid, ts, selected_side)
        obs.update(side_exit_confirmed=self._exit_confirmed,
                   exit_edge_frames=self._edge_frames,
                   exit_missing_frames=self._missing_frames,
                   exit_armed=self._exit_armed, front_clear=False,
                   tracking_valid=False, geometry_valid=False,
                   board_conflict=False, target_is_cached=False)
        if not identity_ok or frame is None:
            return obs
        geometry = obs.get('geometry')
        geometry_valid = bool(geometry and geometry.get('valid'))
        if any('Entrance jumped' in reason for reason in obs.get('reasons', [])):
            geometry_valid = False
        obs['geometry_valid'] = geometry_valid
        if obs.get('valid') and geometry_valid:
            self._trusted_geometry = deepcopy(geometry)
            self._last_target = obs.get('target_x')
        if self.selected_side is None:
            if obs.get('valid') and obs.get('board_boxes'):
                self._track_box = max(obs['board_boxes'], key=lambda b: b[2]*b[3])[:]
                self._track_height = float(self._track_box[3])
            obs['tracking_valid'] = bool(obs.get('valid'))
            return self._remember(obs, fid, ts)
        if selected_side is not None and selected_side != self.selected_side:
            return self._remember(obs, fid, ts)
        if self._track_box is None or self._trusted_geometry is None:
            obs['valid'] = obs['observation_ready'] = False
            obs['reasons'].append('No initial confirmed board track')
            return self._remember(obs, fid, ts)
        current_geometry = geometry if geometry_valid else self._trusted_geometry
        matched, conflicts, search_valid, reason = self._track_upper_board(frame, current_geometry)
        obs['base_reasons'] = obs.get('reasons', [])
        obs['reasons'] = [reason]
        obs['board_conflict'] = bool(conflicts)
        if conflicts:
            obs.update(valid=False, observation_ready=False, blocked='BOTH',
                       board_boxes=conflicts, board_visible=True)
            obs['reasons'].append('Opposite-side blue upright region detected')
            return self._remember(obs, fid, ts)
        if not search_valid:
            obs['valid'] = obs['observation_ready'] = False
            return self._remember(obs, fid, ts)
        h, w = frame.shape[:2]
        expected = 'RIGHT' if self.selected_side == 'LEFT' else 'LEFT'
        margin = max(3, int(math.ceil(.015*w)))
        front_low = (self.front_x-self.front_half_width)*w
        front_high = (self.front_x+self.front_half_width)*w
        if matched is not None:
            x, y, bw, bh, area = matched
            near = x-margin if expected == 'RIGHT' else x+bw+margin
            front_clear = near > front_high if expected == 'RIGHT' else near < front_low
            at_edge = (x+bw > .94*w if expected == 'RIGHT' else x < .06*w)
            progressing = (self._last_edge_near is None
                           or (near >= self._last_edge_near-.025*w if expected == 'RIGHT'
                               else near <= self._last_edge_near+.025*w))
            if front_clear and at_edge and progressing:
                self._edge_frames += 1
                self._last_edge_near = near
                if self._edge_frames >= 2:
                    self._exit_armed = True
            else:
                # A visible retreat from the edge cancels an uncompleted
                # departure; a later disappearance needs a new edge track.
                self._edge_frames = 0
                self._last_edge_near = None
                self._exit_armed = False
            self._missing_frames = 0
            self._track_box = [x, y, bw, bh]
            self._track_height = max(float(self._track_height), float(bh))
            obs.update(blocked=expected, board_visible=True,
                       board_boxes=[[x-margin, y, bw+2*margin, bh]],
                       front_clear=front_clear, clear_of_board=front_clear)
            if self._exit_confirmed and not front_clear:
                obs['board_conflict'] = True
                obs['reasons'].append('Tracked board re-entered the front window')
        else:
            if self._exit_armed:
                self._missing_frames += 1
                if self._missing_frames >= 3:
                    self._exit_confirmed = True
            else:
                self._edge_frames = self._missing_frames = 0
                self._last_edge_near = None
            obs.update(blocked='UNKNOWN', board_visible=False, board_boxes=[],
                       front_clear=bool(self._exit_armed or self._exit_confirmed),
                       clear_of_board=bool(self._exit_armed or self._exit_confirmed))
            obs['reasons'].append('No tracked upper-board pixels in the valid search region')
        tracking_valid = (matched is not None or self._exit_armed or self._exit_confirmed)
        tracking_valid = tracking_valid and not obs['board_conflict']
        obs.update(tracking_valid=bool(tracking_valid),
                   side_exit_confirmed=self._exit_confirmed,
                   exit_edge_frames=self._edge_frames,
                   exit_missing_frames=self._missing_frames,
                   exit_armed=self._exit_armed, open_side=self.selected_side)
        if obs.get('target_x') is None:
            obs['target_x'] = self._last_target
            obs['target_is_cached'] = True
        obs['valid'] = bool(tracking_valid and (geometry_valid or obs['front_clear']))
        obs['observation_ready'] = obs['valid']
        obs['board_bbox_semantics'] = 'TRACKED_UPPER_BOARD_SILHOUETTE_PLUS_HORIZONTAL_MARGIN'
        obs['reasons'].append('Exit needs two edge observations then three distinct empty searches')
        obs['reasons'].append('Side exit is a camera observation, not physical rear-wheel clearance')
        return self._remember(obs, fid, ts)

    def _remember(self, obs, fid, ts):
        self._exit_last_fid = fid
        self._exit_last_ts = ts
        self._exit_seen.add(fid)
        self._exit_cache = deepcopy(obs)
        return obs
