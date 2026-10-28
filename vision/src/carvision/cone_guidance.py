"""Current cone guidance evidence and bounded observation episodes.

These are image/time policies only.  Neither a stable image target nor an
elapsed episode verifies vehicle motion, clearance or cone passage.  The
five-second target-only limit is a trial estimate, not a measured trajectory.
"""
from .brake_output import numeric
from .cone_evidence import paired_lane
from .cone_response import current_visual


MAX_FRAME_AGE_S = .25
TARGET_FRAMES = 3
TARGET_SPAN_S = .15
NO_GUIDANCE_MAX_S = 1.
TARGET_ONLY_MAX_S = 5.


def _image_size(size):
    return (isinstance(size, (tuple, list)) and len(size) == 2
            and all(type(value) is int and value > 0 for value in size))


def _frame(nav, now):
    if not isinstance(nav, dict) or not numeric(now):
        return None
    camera, fid, captured = (nav.get(key) for key in
                              ('source_camera', 'frame_id', 'captured_s'))
    if (camera not in ('primary', 'secondary') or type(fid) is not int or fid < 0
            or not numeric(captured) or not 0 <= now-captured <= MAX_FRAME_AGE_S):
        return None
    return camera, fid, captured


def strict_visual(nav, identifier, now, *, image_size=(480, 360)):
    """Return a current measured pixel entry, or None for incomplete evidence.

    Fractions describe the observed foot point, which must lie inside its
    finite on-image bounding box; they are not a projected ground position.
    """
    frame = _frame(nav, now)
    if (frame is None or not _image_size(image_size)
            or not isinstance(identifier, str) or not 1 <= len(identifier) <= 128
            or not identifier.startswith(frame[0]+'-')
            or len(identifier) <= len(frame[0])+1):
        return None
    entries = nav.get('visual_cones')
    if not isinstance(entries, (list, tuple)) or not all(isinstance(row, dict) for row in entries):
        return None
    entry = current_visual(nav, identifier)
    if entry is None:
        return None
    revision = entry.get('identity_revision')
    box = entry.get('bbox_xyxy')
    x, y, half = (entry.get(key) for key in
                  ('image_fraction', 'y_fraction', 'half_image_width'))
    if (type(entry.get('frame_id')) is not int or not numeric(entry.get('captured_s'))
            or type(revision) is not int or revision < 0
            or not isinstance(box, (list, tuple)) or len(box) != 4
            or not all(numeric(value) for value in (*box, x, y, half))
            or not 0 <= x < 1 or not 0 <= y < 1):
        return None
    width, height = image_size
    x1, y1, x2, y2 = box
    if (not 0 <= x1 < x2 <= width or not 0 <= y1 < y2 <= height
            or not x1 <= x*width <= x2 or not y1 <= y*height <= y2
            or abs(half-(x2-x1)/(2*width)) > 1e-9):
        return None
    return entry


class CurrentTargetEvidence:
    """Three independent current frames for one camera, ID, side and revision."""
    def __init__(self, image_size=(480, 360)):
        if not _image_size(image_size):
            raise ValueError('invalid target evidence image size')
        self.image_size = tuple(image_size)
        self.source_frames = {}
        self.key = self.first_s = self.last_s = None
        self.frames = 0

    def _clear(self):
        self.key = self.first_s = self.last_s = None
        self.frames = 0

    def _result(self, *, current=False, fresh=False, reason):
        span = 0. if self.first_s is None else self.last_s-self.first_s
        stable = bool(current and fresh and self.frames >= TARGET_FRAMES
                      and span >= TARGET_SPAN_S-1e-9)
        return {'current_valid': current, 'fresh_evidence': fresh,
                'target_stable': stable, 'frames': self.frames, 'span_s': span,
                'source_camera': self.key[0] if self.key else None,
                'target_id': self.key[1] if self.key else None,
                'side': self.key[2] if self.key else None,
                'identity_revision': self.key[3] if self.key else None,
                'reason': reason, 'physical_path_verified': False}

    def update(self, nav, target_id, side, now):
        frame = _frame(nav, now)
        visual = strict_visual(nav, target_id, now, image_size=self.image_size)
        if frame is None:
            self._clear()
            return self._result(reason='invalid_or_old_frame')
        camera, fid, captured = frame
        previous = self.source_frames.get(camera)
        fresh = previous is None or fid > previous[0] and captured > previous[1]
        if fresh:
            self.source_frames[camera] = (fid, captured)
        if visual is None:
            self._clear()
            return self._result(fresh=fresh, reason='no_bound_current_visual')
        if side not in ('left', 'right'):
            self._clear()
            return {**self._result(current=True, fresh=fresh, reason='unbound_avoidance_side'),
                    'source_camera': camera, 'target_id': target_id, 'side': side,
                    'identity_revision': visual['identity_revision']}
        key = (camera, target_id, side, visual['identity_revision'])
        if key != self.key:
            self._clear()
        if not fresh:
            return self._result(current=True, reason='duplicate_or_old_frame')
        if self.last_s is None or captured-self.last_s > MAX_FRAME_AGE_S:
            self._clear()
            self.key, self.first_s = key, captured
        self.last_s = captured
        self.frames += 1
        return self._result(current=True, fresh=True, reason='current_target_sequence')


class GuidanceBudgets:
    """No-guidance and target-only deadlines, independent of target identity.

    Call after motion has started. Expiry is latched before accepting the next
    observation; a late good frame cannot retrospectively authorize blind time.
    Only stable paired road evidence ends an existing target-only episode.
    """
    def __init__(self):
        self.none_since = self.target_only_since = None
        self.source_frames = {}
        self.last_now = None
        self.expired_reason = None
        self.mode = 'none'

    def _expire(self, now):
        if self.expired_reason is not None:
            return
        if self.none_since is not None and now >= self.none_since+NO_GUIDANCE_MAX_S-1e-9:
            self.expired_reason = 'no_reliable_guidance_deadline'
        elif self.target_only_since is not None and now >= self.target_only_since+TARGET_ONLY_MAX_S-1e-9:
            self.expired_reason = 'target_only_episode_deadline'

    def diagnostics(self, now):
        none_deadline = None if self.none_since is None else self.none_since+NO_GUIDANCE_MAX_S
        target_deadline = None if self.target_only_since is None else self.target_only_since+TARGET_ONLY_MAX_S
        return {'mode': self.mode, 'expired': self.expired_reason is not None,
                'reason': self.expired_reason,
                'none_since_s': self.none_since, 'none_deadline_s': none_deadline,
                'none_remaining_s': None if none_deadline is None else max(0., none_deadline-now),
                'none_limit_s': NO_GUIDANCE_MAX_S,
                'target_only_since_s': self.target_only_since,
                'target_only_deadline_s': target_deadline,
                'target_only_remaining_s': None if target_deadline is None else max(0., target_deadline-now),
                'target_only_limit_s': TARGET_ONLY_MAX_S,
                'target_only_limit_verified': False, 'physical_path_verified': False}

    def update(self, nav, now, *, paired_stable=False, single_valid=False,
               target_stable=False, target_current_valid=False):
        if not numeric(now) or self.last_now is not None and now < self.last_now:
            raise ValueError('invalid guidance budget clock')
        self.last_now = now
        self._expire(now)
        if self.expired_reason is not None:
            return {**self.diagnostics(now), 'fresh_evidence': False}
        frame = _frame(nav, now)
        fresh = False
        captured = now
        if frame is not None:
            camera, fid, captured = frame
            previous = self.source_frames.get(camera)
            fresh = previous is None or fid > previous[0] and captured > previous[1]
            if fresh:
                self.source_frames[camera] = (fid, captured)
        elif isinstance(nav, dict) and numeric(nav.get('captured_s')) and nav['captured_s'] <= now:
            captured = nav['captured_s']
        road = bool(fresh and paired_stable is True and paired_lane(nav))
        age = (nav or {}).get('prediction_age_s', (nav or {}).get('line_age_s')) if isinstance(nav, dict) else None
        single = bool(fresh and single_valid is True and nav.get('line_state') == 'single'
                      and not nav.get('line_missing') and numeric(age) and 0 <= age <= MAX_FRAME_AGE_S
                      and nav.get('lane_bounds_px') and numeric(nav.get('ego_fraction')))
        target = bool(fresh and target_stable is True)
        current = bool(fresh and (target_current_valid is True or target))
        if road:
            self.mode = 'paired'
            self.none_since = self.target_only_since = None
        else:
            if not single and current and self.target_only_since is None:
                self.target_only_since = captured
            self.mode = 'single' if single else 'target_only' if target else 'none'
            if single or target:
                self.none_since = None
            elif self.none_since is None:
                self.none_since = captured
        self._expire(now)
        return {**self.diagnostics(now), 'fresh_evidence': fresh}
