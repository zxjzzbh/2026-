"""Independent timed red/green votes; OR across cameras, red wins ties."""
from collections import deque

from .brake_parking import numeric
from .traffic_signal import _iou

CAMERAS = ('primary', 'secondary')
FAST_RED_DEFAULTS = {'fast_red_frames':3, 'fast_red_max_span_s':.20,
                     'fast_red_max_gap_s':.15, 'fast_red_max_age_s':.15}


def valid_lamp_state(observation):
    box = observation.get('bbox_xyxy')
    good = (observation.get('fixture_detected') is True and isinstance(box, list) and len(box) == 4
            and all(numeric(x) and x >= 0 for x in box) and box[2] > box[0] and box[3] > box[1])
    state = observation.get('state')
    return (state if good and state in ('red','green','yellow','off') else 'unknown'), (box if good else None)


class SignalFrameVote:
    def __init__(self, window_s, threshold_percent):
        if (not numeric(window_s) or window_s not in (1, 2)
                or not numeric(threshold_percent) or not 1 <= threshold_percent <= 100):
            raise ValueError('invalid signal vote window/percentage')
        self.window_s, self.threshold = window_s, threshold_percent
        self.frames = deque()
        self.started = self.last_capture = self.last_id = None

    def update(self, state, frame_id, captured_s):
        if (type(frame_id) is not int or not numeric(captured_s)
                or self.last_id is not None and frame_id <= self.last_id
                or self.last_capture is not None and not 0 < captured_s-self.last_capture <= .25):
            raise ValueError('signal voting needs distinct continuous frames')
        if self.started is None: self.started = captured_s
        self.last_id, self.last_capture = frame_id, captured_s
        self.frames.append((captured_s, state))
        while self.frames and self.frames[0][0] <= captured_s-self.window_s+1e-9:
            self.frames.popleft()
        return self.summary()

    def summary(self):
        total = len(self.frames)
        span = 0 if self.started is None else min(self.window_s, self.last_capture-self.started)
        ready = span >= self.window_s-1e-9
        result = {'window_s':self.window_s,'threshold_percent':self.threshold,'total_frames':total,
                  'collected_s':span,'ready':ready}
        for color in ('red','green'):
            count = sum(state == color for _,state in self.frames)
            result.update({color+'_frames':count, color+'_percent':100*count/total if total else 0,
                           color+'_confirmed':ready and total > 0 and count*100 >= self.threshold*total})
        result['confirmed'] = result['red_confirmed']  # Old red-only replay API.
        return result


class _FastRedCandidate:
    """Short red streak plus a deadline that missing evidence cannot renew."""
    def __init__(self, required, max_span, max_gap, max_age):
        self.required, self.max_span, self.max_gap, self.max_age = required, max_span, max_gap, max_age
        self.last_id = self.last_capture = self.last_box = None
        self.pending = None
        self.clear_streak()

    def clear_streak(self):
        self.count = 0
        self.first_capture = self.anchor_box = None

    def _expired(self, now):
        if self.pending is None:return None
        if not numeric(now):return 'invalid_clock'
        if now-self.pending['first_s'] > self.max_span+1e-9:return 'candidate_span'
        if now-self.pending['last_red_s'] > self.max_age+1e-9:return 'frame_age'
        if now-self.pending['last_red_s'] > self.max_gap+1e-9:return 'frame_gap'
        return None

    def update(self, state, box, fid, captured, now, *, fresh, allow_late_clear=False):
        # Check before accepting a new observation: late green or a new box
        # cannot retroactively cancel a missed stopping deadline.
        timeout = self._expired(now)
        if not fresh:
            self.clear_streak()
        else:
            duplicate = fid == self.last_id and captured == self.last_capture
            ordered = self.last_id is None or fid > self.last_id and captured > self.last_capture
            continuous = ordered and (self.last_capture is None or captured-self.last_capture <= self.max_gap+1e-9)
            if not duplicate:
                same_streak = (self.anchor_box is not None and box is not None
                               and _iou(box,self.anchor_box) >= .25
                               and self.last_box is not None and _iou(box,self.last_box) >= .25)
                if state == 'red':
                    if self.pending is None:
                        self.pending = {'first_s':captured, 'last_red_s':captured,
                                        'frame_id':fid, 'box':list(box)}
                    elif continuous and _iou(box,self.pending['box']) >= .25:
                        self.pending['last_red_s'] = max(self.pending['last_red_s'],captured)
                    if not continuous or not same_streak or now-captured > self.max_age+1e-9:
                        self.clear_streak()
                    if continuous and now-captured <= self.max_age+1e-9:
                        if not self.count:
                            self.first_capture, self.anchor_box = captured, list(box)
                        self.count += 1
                else:
                    self.clear_streak()
                    if ((not timeout or allow_late_clear) and state in ('off','green') and ordered
                            and self.pending is not None and now-captured <= self.max_age+1e-9
                            and fid > self.pending['frame_id'] and captured > self.pending['first_s']
                            and box is not None and _iou(box,self.pending['box']) >= .25):
                        self.pending = None
                self.last_id, self.last_capture, self.last_box = fid, captured, box
            elif state != 'red':
                self.clear_streak()
        timeout = timeout or self._expired(now)
        confirmed = (fresh and state == 'red' and self.count >= self.required
                     and now-captured <= self.max_age+1e-9 and not timeout
                     and captured-self.first_capture <= self.max_span+1e-9)
        return {'red_streak_frames':self.count, 'fast_red_confirmed':bool(confirmed),
                'fast_red_timeout':timeout, 'fast_red_pending':self.pending is not None,
                'fast_red_pending_since_s':None if self.pending is None else self.pending['first_s']}


class DualSignalVotes:
    def __init__(self, window_s, threshold_percent, max_age_s=.25, *, green_window_s=None,
                 fast_red_frames=3, fast_red_max_span_s=.20, fast_red_max_gap_s=.15, fast_red_max_age_s=.15):
        self.window_s, self.threshold, self.max_age_s = window_s, threshold_percent, max_age_s
        self.green_window_s = window_s if green_window_s is None else green_window_s
        if (type(fast_red_frames) is not int or fast_red_frames not in (2,3)
                or any(not numeric(v) or not 0 < v <= .25
                       for v in (fast_red_max_span_s,fast_red_max_gap_s,fast_red_max_age_s))):
            raise ValueError('invalid fast red evidence budget')
        self.fast_settings = (fast_red_frames,fast_red_max_span_s,fast_red_max_gap_s,fast_red_max_age_s)
        self.reset()

    def reset(self):
        self.votes = {camera:SignalFrameVote(self.window_s,self.threshold) for camera in CAMERAS}
        self.green_votes = {camera:SignalFrameVote(self.green_window_s,self.threshold) for camera in CAMERAS}
        self.boxes = {camera:None for camera in CAMERAS}
        self.fast = {camera:_FastRedCandidate(*self.fast_settings) for camera in CAMERAS}
        self.green_wait = False

    def reset_green(self):
        """A new stop starts a fresh green window, retaining this round's red."""
        self.green_votes = {camera:SignalFrameVote(self.green_window_s,self.threshold) for camera in CAMERAS}

    def begin_green_wait(self):
        """The estimated stop fulfils old fast-stop requests, not formal red."""
        self.reset_green()
        self.green_wait = True
        for candidate in self.fast.values():
            candidate.pending = None
            candidate.clear_streak()

    def _reset_camera(self, camera):
        self.votes[camera] = SignalFrameVote(self.window_s,self.threshold)
        self.green_votes[camera] = SignalFrameVote(self.green_window_s,self.threshold)
        self.boxes[camera] = None
        self.fast[camera].clear_streak()

    def _summary(self, camera):
        red, green = self.votes[camera].summary(), self.green_votes[camera].summary()
        return {**red, 'green_frames':green['green_frames'],
                'green_percent':green['green_percent'], 'green_confirmed':green['green_confirmed'],
                'green_window_s':self.green_window_s, 'green_total_frames':green['total_frames'],
                'green_collected_s':green['collected_s'], 'green_ready':green['ready']}

    def update(self, observations, now):
        results = {}
        for camera in CAMERAS:
            row = observations.get(camera) or {}
            fid, captured = row.get('frame_id'), row.get('captured_s')
            fresh = (row.get('camera_id') == camera and row.get('image_size') == [480,360]
                      and type(fid) is int and fid >= 0 and numeric(captured) and numeric(now)
                     and 0 <= now-captured <= self.max_age_s)
            if not fresh:
                self._reset_camera(camera)
                fast = self.fast[camera].update('unknown',None,fid,captured,now,fresh=False,
                                                allow_late_clear=self.green_wait)
                results[camera] = {**self._summary(camera),**fast,'valid':False,'state':'unknown'}
                continue
            state, box = valid_lamp_state(row.get('signal') or {})
            vote = self.votes[camera]
            duplicate = fid == vote.last_id and captured == vote.last_capture
            if not duplicate:
                if (vote.last_id is not None and (fid <= vote.last_id or not 0 < captured-vote.last_capture <= .25)
                        or box is not None and self.boxes[camera] is not None and _iou(box,self.boxes[camera]) < .25):
                    self._reset_camera(camera)
                    vote = self.votes[camera]
                vote.update(state, fid, captured)
                self.green_votes[camera].update(state, fid, captured)
                if box is not None: self.boxes[camera] = box
            # Cached copies are visible but never add votes or refresh time.
            fast = self.fast[camera].update(state,box,fid,captured,now,fresh=True,
                                            allow_late_clear=self.green_wait)
            results[camera] = {**self._summary(camera),**fast,'valid':True,'state':state,'frame_id':fid,'captured_s':captured}
        red_sources = [c for c,v in results.items() if v['valid'] and v['red_confirmed']]
        green_sources = [c for c,v in results.items() if v['valid'] and v['green_confirmed']]
        red_present = any(v['valid'] and v['state']=='red' for v in results.values())
        current_green_sources = [c for c in green_sources if results[c]['state']=='green']
        yellow = any(v['valid'] and v['state']=='yellow' for v in results.values())
        raw_states = {v['state'] for v in results.values() if v['valid']}
        raw = next((c for c in ('red','yellow','green','off') if c in raw_states),'unknown')
        fast_sources = [c for c,v in results.items() if v['fast_red_confirmed']]
        timeout_sources = [c for c,v in results.items() if v['fast_red_timeout']]
        pending_sources = [c for c,v in results.items() if v['fast_red_pending']]
        return {'window_s':self.window_s,'green_window_s':self.green_window_s,'threshold_percent':self.threshold,'cameras':results,
                'fast_red_sources':fast_sources, 'fast_red_timeout_sources':timeout_sources,
                'fast_red_timeout_reasons':{c:results[c]['fast_red_timeout'] for c in timeout_sources},
                'fast_red_pending_sources':pending_sources,
                'red_sources':red_sources,'green_sources':green_sources,'red_confirmed':bool(red_sources),
                'green_confirmed':bool(current_green_sources) and not red_sources and not red_present and not yellow
                                  and not (self.green_wait and pending_sources),
                'red_present':red_present,'current_green_sources':current_green_sources,
                'conflict':bool(red_sources and green_sources),'yellow_present':yellow,'raw_state':raw,
                'state':'red' if red_sources else 'yellow' if yellow else 'green' if green_sources else 'unknown',
                'any_fresh':any(v['valid'] for v in results.values())}
