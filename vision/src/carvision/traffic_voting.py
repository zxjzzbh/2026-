"""Independent 2-second red/green votes; OR across cameras, red wins ties."""
from collections import deque

from .brake_parking import numeric
from .traffic_signal import _iou

CAMERAS = ('primary', 'secondary')


def valid_lamp_state(observation):
    box = observation.get('bbox_xyxy')
    good = (observation.get('fixture_detected') is True and isinstance(box, list) and len(box) == 4
            and all(numeric(x) and x >= 0 for x in box) and box[2] > box[0] and box[3] > box[1])
    state = observation.get('state')
    return (state if good and state in ('red','green','yellow','off') else 'unknown'), (box if good else None)


class SignalFrameVote:
    def __init__(self, window_s, threshold_percent):
        if window_s != 2 or not numeric(threshold_percent) or not 1 <= threshold_percent <= 100:
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


class DualSignalVotes:
    def __init__(self, window_s, threshold_percent, max_age_s=.25):
        self.window_s, self.threshold, self.max_age_s = window_s, threshold_percent, max_age_s
        self.reset()

    def reset(self):
        self.votes = {camera:SignalFrameVote(self.window_s,self.threshold) for camera in CAMERAS}
        self.boxes = {camera:None for camera in CAMERAS}

    def _reset_camera(self, camera):
        self.votes[camera] = SignalFrameVote(self.window_s,self.threshold)
        self.boxes[camera] = None

    def update(self, observations, now):
        results = {}
        for camera in CAMERAS:
            row = observations.get(camera) or {}
            fid, captured = row.get('frame_id'), row.get('captured_s')
            fresh = (row.get('camera_id') == camera and row.get('image_size') == [480,360]
                     and type(fid) is int and numeric(captured) and numeric(now)
                     and 0 <= now-captured <= self.max_age_s)
            if not fresh:
                self._reset_camera(camera)
                results[camera] = {**self.votes[camera].summary(),'valid':False,'state':'unknown'}
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
                if box is not None: self.boxes[camera] = box
            # Cached copies are visible but never add votes or refresh time.
            results[camera] = {**vote.summary(),'valid':True,'state':state,'frame_id':fid,'captured_s':captured}
        red_sources = [c for c,v in results.items() if v['valid'] and v['red_confirmed']]
        green_sources = [c for c,v in results.items() if v['valid'] and v['green_confirmed']]
        yellow = any(v['valid'] and v['state']=='yellow' for v in results.values())
        raw_states = {v['state'] for v in results.values() if v['valid']}
        raw = next((c for c in ('red','yellow','green','off') if c in raw_states),'unknown')
        return {'window_s':self.window_s,'threshold_percent':self.threshold,'cameras':results,
                'red_sources':red_sources,'green_sources':green_sources,'red_confirmed':bool(red_sources),
                'green_confirmed':bool(green_sources) and not red_sources and not yellow,
                'conflict':bool(red_sources and green_sources),'yellow_present':yellow,'raw_state':raw,
                'state':'red' if red_sources else 'yellow' if yellow else 'green' if green_sources else 'unknown',
                'any_fresh':any(v['valid'] for v in results.values())}
