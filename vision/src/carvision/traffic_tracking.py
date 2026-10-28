"""Traffic-only fixture identity and early stop evidence; no actuator access.

Only current trustworthy red/yellow evidence requests a stop. Image position,
cropping and fixture loss are not a photoelectric gate or distance sensor.
Missing, cropped or ambiguous lamps never become formal red/green votes.
"""
from .brake_parking import numeric
from .traffic_signal import _iou, _reference_geometry


def fixture_box(signal):
    box = signal.get('bbox_xyxy')
    if (signal.get('fixture_detected') is not True
            or signal.get('reason') in ('multiple_fixture_candidates', 'tracked_fixture_missing')
            or not isinstance(box, list) or len(box) != 4
            or not all(numeric(v) for v in box)
            or not 0 <= box[0] < box[2] <= 480 or not 0 <= box[1] < box[3] <= 360):
        return None
    return list(box)


def same_fixture(previous, current):
    """Small consecutive perspective changes, with no jump to a smaller lamp."""
    if previous is None or current is None or _iou(previous, current) < .4:
        return False
    ratios = [(current[i+2]-current[i])/(previous[i+2]-previous[i]) for i in (0, 1)]
    return all(.75 <= ratio <= 1.35 for ratio in ratios)


class FixtureTrack:
    def __init__(self):
        self.box = self.candidate = None
        self.count = 0
        self.last_id = self.last_capture = self.last_seen = None

    def update(self, signal, fid, captured):
        if (type(fid) is not int or not numeric(captured)
                or self.last_id is not None and (fid <= self.last_id or captured <= self.last_capture)):
            return dict(signal), self.diagnostics()
        gap = None if self.last_capture is None else captured-self.last_capture
        self.last_id, self.last_capture = fid, captured
        current = fixture_box(signal)
        matched = current is not None and (self.box is None or same_fixture(self.box, current))
        if self.box is not None and not matched:
            # Retain identity until a new explicit round. Never reacquire a
            # distant fixture after the near fixture has disappeared. A modest
            # shape change may still contain trustworthy current red/yellow:
            # keep that as stop-only evidence, never as a formal color vote.
            filtered = {**signal, 'state':'unknown', 'signal_for_race':'unknown',
                        'fixture_detected':False, 'bbox_xyxy':None,
                        'reason':'tracked_fixture_missing'}
            if current is not None:
                filtered.pop('stop_only_color',None)
                filtered.pop('stop_only_box',None)
                if (signal.get('state') in ('red','yellow')
                        and _reference_geometry(current,self.box)['matched']):
                    filtered.update(stop_only_color=signal['state'],stop_only_box=current)
            return filtered, self.diagnostics()
        if current is None or self.box is None and signal.get('state') not in ('off','red','green','yellow'):
            self.count = 0
            self.candidate = None
            return dict(signal), self.diagnostics()
        if self.box is None:
            if (gap is None or gap > .25 or not same_fixture(self.candidate, current)):
                self.count = 0
            self.count += 1
            self.candidate = current
            if self.count < 2:
                return dict(signal), self.diagnostics()
        self.box, self.last_seen = current, captured
        return dict(signal), self.diagnostics()

    def diagnostics(self):
        return {'locked':self.box is not None, 'bbox_xyxy':self.box,
                'last_seen_s':self.last_seen, 'acquisition_frames':self.count,
                'metric_distance_verified':False}


class TrafficApproachGuard:
    """Filter each camera by identity before voting and latch stop-only evidence."""
    def __init__(self, settings):
        self.s = settings
        self.tracks = {camera:FixtureTrack() for camera in ('primary','secondary')}
        self.stop_reason = None
        self.sources = []
        self.first_red = None
        self.result = {}

    def update(self, observations, now, *, approaching):
        filtered = {}
        stop_color_sources = []
        for camera, track in self.tracks.items():
            row = observations.get(camera) or {}
            captured, fid = row.get('captured_s'), row.get('frame_id')
            fresh = (row.get('camera_id') == camera and row.get('image_size') == [480,360]
                     and type(fid) is int and fid >= 0 and numeric(captured)
                     and 0 <= now-captured <= self.s['max_frame_age_s'])
            ordered = fresh and (track.last_id is None or fid > track.last_id and captured > track.last_capture)
            if fresh:
                signal, info = track.update(row.get('signal') or {}, fid, captured)
                color = signal.get('state') if fixture_box(signal) is not None else None
                if color not in ('red','yellow'):
                    # TrafficCamera may already have filtered a shape change.
                    # Recheck its image-space association to this guard's own
                    # locked fixture; arbitrary metadata cannot select a lamp.
                    box = fixture_box({'fixture_detected':True,
                                       'bbox_xyxy':signal.get('stop_only_box')})
                    color = (signal.get('stop_only_color')
                             if box is not None and track.box is not None
                             and _reference_geometry(box,track.box)['matched'] else None)
                if color in ('red','yellow'):
                    stop_color_sources.append(camera)
                if (approaching and ordered and color in ('red','yellow')
                        and now-captured <= self.s.get('fast_red_max_age_s', .15)):
                    self._stop('first_trustworthy_'+color, camera)
                    if self.first_red is None and color=='red':
                        self.first_red = {'camera':camera, 'frame_id':fid,
                                          'captured_s':captured, 'decision_s':now}
                # A single unassociated frame may stop the car, but cannot
                # establish a formal red/green cycle or borrow another ID.
                if not info['locked']:
                    signal = {**signal, 'state':'unknown', 'signal_for_race':'unknown',
                              'fixture_detected':False, 'bbox_xyxy':None, 'reason':'fixture_acquiring'}
                filtered[camera] = {**row, 'signal':signal}
        self.result = {'stop_reason':self.stop_reason, 'sources':list(self.sources),
                       'current_stop_color_sources':stop_color_sources,
                       'first_red':self.first_red,
                       'cameras':{c:t.diagnostics() for c,t in self.tracks.items()},
                       'any_locked':any(t.box is not None for t in self.tracks.values())}
        return filtered, dict(self.result)

    def _stop(self, reason, camera):
        if self.stop_reason is None:
            self.stop_reason = reason
        if camera not in self.sources:
            self.sources.append(camera)
