"""Traffic-only fixture identity and early stop evidence; no actuator access.

Only current trustworthy red/yellow evidence requests a stop. Image position,
cropping and fixture loss are not a photoelectric gate or distance sensor.
Missing, cropped or ambiguous lamps never become formal red/green votes.
"""
from .brake_parking import numeric
from .traffic_signal import _iou, _reference_geometry, stop_reference_geometry
from .runtime_timing import RACE_CONTROL_WINDOW_S


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


def stop_color_evidence(signal, reference=None):
    """Current red/yellow geometry can stop, never grant a formal color vote."""
    box=fixture_box(signal)
    color=signal.get('state') if box is not None else None
    if color in ('red','yellow'):
        return color,box
    box=fixture_box({'fixture_detected':True,'bbox_xyxy':signal.get('stop_only_box')})
    color=signal.get('stop_only_color')
    if color in ('red','yellow') and box is not None and reference is not None and stop_reference_geometry(box,reference):
        return color,box
    return None,None


class FixtureTrack:
    def __init__(self, *, max_gap_s=.25):
        if not numeric(max_gap_s) or not 0 < max_gap_s <= RACE_CONTROL_WINDOW_S:
            raise ValueError('invalid fixture acquisition interval')
        self.max_gap_s=max_gap_s
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
            if (gap is None or gap > self.max_gap_s+1e-9 or not same_fixture(self.candidate, current)):
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
        self.tracks = {camera:FixtureTrack(max_gap_s=RACE_CONTROL_WINDOW_S) for camera in ('primary','secondary')}
        self.stop_reason = None
        self.sources = []
        self.first_red = None
        self.result = {}
        self.stop_candidates = {camera:None for camera in ('primary','secondary')}

    def update(self, observations, now, *, approaching):
        filtered = {}
        stop_color_sources = []
        confirmed_stop_color_sources = []
        for camera, track in self.tracks.items():
            row = observations.get(camera) or {}
            captured, fid = row.get('captured_s'), row.get('frame_id')
            fresh = (row.get('camera_id') == camera and row.get('image_size') == [480,360]
                     and type(fid) is int and fid >= 0 and numeric(captured)
                     and 0 <= now-captured <= self.s['max_frame_age_s'])
            ordered = fresh and (track.last_id is None or fid > track.last_id and captured > track.last_capture)
            if fresh:
                current=row.get('signal') or {}
                pending=self.stop_candidates[camera]
                # A third current frame may retain weaker LED emission after
                # two strong frames. A blank/off frame can never complete the
                # streak, nor can an unacquired weak candidate start it.
                if (ordered and current.get('weak_red_emission') and current.get('state')=='unknown'
                        and pending is not None and pending['color']=='red' and pending['count']>=2
                        and 0<captured-pending['last_s']<=self.s.get('fast_red_max_gap_s',.25)+1e-9
                        and same_fixture(pending['box'],fixture_box(current))):
                    current={**current,'state':'red','signal_for_race':'red','reason':'current_red_emission_hysteresis'}
                signal, info = track.update(current, fid, captured)
                color,box = stop_color_evidence(signal,track.box)
                if color in ('red','yellow'):
                    stop_color_sources.append(camera)
                candidate=self.stop_candidates[camera]
                if ordered:
                    eligible=(color in ('red','yellow') and box is not None
                              and now-captured<=self.s.get('fast_red_max_age_s',.15))
                    if eligible:
                        match=(candidate is not None and candidate['color']==color
                               and 0<captured-candidate['last_s']<=self.s.get('fast_red_max_gap_s',.15)+1e-9
                               and same_fixture(candidate['box'],box))
                        candidate=({'color':color,'box':box,'first_s':captured,'last_s':captured,'count':1,'confirmed':False}
                                   if not match else {**candidate,'box':box,'last_s':captured,'count':candidate['count']+1})
                        # Allow three fresh frames at the actual ~10 Hz field
                        # rate; once confirmed, continuous same-colour evidence
                        # remains eligible for the formal red/green cycle.
                        span=max(.30,self.s.get('fast_red_max_span_s',.2))
                        if not candidate['confirmed']:
                            if captured-candidate['first_s']>span+1e-9:
                                candidate.update(first_s=captured,count=1)
                            elif candidate['count']>=self.s.get('fast_red_frames',3):
                                candidate['confirmed']=True
                        self.stop_candidates[camera]=candidate
                    else:
                        self.stop_candidates[camera]=candidate=None
                confirmed=(candidate is not None and color==candidate['color'] and info['locked']
                           and candidate['confirmed'] and
                           now-captured<=self.s.get('fast_red_max_age_s',.15)+1e-9)
                if confirmed:
                    confirmed_stop_color_sources.append(camera)
                if approaching and ordered and confirmed:
                    self._stop('confirmed_current_'+color, camera)
                    if self.first_red is None and color=='red':
                        self.first_red = {'camera':camera, 'frame_id':fid,
                                          'captured_s':captured, 'decision_s':now}
                # Unconfirmed colour revokes green, but cannot create a
                # permanent brake transaction before fixture acquisition.
                if color in ('red','yellow') and not confirmed:
                    signal={**signal,'state':'unknown','signal_for_race':'unknown',
                            'reason':'stop_color_confirming'}
                if not info['locked']:
                    signal = {**signal, 'state':'unknown', 'signal_for_race':'unknown',
                              'fixture_detected':False, 'bbox_xyxy':None, 'reason':'fixture_acquiring'}
                filtered[camera] = {**row, 'signal':signal}
            elif not fresh:
                self.stop_candidates[camera]=None
        self.result = {'stop_reason':self.stop_reason, 'sources':list(self.sources),
                       'current_stop_color_sources':stop_color_sources,
                       'confirmed_stop_color_sources':confirmed_stop_color_sources,
                       'first_red':self.first_red,
                       'stop_confirmation':{c:None if v is None else {'color':v['color'],'frames':v['count']}
                                            for c,v in self.stop_candidates.items()},
                       'cameras':{c:t.diagnostics() for c,t in self.tracks.items()},
                       'any_locked':any(t.box is not None for t in self.tracks.values())}
        return filtered, dict(self.result)

    def _stop(self, reason, camera):
        if self.stop_reason is None:
            self.stop_reason = reason
        if camera not in self.sources:
            self.sources.append(camera)
