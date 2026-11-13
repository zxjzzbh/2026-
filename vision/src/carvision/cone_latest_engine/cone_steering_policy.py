"""One road-heading response for approach, passing and recovery."""
import math

from .brake_parking import numeric


def clamp(value, low=-1., high=1.):
    return max(low, min(high, value))


class RoadHeadingControl:
    """Update only from distinct current observations; never integrates travel."""
    def __init__(self):
        self.camera = None
        self.captured = None
        self.error = 0.
        self.command = 0.

    def update(self, geometry):
        if not geometry:
            return 0.
        camera = geometry['camera']
        captured = geometry.get('captured_s', geometry['lane_observed_s'])
        error = geometry['heading_correction_fraction']
        if not all(numeric(v) for v in (captured, error)):
            return 0.
        current = geometry.get('fresh_lane') or geometry.get('single_edge_observed')
        if not current:
            return self.command
        if self.camera == camera and self.captured is not None and captured <= self.captured:
            return self.command
        dt = None if self.camera != camera or self.captured is None else captured-self.captured
        if dt is None or dt > .5:
            self.error = error
            rate = 0.
        else:
            previous = self.error
            bounded = clamp(error, previous-.08, previous+.08)
            self.error += (1-math.exp(-dt/.14))*(bounded-self.error)
            rate = clamp((self.error-previous)/max(dt,.01), -.5, .5)
        self.camera, self.captured = camera, captured
        self.command = clamp(6*self.error+.10*rate, -.65, .65)
        return self.command


def approach_command(lateral, heading, *, settling=False):
    """Retain at least 75% of the requested avoidance before boundary checks."""
    lateral = clamp(lateral, -.75, .75)
    if lateral*heading < 0 and not settling:
        heading = clamp(heading, -abs(lateral)*.25, abs(lateral)*.25)
    return clamp(lateral+heading, -.75, .75)


class ApproachPreview:
    """Unwind from current same-camera lateral progress, without claiming passage."""
    horizon_s = .25

    def __init__(self):
        self.previous = None
        self.report = None

    def update(self, key, captured, error, *, enabled):
        self.report = None
        if not enabled:
            self.previous = None
            return error, False
        previous = self.previous
        velocity = 0.
        if previous and previous[0] == key:
            dt = captured-previous[1]
            if dt == 0:
                error = previous[2]
                velocity = previous[3]
            elif 0 < dt <= .3:
                velocity = .6*clamp((error-previous[2])/dt, -1., 1.)+.4*previous[3]
        if not previous or previous[0] != key or captured > previous[1]:
            self.previous = (key, captured, error, velocity)
        # A shrinking error permits tapering; never reverse across an uncleared
        # cone solely because a short prediction crossed zero.
        shrinking = error*velocity < 0
        predicted = error+clamp(velocity*self.horizon_s, -.12, .12) if shrinking else error
        remaining = max(0., predicted) if error >= 0 else min(0., predicted)
        settling = shrinking and (error*predicted <= 0 or abs(predicted) < .04)
        self.report = dict(error=error, predicted_error=predicted, used_error=remaining,
                           rate_s=velocity, settling=settling, metric_verified=False)
        return remaining, settling


def passing_command(heading, side):
    """Correct road heading alongside the cone, without a center-position term.

    Holding the passing SIDE must not cap the steering needed to stop yawing
    outwards. Output slew is enforced once by ContinuousConeController.
    """
    return clamp(heading, -.65, .65)


class ConeHandoff:
    """Observe the next cone immediately; settle the vehicle before turning out."""
    def __init__(self):
        self.active = False
        self.stable_hits = 0
        self.capture = None
        self.started_s = None
        self.reason = 'inactive'

    max_assist_s = .45

    def begin(self, now=None):
        self.active = True
        self.stable_hits = 0
        self.capture = None
        self.started_s = now
        self.reason = 'brief_heading_assist'

    @staticmethod
    def approaching(target, camera):
        if not target:
            return False
        y = target.get('y_fraction')
        return bool(target.get('image_close') or target.get('near') or
                    numeric(y) and y >= (.70 if camera == 'primary' else .60))

    def advance(self, now, target, camera, *, fresh_target):
        """Bound the heading assist; a visible next cone cannot wait for centering."""
        if not self.active:
            return
        if self.started_s is None:
            self.started_s = now
        urgent = fresh_target and self.approaching(target, camera)
        if urgent or now-self.started_s >= self.max_assist_s:
            self.active = False
            self.reason = 'next_cone_approaching' if urgent else 'heading_assist_budget_reached'

    def update(self, geometry):
        if not self.active or not geometry:
            return
        key = (geometry['camera'], geometry['lane_observed_s'])
        if key == self.capture:
            return
        self.capture = key
        near = geometry.get('near_ego_fraction')
        stable = (geometry.get('fresh_lane') and numeric(near) and .35 <= near <= .65
                  and abs(geometry['heading_correction_fraction']) < .035)
        self.stable_hits = self.stable_hits+1 if stable else 0
        if self.stable_hits >= 2:
            self.active = False
            self.reason = 'road_settled'

    def recovery(self, geometry, road_command, weight):
        if not geometry:
            return road_command
        command = road_command
        near = geometry.get('near_ego_fraction')
        # Position is used only with a currently observed pair. A single stripe
        # can constrain its own edge, but cannot invent a road center.
        if geometry.get('fresh_lane') and numeric(near):
            command += 2*(clamp(near, .4, .6)-near)*weight
        return clamp(command, -.65, .65)
