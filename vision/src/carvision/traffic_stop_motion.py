"""Fresh-frame evidence for traffic-only supplementary braking.

This filters short optical-flow transients; it is not a speed measurement.
Unknown feedback never clears already confirmed sliding.
"""
from .runtime_timing import MOTION_MAX_AGE_S, RACE_CONTROL_WINDOW_S
from .brake_parking import numeric


_NO_INTERVAL=object()


class TrafficStopMotion:
    SETTLE_S = .2
    MOVING_FRAMES = 3
    MOVING_SPAN_S = .2
    CLEAR_SPAN_S = .3

    def __init__(self, stationary_s, *, max_sample_gap_s=MOTION_MAX_AGE_S):
        if not numeric(max_sample_gap_s) or not 0 < max_sample_gap_s <= RACE_CONTROL_WINDOW_S:
            raise ValueError('invalid traffic motion result interval')
        self.max_sample_gap_s=max_sample_gap_s
        self.stationary_s = stationary_s
        self.sliding = False
        self.begin_brake()

    def begin_brake(self, *, preserve_slide=False):
        self.sliding = self.sliding if preserve_slide else False
        self.conflict_after=getattr(self,'conflict_after',None) if preserve_slide else None
        self.observe_after = None
        self.last_capture = self.last_frame = None
        self.moving_since = self.still_since = None
        self.moving_frames = 0
        self.stationary = self.rebrake_ready = False

    def begin_neutral(self, now):
        # Start from actual neutral output, not the nominal pulse deadline.
        self.observe_after = now + self.SETTLE_S

    @property
    def blocking(self):
        return self.sliding or self.moving_frames > 0

    def _clear_samples(self):
        self.moving_since = self.still_since = None
        self.moving_frames = 0
        self.stationary = False

    def _conflict(self, captured):
        self.conflict_after=max(captured,self.conflict_after) if self.conflict_after is not None else captured

    def update(self, moving, captured, frame_id, now, *, continuous_still_since_s=_NO_INTERVAL):
        self.rebrake_ready = False
        if not numeric(now) or not numeric(captured) or not 0 <= now-captured <= MOTION_MAX_AGE_S:
            self._clear_samples()
            # An aged cached result is not a new sampler failure. An unknown
            # result without a timestamp cannot be bridged by an old interval.
            if numeric(now) and not (numeric(captured) and now-captured>MOTION_MAX_AGE_S):
                self._conflict(now)
            return
        if (self.last_capture is not None and captured <= self.last_capture
                or type(frame_id) is int and self.last_frame is not None and frame_id <= self.last_frame):
            return  # A cached frame cannot create another vote or brake request.
        if self.last_capture is None or captured-self.last_capture > self.max_sample_gap_s+1e-9:
            self._clear_samples()
        self.last_capture = captured
        if type(frame_id) is int:
            self.last_frame = frame_id
        if moving is True or type(moving) is not bool:
            self._conflict(captured)
        if self.observe_after is None or captured < self.observe_after or type(moving) is not bool:
            self._clear_samples()
            return
        if moving:
            self.still_since = None
            self.stationary = False
            if self.moving_since is None:
                self.moving_since = captured
            self.moving_frames += 1
            if (self.moving_frames >= self.MOVING_FRAMES
                    and captured-self.moving_since >= self.MOVING_SPAN_S-1e-9):
                self.sliding = self.rebrake_ready = True
        else:
            self.moving_since = None
            self.moving_frames = 0
            if continuous_still_since_s is not _NO_INTERVAL:
                since=continuous_still_since_s
                if (not numeric(since) or since>captured
                        or self.conflict_after is not None and since<=self.conflict_after):
                    self.still_since=None;self.stationary=False
                    return  # Contradictory/unknown sampler history is not stillness.
                # Earlier real rest may overlap this pulse, but only the
                # interval after actual neutral + settle can clear sliding.
                self.still_since=max(since,self.observe_after)
            elif self.still_since is None:
                self.still_since = captured
            span = captured-self.still_since
            if span >= self.CLEAR_SPAN_S-1e-9:
                self.sliding = False
            self.stationary = span >= self.stationary_s-1e-9

    def status(self):
        return {'moving_frames': self.moving_frames,
                'required_frames': self.MOVING_FRAMES,
                'required_span_s': self.MOVING_SPAN_S,
                'settle_s': self.SETTLE_S,
                'observe_after_s': self.observe_after,
                'moving_since_s': self.moving_since,
                'observed_span_s': self.last_capture-self.moving_since if self.moving_since is not None else 0.,
                'last_capture_s': self.last_capture,
                'last_frame_id': self.last_frame,
                'max_result_gap_s': self.max_sample_gap_s,
                'conflict_after_s': self.conflict_after,
                'rebrake_ready': self.rebrake_ready,
                'blocking': self.blocking}
