"""Fresh-frame evidence for traffic-only supplementary braking.

This filters short optical-flow transients; it is not a speed measurement.
Unknown feedback never clears already confirmed sliding.
"""
from .runtime_timing import MOTION_MAX_AGE_S


class TrafficStopMotion:
    SETTLE_S = .2
    MOVING_FRAMES = 3
    MOVING_SPAN_S = .2
    CLEAR_SPAN_S = .3

    def __init__(self, stationary_s):
        self.stationary_s = stationary_s
        self.sliding = False
        self.begin_brake()

    def begin_brake(self, *, preserve_slide=False):
        self.sliding = self.sliding if preserve_slide else False
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

    def update(self, moving, captured, frame_id, now):
        self.rebrake_ready = False
        if (self.observe_after is None or type(moving) is not bool
                or captured is None or not 0 <= now-captured <= MOTION_MAX_AGE_S
                or captured < self.observe_after):
            self.moving_since = self.still_since = None
            self.moving_frames = 0
            self.stationary = False
            return
        if (self.last_capture is not None and captured <= self.last_capture
                or type(frame_id) is int and self.last_frame is not None and frame_id <= self.last_frame):
            return  # A cached frame cannot create another vote or brake request.
        if self.last_capture is None or captured-self.last_capture > MOTION_MAX_AGE_S:
            self.moving_since = self.still_since = None
            self.moving_frames = 0
            self.stationary = False
        self.last_capture = captured
        if type(frame_id) is int:
            self.last_frame = frame_id
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
            if self.still_since is None:
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
                'rebrake_ready': self.rebrake_ready,
                'blocking': self.blocking}
