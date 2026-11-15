"""Stopped re-confirmation for a delayed debug cone decision, not a new run."""
from .brake_parking import numeric
from .cone_evidence import StablePairedLane, paired_lane
from .handoff import StillnessWindow
from .runtime_timing import MOTION_MAX_AGE_S, VISION_MAX_AGE_S, RACE_CONTROL_WINDOW_S


class FeedbackPause(ValueError):
    """Recoverable absence of guidance; not a clock, config or I/O fault."""


class ConeRecovery:
    def __init__(self):
        self.started = self.until = self.conflict = None
        self.lane = StablePairedLane(duration=.15, frames=3, max_gap_s=RACE_CONTROL_WINDOW_S)
        self.still = StillnessWindow()

    @property
    def active(self):
        return self.started is not None

    def update(self, nav, motion, now, *, interrupted=False, stale=False, was_forward=False):
        initial = not self.active
        if initial:
            if not (interrupted or stale):
                return None
            self.started = now
            self.until = now+.35+.1  # Conservative finite-brake + release interval.
            self.lane.reset(); self.still.reset(); self.conflict = None
        nav, motion = nav or {}, motion or {}
        t = nav.get('captured_s')
        fresh = (numeric(t) and 0 <= now-t <= VISION_MAX_AGE_S
                 and nav.get('experimental') is True and nav.get('ready') is True)
        lane_ready = self.lane.update(nav if fresh and paired_lane(nav) else None, now)
        stamp, since = motion.get('captured_s'), motion.get('continuous_still_since_s')
        current = numeric(stamp) and 0 <= now-stamp <= MOTION_MAX_AGE_S
        if current and (motion.get('valid') is not True or motion.get('moving') is not False):
            self.conflict = max(stamp, self.conflict if self.conflict is not None else stamp)
        stopped = self.still.update(motion if current and stamp >= self.until else None, now)
        if (current and motion.get('valid') is True and motion.get('moving') is False
                and numeric(since) and since <= stamp
                and (self.conflict is None or since > self.conflict)
                and stamp-max(since,self.until) >= .3-1e-9):
            stopped = True
        if fresh and lane_ready and stopped and now >= self.until+.3:
            self.started = None
            return {'action':'neutral','esc_us':1500,'steering_us':1610,
                    'phase':'recovering','reason':'新画面和停稳已确认，下一张新帧重新判断绕行',
                    'control_recovering':True,'control_recovered':True}
        brake = initial and was_forward and not interrupted
        return {'action':'brake' if brake else 'neutral','esc_us':1400 if brake else 1500,
                'steering_us':1610,'phase':'recovering',
                'reason':'控制更新暂缓，保留本轮避障并等待新鲜白线与停稳反馈',
                'control_recovering':True}
