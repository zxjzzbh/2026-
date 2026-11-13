"""Independent, bounded clocks for boundary reacquisition and stopping."""
import math

from .parking_limits import RECOVERY_SETTLE_S


class ParkingRecovery:
    def __init__(self, boundary_timeout_s):
        self.boundary_timeout_s=boundary_timeout_s
        self.reset()

    def reset(self):
        self.started=self.release_at=self.missing_since=None
        self.settle_deadline=self.hard_deadline=None
        self.stop_since=None;self.stop_frames=0
        self.was_driving=False

    def start(self,now,release_at,was_driving):
        self.reset()
        self.started=now;self.release_at=release_at;self.was_driving=was_driving
        self.missing_since=now
        self.hard_deadline=release_at+self.boundary_timeout_s+RECOVERY_SETTLE_S

    def update(self,boundary_ok,motion,captured_s,now,max_age_s):
        if boundary_ok:
            self.missing_since=None
            if self.settle_deadline is None:
                self.settle_deadline=max(now,self.release_at)+RECOVERY_SETTLE_S
        elif self.missing_since is None:self.missing_since=now
        stamp=motion.get('captured_s',captured_s)
        valid=(motion.get('valid') is True and type(motion.get('moving')) is bool
               and type(stamp) in (int,float) and math.isfinite(stamp)
               and 0<=now-stamp<=max_age_s)
        stopped=valid and motion['moving'] is False
        # A geometry blink does not erase independent fresh motion evidence.
        if now>=self.release_at and stopped:
            if self.stop_since is None:self.stop_since=now
            self.stop_frames+=1
        else:self.stop_since=None;self.stop_frames=0
        confirmed=(self.stop_since is not None and self.stop_frames>=3
                   and now-self.stop_since>=.2-1e-9)
        cause=('stop_feedback_invalid' if not valid else
               'still_moving' if not stopped else 'stop_confirmation_pending')
        expired=(now>=self.hard_deadline or
                 not boundary_ok and now-self.missing_since>=self.boundary_timeout_s or
                 boundary_ok and self.settle_deadline is not None and now>=self.settle_deadline)
        resumed=not expired and boundary_ok and (not self.was_driving or confirmed)
        return resumed,cause,expired

    def status(self,now):
        return {'boundary_missing_s':None if self.missing_since is None else now-self.missing_since,
                'settle_remaining_s':None if self.settle_deadline is None else max(0.,self.settle_deadline-now),
                'total_limit_remaining_s':None if self.hard_deadline is None else max(0.,self.hard_deadline-now)}
