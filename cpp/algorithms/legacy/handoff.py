"""Fresh, distinct, continuous visual stillness required for a warm handoff."""
import math


class StillnessWindow:
    def __init__(self, hold_s=.3, maximum_gap_s=.25):
        self.hold_s, self.maximum_gap_s = hold_s, maximum_gap_s
        self.first = self.last = None

    def reset(self):
        self.first = self.last = None

    def update(self, motion, now):
        row = motion or {}
        stamp = row.get('captured_s')
        valid = (type(stamp) in (int, float) and math.isfinite(stamp)
                 and 0 <= now-stamp <= self.maximum_gap_s
                 and row.get('valid') is True and row.get('moving') is False)
        if not valid:
            self.reset()
            return False
        if self.last is not None and stamp == self.last:
            return False
        if self.last is None or not 0 < stamp-self.last <= self.maximum_gap_s:
            self.first = stamp
        self.last = stamp
        return stamp-self.first >= self.hold_s-1e-9
