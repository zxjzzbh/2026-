"""Time-based evidence, using media time offline and receipt time live."""


class TimedConfirmation:
    def __init__(self, confirm_ms, max_gap_ms):
        self.confirm_ms = confirm_ms
        self.max_gap_ms = max_gap_ms
        self.reset()

    def reset(self):
        self.candidate = None
        self.since = None
        self.last = None

    def update(self, value, timestamp_ms):
        if value == "unknown" or timestamp_ms is None:
            self.reset()
            return "unknown"
        if self.last is not None and (timestamp_ms <= self.last or timestamp_ms-self.last > self.max_gap_ms):
            self.reset()
        if value != self.candidate:
            self.candidate, self.since = value, timestamp_ms
        self.last = timestamp_ms
        return value if timestamp_ms-self.since >= self.confirm_ms else "unknown"
