"""Lightweight loop timings; no files, devices, or watchdog-policy changes."""
import sys
import time
import traceback


class LoopDiagnostics:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.current = ('startup', clock())
        self.timings = {}
        self.max_ms = {}
        self.frame_id = None
        self.captured_at = None

    def enter(self, name):
        now = self.clock()
        previous, started = self.current
        elapsed = max(0., (now-started)*1000.)
        self.timings[previous] = self.timings.get(previous, 0.) + elapsed
        self.max_ms[previous] = max(self.max_ms.get(previous, 0.), elapsed)
        self.current = (name, now)

    def begin_frame(self):
        self.enter('camera_read')
        self.timings = {}

    def frame(self, fid, captured_at):
        self.frame_id, self.captured_at = fid, captured_at

    def snapshot(self, now=None):
        now = self.clock() if now is None else now
        name, started = self.current
        return dict(phase=name, phase_elapsed_ms=round(max(0., now-started)*1000., 3),
                    frame=self.frame_id,
                    frame_age_ms=None if self.captured_at is None else
                    round((now-self.captured_at)*1000., 3),
                    timing_ms={k: round(v, 3) for k, v in self.timings.copy().items()})


def main_thread_stack(ident):
    frame = sys._current_frames().get(ident)
    return traceback.format_stack(frame, limit=18) if frame is not None else []
