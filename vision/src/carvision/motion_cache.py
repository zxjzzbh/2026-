"""Sample ground motion independently of lane fitting and preview encoding.

Reads the existing camera cache only. Never opens a camera or actuator and
never turns unavailable/stale pixels into evidence of a stationary vehicle.
"""
import threading
import time

import cv2
import numpy as np

from .brake_parking import GroundMotionEstimate, numeric
from .handoff import StillnessWindow
from .runtime_timing import MOTION_MAX_AGE_S
from .camera_frames import control_frame


class GroundMotionCache:
    def __init__(self, states, camera='secondary', *, clock=time.monotonic):
        self.states, self.camera, self.clock = states, camera, clock
        self.estimator = GroundMotionEstimate()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = None
        self.identity = None
        self.latest = {'valid': False, 'moving': None, 'reason': 'warming_up'}
        self.stillness = StillnessWindow()

    def sample_once(self):
        state = self.states.get(self.camera)
        if state is None:
            return
        try:
            view=control_frame(state)
            fid,ms=view['frame_id'],view['captured_ms']
        except ValueError:
            with self.lock:
                self.stillness.reset()
                self.latest={'valid':False,'moving':None,'reason':'camera_unavailable'}
            return
        now = self.clock()
        if not 0 <= now-ms/1000 <= MOTION_MAX_AGE_S:
            with self.lock:
                self.stillness.reset()
                self.latest = {'valid': False, 'moving': None, 'reason': 'camera_unavailable'}
            return
        identity = (fid, ms)
        if identity == self.identity:
            return
        self.identity = identity
        image = view['image']
        row = {**self.estimator.update(image, fid, ms/1000),
               'captured_s': ms/1000, 'frame_id': fid, 'camera_id': self.camera}
        with self.lock:
            self.stillness.update(row, self.clock())
            row['continuous_still_since_s'] = self.stillness.first
            self.latest = row

    def _work(self):
        while not self.stop.is_set():
            try:
                self.sample_once()
            except Exception as exc:
                with self.lock:
                    self.stillness.reset()
                    self.latest = {'valid': False, 'moving': None, 'reason': str(exc)}
            self.stop.wait(.02)

    def snapshot(self):
        with self.lock:
            if self.thread is None and not self.stop.is_set():
                self.thread = threading.Thread(target=self._work, name='ground-motion-cache', daemon=True)
                self.thread.start()
            row = dict(self.latest)
        stamp = row.get('captured_s')
        if self.stop.is_set() or not numeric(stamp) or not 0 <= self.clock()-stamp <= MOTION_MAX_AGE_S:
            row.update(valid=False, moving=None, reason='ground_motion_stale')
        return row

    def close(self):
        self.stop.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=.5)
