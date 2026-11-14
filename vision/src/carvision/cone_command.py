"""Bounded cone-command cadence over the existing leased actuator.

The sender never creates visual evidence. Its deadline is anchored to the
camera timestamp, including when the same decision/frame is submitted again.
Other race stages pass through synchronously with their original lease.
"""
import copy
import threading
import time

from .brake_output import numeric

CONE_COMMAND_LEASE_S = .5
CONE_RESULT_MAX_AGE_S = .5
COMMAND_PERIOD_S = .05


class ConeCommandSender:
    def __init__(self, actuator, *, clock=time.monotonic, threaded=True):
        self.actuator, self.clock = actuator, clock
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.closed = False
        self.slot = self.deadline = self.last_sent = None
        self.fault = None
        self.count = 0
        self.max_gap_s = 0.
        self.source_frames = {}
        self.page_deadline = None
        self.neutral_slot = self.neutral_until = None
        self.thread = None
        if threaded:
            self.thread = threading.Thread(target=self._work, name='cone-command-sender', daemon=True)
            self.thread.start()

    @property
    def run(self):
        return self.actuator.run

    @property
    def close_error(self):
        return getattr(self.actuator, 'close_error', None)

    def timing(self):
        return {'control_period_s': COMMAND_PERIOD_S, 'commands_sent': self.count,
                'maximum_send_gap_s': self.max_gap_s, 'source_valid_until_s': self.deadline,
                'actuator_lease_s': CONE_COMMAND_LEASE_S if self.slot else None,
                'fault': self.fault}

    def _emit(self, intent, now):
        result = self.actuator.send(intent) or {}
        gap = None if self.last_sent is None else max(0., now-self.last_sent)
        if gap is not None:
            self.max_gap_s = max(self.max_gap_s, gap)
        self.last_sent = now
        self.count += 1
        return {**result, 'command_send_gap_s': gap, 'control_timing': self.timing()}

    def _check(self):
        if self.closed:
            raise RuntimeError('cone command sender closed')
        if self.fault:
            raise RuntimeError(self.fault)

    def send(self, intent, *, local_neutral_until=None):
        with self.lock:
            self._check()
            now = self.clock()
            self.neutral_slot = self.neutral_until = None
            if (intent.get('action') == 'neutral' and intent.get('esc_us') == 1500
                    and (intent.get('control_profile') == 'cone' or 'page_deadline_s' in intent
                         or local_neutral_until is not None)):
                # Prepared/paused neutral has no visual authority. Maintain the
                # executor's cadence only until the authenticated page lease.
                deadline = (local_neutral_until if local_neutral_until is not None
                            else intent.get('page_deadline_s', now+CONE_COMMAND_LEASE_S))
                if not numeric(deadline) or deadline <= now:
                    self._expire(now, reason='避障控制页心跳到期，保持中位')
                    raise RuntimeError(self.fault)
                self.slot = self.deadline = None
                self.page_deadline = deadline if local_neutral_until is None else None
                self.neutral_until = deadline
                self.neutral_slot = copy.deepcopy(intent)
                return self._emit(self.neutral_slot, now)
            if intent.get('control_profile') != 'cone' or (intent.get('action') == 'neutral' and intent.get('esc_us') == 1500):
                self.slot = self.deadline = None
                self.page_deadline = None
                self.last_sent = None
                return self._emit(intent, now)
            observed = intent.get('observation_s')
            if not numeric(observed) or not 0 <= now-observed < CONE_RESULT_MAX_AGE_S:
                self._expire(now)
                raise RuntimeError(self.fault)
            if self.deadline is not None and now >= self.deadline:
                self._expire(now)
                raise RuntimeError(self.fault)
            # A later frame may replace an unexpired result; an old frame
            # cannot move the existing visual deadline backwards or forwards.
            camera = intent.get('observation_camera')
            frame = intent.get('observation_frame_id', observed)
            previous = self.source_frames.get(camera)
            if previous and (observed < previous[1] or frame < previous[0]
                             or frame == previous[0] and observed != previous[1]):
                raise ValueError('cone observation frame/timestamp reversed or reused')
            self.source_frames[camera] = (frame, observed)
            page_deadline = intent.get('page_deadline_s')
            if page_deadline is not None:
                if not numeric(page_deadline) or page_deadline <= now:
                    self._expire(now, reason='避障控制页心跳到期，已请求有限制动')
                    raise RuntimeError(self.fault)
                self.page_deadline = page_deadline
            self.slot = copy.deepcopy(intent)
            self.deadline = observed+CONE_RESULT_MAX_AGE_S
            if self.page_deadline is not None:
                self.deadline = min(self.deadline, self.page_deadline)
            return self._emit(self.slot, now)

    def refresh_page_deadline(self, deadline):
        """An authorized heartbeat can renew page ownership, never pixels."""
        with self.lock:
            if self.closed or self.fault or not numeric(deadline):
                return
            now = self.clock()
            if self.page_deadline is not None and now >= self.page_deadline:
                self._expire(now, reason='避障控制页心跳到期，已请求有限制动')
                return
            self.page_deadline = deadline
            if self.neutral_slot:
                self.neutral_until = deadline
            if self.slot:
                self.deadline = min(self.slot['observation_s']+CONE_RESULT_MAX_AGE_S, deadline)

    def _expire(self, now, *, reason=None):
        self.fault = reason or '避障视觉结果超过 500 ms 未更新，已请求有限制动'
        previous = self.slot
        self.slot = self.deadline = None
        self.neutral_slot = self.neutral_until = None
        # Keep the established finite brake duration in the real guard.
        # This brake is sent once; it is never replayed to extend a pulse.
        intent = {'action': 'brake' if previous and previous['esc_us'] > 1500 else 'neutral',
                  'esc_us': 1400 if previous and previous['esc_us'] > 1500 else 1500,
                  'steering_us': 1610, 'control_profile': 'cone'}
        self._emit(intent, now)

    def tick(self):
        with self.lock:
            if self.closed or self.fault or not (self.slot or self.neutral_slot):
                return
            now = self.clock()
            try:
                if self.neutral_slot:
                    if now >= self.neutral_until:
                        reason = ('避障控制页心跳到期，保持中位' if self.page_deadline is not None
                                  else '车端中位指令未更新，保持停止')
                        self._expire(now, reason=reason)
                    elif self.last_sent is None or now-self.last_sent >= COMMAND_PERIOD_S-1e-9:
                        self._emit(self.neutral_slot, now)
                elif now >= self.deadline:
                    page_expired = self.page_deadline is not None and now >= self.page_deadline
                    self._expire(now, reason='避障控制页心跳到期，已请求有限制动' if page_expired else None)
                elif self.last_sent is None or now-self.last_sent >= COMMAND_PERIOD_S-1e-9:
                    self._emit(self.slot, now)
            except Exception as error:
                self.fault = '避障执行器发送失败：'+str(error)
                self.slot = self.deadline = None

    def _work(self):
        while not self.stop.wait(COMMAND_PERIOD_S/2):
            self.tick()

    def close(self):
        self.stop.set()
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.slot = self.deadline = None
            self.neutral_slot = self.neutral_until = None
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=.2)
        self.actuator.close()
