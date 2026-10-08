"""Dry-run-first, bounded S3/S4 executor for RasAdapter5A.

Only the existing transport's steering range and reviewed S4 neutral/forward
points are accepted. Fractional throttle and reverse sequencing are not yet
calibrated/supported. Importing and dry runs never open the UART. Board readback
and planned pulses are not physical position, speed or stopped feedback.

The real-mode watchdog checks deadlines independently of caller ticks. Expiry,
output errors and emergency stop latch motion off for this executor instance.
This software watchdog still depends on OS scheduling and UART writes; it is
not an MCU watchdog or a substitute for physical cutoff.
"""

import math
import threading
import time
from dataclasses import dataclass

from .rasadapter5 import RasAdapter


@dataclass(frozen=True)
class RasAdapterExecutionConfig:
    port: str = "/dev/ttyAMA0"
    steering_channel: int = 3
    esc_channel: int = 4
    steering_left_us: int | None = None
    steering_center_us: int | None = None
    steering_right_us: int | None = None
    esc_neutral_us: int | None = None
    esc_forward_us: int | None = None
    esc_reverse_us: int | None = None
    calibration_confirmed: bool = False
    command_lease_s: float = 0.25
    maximum_motion_s: float = 0.8

    def missing(self):
        names = []
        for name in (
            "steering_left_us",
            "steering_center_us",
            "steering_right_us",
            "esc_neutral_us",
            "esc_forward_us",
            "esc_reverse_us",
        ):
            if getattr(self, name) is None:
                names.append(name)
        if not self.calibration_confirmed:
            names.append("calibration_confirmed")
        return names

    def validate(self, require_ready=False):
        if (type(self.steering_channel) is not int or self.steering_channel != 3
                or type(self.esc_channel) is not int or self.esc_channel != 4):
            raise ValueError("RasAdapter5A autonomous channels are S3 steering and S4 ESC")
        if not isinstance(self.port, str) or not self.port.strip():
            raise ValueError("port must be a nonempty UART path")
        if type(self.calibration_confirmed) is not bool:
            raise ValueError("calibration_confirmed must be a boolean")
        if not _finite(self.command_lease_s) or not 0 < self.command_lease_s <= 0.25:
            raise ValueError("command_lease_s must be in (0, 0.25]")
        if not _finite(self.maximum_motion_s) or not 0 < self.maximum_motion_s <= 0.8:
            raise ValueError("maximum_motion_s must be in (0, 0.8]")
        for name in (
            "steering_left_us",
            "steering_center_us",
            "steering_right_us",
        ):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or not 1550 <= value <= 1750):
                raise ValueError(f"{name} must fit the reviewed S3 1550..1750 us range")
        for name, reviewed in (("esc_neutral_us", 1500), ("esc_forward_us", 1575),
                               ("esc_reverse_us", 1300)):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or value != reviewed):
                raise ValueError(f"{name} must be the reviewed S4 point {reviewed} us")
        steering = (self.steering_left_us, self.steering_center_us, self.steering_right_us)
        if all(value is not None for value in steering) and not (
                steering[0] < steering[1] < steering[2]
                or steering[2] < steering[1] < steering[0]):
            raise ValueError("steering center must lie between left and right")
        if require_ready and self.missing():
            raise ValueError("RasAdapter calibration incomplete: " + ", ".join(self.missing()))
        return self


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class _DryRasAdapter:
    """Protocol-shaped fake that records commands without opening hardware."""

    def __init__(self):
        self.commands = []
        self.opened = False

    def open(self):
        self.opened = True
        self.commands.append(("open",))

    def set_position(self, channel, pulse, seconds=0.02):
        self.commands.append(("steering", channel, pulse, seconds))

    def set_esc(self, channel, pulse):
        self.commands.append(("esc", channel, pulse))

    def close(self):
        self.commands.append(("close",))
        self.opened = False


class RasAdapterS3S4Executor:
    """Bounded S3/S4 executor with latched emergency stop.

    ``run=False`` is the default and records planned commands only. A real
    adapter is opened only with ``run=True`` and a ready calibration. Refreshes
    renew the command lease but never extend the current motion window. Use a
    context manager or explicit ``close()`` for exception cleanup. Faults cannot
    be cleared by neutral, reopening, or further commands on this instance.
    """

    def __init__(self, config, *, run=False, adapter=None, clock=time.monotonic):
        if type(run) is not bool:
            raise ValueError("run must be a boolean")
        self.config = config.validate(require_ready=run)
        self.run = run
        self.adapter = adapter if adapter is not None else (RasAdapter(self.config.port) if run else _DryRasAdapter())
        self.clock = clock
        self.lock = threading.RLock()
        self.opened = False
        self.closed = False
        self.estop_latched = False
        self.error = None
        self.cleanup_errors = []
        self.stop_event = threading.Event()
        self.watchdog = None
        self.expires = None
        self.motion_until = None
        self.last_command = None

    @property
    def commands(self):
        return getattr(self.adapter, "commands", None)

    def open(self):
        with self.lock:
            self._require_available(require_open=False)
            if self.opened:
                return
            if not self.run:
                self.opened = True
                return
            try:
                self.adapter.open()
                self.opened = True
                self._send_neutral()
                self.watchdog = threading.Thread(
                    target=self._watchdog, daemon=True, name="rasadapter-s3s4-watchdog")
                self.watchdog.start()
            except BaseException as exc:
                self._latch_fault(exc)
                try:
                    self.adapter.close()
                except Exception as cleanup:
                    self.cleanup_errors.append(f"open cleanup: {cleanup}")
                self.opened = False
                self.closed = True
                self.stop_event.set()
                raise

    def _require_available(self, require_open=True):
        if self.estop_latched:
            raise RuntimeError("RasAdapter S3/S4 emergency stop is latched")
        if self.error:
            raise RuntimeError("RasAdapter S3/S4 fault is latched: " + self.error)
        if self.closed:
            raise RuntimeError("RasAdapter executor is closed")
        if require_open and self.run and not self.opened:
            raise RuntimeError("RasAdapter executor is not open")

    def _send_neutral(self):
        errors = []
        if self.run and self.opened:
            # Attempt both independently. A failed steering write must not
            # prevent ESC neutral, and failed neutral must not prevent close.
            try:
                self.adapter.set_esc(4, self.config.esc_neutral_us)
            except Exception as exc:
                errors.append(f"ESC neutral: {exc}")
            try:
                self.adapter.set_position(3, self.config.steering_center_us)
            except Exception as exc:
                errors.append(f"steering center: {exc}")
        self.expires = None
        self.motion_until = None
        self.last_command = ("neutral_failed",) if errors else ("neutral",)
        if errors:
            raise RuntimeError("; ".join(errors))

    def _latch_fault(self, reason):
        self.error = self.error or str(reason)
        try:
            self._send_neutral()
        except Exception as exc:
            self.error += "; stop failed: " + str(exc)

    def _now(self):
        now = self.clock()
        if not _finite(now):
            raise ValueError("clock must return finite monotonic seconds")
        return now

    def _check_deadlines(self, now):
        if self.motion_until is not None and now >= self.motion_until:
            self._latch_fault("motion window expired")
        elif self.expires is not None and now >= self.expires:
            self._latch_fault("command lease expired")

    def _watchdog(self):
        while not self.stop_event.wait(.005):
            try:
                self.tick()
            except Exception as exc:
                with self.lock:
                    self._latch_fault(exc)

    def planned_pulses(self, throttle, steering):
        if not _finite(throttle) or throttle not in (0, 1):
            raise ValueError("only neutral (0) and reviewed forward (1) throttle are supported; fractional/reverse throttle is uncalibrated")
        if not _finite(steering) or not -1 <= steering <= 1:
            raise ValueError("steering must be finite and in -1..1")
        if self.config.missing():
            return {"esc_us": None, "steering_us": None, "missing": self.config.missing()}
        c = self.config
        esc = round(c.esc_neutral_us + throttle * (c.esc_forward_us - c.esc_neutral_us))
        endpoint = c.steering_right_us if steering >= 0 else c.steering_left_us
        servo = round(c.steering_center_us + abs(steering) * (endpoint - c.steering_center_us))
        return {"esc_us": esc, "steering_us": servo, "missing": []}

    def apply(self, throttle, steering, *, duration_s=0.02):
        with self.lock:
            self._require_available()
            try:
                now = self._now()
                self._check_deadlines(now)
                self._require_available()
                if not _finite(duration_s) or not 0 < duration_s <= self.config.maximum_motion_s:
                    raise ValueError("duration exceeds the bounded S3/S4 motion window")
                planned = self.planned_pulses(throttle, steering)
                if planned["missing"]:
                    self._send_neutral()
                    return {**planned, "hardware_output": False}
                if throttle == 0 and steering == 0:
                    self._send_neutral()
                    return {**planned, "hardware_output": self.run}
                deadline = now + duration_s
                self.motion_until = deadline if self.motion_until is None else min(self.motion_until, deadline)
                self.expires = min(now + self.config.command_lease_s, self.motion_until)
                # Count UART write time inside the lease. Do not issue throttle
                # after an already-expired steering write or claim stale success.
                if self.run:
                    self.adapter.set_position(3, planned["steering_us"])
                    self._check_deadlines(self._now())
                    self._require_available()
                    self.adapter.set_esc(4, planned["esc_us"])
                self._check_deadlines(self._now())
                self._require_available()
                self.last_command = ("motion", planned["esc_us"], planned["steering_us"])
                return {**planned, "hardware_output": self.run}
            except BaseException as exc:
                self._latch_fault(exc)
                raise

    def neutral(self):
        with self.lock:
            try:
                self._send_neutral()
            except BaseException as exc:
                self._latch_fault(exc)
                raise

    def emergency_stop(self):
        with self.lock:
            self.estop_latched = True
            self.neutral()

    def tick(self):
        with self.lock:
            if not self.closed:
                self._check_deadlines(self._now())

    def close(self):
        with self.lock:
            if self.closed:
                return list(self.cleanup_errors)
            self.closed = True
            self.stop_event.set()
            try:
                self._send_neutral()
            except Exception as exc:
                self.cleanup_errors.append(str(exc))
            finally:
                try:
                    if self.run and self.opened:
                        self.adapter.close()
                except Exception as exc:
                    self.cleanup_errors.append(f"adapter close: {exc}")
                finally:
                    self.opened = False
        if self.watchdog and self.watchdog is not threading.current_thread():
            self.watchdog.join(timeout=1)
            if self.watchdog.is_alive():
                self.cleanup_errors.append("watchdog did not exit")
        return list(self.cleanup_errors)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback):
        errors = self.close()
        if errors and exc_type is None:
            raise RuntimeError("RasAdapter cleanup failed: " + "; ".join(errors))
        return False
