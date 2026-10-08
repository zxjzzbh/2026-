"""Offline-safe bridge from race intents to the RasAdapter S3/S4 executor.

The current reviewed executor supports only neutral and one reviewed forward
point. Continuous speed control, reverse sequencing, and metric closed-loop
motion remain deliberately unavailable.
"""

from dataclasses import dataclass
import threading

from .race import finite_number
from .rasadapter5_executor import RasAdapterExecutionConfig, RasAdapterS3S4Executor


@dataclass(frozen=True)
class ExecutionResult:
    action: str
    reason: str
    actuator: dict


class RaceExecutorBridge:
    """Stop/refuse race intents without inventing calibration or feedback.

    This bridge never turns a positive target speed into the discrete forward
    point. Live observation freshness and EOF/timeout detection still belong
    to the future runtime; its failure paths must call the latch methods below.
    """

    def __init__(self, executor=None):
        self.executor = executor if executor is not None else RasAdapterS3S4Executor(
            RasAdapterExecutionConfig())
        self.locked = False
        self.lock_reason = None
        self.error = None
        self.closed = False
        self.lock = threading.RLock()

    def open(self):
        with self.lock:
            if self.closed or self.locked:
                raise RuntimeError("race executor bridge is closed or locked")
            try:
                self.executor.open()
            except BaseException as exc:
                self._trip("executor_open_failed", exc)
                raise

    def _result(self, reason):
        # A real-mode neutral command is still hardware output. It must not be
        # hidden by labeling every rejected motion as a hardware-free result.
        return ExecutionResult("stop", reason, {
            "hardware_output": self.executor.run and self.executor.opened,
            "motion_output": False,
        }).__dict__

    def _stop(self, reason):
        self.executor.neutral()
        return self._result(reason)

    def _trip(self, reason, error=None):
        self.locked = True
        self.lock_reason = self.lock_reason or reason
        if error is not None:
            self.error = str(error)
        try:
            self.executor.emergency_stop()
        except BaseException as exc:
            self.error = (self.error + "; " if self.error else "") + f"stop failed: {exc}"
            if error is None:
                raise

    def apply(self, intent):
        with self.lock:
            if self.closed:
                raise RuntimeError("race executor bridge is closed")
            try:
                if self.locked or self.executor.estop_latched or self.executor.error:
                    self.locked = True
                    return self._stop("executor_locked")
                if not isinstance(intent, dict):
                    raise ValueError("race intent must be an object")
                action = intent.get("action")
                if not isinstance(action, str) or not action:
                    raise ValueError("race action must be a nonempty string")
                reason = intent.get("reason", "stop_intent")
                if not isinstance(reason, str):
                    raise ValueError("race reason must be a string")
                if intent.get("phase") in ("emergency_stop", "failed", "complete") or reason == "emergency_stop_latched":
                    self._trip(reason)
                    return self._result(reason)
                if action == "stop":
                    return self._stop(reason)
                if action not in {"follow_lane", "follow_corridor"}:
                    return self._stop("unsupported_race_action")
                speed = intent.get("speed_mps")
                if not finite_number(speed) or speed <= 0:
                    self._trip("invalid_positive_speed")
                    return self._result("invalid_positive_speed")
                return self._stop("continuous_speed_mapping_not_calibrated")
            except BaseException as exc:
                self._trip("invalid_intent_or_executor_error", exc)
                raise

    def emergency_stop(self):
        with self.lock:
            self._trip("emergency_stop_latched")

    def input_timeout(self):
        """Runtime notification; this method does not detect input age itself."""
        with self.lock:
            self._trip("live_input_timeout")
            return self._result("live_input_timeout")

    def input_ended(self):
        with self.lock:
            self._trip("input_ended")
            return self._result("input_ended")

    def input_error(self):
        with self.lock:
            self._trip("input_error")
            return self._result("input_error")

    def stop_requested(self):
        with self.lock:
            self._trip("stop_requested")
            return self._result("stop_requested")

    def close(self):
        with self.lock:
            self.closed = True
            return self.executor.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback):
        errors = self.close()
        if errors and exc_type is None:
            raise RuntimeError("race executor cleanup failed: " + "; ".join(errors))
        return False
