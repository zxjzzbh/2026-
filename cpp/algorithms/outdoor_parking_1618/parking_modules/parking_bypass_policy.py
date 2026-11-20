#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hardware-free low-speed bypass policy (Python 3.7+).

This policy confirms an empty-side visual corridor and alignment. It cannot
verify that the rear of the vehicle has passed the board or that all wheels
are in the bay. All timestamps must use the same monotonic clock.
"""

import math


DEFAULT_CONFIG = {
    "stop_duty": 12000,
    "low_duty": 12480,
    "brake_duty": 11840,
    "brake_seconds": 0.20,
    "steer_center_us": 1750,
    "steer_sign": -1,
    "steer_min_us": 1500,
    "steer_max_us": 2000,
    "max_turn_us": 90,
    "steer_slew_us_s": 180,
    "kp_us": 500,
    "front_x": 0.47,
    "target_tolerance": 0.06,
    "heading_tolerance": 0.08,
    "confirm_frames": 3,
    "align_confirm_seconds": 0.30,
    "creep_seconds": 3.0,
    "align_timeout_seconds": 8.0,
    "max_seconds": 12.0,
    "max_observation_age": 0.15,
}


def _finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


class BypassTrialPolicy:
    """Return actuator requests; never import, initialize or access hardware.

    ``update(now, observation)`` accepts frame_id, captured_at, valid,
    observation_ready, blocked, open_side, target_x, clear_of_board, and
    optional heading_error/entrance_y. The perception producer must set
    clear_of_board only when the ground-separated upright board silhouette,
    including its horizontal margin, is outside the front image window and
    the selected empty-side corridor is geometrically valid. This image
    proxy does not establish physical vehicle clearance.
    heading_error is a normalized deviation from a straight-ahead reference.
    It gates the timer; this trial does not assume its steering calibration.

    Forward speed ramping belongs to the actuator adapter, which must apply
    stop/brake requests immediately instead of slewing down through forward.
    max_seconds=0 and align_timeout_seconds=0 disable their respective trial
    time limits. Freshness checks and the post-alignment stop remain active.
    """

    def __init__(self, config_dict=None):
        supplied = {} if config_dict is None else dict(config_dict)
        unknown = set(supplied) - set(DEFAULT_CONFIG)
        if unknown:
            raise ValueError("Unknown bypass config: " + ", ".join(sorted(unknown)))
        self.cfg = dict(DEFAULT_CONFIG)
        self.cfg.update(supplied)
        self._validate_config()
        self.reset()

    def _validate_config(self):
        cfg = self.cfg
        for key, value in cfg.items():
            if not _finite(value):
                raise ValueError("{} must be finite and numeric".format(key))
        if not (0 <= cfg["brake_duty"] < cfg["stop_duty"] < cfg["low_duty"] <= 20000):
            raise ValueError("Expected brake_duty < stop_duty < low_duty (0..20000)")
        if not (500 <= cfg["steer_min_us"] < cfg["steer_center_us"]
                < cfg["steer_max_us"] <= 2500):
            raise ValueError("Invalid servo center or limits")
        if cfg["steer_sign"] not in (-1, 1):
            raise ValueError("steer_sign must be -1 or 1")
        for key in ("brake_seconds", "max_turn_us", "steer_slew_us_s", "kp_us",
                    "target_tolerance", "heading_tolerance", "align_confirm_seconds",
                    "creep_seconds", "max_observation_age"):
            if cfg[key] <= 0:
                raise ValueError("{} must be positive".format(key))
        for key in ("align_timeout_seconds", "max_seconds"):
            if cfg[key] < 0:
                raise ValueError("{} must be nonnegative; zero disables the limit".format(key))
        available_turn = min(cfg["steer_center_us"] - cfg["steer_min_us"],
                             cfg["steer_max_us"] - cfg["steer_center_us"])
        if cfg["max_turn_us"] > available_turn:
            raise ValueError("max_turn_us exceeds servo range")
        if not 0 < cfg["front_x"] < 1:
            raise ValueError("front_x must be normalized inside (0, 1)")
        if (cfg["target_tolerance"] >= min(cfg["front_x"], 1 - cfg["front_x"])
                or cfg["heading_tolerance"] > 1):
            raise ValueError("Invalid normalized alignment tolerances")
        if not isinstance(cfg["confirm_frames"], int) or cfg["confirm_frames"] < 3:
            raise ValueError("confirm_frames must be an integer >= 3")
        if cfg["align_confirm_seconds"] < 0.30:
            raise ValueError("Alignment confirmation must last >= 0.30 seconds")
        if cfg["max_observation_age"] > 0.15:
            raise ValueError("Observation freshness timeout must not exceed 0.15 seconds")
        if (cfg["max_seconds"] > 0
                and ((cfg["align_timeout_seconds"] > 0
                      and cfg["align_timeout_seconds"] >= cfg["max_seconds"])
                     or cfg["creep_seconds"] >= cfg["max_seconds"])):
            raise ValueError("Invalid trial time bounds")
        if (cfg["align_timeout_seconds"] > 0
                and cfg["align_confirm_seconds"] >= cfg["align_timeout_seconds"]):
            raise ValueError("Invalid trial time bounds")

    def reset(self):
        """Reset for a separately authorized trial, with the vehicle stopped."""
        self.state = "WAIT"
        self.reason = "WAIT_FRESH_CONFIRMED_EMPTY_SIDE"
        self.selected_side = None
        self.steer = float(self.cfg["steer_center_us"])
        self.started_at = None
        self.last_now = None
        self.align_started_at = None
        self.creep_started_at = None
        self.brake_started_at = None
        self.last_frame_id = None
        self.last_captured_at = None
        self.last_fresh_at = None
        self.seen_frame_ids = set()
        self.wait_side = None
        self.wait_hits = 0
        self.align_hits = 0
        self.align_first_capture = None
        self.elapsed_at_stop = None

    def _result(self, now):
        if self.state in ("ALIGN", "TIMED_CREEP"):
            duty = self.cfg["low_duty"]
        elif self.state in ("BRAKE", "FAULT_BRAKE"):
            duty = self.cfg["brake_duty"]
        else:
            duty = self.cfg["stop_duty"]
        elapsed = 0.0
        if self.creep_started_at is not None:
            elapsed = (self.elapsed_at_stop if self.elapsed_at_stop is not None
                       else max(0.0, now - self.creep_started_at))
        return {
            "state": self.state,
            "duty": int(round(duty)),
            "steer_us": int(round(self.steer)),
            "reason": self.reason,
            "elapsed_after_align": elapsed,
            "selected_side": self.selected_side,
        }

    def _start_brake(self, now, reason, fault=True):
        self.state = "FAULT_BRAKE" if fault else "BRAKE"
        self.reason = reason
        self.brake_started_at = now
        if self.creep_started_at is not None:
            self.elapsed_at_stop = max(0.0, now - self.creep_started_at)
        return self._result(now)

    def _bad_observation(self, now, reason):
        if self.state != "WAIT":
            return self._start_brake(now, reason)
        self.wait_side = None
        self.wait_hits = 0
        self.reason = "WAIT_" + reason
        return self._result(now)

    def _check_observation(self, now, observation):
        if not isinstance(observation, dict):
            return "OBSERVATION_MISSING"
        if observation.get("observation_ready") is not True:
            return "OBSERVATION_NOT_READY"
        if observation.get("valid") is not True:
            return "GEOMETRY_LOST"
        captured_at = observation.get("captured_at")
        if not _finite(captured_at):
            return "CAPTURE_TIME_INVALID"
        if captured_at > now + 0.025:
            return "CAPTURE_TIME_IN_FUTURE"
        if now - captured_at > self.cfg["max_observation_age"]:
            return "FRAME_EXPIRED"
        frame_id = observation.get("frame_id")
        if (isinstance(frame_id, bool) or not isinstance(frame_id, (str, int))
                or (isinstance(frame_id, str) and not frame_id)):
            return "FRAME_ID_INVALID"
        if observation.get("blocked") not in ("LEFT", "RIGHT", "UNKNOWN", "BOTH"):
            return "BLOCKED_SIDE_INVALID"
        if observation["blocked"] == "BOTH":
            return "BOTH_SIDES_BLOCKED"
        side = observation.get("open_side")
        if side not in ("LEFT", "RIGHT"):
            return "EMPTY_SIDE_UNAVAILABLE"
        if observation["blocked"] == side:
            return "EMPTY_SIDE_CONFLICT"
        if self.selected_side is not None and side != self.selected_side:
            return "SELECTED_SIDE_CHANGED"
        if self.selected_side is None and observation["blocked"] == "UNKNOWN":
            return "BLOCKED_SIDE_NOT_CONFIRMED"
        target = observation.get("target_x")
        if not _finite(target) or not 0 <= target <= 1:
            return "TARGET_INVALID"
        heading = observation.get("heading_error")
        if heading is not None and (not _finite(heading) or abs(heading) > 1):
            return "HEADING_INVALID"
        # The same frame may be polled briefly, but it cannot accumulate hits
        # or conceal a stalled producer by changing its capture timestamp.
        if frame_id == self.last_frame_id:
            if captured_at != self.last_captured_at:
                return "REPEATED_FRAME_TIMESTAMP_CHANGED"
            if now - self.last_fresh_at > self.cfg["max_observation_age"]:
                return "NO_FRESH_FRAME"
            return None
        if frame_id in self.seen_frame_ids:
            return "FRAME_REPLAYED"
        if self.last_captured_at is not None and captured_at <= self.last_captured_at:
            return "CAPTURE_TIME_NOT_ADVANCING"
        if (isinstance(frame_id, int) and isinstance(self.last_frame_id, int)
                and frame_id <= self.last_frame_id):
            return "FRAME_ID_NOT_ADVANCING"
        return None

    def update(self, now, observation):
        if not _finite(now):
            raise ValueError("now must be a finite monotonic timestamp")
        if self.state in ("DONE", "HALT"):
            return self._result(now)
        if self.last_now is not None and now < self.last_now:
            if self.state in ("BRAKE", "FAULT_BRAKE"):
                # Do not re-arm a brake pulse if a caller has a clock error.
                return self._result(self.last_now)
            return self._start_brake(self.last_now, "CLOCK_MOVED_BACKWARD")
        dt = 0.0 if self.last_now is None else now - self.last_now
        self.last_now = now
        if self.started_at is None:
            self.started_at = now
        if self.state in ("BRAKE", "FAULT_BRAKE"):
            if now - self.brake_started_at >= self.cfg["brake_seconds"]:
                fault = self.state == "FAULT_BRAKE"
                self.state = "HALT" if fault else "DONE"
                if not fault:
                    self.reason = "TIMED_TRIAL_STOPPED_NOT_PARKING_VERIFIED"
                self.steer = float(self.cfg["steer_center_us"])
            return self._result(now)
        # A completed timer always requests braking before inspecting vision.
        if (self.state == "TIMED_CREEP"
                and now - self.creep_started_at >= self.cfg["creep_seconds"]):
            return self._start_brake(now, "TIMED_CREEP_COMPLETE", fault=False)
        if (self.cfg["max_seconds"] > 0
                and now - self.started_at >= self.cfg["max_seconds"]):
            return self._start_brake(now, "TRIAL_TIMEOUT")
        if (self.state == "ALIGN"
                and self.cfg["align_timeout_seconds"] > 0
                and now - self.align_started_at >= self.cfg["align_timeout_seconds"]):
            return self._start_brake(now, "ALIGN_TIMEOUT")
        problem = self._check_observation(now, observation)
        if problem is not None:
            return self._bad_observation(now, problem)
        is_new = observation["frame_id"] != self.last_frame_id
        if is_new:
            if (self.last_captured_at is not None
                    and observation["captured_at"] - self.last_captured_at
                    > self.cfg["max_observation_age"]):
                if self.state != "WAIT":
                    return self._start_brake(now, "FRESH_FRAME_GAP_TOO_LONG")
                self.wait_side = None
                self.wait_hits = 0
            self.last_frame_id = observation["frame_id"]
            self.last_captured_at = observation["captured_at"]
            self.last_fresh_at = now
            self.seen_frame_ids.add(self.last_frame_id)
        if self.state == "WAIT":
            if is_new:
                side = observation["open_side"]
                self.wait_hits = self.wait_hits + 1 if side == self.wait_side else 1
                self.wait_side = side
                if self.wait_hits >= self.cfg["confirm_frames"]:
                    self.selected_side = side
                    self.state = "ALIGN"
                    self.align_started_at = now
                    self.reason = "ALIGN_TO_SELECTED_EMPTY_SIDE"
            if self.state == "WAIT":
                return self._result(now)
        elif self.state == "ALIGN" and is_new:
            heading = observation.get("heading_error")
            aligned = (observation.get("clear_of_board") is True
                       and abs(observation["target_x"] - self.cfg["front_x"])
                       <= self.cfg["target_tolerance"]
                       and heading is not None
                       and abs(heading) <= self.cfg["heading_tolerance"])
            if aligned:
                if self.align_hits == 0:
                    self.align_first_capture = observation["captured_at"]
                self.align_hits += 1
                if (self.align_hits >= self.cfg["confirm_frames"]
                        and observation["captured_at"] - self.align_first_capture
                        >= self.cfg["align_confirm_seconds"]):
                    self.state = "TIMED_CREEP"
                    self.creep_started_at = now
                    self.reason = "VISUAL_ALIGNMENT_CONFIRMED_TIMED_CREEP"
            else:
                self.align_hits = 0
                self.align_first_capture = None
        # Continue live target correction during the timed segment. Once it
        # starts, neither small misalignment nor a new valid frame resets it.
        correction = self.cfg["kp_us"] * (observation["target_x"] - self.cfg["front_x"])
        correction = max(-self.cfg["max_turn_us"], min(self.cfg["max_turn_us"], correction))
        desired = self.cfg["steer_center_us"] + self.cfg["steer_sign"] * correction
        max_step = self.cfg["steer_slew_us_s"] * min(dt, 0.15)
        self.steer += max(-max_step, min(max_step, desired - self.steer))
        self.steer = max(self.cfg["steer_min_us"], min(self.cfg["steer_max_us"], self.steer))
        return self._result(now)
