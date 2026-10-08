"""Calibrated SI-unit motion planning and an explicitly gated S3/S4 driver.

The manual bench executor and its 0.8 s window are unchanged. This separate
driver is unavailable until measured feedback and continuous-motion acceptance
are recorded. It has an independent 250 ms lease and a finite total run limit.
"""
import math
import struct
import threading
import time

import numpy as np

from .autonomy_profile import front_extent_m, number, rear_extent_m
from .rasadapter5 import RasAdapter


class MetricMotionController:
    def __init__(self, profile):
        self.profile = profile
        self.integral = 0.0
        self.last_t = None

    def plan(self, intent, observation):
        m, vehicle = self.profile.data["motion"], self.profile.data["vehicle"]
        stopped = {"esc_us": 1500, "steering_us": m["steering_center_us"],
                   "target_speed_mps": 0, "motion_planned": False}
        if intent.get("action") == "stop":
            self.integral = 0
            self.last_t = observation.t_s
            return {**stopped, "reason": intent.get("reason", "stop")}
        missing = self.profile.missing()
        if missing:
            return {**stopped, "reason": "calibration_or_feedback_missing", "missing": missing}
        if intent.get("action") not in ("follow_lane", "follow_corridor"):
            return {**stopped, "reason": "manual_stage_has_separate_authorized_driver"}
        if not observation.source_ok or not observation.telemetry_valid or observation.telemetry_age_s is None or not 0 <= observation.telemetry_age_s <= .25 or not number(observation.speed_mps):
            return {**stopped, "reason": "fresh_measured_speed_required"}
        target = intent.get("speed_mps")
        steer = intent.get("steering_normalized")
        if not number(target) or not 0 < target <= m["max_speed_mps"] or not number(steer) or not -1 <= steer <= 1:
            raise ValueError("invalid SI-unit motion intent")
        if abs(observation.speed_mps) > m["max_speed_mps"] * 1.5:
            raise RuntimeError("measured vehicle overspeed")
        if self.last_t is not None and not 0 < observation.t_s - self.last_t <= .25:
            raise RuntimeError("speed control update gap or timestamp reversal")
        dt = .05 if self.last_t is None else observation.t_s - self.last_t
        self.last_t = observation.t_s
        table = m["speed_table"]
        if target < table[0]["speed_mps"] or target > table[-1]["speed_mps"]:
            return {**stopped, "reason": "target_outside_measured_speed_table"}
        feedforward = float(np.interp(target, [p["speed_mps"] for p in table], [p["pulse_us"] for p in table]))
        error = target - observation.speed_mps
        tentative = max(-.5, min(.5, self.integral + error * dt))
        raw = feedforward + m["speed_kp_us_per_mps"] * error + m["speed_ki_us_per_m"] * tentative
        minimum, maximum = table[0]["pulse_us"], table[-1]["pulse_us"]
        pulse = max(minimum, min(maximum, round(raw)))
        if minimum <= raw <= maximum or (raw > maximum and error < 0) or (raw < minimum and error > 0):
            self.integral = tentative
        delta = steer * m["max_steering_rad"]
        clear, reason = self._swept_clear(observation, delta, target)
        if not clear:
            return {**stopped, "reason": reason}
        endpoint = m["steering_right_us"] if steer >= 0 else m["steering_left_us"]
        servo = round(m["steering_center_us"] + abs(steer) * (endpoint - m["steering_center_us"]))
        return {"esc_us": pulse, "steering_us": servo, "target_speed_mps": target,
                "motion_planned": True, "reason": "calibrated_speed_feedback_and_swept_path"}

    def _swept_clear(self, observation, delta, speed):
        v, m = self.profile.data["vehicle"], self.profile.data["motion"]
        if not observation.lane_valid or not number(observation.lane_width_m) or not number(observation.lane_offset) or not observation.cone_monitor_valid:
            return False, "metric_lane_and_obstacle_visibility_required"
        stopping = speed * .25 + speed * speed / (2 * m["braking_deceleration_mps2"])
        horizon = max(.4, stopping + .08)
        lateral_center = observation.lane_offset * observation.lane_width_m / 2
        half_width = v["width_m"] / 2 + .08
        rear, front = -rear_extent_m(v) - .08, front_extent_m(v) + .08
        x = y = heading = 0.0
        # A short local bicycle sweep includes all corners, not merely a point
        # corridor. Still requires field validation for curved lane geometry.
        for _ in range(21):
            for dx in (-half_width, half_width):
                for dy in (rear, front):
                    cx = x + dx * math.cos(heading) + dy * math.sin(heading)
                    if abs(cx - lateral_center) > observation.lane_width_m / 2:
                        return False, "swept_vehicle_leaves_measured_lane"
            for cone in observation.cones:
                if not all(number(cone.get(k)) for k in ("lateral_m", "forward_m", "radius_m")) or cone["radius_m"] <= 0:
                    return False, "invalid_metric_obstacle"
                rx, ry = cone["lateral_m"] - x, cone["forward_m"] - y
                local_x = rx * math.cos(heading) - ry * math.sin(heading)
                local_y = rx * math.sin(heading) + ry * math.cos(heading)
                distance = math.hypot(max(0, abs(local_x) - half_width), max(rear - local_y, 0, local_y - front))
                if distance <= cone["radius_m"]:
                    return False, "swept_vehicle_hits_obstacle"
            ds = horizon / 20
            x += ds * math.sin(heading)
            y += ds * math.cos(heading)
            heading += ds * math.tan(delta) / v["wheelbase_m"]
        return True, "clear"


class AutonomousS3S4Driver:
    def __init__(self, profile, *, run=False, adapter=None, clock=time.monotonic):
        if type(run) is not bool:
            raise ValueError("run must be boolean")
        if run and profile.missing():
            raise ValueError("autonomous hardware blocked: " + ", ".join(profile.missing()))
        self.profile, self.run, self.clock = profile, run, clock
        self.adapter = adapter if run and adapter is not None else (RasAdapter(profile.data["motion"].get("port", "/dev/ttyAMA0")) if run else None)
        self.lock = threading.RLock()
        self.opened = self.closed = False
        self.fault = None
        self.deadline = self.started_at = None
        self.stop_event = threading.Event()
        self.watchdog = None
        self.cleanup_errors = []
        self.last_clock = None

    def _now(self):
        now = self.clock()
        if not number(now) or now < 0 or (self.last_clock is not None and now < self.last_clock):
            raise RuntimeError("invalid or reversed monotonic clock")
        self.last_clock = now
        return now

    def open(self):
        with self.lock:
            if self.closed or self.fault:
                raise RuntimeError("autonomous driver cannot restart after close/fault")
            if self.opened:
                return
            try:
                if self.run:
                    self.adapter.open()
                self.opened = True
                self.started_at = self._now()
                self._neutral()
                if self.run:
                    self.watchdog = threading.Thread(target=self._watch, daemon=True, name="autonomous-s3s4-lease")
                    self.watchdog.start()
            except BaseException as exc:
                self.fault = str(exc)
                self.close()
                raise

    def _write(self, channel, pulse):
        if type(pulse) is not int or (channel == 3 and not 1550 <= pulse <= 1750) or (channel == 4 and not 1500 <= pulse <= 1575) or channel not in (3, 4):
            raise ValueError("pulse outside reviewed S3/S4 envelope")
        if not self.run:
            return
        self.adapter.send(4, struct.pack("<BHBBH", 1, 20, 1, channel, pulse))

    def _neutral(self):
        self.deadline = None
        errors = []
        if self.opened:
            for channel, pulse in ((4, 1500), (3, self.profile.data["motion"]["steering_center_us"])):
                try:
                    self._write(channel, pulse)
                except Exception as exc:
                    errors.append(str(exc))
        if errors:
            raise RuntimeError("neutral failed: " + "; ".join(errors))

    def trip(self, reason):
        with self.lock:
            self.fault = self.fault or reason
            try:
                self._neutral()
            except Exception as exc:
                self.cleanup_errors.append(str(exc))

    def tick(self):
        with self.lock:
            if self.closed or self.fault or not self.opened:
                return
            now = self._now()
            max_run = self.profile.data["motion"].get("max_run_s")
            if not number(now) or (self.deadline is not None and now >= self.deadline):
                self.trip("command_lease_expired")
            elif max_run is not None and now - self.started_at >= max_run:
                self.trip("total_autonomous_run_limit")

    def _watch(self):
        while not self.stop_event.wait(.005):
            try:
                self.tick()
            except Exception as exc:
                self.trip("watchdog_error: " + str(exc))

    def apply(self, plan):
        with self.lock:
            if self.closed or not self.opened:
                raise RuntimeError("driver is not open")
            try:
                self.tick()
                if self.fault:
                    raise RuntimeError(self.fault)
                if not isinstance(plan, dict) or type(plan.get("motion_planned")) is not bool:
                    raise ValueError("motion plan must have an explicit boolean")
                if plan["motion_planned"]:
                    esc, steering = plan.get("esc_us"), plan.get("steering_us")
                    if type(esc) is not int or type(steering) is not int or not 1500 < esc <= 1575 or not 1550 <= steering <= 1750:
                        raise ValueError("motion plan pulse outside reviewed envelope")
                    if self.profile.missing():
                        raise ValueError("cannot apply a motion plan with missing calibration")
                if not plan.get("motion_planned"):
                    self._neutral()
                else:
                    self.deadline = self._now() + .25
                    self._write(3, plan["steering_us"])
                    self.tick()
                    if self.fault:
                        raise RuntimeError(self.fault)
                    self._write(4, plan["esc_us"])
                    self.tick()
                    if self.fault:
                        raise RuntimeError(self.fault)
                return {"hardware_output": self.run, "motion_output": self.run and bool(plan.get("motion_planned")), "plan": plan, "fault": self.fault}
            except BaseException as exc:
                self.trip(str(exc))
                raise

    def close(self):
        with self.lock:
            if self.closed:
                return list(self.cleanup_errors)
            self.stop_event.set()
            self.trip("closed")
            try:
                if self.run and self.adapter is not None:
                    self.adapter.close()
            except Exception as exc:
                self.cleanup_errors.append("close: " + str(exc))
            self.opened = False
            self.closed = True
        if self.watchdog and self.watchdog is not threading.current_thread():
            self.watchdog.join(timeout=1)
            if self.watchdog.is_alive():
                self.cleanup_errors.append("watchdog did not exit")
        return list(self.cleanup_errors)
