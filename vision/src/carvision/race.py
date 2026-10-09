"""2026 draft race decisions, independent of GPIO and any STM32 wire protocol.

Inputs must come from calibrated perception, vehicle feedback and a trusted 5G
gateway. Outputs are SI-unit intentions, never servo pulses or serial frames.
Unknown measurements stop progress instead of becoming invented geometry.
"""

import json
import math
from dataclasses import asdict, dataclass, field, fields
from enum import Enum
from pathlib import Path


class Phase(str, Enum):
    REMOTE = "remote_5g"
    BOARD = "wait_blue_board"
    CROSSWALK_APPROACH = "approach_crosswalk"
    CROSSWALK_HOLD = "crosswalk_stop"
    CROSSWALK_EXIT = "follow_after_crosswalk"
    LIGHT_APPROACH = "approach_traffic_light"
    LIGHT_HOLD = "wait_green"
    CONES = "avoid_cones"
    PARKING = "enter_clear_parking_slot"
    PAYMENT = "payment_window"
    COMPLETE = "complete"
    FAILED = "failed"
    ESTOP = "emergency_stop"


@dataclass
class RaceConfig:
    school: str = ""
    team: str = ""
    crosswalk_hold_s: float = 10.0
    announcement_required: bool = True
    crosswalk_stop_distance_m: float = 0.25
    payment_window_s: float = 30.0
    max_stationary_s: float = 20.0
    confirmation_s: float = 0.3
    maximum_frame_gap_s: float = 0.6
    maximum_feedback_age_s: float = 0.5
    stopped_speed_mps: float = 0.01
    cruise_speed_mps: float = 0.25
    approach_speed_mps: float = 0.1
    steering_gain: float = 0.7
    maximum_steering_normalized: float = 0.6
    vehicle_width_m: float | None = None
    clearance_m: float = 0.08
    cone_horizon_m: float = 3.0
    parking_stop_distance_m: float = 0.05

    def validate(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if f.name in ("school", "team"):
                if not isinstance(value, str):
                    raise ValueError(f"{f.name} must be a string")
            elif f.name == "announcement_required":
                if type(value) is not bool:
                    raise ValueError("announcement_required must be boolean")
            elif value is not None:
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{f.name} must be finite and positive")
        # The draft conflicts (3 s / 10 s). The team's explicit stage profile
        # selects 3 s; duration is configuration, never a hidden fixed sleep.
        if self.crosswalk_hold_s >= self.max_stationary_s:
            raise ValueError("crosswalk hold must be below the stationary limit")
        if not 0 < self.crosswalk_stop_distance_m < 0.3:
            raise ValueError("crosswalk stopping target must be strictly inside 30 cm")
        if self.payment_window_s != 30 or self.max_stationary_s != 20:
            raise ValueError("2026 draft uses a 30-second payment window and 20-second stationary limit")
        if self.confirmation_s > self.maximum_frame_gap_s:
            raise ValueError("confirmation interval cannot exceed allowed observation gap")
        if self.approach_speed_mps > self.cruise_speed_mps or self.maximum_steering_normalized > 1:
            raise ValueError("invalid speed or normalized steering limits")
        return self

    @property
    def announcement(self):
        if not self.school.strip() or not self.team.strip():
            return None
        return f"我是{self.school.strip()}{self.team.strip()}的智能车，请为我加油！"


def load_race_config(path=None):
    data = {} if path is None else json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("race config must be an object")
    return RaceConfig(**data).validate()


@dataclass
class Observation:
    t_s: float
    source_ok: bool = False
    telemetry_valid: bool = False
    telemetry_age_s: float | None = None
    speed_mps: float | None = None
    start_line_crossed: bool = False
    in_switch_zone: bool = False
    enable_autonomy: bool = False
    emergency_stop: bool = False
    board_monitor_valid: bool = False
    start_board_state: str = "unknown"
    lane_valid: bool = False
    lane_offset: float | None = None
    lane_width_m: float | None = None
    task_monitor_valid: bool = False
    crosswalk_distance_m: float | None = None
    crosswalk_state: str = "unknown"
    announcement_done: bool = False
    traffic_zone_entered: bool = False
    traffic_stop_distance_m: float | None = None
    traffic_light_state: str = "unknown"
    cone_monitor_valid: bool = False
    cones: list[dict] = field(default_factory=list)
    passed_cone_ids: list[str] = field(default_factory=list)
    parking_zone_visible: bool = False
    parking_geometry_valid: bool = False
    parking_slots: list[dict] = field(default_factory=list)
    parking_remaining_m: float | None = None
    parked_slot_id: str | None = None
    wheels_inside: int = 0
    payment_confirmed: bool = False
    payment_amount_cents: int | None = None
    remote: dict | None = None

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ValueError("each observation must be an object")
        obs = cls(**data)
        defaults = cls(0)
        for f in fields(obs):
            value = getattr(obs, f.name)
            if isinstance(getattr(defaults, f.name), bool) and not isinstance(value, bool):
                raise ValueError(f"{f.name} must be a boolean")
        for name in ("t_s", "telemetry_age_s", "speed_mps", "lane_offset", "lane_width_m",
                     "crosswalk_distance_m", "traffic_stop_distance_m", "parking_remaining_m"):
            value = getattr(obs, name)
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)):
                raise ValueError(f"{name} must be a finite number or null")
        if obs.t_s < 0 or (obs.telemetry_age_s is not None and obs.telemetry_age_s < 0):
            raise ValueError("timestamps and feedback age cannot be negative")
        if obs.start_board_state not in ("present", "removed", "unknown"):
            raise ValueError("board state requires a monitored present/removed/unknown result")
        if obs.traffic_light_state not in ("red", "yellow", "green", "unknown"):
            raise ValueError("invalid light state")
        if obs.crosswalk_state not in ("present", "absent", "unknown"):
            raise ValueError("invalid crosswalk state")
        if type(obs.wheels_inside) is not int or not 0 <= obs.wheels_inside <= 4:
            raise ValueError("wheels_inside must be an integer from 0 to 4")
        if not isinstance(obs.passed_cone_ids, list) or any(not isinstance(v, str) or not v for v in obs.passed_cone_ids):
            raise ValueError("passed cone IDs must be nonempty stable tracking IDs")
        if not isinstance(obs.cones, list) or not isinstance(obs.parking_slots, list):
            raise ValueError("cones and parking_slots must be lists")
        if obs.remote is not None and not isinstance(obs.remote, dict):
            raise ValueError("remote must be an object or null")
        return obs


def finite_number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def free_corridor(lane_width_m, cones, config, target_lateral_m=None):
    """Select a collision-clear lateral interval for a measured vehicle width.

    This local corridor is a planning target, not a swept Ackermann trajectory.
    The execution controller still needs steering calibration and path tracking.
    """
    if not finite_number(lane_width_m) or config.vehicle_width_m is None:
        return None
    padding = config.vehicle_width_m / 2 + config.clearance_m
    left, right = -lane_width_m / 2 + padding, lane_width_m / 2 - padding
    if left >= right:
        return None
    blocked = []
    for cone in cones:
        if not isinstance(cone, dict) or not all(finite_number(cone.get(k)) for k in ("lateral_m", "forward_m", "radius_m")):
            return None
        if cone["radius_m"] <= 0:
            return None
        if -config.vehicle_width_m <= cone["forward_m"] <= config.cone_horizon_m:
            lo = max(left, cone["lateral_m"] - cone["radius_m"] - padding)
            hi = min(right, cone["lateral_m"] + cone["radius_m"] + padding)
            if lo <= hi:
                blocked.append((lo, hi))
    gaps, cursor = [], left
    for lo, hi in sorted(blocked):
        if lo > cursor:
            gaps.append((cursor, lo))
        cursor = max(cursor, hi)
    if cursor < right:
        gaps.append((cursor, right))
    gaps = [g for g in gaps if g[1] - g[0] > 0.02]
    if not gaps:
        return None
    if target_lateral_m is not None:
        if finite_number(target_lateral_m) and any(lo < target_lateral_m < hi for lo, hi in gaps):
            return target_lateral_m
        return None
    candidates = [0.0 if lo < 0 < hi else (lo + hi) / 2 for lo, hi in gaps]
    return min(candidates, key=abs)


class RaceController:
    def __init__(self, config, *, scope="full_race"):
        if scope not in ("full_race", "crosswalk"):
            raise ValueError("unknown task scope")
        self.config = config.validate()
        self.scope = scope
        self.phase = Phase.REMOTE if scope == "full_race" else Phase.CROSSWALK_APPROACH
        self.last_t = None
        self.race_started = None
        self.race_finished = None
        self.stationary_since = None
        self.board_seen = False
        self.board_clear_since = None
        self.crosswalk_since = None
        self.crosswalk_seen = False
        self.crosswalk_served = False
        self.announcement_requested = False
        self.announcement_acknowledged = False
        self.green_since = None
        self.light_stopped_since = None
        self.light_wait_s = 0.0
        self.passed_cones = set()
        self.parking_slot = None
        self.parked_at = None
        self.payment_succeeded = False
        self.terminal_reason = None
        self.events = []

    def transition(self, phase, obs):
        self.events.append({"event": "phase_changed", "from": self.phase.value, "to": phase.value, "t_s": obs.t_s})
        self.phase = phase

    def fail(self, reason, obs):
        self.terminal_reason = reason
        self.transition(Phase.FAILED, obs)
        return self.decision("stop", reason)

    def decision(self, action, reason, speed=0.0, steering=0.0, lateral=None):
        c = self.config
        elapsed = (c.crosswalk_hold_s if self.crosswalk_served else
                   max(0.0, self.last_t - self.crosswalk_since) if self.crosswalk_since is not None else 0.0)
        return {"schema_version": "race-intent-0.2", "mode": "decision_only",
                "hardware_output": False, "phase": self.phase.value, "action": action,
                "reason": reason, "speed_mps": min(c.cruise_speed_mps, max(0.0, speed)),
                "steering_normalized": max(-c.maximum_steering_normalized, min(c.maximum_steering_normalized, steering)),
                "target_lateral_m": lateral, "parking_slot_id": self.parking_slot,
                "payment_remaining_s": None if self.parked_at is None else max(0.0, c.payment_window_s - (self.last_t - self.parked_at)),
                "crosswalk_seen": self.crosswalk_seen, "crosswalk_served": self.crosswalk_served,
                "crosswalk_hold_s": c.crosswalk_hold_s,
                "crosswalk_elapsed_s": min(c.crosswalk_hold_s, elapsed),
                "crosswalk_remaining_s": max(0.0, c.crosswalk_hold_s - elapsed),
                "events": list(self.events)}

    def follow(self, obs, reason, speed=None, lateral=None):
        if not obs.lane_valid or not finite_number(obs.lane_offset) or abs(obs.lane_offset) > 1:
            return self.decision("stop", "lane_geometry_unknown")
        offset = obs.lane_offset
        if lateral is not None:
            if not finite_number(obs.lane_width_m) or obs.lane_width_m <= 0:
                return self.decision("stop", "metric_lane_width_unknown")
            offset += 2 * lateral / obs.lane_width_m
        return self.decision("follow_corridor" if lateral is not None else "follow_lane", reason,
                             self.config.cruise_speed_mps if speed is None else speed,
                             offset * self.config.steering_gain, lateral)

    def update(self, obs):
        # Validate direct Python callers as well as JSONL input.
        obs = Observation.from_dict(asdict(obs))
        self.events = []
        if self.last_t is not None and obs.t_s <= self.last_t:
            self.terminal_reason = "non_increasing_timestamp"
            self.transition(Phase.FAILED, obs)
            return self.decision("stop", self.terminal_reason)
        gap = self.last_t is not None and obs.t_s - self.last_t > self.config.maximum_frame_gap_s
        self.last_t = obs.t_s
        if obs.emergency_stop:
            self.terminal_reason = "emergency_stop_latched"
            self.transition(Phase.ESTOP, obs)
        if self.phase in (Phase.ESTOP, Phase.FAILED, Phase.COMPLETE):
            return self.decision("stop", self.terminal_reason or self.phase.value)

        # Keep valid visual evidence even if speed feedback is temporarily lost.
        # Losing an already approached target must never restore cruise speed.
        if (self.phase == Phase.CROSSWALK_APPROACH and obs.source_ok and obs.task_monitor_valid
                and (obs.crosswalk_state == "present" or obs.crosswalk_distance_m is not None)):
            self.crosswalk_seen = True

        # Parking ends the competition clock; payment is a separate human task.
        if self.phase == Phase.PAYMENT:
            elapsed = obs.t_s - self.parked_at
            if elapsed <= self.config.payment_window_s and obs.payment_confirmed and type(obs.payment_amount_cents) is int and obs.payment_amount_cents == 1:
                self.payment_succeeded = True
                self.transition(Phase.COMPLETE, obs)
                self.terminal_reason = "payment_success_reported"
            elif elapsed > self.config.payment_window_s:
                self.transition(Phase.COMPLETE, obs)
                self.terminal_reason = "payment_window_expired_no_bonus"
            return self.decision("stop", self.terminal_reason or "await_remote_wechat_payment_1_cent")

        fresh = (obs.source_ok and obs.telemetry_valid and finite_number(obs.speed_mps)
                 and finite_number(obs.telemetry_age_s) and 0 <= obs.telemetry_age_s <= self.config.maximum_feedback_age_s)
        if not fresh or gap:
            self.crosswalk_since = self.green_since = self.board_clear_since = None
            self.light_stopped_since = None
            return self.decision("stop", "observation_gap" if gap else "vehicle_feedback_or_source_unknown")
        stopped = abs(obs.speed_mps) <= self.config.stopped_speed_mps
        if obs.start_line_crossed and self.race_started is None:
            self.race_started = obs.t_s
        if self.race_started is not None:
            if stopped:
                if self.stationary_since is None:
                    self.stationary_since = obs.t_s
                elif obs.t_s - self.stationary_since > self.config.max_stationary_s:
                    return self.fail("stationary_over_20_seconds", obs)
            else:
                self.stationary_since = None

        if self.phase != Phase.REMOTE and obs.remote is not None:
            self.events.append({"event": "remote_driving_ignored_in_autonomous_phase", "t_s": obs.t_s})

        if self.phase == Phase.REMOTE:
            if obs.enable_autonomy:
                if not obs.in_switch_zone or not stopped:
                    return self.decision("stop", "handover_requires_stopped_vehicle_inside_switch_zone")
                if self.config.announcement_required and self.config.announcement is None:
                    return self.decision("stop", "school_and_team_required_before_autonomy")
                self.board_seen = obs.board_monitor_valid and obs.start_board_state == "present"
                self.transition(Phase.BOARD, obs)
                return self.decision("stop", "autonomy_enabled_wait_for_blue_board")
            cmd = obs.remote
            if not cmd or cmd.get("authenticated") is not True or cmd.get("verified_5g") is not True or cmd.get("deadman") is not True:
                return self.decision("stop", "fresh_authenticated_5g_gateway_command_required")
            age = cmd.get("command_age_s")
            speed, steering = cmd.get("speed_mps"), cmd.get("steering_normalized")
            if not finite_number(age) or not 0 <= age <= self.config.maximum_feedback_age_s or not finite_number(speed) or not finite_number(steering):
                return self.decision("stop", "remote_command_expired_or_invalid")
            return self.decision("remote_intent", "gateway_5g_attestation_required", speed, steering)

        if self.phase == Phase.BOARD:
            if not stopped:
                self.board_clear_since = None
                return self.decision("stop", "must_remain_stopped_during_board_monitoring")
            if not obs.board_monitor_valid or obs.start_board_state == "unknown":
                self.board_clear_since = None
                return self.decision("stop", "board_removal_not_confirmed")
            if obs.start_board_state == "present":
                self.board_seen = True
                self.board_clear_since = None
            elif self.board_seen:
                if self.board_clear_since is None:
                    self.board_clear_since = obs.t_s
                if obs.t_s - self.board_clear_since >= self.config.confirmation_s:
                    self.transition(Phase.CROSSWALK_APPROACH, obs)
                    return self.decision("stop", "board_removal_confirmed")
            return self.decision("stop", "await_observed_blue_board_removal")

        if self.phase in (Phase.CROSSWALK_APPROACH, Phase.CROSSWALK_HOLD):
            if not obs.task_monitor_valid:
                self.crosswalk_since = None
                return self.decision("stop", "calibrated_crosswalk_monitor_required")
            distance = obs.crosswalk_distance_m
            if distance is not None and distance < 0:
                return self.fail("front_bumper_crossed_crosswalk_edge", obs)
            if self.phase == Phase.CROSSWALK_APPROACH:
                if distance is None and self.crosswalk_seen:
                    return self.decision("stop", "crosswalk_distance_lost_after_detection")
                if distance is not None and distance <= self.config.crosswalk_stop_distance_m:
                    self.transition(Phase.CROSSWALK_HOLD, obs)
                else:
                    return self.follow(obs, "seek_crosswalk", self.config.approach_speed_mps if distance is not None and distance < 1 else None)
            if distance is None or not 0 <= distance < 0.3 or not stopped:
                self.crosswalk_since = None
                return self.decision("stop", "need_stationary_feedback_inside_crosswalk_stop_zone")
            if self.crosswalk_since is None:
                self.crosswalk_since = obs.t_s
                self.events.append({"event": "crosswalk_stationary_hold_started", "t_s": obs.t_s})
            if self.config.announcement_required and not self.announcement_requested:
                self.events.append({"event": "play_announcement", "text": self.config.announcement, "t_s": obs.t_s})
                self.announcement_requested = True
            self.announcement_acknowledged |= obs.announcement_done
            if (obs.t_s - self.crosswalk_since + 1e-9 >= self.config.crosswalk_hold_s
                    and (not self.config.announcement_required or self.announcement_acknowledged)):
                self.crosswalk_served = True
                self.events.append({"event": "crosswalk_completed", "hold_s": self.config.crosswalk_hold_s, "t_s": obs.t_s})
                self.transition(Phase.CROSSWALK_EXIT if self.scope == "crosswalk" else Phase.LIGHT_APPROACH, obs)
                return self.decision("stop", "crosswalk_configured_hold_completed")
            return self.decision("stop", "crosswalk_wait_for_full_stop_duration_and_audio_ack")

        if self.phase == Phase.CROSSWALK_EXIT:
            # One crosswalk per stage run. Its remaining visible stripes must
            # not initiate a second stop. Source, speed and lane checks remain.
            return self.follow(obs, "crosswalk_completed_resume_lane")

        if self.phase == Phase.LIGHT_APPROACH:
            if not obs.task_monitor_valid:
                return self.decision("stop", "calibrated_traffic_stop_zone_required")
            if obs.traffic_zone_entered:
                self.transition(Phase.LIGHT_HOLD, obs)
            else:
                return self.follow(obs, "approach_traffic_sensor", self.config.approach_speed_mps)
        if self.phase == Phase.LIGHT_HOLD:
            distance = obs.traffic_stop_distance_m
            if distance is not None and distance < 0:
                return self.fail("front_wheels_crossed_traffic_stop_zone", obs)
            if not obs.task_monitor_valid or not obs.traffic_zone_entered or distance is None or not 0 <= distance <= 1 or not stopped:
                self.green_since = self.light_stopped_since = None
                return self.decision("stop", "need_stationary_feedback_between_sensor_and_traffic_light")
            if self.light_stopped_since is None:
                self.light_stopped_since = obs.t_s
            if obs.traffic_light_state != "green":
                self.green_since = None
            else:
                if self.green_since is None:
                    self.green_since = obs.t_s
                if obs.t_s - self.green_since >= self.config.confirmation_s:
                    self.light_wait_s += obs.t_s - self.light_stopped_since
                    self.transition(Phase.CONES, obs)
                    return self.decision("stop", "observed_green_after_stationary_traffic_stop")
            return self.decision("stop", "wait_for_confirmed_green_never_a_fixed_countdown")

        if self.phase == Phase.CONES:
            if not obs.cone_monitor_valid:
                return self.decision("stop", "calibrated_cone_monitor_required")
            lateral = free_corridor(obs.lane_width_m, obs.cones, self.config)
            if lateral is None:
                return self.decision("stop", "no_verified_collision_clear_corridor")
            self.passed_cones.update(obs.passed_cone_ids)
            if obs.parking_zone_visible:
                if len(self.passed_cones) < 2:
                    return self.decision("stop", "two_distinct_cones_must_be_observed_and_passed")
                self.transition(Phase.PARKING, obs)
            else:
                return self.follow(obs, "avoid_observed_random_cones", self.config.approach_speed_mps, lateral)

        if self.phase == Phase.PARKING:
            if not obs.parking_geometry_valid or not obs.cone_monitor_valid:
                return self.decision("stop", "parking_geometry_or_obstacle_visibility_unknown")
            slots = obs.parking_slots
            if len(slots) != 2 or any(not isinstance(s, dict) or not isinstance(s.get("id"), str) or not s["id"] or not finite_number(s.get("center_lateral_m")) for s in slots):
                return self.decision("stop", "two_measured_parking_slots_required")
            if len({s["id"] for s in slots}) != 2:
                return self.decision("stop", "parking_slot_ids_must_be_distinct")
            clear = [s for s in slots if s.get("availability") == "clear"]
            blocked = [s for s in slots if s.get("availability") == "blocked"]
            if len(clear) != 1 or len(blocked) != 1:
                return self.decision("stop", "need_one_verified_clear_slot_and_one_blocked_slot")
            chosen = clear[0]
            if self.parking_slot is None:
                self.parking_slot = chosen["id"]
            if self.parking_slot != chosen["id"]:
                return self.decision("stop", "selected_slot_became_blocked")
            if free_corridor(obs.lane_width_m, obs.cones, self.config, chosen["center_lateral_m"]) is None:
                return self.decision("stop", "parking_approach_obstructed")
            if obs.parked_slot_id == self.parking_slot and obs.wheels_inside == 4 and stopped:
                self.parked_at = self.race_finished = obs.t_s
                self.transition(Phase.PAYMENT, obs)
                self.events.append({"event": "request_remote_wechat_payment", "amount_cents": 1, "deadline_t_s": obs.t_s + self.config.payment_window_s})
                return self.decision("stop", "four_wheels_inside_clear_slot_race_clock_stopped")
            if obs.parking_remaining_m is None or obs.parking_remaining_m <= self.config.parking_stop_distance_m:
                return self.decision("stop", "await_four_wheel_parking_confirmation")
            return self.follow(obs, "enter_selected_clear_slot", self.config.approach_speed_mps, chosen["center_lateral_m"])
        return self.decision("stop", "unhandled_state")

    def summary(self):
        elapsed = None if self.race_started is None or self.race_finished is None else self.race_finished - self.race_started
        return {"phase": self.phase.value, "hardware_output": False,
                "race_elapsed_s": elapsed, "verified_traffic_wait_s": self.light_wait_s,
                "effective_time_before_penalties_s": None if elapsed is None else max(0, elapsed - self.light_wait_s),
                "passed_cone_ids": sorted(self.passed_cones), "parking_slot_id": self.parking_slot,
                "payment_success_reported": self.payment_succeeded,
                "payment_bonus_points": 5 if self.payment_succeeded else 0,
                "reason": self.terminal_reason}
