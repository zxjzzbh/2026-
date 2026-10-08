"""Race trace replay and marked synthetic demonstrations; no device writes."""

import json
import sys
from dataclasses import asdict, replace
from pathlib import Path

from .race import Observation, RaceController, load_race_config


def demonstration():
    """Full synthetic feedback, including actual-stop and audio acknowledgments."""
    for tick in range(221):
        t = round(tick / 10, 1)
        o = Observation(t, source_ok=True, telemetry_valid=True, telemetry_age_s=0,
                        speed_mps=0.1, lane_valid=True, lane_offset=0, lane_width_m=1.22,
                        task_monitor_valid=True, cone_monitor_valid=True)
        if t < 1:
            o.start_line_crossed = t == 0
            o.remote = {"authenticated": True, "verified_5g": True, "deadman": True,
                        "command_age_s": 0, "speed_mps": 0.1, "steering_normalized": 0}
        elif t <= 1.6:
            o.speed_mps = 0
            o.in_switch_zone = True
            o.enable_autonomy = t == 1
            o.board_monitor_valid = True
            o.start_board_state = "present" if t <= 1.1 else "removed"
        if 2 <= t <= 12.5:
            o.crosswalk_distance_m = 0.2
            o.speed_mps = 0 if t >= 2.1 else 0.1
            o.announcement_done = t >= 2.5
        if 13 <= t <= 15.4:
            o.traffic_zone_entered = True
            o.traffic_stop_distance_m = 0.6
            o.traffic_light_state = "green" if t >= 15 else "red"
            o.speed_mps = 0 if t >= 13.1 else 0.1
        if 15.5 <= t < 16.5:
            o.cones = [{"lateral_m": -0.15, "forward_m": 0.8, "radius_m": 0.039},
                       {"lateral_m": -0.1, "forward_m": 1.5, "radius_m": 0.039}]
        if t >= 16.5:
            o.passed_cone_ids = ["cone-1"]
        if t >= 18:
            o.passed_cone_ids = ["cone-1", "cone-2"]
            o.parking_zone_visible = o.parking_geometry_valid = True
            o.parking_slots = [{"id": "left", "center_lateral_m": -0.3, "availability": "blocked"},
                               {"id": "right", "center_lateral_m": 0.3, "availability": "clear"}]
            o.parking_remaining_m = max(0, round((20 - t) * 0.1, 3))
        if t >= 20:
            o.speed_mps = 0
            o.parked_slot_id = "right"
            o.wheels_inside = 4
        if t >= 21:
            o.payment_confirmed = True
            o.payment_amount_cents = 1
        yield o


def json_line(stream, data):
    stream.write(json.dumps(data, ensure_ascii=False, allow_nan=False) + "\n")
    stream.flush()


def run_trace(observations, config, output, simulated=False):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    controller = RaceController(config)
    metadata = {"simulated": simulated, "hardware_output": False, "config": asdict(config)}
    (output / "config.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    count = 0
    try:
        with (output / "decisions.jsonl").open("w", encoding="utf-8") as log:
            for row in observations:
                observation = row if isinstance(row, Observation) else Observation.from_dict(row)
                decision = controller.update(observation)
                json_line(log, {"t_s": observation.t_s, "simulated": simulated, **decision})
                count += 1
            json_line(log, {"t_s": controller.last_t, "simulated": simulated,
                            **controller.decision("stop", "input_exhausted")})
        summary = {**controller.summary(), "simulated": simulated, "observations": count, "output": str(output)}
    except BaseException as exc:
        with (output / "decisions.jsonl").open("a", encoding="utf-8") as log:
            json_line(log, {"t_s": controller.last_t, "simulated": simulated,
                            **controller.decision("stop", "trace_input_error_or_interruption")})
        summary = {**controller.summary(), "simulated": simulated, "observations": count,
                   "error": f"{type(exc).__name__}: {exc}"}
        (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        raise
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def race_demo(args):
    config = load_race_config(args.race_config)
    config = replace(config, school=config.school or "模拟学校", team=config.team or "模拟队",
                     vehicle_width_m=config.vehicle_width_m or 0.19)
    rows = list(demonstration())
    summary = run_trace(rows, config, args.output, simulated=True)
    (Path(args.output) / "config-for-replay.json").write_text(
        json.dumps(asdict(config), ensure_ascii=False, indent=2), encoding="utf-8")
    with (Path(args.output) / "observations.jsonl").open("w", encoding="utf-8") as out:
        for row in rows:
            json_line(out, asdict(row))
    return summary


def read_observations(path):
    stream = sys.stdin if path == "-" else Path(path).open(encoding="utf-8-sig")
    try:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                yield Observation.from_dict(json.loads(line))
            except (ValueError, TypeError) as exc:
                raise ValueError(f"invalid observation on line {line_number}: {exc}") from exc
    finally:
        if stream is not sys.stdin:
            stream.close()


def race_plan(args):
    return run_trace(read_observations(args.input), load_race_config(args.race_config), args.output)


def race_check(args):
    config = load_race_config(args.race_config)
    missing = ["camera_to_ground_calibration_and_wheel_feedback",
               "validated_race_element_detection_and_tracking",
               "Raspberry_Pi_PWM_pin_configuration_and_speed_feedback",
               "authenticated_actual_5g_gateway",
               "recorded_announcement_and_playback_acknowledgment"]
    if config.announcement is None:
        missing.append("school_and_team")
    if config.vehicle_width_m is None:
        missing.append("measured_vehicle_width")
    return {"rules": "2026-08 draft; crosswalk 10s follows clauses 2.2.7 and 2.4.2",
            "hardware_output": False, "real_vehicle_ready": False,
            "announcement": config.announcement, "remaining_integration": missing,
            "crosswalk_hold_s": config.crosswalk_hold_s, "payment_window_s": config.payment_window_s,
            "max_stationary_s": config.max_stationary_s}
