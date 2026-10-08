"""Measured configuration for the autonomous runtime; unknown stays unknown.

Ground coordinates are metres relative to the rear axle: x right, y forward.
Camera mappings apply only to the recorded resolution and fixed servo pose.
No sample/default configuration grants permission to move hardware.
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def positive(value):
    return number(value) and value > 0


def front_extent_m(vehicle):
    """Prefer the directly surveyed rear-axle-to-frontmost-point distance."""
    direct = vehicle.get("front_to_rear_axle_m")
    if direct is not None:
        return direct if positive(direct) else None
    wheelbase, overhang = vehicle.get("wheelbase_m"), vehicle.get("front_overhang_m")
    return wheelbase + overhang if positive(wheelbase) and positive(overhang) else None


def rear_extent_m(vehicle):
    """Retain the larger reported rear extent when component lengths differ."""
    extents = [vehicle["rear_overhang_m"]] if positive(vehicle.get("rear_overhang_m")) else []
    length, front = vehicle.get("length_m"), front_extent_m(vehicle)
    if positive(length) and positive(front) and length > front:
        extents.append(length - front)
    return max(extents) if extents else None


class AutonomyProfile:
    def __init__(self, data):
        if not isinstance(data, dict) or data.get("schema_version") != 1:
            raise ValueError("autonomy profile requires schema_version=1")
        self.data = data
        for section in ("vehicle", "motion", "telemetry", "cameras", "audio"):
            if not isinstance(data.get(section), dict):
                raise ValueError(f"profile.{section} must be an object")
        for section, flags in {
            "vehicle": ("measured",), "motion": ("calibrated", "continuous_motion_verified"),
            "telemetry": ("verified", "pose_verified"), "audio": ("verified",),
        }.items():
            for flag in flags:
                if type(data[section].get(flag)) is not bool:
                    raise ValueError(f"{section}.{flag} must be boolean")
        motion = data["motion"]
        if data["telemetry"].get("source") not in ("encoder", "optical_ground"):
            raise ValueError("telemetry source must be encoder or optical_ground")
        if motion.get("esc_neutral_us") != 1500:
            raise ValueError("S4 neutral must remain the reviewed 1500 us")
        for name in ("steering_left_us", "steering_center_us", "steering_right_us"):
            pulse = motion.get(name)
            if pulse is not None and (type(pulse) is not int or not 1550 <= pulse <= 1750):
                raise ValueError(f"motion.{name} outside observed S3 range")
        pulses = [motion.get(n) for n in ("steering_left_us", "steering_center_us", "steering_right_us")]
        if all(p is not None for p in pulses) and not min(pulses[0], pulses[2]) < pulses[1] < max(pulses[0], pulses[2]):
            raise ValueError("steering center must be between endpoints")
        for section, names in {
            "vehicle": ("width_m", "wheelbase_m", "track_width_m", "front_overhang_m", "rear_overhang_m", "tyre_width_m", "tyre_length_m", "front_to_rear_axle_m", "length_m"),
            "motion": ("max_speed_mps", "max_steering_rad", "speed_kp_us_per_mps", "speed_ki_us_per_m", "max_run_s", "braking_deceleration_mps2"),
            "telemetry": ("distance_per_count_m", "maximum_speed_mps", "visual_error_bound_m"),
        }.items():
            for name in names:
                value = data[section].get(name)
                if value is not None and not positive(value):
                    raise ValueError(f"{section}.{name} must be positive or null")
        if motion.get("max_speed_mps") is not None and motion["max_speed_mps"] > .25:
            raise ValueError("initial autonomous speed limit is 0.25 m/s")
        if motion.get("max_run_s") is not None and motion["max_run_s"] > 180:
            raise ValueError("autonomous run must be bounded to at most 180 s")
        if motion.get("max_steering_rad") is not None and motion["max_steering_rad"] > .6:
            raise ValueError("initial steering angle limit is 0.6 rad")
        vehicle = data["vehicle"]
        if vehicle.get("front_to_rear_axle_m") is not None and vehicle.get("wheelbase_m") is not None and vehicle["front_to_rear_axle_m"] < vehicle["wheelbase_m"]:
            raise ValueError("measured front extent cannot be behind the front axle")
        if vehicle.get("length_m") is not None and front_extent_m(vehicle) is not None and vehicle["length_m"] <= front_extent_m(vehicle):
            raise ValueError("vehicle length must extend behind the rear axle")
        if all(vehicle.get(n) is not None for n in ("width_m", "track_width_m", "tyre_width_m")) and vehicle["width_m"] < vehicle["track_width_m"] + vehicle["tyre_width_m"]:
            raise ValueError("vehicle width cannot be smaller than the tyre track footprint")
        table = motion.get("speed_table", [])
        if not isinstance(table, list):
            raise ValueError("speed_table must be a list")
        last_speed, last_pulse = 0, 1500
        for row in table:
            if not isinstance(row, dict) or not positive(row.get("speed_mps")) or type(row.get("pulse_us")) is not int:
                raise ValueError("speed table needs measured speed_mps and pulse_us")
            if row["speed_mps"] <= last_speed or not last_pulse < row["pulse_us"] <= 1575:
                raise ValueError("speed table must increase inside reviewed 1500..1575 us envelope")
            last_speed, last_pulse = row["speed_mps"], row["pulse_us"]
        for name, camera in data["cameras"].items():
            if not isinstance(camera, dict) or type(camera.get("verified")) is not bool or type(camera.get("detector_verified")) is not bool:
                raise ValueError(f"camera {name} needs explicit verification flags")
            if camera.get("homography") is not None:
                GroundProjection(camera, require_verified=False)
            region = camera.get("monitored_region_m")
            if region is not None:
                polygon = np.asarray(region, dtype=float)
                if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3 or not np.isfinite(polygon).all() or not cv2.isContourConvex(polygon.astype(np.float32)):
                    raise ValueError("monitored road region must be a finite convex ground polygon")
        course = data.get("course", {})
        if not isinstance(course, dict) or type(course.get("verified")) is not bool:
            raise ValueError("course needs an explicit verified boolean")
        for label in ("traffic_stop_line_world_m", "crosswalk_edge_world_m"):
            line = course.get(label)
            if line is not None:
                points = np.asarray(line, dtype=float)
                if points.shape != (2, 2) or not np.isfinite(points).all() or np.linalg.norm(points[0] - points[1]) < .1:
                    raise ValueError("surveyed task line needs two distinct finite metre coordinates")

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text(encoding="utf-8-sig")))

    def missing(self):
        d, missing = self.data, []
        for section, names in {
            "vehicle": ("width_m", "wheelbase_m", "track_width_m", "front_overhang_m", "rear_overhang_m", "tyre_width_m", "tyre_length_m"),
            "motion": ("max_speed_mps", "max_steering_rad", "speed_kp_us_per_mps", "speed_ki_us_per_m", "max_run_s", "braking_deceleration_mps2", "steering_left_us", "steering_center_us", "steering_right_us"),
            "telemetry": ("maximum_speed_mps",),
        }.items():
            missing.extend(f"{section}.{n}" for n in names if d[section].get(n) is None)
        if d["telemetry"]["source"] == "encoder" and d["telemetry"].get("distance_per_count_m") is None:
            missing.append("telemetry.distance_per_count_m")
        if d["telemetry"]["source"] == "optical_ground":
            if d["telemetry"].get("visual_error_bound_m") is None or d["telemetry"]["visual_error_bound_m"] > .03:
                missing.append("telemetry.visual_odometry_field_error_bound")
            origin = d["telemetry"].get("origin_pose") or {}
            if not all(number(origin.get(k)) for k in ("x_m", "y_m", "yaw_rad")):
                missing.append("telemetry.measured_visual_origin_pose")
        for section, flag in (("vehicle", "measured"), ("motion", "calibrated"), ("motion", "continuous_motion_verified"), ("telemetry", "verified"), ("telemetry", "pose_verified"), ("audio", "verified")):
            if not d[section][flag]:
                missing.append(f"{section}.{flag}")
        if len(d["motion"].get("speed_table", [])) < 2:
            missing.append("motion.speed_table_at_least_two_measured_points")
        primary = d["cameras"].get("primary", {})
        if not primary.get("verified") or primary.get("homography") is None:
            missing.append("cameras.primary.ground_calibration")
        if not primary.get("detector_verified"):
            missing.append("cameras.primary.detector_verified_on_track")
        if not isinstance(primary.get("detector_model_sha256"), str) or len(primary["detector_model_sha256"]) != 64:
            missing.append("cameras.primary.detector_model_sha256")
        if not primary.get("monitored_region_m"):
            missing.append("cameras.primary.measured_road_visibility")
        course = d.get("course", {})
        if course.get("verified") is not True or not course.get("traffic_stop_line_world_m"):
            missing.append("course.traffic_stop_line_survey")
        if d["audio"].get("backend") not in ("wav", "tts"):
            missing.append("audio.real_backend")
        if d["audio"].get("backend") == "wav" and not d["audio"].get("wav_path"):
            missing.append("audio.wav_path")
        if d["audio"].get("backend") == "wav" and not d["audio"].get("wav_sha256"):
            missing.append("audio.wav_sha256")
        if d["audio"].get("backend") == "tts" and any(type(d["audio"].get(k)) is not int for k in ("busy_status", "idle_status")):
            missing.append("audio.measured_busy_idle_status")
        return missing

    def report(self):
        missing = self.missing()
        return {"hardware_output": False, "actuation_mapping_ready": not missing,
                "missing": missing, "secondary_requires_its_own_fixed_pose_calibration": True,
                "manual_bench_limits_changed": False}


class GroundProjection:
    def __init__(self, camera, require_verified=True):
        if require_verified and not camera.get("verified"):
            raise ValueError("camera ground calibration is not verified")
        h = np.asarray(camera.get("homography"), dtype=float)
        poly = np.asarray(camera.get("valid_polygon_px"), dtype=float)
        size = camera.get("image_size")
        if h.shape != (3, 3) or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-12:
            raise ValueError("homography must be finite and nonsingular")
        if poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3 or not np.isfinite(poly).all() or not cv2.isContourConvex(poly.astype(np.float32)):
            raise ValueError("valid_polygon_px must be a convex measured ground region")
        if not isinstance(size, list) or len(size) != 2 or any(type(v) is not int or v <= 0 for v in size):
            raise ValueError("camera image_size must be [width,height]")
        if (poly < 0).any() or (poly[:, 0] >= size[0]).any() or (poly[:, 1] >= size[1]).any():
            raise ValueError("ground region exceeds calibrated image")
        self.camera, self.h, self.poly = camera, h, poly.astype(np.float32)

    def check_frame(self, frame):
        if frame.get("image_size") != self.camera["image_size"]:
            raise ValueError("camera resolution changed; calibration invalid")
        if frame.get("pose_us") != self.camera.get("pose_us"):
            raise ValueError("camera servo pose changed; calibration invalid")

    def point(self, point):
        p = np.asarray(point, dtype=float)
        if p.shape != (2,) or not np.isfinite(p).all() or cv2.pointPolygonTest(self.poly, tuple(p), False) < 0:
            raise ValueError("pixel outside measured ground region")
        q = self.h @ np.append(p, 1)
        if abs(q[2]) < 1e-9:
            raise ValueError("ground projection at horizon")
        out = q[:2] / q[2]
        if not np.isfinite(out).all():
            raise ValueError("nonfinite projected coordinate")
        return out.tolist()


def fit_ground_calibration(data):
    """Fit from labelled metres and report independent held-out error.

    Always returns verified=False. A good fit alone is not a field acceptance.
    """
    pairs, checks = data["points"], data["check_points"]
    if len(pairs) < 4 or len(checks) < 2:
        raise ValueError("need four fit points and two independent check points")
    px = np.asarray([p["pixel"] for p in pairs], dtype=float)
    ground = np.asarray([p["ground_m"] for p in pairs], dtype=float)
    if px.shape != ground.shape or px.shape[1:] != (2,) or not np.isfinite(px).all() or not np.isfinite(ground).all():
        raise ValueError("calibration points must be finite paired 2D coordinates")
    if any(c["pixel"] in [p["pixel"] for p in pairs] for c in checks):
        raise ValueError("check points must be independent of fit points")
    h, _ = cv2.findHomography(px, ground, method=0)
    if h is None:
        raise ValueError("degenerate calibration points")
    result = {"verified": False, "detector_verified": False, "image_size": data["image_size"],
              "pose_us": data.get("pose_us"), "homography": h.tolist(),
              "valid_polygon_px": data["valid_polygon_px"]}
    projection = GroundProjection(result, require_verified=False)
    errors = [math.dist(projection.point(c["pixel"]), c["ground_m"]) for c in checks]
    result["independent_check_rms_m"] = math.sqrt(sum(e * e for e in errors) / len(errors))
    result["independent_check_max_error_m"] = max(errors)
    return result
