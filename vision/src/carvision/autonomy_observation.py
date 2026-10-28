"""Local, timestamped dual-camera/encoder measurements into race observations.

This is a measurement adapter, not an authority to turn browser flags into
physical feedback. Parking corners and task lines must be real detected ground
features; an elevated traffic-light box is never projected onto the road.
"""
import math

import cv2
import numpy as np

from .autonomy_profile import GroundProjection, front_extent_m, number, rear_extent_m
from .race import Observation


class EncoderFeedback:
    def __init__(self, profile):
        self.config = profile.data["telemetry"]
        self.last = None
        self.fault = None

    def update(self, sample, now):
        invalid = {"telemetry_valid": False, "telemetry_age_s": None, "speed_mps": None}
        if self.fault or not self.config["verified"] or self.config.get("distance_per_count_m") is None or not number(self.config.get("maximum_speed_mps")):
            return invalid
        if not isinstance(sample, dict) or sample.get("source") != "encoder" or type(sample.get("count")) is not int or type(sample.get("sequence")) is not int or not number(sample.get("captured_monotonic_s")):
            return invalid
        t = sample["captured_monotonic_s"]
        if not 0 <= now - t <= .25:
            return invalid
        previous = self.last
        if previous is not None and (sample["sequence"] <= previous["sequence"] or t <= previous["captured_monotonic_s"]):
            return invalid
        self.last = dict(sample)
        if previous is None or t - previous["captured_monotonic_s"] > .25:
            return invalid
        speed = (sample["count"] - previous["count"]) * self.config["distance_per_count_m"] / (t - previous["captured_monotonic_s"])
        if abs(speed) > self.config["maximum_speed_mps"]:
            self.fault = "encoder_count_jump_or_overspeed"
            return invalid
        return {"telemetry_valid": True, "telemetry_age_s": now - t, "speed_mps": speed}


def inside_wheels(polygon, vehicle):
    """Count tyres with their entire rectangular footprint inside the bay."""
    required = ("wheelbase_m", "track_width_m", "tyre_width_m", "tyre_length_m")
    if not vehicle.get("measured") or any(vehicle.get(k) is None for k in required):
        return 0
    poly = np.asarray(polygon, dtype=np.float32)
    if poly.shape != (4, 2) or not np.isfinite(poly).all() or not cv2.isContourConvex(poly):
        raise ValueError("parking slot requires four ordered convex ground corners")
    count = 0
    for x in (-vehicle["track_width_m"] / 2, vehicle["track_width_m"] / 2):
        for y in (0, vehicle["wheelbase_m"]):
            corners = [(x + dx, y + dy) for dx in (-vehicle["tyre_width_m"] / 2, vehicle["tyre_width_m"] / 2) for dy in (-vehicle["tyre_length_m"] / 2, vehicle["tyre_length_m"] / 2)]
            count += all(cv2.pointPolygonTest(poly, p, False) >= 0 for p in corners)
    return count


class ConeTracker:
    """Associate stationary cones using verified world pose and conservative gates."""
    def __init__(self):
        self.tracks = {}
        self.next_id = 1

    def update(self, cones, pose, now, rear_overhang):
        if not isinstance(pose, dict) or pose.get("verified") is not True or not all(number(pose.get(k)) for k in ("x_m", "y_m", "yaw_rad", "captured_monotonic_s", "error_bound_m")) or not 0 <= now - pose["captured_monotonic_s"] <= .25 or not 0 <= pose["error_bound_m"] <= .03:
            return []
        cs, sn = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
        used = set()
        for cone in cones:
            x, y = cone["lateral_m"], cone["forward_m"]
            wx = pose["x_m"] + cs * x + sn * y
            wy = pose["y_m"] - sn * x + cs * y
            candidates = [(math.hypot(wx - v["x"], wy - v["y"]), k) for k, v in self.tracks.items() if k not in used]
            candidates.sort()
            if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < .05 and candidates[0][0] <= .15:
                continue  # ambiguous association cannot establish a pass ID
            key = candidates[0][1] if candidates and candidates[0][0] <= .15 else f"cone-{self.next_id}"
            if key not in self.tracks:
                self.next_id += 1
                self.tracks[key] = {"x": wx, "y": wy, "seen_ahead": False, "radius": cone["radius_m"]}
            track = self.tracks[key]
            track["seen_ahead"] |= y > .2
            used.add(key)
        passed = []
        for key, track in self.tracks.items():
            dx, dy = track["x"] - pose["x_m"], track["y"] - pose["y_m"]
            forward = dx * sn + dy * cs
            if track["seen_ahead"] and forward < -rear_overhang - track["radius"] - pose["error_bound_m"] - .08:
                passed.append(key)
        return sorted(passed)


class DualCameraObservationBuilder:
    def __init__(self, profile):
        self.profile = profile
        if profile.data["telemetry"]["source"] == "optical_ground":
            from .ground_odometry import OpticalFeedback
            self.feedback = OpticalFeedback(profile)
        else:
            self.feedback = EncoderFeedback(profile)
        self.cones = ConeTracker()
        self.board_seen = False
        self.last_frame = {}
        self.sequence = 0

    def _camera(self, name, data, now):
        cfg = self.profile.data["cameras"].get(name)
        if not cfg or not cfg.get("verified") or not cfg.get("detector_verified") or not isinstance(data, dict):
            return None
        t, frame_id = data.get("captured_monotonic_s"), data.get("frame_id")
        previous = self.last_frame.get(name)
        if not number(t) or not 0 <= now - t <= .25 or type(frame_id) is not int or (previous is not None and (frame_id < previous[0] or t < previous[1] or (frame_id == previous[0] and t != previous[1]))):
            return None
        if data.get("perception", {}).get("detector_status") != "ok":
            return None
        projection = GroundProjection(cfg)
        projection.check_frame(data)
        new = previous is None or frame_id > previous[0]
        self.last_frame[name] = (frame_id, t)
        return data, projection, new

    def build(self, packet, now):
        if not isinstance(packet, dict) or not number(now):
            raise ValueError("local sensor packet and monotonic time required")
        obs = Observation(now)
        telemetry = self.feedback.update(packet.get("telemetry") if self.profile.data["telemetry"]["source"] == "optical_ground" else packet.get("encoder"), now)
        for key, value in telemetry.items():
            setattr(obs, key, value)
        cameras, issues, sources = [], [], {}
        for name in ("primary", "secondary"):
            try:
                camera = self._camera(name, packet.get(name), now)
                if camera:
                    cameras.append((name, *camera))
            except ValueError as exc:
                issues.append(f"{name}: {exc}")
        obs.source_ok = any(camera[3] for camera in cameras)
        if not obs.source_ok:
            cameras = []
        vehicle = self.profile.data["vehicle"]
        front = front_extent_m(vehicle)
        lane_values, light_values = [], []
        for name, data, projection, _new in cameras:
            result = data.get("perception", {})
            features = data.get("ground_features", {})
            if not isinstance(result, dict) or not isinstance(features, dict):
                raise ValueError("perception/ground_features must be objects")
            lane = result.get("lane", {})
            if lane.get("valid") and lane.get("left") and lane.get("right"):
                try:
                    target_y = lane["target"][1]
                    left = min(lane["left"], key=lambda p: abs(p[1] - target_y))
                    right = min(lane["right"], key=lambda p: abs(p[1] - target_y))
                    lp, rp = projection.point(left), projection.point(right)
                    width = rp[0] - lp[0]
                    if width <= 0 or vehicle.get("width_m") is None or width <= vehicle["width_m"]:
                        raise ValueError("measured lane narrower than vehicle")
                    lane_values.append((name, width, (rp[0] + lp[0]) / width))
                except (ValueError, KeyError, IndexError) as exc:
                    issues.append(f"{name}.lane: {exc}")
            if not obs.board_monitor_valid and features.get("board_roi_visible") is True:
                state = result.get("presence", {}).get("blue_board", "unknown")
                if state in ("present", "absent"):
                    self.board_seen |= state == "present"
                    obs.board_monitor_valid = True
                    obs.start_board_state = "present" if state == "present" else ("removed" if self.board_seen else "unknown")
                    sources["board"] = name
            for label, attribute, offset in (("crosswalk_edge_px", "crosswalk_distance_m", front), ("traffic_stop_line_px", "traffic_stop_distance_m", vehicle.get("wheelbase_m"))):
                if getattr(obs, attribute) is None and features.get(label) is not None and offset is not None:
                    try:
                        points = [projection.point(p) for p in features[label]]
                        if len(points) != 2:
                            raise ValueError("ground line needs two endpoints")
                        setattr(obs, attribute, min(p[1] for p in points) - offset)
                        sources[attribute] = name
                    except (TypeError, ValueError) as exc:
                        issues.append(f"{name}.{label}: {exc}")
            if features.get("task_region_visible") is True:
                obs.task_monitor_valid = True
            light = result.get("traffic_light_state")
            if light in ("red", "yellow", "green"):
                light_values.append(light)
            if not obs.cone_monitor_valid and features.get("obstacle_region_visible") is True:
                measured = []
                for cone in features.get("cones", []):
                    p = projection.point(cone["ground_contact_px"])
                    radius = cone.get("radius_m")
                    if not number(radius) or not 0 < radius <= .5:
                        raise ValueError("cone radius requires measured metres")
                    measured.append({"lateral_m": p[0], "forward_m": p[1], "radius_m": radius})
                obs.cones, obs.cone_monitor_valid = measured, True
                sources["cones"] = name
            slots = features.get("parking_slots")
            if not obs.parking_geometry_valid and isinstance(slots, list) and len(slots) == 2:
                measured = []
                for slot in slots:
                    poly = [projection.point(p) for p in slot["corners_px"]]
                    if len(poly) != 4 or not isinstance(slot.get("id"), str) or not slot["id"]:
                        raise ValueError("parking slot needs stable ID and four measured corners")
                    availability = slot.get("availability", "unknown") if slot.get("occupancy_verified") is True else "unknown"
                    if availability not in ("clear", "blocked", "unknown"):
                        raise ValueError("invalid parking occupancy")
                    wheels = inside_wheels(poly, vehicle)
                    measured.append({"id": slot["id"], "center_lateral_m": float(np.mean(poly, axis=0)[0]), "availability": availability})
                    if availability == "clear":
                        obs.parking_remaining_m = max(p[1] for p in poly) - (front or 0)
                        if wheels == 4:
                            obs.wheels_inside, obs.parked_slot_id = wheels, slot["id"]
                obs.parking_slots = measured
                obs.parking_zone_visible = obs.parking_geometry_valid = True
                sources["parking"] = name
        if lane_values:
            name, obs.lane_width_m, obs.lane_offset = lane_values[0]
            obs.lane_valid = True
            sources["lane"] = name
            if len(lane_values) > 1 and (abs(lane_values[1][1] - obs.lane_width_m) > .15 or abs(lane_values[1][2] - obs.lane_offset) > .25):
                obs.lane_valid = False
                issues.append("dual_camera_metric_lane_conflict")
        obs.traffic_light_state = light_values[0] if light_values and len(set(light_values)) == 1 else "unknown"
        obs.traffic_zone_entered = obs.traffic_stop_distance_m is not None and 0 <= obs.traffic_stop_distance_m <= 1
        obs.passed_cone_ids = self.cones.update(obs.cones, packet.get("pose"), now, rear_extent_m(vehicle) or 0)
        # Gate inputs are supplied by the authenticated local handover gateway,
        # not by camera heuristics or the public preview endpoint.
        gate = packet.get("gateway")
        if isinstance(gate, dict) and gate.get("verified") is True and number(gate.get("captured_monotonic_s")) and 0 <= now - gate["captured_monotonic_s"] <= .25:
            for field in ("start_line_crossed", "in_switch_zone", "enable_autonomy", "emergency_stop", "payment_confirmed"):
                if field in gate:
                    if type(gate[field]) is not bool:
                        raise ValueError(f"gateway {field} must be boolean")
                    setattr(obs, field, gate[field])
            obs.remote = gate.get("remote")
            obs.payment_amount_cents = gate.get("payment_amount_cents")
        self.sequence += 1
        return {"schema_version": 1, "mode": "live", "sequence": self.sequence,
                "captured_monotonic_s": now, "observation": obs.__dict__,
                "source_oldest_monotonic_s": min((c[1]["captured_monotonic_s"] for c in cameras), default=None),
                "measurement_sources": sources, "issues": issues}
