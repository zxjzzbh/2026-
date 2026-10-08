"""Reproduce review findings offline; never import or open actuator drivers.

This is an audit of the 2026-10-08 snapshot, not a hardware acceptance test.
Run from the repository root after installing vision[test].
"""
import json
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vision/src"))

from carvision.autonomy_profile import AutonomyProfile, GroundProjection
from carvision.autonomy_observation import inside_wheels
from carvision.ground_odometry import GroundVisualOdometry
from carvision.race import Observation, Phase, RaceController, load_race_config


def audit():
    profile = AutonomyProfile.load(ROOT / "vision/configs/autonomy-pi5.json")
    output = {"hardware_access": False, "readiness_missing": profile.missing()}
    geometry = {}
    for name, camera in profile.data["cameras"].items():
        projection = GroundProjection(camera)
        measured = [projection.point(p) for p in camera["valid_polygon_px"]]
        target = np.linalg.inv(projection.h) @ np.array([0, .34 + .25, 1])
        pixel = (target[:2] / target[2]).tolist()
        try:
            projection.point(pixel)
            rejected = False
        except ValueError:
            rejected = True
        geometry[name] = {
            "measured_ground_y_m": [round(p[1], 4) for p in measured],
            "crosswalk_target_25cm_pixel": pixel,
            "crosswalk_target_rejected": rejected,
            "wheels_inside_entire_measured_polygon": inside_wheels(measured, profile.data["vehicle"]),
        }
    output["current_geometry"] = geometry

    # Isolate the decision layer. These synthetic observations are NOT physical
    # feedback and never reach a motion driver. Model a detected line leaving
    # the calibrated region while lane/source/feedback remain valid.
    race = RaceController(load_race_config(ROOT / "vision/configs/race-2026.json"))
    race.phase = Phase.CROSSWALK_APPROACH
    shared = dict(source_ok=True, telemetry_valid=True, telemetry_age_s=0,
                  speed_mps=.1, lane_valid=True, lane_offset=0, lane_width_m=1.2,
                  task_monitor_valid=True, cone_monitor_valid=True)
    first = race.update(Observation(t_s=1, crosswalk_distance_m=.51, **shared))
    second = race.update(Observation(t_s=1.1, crosswalk_distance_m=None, **shared))
    output["crosswalk_lost_distance_decision"] = {
        "before": first["action"], "after": second["action"],
        "after_speed_mps": second["speed_mps"], "after_reason": second["reason"],
    }

    # Exercise the real optical-flow correspondence loop, replacing only the
    # image measurements. One tracked point moves outside the valid region.
    from copy import deepcopy
    data = deepcopy(profile.data)
    data["telemetry"].update(verified=True, source="optical_ground", maximum_speed_mps=1)
    odometry = GroundVisualOdometry(AutonomyProfile(data))

    class Projection:
        poly = np.array([[0, 0], [99, 0], [99, 99], [0, 99]], dtype=np.float32)

        def check_frame(self, _):
            pass

        def point(self, p):
            if p[0] >= 100:
                raise ValueError("pixel outside measured ground region")
            return np.asarray(p) / 100

    odometry.projection = Projection()
    points = np.array([[[10 + i % 5 * 12, 10 + i // 5 * 12]] for i in range(21)], dtype=np.float32)
    moved = points.copy()
    moved[-1, 0, 0] = 101
    image = np.zeros((100, 100, 3), np.uint8)
    odometry.previous = (image[:, :, 0], points, 1.0)
    with patch("carvision.ground_odometry.cv2.goodFeaturesToTrack", return_value=points), \
         patch("carvision.ground_odometry.cv2.calcOpticalFlowPyrLK", return_value=(moved, np.ones((21, 1), np.uint8), np.zeros((21, 1), np.float32))):
        try:
            result = odometry.update(image, 1.1, {})
            output["optical_flow_boundary"] = {"raises": False, "valid": result["valid"]}
        except Exception as exc:
            output["optical_flow_boundary"] = {"raises": True, "exception": type(exc).__name__, "message": str(exc)}
    return output


if __name__ == "__main__":
    print(json.dumps(audit(), ensure_ascii=False, indent=2))
