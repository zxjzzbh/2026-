"""Optional measured ground optical flow; no assumed speed from ESC commands.

Only a fixed, ground-calibrated primary camera is eligible. Texture loss,
nonrigid motion, scale drift, excessive error or frame gaps invalidate feedback.
World pose additionally needs a measured origin and field-verified error bound.
An encoder remains preferred when one is present.
"""
import math

import cv2
import numpy as np

from .autonomy_profile import GroundProjection, number


class GroundVisualOdometry:
    def __init__(self, profile):
        self.profile = profile
        self.config = profile.data["telemetry"]
        self.projection = GroundProjection(profile.data["cameras"]["primary"])
        self.previous = None
        self.sequence = 0
        self.pose = dict(self.config.get("origin_pose") or {})
        self.pose_valid = self.config.get("pose_verified") is True and all(number(self.pose.get(k)) for k in ("x_m", "y_m", "yaw_rad"))

    def update(self, image, timestamp, result):
        self.sequence += 1
        invalid = {"source": "optical_ground", "valid": False, "sequence": self.sequence,
                   "captured_monotonic_s": timestamp, "speed_mps": None, "pose": None}
        self.projection.check_frame(result)
        if not number(timestamp) or self.config.get("verified") is not True:
            return invalid
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        mask = np.zeros(grey.shape, np.uint8)
        cv2.fillConvexPoly(mask, self.projection.poly.astype(np.int32), 255)
        # Elevated objects do not share the road homography or static road flow.
        for detection in result.get("detections", []):
            if detection.get("label") != "crosswalk":
                box = detection.get("bbox_xyxy", [])
                if len(box) == 4 and all(number(v) for v in box):
                    x1, y1, x2, y2 = [int(v) for v in box]
                    cv2.rectangle(mask, (x1, y1), (x2, y2), 0, -1)
        points = cv2.goodFeaturesToTrack(grey, maxCorners=120, qualityLevel=.02, minDistance=8, mask=mask)
        old = self.previous
        self.previous = (grey, points, timestamp)
        if old is None:
            return invalid
        old_grey, old_points, old_t = old
        dt = timestamp - old_t
        if old_points is None or points is None or not 0 < dt <= .25:
            self.pose_valid = False
            return invalid
        moved, status, errors = cv2.calcOpticalFlowPyrLK(old_grey, grey, old_points, None,
                                                       winSize=(15, 15), maxLevel=2)
        if moved is None or status is None:
            self.pose_valid = False
            return invalid
        before, after = [], []
        for p, q, good, error in zip(old_points[:, 0], moved[:, 0], status[:, 0], errors[:, 0]):
            if not good or error > 20:
                continue
            try:
                before.append(self.projection.point(p))
                after.append(self.projection.point(q))
            except ValueError:
                pass
        if len(before) < 20:
            self.pose_valid = False
            return invalid
        before, after = np.asarray(before, np.float64), np.asarray(after, np.float64)
        affine, inliers = cv2.estimateAffinePartial2D(before, after, method=cv2.RANSAC,
                                                     ransacReprojThreshold=.015, maxIters=200)
        if affine is None or inliers is None or float(np.mean(inliers)) < .75:
            self.pose_valid = False
            return invalid
        scale = math.hypot(affine[0, 0], affine[1, 0])
        rotation = affine[:, :2] / scale
        yaw_delta = math.atan2(rotation[1, 0], rotation[0, 0])
        displacement = -rotation.T @ affine[:, 2]
        speed = float(displacement[1] / dt)
        predicted = before @ affine[:, :2].T + affine[:, 2]
        residual = float(np.sqrt(np.mean(np.sum((predicted[inliers[:, 0] > 0] - after[inliers[:, 0] > 0]) ** 2, axis=1))))
        maximum = self.config.get("maximum_speed_mps")
        if not number(maximum) or abs(speed) > maximum or abs(displacement[0] / dt) > maximum or abs(yaw_delta / dt) > 1.5 or abs(scale - 1) > .02 or residual > .015:
            self.pose_valid = False
            return invalid
        pose = None
        if self.pose_valid:
            heading = self.pose["yaw_rad"]
            dx, dy = displacement
            self.pose["x_m"] += float(math.cos(heading) * dx + math.sin(heading) * dy)
            self.pose["y_m"] += float(-math.sin(heading) * dx + math.cos(heading) * dy)
            self.pose["yaw_rad"] += yaw_delta
            bound = self.config.get("visual_error_bound_m")
            if number(bound) and 0 <= bound <= .03:
                pose = {**self.pose, "verified": True, "captured_monotonic_s": timestamp, "error_bound_m": bound}
        return {"source": "optical_ground", "valid": True, "sequence": self.sequence,
                "captured_monotonic_s": timestamp, "speed_mps": speed, "pose": pose,
                "inlier_fraction": float(np.mean(inliers)), "rigid_fit_rms_m": residual}


class OpticalFeedback:
    def __init__(self, profile):
        self.config = profile.data["telemetry"]
        self.sequence = self.timestamp = None

    def update(self, sample, now):
        invalid = {"telemetry_valid": False, "telemetry_age_s": None, "speed_mps": None}
        if self.config.get("verified") is not True or not isinstance(sample, dict) or sample.get("source") != "optical_ground" or sample.get("valid") is not True:
            return invalid
        t, sequence, speed = sample.get("captured_monotonic_s"), sample.get("sequence"), sample.get("speed_mps")
        if not number(t) or not 0 <= now - t <= .25 or type(sequence) is not int or not number(speed) or not number(self.config.get("maximum_speed_mps")) or abs(speed) > self.config["maximum_speed_mps"]:
            return invalid
        if self.sequence is not None and (sequence <= self.sequence or t <= self.timestamp):
            return invalid
        self.sequence, self.timestamp = sequence, t
        return {"telemetry_valid": True, "telemetry_age_s": now - t, "speed_mps": speed}
