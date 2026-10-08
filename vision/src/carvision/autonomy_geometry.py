"""Conservative calibrated ground features from images and surveyed task lines.

Parking uses two complete bays bounded by yellow tape and/or white lane lines
with measured dimensions. Incomplete
boundaries, obscured interiors or missing model validation yield unknown. Ground
task lines can also come from a surveyed course and verified local pose; traffic
lamp bounding boxes never stand in for a ground stop line.
"""
import math

import cv2
import numpy as np

from .autonomy_profile import GroundProjection, number


def world_to_vehicle(points, pose):
    cs, sn = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
    return [[(x - pose["x_m"]) * cs - (y - pose["y_m"]) * sn,
             (x - pose["x_m"]) * sn + (y - pose["y_m"]) * cs] for x, y in points]


def fresh_pose(pose, now):
    return isinstance(pose, dict) and pose.get("verified") is True and all(number(pose.get(k)) for k in ("x_m", "y_m", "yaw_rad", "captured_monotonic_s", "error_bound_m")) and 0 <= now - pose["captured_monotonic_s"] <= .25 and 0 <= pose["error_bound_m"] <= .03


class GroundFeatureExtractor:
    def __init__(self, profile, camera_name):
        self.profile, self.name = profile, camera_name
        self.config = profile.data["cameras"][camera_name]
        self.projection = GroundProjection(self.config) if self.config.get("verified") else None

    def _pixel(self, point):
        q = np.linalg.inv(self.projection.h) @ np.array([*point, 1.0])
        if abs(q[2]) < 1e-9:
            raise ValueError("ground feature projects to horizon")
        p = (q[:2] / q[2]).tolist()
        self.projection.point(p)  # require the calibrated visible ground region
        return p

    def extract(self, image, result, *, pose=None, now=None):
        out = {"board_roi_visible": False, "task_region_visible": False,
               "obstacle_region_visible": False, "cones": [], "parking_slots": []}
        if self.projection is None or not self.config.get("detector_verified"):
            return out
        self.projection.check_frame({"image_size": [image.shape[1], image.shape[0]], "pose_us": result.get("pose_us")})
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if np.mean((grey > 20) & (grey < 250)) < .45 or cv2.Laplacian(grey, cv2.CV_64F).var() < 12:
            return out
        out["board_roi_visible"] = True
        # Coverage is an explicitly measured road region, including braking
        # distance. Merely seeing a colour candidate is not complete visibility.
        region = self.config.get("monitored_region_m")
        if isinstance(region, list) and len(region) >= 3:
            try:
                for point in region:
                    self._pixel(point)
                out["task_region_visible"] = out["obstacle_region_visible"] = True
            except ValueError:
                pass
        for detection in result.get("detections", []):
            if not number(detection.get("score")) or detection["score"] < .6:
                continue
            box = detection.get("bbox_xyxy")
            if not isinstance(box, list) or len(box) != 4 or not all(number(v) for v in box):
                continue
            x1, y1, x2, y2 = box
            if x1 <= 1 or y1 <= 1 or x2 >= image.shape[1] - 2 or y2 >= image.shape[0] - 2 or x2 <= x1 or y2 <= y1:
                continue
            try:
                left, right = self.projection.point([x1, y2]), self.projection.point([x2, y2])
                if detection["label"] == "crosswalk" and result.get("presence", {}).get("crosswalk") == "present":
                    out["crosswalk_edge_px"] = [[x1, y2], [x2, y2]]
                elif detection["label"] == "cone":
                    radius = abs(right[0] - left[0]) / 2
                    if .01 <= radius <= .2:
                        out["cones"].append({"ground_contact_px": [(x1 + x2) / 2, y2], "radius_m": radius})
            except ValueError:
                pass
        course = self.profile.data.get("course", {})
        if course.get("verified") is True and now is not None and fresh_pose(pose, now):
            for name in ("crosswalk_edge", "traffic_stop_line"):
                line = course.get(name + "_world_m")
                if isinstance(line, list) and len(line) == 2:
                    try:
                        out[name + "_px"] = [self._pixel(p) for p in world_to_vehicle(line, pose)]
                    except ValueError:
                        pass
        out["parking_slots"] = self._parking(image, result)
        return out

    def _parking(self, image, result):
        # 200 px/m; 2 m lateral, 3 m forward. Pure image transformation, no
        # camera resizing that would silently invalidate the homography.
        scale, width, height = 200, 400, 600
        grid = np.array([[scale, 0, width / 2], [0, -scale, height], [0, 0, 1.0]])
        h = grid @ self.projection.h
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # The rule's parking enclosure and divider are yellow tape; existing
        # white running-track edges may form the other sides of each bay.
        white = cv2.inRange(hsv, (0, 0, 165), (179, 70, 255))
        yellow = cv2.inRange(hsv, (18, 90, 120), (38, 255, 255))
        mask = cv2.bitwise_or(white, yellow)
        valid = np.zeros(mask.shape, np.uint8)
        cv2.fillConvexPoly(valid, self.projection.poly.astype(np.int32), 255)
        mask = cv2.bitwise_and(mask, valid)
        bird = cv2.warpPerspective(mask, h, (width, height), flags=cv2.INTER_NEAREST)
        contours, _ = cv2.findContours(bird, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            if cv2.contourArea(contour) < .35 * .8 * scale ** 2:
                continue
            quad = cv2.approxPolyDP(contour, .02 * cv2.arcLength(contour, True), True)
            if len(quad) != 4 or not cv2.isContourConvex(quad):
                continue
            pixels = quad[:, 0, :].astype(float)
            ground = np.column_stack(((pixels[:, 0] - width / 2) / scale, (height - pixels[:, 1]) / scale))
            span_x, span_y = np.ptp(ground, axis=0)
            if not .35 <= span_x <= .75 or not .8 <= span_y <= 1.2:
                continue
            center = np.mean(ground, axis=0)
            if any(math.dist(center, c[0]) < .15 for c in candidates):
                continue
            try:
                original = [self._pixel(p) for p in ground]
            except ValueError:
                continue
            candidates.append((center, original))
        pairs = [(a, b) for i, a in enumerate(candidates) for b in candidates[i + 1:] if abs(a[0][1] - b[0][1]) < .12 and .4 <= abs(a[0][0] - b[0][0]) <= .8]
        if len(pairs) != 1:
            return []
        pair = sorted(pairs[0], key=lambda p: p[0][0])
        slots = []
        for slot_id, (_, polygon) in zip(("left", "right"), pair):
            poly = np.asarray(polygon, np.float32)
            roi = np.zeros(image.shape[:2], np.uint8)
            cv2.fillConvexPoly(roi, poly.astype(np.int32), 255)
            interior = cv2.erode(roi, np.ones((9, 9), np.uint8))
            area = np.count_nonzero(interior)
            state = "unknown"
            if area > 100:
                mean_s = float(np.mean(hsv[:, :, 1][interior > 0]))
                # A verified detector plus fully visible neutral ground is
                # needed for clear. Coloured/occluded surfaces stay unknown.
                if mean_s < 45 and np.mean(grey_visible(image)[interior > 0]) > .9:
                    state = "clear"
                for detection in result.get("detections", []):
                    if detection.get("label") not in ("blue_board", "cone") or not number(detection.get("score")) or detection["score"] < .6:
                        continue
                    box = detection.get("bbox_xyxy", [])
                    if len(box) != 4:
                        continue
                    if cv2.pointPolygonTest(poly, ((box[0] + box[2]) / 2, box[3]), False) >= 0:
                        state = "blocked"
            slots.append({"id": slot_id, "corners_px": polygon, "availability": state,
                          "occupancy_verified": state != "unknown"})
        return slots


def grey_visible(image):
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return (grey > 20) & (grey < 250)
