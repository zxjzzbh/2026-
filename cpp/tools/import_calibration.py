"""Import fixed-camera calibration evidence. Does not contact or operate a car."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import cv2
import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def project(h, xy):
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    q = np.column_stack((xy, np.ones(len(xy)))) @ np.asarray(h).T
    if not np.isfinite(q).all() or np.any(np.abs(q[:, 2]) < 1e-8):
        raise ValueError("Invalid homogeneous projection")
    return q[:, :2] / q[:, 2, None]


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def import_sources(vision, output):
    static = vision / "calibration/parking-static-20261113"
    index_file = static / "source-index.json"
    index = read(index_file)
    expected = {r["file"]: r["sha256"] for r in index["files"]}
    names = ["measured-1794575104206792400/bay-ground-candidate.json",
             "measured-1794575104206792400/bay-ground-rear-axle-candidate.json",
             "check-1794575933931921900/near-field-validation.json",
             "check45-1794576519124491600/near-field-validation.json"]
    records = []
    manifest = []
    for name in names:
        path = static / name
        if digest(path) != expected[name]:
            raise ValueError("Calibration source checksum mismatch: " + name)
        records.append(read(path))
        target = output / "sources/parking-static-20261113" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
        manifest.append({"source": str(path.resolve()), "file": target.relative_to(output).as_posix(), "sha256": digest(path)})
    fit, rear, *checks = records
    if rear["coordinate_frame"] != "rear_axle_x_right_y_forward" or rear["image_size"] != [480, 360]:
        raise ValueError("Wrong coordinate frame or source image dimensions")
    h = np.array(fit["homography"], float)
    rear_h = np.array(rear["homography"], float)
    pose = rear["estimated_pose_in_bay_frame"]
    c, s = math.cos(math.radians(pose["heading_right_deg"])), math.sin(math.radians(pose["heading_right_deg"]))
    x, y = pose["rear_axle_x_m"], pose["rear_axle_y_m"]
    transform = np.array([[c, -s, -c*x+s*y], [s, c, -s*x-c*y], [0, 0, 1]]) @ h
    if not np.allclose(transform/transform[2, 2], rear_h/rear_h[2, 2], atol=1e-7, rtol=0):
        raise ValueError("Rear-axle transform does not match source ruler measurements")
    check_pixels = []
    for check in checks:
        for side in ("left", "right"):
            item = check["measurements"][side]
            check_pixels.append(item["image_px"])
            errors = np.linalg.norm(project(h, item["image_px_by_frame"])-np.array(item["expected_xy_m"]), axis=1)
            if abs(float(errors.max())-item["max_error_across_frames_m"]) > 1e-7 or errors.max() > .03:
                raise ValueError("Independent static validation failed")
    hull = cv2.convexHull(np.array(rear["valid_polygon_px"]+check_pixels, np.float32))[:, 0]
    recorded = np.array(rear["static_sample_envelope_px"])
    distance = np.linalg.norm(hull[:, None]-recorded[None, :], axis=2)
    if max(distance.min(axis=0).max(), distance.min(axis=1).max()) > 3e-5:
        raise ValueError("Static sampled domain mismatch")
    parking = dict(rear)
    parking.update(valid_polygon_px=hull.tolist(), runtime_verified=False, dynamic_verified=False,
                   current_pose_verified=False, verified=False, max_error_m=.03,
                   source_kind="static_ruler_fit_with_four_held_out_points", camera="secondary",
                   domain_basis="convex_hull_of_actual_fit_and_check_points")
    write(output / "ground_parking.json", parking)
    road_source = vision / "configs/cone-latest-20261029.local.json"
    road = read(road_source)["ground"]
    if road["coordinate_frame"] != "rear_axle_x_right_y_forward":
        raise ValueError("Road calibration frame mismatch")
    road = dict(road)
    road.update(runtime_verified=False, dynamic_verified=False, current_pose_verified=False,
                verified=False, camera="secondary", source_kind="historical_fixed_camera_mapping")
    write(output / "ground_road.json", road)
    for source, relative in [(index_file, "sources/parking-static-20261113/source-index.json"),
                             (road_source, "sources/cone-latest-20261029.local.json")]:
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        manifest.append({"source": str(source.resolve()), "file": relative, "sha256": digest(source)})
    write(output / "provenance.json", {"sources": manifest, "dynamic_verified": False,
          "note": "Source files copied byte-for-byte. Static data does not verify camera mounting or dynamic motion today."})
    print("Imported road and parking maps; static provenance verified; no dynamic validation asserted.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--vision", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1]/"config/calibration")
    args = parser.parse_args()
    import_sources(args.vision.resolve(), args.output.resolve())
