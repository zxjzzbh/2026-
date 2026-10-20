"""Ground-flow boundary regressions with real projection and OpenCV fitting.

Only feature detection/tracking measurements are injected. The coordinates are
synthetic software fixtures, never field calibration or movement permission.
"""
import json
from pathlib import Path
import runpy

import cv2
import numpy as np
import pytest

from carvision.autonomy_profile import AutonomyProfile
from carvision.ground_odometry import GroundVisualOdometry


ROOT = Path(__file__).resolve().parents[2]


def flow_case(monkeypatch, count=21):
    data = json.loads((ROOT / "vision/configs/autonomy-pi5.json").read_text(encoding="utf-8"))
    data["telemetry"].update(
        source="optical_ground", verified=True, pose_verified=True,
        maximum_speed_mps=1., visual_error_bound_m=.02,
        origin_pose={"x_m": 0., "y_m": 0., "yaw_rad": 0.},
    )
    data["cameras"]["primary"].update(
        verified=True, image_size=[100, 100], pose_us=None,
        homography=[[.01, 0., 0.], [0., .01, 0.], [0., 0., 1.]],
        valid_polygon_px=[[0., 0.], [99., 0.], [99., 99.], [0., 99.]],
    )
    odometry = GroundVisualOdometry(AutonomyProfile(data))
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    points = np.array([[[10 + i % 5 * 12, 10 + i // 5 * 12]] for i in range(count)], dtype=np.float32)
    moved = points.copy()
    moved[:, 0, 1] -= 1.  # 1 cm forward in 0.1 s: expected speed is 0.1 m/s.
    odometry.previous = (image[:, :, 0], points, 1.)
    monkeypatch.setattr(cv2, "goodFeaturesToTrack", lambda *a, **kw: points.copy())
    monkeypatch.setattr(cv2, "calcOpticalFlowPyrLK", lambda *a, **kw: (
        moved.copy(), np.ones((count, 1), np.uint8), np.zeros((count, 1), np.float32),
    ))
    actual_fit = cv2.estimateAffinePartial2D
    fit_inputs = []

    def record_fit(before, after, **kwargs):
        fit_inputs.append((before.copy(), after.copy()))
        return actual_fit(before, after, **kwargs)

    monkeypatch.setattr(cv2, "estimateAffinePartial2D", record_fit)
    info = {"image_size": [100, 100], "pose_us": None, "detections": []}
    return odometry, image, info, points, moved, fit_inputs


@pytest.mark.parametrize("index", [0, 10, 20], ids=["first", "middle", "last"])
@pytest.mark.parametrize("invalid_x", [101., float("nan"), float("inf")],
                         ids=["outside-calibration", "nan", "infinite"])
def test_failed_current_projection_discards_whole_pair(monkeypatch, index, invalid_x):
    odometry, image, info, points, moved, fits = flow_case(monkeypatch)
    moved[index, 0, 0] = invalid_x
    result = odometry.update(image, 1.1, info)

    assert result["valid"] is True
    assert result["speed_mps"] == pytest.approx(.1, abs=1e-6)
    assert result["pose"]["y_m"] == pytest.approx(.01, abs=1e-6)
    before, after = fits[0]
    # Check identities as well as lengths: trimming one array would silently
    # pair the wrong ground points after a failure in the middle.
    keep = np.arange(len(points)) != index
    assert before.shape == after.shape == (20, 2)
    np.testing.assert_allclose(before, points[keep, 0].astype(float) * .01)
    np.testing.assert_allclose(after, moved[keep, 0].astype(float) * .01)


def test_failed_previous_projection_also_discards_whole_pair(monkeypatch):
    odometry, image, info, points, moved, fits = flow_case(monkeypatch)
    points[10, 0, 0] = 101.
    result = odometry.update(image, 1.1, info)
    assert result["valid"] is True
    assert result["speed_mps"] == pytest.approx(.1, abs=1e-6)
    keep = np.arange(len(points)) != 10
    np.testing.assert_allclose(fits[0][0], points[keep, 0].astype(float) * .01)
    np.testing.assert_allclose(fits[0][1], moved[keep, 0].astype(float) * .01)


def test_twenty_valid_pairs_preserve_speed_and_pose(monkeypatch):
    odometry, image, info, _, _, fits = flow_case(monkeypatch, count=20)
    result = odometry.update(image, 1.1, info)
    assert result["valid"] is True
    assert result["speed_mps"] == pytest.approx(.1, abs=1e-6)
    assert result["pose"]["y_m"] == pytest.approx(.01, abs=1e-6)
    assert fits[0][0].shape == fits[0][1].shape == (20, 2)


@pytest.mark.parametrize("lost", [1, 20], ids=["nineteen-remain", "none-remain"])
def test_insufficient_pairs_report_unknown_and_do_not_restore_lost_pose(monkeypatch, lost):
    odometry, image, info, points, moved, fits = flow_case(monkeypatch, count=20)
    moved[:lost, 0, 0] = 101.
    result = odometry.update(image, 1.1, info)

    assert result["valid"] is False
    assert result["speed_mps"] is None and result["pose"] is None
    assert fits == []  # Insufficient correspondence must never reach fitting.
    assert odometry.pose_valid is False

    # Fresh reliable flow can recover velocity, but cannot fill the missing
    # displacement to manufacture an uninterrupted world pose.
    moved[:] = points
    moved[:, 0, 1] -= 1.
    recovered = odometry.update(image, 1.2, info)
    assert recovered["valid"] is True
    assert recovered["speed_mps"] == pytest.approx(.1, abs=1e-6)
    assert recovered["pose"] is None


def test_original_snapshot_audit_boundary_case_no_longer_raises():
    audit = runpy.run_path(str(ROOT / "tools/audit_snapshot_20261008.py"))["audit"]
    report = audit()
    assert report["hardware_access"] is False
    assert report["optical_flow_boundary"] == {"raises": False, "valid": True}
