import time

import cv2
import numpy as np
import pytest

from carvision.config import Config, load_config
from carvision.demo import make_frame
from carvision.detection import light_candidate
from carvision.lane import LaneDetector
from carvision.pipeline import Pipeline
from carvision.results import Detection
from carvision.sources import Frame
from carvision.temporal import TimedConfirmation


@pytest.mark.parametrize("shift,sign", [(-.06, -1), (0, 0), (.06, 1)])
def test_lane_reports_signed_image_offset(shift, sign):
    lane, mask = LaneDetector(Config().lane).detect(make_frame(shift=shift))
    assert lane.valid
    assert abs(lane.offset_normalized-2*shift) < .025
    if sign:
        assert lane.offset_normalized*sign > 0
    assert mask.shape == (480, 640)


@pytest.mark.parametrize("missing", ["left", "right", "both"])
def test_missing_boundary_cannot_be_zero_error(missing):
    lane, _ = LaneDetector(Config().lane).detect(make_frame(missing=missing))
    assert not lane.valid
    assert lane.offset_normalized is None


def test_temporal_requires_duration_and_clears_on_gaps_and_unknown():
    t = TimedConfirmation(200, 250)
    assert t.update("present", 0) == "unknown"
    assert t.update("present", 100) == "unknown"
    assert t.update("present", 200) == "present"
    assert t.update("present", 600) == "unknown"
    assert t.update("unknown", 650) == "unknown"
    assert t.update("present", 700) == "unknown"
    assert t.update("present", 600) == "unknown"


def test_disabled_model_is_unknown_not_absence():
    result, _ = Pipeline(Config()).process(Frame(make_frame(), 0, 0, time.monotonic()*1000))
    assert result.detector_status == "disabled"
    assert set(result.presence.values()) == {"unknown"}
    assert result.traffic_light_state == "unknown"


def test_single_image_does_not_confirm_time_based_state():
    class Detector:
        def detect(self, image):
            return [Detection("blue_board", .9, [100., 100., 200., 200.])]
    result, _ = Pipeline(Config(), Detector()).process(Frame(make_frame(), 0, None, time.monotonic()*1000))
    assert result.detections
    assert result.presence["blue_board"] == "unknown"


def test_lamp_unlit_colored_lenses_remain_unknown():
    image = np.zeros((100, 300, 3), np.uint8)
    boxes = [Detection("traffic_light", .9, [0, 0, 300, 100])]
    for x, color in [(50, (0, 0, 70)), (150, (0, 70, 70)), (250, (0, 70, 0))]:
        cv2.circle(image, (x, 50), 25, color, -1)
    assert light_candidate(image, boxes, Config().traffic_light) == "unknown"
    cv2.circle(image, (50, 50), 25, (0, 0, 255), -1)
    assert light_candidate(image, boxes, Config().traffic_light) == "red"
    cv2.circle(image, (250, 50), 25, (0, 255, 0), -1)
    assert light_candidate(image, boxes, Config().traffic_light) == "unknown"


def test_config_rejects_invalid_geometry(tmp_path):
    p = tmp_path/"bad.json"
    p.write_text('{"lane":{"roi_top":0.9}}')
    with pytest.raises(ValueError):
        load_config(p)
