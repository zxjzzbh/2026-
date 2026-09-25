import time

from .detection import light_candidate
from .lane import LaneDetector
from .results import PerceptionResult, RACE_CLASSES
from .temporal import TimedConfirmation


class Pipeline:
    def __init__(self, config, detector=None):
        self.config = config
        self.lane = LaneDetector(config.lane)
        self.detector = detector
        t = config.temporal
        self.confirm = {k: TimedConfirmation(t.confirm_ms, t.max_gap_ms)
                        for k in (*RACE_CLASSES, "light_state")}

    def process(self, frame, live=False):
        started = time.perf_counter()
        lane, mask = self.lane.detect(frame.image)
        detections = [] if self.detector is None else self.detector.detect(frame.image)
        timestamp = frame.receive_monotonic_ms if live else frame.source_time_ms
        presence = {}
        for label in RACE_CLASSES:
            value = "unknown" if self.detector is None else (
                "present" if any(d.label == label for d in detections) else "absent")
            presence[label] = self.confirm[label].update(value, timestamp)
        light = light_candidate(frame.image, detections, self.config.traffic_light)
        light = self.confirm["light_state"].update(light, timestamp)
        result = PerceptionResult(frame.frame_id, frame.source_time_ms, frame.receive_monotonic_ms,
                                  [frame.image.shape[1], frame.image.shape[0]], lane,
                                  "disabled" if self.detector is None else "ok", detections, presence,
                                  traffic_light_state=light)
        result.processing_ms = (time.perf_counter()-started)*1000
        result.receive_to_result_ms = max(0, time.monotonic()*1000-frame.receive_monotonic_ms)
        return result, mask
