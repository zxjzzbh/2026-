"""Optional YOLO backend. COCO weights cannot silently become race detectors."""

from pathlib import Path

import cv2
import numpy as np

from .results import Detection, RACE_CLASSES
from .traffic_lamps import horizontal_lamp_groups, group_light_state


def import_yolo():
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError("YOLO dependency missing; install vision/requirements-yolo.txt") from exc
    return YOLO


class YoloDetector:
    def __init__(self, weights, config):
        path = Path(weights)
        if not path.exists():
            raise FileNotFoundError(f"race weights not found: {path}; train on race data first")
        self.model = import_yolo()(str(path))
        # Exported models may initialize a backend while reading names. Honor the
        # requested device before that initialization instead of auto-picking GPU.
        self.model.overrides["device"] = config.device
        names = self.model.names
        self.names = dict(enumerate(names)) if isinstance(names, list) else names
        if tuple(self.names.get(i) for i in range(len(self.names))) != RACE_CLASSES:
            raise ValueError(f"race model classes must be {RACE_CLASSES}; got {self.names}. "
                             "Use benchmark for generic pretrained weights, not race replay.")
        if self.model.task != "detect":
            raise ValueError("this adapter requires a bounding-box detection model")
        self.config = config

    def detect(self, image):
        c = self.config
        result = self.model.predict(image, imgsz=c.image_size, conf=c.confidence,
                                    iou=c.iou, device=c.device, verbose=False)[0]
        detections = []
        if result.boxes is None:
            return detections
        h, w = image.shape[:2]
        for xyxy, score, cls_id in zip(result.boxes.xyxy.cpu().numpy(),
                                       result.boxes.conf.cpu().numpy(),
                                       result.boxes.cls.cpu().numpy()):
            if not np.isfinite(xyxy).all() or not np.isfinite(score):
                continue
            x1, y1, x2, y2 = xyxy.astype(float)
            box = [float(np.clip(x1, 0, w)), float(np.clip(y1, 0, h)),
                   float(np.clip(x2, 0, w)), float(np.clip(y2, 0, h))]
            if box[2] > box[0] and box[3] > box[1]:
                detections.append(Detection(self.names[int(cls_id)], float(score), box))
        return detections


def light_candidate(image, detections, config):
    """Provisional horizontal-lamp heuristic. Ambiguity returns unknown.

    Brightness contrast is required so a colored, unlit lens alone is insufficient.
    Real lamp footage is required before this output can be used for control.
    """
    states = []
    h, w = image.shape[:2]
    for det in detections:
        if det.label != "traffic_light":
            continue
        x1, y1, x2, y2 = [int(round(v)) for v in det.bbox_xyxy]
        crop = image[max(0, y1):min(h, y2), max(0, x1):min(w, x2)]
        if crop.size == 0 or crop.shape[0] < 8 or crop.shape[1] < 24:
            states.append("unknown")
            continue
        if crop.shape[1]/crop.shape[0] < 1.8:
            states.append("unknown")
            continue
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        groups = horizontal_lamp_groups(hsv)
        readings = [group_light_state(group, config) for group in groups]
        states.append(readings[0] if readings and len(set(readings)) == 1 else 'unknown')
    # Multiple lamps or ambiguous detections must agree; do not pick a random one.
    return states[0] if states and len(set(states)) == 1 else "unknown"
