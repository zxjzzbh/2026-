import cv2
import numpy as np


def overlay(image, result, roi_top):
    out = image.copy()
    h, w = out.shape[:2]
    cv2.line(out, (0, int(h*roi_top)), (w-1, int(h*roi_top)), (180, 180, 180), 1)
    for line, color in [(result.lane.left, (255, 120, 0)), (result.lane.right, (0, 220, 255)),
                        (result.lane.center, (0, 255, 0))]:
        if line:
            cv2.polylines(out, [np.asarray(line, dtype=np.int32)], False, color, 2)
    if result.lane.target:
        cv2.circle(out, tuple(map(int, result.lane.target)), 6, (0, 0, 255), -1)
    for det in result.detections:
        x1, y1, x2, y2 = map(int, det.bbox_xyxy)
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 0, 220), 2)
        cv2.putText(out, f"{det.label} {det.score:.2f}", (x1, max(16, y1-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 0, 220), 1)
    offset = "N/A" if result.lane.offset_normalized is None else f"{result.lane.offset_normalized:+.3f}"
    lines = [f"PERCEPTION ONLY | frame {result.frame_id}",
             f"Lane: {result.lane.reason} | offset {offset}",
             f"YOLO: {result.detector_status} | lamp: {result.traffic_light_state}",
             f"Process: {result.processing_ms:.1f} ms"]
    for i, text in enumerate(lines):
        cv2.putText(out, text, (9, 22+i*21), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
        cv2.putText(out, text, (9, 22+i*21), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    return out
