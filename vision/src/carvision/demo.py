"""Procedural software fixtures. These are NOT race data or accuracy evidence."""

import cv2
import numpy as np

from .sources import write_image


def make_frame(shift=0.0, curve=0.0, missing=None, width=640, height=480):
    image = np.full((height, width, 3), (65, 60, 135), np.uint8)
    ys = np.linspace(.43, .99, 90)
    center = .5 + shift + curve*(1-ys)**2
    half = .06 + .34*(ys-.43)/.56
    for side, xs in [("left", center-half), ("right", center+half)]:
        if missing not in (side, "both"):
            points = np.column_stack((xs*(width-1), ys*(height-1))).astype(np.int32)
            cv2.polylines(image, [points], False, (235, 235, 235), 7)
    cv2.putText(image, "SYNTHETIC - SOFTWARE CHECK ONLY", (10, height-14),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 1)
    return image


def generate_demo(output):
    output.mkdir(parents=True, exist_ok=False)
    path = output / "synthetic-lane.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 20, (640, 480))
    if not writer.isOpened():
        raise RuntimeError("MJPG VideoWriter unavailable")
    try:
        for i in range(160):
            missing = "both" if 100 <= i < 120 else None
            image = make_frame(shift=.045*np.sin(i/18), curve=.12*np.sin(i/30), missing=missing)
            writer.write(image)
            if i == 30:
                write_image(output/"synthetic-example.jpg", image)
    finally:
        writer.release()
    return path
