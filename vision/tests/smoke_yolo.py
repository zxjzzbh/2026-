"""Optional GPU integration check; generated shapes are NOT training evidence.

Run from vision/: .venv/Scripts/python tests/smoke_yolo.py --output outputs/yolo-smoke
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from carvision.cli import main
from carvision.results import RACE_CLASSES
from carvision.sources import write_image


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--base", default="../models/weights/yolov8n.pt")
    parser.add_argument("--device", default="0")
    args = parser.parse_args()
    root = Path(args.output).resolve()
    root.mkdir(parents=True, exist_ok=False)
    dataset = root/"SYNTHETIC_ONLY_DATA"
    for split, n in [("train", 8), ("val", 4)]:
        for i in range(n):
            image = np.full((320, 320, 3), (55+i*3, 45+(12 if split == "val" else 0), 90), np.uint8)
            delta = i*2
            boxes = [(20+delta, 175, 65+delta, 250), (210, 170+delta, 295, 250+delta),
                     (105, 40+delta, 225, 85+delta), (75, 270, 190, 310)]
            cv2.fillConvexPoly(image, np.array([[42+delta, 175], [20+delta, 250], [65+delta, 250]]), (255, 30, 30))
            cv2.rectangle(image, boxes[1][:2], boxes[1][2:], (255, 30, 20), -1)
            cv2.rectangle(image, boxes[2][:2], boxes[2][2:], (10, 10, 10), -1)
            cv2.circle(image, (125, 62+delta), 14, (0, 0, 255), -1)
            for x in range(75, 190, 23):
                cv2.rectangle(image, (x, 270), (x+12, 310), (240, 240, 240), -1)
            session = f"synthetic-{split}"
            path = dataset/"images"/split/session/f"{i:03d}.png"
            write_image(path, image)
            label = dataset/"labels"/split/session/f"{i:03d}.txt"
            label.parent.mkdir(parents=True, exist_ok=True)
            rows = []
            for cls_id, (x1, y1, x2, y2) in enumerate(boxes):
                rows.append(f"{cls_id} {(x1+x2)/640} {(y1+y2)/640} {(x2-x1)/320} {(y2-y1)/320}")
            label.write_text("\n".join(rows)+"\n")
    data = dataset/"dataset.yaml"
    data.write_text(yaml.safe_dump({"path": str(dataset), "train": "images/train", "val": "images/val",
                                    "names": list(RACE_CLASSES)}), encoding="utf-8")
    assert main(["train", "--data", str(data), "--base", args.base, "--output", str(root/"train-check"),
                 "--device", args.device, "--epochs", "1", "--batch", "4", "--imgsz", "320"]) == 0
    weights = root/"train-check/training/weights/best.pt"
    assert weights.exists()
    assert main(["replay", "--source", str(dataset/"images/val/synthetic-val/000.png"),
                 "--weights", str(weights), "--device", args.device, "--output", str(root/"adapter-check")]) == 0
    (root/"NOT_A_RACE_MODEL.txt").write_text(
        "Only validates training/loading API on procedural shapes. Do not deploy these weights.\n", encoding="utf-8")
    print("YOLO training and race-adapter smoke check passed; NOT race accuracy evidence.")


if __name__ == "__main__":
    run()
