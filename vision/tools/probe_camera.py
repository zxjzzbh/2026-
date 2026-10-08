"""Windows USB-device camera test. No fallback to a different camera.

Install requirements-camera.txt. Example:
python tools/probe_camera.py --vid 1bcf --pid 2281 --output outputs/camera-probe
Only tests requested modes; does not enumerate all hardware-supported modes.
"""

import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np
from cv2_enumerate_cameras import enumerate_cameras

from carvision.sources import write_image


def probe(camera, requested, seconds, output):
    cap = cv2.VideoCapture(camera.index, camera.backend)
    info = {"requested": requested, "status": "starting", "read_failures": 0}
    times, differences, digest = [], 0, None
    sample = None
    try:
        if not cap.isOpened():
            raise RuntimeError("selected camera could not be opened")
        results = {}
        if requested:
            for name, prop, value in [("width", cv2.CAP_PROP_FRAME_WIDTH, requested[0]),
                                      ("height", cv2.CAP_PROP_FRAME_HEIGHT, requested[1]),
                                      ("fps", cv2.CAP_PROP_FPS, requested[2])]:
                results[name] = bool(cap.set(prop, value))
        info["request_accepted"] = results
        info["backend"] = cap.getBackendName()
        # Warm up from the first decoded frame, not from opening the device.
        first = None
        measuring = None
        while True:
            ok, image = cap.read()
            now = time.perf_counter()
            if not ok or image is None:
                info["read_failures"] += 1
                raise RuntimeError("camera read failed")
            if first is None:
                first = now
            if now-first < 1.0:
                continue
            if measuring is None:
                measuring = now
            times.append(now)
            current = hashlib.sha256(image.tobytes()).digest()
            if digest is not None and current != digest:
                differences += 1
            digest = current
            sample = image
            if now-measuring >= seconds:
                break
        info["status"] = "completed"
        info["decoded_size"] = [sample.shape[1], sample.shape[0]]
        info["reported_size"] = [cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)]
        info["reported_fps"] = cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_text = "".join(chr((fourcc >> (8*i)) & 255) for i in range(4))
        info["reported_fourcc_raw"] = fourcc
        info["reported_fourcc"] = fourcc_text if all(32 <= ord(c) < 127 for c in fourcc_text) else None
        info["frames"] = len(times)
        info["elapsed_s"] = times[-1]-times[0]
        info["host_delivered_fps"] = (len(times)-1)/(times[-1]-times[0])
        info["changed_consecutive_frames"] = differences
        info["interval_p95_ms"] = float(np.percentile(np.diff(times)*1000, 95))
        info["requested_size_observed"] = None if requested is None else info["decoded_size"] == requested[:2]
        gray = cv2.cvtColor(sample, cv2.COLOR_BGR2GRAY)
        info["gray_mean"] = float(gray.mean())
        info["pixels_below_10_fraction"] = float((gray < 10).mean())
        info["pixels_above_245_fraction"] = float((gray > 245).mean())
        write_image(output/"sample.jpg", sample)
    except Exception as exc:
        info["status"] = "error"
        info["error"] = str(exc)
    finally:
        cap.release()
        (output/"result.json").write_text(json.dumps(info, indent=2, allow_nan=False), encoding="utf-8")
    return info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vid", required=True, help="hex USB vendor ID")
    parser.add_argument("--pid", required=True, help="hex USB product ID")
    parser.add_argument("--backend", choices=["msmf", "dshow"], default="msmf")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.seconds <= 0:
        raise ValueError("seconds must be positive")
    backend = {"msmf": cv2.CAP_MSMF, "dshow": cv2.CAP_DSHOW}[args.backend]
    candidates = [c for c in enumerate_cameras(backend)
                  if c.vid == int(args.vid, 16) and c.pid == int(args.pid, 16)]
    if len(candidates) != 1:
        raise RuntimeError(f"expected exactly one matching USB camera, found {len(candidates)}")
    camera = candidates[0]
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=False)
    summary = {"name": camera.name, "index": camera.index, "backend": backend,
               "vid": args.vid, "pid": args.pid, "tests": [],
               "note": "Host-delivered frames, not sensor exposure rate. Tested modes only; no scene accuracy claim."}
    for name, mode in [("vga30", [640, 480, 30]), ("hd30", [1280, 720, 30]),
                       ("fhd30", [1920, 1080, 30])]:
        target = root/name
        target.mkdir()
        result = probe(camera, mode, args.seconds, target)
        summary["tests"].append({"test": name, **result})
        print(json.dumps({"test": name, **result}, ensure_ascii=True), flush=True)
        (root/"summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
