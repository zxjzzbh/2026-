"""Display actual car cameras in OpenCV. HTTP GET only; no drive endpoints.

Reuses the Pi's existing camera owner and processes snapshots on this PC.
Close the window or press Q/Esc to stop. No web UI or synthetic video.
"""
import argparse
import json
import re
import sys
import threading
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from urllib.request import HTTPRedirectHandler, ProxyHandler, build_opener

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import cv2
import numpy as np

from carvision.config import load_config
from carvision.crosswalk import load_crosswalk_config
from carvision.crosswalk_lab import CrosswalkPerception, default_config
from carvision.sources import Frame, write_image


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


def get_bytes(opener, url, limit):
    with opener.open(url, timeout=3) as response:
        payload = response.read(limit + 1)
    if len(payload) > limit:
        raise ValueError("camera response exceeded size limit")
    return payload


class CameraWorker:
    def __init__(self, base, name, fps, vision_config):
        self.base, self.name, self.fps = base, name, fps
        self.pipeline = CrosswalkPerception(load_config(None), vision_config)
        self.opener = build_opener(ProxyHandler({}), NoRedirect())
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = "Connecting to actual camera..."
        self.thread = threading.Thread(target=self.work, name="remote-" + name, daemon=True)
        self.thread.start()

    def work(self):
        last_id = None
        last_received = None
        try:
            while not self.stop.is_set():
                started = time.monotonic()
                try:
                    status = json.loads(get_bytes(self.opener, self.base + "/api/status?camera=" + self.name, 262144))
                    raw = status.get("raw_preview") or {}
                    frame_id = raw.get("frame_id")
                    age = raw.get("host_frame_age_ms")
                    if (raw.get("state") != "ok" or type(frame_id) is not int
                            or type(age) not in (float, int) or not 0 <= age <= 1500):
                        raise RuntimeError("upstream camera is offline or stale")
                    if frame_id == last_id:
                        if last_received is not None and time.monotonic() - last_received > 1.5:
                            raise RuntimeError("upstream frame counter stopped")
                        self.stop.wait(.05)
                        continue
                    if last_id is not None and frame_id < last_id:
                        self.pipeline.detector.confirm.reset()
                    payload = get_bytes(self.opener, self.base + "/frame.jpg?camera=" + self.name + "&view=raw", 4194304)
                    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        raise RuntimeError("invalid JPEG from car")
                    received = time.monotonic()
                    request_ms = (received - started) * 1000
                    if request_ms > 1500:
                        raise RuntimeError("camera request too slow; frame discarded")
                    result, zebra, mask = self.pipeline.process(Frame(image, frame_id, None, received * 1000), live=True)
                    data = {"camera": self.name, "frame_id_before_snapshot": frame_id,
                            "pc_received_monotonic_s": received, "request_ms": request_ms,
                            "upstream_age_before_request_ms": age, "processing_ms": result.processing_ms,
                            "crosswalk": zebra, "lane": result.lane.__dict__, "hardware_output": False}
                    # Frame JPEG and status are separate GETs: the recorded ID
                    # proves source progress, not exact exposure/JPEG identity.
                    with self.lock:
                        self.latest = (image, result, zebra, mask, data)
                        self.error = None
                    last_id, last_received = frame_id, received
                except Exception as exc:
                    self.pipeline.detector.confirm.reset()
                    with self.lock:
                        self.error = str(exc)
                self.stop.wait(max(.02, 1 / self.fps - (time.monotonic() - started)))
        finally:
            self.stop.set()

    def snapshot(self):
        with self.lock:
            return self.latest, self.error


def label(image, text, x, y, color=(240, 240, 240), scale=.55):
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)


def render(name, snapshot, error, now, show_mask=False):
    pane = np.full((560, 640, 3), (28, 25, 22), np.uint8)
    label(pane, name.upper() + " | ACTUAL CAR CAMERA", 10, 25)
    if snapshot is None:
        label(pane, (error or "Waiting for camera")[:72], 10, 110, (80, 180, 255), .45)
        return pane
    image, result, zebra, mask, data = snapshot
    frame_age = now - data["pc_received_monotonic_s"]
    if error or frame_age > 1.5:
        label(pane, "DISCONNECTED / STALE - recognition unknown", 10, 105, (50, 50, 255), .5)
        label(pane, (error or "No new frame")[:78], 10, 135, scale=.4)
        return pane
    view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if show_mask else image.copy()
    h, w = view.shape[:2]
    if not show_mask:
        for line, color in ((result.lane.left, (255, 160, 0)), (result.lane.right, (0, 220, 255)),
                            (result.lane.center, (0, 240, 0))):
            if line:
                cv2.polylines(view, [np.asarray(line, np.int32)], False, color, 2)
        for x, y, bw, bh in zebra["stripes_xywh"]:
            cv2.rectangle(view, (x, y), (x + bw - 1, y + bh - 1), (0, 220, 255), 2)
        if zebra["bbox_xyxy"]:
            x1, y1, x2, y2 = zebra["bbox_xyxy"]
            cv2.rectangle(view, (x1, y1), (x2, y2), (255, 0, 255), 2)
    pane[65:545] = cv2.resize(view, (640, 480))
    label(pane, f"Crosswalk: {zebra['presence']} | strips: {len(zebra['stripes_xywh'])}", 10, 51,
          (0, 255, 0) if zebra["presence"] == "present" else (240, 240, 240))
    label(pane, f"Frame {data['frame_id_before_snapshot']} | request {data['request_ms']:.0f}ms | process {data['processing_ms']:.1f}ms", 10, 88, scale=.42)
    label(pane, "Lane: " + result.lane.reason, 10, 111, scale=.42)
    return pane


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", required=True)
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--cameras", nargs="+", choices=["primary", "secondary"], default=["primary", "secondary"])
    p.add_argument("--fps", type=float, default=4)
    p.add_argument("--output", required=True)
    p.add_argument("--seconds", type=float, default=0, help="0: keep viewing until Q/Esc or close")
    p.add_argument("--headless", action="store_true", help="read-only connectivity check without a window")
    args = p.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.-]*", args.host) or not 1 <= args.port <= 65535:
        p.error("invalid host or port")
    if not 1 <= args.fps <= 10 or args.seconds < 0 or len(set(args.cameras)) != len(args.cameras):
        p.error("invalid FPS, duration or duplicate camera")
    if args.headless and args.seconds == 0:
        p.error("headless checks require a positive --seconds")
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    # Remote secondary preview is about 2.5 FPS. Only this visual viewer uses
    # this confirmation gap; autonomous/control deadlines are not changed.
    vision = replace(load_crosswalk_config(default_config("crosswalk-vision.json")), confirm_ms=500, max_gap_ms=1200)
    workers = [CameraWorker(f"http://{args.host}:{args.port}", name, args.fps, vision) for name in args.cameras]
    window = "SmartCar - LIVE Crosswalk - Q to close / M mask"
    start, last_save = time.monotonic(), 0
    observed = {name: set() for name in args.cameras}
    mask = False
    try:
        if not args.headless:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, 1280 if len(workers) == 2 else 640, 600)
        with (out / "recognition.jsonl").open("w", encoding="utf-8") as log:
            while not args.seconds or time.monotonic() - start < args.seconds:
                now = time.monotonic()
                panes, status = [], {}
                for worker in workers:
                    snapshot, error = worker.snapshot()
                    panes.append(render(worker.name, snapshot, error, now, mask))
                    fresh = snapshot is not None and not error and now - snapshot[4]["pc_received_monotonic_s"] <= 1.5
                    data = dict(snapshot[4]) if fresh else {"camera": worker.name, "crosswalk": {"presence": "unknown"}}
                    if fresh:
                        frame_id = data["frame_id_before_snapshot"]
                        if frame_id not in observed[worker.name]:
                            log.write(json.dumps(data, ensure_ascii=False) + "\n")
                            log.flush()
                            observed[worker.name].add(frame_id)
                    status[worker.name] = {"fresh": fresh, "error": error, **data}
                canvas = np.hstack(panes)
                footer = np.zeros((35, canvas.shape[1], 3), np.uint8)
                label(footer, "LIVE VIDEO | recognition on this PC | no vehicle commands | Q/Esc close | M mask", 10, 23, scale=.48)
                canvas = np.vstack((canvas, footer))
                if now - last_save >= 1:
                    write_image(out / "latest.jpg", canvas)
                    record = {"updated_at": datetime.now().astimezone().isoformat(), "hardware_commands_sent": False,
                              "read_only_http": True, "unique_source_frames": {k: len(v) for k, v in observed.items()},
                              "cameras": status}
                    temp = out / "status.tmp"
                    temp.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
                    temp.replace(out / "status.json")
                    last_save = now
                if args.headless:
                    time.sleep(.03)
                else:
                    cv2.imshow(window, canvas)
                    key = cv2.waitKey(20) & 0xff
                    if key in (27, ord('q')) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                        break
                    if key == ord('m'):
                        mask = not mask
    finally:
        for worker in workers:
            worker.stop.set()
        for worker in workers:
            worker.thread.join(timeout=3.5)
        if not args.headless:
            cv2.destroyAllWindows()
    summary = {"stopped": True, "read_only_http": True, "hardware_commands_sent": False,
               "unique_source_frames": {k: len(v) for k, v in observed.items()}, "output": str(out)}
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if all(len(v) >= 2 for v in observed.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
