"""Offline sources process every frame. Camera uses a latest-frame slot.

receive_monotonic_ms is host receipt time, NOT camera exposure time.
"""

import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}


@dataclass
class Frame:
    image: np.ndarray
    frame_id: int
    source_time_ms: float | None
    receive_monotonic_ms: float


def read_image(path):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot decode image: {path}")
    return image


def write_image(path, image):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, data = cv2.imencode(path.suffix, image)
    if not ok:
        raise RuntimeError(f"cannot encode image: {path}")
    data.tofile(str(path))


class FileSource:
    def __init__(self, path, fallback_fps=None):
        self.path = Path(path)
        self.cap = None
        if not self.path.is_file():
            raise FileNotFoundError(self.path)
        self.is_image = self.path.suffix.lower() in IMAGE_EXTENSIONS
        self.fps = None
        self.timestamp_basis = "single_image" if self.is_image else "frame_index/fps"
        self.end_status = "not_started"
        if not self.is_image:
            self.cap = cv2.VideoCapture(str(self.path))
            if not self.cap.isOpened():
                self.cap.release()
                raise ValueError(f"cannot open video: {self.path}")
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.fps = float(fps) if np.isfinite(fps) and fps > 0 else fallback_fps
            if self.fps is None or not np.isfinite(self.fps) or self.fps <= 0:
                self.cap.release()
                raise ValueError("video has no valid FPS; supply --fallback-fps explicitly")

    def __iter__(self):
        self.end_status = "reading"
        if self.is_image:
            yield Frame(read_image(self.path), 0, None, time.monotonic() * 1000)
            self.end_status = "completed_image"
            return
        count = 0
        try:
            while True:
                ok, image = self.cap.read()
                if not ok or image is None:
                    # OpenCV does not reliably distinguish EOF from decoder failure.
                    self.end_status = "eof_or_decode_failure"
                    if count == 0:
                        raise ValueError(f"video contains no decodable frames: {self.path}")
                    return
                yield Frame(image, count, count * 1000 / self.fps, time.monotonic() * 1000)
                count += 1
        finally:
            self.close()

    def close(self):
        if self.cap is not None:
            self.cap.release()


class CameraSource:
    def __init__(self, index=0, width=None, height=None, fps=None, timeout_s=3.0):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            self.cap.release()
            raise ValueError(f"cannot open camera index {index}")
        self.request_results = {}
        for name, prop, value in [("width", cv2.CAP_PROP_FRAME_WIDTH, width),
                                  ("height", cv2.CAP_PROP_FRAME_HEIGHT, height),
                                  ("fps", cv2.CAP_PROP_FPS, fps)]:
            if value is not None:
                self.request_results[name] = bool(self.cap.set(prop, value))
        self.reported = {"width": self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                         "height": self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                         "fps": self.cap.get(cv2.CAP_PROP_FPS)}
        self.fps = self.reported["fps"] or None
        self.timestamp_basis = "host_receive_monotonic"
        self.timeout_s = timeout_s
        self.end_status = "reading"
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._latest = None
        self._error = None
        self._thread = threading.Thread(target=self._read, name="camera-reader", daemon=True)
        self._thread.start()

    def _read(self):
        seq = 0
        try:
            while not self._stop.is_set():
                ok, image = self.cap.read()
                if not ok or image is None:
                    raise RuntimeError("camera read failed; previous frame is not reused")
                frame = Frame(image, seq, None, time.monotonic() * 1000)
                with self._condition:
                    self._latest = frame
                    self._condition.notify_all()
                seq += 1
        except Exception as exc:
            with self._condition:
                self._error = exc
                self._condition.notify_all()
        finally:
            self.cap.release()

    def __iter__(self):
        last = -1
        while not self._stop.is_set():
            with self._condition:
                ready = self._condition.wait_for(
                    lambda: self._error is not None or self._stop.is_set() or
                    (self._latest is not None and self._latest.frame_id > last), self.timeout_s)
                if self._stop.is_set():
                    return
                if self._error:
                    self.end_status = "read_error"
                    raise self._error
                if not ready:
                    self.end_status = "timeout"
                    raise TimeoutError("camera did not deliver a fresh frame")
                frame = self._latest
                last = frame.frame_id
            yield frame

    def close(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        # A blocked backend read may outlive this join; daemon prevents an exit hang.
        self._thread.join(timeout=1)


def open_source(source, **kwargs):
    if source.startswith("camera:"):
        return CameraSource(int(source.split(":", 1)[1]),
                            **{k: v for k, v in kwargs.items() if k != "fallback_fps"})
    return FileSource(source, kwargs.get("fallback_fps"))
