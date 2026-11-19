"""Manual preview owns cameras only when the C++ task does not."""
import json
import threading
import time


class Preview:
    def __init__(self, config):
        self.config = config
        self.stop_event = threading.Event()
        self.thread = None
        self.frames = {}
        self.error = None
        self.lock = threading.Lock()

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name='manual-preview', daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(3)
            if self.thread.is_alive():
                raise RuntimeError('摄像头尚未释放，未启动自主驾驶')
        with self.lock:
            self.frames.clear()

    def frame(self, camera):
        with self.lock:
            row = self.frames.get(camera)
            return row[1] if row and time.monotonic() - row[0] < 2 else None

    def _run(self):
        captures = {}
        try:
            import cv2
            config = json.loads(self.config.read_text(encoding='utf-8-sig'))
            # Keys are the actual source package camera setting names.
            for name in ('primary', 'secondary'):
                source = config.get(name + '_camera')
                if not source:
                    continue
                capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
                captures[name] = capture
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                capture.set(cv2.CAP_PROP_FPS, 15)
            while not self.stop_event.is_set():
                for name, capture in captures.items():
                    if self.stop_event.is_set():
                        break
                    ok, frame = capture.read()
                    if ok and frame is not None:
                        ok, jpeg = cv2.imencode('.jpg', cv2.resize(frame, (480, 360)),
                                              [cv2.IMWRITE_JPEG_QUALITY, 65])
                        if ok:
                            with self.lock:
                                self.frames[name] = (time.monotonic(), jpeg.tobytes())
                self.stop_event.wait(.08)
        except Exception as exc:
            self.error = str(exc)
        finally:
            for capture in captures.values():
                capture.release()
