"""Record every post-warmup camera frame for desktop playback."""
import json
import queue
import threading
import time

import cv2
from camera_stream import LatestFrameCamera


class RecordingCamera(LatestFrameCamera):
    def __init__(self, folder):
        super().__init__()
        self.folder = folder
        self.pending = queue.Queue(maxsize=100)
        self.error = None
        self.saved = 0
        self.writer = threading.Thread(target=self._save_frames, daemon=True)

    def start(self):
        self.writer.start()
        return super().start()

    def _reader(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(.005)
                continue
            ts = time.monotonic()
            with self.lock:
                self.frame, self.timestamp = frame, ts
                self.frame_id += 1
            if self.error is None:
                try:
                    self.pending.put_nowait((self.frame_id, ts, frame))
                except queue.Full:
                    self.error = 'Recording queue full'

    def _save_frames(self):
        try:
            with (self.folder/'parking_camera.mjpg').open('wb') as video, (self.folder/'parking_camera.jsonl').open('w') as index:
                while True:
                    item = self.pending.get()
                    if item is None:
                        break
                    fid, ts, frame = item
                    ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if not ok:
                        raise IOError('JPEG encoding failed')
                    size = video.write(jpg.tobytes())
                    index.write(json.dumps([fid, ts, size])+'\n')
                    self.saved += 1
        except Exception as exc:
            self.error = str(exc)

    def stop(self):
        super().stop()
        while self.writer.is_alive():
            try:
                self.pending.put(None, timeout=.2)
                break
            except queue.Full:
                continue
        if self.writer.ident is not None:
            self.writer.join()
        result = dict(frames=self.saved, complete=bool(self.saved and self.saved==self.frame_id and not self.error), error=self.error)
        (self.folder/'parking_camera.json').write_text(json.dumps(result))
        print('[video]', result, flush=True)
