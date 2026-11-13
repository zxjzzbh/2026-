"""One-slot preview encoder: slow viewers never queue old control frames."""
import threading
import cv2


class PreviewEncoder:
    def __init__(self):
        self.condition = threading.Condition()
        self.pending = None
        self.latest = None
        self.closed = False
        self.thread = None

    def submit(self, image):
        with self.condition:
            if self.closed:
                return None
            self.pending = image.copy()
            if self.thread is None:
                self.thread = threading.Thread(target=self._work, name='preview-encoder', daemon=True)
                self.thread.start()
            self.condition.notify()
            return self.latest

    def _work(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                image, self.pending = self.pending, None
            try:
                ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                result = jpeg.tobytes() if ok else None
            except cv2.error:
                result = None
            with self.condition:
                self.latest = result
                self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()
        if self.thread:
            self.thread.join(timeout=.5)
