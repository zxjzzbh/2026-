"""Bounded asynchronous journal: SD-card latency must not delay PWM leases."""
import queue
import threading


class ParkingLog:
    def __init__(self, path):
        self.file = path.open('w', encoding='utf-8')
        self.pending = queue.Queue(maxsize=256)
        self.closed = False
        self.error = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, name='parking-entry-log', daemon=True)
        self.thread.start()

    def write(self, text):
        if self.closed or self.error:
            raise OSError('parking decision log unavailable: ' + str(self.error or 'closed'))
        try:
            self.pending.put_nowait(text)
        except queue.Full:
            raise OSError('parking decision log queue full') from None

    def flush(self):
        # The writer flushes each batch; control must never wait for the disk.
        if self.error:
            raise OSError('parking decision log failed: ' + self.error)

    def _run(self):
        try:
            while not self.stopping.is_set() or not self.pending.empty():
                try:
                    text = self.pending.get(timeout=.05)
                except queue.Empty:
                    continue
                self.file.write(text)
                self.file.flush()
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.file.close()

    def close(self):
        self.closed = True
        self.stopping.set()
        if self.thread is not threading.current_thread():
            self.thread.join(timeout=1)
