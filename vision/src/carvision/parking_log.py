"""Bounded asynchronous journal: SD-card latency must not delay PWM leases."""
import queue
import threading
import json


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


class ParkingFrames:
    """Bounded raw-frame journal; recording never blocks the control loop."""
    def __init__(self, folder, max_bytes=128*1024*1024):
        self.folder=folder
        self.max_bytes=max_bytes
        self.pending=queue.Queue(maxsize=16)
        self.stopping=threading.Event()
        self.error=None;self.dropped=0;self.written=0;self.bytes=0
        self.thread=threading.Thread(target=self._run,name='parking-frames',daemon=True)
        self.thread.start()

    def add(self,jpeg,frame_id,captured_s):
        if self.stopping.is_set() or self.error:return
        try:self.pending.put_nowait((jpeg,frame_id,captured_s))
        except queue.Full:self.dropped+=1

    def _run(self):
        try:
            with (self.folder/'frames.mjpg').open('wb') as video, (self.folder/'frames.jsonl').open('w',encoding='utf8') as index:
                while not self.stopping.is_set() or not self.pending.empty():
                    try:jpeg,fid,t=self.pending.get(timeout=.05)
                    except queue.Empty:continue
                    if self.bytes+len(jpeg)>self.max_bytes:
                        self.dropped+=1;continue
                    video.write(jpeg)
                    index.write(json.dumps({'camera':'secondary','frame_id':fid,'captured_s':t,
                        'offset':self.bytes,'length':len(jpeg)})+'\n')
                    video.flush();index.flush()
                    self.bytes+=len(jpeg);self.written+=1
        except Exception as exc:self.error=str(exc)
        finally:
            (self.folder/'recording.json').write_text(json.dumps({
                'written':self.written,'dropped':self.dropped,'bytes':self.bytes,
                'error':self.error,'max_bytes':self.max_bytes}),encoding='utf8')

    def close(self):
        self.stopping.set()
        if self.thread is not threading.current_thread():self.thread.join(timeout=1)
