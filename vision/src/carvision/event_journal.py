"""Bounded best-effort event journal; disk I/O never runs on a control thread."""
import copy
import json
from pathlib import Path
import queue
import threading
import time


class EventJournal:
    def __init__(self, folder, settings=None, *, capacity=64):
        self.folder=Path(folder)
        self.settings=copy.deepcopy(settings)
        self.queue=queue.Queue(capacity)
        self.stop=threading.Event()
        self.error=None
        self.dropped=0
        self.thread=threading.Thread(target=self._work,name='race-event-journal',daemon=True)
        self.thread.start()

    def submit(self, row):
        if self.stop.is_set() or self.error:
            self.dropped+=1
            return False
        try:self.queue.put_nowait(copy.deepcopy(row))
        except queue.Full:
            self.dropped+=1
            return False
        except Exception as exc:
            self.error=str(exc)
            return False
        return True

    def _work(self):
        try:
            self.folder.mkdir(parents=True,exist_ok=True)
            if self.settings is not None:
                (self.folder/'settings.json').write_text(json.dumps(self.settings,ensure_ascii=False,allow_nan=False,indent=2),encoding='utf-8')
        except Exception as exc:self.error=str(exc)
        while not self.stop.is_set() or not self.queue.empty():
            try:row=self.queue.get(timeout=.05)
            except queue.Empty:continue
            try:
                if self.error:
                    self.dropped+=1
                    continue
                encoded=json.dumps(row,ensure_ascii=False,allow_nan=False)
                if len(encoded)>1024*1024:raise ValueError('event exceeds recording size limit')
                with (self.folder/'events.jsonl').open('a',encoding='utf-8') as out:
                    out.write(encoded+'\n')
                temp=self.folder/'summary.tmp.json'
                temp.write_text(encoded+'\n',encoding='utf-8')
                temp.replace(self.folder/'summary.json')
            except Exception as exc:
                self.error=str(exc);self.dropped+=1
            finally:self.queue.task_done()

    def status(self):
        return {'error':self.error,'dropped':self.dropped,'pending':self.queue.qsize()}

    def flush(self, timeout=.25):
        """Only use after output release, or in offline verification."""
        end=time.monotonic()+timeout
        while self.queue.unfinished_tasks and time.monotonic()<end:time.sleep(.005)
        return not self.queue.unfinished_tasks

    def close(self, timeout=0.):
        self.stop.set()
        if timeout and self.thread is not threading.current_thread():self.thread.join(timeout)
