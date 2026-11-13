"""Bounded recording of published JPEGs and cone decisions. Author: 13246.

Uses existing camera publications, never opens a camera or actuator. Timestamps
are host receive/decision times, not exposure times or measured wheel angles.
"""
import copy
import hashlib
import json
import math
from collections import Counter, deque
from pathlib import Path
import queue
import re
import secrets
import threading
import time


CAMERAS = ('primary', 'secondary')
RUN_ID = re.compile(r'cone-recording-[0-9]+-[a-f0-9]{6}\Z')


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))+'\n').encode('utf8')


class RecordingSession:
    """Producer work is bounded; only the worker accesses recording files."""
    def __init__(self, folder, metadata, *, clock=time.monotonic, capacity=64,
                 queue_bytes=8*1024*1024, max_bytes=512*1024*1024):
        self.folder=Path(folder);self.clock=clock;self.origin=clock()
        self.metadata=copy.deepcopy(metadata)
        self.queue=queue.Queue(capacity);self.queue_limit=queue_bytes;self.max_bytes=max_bytes
        self.lock=threading.Lock();self.stop=threading.Event();self.done=threading.Event()
        self.pending_bytes=0;self.bytes_written=0;self.seen=Counter();self.written=Counter()
        self.dropped=Counter();self.gaps=deque(maxlen=128);self.error='';self.reason=''
        self.finalized=False;self.accepting=True
        self.worker=threading.Thread(target=self._work,name='cone-recording-writer',daemon=True)
        self.worker.start()

    def _drop(self, kind, reason, detail=None):
        self.dropped[kind]+=1
        self.gaps.append({'kind':kind,'reason':reason,'detail':detail})

    def _enqueue(self, kind, payload, detail=None, legacy=None):
        with self.lock:
            if self.stop.is_set():return False
            self.seen[kind]+=1
            if not self.accepting or self.error:
                self._drop(kind,self.error or 'closed',detail);return False
            if len(payload)>1024*1024 or self.pending_bytes+len(payload)>self.queue_limit:
                self._drop(kind,'queue_byte_limit',detail);return False
            try:self.queue.put_nowait((kind,payload,detail,legacy))
            except queue.Full:
                self._drop(kind,'queue_full',detail);return False
            self.pending_bytes+=len(payload)
            return True

    def frame(self, camera, jpg, fid, received_s):
        if (camera not in CAMERAS or not isinstance(jpg,bytes) or not jpg or type(fid) is not int
                or type(received_s) not in (int,float) or not math.isfinite(received_s)):
            with self.lock:self.seen['frame']+=1;self._drop('frame','invalid_frame')
            return False
        row={'camera':camera,'frame_id':fid,'received_s':received_s}
        return self._enqueue('frame',jpg,row)

    def event(self, kind, data, *, legacy=None):
        try:
            payload=encoded({'kind':kind,'recorded_s':self.clock(),'data':data})
            return self._enqueue('event',payload,{'event_kind':kind},legacy)
        except (ValueError,TypeError,OverflowError) as exc:
            with self.lock:
                self.seen['event']+=1;self._drop('event','serialization_error',str(exc))
            return False

    def status(self):
        with self.lock:
            return {'id':self.folder.name,'active':not self.stop.is_set(),
                'accepting':self.accepting and not self.error,'finalized':self.finalized,
                'seen':dict(self.seen),'written':dict(self.written),'dropped':dict(self.dropped),
                'error':self.error,'reason':self.reason,'bytes_written':self.bytes_written,
                'max_bytes':self.max_bytes,'queued_bytes':self.pending_bytes,
                'complete':self.finalized and not self.error and not any(self.dropped.values())}

    def finish(self, reason):
        # No join or file I/O on a control/HTTP thread.
        with self.lock:
            if not self.stop.is_set():self.reason=reason;self.accepting=False;self.stop.set()

    def _write_item(self, item, archive, index, events, journals):
        kind,payload,detail,legacy=item
        if kind=='frame':
            row={**detail,'offset':archive.tell(),'length':len(payload)}
            idx=encoded(row);size=len(payload)+len(idx)
            if self.bytes_written+size>self.max_bytes:raise ValueError('recording_size_limit')
            archive.write(payload);index.write(idx)
        else:
            old=None
            if legacy:
                old=encoded(json.loads(payload)['data'])
            size=len(payload)+(len(old) if old else 0)
            if self.bytes_written+size>self.max_bytes:raise ValueError('recording_size_limit')
            events.write(payload)
            if legacy:
                if legacy not in journals:journals[legacy]=Path(legacy).open('xb')
                journals[legacy].write(old)
        with self.lock:self.bytes_written+=size;self.written[kind]+=1

    def _work(self):
        handles=[];journals={}
        try:
            self.folder.mkdir(parents=True,exist_ok=False)
            (self.folder/'recording.json').write_bytes(encoded({**self.status(),
                'origin_s':self.origin,'metadata':self.metadata,'finalized':False}))
            archive=(self.folder/'frames.mjpg').open('xb');handles.append(archive)
            index=(self.folder/'frames.jsonl').open('xb');handles.append(index)
            events=(self.folder/'events.jsonl').open('xb');handles.append(events)
            while not self.stop.is_set() or not self.queue.empty():
                try:item=self.queue.get(timeout=.05)
                except queue.Empty:continue
                try:
                    if self.error:
                        with self.lock:self._drop(item[0],self.error,item[2])
                    else:self._write_item(item,archive,index,events,journals)
                except Exception as exc:
                    with self.lock:
                        self.error=str(exc);self._drop(item[0],self.error,item[2])
                finally:
                    with self.lock:self.pending_bytes-=len(item[1])
                    self.queue.task_done()
        except Exception as exc:
            with self.lock:self.error=str(exc)
        finally:
            for handle in handles+list(journals.values()):
                try:handle.close()
                except Exception as exc:
                    with self.lock:self.error=self.error or str(exc)
            # Retain a finite error record even when the destination failed at startup.
            while True:
                try:item=self.queue.get_nowait()
                except queue.Empty:break
                with self.lock:
                    self.pending_bytes-=len(item[1]);self._drop(item[0],self.error or 'writer_ended',item[2])
                self.queue.task_done()
            with self.lock:self.accepting=False
            try:
                from .cone_replay import build_replay_data
                data=build_replay_data(self.folder,self.origin)
                (self.folder/'replay-data.json').write_bytes(encoded(data))
                with self.lock:
                    gaps=list(self.gaps);truncated=sum(self.dropped.values())>len(gaps)
                    complete=not self.error and not any(self.dropped.values())
                temp=self.folder/'recording.tmp.json'
                temp.write_bytes(encoded({**self.status(),'finalized':True,'complete':complete,
                    'origin_s':self.origin,'metadata':self.metadata,'recent_gaps':gaps,
                    'gap_details_truncated':truncated,
                    'scope':'JPEGs published by the existing preview pipeline during this session',
                    'physical_stop_verified':False}))
                temp.replace(self.folder/'recording.json')
                with self.lock:self.finalized=True
            except Exception as exc:
                with self.lock:self.error=self.error or str(exc);self.finalized=False
            self.done.set()


class ConeRecording:
    def __init__(self, root, states, clock=time.monotonic):
        self.root=Path(root);self.states=states;self.clock=clock;self.current=None
        self.error='';self.taps=[];self.sessions=[]
        # Per-instance tap composes with the existing payment/camera publisher.
        # It never replaces camera ownership, frame rate or resolution.
        for camera,state in states.items():
            if camera not in CAMERAS or not callable(getattr(state,'publish_raw',None)):continue
            original=state.publish_raw
            def tapped(jpeg,frame,_original=original,_state=state,_camera=camera):
                result=_original(jpeg,frame)
                session=self.current
                if session is not None and not session.stop.is_set():
                    try:
                        if (_state.raw_frame_id==frame.frame_id and
                                _state.raw_received_ms==frame.receive_monotonic_ms):
                            session.frame(_camera,jpeg,frame.frame_id,frame.receive_monotonic_ms/1000.)
                    except Exception as exc:self.error=str(exc)
                return result
            state.publish_raw=tapped;self.taps.append((state,original,tapped,camera))

    def start(self, settings):
        if self.current is not None and not self.current.stop.is_set() and not self.current.done.is_set():return
        options=settings.get('latest_recording',{})
        if options.get('enabled',True) is False:
            self.current=None;return
        try:
            root=Path(__file__).parent
            hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (root/'cone_latest_engine').glob('*.py')}
            name='cone-recording-'+str(time.time_ns())+'-'+secrets.token_hex(3)
            self.current=RecordingSession(self.root/name,{'author':'13246','settings':settings,
                'engine_sha256':hashes,'tapped_cameras':[t[3] for t in self.taps],
                'output_semantics':'command dispatched to existing worker; not physical wheel-angle feedback'},
                clock=self.clock,max_bytes=options.get('max_mb',512)*1024*1024)
            self.sessions=[s for s in self.sessions if not s.done.is_set()]+[self.current]
            self.error=''
        except Exception as exc:self.error=str(exc)

    def event(self, kind, data, *, legacy=None):
        session=self.current
        if session is not None and not session.stop.is_set():session.event(kind,data,legacy=legacy)

    def finish(self, reason):
        if self.current is not None:self.current.finish(reason)

    def status(self):
        result=self.current.status() if self.current else {'active':False,'finalized':False}
        result['tap_error']=self.error
        result['replay_url']=('/api/cone-latest/replay.html?run='+result['id']) if result.get('finalized') else None
        return result

    def close(self):
        self.finish('service_closed')
        for state,original,tapped,_ in self.taps:
            if state.publish_raw is tapped:state.publish_raw=original
        deadline=time.monotonic()+2
        for session in self.sessions:session.done.wait(max(0,deadline-time.monotonic()))
