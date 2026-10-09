"""Real preview-frame collection; no vehicle-control or serial dependencies."""
import copy
import csv
import hashlib
import json
import math
import re
import threading
import time
import uuid
from collections import Counter, OrderedDict
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from .traffic_signal import TrafficSignalDetector,SignalStability,load_profile
from .traffic_preview import CAMERAS

LABELS = ('red', 'yellow', 'green', 'off', 'no_light', 'transition', 'unlabeled')


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def metadata(data):
    result = {}
    for key in ('scene', 'lighting', 'layout', 'notes'):
        value = data.get(key, '')
        if not isinstance(value, str) or len(value) > 300:
            raise ValueError('场景信息必须是300字以内的文字')
        result[key] = value
    distance = data.get('distance_cm')
    if distance is not None and (type(distance) not in (int, float) or not math.isfinite(distance) or not 0 <= distance <= 1000):
        raise ValueError('尺量距离需为0–1000 cm，不确定请留空')
    result.update(distance_cm=distance, distance_source='operator_measurement' if distance is not None else 'unknown',
                  distance_reference='front_wheel_front_to_lamp_ground_projection')
    return result


def normalize_box(box):
    if box is None:
        return None
    if (not isinstance(box, list) or len(box) != 4
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in box)
            or not 0 <= box[0] < box[2] <= 1 or not 0 <= box[1] < box[3] <= 1):
        raise ValueError('标注框必须位于原图范围内且有面积')
    return box


def inspect_jpeg(jpeg,detector=None):
    if not isinstance(jpeg, bytes) or not 0 < len(jpeg) <= 4*1024*1024:
        raise ValueError('JPEG大小无效')
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None or max(image.shape[:2]) > 4096:
        raise ValueError('不是可用的摄像头JPEG')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    observation = (detector or TrafficSignalDetector()).detect(image)
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return {'image_size': [image.shape[1], image.shape[0]],
            'prediction': {'state': observation['state'], 'boxes': [observation['bbox_xyxy']] if observation['bbox_xyxy'] else [],
                           'source': observation['algorithm'], 'observation':observation,'is_ground_truth': False},
            'quality': {'laplacian_variance': float(cv2.Laplacian(grey, cv2.CV_64F).var()),
                        'near_white_fraction': float(((hsv[:,:,1] < 35)&(hsv[:,:,2] >= 245)).mean())}}


class TrafficCollector:
    def __init__(self, output, *, clock=time.monotonic):
        self.root = Path(output).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.clock = clock
        self.signal_profile=load_profile()
        self.signal_detector=TrafficSignalDetector(self.signal_profile)
        self.signal_trackers={c:SignalStability(self.signal_profile) for c in CAMERAS}
        self.lock = threading.RLock()
        self.cache = OrderedDict()
        self.latest = {}
        self.errors = {}
        self.pins = {}
        self.recording = None
        self.last_recording = None
        self.items = OrderedDict()
        self.seen_hashes = set()
        self.data_revision = 0
        self.session_id = None
        self.folder = None
        sessions = self._session_ids()
        if sessions:
            current = None
            try:
                current = json.loads((self.root/'.active-session.json').read_text(encoding='utf-8')).get('session_id')
            except (OSError,ValueError,AttributeError):
                pass
            self.session_id = current if current in sessions else max(
                sessions, key=lambda s: (self._session_folder(s)/'session.json').stat().st_mtime_ns)
            self.folder = self._session_folder(self.session_id)
            self.items = OrderedDict((r['sample_id'],r) for r in self._read_rows(self.session_id))
            self.seen_hashes = {(r['camera'],r['sha256']) for r in self.items.values()}
            self._remember_session()
        else:
            self.new_session()

    def _session_folder(self, session_id):
        if not isinstance(session_id,str) or not re.fullmatch(r'\d{8}T\d{6}Z-[0-9a-f]{8}',session_id):
            raise ValueError('批次编号无效')
        folder=(self.root/session_id).resolve()
        if not folder.is_relative_to(self.root) or not (folder/'session.json').is_file():
            raise ValueError('批次不存在或路径越界')
        return folder

    def _session_ids(self):
        return sorted(p.name for p in self.root.iterdir() if p.is_dir()
                      and re.fullmatch(r'\d{8}T\d{6}Z-[0-9a-f]{8}',p.name)
                      and p.resolve().is_relative_to(self.root) and (p/'session.json').is_file())

    def _remember_session(self):
        temporary=self.root/'.active-session.tmp'
        temporary.write_text(json.dumps({'session_id':self.session_id}),encoding='utf-8')
        temporary.replace(self.root/'.active-session.json')

    def _read_rows(self, session_id):
        folder=self._session_folder(session_id)
        rows=[]
        for path in (folder/'labels').glob('*.json'):
            if not path.resolve().is_relative_to(folder):
                raise ValueError('标签路径越界')
            row=json.loads(path.read_text(encoding='utf-8'))
            sid=row.get('sample_id')
            if (not isinstance(sid,str) or not re.fullmatch(r'[0-9a-f]{32}',sid) or sid!=path.stem
                    or row.get('session_id')!=session_id or row.get('camera') not in CAMERAS
                    or row.get('label') not in LABELS
                    or row.get('image')!=f"images/{row['camera']}-{sid}.jpg"):
                raise ValueError('样本元数据无效：'+path.name)
            image=(folder/row['image']).resolve()
            if not image.is_relative_to(folder) or not image.is_file():
                raise ValueError('原图缺失或路径越界：'+path.name)
            rows.append(row)
        # Windows timestamps may tie across captures. The append-only event
        # log records first insertion order even when later relabels occur.
        order={}
        event_path=folder/'events.jsonl'
        if event_path.is_file() and event_path.resolve().is_relative_to(folder):
            with event_path.open(encoding='utf-8') as stream:
                for line in stream:
                    try:event=json.loads(line)
                    except ValueError:continue
                    sample_id=event.get('sample_id') if isinstance(event,dict) else None
                    if isinstance(sample_id,str) and sample_id not in order:order[sample_id]=len(order)
        return sorted(rows,key=lambda r:(0,order[r['sample_id']]) if r['sample_id'] in order
                      else (1,r.get('saved_utc',''),r['sample_id']))

    def _rows(self, session_id):
        self._session_folder(session_id)
        return list(self.items.values()) if session_id==self.session_id else self._read_rows(session_id)

    def sessions(self):
        with self.lock:
            result=[]
            for sid in reversed(self._session_ids()):
                folder=self._session_folder(sid)
                info=json.loads((folder/'session.json').read_text(encoding='utf-8'))
                result.append({'session_id':sid,'created_utc':info.get('created_utc'),
                               'count':len(self.items) if sid==self.session_id else len(list((folder/'labels').glob('*.json'))),
                               'active_capture':sid==self.session_id})
            return result

    def samples(self, session_id=None, page=1, page_size=16, label='all', camera='all'):
        with self.lock:
            if type(page) is not int or page<1 or type(page_size) is not int or not 1<=page_size<=100:
                raise ValueError('页码应为正整数，每页1–100张')
            if label not in ('all',*LABELS) or camera not in ('all',*CAMERAS):
                raise ValueError('筛选条件无效')
            sid=session_id or self.session_id
            all_rows=self._rows(sid)
            rows=[r for r in all_rows if (label=='all' or r['label']==label)
                  and (camera=='all' or r['camera']==camera)]
            pages=max(1,math.ceil(len(rows)/page_size)); page=min(page,pages)
            start=(page-1)*page_size
            return {'session_id':sid,'total':len(rows),'session_total':len(all_rows),'page':page,'pages':pages,
                    'page_size':page_size,'first':start+1 if rows else 0,'last':min(start+page_size,len(rows)),
                    'items':copy.deepcopy(rows[start:start+page_size]),'data_revision':self.data_revision,
                    'counts':dict(Counter(r['label'] for r in all_rows))}

    def sample_bytes(self, session_id, sample_id):
        with self.lock:
            row=next((r for r in self._rows(session_id or self.session_id) if r['sample_id']==sample_id),None)
            if row is None:raise ValueError('样本不存在')
            folder=self._session_folder(row['session_id'])
            path=(folder/row['image']).resolve()
            if not path.is_relative_to(folder):raise ValueError('原图路径越界')
            return path.read_bytes()

    def new_session(self):
        with self.lock:
            if self.recording:
                raise ValueError('先结束序列采集，再新建批次')
            if self.folder is not None:
                self.export()
            self.session_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
            self.folder = self.root/self.session_id
            (self.folder/'images').mkdir(parents=True, exist_ok=False)
            (self.folder/'labels').mkdir()
            (self.folder/'session.json').write_text(json.dumps({
                'schema_version': 1, 'session_id': self.session_id, 'created_utc': utc_now(),
                'image_source': 'car HTTP raw-preview JPEG, not native full-resolution camera capture',
                'control_commands_sent': False, 'two_cameras_synchronized': False,
                'split_policy': 'Keep an entire physical scene/session in one train/validation split.'
            },ensure_ascii=False,indent=2), encoding='utf-8')
            self.items.clear(); self.seen_hashes.clear(); self.pins.clear()
            for tracker in self.signal_trackers.values():tracker.reset()
            self.data_revision += 1
            self._remember_session()
            return self.session_id

    def publish(self, camera, jpeg, source_status):
        if camera not in CAMERAS:
            raise ValueError('未知摄像头')
        detail = inspect_jpeg(jpeg,self.signal_detector)
        with self.lock:
            now = self.clock()
            frame_id = camera+'-'+uuid.uuid4().hex
            identity=source_status.get('status_frame_id_hint')
            stable=self.signal_trackers[camera].update(detail['prediction']['observation'],
                identity if identity is not None else frame_id,now,now)
            detail['prediction'].update(stability=stable,
                recommended_source=camera==self.signal_profile['recommended_camera'],motion_authorization=False)
            frame = {'id': frame_id, 'camera': camera, 'jpeg': jpeg, 'received_s': now,
                     'received_utc': utc_now(), 'sha256': hashlib.sha256(jpeg).hexdigest(),
                     'source': copy.deepcopy(source_status), **detail}
            self.cache[frame_id] = frame
            while len(self.cache) > 160:
                self.cache.popitem(last=False)
            self.latest[camera] = frame
            self.errors.pop(camera, None)
            self._record_frame(frame)
            return frame_id

    def error(self, camera, message):
        with self.lock:
            self.errors[camera] = str(message)
            self.signal_trackers[camera].reset()

    def frame_bytes(self, frame_id):
        with self.lock:
            frame = self.cache.get(frame_id)
            if frame is None:
                for pin in self.pins.values():
                    frame = next((f for f in pin['frames'].values() if f['id'] == frame_id), None)
                    if frame: break
            if frame is None: raise ValueError('帧已过期，请恢复实时画面')
            return frame['jpeg']

    def freeze(self, ids):
        with self.lock:
            now = self.clock()
            self.pins = {k:v for k,v in self.pins.items() if now-v['created_s'] < 300}
            if not isinstance(ids,dict) or not ids or any(k not in CAMERAS for k in ids):
                raise ValueError('先等待真实画面载入')
            frames = {}
            for camera, frame_id in ids.items():
                frame = self.cache.get(frame_id)
                if not frame or frame['camera'] != camera or now-frame['received_s'] > 3 or camera in self.errors:
                    raise ValueError('画面过期或断流，请重新取图')
                frames[camera] = frame
            if len(self.pins) >= 16:
                self.pins.pop(next(iter(self.pins)))
            token = uuid.uuid4().hex
            self.pins[token] = {'frames':frames, 'created_s':now}
            return token

    def capture(self, token, camera, label, box=None, meta=None):
        with self.lock:
            pin = self.pins.get(token)
            if not pin or self.clock()-pin['created_s'] > 300:
                raise ValueError('冻结画面已失效，请恢复实时后重拍')
            if camera not in pin['frames']:
                raise ValueError('请先冻结所选摄像头')
            return self._save(pin['frames'][camera],label,normalize_box(box),metadata(meta or {}),None)

    def _save(self, frame, label, box, meta, sequence_id):
        if label not in LABELS:
            raise ValueError('标签无效')
        if label == 'no_light' and box is not None:
            raise ValueError('无灯负样本不应有灯体框，请先清除框')
        signature = (frame['camera'],frame['sha256'])
        if signature in self.seen_hashes and sequence_id is None:
            raise ValueError('这张相同画面已经保存；可在最近采集中改标签')
        sample_id = uuid.uuid4().hex
        image_rel = f'images/{frame["camera"]}-{sample_id}.jpg'
        w,h = frame['image_size']
        row = {k: copy.deepcopy(v) for k,v in frame.items() if k not in ('jpeg','received_s')}
        row.update(sample_id=sample_id, session_id=self.session_id, saved_utc=utc_now(),
                   image=image_rel, label=label, label_source='operator' if label!='unlabeled' else 'unreviewed',
                   bbox_normalized_xyxy=box,
                   bbox_xyxy=[round(box[0]*w),round(box[1]*h),round(box[2]*w),round(box[3]*h)] if box else None,
                   annotation_status='unreviewed' if label=='unlabeled' else 'box_and_state' if box else 'state_only',
                   metadata=meta, sequence_id=sequence_id, hardware_output=False,
                   camera_pairing='independent_frames_not_synchronized',
                   same_pixels_seen_before=signature in self.seen_hashes)
        (self.folder/image_rel).write_bytes(frame['jpeg'])
        self._write_label(row)
        self.items[sample_id] = row
        self.seen_hashes.add(signature)
        return copy.deepcopy(row)

    def _write_label(self, row, folder=None):
        folder=folder or self.folder
        path = folder/'labels'/(row['sample_id']+'.json')
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(row,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        temporary.replace(path)
        with (folder/'events.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps({'event':'annotation_saved','sample_id':row['sample_id'],'label':row['label'],'time_utc':utc_now()},ensure_ascii=False)+'\n')
        self.data_revision += 1

    def relabel(self, sample_id, label, session_id=None):
        with self.lock:
            sid=session_id or self.session_id
            original=next((r for r in self._rows(sid) if r['sample_id']==sample_id),None)
            if original is None or label not in LABELS:
                raise ValueError('样本或标签不存在')
            row = copy.deepcopy(original)
            if label == 'no_light' and row['bbox_xyxy'] is not None:
                raise ValueError('有灯体框的样本不能直接改为无灯')
            row.update(label=label,label_source='operator' if label!='unlabeled' else 'unreviewed',
                       annotation_status='unreviewed' if label=='unlabeled' else 'box_and_state' if row['bbox_xyxy'] else 'state_only')
            self._write_label(row,self._session_folder(sid))
            if sid==self.session_id:self.items[sample_id] = row

    def start_recording(self, cameras, seconds=20, fps=3, meta=None):
        with self.lock:
            if self.recording: raise ValueError('序列正在采集')
            if (not isinstance(cameras,list) or not cameras or len(set(cameras))!=len(cameras)
                    or any(c not in CAMERAS for c in cameras)):
                raise ValueError('摄像头选择无效')
            if type(seconds) not in (int,float) or not 1<=seconds<=60 or type(fps) not in (int,float) or not 1<=fps<=5:
                raise ValueError('序列长度1–60秒，保存频率1–5帧/秒')
            self.recording = {'id':uuid.uuid4().hex,'cameras':cameras,'started_s':self.clock(),
                              'seconds':seconds,'fps':fps,'last_saved':{},'saved':0,'identical_observations':0,
                              'metadata':metadata(meta or {})}
            return self.recording['id']

    def _record_frame(self, frame):
        r = self.recording
        if not r: return
        now = self.clock()
        if now-r['started_s'] >= r['seconds']:
            self.stop_recording('duration_reached'); return
        camera = frame['camera']
        if camera not in r['cameras'] or now-r['last_saved'].get(camera,-1e9) < 1/r['fps']:
            return
        r['last_saved'][camera] = now
        if (camera,frame['sha256']) in self.seen_hashes:
            r['identical_observations'] += 1
        self._save(frame,'unlabeled',None,r['metadata'],r['id'])
        r['saved'] += 1

    def stop_recording(self, reason='operator_stopped'):
        with self.lock:
            if self.recording:
                r = self.recording
                report = {k:v for k,v in r.items() if k != 'last_saved'}
                report.update(reason=reason,elapsed_s=self.clock()-r['started_s'],labels='unreviewed',hardware_output=False)
                (self.folder/('sequence-'+r['id']+'.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
                self.last_recording = report; self.recording = None

    def status(self):
        with self.lock:
            if self.recording and self.clock()-self.recording['started_s'] >= self.recording['seconds']:
                self.stop_recording('duration_reached')
            cameras = {}
            for camera in CAMERAS:
                f = self.latest.get(camera)
                cameras[camera] = {'ok':bool(f and camera not in self.errors and self.clock()-f['received_s'] <= 3),
                                   'error':self.errors.get(camera),
                                   'signal':self.signal_trackers[camera].current(self.clock()),
                                   'frame':{k:v for k,v in f.items() if k not in ('jpeg','received_s')} if f else None}
            return {'app':'traffic-light-capture','hardware_output':False,'session_id':self.session_id,
                    'traffic_algorithm':self.signal_profile['algorithm'],'traffic_camera':self.signal_profile['recommended_camera'],
                    'output':str(self.folder),'cameras':cameras,'saved':len(self.items),
                    'counts':dict(Counter(x['label'] for x in self.items.values())),
                    'data_revision':self.data_revision,
                    'recording':copy.deepcopy(self.recording),'last_recording':self.last_recording,
                    'recent':list(self.items.values())[-12:][::-1]}

    def export(self, session_id=None):
        with self.lock:
            sid=session_id or self.session_id
            rows=self._rows(sid)
            path = self._session_folder(sid)/'manifest.csv'
            with path.open('w',newline='',encoding='utf-8-sig') as f:
                keys=['sample_id','session_id','camera','image','label','annotation_status','sequence_id','received_utc']
                writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader()
                writer.writerows({k:x.get(k) for k in keys} for x in rows)
            return str(path)
