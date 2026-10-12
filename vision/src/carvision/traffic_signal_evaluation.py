"""Reproducible development replay against untouched operator labels."""
import hashlib
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from .traffic_signal import TrafficSignalDetector,load_profile


def evaluate(data_root,output,profile_path=None):
    root=Path(data_root).resolve();profile=load_profile(profile_path)
    out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False)
    detector=TrafficSignalDetector(profile);rows=[];hash_sessions={};manifest=[]
    sessions=profile.get('development_sessions',[])
    if not sessions:raise ValueError('explicit development sessions are required')
    for session in sessions:
        folder=(root/session).resolve()
        if not folder.is_relative_to(root) or not (folder/'session.json').is_file():raise ValueError('invalid session')
        for label_file in sorted((folder/'labels').glob('*.json')):
            r=json.loads(label_file.read_text(encoding='utf-8'))
            if r.get('label') not in ('red','yellow','green','off','no_light'):continue
            path=(folder/r['image']).resolve()
            if not path.is_relative_to(folder):raise ValueError('image outside dataset session')
            raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
            if digest!=r['sha256']:raise ValueError('image hash mismatch: '+r['sample_id'])
            image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            d=detector.detect(image)
            predicted=d['state'] if d['fixture_detected'] else 'no_light'
            rows.append({'session_id':session,'sample_id':r['sample_id'],'camera':r['camera'],'label':r['label'],
                         'predicted':predicted,'correct':predicted==r['label'],'result':d})
            manifest.append({'session_id':session,'sample_id':r['sample_id'],'label':r['label'],
                             'image_sha256':digest,'label_sha256':hashlib.sha256(label_file.read_bytes()).hexdigest()})
            hash_sessions.setdefault(digest,set()).add(session)
    def metrics(selected):
        colors=[r for r in selected if r['label']=='green']
        return {'images':len(selected),'correct':sum(r['correct'] for r in selected),
                'accuracy':sum(r['correct'] for r in selected)/len(selected) if selected else None,
                'false_green':sum(r['predicted']=='green' and r['label']!='green' for r in selected),
                'green_recall':sum(r['predicted']=='green' for r in colors)/len(colors) if colors else None,
                'confusion':dict(Counter(r['label']+' -> '+r['predicted'] for r in selected)),
                'processing_ms_median':float(np.median([r['result']['processing_ms'] for r in selected])) if selected else None}
    report={'scope':'development_replay_not_independent_test','all':metrics(rows),
            'by_camera':{c:metrics([r for r in rows if r['camera']==c]) for c in ('primary','secondary')},
            'by_session':{s:metrics([r for r in rows if r['session_id']==s]) for s in sessions},
            'duplicate_hashes_crossing_sessions':sum(len(v)>1 for v in hash_sessions.values()),
            'recommended_camera':profile['recommended_camera'],'field_verified':False,'hardware_output':False,
            'warning':'These indoor scenes were used to develop the algorithm. Collect unseen outdoor sessions before a field accuracy claim.'}
    for name,data in [('report.json',report),('predictions.json',rows),('manifest.json',manifest),('profile.json',profile)]:
        (out/name).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    return report
