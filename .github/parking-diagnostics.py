"""Temporary cross-platform replay probe; no actuator is constructed."""
import json
import sys
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision/tests'))
from parking_replay_support import replay

print('PARKING_ENV '+json.dumps({'cv2':cv2.__version__,'numpy':np.__version__,
    'cpu':cv2.getCPUFeaturesLine(),'threads':cv2.getNumThreads()}),flush=True)
for name in ('parking-bay-pose-20261108','parking-reacquire-20261108'):
    for row in replay(ROOT/'vision/tests/fixtures'/name,staged=True):
        intent=row['intent'];obs=row['sample']['parking']
        print('PARKING_TRACE '+json.dumps({'dataset':name,'fid':row['frame_id'],
            'phase':intent.get('phase'),'reason':intent.get('reason'),'esc':intent.get('esc_us'),
            'progress':intent.get('entry_progress'),
            'geometry':intent.get('geometry'),
            'tracks':obs.get('tracked_bay_boundaries'),
            'landmarks':obs.get('bay_landmarks')},ensure_ascii=True,default=str),flush=True)
