"""Save an image reference only after the operator supplies a measured distance.

Reads camera GET endpoints, never drives the car. Not a metric camera calibration.
"""
import argparse
import hashlib
import json
import sys
import time
import statistics
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from carvision.crosswalk import CrosswalkDetector
from carvision.crosswalk_trial import validate_reference
from watch_remote_crosswalk import NoRedirect, ProxyHandler, build_opener, get_bytes


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', required=True)
    p.add_argument('--camera', choices=['primary','secondary'], required=True)
    p.add_argument('--measured-front-distance-m', type=float, required=True)
    p.add_argument('--parking-target-distance-m', type=float, default=.15,
                   help='desired final stop distance, distinct from measured brake trigger')
    p.add_argument('--confirm-measured', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    import re
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]*',args.host):
        p.error('invalid host')
    if (not args.confirm_measured or not 0 < args.parking_target_distance_m < .3
            or not args.parking_target_distance_m <= args.measured_front_distance_m <= 1.5):
        p.error('先用尺子测量车头至斑马线前缘距离，保持静止后明确确认')
    op = build_opener(ProxyHandler({}),NoRedirect())
    base = f'http://{args.host}:8080'
    status = json.loads(get_bytes(op,base+'/api/drive/status',262144))
    if status.get('motor_pulse_us') not in (None,1500) or status.get('motor_direction') not in (None,'stop'):
        raise ValueError('车辆仍有运动指令，不能采集停车参考')
    samples=[]
    last_id=None
    for _ in range(7):
        raw = json.loads(get_bytes(op,base+'/api/status?camera='+args.camera,262144)).get('raw_preview',{})
        if raw.get('state')!='ok' or not 0 <= raw.get('host_frame_age_ms',99999) <= 1000:
            raise ValueError('camera is not fresh')
        if raw.get('frame_id')==last_id:
            time.sleep(.1)
            continue
        last_id=raw.get('frame_id')
        data = get_bytes(op,base+'/frame.jpg?camera='+args.camera+'&view=raw',4194304)
        image = cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('invalid JPEG')
        zebra,_ = CrosswalkDetector().detect(image)
        if zebra['candidate']:
            samples.append((zebra['far_edge_y_normalized'],data,image))
        if len(samples)>=5:break
        time.sleep(.1)
    if len(samples)<3:
        raise ValueError('有效条纹画面少于三帧，不能建立停车参考')
    ys=[s[0] for s in samples]
    if max(ys)-min(ys)>.035:
        raise ValueError('参考线波动过大，请保持车与摄像头静止后重采')
    median=statistics.median(ys)
    _,data,image=min(samples,key=lambda s:abs(s[0]-median))
    reference = {'schema_version':2,'measured':True,'camera':args.camera,
                 'trigger_distance_m':args.measured_front_distance_m,
                 'parking_target_distance_m':args.parking_target_distance_m,
                 'parking_zone_m':[0,.3],
                 'far_edge_y_normalized':median,
                 'image_size':[image.shape[1],image.shape[0]],'snapshot_sha256':hashlib.sha256(data).hexdigest(),
                 'created_at':datetime.now().astimezone().isoformat(),
                 'sample_count':len(samples),'sample_y_spread':max(ys)-min(ys),
                 'basis':'operator ruler measurement of neutral-command trigger; parking target is a separate goal; fixed camera pose required',
                 'metric_distance_model':False}
    validate_reference(reference)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x',encoding='utf-8') as f:json.dump(reference,f,ensure_ascii=False,indent=2)
    with args.output.with_suffix('.jpg').open('xb') as f:f.write(data)
    print(json.dumps(reference,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
