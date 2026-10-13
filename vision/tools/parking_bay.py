"""Offline real-image inspection / measured observation replay; no motor I/O."""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'vision/src'))
from carvision.parking_bay import load_settings,detect_parking
from carvision.parking_mission import ParkingMission


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'vision/configs/parking-bay.json')
    sub=p.add_subparsers(dest='mode',required=True)
    for name in ('image','decide'):
        q=sub.add_parser(name);q.add_argument('--input',type=Path,required=True);q.add_argument('--output',type=Path,required=True)
    args=p.parse_args();s=load_settings(args.config)
    if args.output.exists():raise ValueError('输出已存在，请换一个新路径，以免覆盖采集结果')
    if args.mode=='image':
        image=cv2.imdecode(np.frombuffer(args.input.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        if image is None:raise ValueError('无法读取图像')
        result,overlay=detect_parking(image,s);ok,encoded=cv2.imencode('.jpg',overlay)
        if not ok:raise ValueError('标注图编码失败')
        args.output.mkdir(parents=True);(args.output/'recognition.jpg').write_bytes(encoded.tobytes())
        (args.output/'recognition.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    else:
        mission=ParkingMission(s);results=[]
        for line in args.input.read_text(encoding='utf-8').splitlines():
            if line.strip():
                packet=json.loads(line);results.append(mission.step(packet,packet['now_s']))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text('\n'.join(json.dumps(r,ensure_ascii=False,allow_nan=False) for r in results)+'\n',encoding='utf-8')
    print(json.dumps({'output':str(args.output),'hardware_output':False},ensure_ascii=True))


if __name__=='__main__':main()
