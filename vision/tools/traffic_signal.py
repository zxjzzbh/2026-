"""Traffic-light perception/evaluation entry; never opens vehicle control."""
import argparse
import json
import sys
import time
from pathlib import Path
from urllib.request import build_opener,ProxyHandler

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'))
import cv2
import numpy as np
from carvision.traffic_signal import TrafficSignalDetector,SignalStability,load_profile,traffic_light_intent
from carvision.traffic_signal_evaluation import evaluate
from carvision.traffic_preview import RemoteFeed,NoRedirect


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    for name in ('evaluate','image','live'):
        s=sub.add_parser(name);s.add_argument('--profile');s.add_argument('--output',type=Path,required=True)
        if name=='evaluate':s.add_argument('--data',type=Path,default=ROOT/'data/raw/traffic_lights')
        if name=='image':s.add_argument('--input',type=Path,required=True)
        if name=='live':
            s.add_argument('--preview',required=True);s.add_argument('--camera',choices=['primary','secondary'],default='primary')
            s.add_argument('--seconds',type=float,default=20)
    args=p.parse_args();profile=load_profile(args.profile)
    if args.command=='evaluate':
        print(json.dumps(evaluate(args.data,args.output,args.profile),ensure_ascii=False,indent=2));return
    args.output.mkdir(parents=True,exist_ok=False)
    detector=TrafficSignalDetector(profile)
    if args.command=='image':
        im=cv2.imdecode(np.frombuffer(args.input.read_bytes(),np.uint8),1);result=detector.detect(im)
        (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(result,ensure_ascii=False,indent=2));return
    if not 1<=args.seconds<=300:raise ValueError('live observation must be 1..300 seconds')
    stability=SignalStability(profile);count=0;errors=0
    with (args.output/'observations.jsonl').open('w',encoding='utf-8') as log:
        class Sink:
            def publish(self,camera,jpeg,source):
                nonlocal count
                now=time.monotonic();im=cv2.imdecode(np.frombuffer(jpeg,np.uint8),1);raw=detector.detect(im)
                count+=1;identity=source.get('status_frame_id_hint')
                stable=stability.update(raw,identity if identity is not None else count,now,time.monotonic())
                record={'camera':camera,'raw':raw,'stable':stable,'source':source,'received_s':now,
                        'intent':traffic_light_intent(stable,camera=camera,profile=profile),'hardware_output':False}
                log.write(json.dumps(record,ensure_ascii=False)+'\n');log.flush()
                return record
        feed=RemoteFeed(args.preview,Sink());opener=build_opener(ProxyHandler({}),NoRedirect());end=time.monotonic()+args.seconds
        while time.monotonic()<end:
            try:feed.fetch_once(args.camera,opener)
            except Exception as exc:
                errors+=1
                stability.reset();log.write(json.dumps({'error':str(exc),'state':'unknown','hardware_output':False})+'\n');log.flush()
            time.sleep(.1)
    summary={'frames':count,'source_errors':errors,'status':'completed' if count else 'no_frames',
             'output':str(args.output),'hardware_output':False}
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))
    if not count:raise SystemExit(1)


if __name__=='__main__':main()
