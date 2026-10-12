"""Independent real-image cone observation and non-actuating planner replay.

live uses GET requests to the existing car camera service. It never loads a
vehicle actuator, replaces the console, restarts the Pi or creates fake frames.
"""
import argparse
import hashlib
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'))

import cv2
import numpy as np

from carvision.cone_planning import load_settings,readiness,ConeMission
from carvision.cone_vision import BlueConeDetector,ImageConeTracker
from carvision.cone_road import ConeRoadContext
from carvision.traffic_preview import RemoteFeed


def encode(path,image):
    ok,data=cv2.imencode(path.suffix,image)
    if not ok:raise ValueError('image encoding failed')
    path.write_bytes(data.tobytes())


def annotate(image,tracks,camera):
    output=image.copy()
    for row in tracks:
        x1,y1,x2,y2=row['bbox_xyxy']
        color=(0,165,255) if row['kind']=='unresolved_blue' else (255,170,50)
        if not row.get('visible',True):color=(150,150,150)
        cv2.rectangle(output,(x1,y1),(x2,y2),color,2)
        label=row.get('id',row['kind'])+(' OK' if row.get('confirmed') else '')
        cv2.putText(output,label,(x1,max(15,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
    cv2.putText(output,camera.upper()+' / OBSERVE ONLY',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
    return output


class LiveSink:
    """PC observations are received frames, never calibrated control feedback."""
    def __init__(self,settings,log):
        self.lock=threading.RLock();self.settings=settings;self.log=log
        self.detectors={c:BlueConeDetector(settings) for c in ('primary','secondary')}
        self.trackers={c:ImageConeTracker(c,settings) for c in self.detectors}
        self.roads={c:ConeRoadContext(settings) for c in self.detectors}
        self.last_hash={};self.ids={};self.views={};self.records={};self.errors={};self.count=0;self.closed=False

    def publish(self,camera,jpeg,source):
        image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),1)
        if image is None:raise ValueError('invalid JPEG')
        if [image.shape[1],image.shape[0]]!=self.settings['image_size']:raise ValueError('expected existing 480x360 preview')
        digest=hashlib.sha256(jpeg).hexdigest();now=time.monotonic()
        with self.lock:
            if self.closed:return None
            if self.last_hash.get(camera)==digest:return None
            self.last_hash[camera]=digest;self.ids[camera]=self.ids.get(camera,0)+1
            rows,mask=self.detectors[camera].detect(image)
            context=self.roads[camera].inspect(image,mask)
            self.roads[camera].enrich(rows,image,context)
            tracks=self.trackers[camera].update(rows,self.ids[camera],now)
            annotated=annotate(image,tracks,camera)
            for points in (context['left_inner'],context['right_inner']):
                if points:cv2.polylines(annotated,[np.asarray(points,np.int32)],False,(0,230,100) if context['valid'] else (0,180,255),2)
            record={'camera':camera,'received_s':now,'source':source,'tracks':tracks,
                'lane':{'valid':context['valid'],'reason':context['reason']},'road_context':context,'hardware_output':False,
                'usable_for_vehicle_control':False,'physical_cones_passed':None}
            self.count+=1;self.records[camera]=record;self.views[camera]=(image,annotated);self.errors.pop(camera,None)
            self.log.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n');self.log.flush()
            return record

    def error(self,camera,message):
        with self.lock:
            if self.closed:return
            if self.errors.get(camera)!=message:
                print(camera+': '+message,flush=True)
                self.log.write(json.dumps({'camera':camera,'error':message,'hardware_output':False},ensure_ascii=False)+'\n');self.log.flush()
            self.errors[camera]=message

    def panel(self):
        with self.lock:
            now=time.monotonic();panes=[];fresh=False
            for camera in ('primary','secondary'):
                record=self.records.get(camera)
                available=record and camera not in self.errors and 0<=now-record['received_s']<=2
                if available:
                    panes.append(self.views[camera][1].copy());fresh=True
                else:
                    pane=np.zeros((360,480,3),np.uint8)
                    cv2.putText(pane,camera.upper()+' NO LIVE FRAME',(15,50),cv2.FONT_HERSHEY_SIMPLEX,.55,(170,170,170),1)
                    panes.append(pane)
            return np.concatenate(panes,axis=1) if fresh else None

    def save(self,output):
        with self.lock:
            prefix=datetime.now().strftime('%Y%m%d-%H%M%S-%f');saved=0
            for camera,record in self.records.items():
                if camera in self.errors or time.monotonic()-record['received_s']>2:continue
                raw,display=self.views[camera];base=output/(prefix+'-'+camera)
                encode(base.with_suffix('.jpg'),raw);encode(base.with_name(base.name+'-annotated').with_suffix('.jpg'),display)
                base.with_suffix('.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8');saved+=1
            print(f'Saved {saved} real camera view(s) to {output}',flush=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'vision/configs/cone-trial.json')
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('check',help='report measured-geometry readiness')
    for command in ('image','live','replay'):
        p=sub.add_parser(command);p.add_argument('--output',type=Path)
        if command in ('image','replay'):p.add_argument('--input',type=Path,required=True)
        if command=='live':
            p.add_argument('--preview',default='http://127.0.0.1:8082')
            p.add_argument('--seconds',type=float,default=600)
            p.add_argument('--headless',action='store_true',help='write observations without an OpenCV window')
    args=parser.parse_args(argv)
    config=args.config
    local=config.with_name(config.stem+'.local.json')
    settings=load_settings(local if local.exists() else config)
    if args.command=='check':
        print(json.dumps(readiness(settings),ensure_ascii=False,indent=2));return 0
    if args.command=='live' and not 1<=args.seconds<=3600:parser.error('--seconds must be 1..3600')
    output=args.output or ROOT/'run'/('cone-'+args.command+'-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    output.mkdir(parents=True,exist_ok=False)
    if args.command=='image':
        image=cv2.imdecode(np.frombuffer(args.input.read_bytes(),np.uint8),1)
        if image is None:raise ValueError('cannot decode input image')
        rows,_=BlueConeDetector(settings).detect(image)
        encode(output/'annotated.jpg',annotate(image,rows,'image'))
        (output/'result.json').write_text(json.dumps({'detections':rows,'hardware_output':False},ensure_ascii=False,indent=2),encoding='utf-8')
        print(output);return 0
    if args.command=='replay':
        mission=ConeMission(settings);count=0
        with args.input.open(encoding='utf-8-sig') as source,(output/'decisions.jsonl').open('w',encoding='utf-8') as log:
            for line in source:
                if not line.strip():continue
                row=json.loads(line);result=mission.step(row['sample'],row['now_s'])
                log.write(json.dumps(result,ensure_ascii=False,allow_nan=False)+'\n');count+=1
        if not count:raise ValueError('empty observation file')
        print(output);return 0
    print('Reading REAL car cameras; no vehicle control. Left=primary, right=H65.',flush=True)
    print('Q/Esc: exit; S: save current actual images. Output: '+str(output),flush=True)
    window='Blue cones - real cameras - observation only';opened=False
    with (output/'observations.jsonl').open('w',encoding='utf-8') as log:
        sink=LiveSink(settings,log);feed=RemoteFeed(args.preview,sink);feed.start();end=time.monotonic()+args.seconds
        try:
            while time.monotonic()<end:
                if not args.headless:
                    panel=sink.panel()
                    if panel is not None:
                        cv2.imshow(window,panel);opened=True
                    elif opened:
                        cv2.destroyWindow(window);opened=False
                    if opened:
                        key=cv2.waitKey(1)&255
                        if key in (27,ord('q')) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:break
                        if key==ord('s'):sink.save(output)
                time.sleep(.05)
        except KeyboardInterrupt:pass
        finally:
            with sink.lock:sink.closed=True
            feed.close()
            if opened:cv2.destroyAllWindows()
    summary={'received_frames':sink.count,'hardware_output':False,'output':str(output),'errors':sink.errors}
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False));return 0 if sink.count else 1


if __name__=='__main__':raise SystemExit(main())
