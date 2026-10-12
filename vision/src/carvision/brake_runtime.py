"""Camera/replay runner for brake parking. Default is output-free.

Camera images come from the existing preview service. A live actuator may only
consume a loopback service on the Pi; remote-PC latency is not a control clock.
"""
import json
import time
from pathlib import Path
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, HTTPRedirectHandler, build_opener

import cv2
import numpy as np

from .brake_parking import BrakeParkingController, GroundMotionEstimate, load_settings, numeric
from .brake_output import BrakeActuator
from .crosswalk import CrosswalkDetector
from .config import load_config
from .lane import LaneDetector
from .parking_speech import ParkingSpeech


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self,*args,**kwargs):return None


class PreviewReader:
    def __init__(self,base,settings):
        parsed=urlsplit(base)
        if (parsed.scheme!='http' or not parsed.hostname or parsed.username or parsed.password
                or parsed.path not in ('','/') or parsed.query or parsed.fragment):
            raise ValueError('use an explicit HTTP preview origin without credentials/path')
        self.base=base.rstrip('/');self.s=settings
        self.opener=build_opener(ProxyHandler({}),NoRedirect())
        self.last_id=None

    def get(self,path,limit):
        with self.opener.open(self.base+path,timeout=1) as r:data=r.read(limit+1)
        if len(data)>limit:raise ValueError('preview response too large')
        return data

    def check_no_driver(self):
        state=json.loads(self.get('/api/drive/status',262144))
        if state.get('hardware_output') or state.get('worker_started') or state.get('pending_hardware_start'):
            raise RuntimeError('existing manual driver still owns the car; end its control session while keeping camera preview running')

    def next(self):
        end=time.monotonic()+.3;camera=self.s['camera']
        while time.monotonic()<end:
            started=time.monotonic()
            before=json.loads(self.get('/api/status?camera='+camera,262144)).get('raw_preview') or {}
            seq,age=before.get('frame_id'),before.get('host_frame_age_ms')
            if before.get('state')!='ok' or type(seq) is not int or not numeric(age) or not 0<=age<=self.s['max_frame_age_s']*1000:
                raise RuntimeError('no fresh camera frame')
            if seq==self.last_id:
                time.sleep(.005);continue
            jpeg=self.get('/frame.jpg?camera='+camera+'&view=raw',4194304)
            after=json.loads(self.get('/api/status?camera='+camera,262144)).get('raw_preview') or {}
            if after.get('state')!='ok' or after.get('frame_id')!=seq:
                continue  # JPEG/status pair crossed a publication boundary.
            image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
            if image is None:raise ValueError('invalid camera JPEG')
            received=time.monotonic()
            captured=started-age/1000  # conservative estimate including HTTP latency
            if received-captured>self.s['max_frame_age_s']:raise RuntimeError('camera sample expired during transfer')
            self.last_id=seq
            return image,seq,captured
        raise TimeoutError('no new consistent camera frame')


def append(stream,data):
    stream.write(json.dumps(data,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()


def replay(args):
    s=load_settings(args.config);controller=BrakeParkingController(s,args.path_mode)
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    count=0
    with Path(args.input).open(encoding='utf-8-sig') as source,(out/'decisions.jsonl').open('w',encoding='utf-8') as log:
        try:
            for line in source:
                if not line.strip():continue
                row=json.loads(line);intent=controller.step(row['sample'],row['now_s'])
                append(log,{'now_s':row['now_s'],'intent':intent,'hardware_output':False});count+=1
            if count==0:raise ValueError('empty observation file')
        finally:
            append(log,{'event':'input_ended','action':'neutral','esc_us':1500,'hardware_output':False})
    result={'frames':count,'phase':controller.phase,'hardware_output':False,'output':str(out),'tuning_verified':False}
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result


def camera(args):
    s=load_settings(args.config)
    if not 1<=args.seconds<=180:raise ValueError('camera session must be 1..180 seconds')
    if args.run and urlsplit(args.preview).hostname not in ('127.0.0.1','localhost','::1'):
        raise ValueError('live parking must run on the Pi with its loopback camera feed')
    if args.run and args.esc_mode!='F/B':raise ValueError('--run requires explicit --esc-mode F/B')
    source=PreviewReader(args.preview,s)
    if args.run:source.check_no_driver()
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'settings.json').write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf-8')
    controller=BrakeParkingController(s,args.path_mode)
    motion=GroundMotionEstimate();detector=CrosswalkDetector();lane_detector=LaneDetector(load_config(None).lane)
    actuator=speech=None
    started=time.monotonic();frames=0;failure=None
    try:
        actuator=BrakeActuator(s,run=args.run,esc_mode=args.esc_mode,port=args.port)
        speech=ParkingSpeech(Path(__file__).resolve().parents[3],text=s['speech_text']) if args.run else None
        arm_until=time.monotonic()+(s['arming_s'] if args.run else 0)
        with (out/'observations.jsonl').open('w',encoding='utf-8') as observations,(out/'decisions.jsonl').open('w',encoding='utf-8') as log:
            while time.monotonic()-started<args.seconds:
                image,frame_id,captured=source.next()
                zebra,mask=detector.detect(image,captured*1000)
                clean=image.copy()
                for x,y,w,h in zebra['stripes_xywh']:clean[y:y+h,x:x+w]=0
                lane,_=lane_detector.detect(clean)
                sample={'fresh':True,'frame_id':frame_id,'captured_s':captured,'image_size':[image.shape[1],image.shape[0]],
                        'candidate':zebra['candidate'],'far_edge_y_normalized':zebra['far_edge_y_normalized'],
                        'lane_valid':lane.valid,'lane_offset':lane.offset_normalized,
                        'motion':motion.update(image,frame_id,captured)}
                if args.feedback:
                    data=json.loads(Path(args.feedback).read_text(encoding='utf-8'))
                    if (data.get('verified') is not True or not numeric(data.get('speed_mps'))
                            or not numeric(data.get('captured_monotonic_s'))):
                        raise ValueError('explicit speed feedback is unavailable or invalid')
                # Include detection/optical-flow/file IO time in freshness and
                # control-gap checks, not just the HTTP transfer time.
                now=time.monotonic()
                if args.feedback:
                    sample.update(speed_feedback_valid=True,speed_mps=data['speed_mps'],
                                  speed_age_s=now-data['captured_monotonic_s'])
                if now<arm_until:
                    intent=controller.output();intent.update(phase='arming',reason='保持中位初始化',arm_remaining_s=arm_until-now)
                else:intent=controller.step(sample,now)
                append(observations,{'sample':sample,'now_s':now})
                if intent['phase']=='fault':
                    actuator.close();execution={'hardware_output':args.run,'failsafe_close_requested':True}
                else:execution=actuator.send(intent)
                if speech and any(e.get('event')=='speak' for e in intent['events']):speech.start()
                append(log,{'now_s':now,'intent':intent,'execution':execution,'speech':speech.status() if speech else None})
                frames+=1
                if args.show:
                    view=image.copy()
                    if zebra['bbox_xyxy']:
                        x1,y1,x2,y2=zebra['bbox_xyxy'];cv2.rectangle(view,(x1,y1),(x2,y2),(0,255,255),2)
                    for i,text in enumerate([intent['phase'],f"{intent['action']} | {intent['esc_us']} us",'F/B PROGRAM - UNVERIFIED TUNING']):
                        cv2.putText(view,text,(8,25+22*i),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),3)
                        cv2.putText(view,text,(8,25+22*i),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
                    cv2.imshow('Brake parking - actual camera',view)
                    if cv2.waitKey(1)&0xff in (27,ord('q')):break
                if intent['phase'] in ('done','fault'):break
            append(log,{'event':'shutdown_requested','hardware_output':args.run})
    except BaseException as exc:
        failure=f'{type(exc).__name__}: {exc}';raise
    finally:
        if actuator:actuator.close()
        if speech:speech.close()
        if args.show:cv2.destroyAllWindows()
        with (out/'decisions.jsonl').open('a',encoding='utf-8') as log:
            append(log,{'event':'session_ended','error':failure,'actuator_closed':actuator.closed if actuator else True,
                        'physical_stop_verified':False,'hardware_output':args.run})
        result={'frames':frames,'phase':controller.phase,'hardware_output':args.run,'error':failure,
                'physical_stop_verified':False,'parking_zone_verified':False,'tuning_verified':False,'output':str(out)}
        (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result
