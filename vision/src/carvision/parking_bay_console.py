"""Real-camera parking observer and stable bay recommendation. No motor I/O."""
import copy
import json
import secrets
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .parking_bay import load_settings,validate,readiness,detect_parking


class BaySelection:
    def __init__(self,confirm_s=.5):
        self.confirm_s=confirm_s;self.pending=None;self.since=None;self.count=0;self.selected=None;self.last=None

    def update(self,observation,fid,captured,now):
        base={'selected_slot':self.selected,'confirmed':False,'hardware_output':False}
        if not isinstance(captured,(int,float)) or not 0<=now-captured<=.25:
            self.pending=None;self.count=0;self.since=None
            return {**base,'reason':'图像超时，不能选择或进入车位'}
        if self.last is not None and (fid<=self.last[0] or captured<=self.last[1]):
            return {**base,'reason':'等待新的相机帧'}
        if self.last is not None and captured-self.last[1]>.25:self.pending=None;self.count=0;self.since=None
        self.last=(fid,captured)
        slots=observation.get('slots',[]);clear=[s['id'] for s in slots if s['availability']=='clear'];blocked=[s['id'] for s in slots if s['availability']=='blocked']
        if len(slots)!=2 or len(clear)!=1 or len(blocked)!=1:
            self.pending=None;self.count=0;self.since=None
            return {**base,'reason':'需同时看到两个半区，并确认一侧入口遮挡、另一侧可见空闲'}
        chosen=clear[0]
        if self.selected and chosen!=self.selected:return {**base,'reason':'已选半区状态改变，请停止并重新观察，不能跨区换位'}
        if self.pending!=chosen:self.pending=chosen;self.since=captured;self.count=0
        self.count+=1
        if self.count<3 or captured-self.since<self.confirm_s:return {**base,'candidate_slot':chosen,'reason':'正在连续确认空闲半区'}
        self.selected=chosen
        return {**base,'selected_slot':chosen,'confirmed':True,'reason':'候选选区已确认；实际入位仍须米制标定、路径和位姿反馈'}


class ParkingBayDrive:
    def __init__(self,drive,states,root,*,clock=time.monotonic):
        self.drive,self.states,self.root,self.clock=drive,states,Path(root),clock
        self.path=self.root/'vision/configs/parking-bay.local.json'
        self.settings_error=None
        self.settings=load_settings(self.root/'vision/configs/parking-bay.json')
        if self.path.exists():
            try:self.settings=load_settings(self.path)
            except (OSError,ValueError,TypeError,KeyError):self.settings_error='停车本地配置无效；请修复 parking-bay.local.json 后重启，原文件未覆盖'
        self.lock=threading.RLock();self.stop=threading.Event();self.thread=None;self.active=self.closed=False
        self.revision=secrets.token_hex(8);self.selector=BaySelection(self.settings['selection_confirm_s'])
        self.latest={};self.recommendation={};self.preview=None;self.updated=None;self.folder=None;self.last=None
        self.reason='停车模块待观察；当前只显示实际识别与选区，不启动车辆'

    def status(self):
        with self.lock:
            s=self.drive.status();age=None if self.updated is None else max(0,self.clock()-self.updated)
            fresh=age is not None and age<=self.settings['max_frame_age_s']
            s['parking_bay']={'program':'two-bay-parking-v1','active':self.active,'reason':self.settings_error or self.reason,
                'can_observe':not self.settings_error and not self.active and not self.closed and not (self.thread and self.thread.is_alive()),
                'settings_error':self.settings_error,
                'settings':copy.deepcopy(self.settings),'settings_revision':self.revision,
                'observation':self.latest if fresh else None,'selection':self.recommendation if fresh else {},
                'frame_age_s':age,'readiness':readiness(self.settings),'motor_output_enabled':False,
                'execution_adapter_connected':False,'payment_ready':False,'record_directory':str(self.folder) if self.folder else None}
            s['parking_bay_status_s']=self.clock();return s

    def request(self,action,payload):
        with self.lock:
            if not action.startswith('parking_bay_'):
                self.drive.request(action,payload);return self.status()
            if self.closed:raise ValueError('停车观察服务已关闭')
            if action=='parking_bay_observe':
                if not self.status()['parking_bay']['can_observe']:raise ValueError('停车观察已运行或尚在结束')
                self.selector=BaySelection(self.settings['selection_confirm_s']);self.latest={};self.recommendation={};self.last=None
                self.preview=None;self.updated=None;self.folder=self.root/'run'/('parking-bay-'+str(time.time_ns()));self.folder.mkdir(parents=True,exist_ok=False)
                self.active=True;self.stop.clear();self.reason='正在读取第二路实际停车区画面'
                self.thread=threading.Thread(target=self._run,name='parking-bay-observation',daemon=True);self.thread.start()
            elif action=='parking_bay_stop':
                self.active=False;self.stop.set();self.reason='停车观察已停止；未发送车辆动作'
            elif action=='parking_bay_save':
                if self.settings_error:raise ValueError(self.settings_error)
                if self.active:raise ValueError('先停止观察再调整停车识别参数')
                if payload.get('setup_token')!=self.drive.status().get('setup_token') or payload.get('settings_revision')!=self.revision:
                    raise ValueError('页面或参数已变化，请刷新核对')
                candidate=copy.deepcopy(self.settings)
                for key in ('blocked_blue_fraction','clear_ground_fraction','lane_width_m'):
                    if key in payload:candidate[key]=payload[key]
                candidate=validate(candidate);self.path.parent.mkdir(parents=True,exist_ok=True)
                tmp=self.path.with_suffix('.json.tmp');tmp.write_text(json.dumps(candidate,ensure_ascii=False,indent=2),encoding='utf-8');tmp.replace(self.path)
                self.settings=candidate;self.revision=secrets.token_hex(8)
            else:raise ValueError('未知停车观察动作；本接口不提供电机输出')
            return self.status()

    def _run(self):
        try:
            while not self.stop.is_set():
                try:
                    state=self.states['secondary']
                    with state.condition:jpg,fid,ms=state.raw_jpeg,state.raw_frame_id,state.raw_received_ms;failed=state.state in ('error','stopped')
                    t=ms/1000 if isinstance(ms,(int,float)) else -1
                    if failed or not jpg or type(fid) is not int or not 0<=self.clock()-t<=self.settings['max_frame_age_s']:raise ValueError('第二路没有新鲜画面')
                    if self.last==(fid,t):
                        if self.stop.wait(.04):break
                        continue
                    image=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
                    if image is None:raise ValueError('相机图像无法解码')
                    observation,overlay=detect_parking(image,self.settings);now=self.clock()
                    recommendation=self.selector.update(observation,fid,t,now)
                    ok,encoded=cv2.imencode('.jpg',overlay)
                    with self.lock:
                        if not self.active:break
                        self.last=(fid,t);self.latest={**observation,'frame_id':fid,'captured_s':t,'camera_id':'secondary'}
                        self.recommendation=recommendation;self.updated=t;self.preview=encoded.tobytes() if ok else None
                        self.reason=recommendation['reason']
                        with (self.folder/'observations.jsonl').open('a',encoding='utf-8') as f:
                            f.write(json.dumps({'observation':self.latest,'selection':recommendation,'hardware_output':False},ensure_ascii=False,allow_nan=False)+'\n')
                        if fid%10==0:(self.folder/'latest.jpg').write_bytes(jpg)
                except (ValueError,KeyError,cv2.error) as exc:
                    with self.lock:
                        self.reason=str(exc);self.latest={};self.recommendation={};self.preview=None;self.updated=None
                        self.selector.pending=None;self.selector.since=None;self.selector.count=0
                if self.stop.wait(.18):break
        except Exception as exc:
            with self.lock:self.reason='停车观察异常：'+str(exc);self.preview=None;self.updated=None
        finally:
            with self.lock:self.active=False

    def image(self):
        with self.lock:return self.preview if self.updated is not None and self.clock()-self.updated<=.5 else None

    def close(self):
        self.closed=True;self.stop.set()
        if self.thread and self.thread is not threading.current_thread():self.thread.join(timeout=2)
