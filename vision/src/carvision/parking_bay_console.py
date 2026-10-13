"""Real-camera bay suggestions and an explicitly started parking-entry trial."""
import copy
import json
import secrets
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .parking_bay import load_settings,validate,readiness,detect_parking
from .brake_parking import GroundMotionEstimate,load_settings as load_brake
from .cone_planning import load_settings as load_cones
from .cone_navigation import reuse_legacy_ground
from .parking_entry_run import ParkingEntryRun


class BaySelection:
    def __init__(self,confirm_s=.5):
        self.confirm_s=confirm_s;self.pending=None;self.since=None;self.count=0;self.selected=None;self.last=None
        self.suggestion=None;self.basis=None;self.source=None

    def snapshot(self,reason=None,stale=False):
        side=self.suggestion or self.selected or 'right'
        basis=self.basis or '尚无方向证据，按固定默认值暂选右侧'
        return {'selected_slot':side,'candidate_slot':side,'confirmed':False,'tentative':True,
                'suggestion_basis':basis,'suggestion_source':self.source or 'default_right',
                'stale':stale,'reason':reason or '已给出暂定方向，等待更多证据',
                'hardware_output':False,'autonomous_entry_allowed':False}

    def _suggest(self,observation,clear,blocked):
        if len(clear)==1 and len(blocked)==1 and clear[0]!=blocked[0]:
            return clear[0],'一侧入口挡板、另一侧可见空闲','layout'
        if len(blocked)==1:
            return ('right' if blocked[0]=='left' else 'left'),'一侧入口已见遮挡，暂选另一侧','blocked_side'
        if len(clear)==1:return clear[0],'当前只确认一侧可见空闲','visible_ground'
        hint=observation.get('direction_hint') or {}
        if hint.get('suggested_slot') in ('left','right'):
            return hint['suggested_slot'],hint.get('basis','依据蓝色挡板位置暂选另一侧'),hint.get('source','blue_board_position')
        if (self.suggestion or self.selected) and (self.selected or self.source not in (None,'default_right')):
            return self.suggestion or self.selected,'本帧证据不足，沿用上次方向；不代表当前已空闲','history'
        if len(blocked)==2:return 'right','两侧均有遮挡；仅按默认值暂选右侧，不表示可通行','default_right'
        if len(clear)==2:return 'right','两侧均可见空闲，按固定默认值暂选右侧','default_right'
        return 'right','尚无方向证据，按固定默认值暂选右侧','default_right'

    def update(self,observation,fid,captured,now):
        if not isinstance(captured,(int,float)) or not 0<=now-captured<=.25:
            self.pending=None;self.count=0;self.since=None
            return self.snapshot('图像过期，仅保留建议，不能据此进入车位',stale=True)
        if self.last is not None and (fid<=self.last[0] or captured<=self.last[1]):
            return self.snapshot('等待新帧；不使用重复画面累积确认')
        if self.last is not None and captured-self.last[1]>.25:self.pending=None;self.count=0;self.since=None
        self.last=(fid,captured)
        slots=observation.get('slots',[])
        clear=[s['id'] for s in slots if s['availability']=='clear'];blocked=[s['id'] for s in slots if s['availability']=='blocked']
        self.suggestion,self.basis,self.source=self._suggest(observation,clear,blocked)
        base=self.snapshot()
        if len(slots)!=2 or len(clear)!=1 or len(blocked)!=1 or clear[0]==blocked[0] or set(clear+blocked)!={'left','right'}:
            self.pending=None;self.count=0;self.since=None
            return {**base,'reason':observation.get('reason') or '结构或空闲证据不足，仍给出暂定侧'}
        chosen=clear[0]
        if self.pending!=chosen:self.pending=chosen;self.since=captured;self.count=0
        self.count+=1
        if self.count<3 or captured-self.since<self.confirm_s:
            return {**base,'reason':'先选出一侧，继续积累视觉确认'}
        self.selected=chosen
        partial=observation.get('layout_complete') is False
        reason=('部分结构下的视觉选区已确认；隐藏边界不代表安全入位' if partial else
                '视觉选区已确认；尚未实际入位')
        return {**base,'selected_slot':chosen,'confirmed':True,'tentative':False,'layout_complete':not partial,'reason':reason}


class ParkingBayDrive:
    def __init__(self,drive,states,root,*,clock=time.monotonic,entry_factory=None):
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
        self.entry=ParkingEntryRun(factory=entry_factory,log_root=self.root/'run')
        self.motion=GroundMotionEstimate();self.entry_sample=None

    def status(self):
        with self.lock:
            s=self.drive.status();entry=self.entry.status(s,self.clock());age=None if self.updated is None else max(0,self.clock()-self.updated)
            fresh=age is not None and age<=self.settings['max_frame_age_s']
            selection=(self.recommendation if fresh else self.selector.snapshot('画面过期，仅保留方向建议',stale=True)) if (self.active or self.recommendation) else {}
            s['parking_bay']={'program':'two-bay-parking-v1','active':self.active,'reason':self.settings_error or self.reason,
                'can_observe':not self.settings_error and not self.active and not self.closed and not (self.thread and self.thread.is_alive()),
                'settings_error':self.settings_error,
                'settings':copy.deepcopy(self.settings),'settings_revision':self.revision,
                'observation':self.latest if fresh else None,'selection':selection,
                'frame_age_s':age,'readiness':readiness(self.settings),'motor_output_enabled':entry['hardware_output'],
                'execution_adapter_connected':True,'payment_ready':False,'record_directory':str(self.folder) if self.folder else None}
            s['parking_trial']=entry;s['parking_trial_status_s']=self.clock()
            if entry['active']:
                s['hardware_output']=entry['hardware_output']
                for name in ('crosswalk_trial','traffic_trial','departure_trial'):
                    if name in s:s[name]={**s[name],'can_prepare':False,'can_start':False,'can_request_start':False}
                if 'cone_trial' in s:
                    s['cone_trial']=copy.deepcopy(s['cone_trial'])
                    for name in ('avoidance','short_test'):
                        if name in s['cone_trial']:s['cone_trial'][name].update(can_prepare=False,can_start=False,can_request_start=False)
                if 'parking_payment' in s:s['parking_payment']={**s['parking_payment'],'can_begin':False}
            s['parking_bay_status_s']=self.clock();return s

    def _start_observation(self):
        if self.active:return
        if self.thread and self.thread.is_alive():raise ValueError('停车观察正在结束，请稍后重试')
        self.selector=BaySelection(self.settings['selection_confirm_s']);self.latest={}
        self.recommendation=self.selector.snapshot('等待画面，先给出暂定方向');self.last=None
        self.preview=None;self.updated=None;self.entry_sample=None;self.motion=GroundMotionEstimate()
        self.folder=self.root/'run'/('parking-bay-'+str(time.time_ns()));self.folder.mkdir(parents=True,exist_ok=False)
        self.active=True;self.stop.clear();self.reason='正在读取第二路实际停车区画面'
        self.thread=threading.Thread(target=self._run,name='parking-bay-observation',daemon=True);self.thread.start()

    def _entry_settings(self):
        configs=self.root/'vision/configs';parking=copy.deepcopy(self.settings)
        cone_path=configs/'cone-trial.local.json'
        cones=load_cones(cone_path if cone_path.exists() else configs/'cone-trial.json')
        if not cones['ground'].get('historical_verified') and not cones['ground'].get('verified'):
            reference=json.loads((configs/'cone-legacy-ground-candidate.json').read_text(encoding='utf-8'))
            cones=reuse_legacy_ground(cones,reference)
        geometry=cones['geometry']
        for target,source in [('lane_width_m','lane_width_m'),('vehicle_width_m','vehicle_width_m'),
                              ('front_extent_m','rear_axle_to_front_m'),('rear_extent_m','rear_overhang_m')]:
            if type(geometry.get(source)) in (int,float) and geometry[source]>0:parking[target]=geometry[source]
        return parking,load_brake(configs/'brake-parking.json'),cones

    def request(self,action,payload):
        if not isinstance(payload,dict):raise ValueError('JSON object required')
        with self.lock:
            if self.closed:raise ValueError('停车观察服务已关闭')
            if action=='parking_trial_cancel':
                if self.entry.active:self.entry.cancel('用户手动停止停车入位')
                return self.status()
            if action=='parking_trial_prepare':
                if self.settings_error:raise ValueError(self.settings_error)
                selected=payload.get('selected_slot') or self.selector.snapshot()['selected_slot']
                self.entry.prepare(*self._entry_settings(),self.drive.status(),{**payload,'selected_slot':selected},self.clock())
                try:self._start_observation()
                except Exception:
                    self.entry.cancel('停车观察未能启动',phase='fault');raise
                return self.status()
            if action in ('parking_trial_start','parking_trial_keepalive'):
                self.entry.command(action,payload,self.clock());return self.status()
            if self.entry.active and not action.startswith('parking_bay_'):
                if action=='status':return self.status()
                if action in ('stop','emergency','finish_test','traffic_cancel','crosswalk_cancel','cone_cancel'):
                    self.entry.cancel('人工停止或页面离开，已结束停车入位');return self.status()
                raise ValueError('停车入位占用控制权，停止后再使用遥控或其他任务')
            if not action.startswith('parking_bay_'):
                self.drive.request(action,payload);return self.status()
            if action=='parking_bay_observe':
                if not self.status()['parking_bay']['can_observe']:raise ValueError('停车观察已运行或尚在结束')
                self._start_observation()
            elif action=='parking_bay_stop':
                if self.entry.active:self.entry.cancel('停车识别已停止，入位输出已释放')
                self.active=False;self.stop.set();self.reason='停车观察已停止；未发送车辆动作'
            elif action=='parking_bay_save':
                if self.settings_error:raise ValueError(self.settings_error)
                if self.active:raise ValueError('先停止观察再调整停车识别参数')
                if payload.get('setup_token')!=self.drive.status().get('setup_token') or payload.get('settings_revision')!=self.revision:
                    raise ValueError('页面或参数已变化，请刷新核对')
                candidate=copy.deepcopy(self.settings)
                for key in ('blocked_blue_fraction','clear_ground_fraction','lane_width_m','surface'):
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
                        self.entry.tick(self.entry_sample,self.clock())
                        if self.stop.wait(.04):break
                        continue
                    image=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
                    if image is None:raise ValueError('相机图像无法解码')
                    observation,overlay=detect_parking(image,self.settings);now=self.clock()
                    entry_sample={'fresh':True,'frame_id':fid,'captured_s':t,'camera_id':'secondary',
                                  'image_size':[image.shape[1],image.shape[0]],'parking':observation,
                                  'motion':self.motion.update(image,fid,t)}
                    self.entry_sample=entry_sample
                    recommendation=self.selector.update(observation,fid,t,now)
                    label='SUGGEST '+recommendation['selected_slot'].upper()+(' / VISUAL' if recommendation['confirmed'] else ' / TENTATIVE')
                    cv2.putText(overlay,label,(8,24),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,210,0) if recommendation['confirmed'] else (0,170,255),2)
                    ok,encoded=cv2.imencode('.jpg',overlay)
                    with self.lock:
                        if not self.active:break
                        self.last=(fid,t);self.latest={**observation,'frame_id':fid,'captured_s':t,'camera_id':'secondary'}
                        self.recommendation=recommendation;self.updated=t;self.preview=encoded.tobytes() if ok else None
                        self.reason=(observation.get('reason') or recommendation['reason']) if not observation.get('layout_found') else recommendation['reason']
                        with (self.folder/'observations.jsonl').open('a',encoding='utf-8') as f:
                            handle=getattr(self.entry,'actuator',None)
                            f.write(json.dumps({'observation':self.latest,'selection':recommendation,
                                'hardware_output':bool(handle and getattr(handle,'run',False)),
                                'output_scope':'parking_entry','perception_hardware_output':False},ensure_ascii=False,allow_nan=False)+'\n')
                        if fid%10==0:(self.folder/'latest.jpg').write_bytes(jpg)
                    self.entry.tick(entry_sample,self.clock())
                except (ValueError,KeyError,cv2.error) as exc:
                    self.entry.tick({'fresh':False,'reason':str(exc)},self.clock())
                    with self.lock:
                        self.reason=str(exc);self.latest={};self.recommendation=self.selector.snapshot(str(exc),stale=True);self.preview=None;self.updated=None
                        self.selector.pending=None;self.selector.since=None;self.selector.count=0
                if self.stop.wait(.03 if self.entry.active else .10):break
        except Exception as exc:
            with self.lock:self.reason='停车观察异常：'+str(exc);self.preview=None;self.updated=None
        finally:
            if self.entry.active:self.entry.cancel('停车观察线程结束，输出已释放',phase='fault')
            with self.lock:self.active=False

    def image(self):
        with self.lock:return self.preview if self.updated is not None and self.clock()-self.updated<=.5 else None

    def close(self):
        self.closed=True;self.stop.set()
        if self.entry.active:self.entry.cancel('停车服务关闭，输出已释放')
        if self.thread and self.thread is not threading.current_thread():self.thread.join(timeout=2)
