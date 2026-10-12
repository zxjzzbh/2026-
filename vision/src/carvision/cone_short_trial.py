"""Page-owned fixed steering/brake measurements, independent of cone geometry."""
import json
import secrets
import threading
import time
from pathlib import Path

from .cone_execution import base_idle
from .cone_short_output import ShortActuator,validate_parameters


class ConeShortRun:
    def __init__(self,log_root,*,factory=None,clock=time.monotonic,on_event=None):
        self.factory=factory or (lambda:ShortActuator(run=True,esc_mode='F/B'))
        self.clock=clock;self.on_event=on_event;self.root=Path(log_root);self.lock=threading.RLock()
        self.active=self.closing=self.opening=False;self.phase='idle';self.reason='尚未准备固定档短测'
        self.owner=self.token=None;self.sequence=-1;self.lease_until=self.prepared_s=0.
        self.actuator=self.folder=self.log=None;self.latest={};self.round=0;self.last_record=None

    def status(self,base,other_active=False):
        with self.lock:
            return {'program':'cone-fixed-short-v1','active':self.active,'phase':self.phase,'reason':self.reason,
                'owner_client':self.owner,'run_token':self.token,'round':self.round,'output':dict(self.latest),
                'can_prepare':not self.active and not other_active and base_idle(base),
                'can_start':self.active and not self.closing and self.phase in ('ready','complete'),
                'hardware_output':bool(self.actuator and getattr(self.actuator,'run',False)),
                'needs_geometry':False,'physical_stop_verified':False,
                'forward_limits_us':[1501,1625],'duration_limits_s':[.2,2.],
                'log_directory':str(self.folder) if self.folder else None}

    def prepare(self,base,payload,*,other_active=False):
        with self.lock:
            if self.active or other_active or not base_idle(base):raise ValueError('先结束遥控、红绿灯或避障控制，再准备短测')
            if payload.get('setup_token')!=base.get('setup_token'):raise ValueError('页面已过期，请刷新')
            if payload.get('esc_off') is not True or payload.get('fb_mode_confirmed') is not True:raise ValueError('准备时电调关闭、保持 F/B')
            client=payload.get('client')
            if not isinstance(client,str) or not 1<=len(client)<=128:raise ValueError('页面身份无效')
            self.folder=self.root/('cone-short-'+str(time.time_ns()));self.folder.mkdir(parents=True,exist_ok=False)
            self.log=(self.folder/'events.jsonl').open('w',encoding='utf-8')
            self.owner,self.token=client,secrets.token_hex(16);self.sequence=-1;self.round=0
            self.active=True;self.closing=False;self.phase='preparing';self.reason='正在建立短测中位输出'
            self.lease_until=self.clock()+.5;self.prepared_s=self.clock();self.latest={};self.last_record=None
            self._record({'event':'prepare','now_s':self.clock(),'geometry_not_required':True})

    def command(self,action,payload):
        with self.lock:
            now=self.clock()
            if not self.active or self.closing or payload.get('client')!=self.owner or payload.get('run_token')!=self.token:
                raise ValueError('本页未持有短测控制权')
            if now>self.lease_until:raise ValueError('短测页面心跳已过期，请停止后重新准备')
            seq=payload.get('trial_sequence')
            if type(seq) is not int or seq<=self.sequence:raise ValueError('old or invalid short-test sequence')
            if action=='cone_short_start':
                if (self.phase not in ('ready','complete') or not self.actuator or type(payload.get('expected_round')) is not int
                        or payload['expected_round']!=self.round):raise ValueError('短测正在进行或轮次已变化，不会重复启动')
                if any(payload.get(k) is not True for k in ('fb_mode_confirmed','esc_on_static','field_ready')):
                    raise ValueError('开始前需 F/B、电调打开且车轮静止、空旷且能随时断电')
                mode,pulse,duration=payload.get('mode'),payload.get('forward_us'),payload.get('duration_s')
                validate_parameters(mode,pulse,duration)
                self.actuator.send('start',mode=mode,forward_us=pulse,duration_s=duration,expected_round=self.round)
                self.round+=1;self.phase='arming';self.reason='3 秒中位初始化，随后执行固定档短测'
                self._record({'event':'start_requested','now_s':now,'round':self.round,'mode':mode,'forward_us':pulse,'duration_s':duration})
            elif action!='cone_short_keepalive':raise ValueError('未知短测动作')
            self.sequence=seq;self.lease_until=now+.5

    def tick(self,frame_fresh):
        with self.lock:
            if not self.active or self.closing or self.opening:return
            token=self.token
            if self.clock()>self.lease_until:failure='页面心跳中断，短测停止'
            elif not frame_fresh:failure='两路均无新鲜画面，短测停止'
            else:failure=None
            if self.actuator is None and not failure:self.opening=True
            open_needed=self.opening
        if failure:self.cancel(failure,phase='fault');return
        if open_needed:
            handle=None
            try:handle=self.factory()
            except Exception as exc:
                with self.lock:
                    self.opening=False
                    if self.closing:self._finish(self.reason,'cancelled');return
                self.cancel(str(exc),phase='fault');return
            with self.lock:
                self.opening=False
                cancelled=self.closing or not self.active or token!=self.token or self.clock()>self.lease_until
                if not cancelled:
                    self.actuator=handle;self.phase='ready';self.reason='准备完成；打开电调并确认静止后选择短测'
            if cancelled:
                handle.close()
                with self.lock:
                    if token==self.token:self._finish(self.reason if self.closing else '准备期间已失联或取消','cancelled')
                return
        try:
            with self.lock:
                if not self.active or self.closing or token!=self.token:return
                if self.clock()>self.lease_until:raise ValueError('页面心跳中断，短测停止')
                if self.clock()-self.prepared_s>=180:raise ValueError('短测会话达到 3 分钟，请重新准备')
                self.actuator.send('keepalive')
                status=self.actuator.poll()
                # Delayed READY from before START may not re-enable buttons.
                if status.get('round',0)<self.round:return
                self.latest=status;self.phase=status['phase'];self.reason=status.get('reason',self.phase)
                key=(status.get('round'),status['phase'],status.get('brake_count'))
                if key!=self.last_record:
                    self._record({'event':'worker_phase','received_s':self.clock(),'output':status});self.last_record=key
                    if self.on_event and status['phase'] in ('driving','braking','complete'):
                        self.on_event(self.folder,status)
        except Exception as exc:self.cancel(str(exc),phase='fault')

    def _record(self,data):
        if self.log and not self.log.closed:
            self.log.write(json.dumps(data,ensure_ascii=False,allow_nan=False)+'\n');self.log.flush()

    def _finish(self,reason,phase):
        self.actuator=None;self.active=False;self.closing=False;self.phase=phase;self.reason=reason
        self._record({'event':'closed','now_s':self.clock(),'phase':phase,'reason':reason,'physical_stop_verified':False})
        if self.log and not self.log.closed:self.log.close()

    def cancel(self,reason,phase='cancelled'):
        with self.lock:
            if not self.active or self.closing:return
            self.closing=True;self.phase='closing';self.reason=reason;handle=self.actuator
            if self.opening:return  # startup owns its late handle and will close it
        try:
            if handle:handle.close()
        finally:
            with self.lock:self._finish(reason,phase)
