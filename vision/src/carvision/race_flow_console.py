"""Exclusive full-race console owner; other single-task interfaces stay intact."""
import copy
import json
import secrets
import threading
import time
from pathlib import Path

from .cone_trial_output import ConeTrialActuator
from .parking_speech import ParkingSpeech
from .payment_console import outputs_released
from .race_flow import RaceFlowController, load_bundle, LABELS, parking_blockers
from .race_flow_camera import RaceFlowCamera

STOP_ACTIONS={'stop','emergency','finish_test','race_cancel','race_exit'}


def other_task_active(s):
    return (any((s.get(k) or {}).get('active') for k in ('crosswalk_trial','traffic_trial','departure_trial','parking_trial'))
        or any(((s.get('cone_trial') or {}).get(k) or {}).get('active') for k in ('avoidance','short_test'))
        or (s.get('parking_payment') or {}).get('motion_locked'))


class RaceFlowDrive:
    def __init__(self,drive,states,root,*,live=False,clock=time.monotonic,actuator_factory=None,
                 reader_factory=RaceFlowCamera,speech_factory=None):
        self.drive,self.states,self.root,self.live,self.clock=drive,states,Path(root),live,clock
        self.reader_factory,self.speech_factory=reader_factory,speech_factory or (lambda text:ParkingSpeech(self.root,text=text))
        self.factory=actuator_factory or self._make_actuator
        self.lock=threading.RLock();self.cancel_event=threading.Event();self.thread=None
        self.active=False;self.closed=False;self.controller=None;self.actuator=None;self.reader=None
        self.owner=self.token=None;self.sequence=-1;self.lease_until=0.;self.last_intent=None;self.preview=None;self.updated=None
        self.reason='遥控结束并在切换区停稳后，手动进入自动流程';self.stage='idle'
        self.speech={};self.record=None;self.error=None;self.payment_scanned=False;self.log_key=None
        try:self.bundle=load_bundle(self.root,self.drive)
        except Exception as exc:self.bundle=None;self.error=str(exc)

    def _make_actuator(self,b):
        f=b['flow'];settings={'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':f['forward_us'],
            'creep_us':b['brake']['pwm']['creep_us'],'brake_us':1400},'brake_pulse_s':.12,'brake_release_s':.1,'actuator_lease_s':.2}
        return ConeTrialActuator(settings,left_us=f['left_us'],right_us=f['right_us'],run=self.live,esc_mode='F/B')

    def status(self):
        with self.lock:
            base=self.drive.status();core=self.controller.status(self.clock()) if self.controller else {}
            if self.active:
                for key in ('crosswalk_trial','traffic_trial','departure_trial'):
                    if key in base:base[key]={**base[key],'can_prepare':False,'can_start':False,'can_request_start':False}
                if 'cone_trial' in base:
                    base['cone_trial']=copy.deepcopy(base['cone_trial'])
                    for key in ('avoidance','short_test'):
                        if key in base['cone_trial']:base['cone_trial'][key].update(can_prepare=False,can_start=False,can_request_start=False)
                if 'parking_payment' in base:base['parking_payment']={**base['parking_payment'],'can_begin':False}
                if 'parking_trial' in base:base['parking_trial']={**base['parking_trial'],'can_prepare':False,'can_start':False,'can_request_start':False}
                base['hardware_output']=bool(self.actuator and self.actuator.run)
            report={**core,'stage':self.stage,'stage_label':LABELS.get(self.stage,self.stage),'reason':self.error or self.reason,
                'active':self.active,'live_enabled':self.live,'hardware_output':bool(self.actuator and self.actuator.run),
                'configuration_error':self.error,
                'owner_client':self.owner,'run_token':self.token,'can_enter':self.live and not self.closed and not self.active and not self.error and not other_task_active(base),
                'can_report_paid':self.active and self.stage=='payment','intent':self.last_intent,
                'parking_missing':parking_blockers(self.bundle) if self.bundle else ['configuration_unavailable'],
                'speech':{key:handle.status() for key,handle in self.speech.items()},
                'preview_age_s':None if self.updated is None else self.clock()-self.updated,
                'record_directory':str(self.record) if self.record else None,'field_verified':False}
            return {**base,'race_flow':report,'race_flow_status_s':self.clock()}

    def _owner(self,p,now,lease=True):
        if not self.active or p.get('client')!=self.owner or p.get('run_token')!=self.token:raise ValueError('本页未持有总流程控制权')
        if lease and self.stage not in ('payment','complete') and now>self.lease_until:raise ValueError('总流程页面心跳已超时，重新进入自动模式')
        seq=p.get('race_sequence')
        if type(seq) is not int or seq<=self.sequence:raise ValueError('总流程请求过期或重复')
        self.sequence=seq

    def request(self,action,payload):
        if not isinstance(payload,dict):raise ValueError('JSON object required')
        if action=='status':return self.status()
        if action in STOP_ACTIONS:
            # Switching to WeChat must not reset a running payment deadline.
            if self.active and self.stage in ('payment','complete') and action in ('stop','emergency','finish_test'):
                return self.status()
            self.cancel('已停止全流程，车辆控制已释放')
            if action in ('race_cancel','race_exit'):return self.status()
            self.drive.request(action,payload);return self.status()
        with self.lock:
            if self.closed:raise ValueError('总流程服务已关闭')
            if action=='race_enter':
                if self.active:raise ValueError('本轮自动流程已开始')
                if not self.live:raise ValueError('当前仅加载软件；车端使用 --race-live 才允许显式启动')
                before=self.drive.status()
                if other_task_active(before):raise ValueError('先结束正在运行的单项测试或付款记录')
                if payload.get('setup_token')!=before.get('setup_token'):raise ValueError('页面已更新，请刷新')
                if any(payload.get(k) is not True for k in ('stationary_confirmed','in_switch_zone','fb_mode_confirmed')):
                    raise ValueError('请在切换区停稳，保持 F/B 后进入自动模式')
                client=payload.get('client')
                if not isinstance(client,str) or not 1<=len(client)<=128:raise ValueError('页面身份无效')
                bundle=load_bundle(self.root,self.drive)
                # Release the authenticated manual session before acquiring UART.
                # This is not a fabricated ESC-off declaration for another trial.
                if not outputs_released(before):self.drive.request('finish_test',payload)
                if not outputs_released(self.drive.status()):raise ValueError('原遥控尚未释放控制，请稍后重新进入')
                for stop_action in ('cone_stop','departure_stop','parking_bay_stop'):
                    try:self.drive.request(stop_action,{'client':client})
                    except ValueError:pass
                self.bundle=bundle;self.owner=client;self.token=secrets.token_hex(16);self.sequence=-1
                self.cancel_event=threading.Event();self.lease_until=self.clock()+.5;self.active=True
                self.controller=RaceFlowController(bundle,self.clock());self.stage='arming';self.reason=self.controller.reason
                self.error=None;self.speech={};self.payment_scanned=False;self.last_intent=None
                self.record=self.root/'run'/('race-flow-'+str(time.time_ns()));self.record.mkdir(parents=True)
                (self.record/'settings.json').write_text(json.dumps(bundle,ensure_ascii=False,indent=2),encoding='utf-8')
                self.thread=threading.Thread(target=self._work,name='race-flow',daemon=True);self.thread.start()
                return self.status()
            if action=='race_keepalive':
                self._owner(payload,self.clock());self.lease_until=self.clock()+.5;return self.status()
            if action=='race_report_paid':
                self._owner(payload,self.clock(),lease=False)
                if payload.get('operator_paid') is not True:raise ValueError('仅在队员实际完成支付后记录')
                self.controller.report_paid(self.clock());self.stage=self.controller.stage;self.reason=self.controller.reason
                self._record('operator_reported_paid');return self.status()
            if self.active:
                if self.stage in ('payment','complete') and action in ('payment_scan','payment_scan_stop','payment_select'):
                    self.drive.request(action,payload);return self.status()
                raise ValueError('全流程占用控制权；急停退出后可使用遥控及单项测试')
            if action.startswith('race_'):raise ValueError('未知总流程操作')
        self.drive.request(action,payload);return self.status()

    def _record(self,event):
        if self.record is None:return
        core=self.controller.status(self.clock()) if self.controller else {}
        # No control identities/tokens, QR payloads or payment account data.
        row={'event':event,'monotonic_s':self.clock(),'state':core,'intent':self.last_intent}
        with (self.record/'events.jsonl').open('a',encoding='utf-8') as out:out.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        (self.record/'summary.json').write_text(json.dumps(row,ensure_ascii=False,indent=2),encoding='utf-8')

    def _work(self):
        handle=None
        try:
            reader=self.reader_factory(self.states,self.bundle,self.root,self.clock);self.reader=reader
            handle=self.factory(self.bundle)
            with self.lock:
                if self.cancel_event.is_set():return
                self.actuator=handle
                # The 3-second neutral period starts after the actuator reports
                # neutral, not before potentially slow UART initialization.
                self.controller=RaceFlowController(self.bundle,self.clock())
                self.stage=self.controller.stage;self.reason=self.controller.reason
            self._record('manual_to_auto_handoff')
            while not self.cancel_event.is_set():
                now=self.clock()
                with self.lock:
                    if self.stage not in ('payment','complete') and now>self.lease_until:raise ValueError('控制页心跳中断，已结束全流程')
                    stage=self.stage
                if stage in ('payment','complete'):
                    if handle is not None:
                        handle.close();handle=None
                        with self.lock:self.actuator=None
                    with self.lock:
                        if self.cancel_event.is_set():break
                        if not self.payment_scanned and stage=='payment':
                            self.drive.request('payment_scan',{'client':self.owner});self.payment_scanned=True
                        self.controller.step(None,self.clock());self.stage=self.controller.stage;self.reason=self.controller.reason
                    self.cancel_event.wait(.1);continue
                sample=reader.read(stage,now)
                if sample is None:
                    # No old image may keep a moving actuator lease alive.
                    if self.updated is not None and self.clock()-self.updated>.25:raise ValueError('相机停止更新，已结束全流程')
                    self.cancel_event.wait(.005);continue
                now=self.clock()
                with self.lock:
                    if self.cancel_event.is_set():break
                    audio={key:obj.status()['state'] for key,obj in self.speech.items()}
                    intent=self.controller.step(sample,now,audio);self.stage=self.controller.stage;self.reason=self.controller.reason
                    self.preview=reader.preview;self.updated=sample['captured_s']
                    if intent is None:continue
                    self.last_intent=intent
                    if self.stage=='fault':raise ValueError(self.reason)
                    # Linearize cancel against every hardware send.
                    handle.send(intent)
                    for event in intent.get('events',[]):
                        if event['event']=='speak' and event['id'] not in self.speech:
                            speech=self.speech_factory(event['text']);self.speech[event['id']]=speech;speech.start()
                    key=(self.stage,intent['action'],intent['reason'])
                    if key!=self.log_key:self._record('decision');self.log_key=key
                self.cancel_event.wait(.005)
        except Exception as exc:
            with self.lock:
                self.stage='fault';self.reason=str(exc)
                if self.controller:self.controller.fail(str(exc))
                self.cancel_event.set()
        finally:
            if handle is not None:handle.close()
            with self.lock:
                self.actuator=None
                if self.stage not in ('payment','complete'):self.active=False
                self._record('worker_stopped')
            for speech in self.speech.values():speech.close()

    def cancel(self,reason):
        with self.lock:
            self.cancel_event.set();self.reason=reason
            if self.active:
                self.stage='cancelled'
                if self.controller:self.controller.stage='cancelled';self.controller.reason=reason
            handle=self.actuator;self.actuator=None
        if handle:handle.close()
        if self.thread and self.thread is not threading.current_thread():self.thread.join(timeout=4)
        if self.payment_scanned:
            try:self.drive.request('payment_scan_stop',{'client':self.owner})
            except (ValueError,RuntimeError):pass
            self.payment_scanned=False
        with self.lock:self.active=False;self._record('cancelled')

    def close(self):
        self.closed=True;self.cancel('服务结束，已停止全流程')
