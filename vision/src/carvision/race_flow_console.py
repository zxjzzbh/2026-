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
from .parking_control import calibration_tables, parking_readiness
from .brake_parking import crosswalk_trigger_reference
from .handoff import StillnessWindow

STOP_ACTIONS={'stop','emergency','finish_test','race_cancel','race_exit'}


def other_task_active(s):
    return (any((s.get(k) or {}).get('active') for k in ('crosswalk_trial','traffic_trial','departure_trial','parking_trial'))
        or any(((s.get('cone_trial') or {}).get(k) or {}).get('active') for k in ('avoidance','short_test'))
        or (s.get('parking_payment') or {}).get('motion_locked'))


def start_readiness(bundle, parking):
    """Entry configuration and later metric parking are separate requirements.

    Existing stage controllers still enforce live image/motion checks. Missing
    parking calibration neither qualifies that stage nor blocks earlier tasks.
    """
    if bundle is None:
        return {'ready':False, 'missing':list(parking['missing']),
                'scope':'unavailable', 'parking_ready':False}
    ready = parking['ready']
    return {'ready':True, 'missing':[], 'parking_ready':ready,
            'scope':'full_race' if ready else 'through_cones'}


class RaceFlowDrive:
    def __init__(self,drive,states,root,*,live=False,clock=time.monotonic,actuator_factory=None,
                 reader_factory=RaceFlowCamera,speech_factory=None,require_ready=True):
        self.drive,self.states,self.root,self.live,self.clock=drive,states,Path(root),live,clock
        self.reader_factory,self.speech_factory=reader_factory,speech_factory or (lambda text:ParkingSpeech(self.root,text=text))
        self.factory=actuator_factory or self._make_actuator
        self.require_ready=require_ready
        self.lock=threading.RLock();self.cancel_event=threading.Event();self.thread=None
        self.active=False;self.closed=False;self.controller=None;self.actuator=None;self.reader=None
        self.owner=self.token=None;self.sequence=-1;self.lease_until=0.;self.last_intent=None;self.preview=None;self.updated=None
        self.reason='遥控结束并在切换区停稳后，手动进入自动流程';self.stage='idle'
        self.speech={};self.record=None;self.error=None;self.payment_scanned=False;self.log_key=None
        self.handoff={};self.cleanup_error=None;self.record_error=None
        try:self.bundle=load_bundle(self.root,self.drive)
        except Exception as exc:self.bundle=None;self.error=str(exc)

    def _make_actuator(self,b):
        f=b['flow'];settings={'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':f['forward_us'],
            'creep_us':b['brake']['pwm']['creep_us'],'brake_us':1400},
            'brake_pulse_s':max(b['brake']['brake_pulse_s'],b['traffic']['brake_pulse_s']),
             'brake_release_s':.1,'actuator_lease_s':.2,
             'traffic_forward_us':b['traffic']['pwm']['search_us']}
        settings['cone_forward_us']=sorted({min(f['forward_us'],1570),min(f['forward_us'],1580)})
        if not parking_blockers(b):
            settings['parking_forward_us']=calibration_tables(b['parking'])[0][1]
        if callable(getattr(getattr(self,'drive',None),'borrow_actuator',None)):
            settings.update(left_us=f['left_us'],right_us=f['right_us'])
            return self.drive.borrow_actuator(settings,self.owner)
        return ConeTrialActuator(settings,left_us=f['left_us'],right_us=f['right_us'],run=self.live,esc_mode='F/B')

    def readiness(self):
        if self.bundle is None:
            return {'ready':False,'missing':[self.error or 'configuration_unavailable'],'groups':{},'items':[]}
        return parking_readiness(self.bundle)

    def _thread_alive(self):
        return bool(self.thread and getattr(self.thread,'is_alive',lambda:False)())

    def _current(self,cancel,token):
        return self.cancel_event is cancel and self.token==token

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
            current_intent=copy.deepcopy(self.last_intent)
            stopped_intent=None
            if not self.active and self.stage in ('fault','cancelled'):
                stopped_intent=current_intent
                current_intent={**(current_intent or {}),'action':'neutral','esc_us':1500,
                                'steering_us':1610,'reason':self.error or self.reason}
            readiness=self.readiness()
            entry=start_readiness(self.bundle, readiness)
            report={**core,'stage':self.stage,'stage_label':LABELS.get(self.stage,self.stage),'reason':self.error or self.reason,
                'active':self.active,'live_enabled':self.live,'hardware_output':None if self.cleanup_error else bool(self.actuator and self.actuator.run),
                'configuration_error':self.error,
                'crosswalk_calibration':crosswalk_trigger_reference(self.bundle['brake']) if self.bundle else None,
                'owner_client':self.owner,'run_token':self.token,'can_enter':self.live and not self.closed and not self.active and not self._thread_alive() and not self.error and not self.cleanup_error and not other_task_active(base) and (entry['ready'] or not self.require_ready),
                'readiness':readiness,'start_readiness':entry,
                'handoff':dict(self.handoff),'cleanup_error':self.cleanup_error,'record_error':self.record_error,
                'can_report_paid':self.active and self.stage=='payment','intent':current_intent,'last_intent_before_stop':stopped_intent,
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
            if self.active and action!='emergency' and payload.get('client')!=self.owner:
                return self.status()
            # Switching to WeChat must not reset a running payment deadline.
            if self.active and self.stage in ('payment','complete') and action in ('stop','emergency','finish_test'):
                if action=='emergency':
                    handle=self.actuator
                    if handle:self._close_handle(handle)
                    self.drive.request('emergency',payload)
                return self.status()
            self.cancel('已停止全流程，车辆控制已释放')
            if action in ('race_cancel','race_exit'):return self.status()
            self.drive.request(action,payload);return self.status()
        with self.lock:
            if self.closed:raise ValueError('总流程服务已关闭')
            if action=='parking_trial_keepalive' and payload.get('compact_ack') is True:
                if self.active:raise ValueError('全流程占用控制权')
                return self.drive.request(action,payload)
            if action=='race_enter':
                if self.active or self._thread_alive():raise ValueError('上一轮自动流程仍在运行或清理')
                if self.cleanup_error:raise ValueError('执行器停止尚未确认，请重新准备：'+self.cleanup_error)
                if not self.live:raise ValueError('当前仅加载软件；车端使用 --race-live 才允许显式启动')
                before=self.drive.status()
                if other_task_active(before):raise ValueError('先结束正在运行的单项测试或付款记录')
                if payload.get('setup_token')!=before.get('setup_token'):raise ValueError('页面已更新，请刷新')
                if any(payload.get(k) is not True for k in ('stationary_confirmed','in_switch_zone','fb_mode_confirmed')):
                    raise ValueError('请在切换区停稳，保持 F/B 后进入自动模式')
                client=payload.get('client')
                if not isinstance(client,str) or not 1<=len(client)<=128:raise ValueError('页面身份无效')
                bundle=load_bundle(self.root,self.drive)
                readiness=parking_readiness(bundle)
                entry=start_readiness(bundle,readiness)
                if self.require_ready and not entry['ready']:
                    raise ValueError('自主驾驶启动条件尚未就绪：'+'、'.join(entry['missing']))
                # Release the authenticated manual session before acquiring UART.
                # This is not a fabricated ESC-off declaration for another trial.
                if before.get('owner_client') not in (None,client):
                    raise ValueError('遥控由另一页面持有，请在原页面切换')
                record=self.root/'run'/('race-flow-'+str(time.time_ns()))
                record.mkdir(parents=True)
                (record/'settings.json').write_text(json.dumps(bundle,ensure_ascii=False,indent=2),encoding='utf-8')
                self.bundle=bundle;self.owner=client;self.token=secrets.token_hex(16);self.sequence=-1
                self.cancel_event=threading.Event();self.lease_until=self.clock()+.5;self.active=True
                self.controller=RaceFlowController(bundle,self.clock());self.stage='handoff';self.reason='正在停车并交接控制权'
                self.handoff={'requested_s':self.clock(),'shared_executor':False}
                self.error=None;self.speech={};self.payment_scanned=False;self.last_intent=None
                self.record=record;self.record_error=None
                self.thread=threading.Thread(target=self._work,args=(self.cancel_event,self.token,payload),name='race-flow',daemon=True)
                try:self.thread.start()
                except Exception:
                    self.cancel_event.set();self.active=False;self.thread=None
                    self.stage='fault';self.reason='自动任务线程未能启动'
                    raise
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

    def _work(self,cancel=None,token=None,handoff_payload=None):
        cancel=self.cancel_event if cancel is None else cancel
        token=self.token if token is None else token
        payload=handoff_payload or {}
        handle=None
        try:
            reader=self.reader_factory(self.states,self.bundle,self.root,self.clock);self.reader=reader
            shared=bool(callable(getattr(self.drive,'borrow_actuator',None)))
            if payload:
                self.drive.request('stop',{'client':self.owner,'stop_source':'key_release'})
                if not shared and not outputs_released(self.drive.status()):
                    self.drive.request('finish_test',payload)
                    deadline=self.clock()+8
                    while not outputs_released(self.drive.status()):
                        if cancel.wait(.01):return
                        if self.clock()>deadline:raise ValueError('原遥控清理超时，未接管串口')
            if cancel.is_set():return
            handle=self.factory(self.bundle)
            with self.lock:
                if cancel.is_set() or not self._current(cancel,token):return
                self.actuator=handle
                # The 3-second neutral period starts after the actuator reports
                # neutral, not before potentially slow UART initialization.
                self.controller=RaceFlowController(self.bundle,self.clock())
                self.stage=self.controller.stage;self.reason=self.controller.reason
                self.handoff.update(executor_ready_s=self.clock(),shared_executor=shared)
            if shared:
                stillness=StillnessWindow();deadline=self.clock()+3
                self.stage='handoff';self.reason='执行器已就绪，正在核对连续静止画面'
                while not cancel.is_set():
                    now=self.clock()
                    if now>deadline:raise ValueError('交接时未取得连续静止反馈，保持停止')
                    if now>self.lease_until:raise ValueError('交接期间页面心跳中断')
                    handle.send({'action':'neutral','esc_us':1500,'steering_us':1610})
                    sample=reader.read('arming',now)
                    if sample:
                        if stillness.update(sample.get('motion'),now):break
                    elif stillness.last is not None and now-stillness.last>.25:
                        stillness.reset()
                    cancel.wait(.01)
                with self.lock:
                    if cancel.is_set() or not self._current(cancel,token):return
                    self.controller=RaceFlowController(self.bundle,self.clock(),initialized=True)
                    self.stage=self.controller.stage;self.reason=self.controller.reason
                    self.handoff['wait_board_s']=self.clock()
            self._record('manual_to_auto_handoff')
            while not cancel.is_set():
                now=self.clock()
                with self.lock:
                    if not self._current(cancel,token):return
                    if self.stage not in ('payment','complete') and now>self.lease_until:raise ValueError('控制页心跳中断，已结束全流程')
                    stage=self.stage
                if stage in ('payment','complete'):
                    if handle is not None:
                        self._close_handle(handle);handle=None
                        with self.lock:self.actuator=None
                    with self.lock:
                        if cancel.is_set() or not self._current(cancel,token):break
                        if not self.payment_scanned and stage=='payment':
                            self.drive.request('payment_scan',{'client':self.owner});self.payment_scanned=True
                        self.controller.step(None,self.clock());self.stage=self.controller.stage;self.reason=self.controller.reason
                    cancel.wait(.1);continue
                hint=self.last_intent or {}
                if hasattr(reader,'set_avoidance_target'):
                    reader.set_avoidance_target(hint.get('target_camera') if stage=='cones' else None,
                        hint.get('target_id') if stage=='cones' else None)
                sample=reader.read(stage,now)
                if sample is None:
                    # No old image may keep a moving actuator lease alive.
                    limit=self.controller.frame_age_limit() if self.controller else .25
                    if self.updated is not None and self.clock()-self.updated>limit:raise ValueError('相机停止更新，已结束全流程')
                    cancel.wait(.005);continue
                now=self.clock()
                with self.lock:
                    if cancel.is_set() or not self._current(cancel,token):break
                    audio={key:obj.status()['state'] for key,obj in self.speech.items()}
                    intent=self.controller.step(sample,now,audio);self.stage=self.controller.stage;self.reason=self.controller.reason
                    self.preview=reader.preview;self.updated=sample['captured_s']
                    if intent is None:continue
                    if self.stage in ('parking_search','parking_entry','payment'):
                        missing=parking_blockers(self.bundle)
                        if missing:
                            # Reject the first parking-search forward intent
                            # before it reaches the actuator. Finally releases
                            # through its existing bounded brake/neutral path.
                            raise ValueError('前段已结束，停车条件待补，已停止自主流程：'+'、'.join(missing))
                    self.last_intent=intent
                    if self.stage=='fault':raise ValueError(self.reason)
                    # Linearize cancel against every hardware send.
                    if intent.get('fresh_control') is not False or intent['esc_us']<=1500:
                        handle.send(intent)
                    for event in intent.get('events',[]):
                        if event['event']=='speak' and event['id'] not in self.speech:
                            speech=self.speech_factory(event['text']);self.speech[event['id']]=speech;speech.start()
                    key=(self.stage,intent['action'],intent['reason'],
                         intent.get('source_camera'),bool(intent.get('distance_disagreement')))
                    if key!=self.log_key:self._record('decision');self.log_key=key
                cancel.wait(.005)
        except Exception as exc:
            with self.lock:
                if self._current(cancel,token):
                    self.stage='fault';self.reason=str(exc)
                    if self.controller:self.controller.fail(str(exc))
                    cancel.set()
        finally:
            if handle is not None:self._close_handle(handle)
            with self.lock:
                if self._current(cancel,token):
                    self.actuator=None
                    if self.stage not in ('payment','complete'):self.active=False
                    self._safe_record('worker_stopped')
                    for speech in self.speech.values():
                        try:speech.close()
                        except Exception as exc:self.record_error=str(exc)

    def _close_handle(self,handle):
        try:
            handle.close()
            if getattr(handle,'close_error',None):
                raise RuntimeError(handle.close_error)
        except Exception as exc:
            # A cleanup failure must never make the actuator appear reusable.
            self.cleanup_error=str(exc)
            self.reason='执行器停止未确认：'+str(exc)
            self.stage='fault'
        return self.cleanup_error is None

    def _safe_record(self,event):
        try:self._record(event)
        except (OSError,ValueError,TypeError) as exc:self.record_error=str(exc)

    def cancel(self,reason):
        with self.lock:
            self.cancel_event.set()
            if self.stage!='fault':self.reason=reason
            if self.active and self.stage!='fault':
                self.stage='cancelled'
                if self.controller:self.controller.stage='cancelled';self.controller.reason=reason
            handle=self.actuator;self.actuator=None
        if handle:self._close_handle(handle)
        if self.thread and self.thread is not threading.current_thread():self.thread.join(timeout=4)
        if self.payment_scanned:
            try:self.drive.request('payment_scan_stop',{'client':self.owner})
            except (ValueError,RuntimeError):pass
            self.payment_scanned=False
        with self.lock:
            self.active=self._thread_alive()
            if self.active:self.reason='正在等待旧执行器完成停止和释放'
            self._safe_record('cancelled')

    def close(self):
        self.closed=True;self.cancel('服务结束，已停止全流程')
