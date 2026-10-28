"""Operator-controlled whole-course fallback; reuses the installed driver.

No new UART owner, automatic task, QR fabrication or payment API. Timed holds
gate movement at the server; observed/paid declarations remain operator reports.
"""
import copy
import threading
import time
from pathlib import Path

from .parking_speech import ParkingSpeech
from .parking_log import ParkingLog
import json

PHRASES={'team':'我是山东大学启航队的智能车，请为我加油！','park':'我停车了啊'}
STOP={'stop','emergency','finish_test','race_exit','race_cancel'}
MANUAL={'enable','command','speed','select_mode','next_trial','steering_pwm','reverse_trim','reset'}


def idle(s):
    return (s.get('mode')=='disabled' and s.get('boot_test_phase')=='esc_off'
        and not any(s.get(k) for k in ('hardware_output','worker_started','pending_hardware_start'))
        and not any((s.get(k) or {}).get('active') for k in
            ('race_flow','parking_trial','crosswalk_trial','traffic_trial','departure_trial'))
        and not any(((s.get('cone_trial') or {}).get(k) or {}).get('active') for k in ('avoidance','short_test'))
        and not (s.get('parking_payment') or {}).get('motion_locked'))


def finishable_manual(s):
    """Allow the existing stop/finish action, then require an idle new setup."""
    return (s.get('boot_test_phase') == 'ready' and s.get('test_mode') == 'driving'
            and s.get('ground_continuous_trial') is True
            and not s.get('pending_hardware_start')
            and idle({**s, 'mode': 'disabled', 'boot_test_phase': 'esc_off',
                      'hardware_output': False, 'worker_started': False,
                      'pending_hardware_start': False}))


class PlanBDrive:
    def __init__(self,drive,root,*,clock=time.monotonic,speech_factory=None):
        self.drive,self.root,self.clock=drive,Path(root),clock
        self.speech_factory=speech_factory or (lambda text:ParkingSpeech(self.root,text=text))
        self.lock=threading.RLock();self.active=self.closed=False
        self.owner=None;self.phase='idle';self.hold_until=None
        self.reason='选择Plan B后，全程由操作员遥控'
        self.speech=None;self.last_speech=-1e9;self.folder=self.log=None
        self.parked_s=self.reported_paid_s=None

    def _record(self,event):
        if self.log:
            self.log.write(json.dumps({'event':event,'monotonic_s':self.clock(),
                'phase':self.phase,'reason':self.reason,'physical_stop_verified':False,
                'parking_elapsed_s':None if self.parked_s is None else max(0,self.clock()-self.parked_s),
                'operator_reported_within_30s':None if self.reported_paid_s is None else self.reported_paid_s-self.parked_s<=30,
                'payment_provider_verified':False},ensure_ascii=False)+'\n')
            self.log.flush()

    def _refresh(self):
        if self.phase=='crosswalk_hold' and self.hold_until is not None and self.clock()>=self.hold_until:
            self.phase='manual';self.hold_until=None
            self.reason='斑马线10秒已结束；松开全部按键后继续遥控'
            self._record('crosswalk_hold_complete')

    def status(self):
        with self.lock:
            self._refresh();s=self.drive.status()
            p=s.get('parking_payment') or {}
            if self.active and self.phase=='payment' and p.get('motion_locked') and p.get('operator_reported_paid'):
                if self.phase!='complete':
                    self.phase='complete';self.reason='已记录队员支付报告；请展示手机支付成功页';self._record('operator_reported_paid')
            blocked=self.active and self.phase in ('crosswalk_hold','red_hold','payment','complete')
            can_finish = not self.closed and not self.active and finishable_manual(s)
            report={'installed':True,'version':'plan-b-manual-speed-v2','active':self.active,'owner_client':self.owner,
                'phase':self.phase,'reason':('当前普通遥控尚未结束；关闭电调后点击选择按钮，会先结束旧控制再进入Plan B'
                                            if can_finish else self.reason),'movement_blocked':blocked,
                'manual_speed_available':s.get('manual_speed_adjustable') is True,
                'hold_remaining_s':None if self.hold_until is None else max(0,self.hold_until-self.clock()),
                'can_begin':not self.closed and not self.active and idle(s),
                'can_finish_manual':can_finish,
                'speech':self.speech.status() if self.speech else {'state':'not_requested'},
                'phrases':dict(PHRASES),'payment_amount_cents':1,'crosswalk_hold_s':10,
                'payment_remaining_s':None if self.parked_s is None else max(0,30-(self.clock()-self.parked_s)),
                'operator_reported_within_30s':None if self.reported_paid_s is None else self.reported_paid_s-self.parked_s<=30,
                'hardware_adapter':'existing_manual_driver','autonomous_driving':False,
                'physical_stop_verified':False,'payment_provider_verified':False,
                'record_directory':str(self.folder) if self.folder else None}
            return {**s,'plan_b':report,'plan_b_status_s':self.clock()}

    def _owner(self,payload):
        if not self.active or payload.get('client')!=self.owner:
            raise ValueError('请回到选择Plan B的页面操作')

    def request(self,action,payload):
        if not isinstance(payload,dict):raise ValueError('JSON object required')
        with self.lock:
            if self.closed:raise ValueError('Plan B服务已关闭')
            self._refresh()
            if action=='status':return self.status()
            if action in STOP:
                self.drive.request(action,payload);return self.status()
            if action=='plan_b_begin':
                before=self.drive.status()
                if not idle(before):raise ValueError('先结束当前任务并关闭电调，再选择Plan B')
                if payload.get('setup_token')!=before.get('setup_token') or payload.get('esc_off') is not True:
                    raise ValueError('刷新页面并确认电调关闭')
                client=payload.get('client')
                if not isinstance(client,str) or not 8<=len(client)<=100 or not client.isascii():
                    raise ValueError('页面身份无效')
                if before.get('test_mode')!='driving':
                    self.drive.request('select_mode',{**payload,'test_mode':'driving'})
                    before=self.drive.status()
                if before.get('ground_continuous_trial') is not True:
                    raise ValueError('车端尚未配置已验证的持续遥控；本程序不自动扩大控制范围')
                for stop in ('cone_stop','departure_stop','parking_bay_stop'):
                    try:self.drive.request(stop,{'client':client})
                    except ValueError:pass
                if self.log:self.log.close()
                self.folder=self.root/'run'/('plan-b-'+str(time.time_ns()));self.folder.mkdir(parents=True)
                self.log=ParkingLog(self.folder/'events.jsonl')
                self.active=True;self.owner=client;self.phase='manual';self.hold_until=None
                self.parked_s=self.reported_paid_s=None
                self.reason='Plan B已选择；在驾驶面板完成电调关闭/打开的准备步骤'
                self._record('plan_b_selected');return self.status()
            if action=='plan_b_exit':
                self.drive.request('emergency',{'stop_source':'stop_button'})
                before=self.drive.status()
                if before.get('boot_test_phase')=='ready' and not (before.get('parking_payment') or {}).get('motion_locked'):
                    self.drive.request('finish_test',{**payload,'setup_token':before.get('setup_token')})
                self.active=False;self.owner=None;self.hold_until=None;self.phase='idle'
                self.reason='Plan B已结束；车辆不会自动继续行驶';self._record('plan_b_exit')
                return self.status()
            if action.startswith('plan_b_'):
                self._owner(payload)
                if action=='plan_b_announce':
                    kind=payload.get('phrase')
                    if kind not in PHRASES:raise ValueError('请选择固定播报按钮')
                    if self.speech and self.speech.status()['state']=='sending' or self.clock()-self.last_speech<6:
                        raise ValueError('上次播报尚在发送，请稍后再播报')
                    if self.speech:self.speech.close()
                    self.speech=self.speech_factory(PHRASES[kind]);self.speech.start()
                    self.last_speech=self.clock();self._record('announcement_'+kind)
                elif action=='plan_b_hold':
                    kind=payload.get('kind')
                    if kind not in ('crosswalk','red'):raise ValueError('未知停车任务')
                    if self.phase in ('payment','complete'):raise ValueError('缴费阶段保持停车')
                    if kind=='crosswalk' and payload.get('stationary_confirmed') is not True:
                        raise ValueError('请先确认斑马线前已经停稳')
                    self.drive.request('stop',{'client':self.owner,'stop_source':'stop_button'})
                    self.phase='crosswalk_hold' if kind=='crosswalk' else 'red_hold'
                    self.hold_until=self.clock()+10 if kind=='crosswalk' else None
                    self.reason='斑马线停稳声明后锁止10秒' if kind=='crosswalk' else '红灯停车；看到绿灯后点击解锁'
                    self._record(kind+'_hold')
                elif action=='plan_b_green':
                    if self.phase!='red_hold' or payload.get('green_confirmed') is not True:
                        raise ValueError('仅在红灯停车后，确认实际绿灯再解锁')
                    self.phase='manual';self.reason='已确认绿灯；松开按键后继续遥控';self._record('operator_green_confirmed')
                elif action=='plan_b_park':
                    if self.phase!='manual':raise ValueError('先完成当前停车等待，再进入缴费阶段')
                    if payload.get('parked_confirmed') is not True:raise ValueError('请先确认四轮入位且车辆停稳')
                    self.drive.request('stop',{'client':self.owner,'stop_source':'stop_button'})
                    before=self.drive.status()
                    if before.get('boot_test_phase')=='ready':
                        self.drive.request('finish_test',{**payload,'setup_token':before.get('setup_token')})
                    self.phase='payment';self.hold_until=None
                    self.parked_s=self.clock();self.reported_paid_s=None
                    self.reason='已按停车声明锁止；第二路识别二维码后使用下方缴费面板'
                    self._record('operator_parked_confirmed')
                    self.drive.request('payment_scan',{'client':payload.get('payment_client') or self.owner})
                else:raise ValueError('未知Plan B操作')
                return self.status()
            if self.active:
                if action in MANUAL:
                    self._owner(payload)
                    if self.phase in ('crosswalk_hold','red_hold','payment','complete'):
                        raise ValueError(self.reason+'；当前行驶入口锁止')
                    if action=='select_mode' and payload.get('test_mode')!='driving':
                        raise ValueError('Plan B使用持续遥控；先结束本模式再调整PWM')
                elif action.startswith('payment_'):
                    if self.phase not in ('payment','complete'):
                        raise ValueError('先确认入位停稳，再打开缴费流程')
                    if action=='payment_begin' and (self.drive.status().get('parking_payment') or {}).get('outputs_released') is not True:
                        raise ValueError('正在释放驱动控制，完成停车后再打开支付记录')
                else:
                    raise ValueError('Plan B正在全程遥控；先结束本模式再启动自动任务')
            result=self.drive.request(action,payload)
            if (action=='payment_report' and self.active and self.phase=='payment'
                    and (result.get('parking_payment') or {}).get('operator_reported_paid')):
                self.reported_paid_s=self.clock()
            return self.status()

    def close(self):
        with self.lock:
            self.closed=True
            if self.active:
                try:self.drive.request('stop',{'stop_source':'stop_button'})
                except (ValueError,RuntimeError):pass
            if self.speech:self.speech.close()
            if self.log:self.log.close()
            self.active=False
