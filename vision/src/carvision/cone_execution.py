"""Exclusive, leased cone-driving session; explicit prepare/start only."""
import copy
import secrets
import json
import time
from pathlib import Path

from .brake_output import BrakeActuator
from .cone_navigation import execution_readiness,actuator_settings,ConeDrivingController
from .cone_low_speed import PROFILE,trial_settings,LowSpeedController


def base_idle(base):
    return (base.get('mode')=='disabled' and base.get('boot_test_phase')=='esc_off'
        and not any(base.get(k) for k in ('hardware_output','worker_started','pending_hardware_start'))
        and not any((base.get(k) or {}).get('active') for k in ('traffic_trial','crosswalk_trial')))


class ConeRun:
    def __init__(self,actuator_factory=None,log_root=None):
        self.factory=actuator_factory or (lambda s:BrakeActuator(actuator_settings(s),run=True,esc_mode='F/B',forward_limit_us=1625))
        self.active=False;self.phase='idle';self.reason='尚未准备避障';self.actuator=None
        self.owner=self.token=None;self.sequence=-1;self.lease_until=0.;self.prepared_s=0.
        self.s=self.controller=self.navigation=self.intent=self.execution=None
        self.log_root=Path(log_root) if log_root else None;self.folder=self.log=None;self.last_record=None
        self.profile='calibrated'

    def status(self,settings,base,now):
        ready=execution_readiness(settings);nav=self.navigation or {}
        can_start=(self.active and self.phase=='ready' and nav.get('action')=='follow_candidate_path'
            and 0<=now-nav.get('captured_s',-1e9)<=.25 and abs((nav.get('motion') or {}).get('speed_mps',1))<=.025)
        if self.profile==PROFILE:
            can_start=(self.active and self.phase=='ready' and nav.get('experimental') and nav.get('ready')
                       and 0<=now-nav.get('captured_s',-1e9)<=.25)
        return {'active':self.active,'phase':self.phase,'reason':self.reason,'owner_client':self.owner,'run_token':self.token,
            'profile':self.profile,'low_speed_test':self.controller.progress() if isinstance(self.controller,LowSpeedController) else None,
            'can_prepare':not self.active and base_idle(base),'can_start':can_start,
            'can_request_start':self.active and self.phase=='ready',
            'hardware_output':bool(self.actuator and getattr(self.actuator,'run',False)),
            'readiness':ready,'intent':self.intent,'field_verified':False,'log_directory':str(self.folder) if self.folder else None,
            'slalom_required':True,'slalom':nav.get('slalom'),
            'start_blocked_reason':'' if can_start else ('需要任一路可靠白线及新鲜画面，保持 F/B 并确认静止；无需先看到锥桶' if self.profile==PROFILE
                else '需要完成标定、确认两桶与左右白线、得到交错路线与静止反馈')}

    def prepare(self,settings,base,payload,now):
        if self.active or not base_idle(base):raise ValueError('先结束当前遥控、斑马线或红绿灯测试')
        profile=payload.get('profile','calibrated')
        if profile not in ('calibrated',PROFILE):raise ValueError('未知避障测试类型')
        candidate=trial_settings(settings,payload) if profile==PROFILE else copy.deepcopy(settings)
        if profile!='legacy_low_speed':
            report=execution_readiness(settings)
            if not report['ready']:raise ValueError('避障标定未完成：'+', '.join(report['missing']))
        if payload.get('setup_token')!=base.get('setup_token'):raise ValueError('页面已变化，请刷新')
        if payload.get('esc_off') is not True or payload.get('fb_mode_confirmed') is not True:raise ValueError('准备时电调关闭、保持 F/B')
        client=payload.get('client')
        if not isinstance(client,str) or not 1<=len(client)<=128:raise ValueError('页面身份无效')
        self.s=candidate;self.profile=profile;self.active=True;self.phase='preparing'
        self.reason='等待任一路可靠白线；旧数据持续低速试验' if profile==PROFILE else '等待新鲜跑道画面与地面位移反馈'
        self.owner,self.token=client,secrets.token_hex(16);self.sequence=-1;self.lease_until=now+.5;self.prepared_s=now
        self.controller=self.navigation=self.intent=self.execution=None
        if self.log_root:
            try:
                self.folder=self.log_root/('cone-drive-'+str(time.time_ns()));self.folder.mkdir(parents=True,exist_ok=False)
                (self.folder/'settings.json').write_text(json.dumps(self.s,ensure_ascii=False,indent=2),encoding='utf-8')
                self.log=(self.folder/'decisions.jsonl').open('w',encoding='utf-8')
            except Exception:
                self.active=False;self.phase='fault';raise

    def command(self,action,payload,now):
        if not self.active or payload.get('client')!=self.owner or payload.get('run_token')!=self.token:raise ValueError('本页未持有当前避障控制权')
        if now>self.lease_until:
            self.cancel('页面心跳超时，必须重新准备',phase='fault');raise ValueError(self.reason)
        sequence=payload.get('trial_sequence')
        if type(sequence) is not int or sequence<=self.sequence:raise ValueError('old or invalid cone sequence')
        if action=='cone_start':
            nav=self.navigation or {}
            acceptable=(nav.get('experimental') and nav.get('ready')) if self.profile==PROFILE else (
                nav.get('action')=='follow_candidate_path' and abs((nav.get('motion') or {}).get('speed_mps',1))<=.025)
            if (self.phase!='ready' or not acceptable
                    or not 0<=now-nav.get('captured_s',-1e9)<=.25
                    ):
                raise ValueError('暂不能开始：需要新鲜白线和无未确认近处障碍的画面')
            if any(payload.get(k) is not True for k in ('fb_mode_confirmed','esc_on_static','field_ready')):raise ValueError('开始前需 F/B、电调打开且静止、场地就绪')
            self.controller=LowSpeedController(self.s,now) if self.profile==PROFILE else ConeDrivingController(self.s,now)
            self.phase='arming';self.reason='3 秒中位后开始低速避障'
        elif action=='cone_test_confirm':
            if not isinstance(self.controller,LowSpeedController) or payload.get('tail_passed') is not True:
                raise ValueError('仅低速测试允许现场确认车尾通过')
            self.controller.confirm_pass()
        elif action!='cone_keepalive':raise ValueError('未知避障请求')
        self.sequence=sequence;self.lease_until=now+.5

    def tick(self,navigation,now):
        if not self.active:return
        try:
            if now>self.lease_until:raise ValueError('控制页心跳中断，已结束避障')
            if self.controller is None and now-self.prepared_s>120:raise ValueError('准备等待超时')
            self.navigation=navigation
            motion=(navigation or {}).get('motion') or {}
            fresh=navigation and 0<=now-navigation.get('captured_s',-1e9)<=.25
            feedback_ok=bool(motion.get('valid')) if self.profile!=PROFILE else bool(
                navigation and navigation.get('experimental') and navigation.get('lane_bounds_px'))
            if self.actuator is None:
                if not fresh or not feedback_ok:
                    if now-self.prepared_s>5:raise ValueError('未取得可信跑道与位移反馈，准备失败')
                    return
                token=self.token;handle=self.factory(self.s)
                if not self.active or self.token!=token:
                    handle.close();return
                self.actuator=handle
                if not self.active:
                    handle.close();self.actuator=None;return
            if not fresh or (not feedback_ok and self.profile!=PROFILE):
                raise ValueError((navigation or {}).get('reason','图像／白线或位移反馈失效，主动制动结束'))
            if self.controller is None:
                self.phase='ready';self.reason='准备完成，打开电调并确认静止后开始'
                intent={'action':'neutral','esc_us':1500,'steering_us':1610,'phase':'ready','reason':self.reason}
            else:
                intent=self.controller.step(navigation,now);self.phase,self.reason=intent['phase'],intent['reason']
            handle=self.actuator
            if not self.active or handle is None:return
            self.intent=intent;self.execution=handle.send(intent)
            key=(navigation.get('captured_s'),intent['phase'],intent['action'])
            if self.log and not self.log.closed and key!=self.last_record:
                self.log.write(json.dumps({'now_s':now,'intent':intent,
                    'navigation':{k:v for k,v in navigation.items() if k not in ('path','sample')},
                    'sample':navigation.get('sample'),'path':navigation.get('path',[])[::5],
                    'execution':self.execution},ensure_ascii=False,allow_nan=False)+'\n');self.log.flush();self.last_record=key
            if self.phase=='complete':self.cancel(self.reason,phase='complete')
        except Exception as exc:
            if self.active:self.cancel(str(exc),phase='fault')

    def cancel(self,reason,phase='cancelled'):
        # close() engages the existing finite F/B failsafe if it was driving.
        try:
            if self.actuator:self.actuator.close()
        finally:
            self.actuator=None;self.active=False;self.phase=phase;self.reason=reason
            if self.log and not self.log.closed:self.log.close()
            if self.folder:
                try:(self.folder/'summary.json').write_text(json.dumps({'phase':phase,'reason':reason,'field_verified':False},ensure_ascii=False,indent=2),encoding='utf-8')
                except OSError:pass
