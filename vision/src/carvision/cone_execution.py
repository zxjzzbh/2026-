"""Exclusive, leased cone-driving session; explicit prepare/start only."""
from .runtime_timing import VISION_MAX_AGE_S
import copy
import secrets
import json
import time
import threading
from pathlib import Path

from .brake_output import BrakeActuator
from .cone_navigation import execution_readiness,actuator_settings,ConeDrivingController
from .cone_low_speed import PROFILE,trial_settings,LowSpeedController,DEFAULT_STEERING_GAIN
from .cone_continuous import ContinuousConeController,OBSTACLE_LINE_RECOVERY_S,DEFAULT_STOP_FOR_CONES
from .cone_trial_output import ConeTrialActuator
from .cone_command import ConeCommandSender
from .cone_evidence import StablePairedLane
from .cone_recovery import ConeRecovery,FeedbackPause


from .page_heartbeat import PAGE_LEASE_S as CONE_PAGE_LEASE_S

def make_actuator(settings):
    options=settings.get('low_speed_test')
    if options:
        return ConeCommandSender(ConeTrialActuator(actuator_settings(settings),right_us=options['right_us'],left_us=options['left_us'],run=True,esc_mode='F/B'))
    return BrakeActuator(actuator_settings(settings),run=True,esc_mode='F/B',forward_limit_us=1625)


def base_idle(base):
    return (base.get('mode')=='disabled' and base.get('boot_test_phase')=='esc_off'
        and not any(base.get(k) for k in ('hardware_output','worker_started','pending_hardware_start'))
        and not any((base.get(k) or {}).get('active') for k in ('traffic_trial','crosswalk_trial')))


class ConeRun:
    def __init__(self,actuator_factory=None,log_root=None):
        self.factory=actuator_factory or make_actuator
        self.active=False;self.phase='idle';self.reason='尚未准备避障';self.actuator=None
        self.owner=self.token=None;self.sequence=-1;self.lease_until=0.;self.prepared_s=0.
        self.s=self.controller=self.navigation=self.intent=self.execution=None
        self.log_root=Path(log_root) if log_root else None;self.folder=self.log=None;self.last_record=None
        self.profile='calibrated'
        self.lane_start=StablePairedLane()
        self.first_stop=None;self.stop_history=[];self.cleanup_error=None
        self.failure_navigation=None;self.last_requested_intent=None
        self.output_lock=threading.RLock();self.control_timing=None
        self.recovery=ConeRecovery();self.resume_capture=None
        self.record_error=None
        self.heartbeat_sequence=-1;self.heartbeat_count=0;self.last_heartbeat_s=None;self.maximum_heartbeat_gap_s=0.

    def status(self,settings,base,now):
        ready=execution_readiness(settings);nav=self.navigation or {}
        turn=(self.s or {}).get('low_speed_test') or {}
        can_start=(self.active and self.phase=='ready' and nav.get('action')=='follow_candidate_path'
            and 0<=now-nav.get('captured_s',-1e9)<=VISION_MAX_AGE_S and abs((nav.get('motion') or {}).get('speed_mps',1))<=.025)
        if self.profile==PROFILE:
            can_start=(self.active and self.phase=='ready' and nav.get('experimental') and nav.get('ready')
                       and 0<=now-nav.get('captured_s',-1e9)<=VISION_MAX_AGE_S and self.lane_start.ready(now))
        return {'active':self.active,'phase':self.phase,'reason':self.reason,'owner_client':self.owner,'run_token':self.token,
            'profile':self.profile,'low_speed_test':self.controller.progress() if isinstance(self.controller,LowSpeedController) else None,
            'line_recovery_s':OBSTACLE_LINE_RECOVERY_S,
            'default_steering_gain':DEFAULT_STEERING_GAIN,
            'low_speed_stop_for_cones':getattr(self.controller,'stop_for_cones',DEFAULT_STOP_FOR_CONES),
            'steering_limits_us':[turn['right_us'],turn['left_us']] if self.profile==PROFILE and 'left_us' in turn else None,
            'can_prepare':not self.active and not self.cleanup_error and base_idle(base),'can_start':can_start,
            'can_request_start':bool(can_start),
            'hardware_output':bool(self.actuator and getattr(self.actuator,'run',False)),
            'readiness':ready,'intent':self.intent,'field_verified':False,'log_directory':str(self.folder) if self.folder else None,
            'control_timing':self.actuator.timing() if callable(getattr(self.actuator,'timing',None)) else self.control_timing,
            'page_heartbeat':self.heartbeat_status(),
            'first_stop':copy.deepcopy(self.first_stop),'stop_history':copy.deepcopy(self.stop_history),
            'cleanup_error':self.cleanup_error,'record_error':self.record_error,'stable_lane_frames':self.lane_start.count,
            'last_requested_intent':copy.deepcopy(self.last_requested_intent),
            'slalom_required':True,'slalom':nav.get('slalom'),
            'start_blocked_reason':'' if can_start else ('等待至少 3 帧且持续 0.15 秒的新鲜双白线，保持 F/B 并确认静止' if self.profile==PROFILE
                else '需要完成标定、确认两桶与左右白线、得到交错路线与静止反馈')}

    def prepare(self,settings,base,payload,now):
        with self.output_lock:
            return self._prepare(settings,base,payload,now)

    def _prepare(self,settings,base,payload,now):
        if self.cleanup_error:raise ValueError('上轮执行器释放未确认，请重新准备工作进程：'+self.cleanup_error)
        if self.active or not base_idle(base):raise ValueError('先结束当前遥控、斑马线或红绿灯测试')
        profile=payload.get('profile','calibrated')
        if profile not in ('calibrated',PROFILE):raise ValueError('未知避障测试类型')
        candidate=trial_settings(settings,payload) if profile==PROFILE else copy.deepcopy(settings)
        if profile==PROFILE:
            candidate['low_speed_test'].update(manual_stop_only=True,max_run_s=None)
        if profile!='legacy_low_speed':
            report=execution_readiness(settings)
            if not report['ready']:raise ValueError('避障标定未完成：'+', '.join(report['missing']))
        if payload.get('setup_token')!=base.get('setup_token'):raise ValueError('页面已变化，请刷新')
        if payload.get('esc_off') is not True or payload.get('fb_mode_confirmed') is not True:raise ValueError('准备时电调关闭、保持 F/B')
        client=payload.get('client')
        if not isinstance(client,str) or not 1<=len(client)<=128:raise ValueError('页面身份无效')
        self.s=candidate;self.profile=profile;self.active=True;self.phase='preparing'
        self.reason='等待新鲜相机画面；持续低速试验' if profile==PROFILE else '等待新鲜跑道画面与地面位移反馈'
        self.owner,self.token=client,secrets.token_hex(16);self.sequence=-1;self.lease_until=now+CONE_PAGE_LEASE_S;self.prepared_s=now
        self.controller=self.navigation=self.intent=self.execution=None
        self.control_timing=None
        self.recovery=ConeRecovery();self.resume_capture=None
        self.record_error=None
        self.heartbeat_sequence=-1;self.heartbeat_count=0;self.last_heartbeat_s=now;self.maximum_heartbeat_gap_s=0.
        self.first_stop=None;self.stop_history=[];self.cleanup_error=None;self.lane_start.reset();self.last_record=None
        self.failure_navigation=None;self.last_requested_intent=None
        if self.log_root:
            try:
                self.folder=self.log_root/('cone-drive-'+str(time.time_ns()));self.folder.mkdir(parents=True,exist_ok=False)
                (self.folder/'settings.json').write_text(json.dumps(self.s,ensure_ascii=False,indent=2),encoding='utf-8')
                self.log=(self.folder/'decisions.jsonl').open('w',encoding='utf-8')
            except Exception:
                self.active=False;self.phase='fault';raise

    def heartbeat_status(self):
        return {'lease_s':CONE_PAGE_LEASE_S,'last_received_s':self.last_heartbeat_s,
                'maximum_receive_gap_s':self.maximum_heartbeat_gap_s,'received':self.heartbeat_count}

    def heartbeat(self,payload,now):
        # Separate sequence space: an out-of-order heartbeat cannot invalidate
        # a start/stop command or an already accepted newer heartbeat.
        with self.output_lock:
            if (not self.active or payload.get('client')!=self.owner
                    or payload.get('run_token')!=self.token):
                raise ValueError('本页未持有当前避障控制权')
            if now>self.lease_until:
                self.cancel('控制页心跳超过 3 秒，已结束避障',phase='fault',source='page_lease',now=now)
                raise ValueError(self.reason)
            seq=payload.get('heartbeat_sequence')
            if type(seq) is not int or seq<0:raise ValueError('invalid heartbeat sequence')
            accepted=seq>self.heartbeat_sequence
            if accepted:
                if self.last_heartbeat_s is not None:
                    self.maximum_heartbeat_gap_s=max(self.maximum_heartbeat_gap_s,now-self.last_heartbeat_s)
                self.last_heartbeat_s=now;self.heartbeat_count+=1;self.heartbeat_sequence=seq
                self.lease_until=now+CONE_PAGE_LEASE_S
                if callable(getattr(self.actuator,'refresh_page_deadline',None)):
                    self.actuator.refresh_page_deadline(self.lease_until)
            return {'heartbeat_ack':True,'accepted':accepted,'heartbeat_sequence':seq,
                    'server_received_s':now,'page_lease_s':CONE_PAGE_LEASE_S}

    def command(self,action,payload,now):
        with self.output_lock:
            return self._command(action,payload,now)

    def _command(self,action,payload,now):
        if not self.active or payload.get('client')!=self.owner or payload.get('run_token')!=self.token:raise ValueError('本页未持有当前避障控制权')
        if now>self.lease_until:
            self.cancel('页面心跳超时，必须重新准备',phase='fault',source='page_lease',now=now);raise ValueError(self.reason)
        sequence=payload.get('trial_sequence')
        if type(sequence) is not int or sequence<=self.sequence:raise ValueError('old or invalid cone sequence')
        if action=='cone_start':
            nav=self.navigation or {}
            acceptable=(nav.get('experimental') and nav.get('ready') and self.lane_start.ready(now)) if self.profile==PROFILE else (
                nav.get('action')=='follow_candidate_path' and abs((nav.get('motion') or {}).get('speed_mps',1))<=.025)
            if (self.phase!='ready' or not acceptable
                    or not 0<=now-nav.get('captured_s',-1e9)<=VISION_MAX_AGE_S
                    ):
                raise ValueError('暂不能开始：需要连续可靠的双白线与新鲜相机画面')
            if any(payload.get(k) is not True for k in ('fb_mode_confirmed','esc_on_static','field_ready')):raise ValueError('开始前需 F/B、电调打开且静止、场地就绪')
            self.controller=ContinuousConeController(self.s,now) if self.profile==PROFILE else ConeDrivingController(self.s,now)
            self.phase='arming';self.reason='3 秒中位后开始低速避障'
        elif action=='cone_test_confirm':
            if not isinstance(self.controller,LowSpeedController) or payload.get('tail_passed') is not True:
                raise ValueError('仅低速测试允许现场确认车尾通过')
            self.controller.confirm_pass()
        elif action!='cone_keepalive':raise ValueError('未知避障请求')
        self.sequence=sequence;self.lease_until=now+CONE_PAGE_LEASE_S
        if callable(getattr(self.actuator,'refresh_page_deadline',None)):
            self.actuator.refresh_page_deadline(self.lease_until)

    def _send(self,handle,intent,navigation=None):
        if self.profile==PROFILE and callable(getattr(handle,'send_race',None)):
            intent={**intent,'control_profile':'cone'}
            stamp=(navigation or {}).get('captured_s')
            return handle.send_race(intent,run_until=self.lease_until,page_deadline_s=self.lease_until,
                evidence_until=stamp+VISION_MAX_AGE_S if isinstance(stamp,(int,float)) else None,
                recovering=intent.get('control_recovering',False),
                recovered=intent.get('control_recovered',False))
        return handle.send(intent)

    def _record(self,navigation,now,*,motion=None):
        nav=navigation or {};intent=self.intent or {}
        key=(nav.get('captured_s'),intent.get('phase'),intent.get('action'),
             bool(intent.get('control_recovered')))
        if not self.log or self.log.closed or key==self.last_record:return
        try:
            self.log.write(json.dumps({'now_s':now,'intent':intent,
                'navigation':{k:v for k,v in nav.items() if k not in ('path','sample')},
                'motion':motion,'sample':nav.get('sample'),'path':nav.get('path',[])[::5],
                'execution':self.execution},ensure_ascii=False,allow_nan=False)+'\n')
            self.log.flush();self.last_record=key
        except (OSError,ValueError,TypeError) as exc:
            # A recording problem is visible, but is not a control failure.
            self.record_error=str(exc)

    def tick(self,navigation,now,*,motion=None):
        if not self.active:return
        neutral={'action':'neutral','esc_us':1500,'steering_us':1610}
        if self.profile==PROFILE:neutral.update(control_profile='cone',page_deadline_s=self.lease_until)
        try:
            if now>self.lease_until:raise ValueError('控制页心跳中断，已结束避障')
            if self.controller is None and now-self.prepared_s>120:raise ValueError('准备等待超时')
            if (self.controller is None and self.actuator is not None
                    and (navigation or {}).get('captured_s',-1e9)<=getattr(self,'initialized_capture',-1e9)):
                self.navigation=None
                self._send(self.actuator,neutral)
                return
            self.navigation=navigation
            if self.profile==PROFILE:self.lane_start.update(navigation,now)
            motion=motion if motion is not None else (navigation or {}).get('motion') or {}
            fresh=navigation and 0<=now-navigation.get('captured_s',-1e9)<=VISION_MAX_AGE_S
            feedback_ok=bool(motion.get('valid')) if self.profile!=PROFILE else bool(
                navigation and navigation.get('experimental') and navigation.get('ready'))
            if self.actuator is None:
                if not fresh or not feedback_ok:
                    if now-self.prepared_s>5:raise ValueError('等待新鲜可用相机画面超时，准备未完成' if self.profile==PROFILE else '未取得可信跑道与位移反馈，准备失败')
                    return
                token=self.token;handle=self.factory(self.s)
                if not self.active or self.token!=token:
                    handle.close();return
                self.actuator=handle
                self.initialized_capture=navigation['captured_s']
                self.navigation=None
                self.phase='preparing';self.reason='执行器已就绪，等待初始化后的新画面'
                self.intent={**neutral,'phase':self.phase,'reason':self.reason}
                self.execution=self._send(handle,self.intent,navigation)
                return
            if self.controller is None and self.actuator is not None and not fresh:
                self.phase='preparing';self.reason='中位保持，等待新的相机画面'
                self.intent={**neutral,'phase':self.phase,'reason':self.reason}
                self.execution=self._send(self.actuator,self.intent,navigation)
                return
            resumable=(self.profile==PROFILE and self.controller is not None
                       and callable(getattr(self.actuator,'race_interrupted',None)))
            if resumable:
                interrupted=self.actuator.race_interrupted()
                pause=self.recovery.update(navigation,motion,now,interrupted=interrupted,stale=not fresh,
                    was_forward=(self.intent or {}).get('esc_us',1500)>1500)
                identity=tuple((navigation or {}).get(k) for k in ('source_camera','frame_id','captured_s'))
                if pause is not None:
                    if pause.get('control_recovered'):
                        self.controller.recover_control(now);self.resume_capture=identity
                    self.intent=pause;self.phase,self.reason=pause['phase'],pause['reason']
                    with self.output_lock:
                        if self.active and self.actuator is not None:
                            self.execution=self._send(self.actuator,pause,navigation)
                    self._record(navigation,now,motion=motion)
                    return
                if self.resume_capture is not None:
                    if identity==self.resume_capture:
                        self.execution=self._send(self.actuator,neutral,navigation);return
                    self.resume_capture=None
            if not fresh:
                age=now-(navigation or {}).get('captured_s',-1e9)
                raise ValueError(f'避障视觉图像过期：{age*1000:.0f} ms，已结束试验')
            if not feedback_ok and self.profile!=PROFILE:
                raise ValueError((navigation or {}).get('reason','白线或位移反馈失效，主动制动结束'))
            if self.controller is None:
                self.phase='ready';self.reason=('准备完成，打开电调并确认静止后开始' if self.profile!=PROFILE or self.lane_start.ready(now)
                    else '中位准备完成，等待连续可靠双白线后才能开始')
                intent={'action':'neutral','esc_us':1500,'steering_us':1610,'phase':'ready','reason':self.reason}
            else:
                try:
                    intent=self.controller.step(navigation,now)
                except FeedbackPause:
                    if not resumable:raise
                    intent=self.recovery.update(navigation,motion,now,stale=True,
                        was_forward=(self.intent or {}).get('esc_us',1500)>1500)
                self.phase,self.reason=intent['phase'],intent['reason']
            handle=self.actuator
            if not self.active or handle is None:return
            if self.profile==PROFILE:
                intent={**intent,'control_profile':'cone','observation_s':navigation['captured_s'],
                        'page_deadline_s':self.lease_until,
                        'observation_camera':navigation.get('source_camera'),
                        'observation_frame_id':navigation.get('frame_id',navigation['captured_s'])}
            self.intent=intent
            if intent.get('fresh_control') is False:return
            with self.output_lock:
                if not self.active or self.actuator is not handle:return
                self.execution=self._send(handle,intent,navigation)
            self._record(navigation,now,motion=motion)
            if self.phase=='complete':self.cancel(self.reason,phase='complete',source='controller',now=now)
        except Exception as exc:
            if self.active:self.cancel(str(exc),phase='fault',source='controller',now=now)

    def cancel(self,reason,phase='cancelled',*,source='internal',now=None):
        with self.output_lock:
            self._cancel(reason,phase,source=source,now=now)

    def _cancel(self,reason,phase,*,source,now):
        # close() engages the existing finite F/B failsafe if it was driving.
        sources={'internal','controller','page_lease','stop_button','window_blur','page_hidden','page_exit','connection_lost'}
        event={'phase':phase,'reason':str(reason),'source':source if source in sources else 'unknown',
               'at_s':time.monotonic() if now is None else now}
        self.stop_history.append(event);self.stop_history=self.stop_history[-8:]
        if self.first_stop is None:
            self.first_stop=dict(event)
            self.failure_navigation=copy.deepcopy(self.navigation) if phase=='fault' else None
            self.last_requested_intent=copy.deepcopy(self.intent)
        primary=self.first_stop
        handle=self.actuator;self.actuator=None;self.active=False
        try:
            if handle:
                if callable(getattr(handle,'timing',None)):self.control_timing=handle.timing()
                handle.close()
                if getattr(handle,'close_error',None):self.cleanup_error=str(handle.close_error)
        except Exception as exc:
            self.cleanup_error=str(exc)
        finally:
            self.phase=primary['phase'];self.reason=primary['reason']
            self.intent={'action':'neutral','esc_us':1500,'steering_us':1610,'phase':self.phase,
                         'reason':self.reason,'requested_stop':True,'physical_stop_verified':False}
            if self.log and not self.log.closed:self.log.close()
            if self.folder:
                try:(self.folder/'summary.json').write_text(json.dumps({'phase':self.phase,'reason':self.reason,'field_verified':False,
                    'first_stop':self.first_stop,'stop_history':self.stop_history,'cleanup_error':self.cleanup_error,
                    'record_error':self.record_error,
                    'control_timing':self.control_timing,'page_heartbeat':self.heartbeat_status(),
                    'failure_navigation':self.failure_navigation,
                    'last_requested_intent':self.last_requested_intent},ensure_ascii=False,indent=2),encoding='utf-8')
                except OSError:pass
