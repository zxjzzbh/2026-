"""Independent latest-cone task for the installed debug registry. Author: 13246.

Perception/control extend the 7fb9989 baseline with bounded lane recovery.
Hardware output uses the
installed vehicle actuator, acquired only by explicit prepare/start actions.
"""
import copy
import json
from pathlib import Path
import time

from .cone_latest_engine.cone_observer import ConeObserverDrive
from .cone_latest_engine.cone_low_speed import trial_settings,PROFILE
from .cone_latest_engine.cone_navigation import actuator_settings
from .cone_latest_engine.cone_power import with_reduced_power
from .cone_latest_engine.cone_planning import validate_settings
from .cone_recording import ConeRecording

VERSION='latest-cone-7fb9989'
FIELD_REVISION='cone-first-maneuver-20261108'
DEFAULTS={'test_forward_us':1600,'test_left_us':1870,'test_right_us':1350,'test_steering_gain':1.0}


def installed_actuator(settings):
    # Preserve the currently installed worker, UART protocol and hardware lock.
    from .cone_trial_output import ConeTrialActuator
    from .cone_command import ConeCommandSender
    options=settings['low_speed_test']
    return ConeCommandSender(ConeTrialActuator(with_reduced_power(actuator_settings(settings)),right_us=options['right_us'],
        left_us=options['left_us'],run=True,esc_mode='F/B'))


class LatestConeTask:
    def __init__(self,base,states,root,clock=time.monotonic,*,actuator_factory=None):
        self.base,self.root,self.clock=base,Path(root),clock
        self.inner=ConeObserverDrive(base,states,self.root/'vision/configs/cone-latest-20261029.json',
            self.root,clock,actuator_factory=actuator_factory or installed_actuator)
        self.inner.run.log_root=self.root/'run/cone-latest'
        self.inner.reason='最新版连续绕桶已加载；原锥桶测试保持独立'
        self.recording=ConeRecording(self.root/'run/cone-latest/recordings',states,clock)
        self.inner.on_sample=lambda sample:self.recording.event('observation',{
            'sequence':sample['sequence'],'cameras':sample['cameras']})
        self.inner.run.record_clock=clock
        self.inner.on_observer_stop=lambda:self.recording.finish('observer_stopped')

    def _run_event(self,kind,data):
        path=str(self.inner.run.folder/'decisions.jsonl') if kind=='decision' and self.inner.run.folder else None
        self.recording.event(kind,data,legacy=path)
        if kind=='stop':self.recording.finish(data['reason'])

    def _start_recording(self):
        self.recording.start(self.settings)
        # Disabled recording retains the original decisions.jsonl behavior.
        self.inner.run.event_sink=self._run_event if self.recording.current is not None else None

    @property
    def settings(self):return self.inner.settings

    def status(self):
        original=self.base.status();combined=self.inner.status();row=combined['cone_trial']
        row.update(program=VERSION,algorithm_revision='7fb9989',logic_revision=FIELD_REVISION,separate_from_existing=True)
        row['parameters']={**DEFAULTS,**self.settings.get('latest_trial',{})}
        row['avoidance']['readiness']={'ready':True,'missing':[]}
        row['recording']=self.recording.status()
        row['recording_revision']='dual-camera-recording-20261029'
        row['recording_settings']={'enabled':True,'max_mb':512,**self.settings.get('latest_recording',{})}
        row['recording_settings']['mode']='on' if row['recording_settings']['enabled'] else 'off'
        return {**original,'cone_latest':row,'cone_latest_status_s':self.clock()}

    def image(self):
        with self.inner.lock:
            fresh=self.inner.observing and self.inner.updated is not None and 0<=self.clock()-self.inner.updated<=.25
            return self.inner.preview if fresh else None

    def request(self,action,payload):
        if not isinstance(payload,dict):raise ValueError('JSON object required')
        with self.inner.lock:
            if action=='status':return self.status()
            if action in ('stop','emergency','finish_test','latest_cone_stop'):
                if action=='latest_cone_stop' and self.inner.run.active and payload.get('client')!=self.inner.run.owner:
                    raise ValueError('本页未持有最新版锥桶控制权')
                if self.inner.run.active:self.inner.run.cancel('最新版锥桶测试已由操作者停止')
                self.inner.stop.set();self.inner.reason='最新版锥桶识别与行驶已停止'
                self.recording.finish(self.inner.reason)
                return self.status()
            allowed={'latest_cone_settings','latest_cone_observe','latest_cone_prepare','latest_cone_start','latest_cone_keepalive'}
            if action not in allowed:raise ValueError('未知的最新版锥桶动作')
            if action=='latest_cone_settings':
                if self.inner.run.active or self.inner.observing:raise ValueError('先停止本项，再保存参数')
                candidate=copy.deepcopy(self.settings)
                surface=payload.get('scene_surface',candidate['scene']['surface'])
                if surface not in ('red_running_track','indoor_white_line_track'):raise ValueError('本版识别蓝色锥桶，请选红色跑道或室内白线场地')
                candidate['scene']['surface']=surface
                candidate['scene']['white_v_min']=payload.get('white_v_min',candidate['scene'].get('white_v_min',200))
                params={**DEFAULTS,**candidate.get('latest_trial',{})}
                for key in DEFAULTS:
                    if key in payload:params[key]=payload[key]
                trial_settings(candidate,params);validate_settings(candidate)
                candidate['latest_trial']=params
                recording={**candidate.get('latest_recording',{})}
                if 'record_enabled' in payload:
                    if payload['record_enabled'] not in ('on','off'):raise ValueError('录像开关无效')
                    recording['enabled']=payload['record_enabled']=='on'
                if 'record_max_mb' in payload:
                    value=payload['record_max_mb']
                    if type(value) not in (int,float) or not 32<=value<=2048 or int(value)!=value:raise ValueError('记录容量应为 32–2048 MB 的整数')
                    recording['max_mb']=int(value)
                candidate['latest_recording']=recording;self.inner._save(candidate)
                self.inner.reset_reader=True
            elif action=='latest_cone_prepare':
                body={**payload,**DEFAULTS,**self.settings.get('latest_trial',{}),'profile':PROFILE}
                before=self.recording.current
                self._start_recording()
                try:self.inner.request('cone_prepare',body)
                except Exception:
                    if self.recording.current is not before:self.recording.finish('prepare_rejected')
                    raise
            elif action=='latest_cone_observe':
                if self.inner.run.active:raise ValueError('本项正在运行')
                self.inner.run.profile=PROFILE
                self.inner.run.s=trial_settings(self.settings,{**DEFAULTS,**self.settings.get('latest_trial',{})})
                self.inner.reset_navigation=True
                self._start_recording()
                self.inner.request('cone_observe',payload)
            else:
                self.inner.request(action.replace('latest_cone_','cone_',1),payload)
            return self.status()

    def close(self):
        try:self.inner.close()
        finally:self.recording.close()


def install(application):
    """Extend the live debug registry without changing its existing task code."""
    if getattr(application,'_latest_cone_installed',False):return
    original=application.DebugApplication;owners=application.active_owners;images=application.register_images
    class LatestDebugApplication(original):
        def __init__(self,manual,states,root):
            super().__init__(manual,states,root)
            self.tasks['cone_latest']=LatestConeTask(application.BaseView(self),states,root)
            self.prefixes['latest_cone_']='cone_latest'

        def request(self,action,payload):
            if action=='latest_cone_page_heartbeat':
                if self.closed:raise ValueError('调试服务已关闭')
                if not isinstance(payload,dict):raise ValueError('JSON object required')
                return self.tasks['cone_latest'].inner.run.heartbeat(payload,self.tasks['cone_latest'].clock())
            return super().request(action,payload)

        @staticmethod
        def task_active(key,state):
            if key=='cone_latest':return bool(((state.get('cone_latest') or {}).get('avoidance') or {}).get('active'))
            return original.task_active(key,state)

    def active_owners(state):
        result=owners(state);row=(state.get('cone_latest') or {}).get('avoidance') or {}
        if row.get('active'):result.append(row)
        return result

    def register_images(server,app,*,mode):
        images(server,app,mode=mode)
        if mode=='debug':
            server.routes['/api/cone-latest/frame.jpg']=lambda _:application.image_response(app.tasks['cone_latest'].image())
            from .cone_replay import response
            root=app.tasks['cone_latest'].recording.root
            for suffix,kind in (('replay.html','html'),('recording.json','data'),('recorded-frame.jpg','frame')):
                server.routes['/api/cone-latest/'+suffix]=lambda url,kind=kind:response(root,url,kind)

    application.DebugApplication=LatestDebugApplication
    application.active_owners=active_owners
    application.register_images=register_images
    application._latest_cone_installed=True
