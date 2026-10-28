"""Speech-only departure trial on existing camera caches; never opens UART."""
import json
import secrets
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .departure import (CAMERAS,MAX_AGE_S,PHRASE,DEFAULT_MIN_BLUE_PERCENT,
                        DEFAULT_CLEAR_CONFIRM_S,DepartureSequence,detect_board,
                        validate_blue_percent)
from .payment_console import outputs_released
from .parking_speech import ParkingSpeech


class DepartureTrialDrive:
    def __init__(self,drive,states,root,*,clock=time.monotonic,speech_factory=None):
        self.drive,self.states,self.root,self.clock=drive,states,Path(root),clock
        self.speech_factory=speech_factory or (lambda:ParkingSpeech(self.root,text=PHRASE))
        self.lock=threading.RLock();self.stop=threading.Event();self.thread=None
        self.sequence=DepartureSequence();self.active=False;self.closed=False;self.speech=None
        self.observations={};self.preview=None;self.preview_at=None;self.folder=None
        self.started=None;self.owner=None;self.last_ids={};self.frame_images={};self.last_logged=None
        self.phase='idle';self.reason='发车测试只识别挡板并播报，不驱动车辆'
        self.settings={'min_blue_percent':DEFAULT_MIN_BLUE_PERCENT,'clear_confirm_s':DEFAULT_CLEAR_CONFIRM_S}
        self.settings_path=self.root/'vision/configs/departure.local.json';self.settings_error=None
        if self.settings_path.exists():
            try:self.settings=self._parameters(json.loads(self.settings_path.read_text(encoding='utf-8')))
            except (OSError,ValueError,TypeError):self.settings_error='已保存的发车参数无效，请重新填写并保存'
        self.active_settings=dict(self.settings);self.sequence=DepartureSequence(self.settings['clear_confirm_s'])
        self.settings_revision=secrets.token_hex(8)
        self.speech_test=None;self.last_speech_state=None;self.last_observation_key=None

    def _parameters(self,payload):
        if not isinstance(payload,dict):raise ValueError('发车参数须为JSON对象')
        area=validate_blue_percent(payload.get('min_blue_percent',self.settings['min_blue_percent']))
        clear=DepartureSequence(payload.get('clear_confirm_s',self.settings['clear_confirm_s'])).clear_s
        return {'min_blue_percent':area,'clear_confirm_s':clear}

    def _save(self,settings):
        self.settings_path.parent.mkdir(parents=True,exist_ok=True)
        temp=self.settings_path.with_suffix('.json.tmp')
        temp.write_text(json.dumps(settings,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8');temp.replace(self.settings_path)
        self.settings=settings;self.settings_error=None;self.settings_revision=secrets.token_hex(8)

    def status(self):
        with self.lock:
            s=self.drive.status()
            editable=(not self.active and not self.closed and not (self.thread and self.thread.is_alive())
                       and outputs_released(s) and not (s.get('parking_payment') or {}).get('motion_locked'))
            speech_busy=any(v is not None and v.status().get('state')=='sending' for v in (self.speech,self.speech_test))
            can_start=editable and not self.settings_error and not speech_busy
            if self.speech:
                speech_state=self.speech.status().get('state')
                if speech_state in ('command_sent','failed','cancelled') and speech_state!=self.last_speech_state:
                    self.last_speech_state=speech_state;self._record('speech_result')
            age=None if self.preview_at is None else max(0,self.clock()-self.preview_at)
            s['departure_trial']={'program':'blue-board-speech-v1','active':self.active,'can_start':can_start,
                'phase':self.phase,'reason':self.reason,'owner_client':self.owner,'speech_only':True,
                'settings':dict(self.settings),'settings_revision':self.settings_revision,'settings_error':self.settings_error,
                'can_save':editable,'can_test_speech':editable and not speech_busy,'speech_test':self.speech_test.status() if self.speech_test else None,
                'forward_output_enabled':False,'board_dimensions_cm':{'width':40,'height':50},
                'clear_confirm_s':self.sequence.clear_s,'min_blue_percent':self.active_settings['min_blue_percent'],'clear_elapsed_s':self.sequence.clear_elapsed,
                'board_seen_cameras':list(self.sequence.seen_cameras),'observations':dict(self.observations),
                'diagnostics':self.sequence.diagnostics(),
                'frame_age_s':age,'speech':self.speech.status() if self.speech else None,
                'record_directory':str(self.folder) if self.folder else None}
            s['departure_status_s']=self.clock()
            return s

    def request(self,action,payload):
        with self.lock:
            if action=='status':return self.status()
            if action in ('departure_save','departure_test_speech'):
                state=self.status()['departure_trial']
                allowed=state['can_save'] if action=='departure_save' else state['can_test_speech']
                if not allowed:raise ValueError('先停止当前测试或驾驶任务，再保存参数或试听')
                if payload.get('setup_token')!=self.drive.status().get('setup_token'):raise ValueError('页面已更新，请刷新')
                if action=='departure_save':
                    if payload.get('settings_revision')!=self.settings_revision:raise ValueError('参数已被其他页面修改，请刷新核对')
                    self._save(self._parameters(payload))
                else:
                    if self.speech_test:self.speech_test.close()
                    self.speech_test=self.speech_factory();self.speech_test.start()
                return self.status()
            if action=='departure_start':
                if not self.status()['departure_trial']['can_start']:raise ValueError('先结束当前驾驶或付款流程，再开始发车识别测试')
                if payload.get('setup_token')!=self.drive.status().get('setup_token'):raise ValueError('页面已更新，请刷新后开始')
                client=payload.get('client')
                if not isinstance(client,str) or not 1<=len(client)<=128:raise ValueError('页面身份无效')
                if payload.get('settings_revision',self.settings_revision)!=self.settings_revision:raise ValueError('参数已变化，请刷新后开始')
                settings=self._parameters(payload);seq=DepartureSequence(settings['clear_confirm_s'])
                self._save(settings);self.active_settings=dict(settings)
                if self.speech:self.speech.close()
                if self.speech_test:self.speech_test.close()
                self.speech_test=None;self.last_speech_state=None;self.last_observation_key=None
                self.sequence=seq;self.speech=None;self.phase=seq.phase;self.reason=seq.reason
                self.observations={};self.preview=None;self.preview_at=None;self.last_ids={};self.frame_images={}
                self.started=self.clock();self.owner=client
                self.folder=self.root/'run'/('departure-'+str(time.time_ns()));self.folder.mkdir(parents=True,exist_ok=False)
                self.active=True;self.stop.clear();self.last_logged=None;self._record('started')
                self.thread=threading.Thread(target=self._run,name='departure-observation',daemon=True);self.thread.start()
                return self.status()
            if action=='departure_stop':
                self._cancel('测试已停止；不会播报或驱动车辆');return self.status()
            if self.active:
                if action in ('stop','emergency','finish_test'):
                    self._cancel('已停止发车识别测试');return self.status()
                raise ValueError('发车识别测试中；先停止测试，再切换其他任务')
            self.drive.request(action,payload)
            return self.status()

    def _cancel(self,reason):
        self.stop.set();self.active=False;self.phase='cancelled';self.reason=reason
        if self.speech:self.speech.close()
        if self.speech_test:self.speech_test.close()
        self._record('cancelled')

    def _record(self,event):
        if self.folder is None:return
        data={'event':event,'monotonic_s':self.clock(),'phase':self.phase,'reason':self.reason,
              'observations':self.observations,'speech':self.speech.status() if self.speech else None,
              'vehicle_commands_sent':False,'forward_output_enabled':False,'board_dimensions_cm':{'width':40,'height':50}}
        data['settings']=dict(self.active_settings)
        with (self.folder/'events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(data,ensure_ascii=False,allow_nan=False)+'\n')
        if self.preview:(self.folder/(event+'-'+str(time.time_ns())+'.jpg')).write_bytes(self.preview)

    def _observe(self):
        rows={};panes=[];new_ids={}
        for camera in CAMERAS:
            try:
                state=self.states[camera]
                with state.condition:
                    jpg,fid,ms=state.raw_jpeg,state.raw_frame_id,state.raw_received_ms
                    failed=state.state in ('error','stopped')
                captured=ms/1000 if isinstance(ms,(int,float)) else -1
                if failed or not jpg or type(fid) is not int or not 0<=self.clock()-captured<=MAX_AGE_S:raise ValueError('无新鲜画面')
                if (fid,captured)==self.last_ids.get(camera):
                    row=self.observations[camera];image=self.frame_images[camera].copy()
                else:
                    image=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
                    if image is None:raise ValueError('图像解码失败')
                    row={**detect_board(image,self.active_settings['min_blue_percent']),'frame_id':fid,'captured_s':captured,'camera_id':camera}
                rows[camera]=row;new_ids[camera]=(fid,captured);self.frame_images[camera]=image.copy()
                for x1,y1,x2,y2 in row['boxes']:cv2.rectangle(image,(x1,y1),(x2,y2),(0,220,255),2)
                cv2.putText(image,f"{camera.upper()} #{fid} {row['state'].upper()}",(8,24),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,200,255),2)
            except (ValueError,KeyError,cv2.error):
                rows[camera]={'state':'unknown','reason':'无新鲜画面','camera_id':camera}
                image=np.zeros((360,480,3),np.uint8)
                cv2.putText(image,camera.upper()+' NO FRESH FRAME',(8,24),cv2.FONT_HERSHEY_SIMPLEX,.55,(160,160,160),1)
            panes.append(cv2.resize(image,(480,360)))
        self.last_ids=new_ids
        ok,jpg=cv2.imencode('.jpg',np.concatenate(panes,axis=1),[cv2.IMWRITE_JPEG_QUALITY,85])
        return rows,jpg.tobytes() if ok else None

    def _run(self):
        try:
            while not self.stop.is_set():
                with self.lock:
                    if not self.active:break
                    if self.clock()-self.started>120:
                        self._cancel('两分钟测试窗口已结束，请重新开始');break
                    rows,preview=self._observe();self.observations=rows;self.preview=preview;self.preview_at=self.clock()
                    completed=self.sequence.update(rows,self.clock())
                    self.phase=self.sequence.phase;self.reason=self.sequence.reason
                    key=tuple((c,r.get('frame_id'),r['state']) for c,r in rows.items())
                    if key!=self.last_observation_key:
                        sample={'monotonic_s':self.clock(),'phase':self.phase,'reason':self.reason,'observations':rows,
                                'diagnostics':self.sequence.diagnostics(),
                                'clear_elapsed_s':self.sequence.clear_elapsed,'settings':self.active_settings}
                        with (self.folder/'observations.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(sample,ensure_ascii=False,allow_nan=False)+'\n')
                        self.last_observation_key=key
                    if self.phase!=self.last_logged:
                        self._record(self.phase);self.last_logged=self.phase
                    if completed:
                        self.speech=self.speech_factory();self.speech.start();self._record('speech_requested')
                        self.active=False;break
                if self.stop.wait(.12):break
        except Exception as exc:
            with self.lock:
                self.active=False;self.phase='fault';self.reason='发车识别异常：'+str(exc)
                self._record('fault')

    def image(self):
        with self.lock:
            if self.preview_at is None or (self.phase!='complete' and self.clock()-self.preview_at>1):return None
            return self.preview

    def close(self):
        with self.lock:
            self.closed=True;self.stop.set();self.active=False
            if self.speech:self.speech.close()
            if self.speech_test:self.speech_test.close()
        if self.thread and self.thread is not threading.current_thread():self.thread.join(timeout=2)
