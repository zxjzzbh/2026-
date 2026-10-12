"""Real cone observation plus explicit, measured and leased avoidance control.

Observation/settings/startup never open the actuator. Only a calibrated,
explicit prepare may acquire neutral output; start is a separate user action.
"""
import copy
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .brake_parking import numeric
from .cone_planning import load_settings, readiness, project_tracks
from .cone_vision import BlueConeDetector, ImageConeTracker
from .cone_road import ConeRoadContext
from .cone_navigation import (ConeNavigator,driving_settings,validate_driving,execution_readiness,import_ground_calibration)
from .cone_execution import ConeRun
from .cone_short_trial import ConeShortRun


class ConeCamera:
    def __init__(self,states,settings,clock=time.monotonic):
        self.states,self.s,self.clock=states,settings,clock
        self.detectors={c:BlueConeDetector(settings) for c in ('primary','secondary')}
        self.trackers={c:ImageConeTracker(c,settings) for c in self.detectors}
        self.ids={};self.views={};self.rows={};self.road=ConeRoadContext(settings)
        self.preview=None;self.sequence=0
        self.navigator=ConeNavigator(settings);self.navigation=None

    def read(self):
        changed=False
        for camera in ('primary','secondary'):
            try:
                state=self.states[camera]
                with state.condition:
                    jpg,fid,ms=state.raw_jpeg,state.raw_frame_id,state.raw_received_ms
                    failed=state.state in ('stopped','error')
                now=self.clock()
                if failed or not jpg or type(fid) is not int or not numeric(ms) or not 0<=now-ms/1000<=self.s['max_frame_age_s']:
                    raise ValueError('camera unavailable')
                if self.ids.get(camera)==fid:continue
                image=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
                if image is None or [image.shape[1],image.shape[0]]!=self.s['image_size']:raise ValueError('image size changed')
                if camera in self.ids and fid<self.ids[camera]:self.trackers[camera]=ImageConeTracker(camera,self.s)
                detections,mask=self.detectors[camera].detect(image)
                context=self.road.inspect(image,mask)
                self.road.enrich(detections,image,context)
                tracks=self.trackers[camera].update(detections,fid,ms/1000)
                row={'camera_id':camera,'frame_id':fid,'captured_s':ms/1000,'tracks':tracks,
                     'lane_valid':context['valid'],'lane_reason':context['reason'],'road_context':context,
                     'metric_cones':[],'metric_unresolved':[]}
                if camera==self.s['camera'] and readiness(self.s)['metric_planning_ready']:
                    row['metric_cones'],row['metric_unresolved']=project_tracks(tracks,self.s)
                if camera==self.s['camera']:
                    try:self.navigation=self.navigator.update(image,row,self.clock())
                    except (ValueError,cv2.error) as exc:
                        self.navigation={'action':'stop','phase':'waiting_for_feedback','reason':str(exc),'hardware_output':False}
                        self.navigator.latest=self.navigation
                display=self.navigator.draw(image) if camera==self.s['camera'] else image.copy()
                for track in tracks:
                    x1,y1,x2,y2=track['bbox_xyxy'];color=(0,160,255) if track['kind']=='unresolved_blue' else (255,160,40)
                    if not track['visible']:color=(150,150,150)
                    cv2.rectangle(display,(x1,y1),(x2,y2),color,2)
                    cv2.putText(display,track['id']+(' OK' if track['confirmed'] else ' ?'),(x1,max(15,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
                for points in (context['left_inner'],context['right_inner']):
                    if points:cv2.polylines(display,[np.asarray(points,np.int32)],False,(0,230,100) if context['valid'] else (0,180,255),2)
                cv2.putText(display,camera.upper()+' BLUE CONES / LIVE',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.4,(240,240,240),1)
                self.rows[camera]=row;self.views[camera]=display;self.ids[camera]=fid;changed=True
            except (KeyError,ValueError,cv2.error):
                self.rows.pop(camera,None);self.views.pop(camera,None)
        if not self.rows:raise ValueError('两路摄像头均无新鲜画面')
        if not changed:return None
        self.sequence+=1
        panes=[]
        for camera in ('primary','secondary'):
            if camera in self.views:panes.append(self.views[camera])
            else:
                blank=np.zeros((360,480,3),np.uint8)
                cv2.putText(blank,camera.upper()+' NO FRAME',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(170,170,170),1);panes.append(blank)
        ok,jpg=cv2.imencode('.jpg',np.concatenate(panes,axis=1))
        self.preview=jpg.tobytes() if ok else None
        return {'sequence':self.sequence,'cameras':copy.deepcopy(self.rows),
                'captured_s':min(row['captured_s'] for row in self.rows.values()),
                'physical_cones_passed':len(self.navigation['passed_ids']) if self.navigation and 'passed_ids' in self.navigation else None,
                'navigation':{k:v for k,v in (self.navigation or {}).items() if k not in ('path','sample')},
                'hardware_output':False,'metric_readiness':readiness(self.s)}


class ConeObserverDrive:
    def __init__(self,drive,states,config,root,clock=time.monotonic,*,actuator_factory=None,short_actuator_factory=None):
        self.drive,self.states,self.root,self.clock=drive,states,Path(root),clock
        self.config=Path(config);self.local_config=self.config.with_name(self.config.stem+'.local.json')
        self.settings=load_settings(self.local_config if self.local_config.exists() else self.config)
        self.settings['driving']=validate_driving(self.settings)
        self.lock=threading.RLock();self.stop=threading.Event()
        self.thread=None;self.observing=False;self.closed=False
        self.latest=self.preview=None;self.reason='已录入车宽、车长、赛道及锥桶尺寸；可开始实际画面识别'
        self.updated=None
        self.run=ConeRun(actuator_factory,self.root/'run');self.reset_navigation=False
        self.short=ConeShortRun(self.root/'run',factory=short_actuator_factory,clock=self.clock,on_event=self._short_evidence)

    def status(self):
        with self.lock:
            base=self.drive.status()
            run=self.run.status(self.settings,base,self.clock())
            short=self.short.status(base,self.run.active)
            if self.short.active:run['can_prepare']=False
            if self.run.active or self.short.active:
                for key in ('traffic_trial','crosswalk_trial'):
                    base[key]={**base.get(key,{}),'can_prepare':False,'can_start':False,'can_request_start':False}
                base['pending_hardware_start']=not (run['hardware_output'] or short['hardware_output'])
                base['hardware_output']=run['hardware_output'] or short['hardware_output']
            return {**base,'cone_status_s':self.clock(),'cone_trial':{'program':'blue-cone-avoidance-v2','observing':self.observing,
                'logic_revision':'slalom-continuity-20261012',
                'hardware_output':run['hardware_output'] or short['hardware_output'],'can_observe':not self.observing and not self.closed,
                'can_drive':execution_readiness(self.settings)['ready'],'avoidance':run,'short_test':short,
                'reason':self.run.reason if self.run.active else self.reason,'settings':copy.deepcopy(self.settings),'readiness':execution_readiness(self.settings),
                'latest':self.latest,'frame_age_s':None if self.updated is None else self.clock()-self.updated,
                'limitations':['真实绕行仍需完成现场标定及试验'],
                'control_available':self.run.active or self.short.active}}

    def request(self,action,payload):
        if not isinstance(payload,dict):raise ValueError('JSON object required')
        with self.lock:
            if self.closed and action not in ('stop','emergency','cone_short_stop','cone_stop','cone_cancel'):raise ValueError('服务已关闭')
            if not action.startswith('cone_'):
                if self.run.active or self.short.active:
                    if action=='status':return self.status()
                    if action in ('stop','emergency','finish_test','traffic_cancel','crosswalk_cancel'):
                        if self.run.active:self.run.cancel('人工停止或页面离开，已结束避障')
                        self.short.cancel('人工停止或页面离开，已结束短测');return self.status()
                    raise ValueError('避障占用控制权，停止后再使用遥控或其他任务')
                self.drive.request(action,payload);return self.status()
            if action=='cone_short_prepare':
                self.short.prepare(self.drive.status(),payload,other_active=self.run.active)
                self._start_observer();return self.status()
            if action in ('cone_short_start','cone_short_keepalive'):
                self.short.command(action,payload);return self.status()
            if action=='cone_short_stop':
                self.short.cancel('用户结束固定档短测，输出已回中位');return self.status()
            if action in ('cone_start','cone_keepalive'):
                if self.short.active:raise ValueError('固定档短测占用控制权')
                self.run.command(action,payload,self.clock());return self.status()
            if action=='cone_prepare':
                if self.short.active:raise ValueError('先结束固定档短测，再准备自动避障')
                self.run.prepare(self.settings,self.drive.status(),payload,self.clock())
                self.reset_navigation=True
                self._start_observer();return self.status()
            if action=='cone_cancel':
                self.run.cancel('用户停止避障');return self.status()
            if action=='cone_calibration':
                if self.observing or self.run.active or self.short.active:raise ValueError('先停止识别、短测和避障，再保存标定')
                self._save(import_ground_calibration(self.settings,payload.get('calibration')))
                return self.status()
            if action=='cone_settings':
                if self.observing or self.run.active or self.short.active:raise ValueError('先停止识别、短测和避障，再保存参数')
                candidate=copy.deepcopy(self.settings)
                if 'scene_surface' in payload:
                    if payload['scene_surface'] not in ('red_running_track','indoor_white_line_track'):
                        raise ValueError('请选择红色跑道或室内白线赛道')
                    candidate.setdefault('scene',{})['surface']=payload['scene_surface']
                for key in ('blue_hsv_lower','blue_hsv_upper','confirm_frames'):
                    if key in payload:candidate[key]=payload[key]
                if 'red_ground_percent' in payload:
                    percent=payload['red_ground_percent']
                    if not numeric(percent) or not 1<=percent<=100:raise ValueError('红色地面辅助比例应为 1–100%')
                    candidate.setdefault('scene',{})['red_ground_min_fraction']=percent/100
                if 'geometry_cm' in payload:
                    geometry=payload['geometry_cm']
                    keys=('lane_width_m','vehicle_width_m','vehicle_length_m','rear_axle_to_front_m',
                          'rear_overhang_m','cone_height_m','cone_base_width_m','clearance_m')
                    if not isinstance(geometry,dict) or any(key+'_m' not in keys for key in geometry):
                        raise ValueError('未知的尺寸字段')
                    for key,value in geometry.items():
                        if value is not None and (not numeric(value) or not 0<value<=2000):
                            raise ValueError('尺寸请填写 0 到 2000 cm 之间的正数；待测项目留空')
                        candidate['geometry'][key+'_m']=None if value is None else value/100
                    g=candidate['geometry']
                    lengths=[g.get(k) for k in ('vehicle_length_m','rear_axle_to_front_m','rear_overhang_m')]
                    if all(numeric(v) for v in lengths) and abs(lengths[1]+lengths[2]-lengths[0])>.02:
                        raise ValueError('后轴到车头距离 + 后悬长度应等于车长（允许 2 cm 测量误差）')
                    g['measured']=all(numeric(g.get(k)) and g[k]>0 for k in keys)
                if 'driving' in payload:
                    values=payload['driving']
                    if not isinstance(values,dict):raise ValueError('driving object required')
                    allowed=set(driving_settings(candidate))-{'brake_pulse_s','brake_release_s','actuator_lease_s','fixed_camera_confirmed'}
                    if set(values)-allowed:raise ValueError('不允许的避障控制参数')
                    candidate['driving']={**driving_settings(candidate),**values}
                    d=validate_driving(candidate);m=candidate['motion']
                    radii=[d['steering_radius_1570_cm'],d['steering_radius_1650_cm']]
                    m['minimum_turn_radius_m']=max(abs(r) for r in radii)/100 if all(r is not None for r in radii) else None
                    m['pose_feedback_verified']=d['odometry_verified']
                    m['verified']=d['steering_verified'] and numeric(m.get('braking_distance_m'))
                if 'motion_cm' in payload:
                    values=payload['motion_cm']
                    if not isinstance(values,dict) or set(values)-{'braking_distance','maximum_tracking_error'}:raise ValueError('未知运动标定字段')
                    for k,v in values.items():
                        if v is not None and (not numeric(v) or not 0<v<=200):raise ValueError('运动标定以 cm 填正数或留空')
                        candidate['motion'][k+'_m']=None if v is None else v/100
                    candidate['motion']['verified']=driving_settings(candidate)['steering_verified'] and numeric(candidate['motion'].get('braking_distance_m'))
                from .cone_planning import validate_settings
                validate_settings(candidate)
                self._save(candidate)
                return self.status()
            if action=='cone_stop':
                if self.run.active:self.run.cancel('用户停止识别与避障')
                self.short.cancel('用户停止识别与短测')
                self.stop.set();self.reason='锥桶识别已停止';return self.status()
            if action!='cone_observe':raise ValueError('未知锥桶动作')
            if self.closed:raise ValueError('服务已关闭')
            self._start_observer()
            return self.status()

    def _short_evidence(self,folder,output):
        # These cached frames are timestamped evidence, not an exact physical
        # brake-position measurement. No camera owner or gimbal is changed.
        prefix=f"round-{output['round']:03d}-{output['phase']}-{output.get('brake_count',0)}"
        records={}
        for camera,state in self.states.items():
            with state.condition:jpg,fid,ms=state.raw_jpeg,state.raw_frame_id,state.raw_received_ms
            if jpg:
                (folder/(prefix+'-'+camera+'.jpg')).write_bytes(jpg)
                records[camera]={'frame_id':fid,'captured_s':ms/1000 if numeric(ms) else None}
        (folder/(prefix+'.json')).write_text(json.dumps({'output':output,'frames':records,
            'frame_is_exact_brake_time':False,'physical_stop_verified':False},ensure_ascii=False,indent=2),encoding='utf-8')

    def _save(self,candidate):
        candidate['driving']=validate_driving(candidate)
        self.local_config.parent.mkdir(parents=True,exist_ok=True)
        tmp=self.local_config.with_suffix('.tmp')
        tmp.write_text(json.dumps(candidate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        tmp.replace(self.local_config);self.settings=candidate
        self.reason='参数已保存；准备前会检查地面、转向与位移标定'

    def _start_observer(self):
        if self.closed:raise ValueError('服务已关闭')
        if not self.observing:
            self.stop=threading.Event();self.observing=True;self.reason='正在读取实际摄像头'
            self.thread=threading.Thread(target=self._work,name='cone-observer',daemon=True);self.thread.start()

    def _work(self):
        try:
            reader=ConeCamera(self.states,self.settings,self.clock)
            while not self.stop.is_set():
                with self.lock:
                    if self.reset_navigation:
                        reader.navigator=ConeNavigator(self.settings);reader.navigation=None;self.reset_navigation=False
                try:sample=reader.read()
                except (ValueError,cv2.error) as exc:
                    with self.lock:
                        self.latest=self.preview=self.updated=None;self.reason=str(exc)+'；正在等待恢复'
                        if self.run.active:self.run.cancel('相机画面中断，已结束避障',phase='fault')
                        self.short.cancel('相机画面中断，已结束短测',phase='fault')
                    self.stop.wait(.3);continue
                if sample is not None:
                    with self.lock:
                        self.latest,self.preview,self.updated=sample,reader.preview,sample['captured_s']
                        self.reason='实际双摄识别中；当前仅观察，不驱动车辆'
                # Actuator startup can take time; do not block page lease /
                # cancel requests behind the camera status lock.
                self.run.tick(reader.navigation,self.clock())
                self.short.tick(self.updated is not None and 0<=self.clock()-self.updated<=self.settings['max_frame_age_s'])
                self.stop.wait(.02 if self.run.active or self.short.active else .08)
        except Exception as exc:
            with self.lock:self.reason=str(exc)
        finally:
            with self.lock:
                if self.run.active:self.run.cancel('识别线程停止，已结束避障',phase='fault')
                self.short.cancel('识别线程停止，已结束短测',phase='fault')
                self.observing=False
                if self.stop.is_set():self.reason='锥桶识别已停止'

    def close(self):
        self.closed=True;self.stop.set()
        if self.thread:self.thread.join(timeout=2)
        self.drive.close()
