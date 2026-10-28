"""New Rain F/B parking: slow approach, bounded brake, motion feedback, hold.

The 65/50/25 cm camera markers are preserved. They are NOT a verified full-field
metric camera model. New PWM values are provisional tuning values, not measured
speed/braking calibration. No hardware is opened by this module.
"""

from .steering import from_offset
from .brake_timing import CROSSWALK_BRAKE_PULSE_S, MAX_BRAKE_PULSE_S
import json
import copy
import math
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from .crosswalk_calibration import (load_profile, validate_profile, reference_y,
                                   estimate_distance, IMAGE_FEATURE_NEAR)


def numeric(v):
    return type(v) in (int, float) and math.isfinite(v)


def load_settings(path):
    s=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if s.get('schema_version')!=1 or s.get('required_esc_mode')!='F/B':
        raise ValueError('this program is specifically for the non-reversing F/B mode')
    if s.get('camera')!='secondary' or s.get('image_size')!=[480,360]:
        raise ValueError('reuse the measured secondary 480x360 view')
    refs=s.get('references',[])
    if len(refs)!=3 or [r.get('distance_m') for r in refs]!=[.65,.5,.25]:
        raise ValueError('the existing 65/50/25 cm reference points must be preserved')
    ys=[r.get('far_edge_y_normalized') for r in refs]
    if not all(numeric(y) and .4<y<.95 for y in ys) or not ys[0]<ys[1]<ys[2]:
        raise ValueError('invalid reference ordering')
    p=s['pwm']
    if (p.get('neutral_us')!=1500 or p.get('steering_center_us')!=1610
            or not all(type(p.get(k)) is int for k in ['search_us','creep_us','brake_us'])
            or not 1500<p['creep_us']<=p['search_us']<=1575 or not 1300<=p['brake_us']<1500):
        raise ValueError('PWM configuration outside this program envelope')
    for key in ['brake_pulse_s','brake_release_s','stop_confirm_s','hold_s','max_frame_age_s','max_approach_s','brake_timeout_s','actuator_lease_s','arming_s','no_progress_s']:
        if not numeric(s.get(key)) or s[key]<=0:raise ValueError('invalid '+key)
    if (s['brake_pulse_s']>MAX_BRAKE_PULSE_S or s['max_frame_age_s']>.25 or s['hold_s']>30
            or s['actuator_lease_s']>.25 or not 1<=s['arming_s']<=12 or s['no_progress_s']>5):
        raise ValueError('brake pulse/feedback/hold limits exceeded')
    if type(s.get('max_brake_pulses')) is not int or not 1<=s['max_brake_pulses']<=3:
        raise ValueError('max_brake_pulses must be 1..3')
    from .autonomy_audio import tts_frames
    tts_frames(s['speech_text'],9)
    brake_trigger_reference(s)
    return s


def brake_trigger_reference(settings):
    """Invert the existing in-range linear model; never relabel interpolation as measured."""
    refs=settings['references']
    distance=settings.get('brake_trigger_distance_m',.25)
    if not numeric(distance) or not refs[-1]['distance_m']<=distance<=refs[0]['distance_m']:
        raise ValueError('brake trigger must remain inside the measured 25..65 cm range')
    ascending=list(reversed(refs))
    y=float(np.interp(distance,[r['distance_m'] for r in ascending],
                      [r['far_edge_y_normalized'] for r in ascending]))
    measured=any(distance==r['distance_m'] for r in refs)
    return {'distance_m':distance,'far_edge_y_normalized':y,'measured':measured,
            'source':'existing_measured_point' if measured else 'linear_interpolation_of_existing_points'}


def crosswalk_distance_limits(settings):
    profile=settings.get('crosswalk_calibration')
    if profile is None:return [settings['references'][-1]['distance_m']*100,settings['references'][0]['distance_m']*100]
    return [v*100 for v in validate_profile(profile,require_ready=True)['overlap_range_m']]


def crosswalk_trigger_reference(settings):
    """Explicit crosswalk lookup; shared parking retains its legacy references."""
    profile=settings.get('crosswalk_calibration')
    if profile is None:return brake_trigger_reference(settings)
    profile=validate_profile(profile,require_ready=True)
    if profile['image_feature']!=IMAGE_FEATURE_NEAR:raise ValueError('当前双摄程序需要近边标定')
    distance=settings.get('crosswalk_distance_trigger_m',profile['overlap_range_m'][0])
    if not numeric(distance) or not profile['overlap_range_m'][0]<=distance<=profile['overlap_range_m'][1]:
        raise ValueError('刹车触发距离超出本次两路共同实测范围')
    references={cam:reference_y(profile,cam,distance) for cam in ('primary','secondary')}
    return {'distance_m':distance,'camera':'dual','calibration_id':profile['profile_id'],
            'image_feature':IMAGE_FEATURE_NEAR,'camera_references':references,
            'measured':all(r['measured'] for r in references.values()),
            'source':'current_camera_near_edge_calibration','physical_stop_verified':False}


def crosswalk_calibrated_settings(settings,root):
    """Freeze the deployed crosswalk table without replacing parking's old table."""
    settings=copy.deepcopy(settings);configs=Path(root)/'vision/configs'
    profile=load_profile(configs/'crosswalk-distance-calibration.json')
    if profile is None:
        if settings.get('crosswalk_calibration') is not None:
            raise ValueError('已加载的斑马线标定文件缺失，请恢复后重新准备')
        return settings
    profile=validate_profile(profile,require_ready=True)
    settings['crosswalk_calibration']=profile
    # Keep the parking/legacy profile unchanged; tune current crosswalk stops.
    settings['brake_pulse_s']=CROSSWALK_BRAKE_PULSE_S
    # Distance-camera latency is separate from the actuator/control deadline.
    # Shared parking and legacy braking keep their existing 250ms settings.
    settings['crosswalk_max_frame_age_s']=.5
    saved_path=configs/'crosswalk-distance.local.json'
    distance=profile['overlap_range_m'][0]
    if saved_path.exists():
        saved=json.loads(saved_path.read_text(encoding='utf-8-sig'))
        if (not isinstance(saved,dict) or type(saved.get('schema_version')) is not int
                or saved['schema_version']!=1 or saved.get('calibration_id')!=profile['profile_id']):
            raise ValueError('已保存刹车触发点与当前斑马线标定版本不一致')
        distance=saved['brake_trigger_distance_m']
    settings['crosswalk_distance_trigger_m']=distance
    crosswalk_trigger_reference(settings)
    return settings


class GroundMotionEstimate:
    """Image-motion evidence, explicitly an estimate, never encoder verification.

Tracks textured ground above the bonnet. Flat/occluded frames stay unknown;
fresh frame IDs alone are not evidence that the vehicle is stationary.
"""
    def __init__(self):
        self.previous=None

    def update(self,image,frame_id,captured_s):
        grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        h,w=grey.shape
        mask=np.zeros_like(grey)
        mask[int(.43*h):int(.78*h),int(.10*w):int(.90*w)]=255
        points=cv2.goodFeaturesToTrack(grey,100,.01,7,mask=mask)
        old=self.previous
        self.previous=(grey,points,frame_id,captured_s)
        unknown={'valid':False,'moving':None,'source':'ground_optical_estimate','physical_stop_verified':False}
        if old is None:return unknown
        before,old_points,old_id,t=old
        dt=captured_s-t
        if before.shape!=grey.shape or old_id>=frame_id or not .015<=dt<=.25 or old_points is None or len(old_points)<15:
            return unknown
        after,ok,error=cv2.calcOpticalFlowPyrLK(before,grey,old_points,None,winSize=(21,21),maxLevel=2)
        if after is None or ok is None or error is None:return unknown
        back,back_ok,_=cv2.calcOpticalFlowPyrLK(grey,before,after,None,winSize=(21,21),maxLevel=2)
        if back is None or back_ok is None:return unknown
        valid=(ok[:,0]>0)&(back_ok[:,0]>0)&(error[:,0]<20)&(np.linalg.norm(back[:,0]-old_points[:,0],axis=1)<1)
        if int(valid.sum())<15:return unknown
        displacement=np.linalg.norm(after[valid,0]-old_points[valid,0],axis=1)
        # Normalize displacement to a 100-ms interval. Require most points,
        # not one frozen patch, to agree on a small motion estimate.
        value=float(np.percentile(displacement,75))*.1/dt
        return {'valid':True,'moving':value>.4,'pixel_motion_per_100ms':value,'tracked_points':int(valid.sum()),
                'source':'ground_optical_estimate','physical_stop_verified':False}


class BrakeParkingController:
    def __init__(self,settings,path_mode='straight',*,first_detection_stop=False,
                 image_brake_y=None,image_creep_y=None,crosswalk=False):
        if path_mode not in ('straight','lane'):raise ValueError('invalid path mode')
        self.s,self.path_mode=settings,path_mode
        self.brake_reference=brake_trigger_reference(settings)
        self.crosswalk_profile=None
        self.source_camera=None
        self.distance_disagreement=None
        if crosswalk and settings.get('crosswalk_calibration') is not None:
            if image_brake_y is not None or image_creep_y is not None:raise ValueError('停车图像参考不能混入双摄斑马线标定')
            self.crosswalk_profile=validate_profile(settings['crosswalk_calibration'],require_ready=True)
            self.brake_reference=crosswalk_trigger_reference(settings)
        self.frame_age_limit_s=(settings.get('crosswalk_max_frame_age_s',settings['max_frame_age_s'])
                                if self.crosswalk_profile is not None else settings['max_frame_age_s'])
        if not numeric(self.frame_age_limit_s) or not 0<self.frame_age_limit_s<=.5:
            raise ValueError('停车画面期限无效')
        self.image_reference=image_brake_y is not None
        if self.image_reference:
            if not numeric(image_brake_y) or not .3<=image_brake_y<=.95:
                raise ValueError('invalid image-only brake reference')
            self.brake_reference={'distance_m':None,'far_edge_y_normalized':image_brake_y,
                                  'measured':False,'source':'parking_image_reference'}
        if image_creep_y is not None and (not numeric(image_creep_y)
                or not .3<=image_creep_y<self.brake_reference['far_edge_y_normalized']):
            raise ValueError('image creep reference must precede braking')
        self.image_creep_y=image_creep_y
        self.phase='search'
        self.started=self.last_t=self.last_frame=self.last_capture=None
        self.seen_at=self.brake_started=self.pulse_until=self.release_until=None
        self.stop_since=self.hold_since=None
        self.no_progress_since=None
        self.brake_count=0
        self.spoken=False
        self.reason='等待新画面'
        self.metric_history=deque(maxlen=6)
        self.distance=self.speed=None
        # Opt in only at crosswalk entry points; parking uses the same core.
        self.first_detection_stop=bool(first_detection_stop)
        self.first_detection_handled=False
        self.preliminary_stop=False
        self.preliminary_started=None
        self.first_detection_pause_s=.5

    def _hold_seconds(self):
        return self.first_detection_pause_s if self.preliminary_stop else self.s['hold_s']

    def output(self,action='neutral',events=None):
        p=self.s['pwm']
        pulse={'neutral':p['neutral_us'],'search':p['search_us'],'creep':p['creep_us'],'brake':p['brake_us']}[action]
        return {'schema_version':1,'phase':self.phase,'action':action,'esc_us':pulse,
                'steering_us':p['steering_center_us'],'reason':self.reason,
                'events':events or [],'brake_pulses':self.brake_count,
                'brake_pulse_s':self.s['brake_pulse_s'],
                'brake_trigger':self.brake_reference,
                'calibration_id':self.crosswalk_profile['profile_id'] if self.crosswalk_profile else None,
                'source_camera':self.source_camera,
                'distance_disagreement':copy.deepcopy(self.distance_disagreement),
                'distance_estimate_m':self.distance,'closing_speed_estimate_mps':self.speed,
                'hold_remaining_s':None if self.hold_since is None else max(0,self._hold_seconds()-(self.last_t-self.hold_since)),
                'stop_stage':'first_detection' if self.preliminary_stop else 'original_operation',
                'first_detection_handled':self.first_detection_handled,
                'first_detection_pause_s':self.first_detection_pause_s if self.first_detection_stop else None,
                'physical_stop_verified':False,'parking_zone_verified':False,'tuning_verified':False}

    def fault(self,reason):
        self.phase,self.reason='fault',reason
        return self.output()

    def _estimate_distance(self,y,t):
        refs=self.s['references'];ys=[r['far_edge_y_normalized'] for r in refs]
        self.distance=self.speed=None
        if numeric(y) and ys[0]<=y<=ys[-1]:
            self.distance=float(np.interp(y,ys,[r['distance_m'] for r in refs]))
            self.metric_history.append((t,self.distance))
            recent=[v for v in self.metric_history if t-v[0]<=.6]
            if len(recent)>=3 and recent[-1][0]-recent[0][0]>=.15:
                ts=np.array([v[0]-recent[0][0] for v in recent]);ds=np.array([v[1] for v in recent])
                self.speed=float(-np.polyfit(ts,ds,1)[0])

    def _calibrated_observation(self,sample,t):
        profile=self.crosswalk_profile;camera=sample.get('source_camera')
        if sample.get('calibration_id')!=profile['profile_id'] or camera not in ('primary','secondary'):
            raise ValueError('斑马线画面与当前双摄标定版本或来源不一致')
        if sample.get('image_feature',IMAGE_FEATURE_NEAR)!=IMAGE_FEATURE_NEAR:
            raise ValueError('斑马线输入必须使用当前近边坐标')
        if camera!=self.source_camera or sample.get('source_changed') is True:
            self.metric_history.clear();self.speed=None
        self.source_camera=camera
        self.distance_disagreement=copy.deepcopy(sample.get('distance_disagreement'))
        y=sample.get('near_edge_y_normalized') if sample.get('candidate') is True else None
        clipped=sample.get('near_limit_reached') is True and sample.get('candidate') is True
        if sample.get('candidate') is True and not clipped and (not numeric(y) or not 0<y<1):
            raise ValueError('斑马线近边坐标失效')
        self.distance=self.speed=None
        refs=profile['cameras'][camera]['references'];ys=[r['near_edge_y_normalized'] for r in refs]
        if not clipped and numeric(y) and min(ys)<=y<=max(ys):
            self.distance=estimate_distance(profile,camera,y)['distance_m']
            self.metric_history.append((t,self.distance))
            recent=[v for v in self.metric_history if t-v[0]<=.6]
            if len(recent)>=3 and recent[-1][0]-recent[0][0]>=.15:
                ts=np.array([v[0]-recent[0][0] for v in recent]);ds=np.array([v[1] for v in recent])
                self.speed=float(-np.polyfit(ts,ds,1)[0])
        brake_y=self.brake_reference['camera_references'][camera]['near_edge_y_normalized']
        creep_distance=max(profile['overlap_range_m'][0],min(.5,profile['overlap_range_m'][1]))
        creep_y=reference_y(profile,camera,creep_distance)['near_edge_y_normalized']
        return y,(clipped or numeric(y) and y>=brake_y),(clipped or numeric(y) and y>=creep_y)

    def step(self,sample,now):
        if self.phase in ('done','fault'):return self.output()
        if not numeric(now) or self.last_t is not None and not 0<now-self.last_t<=.25:
            return self.fault('控制间隔失效')
        if self.started is None:self.started=now
        self.last_t=now
        t,frame_id=sample.get('captured_s'),sample.get('frame_id')
        if (sample.get('fresh') is not True or not numeric(t) or not 0<=now-t<=self.frame_age_limit_s
                or type(frame_id) is not int or self.last_frame is not None and frame_id<=self.last_frame
                or self.last_capture is not None and t<=self.last_capture):
            return self.fault('画面过期或重复')
        if sample.get('image_size')!=self.s['image_size']:return self.fault('画面尺寸与既有标定不一致')
        if sample.get('speed_feedback_valid') is True:
            speed,age=sample.get('speed_mps'),sample.get('speed_age_s')
            if not numeric(speed) or not numeric(age) or not 0<=age<=.25:
                return self.fault('指定的速度反馈失效')
        self.last_frame,self.last_capture=frame_id,t
        if self.crosswalk_profile:
            try:y,brake_reached,creep_reached=self._calibrated_observation(sample,t)
            except (ValueError,KeyError,TypeError) as exc:return self.fault(str(exc))
        else:
            y=sample.get('far_edge_y_normalized') if sample.get('candidate') is True else None
            brake_reached=numeric(y) and y>=self.brake_reference['far_edge_y_normalized']
            creep_y=self.image_creep_y if self.image_creep_y is not None else self.s['references'][1]['far_edge_y_normalized']
            creep_reached=numeric(y) and y>=creep_y
        if numeric(y) or self.crosswalk_profile and brake_reached:self.seen_at=now
        if not self.image_reference and not self.crosswalk_profile:self._estimate_distance(y,t)
        # If coasting already reaches the calibrated stop reference, retain the
        # stop and perform the original hold rather than driving over the line.
        if self.preliminary_stop and brake_reached:
            self.preliminary_stop=False;self.hold_since=None
            self.reason='首次制动已到原停车参考，直接完成原有停车操作'
        if self.phase in ('search','approach','creep'):
            if now-self.started>self.s['max_approach_s']:return self.fault('接近时限已到')
            if self.seen_at is not None and now-self.seen_at>.25:return self.fault('已见斑马线持续丢失')
            first_seen=(self.first_detection_stop and not self.first_detection_handled
                        and sample.get('candidate') is True)
            if first_seen:self.first_detection_handled=True
            if brake_reached:
                self.phase='brake_pulse'
                self.reason=('新标定近边到达刹车触发点，发出有限 F/B 制动请求' if self.crosswalk_profile else
                    '实际远线到达停车图像停止参考，发出有限 F/B 制动请求' if self.image_reference else
                    f"到达约 {self.brake_reference['distance_m']*100:g} cm 刹车触发参考，发出 F/B 制动请求")
                self.brake_started=now;self.pulse_until=now+self.s['brake_pulse_s'];self.brake_count=1
                return self.output('brake')
            if first_seen:
                self.preliminary_stop=True;self.preliminary_started=now
                self.phase='brake_pulse';self.reason='首次识别斑马线，立即制动；停稳后短停 0.5 秒再接近'
                self.brake_started=now;self.pulse_until=now+self.s['brake_pulse_s'];self.brake_count=1
                return self.output('brake',events=[{'event':'crosswalk_first_detection_stop'}])
            motion=sample.get('motion') or {}
            if motion.get('valid') is True and motion.get('moving') is False:
                if self.no_progress_since is None:self.no_progress_since=now
                if now-self.no_progress_since>=self.s['no_progress_s']:
                    return self.fault('持续前进指令但未观察到移动；检查电调解锁、供电与起步输出')
            else:
                self.no_progress_since=None
            if self.phase=='creep' or creep_reached:
                self.phase='creep'
                self.reason=('当前双摄近边已进入减速区域' if self.crosswalk_profile else
                             '原 50 cm 区域内降低输出接近' if self.s['pwm']['creep_us']<self.s['pwm']['search_us']
                             else '已进入原 50 cm 区域，保持已知起步档接近')
                result=self.output('creep')
            else:
                self.phase='approach' if numeric(y) else 'search'
                self.reason='低输出前进搜索斑马线'
                result=self.output('search')
            if self.path_mode=='lane':
                visible=sample.get('lane_valid') is True and numeric(sample.get('lane_offset'))
                result['steering_us']=from_offset(sample['lane_offset']) if visible else self.s['pwm']['steering_center_us']
                result['line_search']=not visible
                if not visible:result['reason']+='；白线暂失，保持低速直行搜索'
            return result
        if now-self.brake_started>self.s['brake_timeout_s'] and self.phase!='hold':
            return self.fault('制动后未取得连续停稳反馈')
        if self.phase=='brake_pulse':
            if now<self.pulse_until:return self.output('brake')
            self.phase='settle';self.release_until=now+self.s['brake_release_s']
            self.reason='制动脉冲结束，观察运动反馈'
        motion=sample.get('motion',{})
        reliable=motion.get('valid') is True
        moving=motion.get('moving') if reliable else None
        if sample.get('speed_feedback_valid') is True:
            moving=abs(sample['speed_mps'])>.01
        if moving is False:
            if self.stop_since is None:self.stop_since=now
            if now-self.stop_since>=self.s['stop_confirm_s']:
                if self.hold_since is None:self.hold_since=now
                self.phase='hold';self.reason=f"停稳候选持续成立，保持停止 {self._hold_seconds():g} 秒"
                events=[]
                if not self.preliminary_stop and not self.spoken:
                    self.spoken=True;events=[{'event':'speak','text':self.s['speech_text']}]
                if now-self.hold_since>=self._hold_seconds() and self.preliminary_stop:
                    self.started+=now-self.preliminary_started
                    self.preliminary_stop=False
                    self.phase='search';self.reason='首次短停完成，继续原有斑马线接近与定点停车'
                    self.brake_started=self.pulse_until=self.release_until=None
                    self.stop_since=self.hold_since=self.no_progress_since=None
                    self.seen_at=now if numeric(y) else None
                    self.brake_count=0;self.metric_history.clear()
                    # Resume on the next fresh frame; do not reuse this frame
                    # to issue a forward pulse while finishing the stop.
                    return self.output(events=[{'event':'crosswalk_first_detection_resume'}])
                if now-self.hold_since>=self.s['hold_s']:
                    self.phase,self.reason='done','停车流程完成，保持停止'
                return self.output(events=events)
        else:
            self.stop_since=self.hold_since=None
            if self.phase=='hold':
                self.phase='settle';self.brake_started=now
            if moving is True and now>=(self.release_until or 0):
                if self.brake_count>=self.s['max_brake_pulses']:
                    self.reason='制动脉冲已用完，保持中位观察停稳；继续移动则超时退出'
                    return self.output()
                self.brake_count+=1;self.phase='brake_pulse'
                self.pulse_until=now+self.s['brake_pulse_s'];self.reason='仍有运动反馈，再请求一个有限制动脉冲'
                return self.output('brake')
            self.reason='停稳反馈未知，保持零油门' if moving is None else '等待制动释放间隔'
        return self.output()
