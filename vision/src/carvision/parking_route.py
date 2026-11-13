"""Parking approach, entrance crossing and stopping from current evidence.

Image targets are guidance, not vehicle footprints. Ground positions are
range-checked measurements or bounded local-motion estimates, never marked
as field-verified. This module has no actuator, network or file access.
"""
import math
import numpy as np
from .parking_ground_view import ParkingGroundView
from .parking_recovery import ParkingRecovery
from .parking_stop_evidence import ParkingStopEvidence
from .parking_bay_pose import ParkingBayPose
from .parking_progress import ParkingProgress
from .parking_approach import ParkingApproach

# Covers the last calibrated entrance at about 0.84 m, rear clearance and
# the reported approximately 0.25 m brake travel. These remain trial budgets.
POSITION_MAX_TRAVEL_M = 1.5
POSITION_MAX_AGE_S = 6.
POSITION_MAX_ERROR_M = .08


class ParkingRoute:
    def __init__(self,ground,*,width,front,rear,bay_width,bay_length,brake_s=.35,acquire_timeout_s=5.):
        self.view=ParkingGroundView(ground)
        self.width,self.front,self.rear=width,front,rear
        self.bay_width,self.bay_length=bay_width,bay_length
        self.brake_s=brake_s
        self.anchor=None;self.frame=None;self.capture=None
        self.reference=None;self.position_problem=None
        self.position_acquired=False
        self.anchor_time=None;self.travel=0.;self.error=self.view.error;self.fit_variance=0.
        self.started=None;self.paused=None;self.pause_reason=None
        self.recovery=ParkingRecovery(1.2);self.plan_count=0;self.plan_since=None
        self.plan_revision=0;self.stopping=None;self.stop_since=None;self.stop_frames=0
        self.brake_until=0.
        self.report={};self.last_plan=None
        self.stop_evidence=ParkingStopEvidence();self.stop_motion={}
        self.bay_pose=ParkingBayPose(self.view,bay_width,bay_length)
        self.progress=ParkingProgress();self.last_position=None
        self.measurement_error=self.view.error
        self.bay_model_consistent=True
        self.initial=ParkingApproach(self.view,front,acquire_timeout_s)
        self.entrance_line=None

    def retarget(self):
        # Do not retain the former bay's entrance or bypass an existing brake.
        self.anchor=None;self.frame=self.capture=self.anchor_time=None
        self.reference=None;self.position_problem=None
        self.position_acquired=False
        self.travel=0.;self.error=self.view.error;self.fit_variance=0.
        self.plan_since=None;self.plan_count=0;self.plan_revision+=1
        self.stop_evidence=ParkingStopEvidence()
        self.progress=ParkingProgress();self.last_position=None
        self.measurement_error=self.view.error
        self.bay_model_consistent=True
        self.initial.reset();self.entrance_line=None

    def _entrance(self,q,near,flow,fid,t,landmarks=None):
        self.position_problem=None
        if self.anchor is None and self.reference is not None:
            old=self.reference
            if 0<t-old['capture']<=.5:
                self.anchor=old['anchor'];self.frame=old['frame'];self.capture=old['capture']
                self.anchor_time=old['anchor_time'];self.travel=old['travel']
                self.error=old['error'];self.fit_variance=old['fit_variance']
                self.measurement_error=old['measurement_error']
            else:self.reference=None;self.position_problem='position_recovery_expired'
        self.entrance_line=self.view.entrance_line(q) if q is not None and near else None
        measured=self.view.entrance(q) if self.entrance_line is not None else None
        if measured is None and self.entrance_line is not None and self.bay_model_consistent:
            measured=self.bay_pose.entrance(q,self.entrance_line)
            self.report['entrance_pose_failure']=self.bay_pose.reason
        self.report['entrance_line']=self.entrance_line
        if measured is not None and measured.get('centre_inferred'):
            self.report['entrance_centre_estimate']={k:v for k,v in measured.items()
                                                   if k not in ('normal','center_m')}
        bay=self.bay_pose.measure(landmarks,fid,t) if self.position_acquired and self.bay_model_consistent else None
        self.report['bay_pose_failure']=self.bay_pose.reason if landmarks is not None else 'no_current_bay_landmarks'
        # Current entrance remains the strongest observation. A rectangular
        # far/side fit is useful only for this already acquired bay, and must
        # agree whenever both observations overlap.
        if measured is not None and bay is not None:
            if (np.linalg.norm(np.asarray(measured['center_m'])-bay['center_m'])>.15
                    or np.dot(measured['normal'],bay['normal'])<.96):
                self.report['bay_pose_failure']='bay_entrance_disagrees';bay=None
                self.bay_model_consistent=False
        if not self.bay_model_consistent:self.report['bay_pose_failure']='bay_entrance_disagrees'
        if measured is None and bay is not None:
            # An old pose is only a plausibility bound, never current evidence.
            # With no fresh motion chain, allow bounded travel/yaw, not a
            # reappearance on an arbitrary other bay after a long interval.
            old=self.last_position
            dt=t-old['capture'] if old is not None else float('inf')
            displacement=(np.linalg.norm(np.asarray(bay['center_m'])-old['anchor']['center_m'])
                          if old is not None else float('inf'))
            alignment=(np.dot(bay['normal'],old['anchor']['normal']) if old is not None else -1.)
            if (not 0<=dt<=1.2 or displacement>.12+1.5*dt
                    or alignment<math.cos(min(.45,.10+1.5*dt))):
                self.report['bay_pose_failure']='bay_reacquisition_conflict'
            else:measured=bay
        propagated=False
        if self.anchor is not None and self.frame!=fid:
            candidates=[flow]+[v for v in flow.get('relocalizations',[]) if isinstance(v,dict)]
            matching=[v for v in candidates if v.get('valid') is True and v.get('frame_id')==fid
                      and v.get('captured_s')==t and v.get('previous_frame_id')==self.frame
                      and v.get('previous_captured_s')==self.capture and 0<t-self.capture<=.5]
            flow=matching[0] if matching else flow
            valid=(flow.get('valid') is True and flow.get('frame_id')==fid
                   and flow.get('captured_s')==t and flow.get('previous_frame_id')==self.frame
                   and flow.get('previous_captured_s')==self.capture and 0<t-self.capture<=.5)
            if valid:
                try:
                    affine=np.asarray(flow['transform'],float)
                    if affine.shape!=(2,3) or not np.isfinite(affine).all():raise ValueError()
                    rotation=affine[:,:2]
                    distance=float(flow['translation_m']);rms=float(flow['rms_m'])
                    if (not np.allclose(rotation.T@rotation,np.eye(2),atol=.01)
                            or np.linalg.det(rotation)<.99
                            or not math.isfinite(distance) or not math.isfinite(rms)
                            or distance<0 or rms<0):raise ValueError()
                    center=rotation@np.asarray(self.anchor['center_m'])+affine[:,2]
                    normal=rotation@np.asarray(self.anchor['normal'])
                    self.travel+=distance
                    # Residual noise accumulates as RSS; a separate drift
                    # allowance grows with travel. This is an unverified
                    # uncertainty estimate, not a measured error guarantee.
                    self.fit_variance+=rms*rms
                    self.error=self.measurement_error+math.sqrt(self.fit_variance)+.02*self.travel
                    self.anchor={**self.anchor,'center_m':center.tolist(),'normal':normal.tolist(),
                        'distance_m':float(normal@center),'source':'bounded_ground_motion_estimate'}
                    propagated=True
                except (ValueError,KeyError,TypeError):valid=False
            if not valid:
                self.anchor=None;self.position_problem='ground_motion_invalid'
        if measured is not None:
            # A new measurement must agree with a still-valid propagated line.
            if propagated and (np.linalg.norm(np.asarray(measured['center_m'])-self.anchor['center_m'])>.2
                               or np.dot(measured['normal'],self.anchor['normal'])<.96):
                self.anchor=None
                self.reference=None;self.position_problem='entrance_measurement_conflict'
            else:
                self.anchor=measured;self.anchor_time=t;self.travel=0.;self.fit_variance=0.
                self.measurement_error=measured.get('error_estimate_m',self.view.error)
                self.error=self.measurement_error
        if (self.anchor is not None and
                (t-self.anchor_time>POSITION_MAX_AGE_S or self.travel>POSITION_MAX_TRAVEL_M
                 or self.error>POSITION_MAX_ERROR_M)):
            self.anchor=None
            self.reference=None;self.position_problem='position_budget_exceeded'
        self.frame,self.capture=fid,t
        if self.anchor is not None:
            self.reference={'anchor':self.anchor.copy(),'frame':fid,'capture':t,'anchor_time':self.anchor_time,
                            'travel':self.travel,'error':self.error,'fit_variance':self.fit_variance,
                            'measurement_error':self.measurement_error}
            self.last_position=self.reference.copy()
            self.position_problem=None
            self.position_acquired=True
        elif near and q is not None and measured is None:
            self.position_problem='entrance_outside_calibration'
        elif self.position_problem is None:
            self.position_problem='entrance_outside_calibration' if near and q is not None else 'entrance_line_missing'
        return self.anchor

    def _position_reason(self):
        return {'ground_motion_invalid':'地面运动匹配失败，正在用当前远边、侧边重新定位',
                'position_recovery_expired':'短时定位参考已过期，需要当前车位边线重新定位',
                'entrance_measurement_conflict':'车位测量与连续位置不一致，需要重新定位',
                'position_budget_exceeded':'连续位置估计达到时长、行程或误差上限',
                'entrance_outside_calibration':'已看到入口线，但其位置不在标定有效范围内',
                'entrance_line_missing':('入口已不在视野，当前远边、侧边或运动定位不足'
                    if self.position_acquired else '当前入口尚未识别，需要先确认目标车位')}.get(self.position_problem,'车位位置需要重新确认')

    def _body(self,entry):
        center=np.asarray(entry['center_m']);normal=np.asarray(entry['normal'])
        tangent=np.array([normal[1],-normal[0]])
        corners=np.array([[-self.width/2,-self.rear],[self.width/2,-self.rear],
                          [self.width/2,self.front],[-self.width/2,self.front]])
        forward=(corners-center)@normal
        lateral=(corners-center)@tangent
        clearance=float(self.bay_width/2-max(abs(lateral)))-self.error-.02
        front_crossed=bool(forward.max()>self.error)
        rear_inside=bool(forward.min()>self.error+.02)
        far_clearance=float(self.bay_length-forward.max())-self.error-.02
        return {'entry_distance_m':float(entry['distance_m']),
                'front_entry_gap_m':float(-forward.max()),'rear_inside_estimate':rear_inside,
                'front_crossed_estimate':front_crossed,'side_clearance_estimate_m':clearance,
                'far_clearance_estimate_m':far_clearance,
                'lateral_error_m':float(-center@tangent),
                'heading_error_rad':float(math.atan2(normal[0],normal[1])),
                'pose_source':entry['source'],'position_error_estimate_m':self.error,
                'tracked_distance_m':self.travel,'physical_pose_verified':False}

    def _plan(self,q,near,geometry,white,entry,now,sample=None):
        if entry is None and self.position_acquired:
            return {'stage':self.progress.stage,'allowed':False,'reason':self._position_reason()}
        if entry is None:
            plan=self.initial.plan(q,near,geometry,white,self.entrance_line,now,sample or {})
            self.report['initial_position']=self.initial.report.copy()
            return plan
        current_far=geometry.get('far_edge_y_normalized') is not None
        if q is None and not current_far:
            return {'stage':'locate_entry','allowed':False,'reason':self._position_reason()}
        if q is not None and geometry.get('availability')!='clear':
            return {'stage':'locate_entry','allowed':False,'reason':'当前停车区空闲证据不足'}
        body=self._body(entry);self.report.update(body)
        gap=body['front_entry_gap_m'];clearance=body['side_clearance_estimate_m']
        heading=body['heading_error_rad'];lateral=body['lateral_error_m']
        stage=('inside_estimate' if body['rear_inside_estimate'] else
               'cross_entry' if body['front_crossed_estimate'] else
               'align_entry' if gap<.5 else 'approach')
        if self.progress.entered and gap>.15:
            return {'stage':self.progress.stage,'allowed':False,
                    'reason':'当前车位位置与已确认过线阶段冲突，暂停重新定位'}
        if gap<=.3 and (clearance<=0 or abs(heading)>.35):
            return {'stage':stage,'allowed':False,'reason':'入口前对齐余量不足，暂停重新规划'}
        if body['far_clearance_estimate_m']<.25 and not body['rear_inside_estimate']:
            return {'stage':stage,'allowed':False,'reason':'剩余入位与制动空间不足，暂停重新规划'}
        center=np.asarray(entry['center_m']);normal=np.asarray(entry['normal'])
        # Aim at an upstream centre point, then continue along the bay axis.
        # This keeps position and heading in a single geometric target.
        target=center-normal*(self.front+.35)
        if target[1]<.4:target=center+normal*.4
        bearing=math.atan2(float(target[0]),max(.25,float(target[1])))
        error=float(np.clip(bearing*1.2,-.42,.42))
        # Within the entry approach, a heading correction cannot consume the
        # last lateral margin. Inward correction precedes further alignment.
        margin_override=False
        if gap<.6 and clearance<.07 and abs(lateral)>.025:
            inward=-math.copysign(1.,lateral)
            if error*inward<.10:error=inward*.16;margin_override=True
        return {'stage':stage,'allowed':True,'error':error,'margin_override':margin_override,
                'target_m':target.tolist(),'reason':('入口离开视野，结合当前远边和侧边继续定位'
                    if entry['source']=='current_far_side_bay_pose' else '结合车位位置、方向和侧向余量接近'),
                'stop_candidate':body['rear_inside_estimate'] and abs(heading)<.15
                    and clearance>.02 and (body['entry_distance_m']<=-.16 or body['far_clearance_estimate_m']<=.35)}

    def step(self,sample,q,near,geometry,white,now,*,was_driving=False,landmarks=None):
        if self.started is None:self.started=now
        if self.initial.started is None:self.initial.started=now
        fid,t=sample['frame_id'],sample['captured_s']
        self.report={'physical_pose_verified':False,'image_corridor_is_vehicle_body':False,
                     'plan_revision':self.plan_revision}
        entry=self._entrance(q,near,sample.get('parking_ground_motion') or {},fid,t,landmarks)
        if entry is not None and entry['source']=='current_far_side_bay_pose':
            geometry={**geometry,'far_edge_y_normalized':float(np.mean(np.asarray(landmarks['far_edge_px'])[:,1]))/359.}
            self.report['bay_pose']={k:v for k,v in entry.items() if k not in ('center_m','normal')}
        self.report.update(position_failure=self.position_problem,
            retained_reference_frame=self.reference['frame'] if self.reference else None,
            ground_motion_valid=(sample.get('parking_ground_motion') or {}).get('valid') is True,
            ground_motion_failure=(sample.get('parking_ground_motion') or {}).get('reason'))
        plan=self._plan(q,near,geometry,white,entry,now,sample)
        if entry is not None and plan['stage'] in self.progress.ORDER:
            self.progress.observe(plan['stage'],t)
        else:self.progress.missing()
        self.report.update(confirmed_stage=self.progress.stage,
            front_crossing_confirmed=self.progress.entered,
            rear_entry_confirmed=self.progress.stage=='inside_estimate')
        self.report.update(stage=self.progress.stage if self.position_acquired else plan['stage'],
                           current_position_stage=plan['stage'],
                           current_entrance_measurement=bool(entry and entry['source'] in (
                               'current_calibrated_entrance','current_entrance_side_estimate')),
                           entrance_position_available=entry is not None,reason=plan['reason'],
                           target_m=plan.get('target_m'),margin_override=plan.get('margin_override',False))
        self.last_plan=plan
        self.stop_motion=self.stop_evidence.update(sample,enabled=(
            (self.paused is not None or self.stopping is not None) and now>=self.brake_until+.1))
        self.report['stop_evidence']=self.stop_motion
        if self.stopping is not None:return self._finish(sample,now)
        if plan.get('stop_candidate'):
            self.stopping=now;self.brake_until=now+(self.brake_s if was_driving else 0.)
            return self._finish(sample,now)
        if not plan['allowed'] or self.paused is not None:
            if self.paused is None:
                self.paused=now;self.pause_reason=plan['reason']
                self.brake_until=now+(self.brake_s if was_driving else 0.)
                self.recovery.boundary_timeout_s=(1.2 if self.position_acquired
                    else max(.1,self.initial.remaining(now)+self.brake_s+.1))
                self.recovery.start(now,self.brake_until+.1,was_driving)
                self.plan_since=None;self.plan_count=0
            # Bridging may continue an already moving approach, but cannot
            # accumulate confirmation for a restart after stopping. Its 250ms
            # lifetime would be almost spent by the 200ms recovery check.
            restart_evidence=(plan['allowed'] and not (
                self.report.get('initial_position') or {}).get('bridge_used',False))
            if restart_evidence:
                if self.plan_since is None:self.plan_since=now
                self.plan_count+=1
            else:self.plan_since=None;self.plan_count=0
            stable=self.plan_count>=3 and self.plan_since is not None and now-self.plan_since>=.2-1e-9
            resumed,cause,expired=self.recovery.update(stable,self.stop_motion,t,now,.5)
            initial_expired=(not self.position_acquired and self.initial.remaining(now)<=0
                             and now>=self.brake_until)
            failure='route_unavailable' if not stable else cause
            self.report.update(recovery_cause=cause,plan_confirmed=stable,pause_reason=self.pause_reason,
                               recovery_failure=failure,recovery_timing=self.recovery.status(now),
                               restart_evidence_current=restart_evidence)
            if initial_expired:
                return {'phase':'fault','action':'neutral','reason':'初次车位定位超时，保持停止；需要重新对准或补充定位依据'}
            if expired:
                detail=(plan['reason'] if not plan['allowed'] else '新路线连续确认不足' if not stable else
                    {'still_moving':'仍观察到移动','stop_feedback_invalid':'停稳反馈无效',
                     'stop_confirmation_pending':'停稳连续确认不足'}.get(cause,cause))
                return {'phase':'fault','action':'neutral','reason':detail+'；暂停恢复超时，保持停止'}
            if not resumed:
                return {'phase':'boundary_wait','action':'brake' if now<self.brake_until else 'neutral',
                        'reason':self.pause_reason+'；等待新鲜路线和连续停稳'}
            self.plan_revision+=1;self.report['plan_revision']=self.plan_revision
            self.paused=None;self.recovery.reset()
        return {'phase':'approach' if plan['stage']!='seek' else 'seek','action':'creep',
                'steering_error':plan['error'],'reason':plan['reason']}

    def _finish(self,sample,now):
        # During braking an edge may leave the camera before the car settles.
        # A fresh bounded motion update still provides current body position;
        # losing image guidance must not erase it. Missing/stale pose continues
        # to prevent completion, regardless of the retained progress stage.
        if (self.anchor is not None and self.frame==sample['frame_id']
                and self.capture==sample['captured_s']):
            self.report.update(self._body(self.anchor))
        if now<self.brake_until:
            return {'phase':'brake_pulse','action':'brake','reason':'有限制动中，持续检查入位位置'}
        motion=self.stop_motion
        stopped=motion.get('valid') is True and motion.get('moving') is False
        inside=(self.report.get('rear_inside_estimate') is True
                and self.report.get('side_clearance_estimate_m',-1)>0
                and self.report.get('far_clearance_estimate_m',-1)>0
                and abs(self.report.get('heading_error_rad',9))<.15)
        if stopped and inside:
            if self.stop_since is None:self.stop_since=now
            self.stop_frames+=1
            if self.stop_frames>=3 and now-self.stop_since>=.5:
                return {'phase':'done','action':'neutral','reason':'入位位置与停稳估计连续成立；仍待现场验收'}
        else:self.stop_since=None;self.stop_frames=0
        if now-self.stopping>2.5:
            return {'phase':'fault','action':'neutral','reason':'制动后未能持续确认车身位置和停稳，保持停止'}
        return {'phase':'settle','action':'neutral','reason':'保持停止，连续核对车尾、侧向余量和运动反馈'}
