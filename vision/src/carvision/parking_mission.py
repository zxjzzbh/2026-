"""Measured parking decisions and whole-body route checks, without motor I/O.

World axes: x right, y along the parking area's longitudinal divider. Pose is
the rear axle. Caller must provide verified live pose and two world-frame bays.
"""
import math

import cv2
import numpy as np

from .parking_bay import number,readiness,validate
from .cone_planning import smooth_path


def bounds(slot):
    p=np.asarray(slot.get('polygon_m'),dtype=float)
    if p.shape!=(4,2) or not np.isfinite(p).all():raise ValueError('停车区需要四个有效的米制角点')
    x0,y0=p.min(axis=0);x1,y1=p.max(axis=0)
    if x1<=x0 or y1<=y0:raise ValueError('停车区退化')
    if not cv2.isContourConvex(p.astype(np.float32)) or abs(cv2.contourArea(p.astype(np.float32)))<.9*(x1-x0)*(y1-y0):
        raise ValueError('停车区角点顺序或覆盖范围无效')
    # The planner's frame follows the straight track, not image axes.
    if any(min(abs(x-x0),abs(x-x1))>.035 or min(abs(y-y0),abs(y-y1))>.035 for x,y in p):
        raise ValueError('停车区尚未转换到沿赛道方向的坐标系')
    return tuple(float(v) for v in (x0,y0,x1,y1))


def layout(slots,s):
    if not isinstance(slots,list) or len(slots)!=2 or any(not isinstance(r,dict) for r in slots) or {r.get('id') for r in slots}!={'left','right'}:
        raise ValueError('必须有车辆前进方向的左、右两个半区')
    by_id={r['id']:r for r in slots};a,b=bounds(by_id['left']),bounds(by_id['right'])
    if (abs(a[2]-b[0])>.04 or abs(a[1]-b[1])>.06 or abs(a[3]-b[3])>.06
            or any(abs((r[3]-r[1])-s['zone_length_m'])>.12 for r in (a,b))
            or any(abs((r[2]-r[0])-s['lane_width_m']/2)>.06 for r in (a,b))):
        raise ValueError('两个半区的边界、长度或赛道宽度不一致')
    return by_id,{k:bounds(v) for k,v in by_id.items()}


def body(pose,s,pad=0.):
    co,si=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
    points=[]
    for x,y in [(-s['vehicle_width_m']/2-pad,-s['rear_extent_m']-pad),
                (s['vehicle_width_m']/2+pad,-s['rear_extent_m']-pad),
                (s['vehicle_width_m']/2+pad,s['front_extent_m']+pad),
                (-s['vehicle_width_m']/2-pad,s['front_extent_m']+pad)]:
        points.append([pose['x_m']+co*x+si*y,pose['y_m']-si*x+co*y])
    return np.asarray(points)


def contained(pose,box,s,pad):
    p=body(pose,s,pad);x0,y0,x1,y1=box
    return bool(np.all(p[:,0]>=x0) and np.all(p[:,0]<=x1) and np.all(p[:,1]>=y0) and np.all(p[:,1]<=y1))


def plan_entry(slots,chosen,pose,s):
    _,bs=layout(slots,s);box=bs[chosen];x0,y0,x1,y1=box
    if abs(pose['yaw_rad'])>.05:raise ValueError('入位规划前需先与直赛道方向对齐')
    pad=s['clearance_m']+s['maximum_pose_error_m']+.015
    center=(x0+x1)/2;align=y0-s['front_extent_m']-pad
    goal=(y0+y1)/2+(s['rear_extent_m']-s['front_extent_m'])/2
    if align<=pose['y_m']+.1:raise ValueError('接近距离不足以在入口前完成横移，请更早识别停车区')
    path=smooth_path([(pose['y_m'],pose['x_m']),(align,center),(goal,center)],step=.015)
    for row in path:
        if abs(row['curvature'])>1/s['minimum_turn_radius_m']:raise ValueError('所需转向超过实测能力')
        p=body(row,s,pad)
        if p[:,0].min()<bs['left'][0] or p[:,0].max()>bs['right'][2]:raise ValueError('整车扫过范围超出赛道')
        if p[:,1].max()>=y0 and (p[:,0].min()<x0 or p[:,0].max()>x1):raise ValueError('入口处整车会压分隔线或进入遮挡半区')
        if p[:,1].max()>y1:raise ValueError('路径越过停车区末端')
    # There must still be a full-car-inside braking corridor before the goal.
    if goal-s['braking_distance_m']-s['max_speed_mps']*s['max_frame_age_s']-s['maximum_pose_error_m']<y0+s['rear_extent_m']+pad:
        raise ValueError('当前制动距离超过车位内余量，需降低实测接近速度')
    if not contained(path[-1],box,s,pad):raise ValueError('目标位置无法容纳整车')
    return path


class ParkingMission:
    def __init__(self,settings):
        self.s=validate(settings);self.phase='observe';self.selected=None;self.pending=None;self.pending_since=None;self.pending_count=0
        self.last_t=None;self.last_id=None;self.path=[];self.selected_bounds=None;self.still_since=None;self.brake_since=None
        self.previous_pose=None
        self.enter_started=None;self.stall_since=None

    def _result(self,action,reason,**kw):
        return {'action':action,'phase':self.phase,'reason':reason,'selected_slot':self.selected,
                'hardware_output':False,'payment_ready':self.phase=='parked','payment_amount_cents':1,
                'payment_provider_verified':False,**kw}

    def step(self,sample,now):
        if self.phase=='fault':return self._result('stop','需重新开始本轮停车')
        if not isinstance(sample,dict):
            self.phase='fault';return self._result('stop','停车观测格式无效')
        t=sample.get('captured_s');fid=sample.get('frame_id')
        if (not number(now) or not number(t) or type(fid) is not int or not 0<=now-t<=self.s['max_frame_age_s']
                or self.last_t is not None and (t<=self.last_t or fid<=self.last_id)):
            self.phase='fault';return self._result('stop','图像或反馈时间无效，停车并重新准备')
        if self.last_t is not None and t-self.last_t>self.s['max_frame_age_s']:
            self.pending=None;self.pending_since=None;self.pending_count=0;self.still_since=None
            if self.selected:
                self.phase='fault';return self._result('brake','反馈有中断，不能沿旧路径继续入位')
        self.last_t,self.last_id=t,fid
        if self.enter_started is not None and self.phase!='parked' and now-self.enter_started>self.s['max_run_s']:
            self.phase='fault';return self._result('brake','本轮入位超过时限，停止并重新准备')
        ready=readiness(self.s)
        if not ready['ready']:return self._result('stop','等待实测标定',missing=ready['missing'])
        pose=sample.get('pose') or {};speed=sample.get('speed_mps')
        valid=(pose.get('verified') is True and all(number(pose.get(k)) for k in ('x_m','y_m','yaw_rad','error_bound_m','captured_s'))
               and 0<=now-pose['captured_s']<=self.s['max_frame_age_s'] and 0<=pose['error_bound_m']<=self.s['maximum_pose_error_m']
               and number(speed) and abs(speed)<=self.s['max_speed_mps'] and speed>=-.025)
        if not valid or sample.get('obstacle_region_verified') is not True or sample.get('coordinate_frame')!='parking_road_world':
            self.pending=None;self.pending_since=None;self.pending_count=0;self.still_since=None
            if self.phase=='parked':self.phase='braking'
            return self._result('brake' if self.selected else 'stop','位姿、速度或路径可见性未确认')
        if self.previous_pose:
            prev=self.previous_pose;dt=pose['captured_s']-prev['captured_s']
            if dt<=0 or math.hypot(pose['x_m']-prev['x_m'],pose['y_m']-prev['y_m'])>self.s['max_speed_mps']*dt+2*self.s['maximum_pose_error_m']:
                self.phase='fault';return self._result('brake','位姿跳变，不能据此确认入位')
            yaw_delta=math.atan2(math.sin(pose['yaw_rad']-prev['yaw_rad']),math.cos(pose['yaw_rad']-prev['yaw_rad']))
            if abs(yaw_delta)>self.s['max_speed_mps']/self.s['minimum_turn_radius_m']*dt+.08:
                self.phase='fault';return self._result('brake','航向跳变超过已验证运动范围')
        self.previous_pose=dict(pose)
        try:slots,bs=layout(sample.get('slots'),self.s)
        except (ValueError,TypeError) as exc:
            self.pending=None;self.pending_since=None;self.pending_count=0;self.still_since=None
            if self.phase=='parked':self.phase='braking'
            return self._result('brake' if self.selected else 'stop',str(exc))
        if any(r.get('occupancy_verified') is not True or r.get('availability') not in ('clear','blocked') for r in slots.values()):
            self.pending=None;self.pending_since=None;self.pending_count=0;self.still_since=None
            if self.phase=='parked':self.phase='braking'
            return self._result('brake' if self.selected else 'stop','入口或半区占用不明确')
        clear=[k for k,v in slots.items() if v['availability']=='clear'];blocked=[k for k,v in slots.items() if v['availability']=='blocked']
        if len(clear)!=1 or len(blocked)!=1:
            self.pending=None;self.pending_since=None;self.pending_count=0;self.still_since=None
            if self.phase=='parked':self.phase='braking'
            return self._result('brake' if self.selected else 'stop','需同时确认一个遮挡半区和一个空闲半区')
        chosen=clear[0]
        if self.selected and chosen!=self.selected:
            self.phase='fault';return self._result('brake','已选车位变为遮挡，禁止临时跨区换位')
        if self.selected is None:
            if abs(speed)>.025:return self._result('stop','先停车确认目标半区')
            if self.pending!=chosen:self.pending=chosen;self.pending_since=t;self.pending_count=0
            self.pending_count+=1
            if self.pending_count<3 or t-self.pending_since<self.s['selection_confirm_s']:return self._result('stop','正在连续确认未遮挡半区',candidate_slot=chosen)
            try:self.path=plan_entry(list(slots.values()),chosen,pose,self.s)
            except ValueError as exc:return self._result('stop',str(exc))
            self.selected=chosen;self.selected_bounds=bs[chosen];self.phase='entering';self.enter_started=now
        if max(abs(a-b) for a,b in zip(self.selected_bounds,bs[chosen]))>.05:
            self.phase='fault';return self._result('brake','停车区坐标跳变或重定位不一致')
        pad=self.s['clearance_m']+pose['error_bound_m']
        inside=contained(pose,bs[chosen],self.s,pad)
        points=body(pose,self.s,pad)
        if points[:,1].max()>bs[chosen][3] or points[:,0].min()<bs['left'][0] or points[:,0].max()>bs['right'][2]:
            self.phase='fault';return self._result('brake','整车已接近外边界，立即停车')
        if points[:,1].max()>=bs[chosen][1] and (points[:,0].min()<bs[chosen][0] or points[:,0].max()>bs[chosen][2]):
            self.phase='fault';return self._result('brake','实际车身扫过范围越过选中半区，立即停车')
        nearest=min(self.path,key=lambda p:math.hypot(p['x_m']-pose['x_m'],p['y_m']-pose['y_m']))
        if math.hypot(nearest['x_m']-pose['x_m'],nearest['y_m']-pose['y_m'])>self.s['maximum_pose_error_m']+.06:
            self.phase='fault';return self._result('brake','实际位姿偏离已校核路径')
        remaining=self.path[-1]['y_m']-pose['y_m']
        if self.phase in ('braking','parked') or remaining<=self.s['braking_distance_m']+max(speed,0)*self.s['max_frame_age_s']+pose['error_bound_m']:
            if self.brake_since is None:self.brake_since=now
            self.phase='braking'
            if abs(speed)<=.025 and inside:
                if self.still_since is None:self.still_since=t
                if t-self.still_since>=self.s['stop_hold_s']:
                    self.phase='parked';return self._result('hold','整车外廓在正确半区内且持续停稳，可进入人工扫码缴费',four_wheels_inside=True)
            else:self.still_since=None
            if now-self.brake_since>3:
                self.phase='fault';return self._result('brake','制动后未取得正确车位内的停稳反馈')
            return self._result('brake' if abs(speed)>.025 else 'hold','主动制动并核对整车入位',four_wheels_inside=inside)
        ahead=next((p for p in self.path if p['y_m']>=pose['y_m']+.25),self.path[-1])
        dx,dy=ahead['x_m']-pose['x_m'],ahead['y_m']-pose['y_m'];co,si=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
        lateral=co*dx-si*dy;forward=si*dx+co*dy
        if forward<=0:
            self.phase='fault';return self._result('brake','路径目标落在车后，禁止盲目前进')
        if abs(speed)<.01:
            if self.stall_since is None:self.stall_since=now
            if now-self.stall_since>=2:
                self.phase='fault';return self._result('brake','持续请求前进但未观察到移动')
        else:self.stall_since=None
        curvature=2*lateral/max(.01,lateral*lateral+forward*forward)
        if abs(curvature)>1/self.s['minimum_turn_radius_m']:
            return self._result('brake','实时转向需求超出已验证范围')
        return self._result('follow_path','低速沿已校核路线进入选中半区',speed_limit_mps=self.s['max_speed_mps'],
                            target_curvature_1pm=curvature,remaining_m=remaining,path=self.path)
