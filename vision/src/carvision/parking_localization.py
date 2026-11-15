"""Rigid parking coordinates and stopped relocalization evidence; no I/O."""
import copy
import math

import numpy as np

from .parking_bay import number
from .parking_mission import layout
from .runtime_timing import MOTION_MAX_AGE_S


def transform_slots(slots, pose):
    co,si=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
    result=copy.deepcopy(slots)
    for slot in result:
        slot['polygon_m']=[[pose['x_m']+co*x+si*y,pose['y_m']-si*x+co*y]
                           for x,y in slot['polygon_m']]
    return result


def verified_slots(view):
    slots=view.get('slots')
    if (not view.get('metric_geometry') or not isinstance(slots,list) or len(slots)!=2
            or any(not isinstance(s,dict) for s in slots)
            or {s.get('id') for s in slots}!={'left','right'}
            or sorted(s.get('availability','') for s in slots)!=['blocked','clear']
            or any(s.get('occupancy_verified') is not True for s in slots)):
        raise ValueError('完整米制车位与遮挡半区尚未确认')
    for slot in slots:
        points=np.asarray(slot.get('polygon_m'),dtype=float)
        if points.shape!=(4,2) or not np.isfinite(points).all():
            raise ValueError('车位角点无效')
    return sorted(slots,key=lambda s:s['id'])


def road_pose(slots, settings):
    """Establish the road axis at the current rear axle, never image axes."""
    directions=[np.asarray(s['polygon_m'][3])-s['polygon_m'][0] for s in slots]
    angles=[math.atan2(v[0],v[1]) for v in directions]
    if any(v[1]<=0 for v in directions) or max(angles)-min(angles)>.025:
        raise ValueError('停车区纵向边方向不一致')
    heading=sum(angles)/len(angles)
    if abs(heading)>.4:raise ValueError('停车区朝向超出支持范围')
    pose={'x_m':0.,'y_m':0.,'yaw_rad':-heading}
    layout(transform_slots(slots,pose),settings)
    return pose


def register_slots(slots, anchor, settings, last_pose, elapsed, error_bound):
    """Fit current local corners onto the SAME original world-frame bays."""
    current=sorted(slots,key=lambda s:s['id']);reference=sorted(anchor,key=lambda s:s['id'])
    if any(a['id']!=b['id'] or a['availability']!=b['availability'] for a,b in zip(current,reference)):
        raise ValueError('原车位遮挡或身份发生变化')
    a=np.concatenate([np.asarray(s['polygon_m'],float) for s in current])
    b=np.concatenate([np.asarray(s['polygon_m'],float) for s in reference])
    ac,bc=a.mean(axis=0),b.mean(axis=0)
    u,_,vt=np.linalg.svd((a-ac).T@(b-bc));rotation=u@vt
    if np.linalg.det(rotation)<.999:raise ValueError('车位角点反射或顺序错误')
    offset=bc-ac@rotation
    residual=float(np.linalg.norm(a@rotation+offset-b,axis=1).max())
    error=error_bound+residual
    if error>settings['maximum_pose_error_m']+1e-9:raise ValueError('重新定位误差超过预算')
    pose={'x_m':float(offset[0]),'y_m':float(offset[1]),
          'yaw_rad':math.atan2(rotation[1,0],rotation[0,0]),'error_bound_m':error}
    distance=math.hypot(pose['x_m']-last_pose['x_m'],pose['y_m']-last_pose['y_m'])
    bound=min(settings['max_speed_mps']*elapsed+2*settings['maximum_pose_error_m'],settings['lane_width_m']*.35)
    yaw=math.atan2(math.sin(pose['yaw_rad']-last_pose['yaw_rad']),math.cos(pose['yaw_rad']-last_pose['yaw_rad']))
    if (distance>bound or abs(yaw)>min(.35,settings['max_speed_mps']/settings['minimum_turn_radius_m']*elapsed+.08)):
        raise ValueError('重新定位与最后有效位姿不连续')
    layout(transform_slots(slots,pose),settings)
    return pose


def continuous_rest(motion, now, after):
    row=motion or {};stamp=row.get('captured_s');since=row.get('continuous_still_since_s')
    return (row.get('valid') is True and row.get('moving') is False and number(stamp)
            and number(since) and 0<=now-stamp<=MOTION_MAX_AGE_S and since<=stamp
            and stamp-max(since,after)>=.3-1e-9)
