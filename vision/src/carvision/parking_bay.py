"""Yellow-tape parking layout and blue entrance obstruction on real images.

Left/right are relative to the forward-facing camera, not the paper diagram.
Image-only candidates never claim metric pose or clearance verification.
"""
import copy
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .autonomy_profile import GroundProjection


def number(x):return type(x) in (int,float) and math.isfinite(x)


def validate(s):
    if not isinstance(s,dict) or s.get('schema_version')!=1 or s.get('camera')!='secondary' or s.get('image_size')!=[480,360]:
        raise ValueError('停车使用第二路 H65 的480×360原图')
    s=copy.deepcopy(s);s.setdefault('max_run_s',30.)
    for name,lo,hi in [('zone_length_m',1,1),('lane_width_m',.6,2),('vehicle_width_m',.1,.6),
                       ('front_extent_m',.1,.8),('rear_extent_m',.01,.4),('clearance_m',.01,.1),
                       ('blocked_blue_fraction',.005,.3),('clear_ground_fraction',.5,.95),
                       ('selection_confirm_s',.2,2),('stop_hold_s',.4,2),('max_frame_age_s',.1,.25),('max_speed_mps',.02,.25),('max_run_s',1,60)]:
        if not number(s.get(name)) or not lo<=s[name]<=hi:raise ValueError('停车参数越界：'+name)
    for name in ('minimum_turn_radius_m','braking_distance_m','maximum_pose_error_m'):
        if s.get(name) is not None and (not number(s[name]) or s[name]<=0):raise ValueError(name+' must be measured positive metres or null')
    if type(s.get('perception_verified')) is not bool:raise ValueError('perception_verified must be boolean')
    for color in ('yellow','blue'):
        a,b=s.get(color+'_lower'),s.get(color+'_upper')
        if not all(isinstance(v,list) and len(v)==3 and all(type(x) is int for x in v) for v in (a,b)):
            raise ValueError('HSV must contain three integers')
        if not all(0<=a[i]<=b[i]<=(179 if i==0 else 255) for i in range(3)):raise ValueError('HSV range invalid')
    if s.get('ground') is not None:
        if not isinstance(s['ground'],dict) or type(s['ground'].get('verified')) is not bool:raise ValueError('ground.verified must be boolean')
        GroundProjection(s['ground'],require_verified=False)
        if s['ground'].get('image_size')!=s['image_size']:raise ValueError('地面标定分辨率不匹配')
    return copy.deepcopy(s)


def load_settings(path):return validate(json.loads(Path(path).read_text(encoding='utf-8-sig')))


def readiness(s):
    missing=[]
    if (s.get('ground') or {}).get('verified') is not True:missing.append('第二路固定姿态的近场地面标定')
    if not s.get('perception_verified'):missing.append('停车区/入口遮挡识别的现场验证')
    for key,label in [('minimum_turn_radius_m','实测转弯半径'),('braking_distance_m','当前低速的实测制动距离'),('maximum_pose_error_m','持续位姿反馈的误差上限')]:
        if s.get(key) is None:missing.append(label)
    if s['lane_width_m']/2<=s['vehicle_width_m']+2*s['clearance_m']:missing.append('半区宽度不足以容纳整车及余量')
    if s['front_extent_m']+s['rear_extent_m']+2*s['clearance_m']>=s['zone_length_m']:missing.append('车位长度不足以容纳整车及余量')
    return {'ready':not missing,'missing':missing,'hardware_output_enabled':False}


def _order(points):
    p=np.asarray(points,dtype=float)
    near=p[np.argsort(p[:,1])[-2:]];far=p[np.argsort(p[:,1])[:2]]
    near=near[np.argsort(near[:,0])];far=far[np.argsort(far[:,0])]
    return np.array([near[0],near[1],far[1],far[0]],np.float32)


def _edge_support(mask,a,b):
    pts=np.linspace(a,b,max(12,int(np.linalg.norm(np.asarray(a)-b))))
    h,w=mask.shape;valid=0
    for x,y in np.rint(pts).astype(int):
        if 0<=x<w and 0<=y<h and np.any(mask[max(0,y-3):min(h,y+4),max(0,x-3):min(w,x+4)]):valid+=1
    return valid/len(pts)


def pixel_layout(yellow,white):
    """Find two side-by-side enclosures sharing a yellow longitudinal divider."""
    mask=cv2.morphologyEx(cv2.bitwise_or(yellow,white),cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    h,w=mask.shape;quads=[]
    for contour in contours:
        area=cv2.contourArea(contour)
        if not .02*w*h<area<.7*w*h:continue
        q=cv2.approxPolyDP(contour,.025*cv2.arcLength(contour,True),True)
        if len(q)!=4 or not cv2.isContourConvex(q):continue
        q=_order(q[:,0,:]);center=q.mean(axis=0)
        if np.any(q<=2) or np.any(q[:,0]>=w-3) or np.any(q[:,1]>=h-3):continue
        if any(np.linalg.norm(center-old.mean(axis=0))<.035*w for old in quads):continue
        if min(_edge_support(yellow,q[0],q[1]),_edge_support(yellow,q[2],q[3]))<.45:continue
        quads.append(q)
    pairs=[]
    for i,a in enumerate(quads):
        for b in quads[i+1:]:
            left,right=sorted((a,b),key=lambda q:q[:,0].mean())
            gap=max(np.linalg.norm(left[1]-right[0]),np.linalg.norm(left[2]-right[3]))
            divider=(left[[1,2]]+right[[0,3]])/2
            if gap<=.06*w and _edge_support(yellow,*divider)>=.6:
                pairs.append((left,right))
    return list(pairs[0]) if len(pairs)==1 else []


def _clusters(values):
    groups=[]
    for value in values:
        if not groups or value-groups[-1][-1]>3:groups.append([int(value)])
        else:groups[-1].append(int(value))
    return [int(round(np.mean(g))) for g in groups]


def metric_layout(yellow,white,s):
    """Infer the occluded half only when measured width and visible lines agree.

    Axis-aligned straight approach only. No unsupported corner extrapolation.
    """
    p=GroundProjection(s['ground']);scale=160.;grid=np.array([[scale,0,240],[0,-scale,480],[0,0,1.]])
    warp=grid@p.h;size=(480,512)
    y=cv2.warpPerspective(yellow,warp,size,flags=cv2.INTER_NEAREST)>0
    edges=cv2.warpPerspective(cv2.bitwise_or(yellow,white),warp,size,flags=cv2.INTER_NEAREST)>0
    rows=_clusters(np.flatnonzero(y.sum(axis=1)>s['lane_width_m']*scale*.2))
    found=[];half=s['lane_width_m']*scale/2
    for far in rows:
        for near in rows:
            if not .88*scale<=near-far<=1.12*scale:continue
            centers=_clusters(np.flatnonzero(y[far:near+1].sum(axis=0)>(near-far)*.6))
            for mid in centers:
                bounds=[]
                for expected in (mid-half,mid+half):
                    lo=max(0,int(expected-6));hi=min(480,int(expected+7))
                    if hi<=lo:break
                    support=edges[far:near+1,lo:hi].sum(axis=0)
                    x=lo+int(support.argmax())
                    if support.max()<(near-far)*.45:break
                    bounds.append(x)
                if len(bounds)!=2:continue
                left,right=bounds
                if not abs((right-left)/scale-s['lane_width_m'])<=.08 or abs(mid-(left+right)/2)>6:continue
                metric=[];pixels=[]
                try:
                    for a,b in ((left,mid),(mid,right)):
                        ground=np.array([[(a-240)/scale,(480-near)/scale],[(b-240)/scale,(480-near)/scale],
                                         [(b-240)/scale,(480-far)/scale],[(a-240)/scale,(480-far)/scale]])
                        inv=np.linalg.inv(p.h);q=np.c_[ground,np.ones(4)]@inv.T;q=q[:,:2]/q[:,2:]
                        for point in q:p.point(point)
                        metric.append(ground.tolist());pixels.append(q.astype(np.float32))
                except (ValueError,np.linalg.LinAlgError):continue
                found.append((pixels,metric))
    return found[0] if len(found)==1 else ([],[])


def detect_parking(image,s):
    if [image.shape[1],image.shape[0]]!=s['image_size']:raise ValueError('停车图像尺寸与配置不一致')
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    yellow=cv2.inRange(hsv,tuple(s['yellow_lower']),tuple(s['yellow_upper']))
    white=cv2.inRange(hsv,(0,0,150),(179,80,255))
    blue=cv2.inRange(hsv,tuple(s['blue_lower']),tuple(s['blue_upper']))
    red=cv2.bitwise_or(cv2.inRange(hsv,(0,55,35),(15,255,255)),cv2.inRange(hsv,(165,55,35),(179,255,255)))
    grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);visible=(grey>18)&(grey<250)
    pixels=[];metric=[]
    if (s.get('ground') or {}).get('verified') is True:
        pixels,metric=metric_layout(yellow,white,s)
    if not pixels:pixels=pixel_layout(yellow,white)
    slots=[];display=image.copy()
    for i,poly in enumerate(pixels):
        interior=np.zeros(grey.shape,np.uint8);cv2.fillConvexPoly(interior,poly.astype(np.int32),255)
        interior=cv2.erode(interior,np.ones((7,7),np.uint8))>0
        # In image coordinates this near-edge band is for candidate visibility;
        # no blue top corner is projected as a ground contact point.
        band=np.array([poly[0],poly[1],poly[1]*.7+poly[2]*.3,poly[0]*.7+poly[3]*.3],np.int32)
        entrance=np.zeros(grey.shape,np.uint8);cv2.fillConvexPoly(entrance,band,255);entrance=entrance>0
        enough=int(interior.sum())>150 and int(entrance.sum())>50
        br=float(np.mean(blue[entrance]>0)) if enough else 0.
        floor=float(np.mean(red[interior]>0)) if enough else 0.
        vis=float(visible[interior].mean()) if enough else 0.
        blue_inside=float(np.mean(blue[interior]>0)) if enough else 0.
        availability=('blocked' if enough and br>=s['blocked_blue_fraction'] else
                      'clear' if enough and br<s['blocked_blue_fraction']/2 and blue_inside<.01 and floor>=s['clear_ground_fraction'] and vis>=.65 else 'unknown')
        verified=bool(metric and s['perception_verified'] and availability!='unknown')
        slot={'id':'left' if i==0 else 'right','availability':availability,'occupancy_verified':verified,
              'corners_px':poly.tolist(),'polygon_m':metric[i] if metric else None,
              'blue_entrance_fraction':br,'red_ground_fraction':floor,'visible_fraction':vis}
        slots.append(slot);color={'blocked':(0,0,255),'clear':(0,200,0),'unknown':(0,180,255)}[availability]
        cv2.polylines(display,[poly.astype(np.int32)],True,color,2)
        center=tuple(poly.mean(axis=0).astype(int));cv2.putText(display,slot['id']+' '+availability,center,cv2.FONT_HERSHEY_SIMPLEX,.45,color,2)
    # Always show observed yellow tape, even without a complete layout.
    contours,_=cv2.findContours(yellow,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(display,contours,-1,(0,255,255),1)
    return {'layout_found':len(slots)==2,'metric_geometry':bool(metric),'slots':slots,
            'reason':'two_bay_candidates' if slots else '停车结构未完整确认；需看到近/远黄线、黄色分隔线和两侧边界',
            'perception_verified':s['perception_verified'],'hardware_output':False},display
