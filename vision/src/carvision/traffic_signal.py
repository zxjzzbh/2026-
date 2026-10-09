"""Horizontal traffic fixture perception, including clipped/bloomed lenses.

No motion/serial interfaces. Colors are lamp POSITION plus emission evidence,
not the color of an unlit plastic lens or a countdown timeout.
"""
import json
import math
import time
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

from .traffic_lamps import _color

DEFAULTS = {'schema_version':1,'algorithm':'fixture_photometry_v1',
            'min_emission_value':230.0,'min_brightness_margin':25.0,
            'min_emission_fraction':.25,'off_max_value':225.0,'bloom_min_brightness_margin':15.0,
            'nonwhite_min_brightness_margin':75.0,
            'green_confirm_s':.6,'green_min_frames':3,'max_frame_gap_s':.8,
            'recommended_camera':'primary','field_verified':False}

PROFILE_PATH=Path(__file__).resolve().parents[2]/'configs/traffic-signal.json'


def load_profile(path=None):
    profile=dict(DEFAULTS)
    if path is None and PROFILE_PATH.is_file():path=PROFILE_PATH
    if path:
        data=json.loads(Path(path).read_text(encoding='utf-8-sig'))
        if not isinstance(data,dict) or data.get('schema_version')!=1 or data.get('algorithm')!='fixture_photometry_v1':raise ValueError('invalid traffic profile')
        profile.update(data)
    for name,low,high in [('min_emission_value',200,255),('min_brightness_margin',15,100),
                          ('min_emission_fraction',.1,1),('off_max_value',100,229),
                          ('bloom_min_brightness_margin',10,100),
                          ('nonwhite_min_brightness_margin',25,150),
                          ('green_confirm_s',.3,3),('max_frame_gap_s',.1,2)]:
        x=profile.get(name)
        if type(x) not in (int,float) or not math.isfinite(x) or not low<=x<=high:
            raise ValueError('invalid '+name)
    if type(profile['green_min_frames']) is not int or not 3<=profile['green_min_frames']<=20:
        raise ValueError('green confirmation needs at least three distinct frames')
    if profile['recommended_camera'] not in ('primary','secondary'):raise ValueError('invalid camera selection')
    return profile


def _round_candidates(image,hsv):
    h,w=image.shape[:2];hue,sat,val=cv2.split(hsv)
    candidates=[]
    def add(x,y,r,weight):
        if not 4<=r<=min(h*.3,w*.2):return
        for old in candidates:
            if np.hypot(x-old[0],y-old[1])<min(r,old[2])*.35 and max(r,old[2])/min(r,old[2])<1.6:
                return
        candidates.append((float(x),float(y),float(r),weight))
    # Colored lenses can remain usable even when their outer rim touches the
    # image edge. Do not require a full padding rectangle to be visible.
    for index in range(3):
        mask=(_color(hue,index)&(sat>=40)&(val>=35)).astype('uint8')*255
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x,y,bw,bh=cv2.boundingRect(c);area=cv2.contourArea(c)
            if (min(bw,bh)<5 or max(bw,bh)>min(w*.35,h*.55) or not .45<=bw/bh<=2.4
                    or area<.35*bw*bh):continue
            add(x+(bw-1)/2,y+(bh-1)/2,(bw+bh)/4,1)
    # Color-independent ellipse contours recover unlit neighbors when flare
    # shifts their apparent hue. The third lamp still needs its own evidence.
    edge=cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),(3,3),0),40,100)
    contours,_=cv2.findContours(edge,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    for c in contours:
        if len(c)<20:continue
        (x,y),(a,b),_=cv2.fitEllipse(c)
        if min(a,b)<8 or max(a,b)>min(w*.35,h*.55) or max(a,b)/min(a,b)>2.2:continue
        if not .65<cv2.contourArea(c)/(np.pi*a*b/4)<1.2:continue
        add(x,y,(a+b)/4,1)
    return sorted(candidates,key=lambda c:c[2],reverse=True)[:45]


def _features(hsv,cx,cy,r,index,anchors):
    h,w=hsv.shape[:2];pad=r*1.6
    x1,y1=max(0,int(cx-pad)),max(0,int(cy-pad))
    x2,y2=min(w,int(cx+pad+1)),min(h,int(cy+pad+1))
    if x2<=x1 or y2<=y1:return None
    yy,xx=np.ogrid[y1:y2,x1:x2];rr=(xx-cx)**2+(yy-cy)**2
    core=rr<=(r*.55)**2;ring=(rr>=(r*.72)**2)&(rr<=(r*1.5)**2)
    if core.sum()<max(12,.5*np.pi*(r*.55)**2) or ring.sum()<20:return None
    hue,sat,val=cv2.split(hsv[y1:y2,x1:x2])
    color=_color(hue,index)&(sat>=35)&(val>=40)
    quadrants=[ring&(xx<cx if left else xx>=cx)&(yy<cy if top else yy>=cy)
               for left,top in ((True,True),(True,False),(False,True),(False,False))]
    visible=[q for q in quadrants if q.sum()>=max(5,.1*np.pi*((r*1.5)**2-(r*.72)**2))]
    distributed=len(visible)>=2 and sum(float(color[q].mean())>=.025 for q in visible)>=min(3,len(visible))
    shape=any(np.hypot(cx-a[0],cy-a[1])<r*.5 and .55<r/a[2]<1.8 for a in anchors)
    return {'median_v':float(np.median(val[core])),
            'emission_fraction':float(((val>=230)&((sat<=45)|color))[core].mean()),
            'white_fraction':float(((val>=238)&(sat<=45))[core].mean()),
            'color_fraction':float(color[core].mean()),'halo_fraction':float(color[ring].mean()),
            'saturated_fraction':float((sat[core]>=50).mean()),'shape_support':bool(shape),
            'distributed_halo':bool(distributed),
            'visible_core_fraction':min(1.0,float(core.sum()/(np.pi*(r*.55)**2)))}


def _iou(a,b):
    area=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    return area/max(1,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area)


class TrafficSignalDetector:
    def __init__(self,profile=None):self.profile=load_profile() if profile is None else profile

    def detect(self,image):
        started=time.perf_counter()
        if image is None or image.ndim!=3 or image.shape[2]!=3:raise ValueError('BGR image required')
        h,w=image.shape[:2];hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);anchors=_round_candidates(image,hsv)
        proposals=[];seen=set()
        for first,second in combinations(sorted(anchors),2):
            if max(first[2],second[2])/min(first[2],second[2])>1.8:continue
            radius=(first[2]+second[2])/2
            if abs(second[1]-first[1])>radius*.6:continue
            for i,j in ((0,1),(1,2),(0,2)):
                dx,dy=(second[0]-first[0])/(j-i),(second[1]-first[1])/(j-i)
                if not 2.3*radius<=dx<=5.5*radius:continue
                centers=[(first[0]+(k-i)*dx,first[1]+(k-i)*dy) for k in range(3)]
                if centers[0][0]<-radius*.1 or centers[-1][0]>w+radius*.1:continue
                signature=tuple(round(v/2) for v in (*centers[1],radius))
                if signature in seen:continue
                seen.add(signature)
                fs=[_features(hsv,x,y,radius,k,anchors) for k,(x,y) in enumerate(centers)]
                if any(f is None for f in fs):continue
                evidence=[f['shape_support'] or (f['white_fraction']>.45 and f['halo_fraction']>.04) for f in fs]
                if not all(evidence):continue
                color_support=sum(f['color_fraction']>.12 or f['halo_fraction']>.1 for f in fs)
                if color_support<2:continue
                bbox=[max(0,int(centers[0][0]-radius*1.3)),max(0,int(min(y for x,y in centers)-radius*1.3)),
                      min(w,int(centers[-1][0]+radius*1.3+1)),min(h,int(max(y for x,y in centers)+radius*1.3+1))]
                score=sum(min(f['color_fraction'],.8)+min(f['halo_fraction'],.3)+.3*f['shape_support'] for f in fs)
                score+=min(radius/100,.3)
                proposals.append({'bbox':bbox,'centers':centers,'radius':radius,'features':fs,'score':score})
        proposals.sort(key=lambda p:p['score'],reverse=True)
        groups=[]
        for p in proposals:
            if not any(_iou(p['bbox'],g['bbox'])>.4 for g in groups):groups.append(p)
        result={'schema_version':1,'state':'unknown','signal_for_race':'unknown','fixture_detected':bool(groups),
                'bbox_xyxy':None,'lamp_features':[],'reason':'no_supported_fixture',
                'algorithm':self.profile['algorithm'],'field_verified':False,'hardware_output':False}
        if groups:
            g=groups[0];fs=g['features'];vs=[f['median_v'] for f in fs]
            result.update(bbox_xyxy=g['bbox'],lamp_features=fs,lamp_centers=g['centers'],lamp_radius_px=g['radius'])
            active=[]
            for i,f in enumerate(fs):
                margin=self.profile['min_brightness_margin']
                if f['white_fraction']<.3:
                    margin=max(margin,self.profile['nonwhite_min_brightness_margin'])
                if f['white_fraction']>=.6 and max(fs[j]['white_fraction'] for j in range(3) if j!=i)<.2:
                    margin=self.profile['bloom_min_brightness_margin']
                if (f['median_v']>=self.profile['min_emission_value']
                        and f['median_v']-max(vs[j] for j in range(3) if j!=i)>=margin
                        and f['emission_fraction']>=self.profile['min_emission_fraction']
                        and (f['color_fraction']>=.18 or f['halo_fraction']>=.05 and f['distributed_halo'])):
                    active.append(i)
            if len(groups)>1 and groups[1]['score']>=g['score']*.9:
                result['reason']='multiple_fixture_candidates'
            elif len(active)==1:
                state=('red','yellow','green')[active[0]]
                result.update(state=state,signal_for_race=state,reason='position_and_emission_agree')
            elif max(vs)<self.profile['off_max_value'] and all(f['color_fraction']>.12 for f in fs):
                result.update(state='off',reason='fixture_present_without_emission')
            else:result['reason']='ambiguous_emission'
        result['processing_ms']=(time.perf_counter()-started)*1000
        return result


class SignalStability:
    """Green needs distinct, fresh frames of the same fixture. Non-green cancels immediately."""
    def __init__(self,profile=None):
        self.profile=profile or load_profile()
        self.reset()

    def reset(self):
        self.last_id=self.last_time=self.green_since=self.box=None
        self.count=0
        self.result={'state':'unknown','green_confirmed':False,'reason':'no_fresh_evidence','frames':0}

    def update(self,observation,frame_id,captured_s,now_s):
        p=self.profile
        if (type(captured_s) not in (int,float) or type(now_s) not in (int,float)
                or not math.isfinite(captured_s) or not math.isfinite(now_s)
                or not 0<=now_s-captured_s<=p['max_frame_gap_s']):
            self.reset();return dict(self.result)
        raw=observation.get('state','unknown')
        if (type(frame_id) not in (int,str) or frame_id==self.last_id
                or type(frame_id) is int and type(self.last_id) is int and frame_id<self.last_id
                or self.last_time is not None and captured_s<=self.last_time):
            self.reset();self.result['reason']='duplicate_or_reversed_frame';return dict(self.result)
        if self.last_time is not None and captured_s-self.last_time>p['max_frame_gap_s']:
            self.reset()
        box=observation.get('bbox_xyxy')
        box_ok=(isinstance(box,list) and len(box)==4
                and all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in box)
                and box[2]>box[0] and box[3]>box[1])
        if raw!='green' or observation.get('fixture_detected') is not True or not box_ok:
            self.green_since=None;self.count=0;self.box=box
            self.result={'state':raw if raw in ('red','yellow','off') else 'unknown',
                         'green_confirmed':False,'reason':'not_green','frames':0}
        else:
            if self.green_since is None or self.box is None or _iou(box,self.box)<.4:
                self.green_since=captured_s;self.count=0
            self.count+=1;self.box=box
            confirmed=self.count>=p['green_min_frames'] and captured_s-self.green_since>=p['green_confirm_s']
            self.result={'state':'green' if confirmed else 'unknown','green_confirmed':confirmed,
                         'reason':'stable_green' if confirmed else 'confirming_green',
                         'frames':self.count,'green_span_s':captured_s-self.green_since}
        self.last_id,self.last_time=frame_id,captured_s
        self.result.update(captured_s=captured_s,valid_until_s=captured_s+p['max_frame_gap_s'])
        return dict(self.result)

    def current(self,now_s):
        if self.last_time is None or not 0<=now_s-self.last_time<=self.profile['max_frame_gap_s']:
            return {'state':'unknown','green_confirmed':False,'reason':'stale_frame','frames':0}
        return dict(self.result)


def traffic_light_intent(stable, *, camera, in_stop_zone=False, stopped_verified=False,
                         red_seen_after_entry=False, now_s=None, profile=None):
    """High-level rule adapter only. No measured zone/stop input -> keep waiting."""
    p=profile or load_profile()
    now=time.monotonic() if now_s is None else now_s
    captured,deadline=stable.get('captured_s'),stable.get('valid_until_s')
    fresh=(type(now) in (int,float) and math.isfinite(now) and type(captured) in (int,float)
           and type(deadline) in (int,float) and math.isfinite(captured) and math.isfinite(deadline)
           and captured<=now<=deadline)
    eligible=(camera==p['recommended_camera'] and in_stop_zone is True and stopped_verified is True
              and red_seen_after_entry is True
              and fresh and stable.get('green_confirmed') is True and stable.get('state')=='green')
    return {'intent':'may_continue' if eligible else 'hold',
            'reason':'confirmed_green_after_stop' if eligible else 'waiting_for_zone_stop_and_fresh_green',
            'hardware_output':False,'requires_real_stop_zone_and_speed_feedback':True}


class TrafficLightStopLogic:
    """Observe the triggered red-to-green cycle inside a separately verified zone."""
    def __init__(self,profile=None):
        self.profile=profile or load_profile()
        self.tracker=SignalStability(self.profile)
        self.entered=False;self.red_seen=False

    def update(self,observation,*,camera,frame_id,captured_s,now_s,in_stop_zone=False,stopped_verified=False):
        if in_stop_zone is not True or camera!=self.profile['recommended_camera']:
            self.entered=False;self.red_seen=False;self.tracker.reset()
            return traffic_light_intent({},camera=camera,now_s=now_s,profile=self.profile)
        if not self.entered:
            self.tracker.reset();self.red_seen=False;self.entered=True
        stable=self.tracker.update(observation,frame_id,captured_s,now_s)
        if stable.get('state')=='red':self.red_seen=True
        result=traffic_light_intent(stable,camera=camera,in_stop_zone=True,stopped_verified=stopped_verified,
                                   red_seen_after_entry=self.red_seen,now_s=now_s,profile=self.profile)
        result.update(stable=stable,red_seen_after_entry=self.red_seen)
        return result
