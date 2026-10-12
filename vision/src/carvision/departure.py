"""Blue start-board observation. This module has no motor output path."""
import math

import cv2
import numpy as np

PHRASE = '我发车啦'
CAMERAS = ('primary','secondary')
MAX_AGE_S = .6
DEFAULT_MIN_BLUE_PERCENT = 3.


def validate_blue_percent(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not .5<=value<=30:
        raise ValueError('最小蓝色面积占比须在0.5–30%之间')
    return float(value)


def detect_board(image,min_blue_percent=DEFAULT_MIN_BLUE_PERCENT):
    threshold=validate_blue_percent(min_blue_percent)/100
    h,w=image.shape[:2]
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(90,90,50),(135,255,255))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    boxes=[];ambiguous=False;largest=0.
    for contour in contours:
        area=float(cv2.contourArea(contour));x,y,bw,bh=cv2.boundingRect(contour)
        if bw<6 or bh<6:continue
        fraction=area/(w*h);fill=area/(bw*bh);aspect=bh/bw
        # Peripheral blue props are not a board directly ahead of the car.
        if x+bw<.15*w or x>.85*w:continue
        largest=max(largest,fraction)
        vertices=len(cv2.approxPolyDP(contour,.04*cv2.arcLength(contour,True),True))
        clipped=x<=1 or y<=1 or x+bw>=w-1 or y+bh>=h-1
        normal=(not clipped and fraction>=threshold and bw/w>=.10 and bh/h>=.20
                and .5<=aspect<=2.5 and fill>=.75 and 4<=vertices<=6)
        near=(clipped and fraction>=max(.25,threshold) and bw/w>=.55 and bh/h>=.35 and fill>=.85 and 4<=vertices<=6)
        if normal or near:boxes.append([int(x),int(y),int(x+bw),int(y+bh)])
        elif fraction>=threshold:ambiguous=True
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    healthy=float(gray.std())>=6 and float(gray.mean())>=20 and float((gray>=250).mean())<.9
    state='present' if boxes else 'unknown' if ambiguous or not healthy else 'absent'
    reason={'present':'前方有大块蓝色矩形挡板','absent':'画面有效，未见挡板或大块不明确蓝色区域',
            'unknown':'画面遮挡、过暗、过曝或蓝色形状不明确，不能认定移开'}[state]
    return {'state':state,'reason':reason,'boxes':boxes,'largest_blue_fraction':largest,
            'min_blue_percent':float(min_blue_percent),'unknown_cause':'blue_shape' if ambiguous and healthy else 'image_quality' if not healthy and not boxes else None,
            'scene_valid':bool(healthy or boxes),'dimensions_cm':{'width':40,'height':50},
            'metric_distance_available':False}


class DepartureSequence:
    """Fresh images must show present first, then sustained clear in both views."""
    def __init__(self,clear_s=1.):
        if isinstance(clear_s,bool) or not isinstance(clear_s,(int,float)) or not math.isfinite(clear_s) or not .5<=clear_s<=3:
            raise ValueError('移开确认时间须在0.5–3秒之间')
        self.clear_s=float(clear_s);self.phase='wait_board';self.reason='先将蓝色挡板放在车前，等待确认存在'
        self.ids={};self.times={};self.present={};self.clear_since=None;self.clear_frames=0
        self.seen_cameras=[];self.clear_elapsed=0.;self.last_pair=None

    def invalidate(self):
        self.present={};self.clear_since=None;self.clear_frames=0;self.clear_elapsed=0.;self.last_pair=None
        # Reconnection must see the board again; a missing camera is not removal.
        self.phase='wait_board';self.seen_cameras=[]
        self.reason='图像中断或无效；需重新看到挡板，不能据此播报发车'

    def update(self,observations,now):
        if self.phase=='complete':return False
        valid=True
        for camera in CAMERAS:
            row=observations.get(camera,{})
            t=row.get('captured_s');fid=row.get('frame_id')
            if (type(fid) is not int or not isinstance(t,(int,float)) or not math.isfinite(t)
                    or not 0<=now-t<=MAX_AGE_S or row.get('state') not in ('present','absent','unknown')
                    or row.get('state')=='unknown' and row.get('scene_valid') is not True):valid=False
        if not valid:
            self.invalidate();return False
        # Process only pairs where both cameras supplied a new frame. Repeat
        # HTTP/cache reads never count as multiple removal observations.
        changed=all(observations[c]['frame_id']!=self.ids.get(c) for c in CAMERAS)
        if not changed:return False
        stamp=min(observations[c]['captured_s'] for c in CAMERAS)
        if any(observations[c]['captured_s']<=self.times.get(c,-math.inf) for c in CAMERAS):
            self.invalidate();return False
        if self.last_pair is not None and stamp-self.last_pair>MAX_AGE_S:self.invalidate()
        self.last_pair=stamp
        for c in CAMERAS:self.ids[c]=observations[c]['frame_id'];self.times[c]=observations[c]['captured_s']
        seen=[c for c in CAMERAS if observations[c]['state']=='present']
        if seen:
            self.clear_since=None;self.clear_frames=0;self.clear_elapsed=0.
            for c in CAMERAS:
                if c in seen:
                    since,count=self.present.get(c,(observations[c]['captured_s'],0))
                    self.present[c]=(since,count+1)
                else:self.present.pop(c,None)
            confirmed=[c for c,(since,count) in self.present.items() if count>=3 and observations[c]['captured_s']-since>=.5]
            if confirmed:
                self.phase='wait_removed';self.seen_cameras=confirmed
                self.reason='已确认蓝色挡板，等待移开；此时不会发车'
            return False
        if any(observations[c]['state']=='unknown' for c in CAMERAS):
            # A healthy frame with a partly lifted/blurred blue board is not
            # a camera failure. Keep prior presence, but never count it clear.
            self.present={};self.clear_since=None;self.clear_frames=0;self.clear_elapsed=0.
            self.reason=('挡板形状正在变化，保留已确认记录；等待两路画面明确清空' if self.phase=='wait_removed' else
                         '蓝色形状不明确，尚未确认挡板；继续等待')
            return False
        self.present={}
        if self.phase!='wait_removed':
            self.reason='尚未确认挡板存在；空场景不会触发发车播报';return False
        if self.clear_since is None:self.clear_since=stamp
        self.clear_frames+=1;self.clear_elapsed=max(0,stamp-self.clear_since)
        self.reason='两路均未见挡板，正在连续确认移开'
        if self.clear_frames>=4 and self.clear_elapsed>=self.clear_s:
            self.phase='complete';self.reason='挡板移开已确认；仅播报“我发车啦”，不输出前进'
            return True
        return False
