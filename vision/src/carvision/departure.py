"""Blue start-board observation. This module has no motor output path."""
import math

import cv2
import numpy as np

PHRASE = '我发车啦'
CAMERAS = ('primary','secondary')
MAX_AGE_S = .6
QUALITY_RECOVERY_S = 1.
LOGIC_REVISION = 'departure_quality_recovery_v1'
# Preserve the reference's scaled 60000/10000 px thresholds at 640x480.
# Presence now requires one solid quadrilateral and its actual blue pixels;
# disconnected blobs and non-blue holes cannot supply the required area.
DEFAULT_MIN_BLUE_PERCENT = 60000 / (640 * 480) * 100
DEFAULT_CLEAR_BLUE_PERCENT = 10000 / (640 * 480) * 100
DEFAULT_CLEAR_CONFIRM_S = .6
MIN_CLEAR_CONFIRM_S = .5


def validate_blue_percent(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not .5<=value<=100:
        raise ValueError('最小蓝色面积占比须在0.5–100%之间')
    return float(value)


def detect_board(image,min_blue_percent=DEFAULT_MIN_BLUE_PERCENT,
                 clear_blue_percent=DEFAULT_CLEAR_BLUE_PERCENT):
    threshold=validate_blue_percent(min_blue_percent)/100
    clear_threshold=validate_blue_percent(clear_blue_percent)/100
    # The reference uses independent 60000/10000 px thresholds. For an
    # intentionally lower offline tuning threshold, keep the clear threshold
    # below it rather than making the configuration invalid.
    clear_threshold=min(clear_threshold, threshold*.5)
    h,w=image.shape[:2]
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(100,150,0),(124,255,255))
    kernel=np.ones((5,5),np.uint8)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
    # Do not close gaps between components: two adjacent blue objects cannot
    # combine their areas to become one qualifying board.
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # Geometry is evaluated on the cleaned contour, but area and solidity are
    # measured from the original HSV pixels. Contour/hull smoothing must never
    # invent the blue interior of a hollow frame.
    raw_blue=cv2.inRange(hsv,(100,150,0),(124,255,255))
    boxes=[];candidates=[];largest_pixels=0;largest_board_pixels=0
    for contour in contours:
        x,y,bw,bh=cv2.boundingRect(contour)
        if bw<6 or bh<6:continue
        center_x=(x+bw*.5)/w
        if center_x<.25 or center_x>.75:continue
        region=np.zeros((h,w),np.uint8)
        cv2.drawContours(region,[contour],-1,255,cv2.FILLED)
        region_pixels=cv2.countNonZero(region)
        real_pixels=cv2.countNonZero(cv2.bitwise_and(raw_blue,region))
        largest_pixels=max(largest_pixels,real_pixels)
        hull=cv2.convexHull(contour)
        polygon=cv2.approxPolyDP(hull,.02*cv2.arcLength(hull,True),True)
        hull_area=cv2.contourArea(hull)
        if len(polygon)!=4:
            # A small noisy corner may leave five hull vertices in actual
            # camera frames. A tightly filled rotated rectangle can recover
            # it; circles/ellipses cover only about 79% of that rectangle.
            rectangle=cv2.minAreaRect(contour)
            rectangle_area=rectangle[1][0]*rectangle[1][1]
            if rectangle_area>0 and cv2.contourArea(contour)/rectangle_area>=.87:
                polygon=np.rint(cv2.boxPoints(rectangle)).astype(np.int32).reshape(-1,1,2)
        shape_ok=(len(polygon)==4 and cv2.isContourConvex(polygon)
                  and hull_area>0 and cv2.contourArea(contour)/hull_area>=.92
                  and region_pixels>0 and real_pixels/region_pixels>=.85
                  and .35<=bw/bh<=2.2)
        # A narrow blue wall spanning both image sides is background, rather
        # than a front-facing board. A board filling the full near view remains
        # recognizable; no metric distance is inferred from this exception.
        spans_sides=x<=1 and x+bw>=w-1
        if spans_sides and bh<.85*h:shape_ok=False
        if not shape_ok:continue
        boxes.append([int(x),int(y),int(x+bw),int(y+bh)])
        candidates.append({'corners_px':polygon[:,0,:].tolist(),
                           'blue_pixels':int(real_pixels),
                           'fill_fraction':real_pixels/region_pixels})
        largest_board_pixels=max(largest_board_pixels,real_pixels)
    blue_fraction=largest_pixels/(w*h)
    board_fraction=largest_board_pixels/(w*h)
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    quality={'mean':float(gray.mean()),'std':float(gray.std()),
             'overexposed_fraction':float((gray>=250).mean())}
    healthy=quality['std']>=6 and quality['mean']>=20 and quality['overexposed_fraction']<.9
    state='present' if board_fraction>=threshold else 'absent' if blue_fraction<=clear_threshold and healthy else 'unknown'
    reason={'present':'前方有大块蓝色矩形挡板','absent':'画面有效，未见挡板或大块不明确蓝色区域',
            'unknown':'画面遮挡、过暗、过曝或蓝色形状不明确，不能认定移开'}[state]
    return {'state':state,'reason':reason,'boxes':boxes,'largest_blue_fraction':blue_fraction,
            'blue_pixel_fraction':blue_fraction,'board_blue_fraction':board_fraction,'board_candidates':candidates,
            'detector_revision':'solid_quadrilateral_pixels_v1',
            'min_blue_percent':float(min_blue_percent),'clear_blue_percent':float(clear_blue_percent),
            'unknown_cause':'blue_area' if state=='unknown' and healthy else 'image_quality' if not healthy else None,
            'scene_valid':bool(healthy or boxes),'dimensions_cm':{'width':40,'height':50},
            'image_quality':quality,
            'metric_distance_available':False}


class DepartureSequence:
    """Fresh images must show present first, then sustained clear in both views."""
    def __init__(self,clear_s=DEFAULT_CLEAR_CONFIRM_S):
        if isinstance(clear_s,bool) or not isinstance(clear_s,(int,float)) or not math.isfinite(clear_s) or not MIN_CLEAR_CONFIRM_S<=clear_s<=3:
            raise ValueError('移开确认时间须在0.5–3秒之间')
        self.clear_s=float(clear_s);self.phase='wait_board';self.reason='先将蓝色挡板放在车前，等待确认存在'
        self.ids={};self.times={};self.present={};self.clear_since=None;self.clear_frames=0
        self.seen_cameras=[];self.clear_elapsed=0.;self.last_pair=None
        self.quality_since=None;self.last_update=None;self.input_issues={};self.input_details={}

    def invalidate(self,reason=None):
        self.present={};self.clear_since=None;self.clear_frames=0;self.clear_elapsed=0.;self.last_pair=None
        self.quality_since=None
        # Reconnection must see the board again; a missing camera is not removal.
        self.phase='wait_board';self.seen_cameras=[]
        self.reason=reason or '图像中断或无效；需重新看到挡板，不能据此播报发车'

    def diagnostics(self):
        return {'logic_revision':LOGIC_REVISION,'quality_grace_s':QUALITY_RECOVERY_S,
                'quality_wait_s':None if self.quality_since is None else max(0.,self.last_update-self.quality_since),
                'input_issues':dict(self.input_issues),
                'cameras':{c:dict(row) for c,row in self.input_details.items()},
                'clear_frames':self.clear_frames,'clear_elapsed_s':self.clear_elapsed,
                'confirmed_cameras':list(self.seen_cameras)}

    def update(self,observations,now):
        if self.phase=='complete':return False
        if type(now) not in (int,float) or not math.isfinite(now):
            self.invalidate('发车时钟无效；需重新看到挡板');return False
        self.last_update=now;self.input_issues={};self.input_details={}
        quality_only=True
        for camera in CAMERAS:
            row=observations.get(camera,{})
            t=row.get('captured_s');fid=row.get('frame_id')
            stamp_ok=type(t) in (int,float) and math.isfinite(t)
            age=now-t if stamp_ok else None
            self.input_details[camera]={'state':row.get('state'),'frame_id':fid if type(fid) is int else None,
                'age_ms':round(age*1000,2) if age is not None else None,
                'unknown_cause':row.get('unknown_cause'),'image_quality':row.get('image_quality')}
            if type(fid) is not int or not stamp_ok:issue='missing_frame'
            elif not 0<=age<=MAX_AGE_S:issue='stale_or_future_frame'
            elif row.get('state') not in ('present','absent','unknown'):issue='invalid_observation'
            elif row.get('state')=='unknown' and row.get('scene_valid') is not True:
                issue='image_quality' if row.get('unknown_cause')=='image_quality' else 'invalid_scene'
            else:continue
            self.input_issues[camera]=issue
            quality_only=quality_only and issue=='image_quality'
        if self.input_issues:
            self.present={};self.clear_since=None;self.clear_frames=0;self.clear_elapsed=0.
            names='、'.join('第一路' if c=='primary' else '第二路' for c in self.input_issues)
            if quality_only and self.phase=='wait_removed':
                if self.quality_since is None:self.quality_since=now
                if now-self.quality_since<QUALITY_RECOVERY_S:
                    self.reason=names+'画面质量暂不足；保留已确认挡板，暂停移开计时'
                    return False
                self.invalidate(names+'画面质量持续不足；需重新看到挡板')
            else:
                detail='、'.join(c+':'+v for c,v in self.input_issues.items())
                self.invalidate('发车画面不可用（'+detail+'）；需重新看到挡板')
            return False
        recovered_quality=self.quality_since is not None
        if recovered_quality:
            if now-self.quality_since>=QUALITY_RECOVERY_S:
                self.invalidate('画面质量恢复过晚；需重新看到挡板');return False
            # Never reuse any clear duration accumulated before the bad image.
            self.quality_since=None;self.clear_since=None;self.clear_frames=0;self.clear_elapsed=0.
        # Process only pairs where both cameras supplied a new frame. Repeat
        # HTTP/cache reads never count as multiple removal observations.
        changed=all(observations[c]['frame_id']!=self.ids.get(c) for c in CAMERAS)
        if not changed:return False
        stamp=min(observations[c]['captured_s'] for c in CAMERAS)
        if any(observations[c]['captured_s']<=self.times.get(c,-math.inf) for c in CAMERAS):
            self.invalidate();return False
        if self.last_pair is not None and stamp-self.last_pair>MAX_AGE_S and not recovered_quality:
            self.input_issues={'pair':'capture_gap'}
            self.invalidate('两路新帧间隔过长；需重新看到挡板');return False
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
