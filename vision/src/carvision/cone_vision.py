"""Blue-cone candidates and per-camera tracking, without actuator imports.

A large/clipped blue shape is retained as an unresolved obstruction rather
than silently disappearing or being counted as a passed cone.
"""
import math

import cv2
import numpy as np


def iou(a,b):
    area=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area
    return area/union if union>0 else 0.


class BlueConeDetector:
    def __init__(self, settings):
        self.lower=np.asarray(settings['blue_hsv_lower'],np.uint8)
        self.upper=np.asarray(settings['blue_hsv_upper'],np.uint8)

    def detect(self,image):
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,self.lower,self.upper)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        h,w=mask.shape
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        rows=[]
        for contour in contours:
            x,y,bw,bh=cv2.boundingRect(contour);area=float(cv2.contourArea(contour))
            if bw<max(8,w*.02) or bh<max(12,h*.035) or area<max(25,w*h*.00015) or y+bh<h*.25:continue
            clipped=x<=1 or y<=1 or x+bw>=w-1 or y+bh>=h-1
            local=np.zeros((bh,bw),np.uint8)
            cv2.drawContours(local,[contour-np.array([[[x,y]]],np.int32)],-1,1,-1)
            widths=np.count_nonzero(local,axis=1)
            broad=np.flatnonzero(widths>=widths.max()*.9)
            base=int(np.median(broad))
            start=max(0,round((base+1)*.1));end=max(start+1,round((base+1)*.3))
            upper=float(np.mean(widths[start:end]))
            lower=float(np.mean(widths[round((base+1)*.8):base+1]))
            hull=float(cv2.contourArea(cv2.convexHull(contour)))
            tapered=(base>=bh*.55 and lower>=max(1,upper)*1.5 and .7<=bh/bw<=4
                     and .28<=area/(bw*bh)<=.85 and hull>0 and area/hull>=.75)
            perimeter=cv2.arcLength(contour,True)
            rounded=(.75<=bh/bw<=1.35 and .67<=area/(bw*bh)<=.9 and hull>0 and area/hull>=.9
                     and perimeter>0 and 4*math.pi*area/(perimeter*perimeter)>=.72)
            kind='cone_candidate' if (tapered or rounded) and not clipped else 'unresolved_blue'
            # Small non-cone colored patches are not used as road obstacles;
            # close/large/edge-intersecting patches remain visible to the user.
            if kind=='unresolved_blue' and not (clipped or area/(w*h)>=.012 or y+bh>=h*.8):continue
            bottom=np.flatnonzero(local[max(0,base-1):min(bh,base+2)].any(axis=0))
            foot_x=x+float(np.mean(bottom)) if len(bottom) else x+bw/2
            rows.append({'kind':kind,'bbox_xyxy':[x,y,x+bw,y+bh],
                         'foot_pixel':[foot_x,float(y+base)],'clipped':clipped,
                         'shape_evidence':'tapered' if tapered else 'round_top_view' if rounded else 'unresolved',
                         'ground_contact_usable':kind=='cone_candidate' and tapered and not clipped,
                         'area_fraction':area/(w*h),'position_in_image':
                            'left' if foot_x<w*.4 else 'right' if foot_x>w*.6 else 'center'})
        rows.sort(key=lambda r:r['bbox_xyxy'][3],reverse=True)
        return rows,mask


class ImageConeTracker:
    """Per-view IDs only. Two camera IDs never imply two physical cones."""
    def __init__(self,camera,settings):
        self.camera,self.s=camera,settings
        self.tracks={};self.next_id=1;self.last_id=self.last_t=None

    def update(self,detections,frame_id,captured_s):
        if (type(frame_id) is not int or not isinstance(captured_s,(int,float)) or not math.isfinite(captured_s)
                or self.last_id is not None and frame_id<=self.last_id
                or self.last_t is not None and captured_s<=self.last_t):
            raise ValueError('tracking requires distinct ordered camera frames')
        if self.last_t is not None and captured_s-self.last_t>self.s['max_frame_age_s']:
            self.tracks.clear()
        self.last_id,self.last_t=frame_id,captured_s
        used=set();output=[]
        for detection in detections:
            matches=[(iou(t['bbox_xyxy'],detection['bbox_xyxy']),key) for key,t in self.tracks.items()
                     if key not in used and captured_s-t['seen_s']<=self.s['track_timeout_s']]
            score,key=max(matches,default=(0,None))
            if score<.15:
                key=f'{self.camera}-{self.next_id}';self.next_id+=1
                track={'id':key,'first_s':captured_s,'hits':0,'confirmed':False}
            else:track=dict(self.tracks[key])
            used.add(key)
            if detection['kind']=='cone_candidate':track['hits']+=1
            track.update(detection,seen_s=captured_s,visible=True)
            track['confirmed']=(track['confirmed'] or detection['kind']=='cone_candidate'
                and track['hits']>=self.s['confirm_frames'] and captured_s-track['first_s']>=self.s['confirm_s'])
            # A clipped continuation keeps its identity but has no new metric
            # contact point. It is never interpreted as already passed.
            self.tracks[key]=track;output.append(dict(track))
        for key,track in list(self.tracks.items()):
            if key in used:continue
            if captured_s-track['seen_s']>self.s['track_timeout_s']:
                del self.tracks[key]
            else:output.append({**track,'visible':False,'ground_contact_usable':False})
        return output
