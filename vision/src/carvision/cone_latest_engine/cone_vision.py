"""Blue-cone candidates and per-camera tracking, without actuator imports.

A large/clipped blue shape is retained as an unresolved obstruction rather
than silently disappearing or being counted as a passed cone.
"""
import math

import cv2
import numpy as np
from .cone_timing import FRAME_MAX_AGE_S


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
            lower=float(np.mean(widths[min(base,round((base+1)*.8)):base+1]))
            hull=float(cv2.contourArea(cv2.convexHull(contour)))
            shoulder=float(np.median(widths[round(bh*.15):max(round(bh*.15)+1,round(bh*.30))]))/bw
            middle=float(np.median(widths[round(bh*.30):max(round(bh*.30)+1,round(bh*.50))]))/bw
            # A rectangular blue board with a glare-cut top can mimic a taper
            # near its first few rows. Require taper across the whole silhouette.
            tapered=(base>=bh*.65 and shoulder<=.70 and middle<=.85
                     and lower>=max(1,upper)*1.5 and .85<=bh/bw<=4
                     and .28<=area/(bw*bh)<=.85 and hull>0 and area/hull>=.75)
            perimeter=cv2.arcLength(contour,True)
            rounded=(.75<=bh/bw<=1.35 and .67<=area/(bw*bh)<=.9 and hull>0 and area/hull>=.9
                     and perimeter>0 and 4*math.pi*area/(perimeter*perimeter)>=.72)
            kind='cone_candidate' if tapered and not clipped else 'unresolved_blue'
            # Small non-cone colored patches are not used as road obstacles;
            # close/large/edge-intersecting patches remain visible to the user.
            if kind=='unresolved_blue' and not (clipped or area/(w*h)>=.012 or y+bh>=h*.8):continue
            bottom=np.flatnonzero(local[max(0,base-1):min(bh,base+2)].any(axis=0))
            foot_x=x+float(np.mean(bottom)) if len(bottom) else x+bw/2
            rows.append({'kind':kind,'bbox_xyxy':[x,y,x+bw,y+bh],
                         'foot_pixel':[foot_x,float(y+base)],'clipped':clipped,
                         'shape_evidence':'tapered' if tapered else 'round_top_view' if rounded else 'unresolved',
                         'shape_profile':{'shoulder_width_fraction':shoulder,'middle_width_fraction':middle},
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

    def reset_stream(self):
        """Forget image matching after a restart, but never recycle physical aliases."""
        self.tracks.clear();self.last_id=self.last_t=None

    @staticmethod
    def _match_cost(track,detection,captured_s):
        """Associate current boxes through a bounded image-motion prediction."""
        a=np.asarray(track['bbox_xyxy'],float);b=np.asarray(detection['bbox_xyxy'],float)
        dt=captured_s-track['seen_s']
        size=np.maximum(a[2:]-a[:2],1.);new_size=b[2:]-b[:2]
        ratio=new_size/size
        if np.any(ratio<.45) or np.any(ratio>2.3):return None
        # A clipped near cone may lose its taper, but a remote blue board must
        # not inherit a cone's confirmed identity.
        if track['kind']!=detection['kind'] and not (
                track.get('confirmed') and detection.get('clipped') and b[3]>=a[3]-10):return None
        velocity=np.asarray(track.get('velocity_px_s',[0.,0.]),float)
        offset=np.clip(velocity*dt,-60.,60.)
        predicted=a+np.tile(offset,2)
        center=(b[:2]+b[2:])/2;expected=(predicted[:2]+predicted[2:])/2
        scale=np.maximum(np.maximum(size,new_size),[24.,28.])
        residual=np.abs(center-expected)/scale
        if residual[0]>1.5 or residual[1]>1.15:return None
        cost=float(np.linalg.norm(residual)+.3*np.linalg.norm(np.log(ratio))-.4*iou(predicted,b))
        return cost if cost<1.55 else None

    def update(self,detections,frame_id,captured_s):
        if (type(frame_id) is not int or not isinstance(captured_s,(int,float)) or not math.isfinite(captured_s)
                or self.last_id is not None and frame_id<=self.last_id
                or self.last_t is not None and captured_s<=self.last_t):
            raise ValueError('tracking requires distinct ordered camera frames')
        if self.last_t is not None and captured_s-self.last_t>FRAME_MAX_AGE_S:
            self.tracks.clear()
        self.last_id,self.last_t=frame_id,captured_s
        # Resolve only mutual, unambiguous best matches. Processing near boxes
        # first must not let one detection steal another cone's identity.
        candidates={}
        for index,detection in enumerate(detections):
            costs=[]
            for key,track in self.tracks.items():
                if not 0<captured_s-track['seen_s']<=min(.5,self.s['track_timeout_s']):continue
                cost=self._match_cost(track,detection,captured_s)
                if cost is not None:costs.append((cost,key))
            candidates[index]=sorted(costs)
        assignments={}
        for index,costs in candidates.items():
            if not costs or len(costs)>1 and costs[1][0]-costs[0][0]<.20:continue
            cost,key=costs[0]
            competitors=sorted((c,i) for i,values in candidates.items() for c,k in values if k==key)
            if competitors[0][1]!=index or len(competitors)>1 and competitors[1][0]-cost<.20:continue
            assignments[index]=key
        used=set();output=[]
        for index,detection in enumerate(detections):
            key=assignments.get(index)
            if key is None:
                key=f'{self.camera}-{self.next_id}';self.next_id+=1
                track={'id':key,'first_s':captured_s,'hits':0,'confirmed':False}
            else:
                track=dict(self.tracks[key])
                old=np.asarray(track['bbox_xyxy'],float);new=np.asarray(detection['bbox_xyxy'],float)
                velocity=((new[:2]+new[2:])-(old[:2]+old[2:]))/(2*(captured_s-track['seen_s']))
                track['velocity_px_s']=np.clip(.75*velocity+.25*np.asarray(track.get('velocity_px_s',[0.,0.])), -600.,600.).tolist()
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
