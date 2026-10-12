"""Red-track context and observed WHITE INNER edges, without metric guessing."""
import cv2
import numpy as np

from .config import load_config
from .lane import LaneDetector


def red_mask(image):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    return cv2.bitwise_or(cv2.inRange(hsv,(0,55,35),(15,255,255)),
                         cv2.inRange(hsv,(165,55,35),(179,255,255)))


def inner_edge(mask,x,y,direction):
    """Move from the observed paint run towards the driving corridor."""
    h,w=mask.shape;yi=int(round(y));xi=int(round(x))
    if not 0<=yi<h:return None
    row=(mask[max(0,yi-1):min(h,yi+2)]>0).mean(axis=0)>=.5
    indices=np.flatnonzero(row[max(0,xi-8):min(w,xi+9)])+max(0,xi-8)
    if not len(indices):return None
    cursor=int(min(indices,key=lambda q:abs(q-xi)))
    while 0<=cursor<w and row[cursor]:cursor+=direction
    if not 0<=cursor<w:return None
    return [float(cursor),float(y)]


class ConeRoadContext:
    def __init__(self,settings):
        self.s=settings;self.lane=LaneDetector(load_config(None).lane)

    def inspect(self,image,blue_mask):
        road=image.copy();road[blue_mask>0]=0
        lane,white=self.lane.detect(road);red=red_mask(image)
        left=[];right=[]
        if lane.valid:
            for lp,rp in zip(lane.left,lane.right):
                l=inner_edge(white,*lp,1);r=inner_edge(white,*rp,-1)
                if l and r and l[0]<r[0]:left.append(l);right.append(r)
        enough=lane.valid and len(left)>=max(6,len(lane.left)*.65)
        region=np.zeros(image.shape[:2],np.uint8)
        if enough:
            cv2.fillPoly(region,[np.asarray(left+right[::-1],np.int32)],255)
            support=(region>0)&(blue_mask==0)&(white==0)
        else:
            support=np.zeros(image.shape[:2],bool)
            support[int(image.shape[0]*.55):]=True
            support&=(blue_mask==0)&(white==0)
        ratio=float(np.mean(red[support]>0)) if support.any() else 0.
        red_supported=ratio>=self.s.get('scene',{}).get('red_ground_min_fraction',.15)
        valid=bool(enough and red_supported)
        return {'valid':valid,'white_lines_detected':bool(enough),'red_ground_supported':red_supported,
                'red_ground_fraction':ratio,'left_inner':left if enough else [],'right_inner':right if enough else [],
                'reason':'white_inner_edges_on_red_track' if valid else 'red_ground_not_confirmed' if enough else lane.reason if not lane.valid else 'white_inner_edges_incomplete',
                'boundary_policy':'whole_vehicle_inside_white_inner_edges',
                'metric_boundary_verified':False}

    @staticmethod
    def enrich(detections,image,context):
        red=red_mask(image);h,w=red.shape
        for row in detections:
            x1,y1,x2,y2=row['bbox_xyxy'];pad=max(3,round((x2-x1)*.25))
            patch=red[max(0,y2):min(h,y2+pad),max(0,x1-pad):min(w,x2+pad)]
            row['red_ground_near_base']=float(np.mean(patch>0)) if patch.size else 0.
            row['boundary_relation']='unknown'
            # Top-view/occluded bases have no verified contact point. Do not
            # pretend a bounding-box point is a calibrated ground position.
            if context['valid'] and row['ground_contact_usable']:
                left,right=context['left_inner'],context['right_inner'];x,y=row['foot_pixel']
                if left[0][1]<=y<=left[-1][1]:
                    lo=float(np.interp(y,[p[1] for p in left],[p[0] for p in left]))
                    hi=float(np.interp(y,[p[1] for p in right],[p[0] for p in right]))
                    row['boundary_relation']='inside' if lo<x<hi else 'outside'
            row['scene_supported']=row['red_ground_near_base']>=.15 and row['boundary_relation']=='inside'
        return detections
