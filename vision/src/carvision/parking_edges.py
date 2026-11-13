"""Track an acquired bay, then refit each boundary to current tape pixels.

Optical flow only places the search bands. Every returned edge still requires
current marking support, free interior and consistent bay geometry.
"""
import cv2
import numpy as np

from .parking_limits import FRAME_MAX_AGE_S
from .parking_inner_edges import inner_quad,associated_quad


def support(mask,a,b):
    points=np.rint(np.linspace(a,b,max(20,int(np.linalg.norm(b-a))))).astype(int)
    h,w=mask.shape
    if (points<0).any() or (points[:,0]>=w).any() or (points[:,1]>=h).any():return 0.
    return float(np.mean(mask[points[:,1],points[:,0]]>0))


def valid_quad(q,shape):
    h,w=shape
    return (q.shape==(4,2) and np.isfinite(q).all() and (q>=2).all()
        and (q[:,0]<w-2).all() and (q[:,1]<h-2).all()
        and cv2.isContourConvex(q) and 600<cv2.contourArea(q)<w*h*.65
        and min(q[1,0]-q[0,0],q[2,0]-q[3,0])>25
        and min(q[0,1]-q[3,1],q[1,1]-q[2,1])>12)


class ParkingBoundaryTracker:
    def __init__(self):
        self.grey=None;self.capture=self.frame=None;self.tracks={}

    @staticmethod
    def features(grey,q):
        mask=np.zeros_like(grey)
        cv2.polylines(mask,[np.rint(q).astype(np.int32)],True,255,12)
        return cv2.goodFeaturesToTrack(grey,90,.005,3,mask=mask,blockSize=3)

    def update(self,image,observation,frame_id,captured_s,markings,ground,blue):
        grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        fresh=(self.capture is not None and 0<captured_s-self.capture<=FRAME_MAX_AGE_S
               and frame_id>self.frame and self.grey.shape==grey.shape)
        expanded=cv2.dilate(markings,np.ones((7,7),np.uint8))
        tracks={};results=[]
        if fresh:
            for side,old in self.tracks.items():
                pts=old['points']
                if pts is None or len(pts)<12:continue
                nxt,ok,err=cv2.calcOpticalFlowPyrLK(self.grey,grey,pts,None,winSize=(21,21),maxLevel=3)
                if nxt is None:continue
                back,back_ok,_=cv2.calcOpticalFlowPyrLK(grey,self.grey,nxt,None,winSize=(21,21),maxLevel=3)
                if back is None:continue
                good=(ok.ravel()>0)&(back_ok.ravel()>0)&(err.ravel()<30)&(np.linalg.norm(back-pts,axis=2).ravel()<1.5)
                before=pts[good].reshape(-1,2);after=nxt[good].reshape(-1,2)
                if len(after)<12:continue
                matrix,inliers=cv2.estimateAffine2D(before,after,method=cv2.RANSAC,ransacReprojThreshold=2.)
                plausible=matrix is not None and inliers is not None and int(inliers.sum())>=12
                if plausible:
                    scales=np.linalg.svd(matrix[:,:2],compute_uv=False)
                    plausible=scales.min()>=.8 and scales.max()<=1.3 and np.linalg.det(matrix[:,:2])>0
                if not plausible:
                    # A poor affine fit cannot erase visible tape. Use only
                    # bounded median pixel displacement to place a new search;
                    # every accepted edge still has to fit this frame's pixels.
                    shift=np.median(after-before,axis=0)
                    inliers=(np.linalg.norm(after-before-shift,axis=1)<12).astype(np.uint8)[:,None]
                    if np.linalg.norm(shift)>35 or int(inliers.sum())<12:continue
                    matrix=np.array([[1.,0.,shift[0]],[0.,1.,shift[1]]])
                predicted=cv2.transform(old['quad'][None,:,:],matrix)[0]
                if not valid_quad(predicted,grey.shape) or np.max(np.linalg.norm(predicted-old['quad'],axis=1))>45:continue
                dt=captured_s-self.capture
                near_at=old.get('near_captured_s',self.capture)
                entrance=old.get('entrance_edge',old['quad'][:2])
                entrance=cv2.transform(np.asarray(entrance)[None,:,:],matrix)[0]
                near_prediction=predicted.copy();near_prediction[:2]=entrance
                q=None;radius=None
                # Even a three-edge track retries the real entrance every frame.
                # Its separate reference must not be replaced by a floor cut.
                if captured_s-near_at<=FRAME_MAX_AGE_S and valid_quad(near_prediction,grey.shape):
                    q,radius=associated_quad(markings,ground,near_prediction,dt)
                near_observed=q is not None
                if q is None:
                    # A contour can fragment before the entrance leaves view.
                    # Use only the three currently fitted edges and a floor cut.
                    cut=predicted.copy()
                    cut[:2,1]=min(predicted[:2,1])-5.
                    q,radius=associated_quad(markings,ground,cut,dt,near_observed=False)
                if q is None or not valid_quad(q,grey.shape):continue
                evidence=[support(expanded,a,b) for a,b in zip(q,np.roll(q,-1,axis=0))]
                if min(evidence if near_observed else evidence[1:])<.72:continue
                interior=np.zeros_like(grey);cv2.fillConvexPoly(interior,np.rint(q).astype(np.int32),255)
                interior=cv2.erode(interior,np.ones((9,9),np.uint8))>0
                if interior.sum()<200:continue
                floor=float(np.mean(ground[interior]>0));occupied=float(np.mean(blue[interior]>0))
                if floor<.9 or occupied>.003:continue
                points=self.features(grey,q)
                tracks[side]={'quad':q,'points':points,'near_boundary_observed':near_observed,
                    'entrance_edge':q[:2].copy() if near_observed else entrance,
                    'near_captured_s':captured_s if near_observed else near_at}
                results.append({'id':side,'corners_px':q.tolist(),'availability':'clear',
                    'source':'current_tracked_bay_edges','frame_id':frame_id,'captured_s':captured_s,
                    'edge_support':evidence,'tracked_points':int(inliers.sum()),'ground_fraction':floor,
                    'blue_inside_fraction':occupied,'near_boundary_observed':near_observed,'boundary_reference':'inner_edge',
                    'far_boundary_observed':True,'occupancy_verified':False,
                    'search_radius_px':radius,'entrance_reacquired':near_observed and not old.get('near_boundary_observed',True)})
        # Detector seeds need explicit free-space evidence. A blocked/unknown
        # observation invalidates a track instead of being masked by tracking.
        for side in ('left','right'):
            slots=[v for v in observation.get('slots',[]) if v.get('id')==side]
            if any(v.get('availability')!='clear' for v in slots):
                tracks.pop(side,None);results=[v for v in results if v['id']!=side];continue
            candidates=slots or [v for v in observation.get('entry_candidates',[])
                                if v.get('id')==side and v.get('availability')=='clear']
            if len(candidates)!=1:continue
            q=np.asarray(candidates[0].get('corners_px'),np.float32)
            if not valid_quad(q,grey.shape):continue
            q=inner_quad(markings,ground,q)
            if q is None or not valid_quad(q,grey.shape):continue
            # Do not reset an established, currently validated track to a
            # fluctuating contour, especially a narrow fragment of the bay.
            if side in tracks:
                if tracks[side]['near_boundary_observed']:continue
                previous=tracks[side]['quad']
                if np.max(np.linalg.norm(previous[2:]-q[2:],axis=1))>35:continue
                # A current full detector observation can restore the entrance
                # of an associated partial track, rather than being ignored.
                results=[v for v in results if v['id']!=side]
            tracks[side]={'quad':q.copy(),'points':self.features(grey,q),'near_boundary_observed':True,
                          'entrance_edge':q[:2].copy(),'near_captured_s':captured_s}
            # Normalize detector output too, so first acquisition and tracking
            # use the same floor-side coordinate convention.
            candidates[0]['corners_px']=q.tolist()
            candidates[0]['boundary_reference']='inner_edge'
        self.grey,self.capture,self.frame,self.tracks=grey,captured_s,frame_id,tracks
        return results
