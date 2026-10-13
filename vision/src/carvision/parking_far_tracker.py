"""Track a previously observed far tape edge in fresh camera pixels only."""
import cv2
import numpy as np

from .parking_bay import contrast_markings


class ParkingFarTracker:
    def __init__(self):
        self.grey=None
        self.capture=self.frame=None
        self.tracks={}

    @staticmethod
    def _features(grey,edge):
        mask=np.zeros_like(grey)
        cv2.line(mask,tuple(np.rint(edge[0]).astype(int)),tuple(np.rint(edge[1]).astype(int)),255,14)
        return cv2.goodFeaturesToTrack(grey,40,.005,2,mask=mask,blockSize=3)

    @staticmethod
    def _edge(value,shape):
        try:edge=np.asarray(value,dtype=np.float32)
        except (TypeError,ValueError):return None
        h,w=shape
        if (edge.shape!=(2,2) or not np.isfinite(edge).all() or edge[1,0]-edge[0,0]<20
                or np.any(edge[:,0]<0) or np.any(edge[:,0]>=w)
                or np.any(edge[:,1]<h*.28) or np.any(edge[:,1]>=h-2)
                or abs(float(edge[1,1]-edge[0,1]))>h*.18):return None
        return edge

    def update(self,image,observation,frame_id,captured_s,settings):
        grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        edges,_=contrast_markings(hsv,grey,settings)
        edges=cv2.bitwise_or(edges,cv2.inRange(hsv,tuple(settings['yellow_lower']),tuple(settings['yellow_upper'])))
        support=cv2.dilate(edges,np.ones((11,11),np.uint8))
        valid=self.capture is not None and 0<captured_s-self.capture<=.25 and frame_id>self.frame and grey.shape==self.grey.shape
        tracks={};results=[]
        if valid:
            for side,old in self.tracks.items():
                pts=old['points']
                if pts is None or len(pts)<6:continue
                nxt,ok,err=cv2.calcOpticalFlowPyrLK(self.grey,grey,pts,None,winSize=(21,21),maxLevel=3)
                if nxt is None:continue
                back,back_ok,_=cv2.calcOpticalFlowPyrLK(grey,self.grey,nxt,None,winSize=(21,21),maxLevel=3)
                if back is None:continue
                good=(ok.ravel()>0)&(back_ok.ravel()>0)&(err.ravel()<30)&(np.linalg.norm(back-pts,axis=2).ravel()<1.5)
                before=pts[good].reshape(-1,2);after=nxt[good].reshape(-1,2)
                if len(after)<6:continue
                matrix,inliers=cv2.estimateAffinePartial2D(before,after,method=cv2.RANSAC,ransacReprojThreshold=2.)
                if matrix is None or inliers is None or int(inliers.sum())<6:continue
                scale=float(np.hypot(matrix[0,0],matrix[1,0]))
                if not .8<=scale<=1.3 or abs(float(np.arctan2(matrix[1,0],matrix[0,0])))>.22:continue
                edge=self._edge(cv2.transform(old['edge'][None,:,:],matrix)[0],grey.shape)
                if edge is None or np.max(np.linalg.norm(edge-old['edge'],axis=1))>50:continue
                samples=np.rint(np.linspace(edge[0],edge[1],40)).astype(int)
                fraction=float(np.mean(support[samples[:,1],samples[:,0]]>0))
                if fraction<.65:continue
                points=self._features(grey,edge)
                tracks[side]={**old,'edge':edge,'points':points}
                results.append({'id':side,'source':'optical_flow_far_edge','edge_px':edge.tolist(),
                    'frame_id':frame_id,'captured_s':captured_s,'tracked_points':int(inliers.sum()),
                    'marking_support':fraction,'seed_frame_id':old['seed_frame_id'],
                    'seed_age_s':captured_s-old['seed_capture'],'occupancy_verified':False})
        # Only actual detector corners can seed or relabel an edge. Flow never
        # creates a new bay or fills in a missing entrance/side boundary.
        for side in ('left','right'):
            candidates=[v for v in observation.get('slots',[]) if v.get('id')==side and v.get('availability')!='blocked' and v.get('corners_px')]
            if not candidates:candidates=[v for v in observation.get('entry_candidates',[]) if v.get('id')==side and v.get('availability')=='clear']
            if len(candidates)!=1:continue
            q=np.asarray(candidates[0].get('corners_px'),dtype=np.float32)
            if q.shape!=(4,2) or not np.isfinite(q).all():continue
            edge=self._edge(q[[3,2]],grey.shape)
            if edge is None:continue
            points=self._features(grey,edge)
            if points is not None and len(points)>=6:
                tracks[side]={'edge':edge,'points':points,'seed_frame_id':frame_id,'seed_capture':captured_s}
        self.grey,self.frame,self.capture,self.tracks=grey,frame_id,captured_s,tracks
        return results
