"""Range-limited ground measurements and local motion for parking diagnostics.

The supplied camera pose is operator-confirmed. This does not verify motion
error, hidden boundaries or formal parking. Pixels outside the calibration
polygon are never projected.
"""
import math
import cv2
import numpy as np


class ParkingGroundView:
    def __init__(self,ground):
        if (not isinstance(ground,dict) or ground.get('coordinate_frame')!='rear_axle_x_right_y_forward'
                or not (ground.get('verified') is True or ground.get('historical_verified') is True)):
            raise ValueError('停车需要已有验收记录的地面映射')
        self.h=np.asarray(ground['homography'],np.float64)
        self.poly=np.asarray(ground['valid_polygon_px'],np.float32)
        if (ground.get('image_size')!=[480,360] or self.h.shape!=(3,3)
                or self.poly.ndim!=2 or self.poly.shape[1]!=2 or len(self.poly)<3
                or not np.isfinite(self.h).all() or not np.isfinite(self.poly).all()
                or abs(np.linalg.det(self.h))<1e-12 or not cv2.isContourConvex(self.poly)):
            raise ValueError('停车地面映射或有效范围无效')
        if type(ground.get('max_error_m')) not in (int,float):raise ValueError('停车地面映射误差无效')
        self.error=float(ground['max_error_m'])
        if not 0<self.error<=.05:raise ValueError('停车地面映射误差无效')

    def point(self,p):
        if cv2.pointPolygonTest(self.poly,tuple(map(float,p)),False)<0:return None
        v=self.h@np.array([*p,1.])
        if abs(v[2])<1e-6:return None
        q=v[:2]/v[2]
        return q if np.isfinite(q).all() else None

    def segment(self,a,b):
        points=[v for p in np.linspace(a,b,60) if (v:=self.point(p)) is not None]
        if len(points)<8:return None
        points=np.asarray(points)
        if np.linalg.norm(points[-1]-points[0])<.12:return None
        return points

    def entrance_line(self,q):
        """Measure the visible part of the entrance without requiring its centre."""
        points=self.segment(q[0],q[1])
        if points is None:return None
        vector=points[-1]-points[0];vector/=np.linalg.norm(vector)
        if vector[0]<0:vector=-vector
        normal=np.array([-vector[1],vector[0]])
        if normal[1]<.7:return None
        distance=float(np.median(points@normal))
        return {'normal':normal.tolist(),'distance_m':distance,
                'visible_span_m':float(np.linalg.norm(points[-1]-points[0])),
                'error_estimate_m':self.error,'physical_pose_verified':False}

    def entrance(self,q):
        line=self.entrance_line(q)
        if line is None:return None
        # A bay centre may be used only when that actual pixel is in range.
        center=self.point((q[0]+q[1])/2)
        if center is None:return None
        ends=[self.point(p) for p in q[:2]]
        if all(p is not None for p in ends):center=np.mean(ends,axis=0)
        return {**line,'center_m':center.tolist(),'source':'current_calibrated_entrance'}


class ParkingGroundMotion:
    """Rigid ground motion within the same polygon; never derives speed from PWM."""
    def __init__(self,ground):
        self.view=ParkingGroundView(ground);self.previous=None
        self.references=[];self.retry_frames=0

    def update(self,image,frame_id,captured_s,ground_mask):
        grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);mask=np.zeros_like(grey)
        cv2.fillConvexPoly(mask,self.view.poly.astype(np.int32),255)
        mask=cv2.bitwise_and(mask,ground_mask)
        points=cv2.goodFeaturesToTrack(grey,120,.005,4,mask=mask)
        old=self.previous
        result=self._measure(old,grey,mask,frame_id,captured_s)
        if not result['valid']:self.retry_frames=3
        alternatives=[]
        if self.retry_frames:
            # Directly match earlier images to this image. Never concatenate
            # a missing transform or infer displacement from throttle.
            for reference in reversed(self.references[:-1]):
                if not 0<captured_s-reference[3]<=.5:continue
                recovered=self._measure(reference,grey,mask,frame_id,captured_s)
                if recovered['valid']:alternatives.append(recovered)
            self.retry_frames-=1
        self.previous=(grey,points,frame_id,captured_s)
        self.references=(self.references+[self.previous])[-4:]
        return {**result,'relocalizations':alternatives}

    def _measure(self,old,grey,mask,frame_id,captured_s):
        bad={'valid':False,'frame_id':frame_id,'captured_s':captured_s,'physical_pose_verified':False}
        if old is None:return {**bad,'reason':'collecting_ground_motion'}
        before_grey,old_points,old_id,old_t=old;dt=captured_s-old_t
        if old_points is None or len(old_points)<12 or not 0<dt<=.5 or frame_id<=old_id:
            return {**bad,'reason':'ground_motion_gap'}
        moved,ok,err=cv2.calcOpticalFlowPyrLK(before_grey,grey,old_points,None,winSize=(21,21),maxLevel=3)
        if moved is None:return {**bad,'reason':'ground_motion_missing'}
        back,back_ok,_=cv2.calcOpticalFlowPyrLK(grey,before_grey,moved,None,winSize=(21,21),maxLevel=3)
        if back is None:return {**bad,'reason':'ground_motion_missing'}
        pairs=[]
        for p,q,reverse,good,reverse_good,error in zip(old_points[:,0],moved[:,0],back[:,0],ok[:,0],back_ok[:,0],err[:,0]):
            x,y=np.rint(q).astype(int)
            if not good or not reverse_good or error>25 or np.linalg.norm(reverse-p)>1.5:continue
            if not (0<=x<480 and 0<=y<360 and mask[y,x]):continue
            a,b=self.view.point(p),self.view.point(q)
            if a is not None and b is not None:pairs.append((a,b))
        if len(pairs)<12:return {**bad,'reason':'ground_motion_texture_missing'}
        before,after=(np.ascontiguousarray(v,dtype=np.float64) for v in np.asarray(pairs).transpose(1,0,2))
        affine,inliers=cv2.estimateAffinePartial2D(before,after,method=cv2.RANSAC,ransacReprojThreshold=.02,maxIters=200)
        if affine is None or inliers is None or np.mean(inliers)<.75:return {**bad,'reason':'ground_motion_fit'}
        scale=float(np.hypot(affine[0,0],affine[1,0]));keep=inliers[:,0]>0
        # Refit a rigid transform: retaining scaled-fit translation while
        # removing its scale biases the entrance position on every frame.
        a,b=before[keep],after[keep]
        u,_,vt=np.linalg.svd((a-a.mean(axis=0)).T@(b-b.mean(axis=0)))
        rotation=vt.T@u.T
        if np.linalg.det(rotation)<0:return {**bad,'reason':'ground_motion_reflection'}
        translation=b.mean(axis=0)-rotation@a.mean(axis=0)
        rms=float(np.sqrt(np.mean(np.sum((a@rotation.T+translation-b)**2,axis=1))))
        delta=-rotation.T@translation
        yaw=math.atan2(rotation[1,0],rotation[0,0])
        if not .97<=scale<=1.03 or rms>.02 or np.linalg.norm(delta)/dt>1.5 or abs(yaw/dt)>1.5:
            return {**bad,'reason':'ground_motion_inconsistent'}
        rigid=np.column_stack((rotation,translation))
        return {**bad,'valid':True,'previous_frame_id':old_id,'previous_captured_s':old_t,
                'transform':rigid.tolist(),'translation_m':float(np.linalg.norm(delta)),
                'forward_speed_mps':float(delta[1]/dt),'rms_m':rms,'tracked_points':int(keep.sum()),
                'source':'bounded_calibrated_ground_motion'}
