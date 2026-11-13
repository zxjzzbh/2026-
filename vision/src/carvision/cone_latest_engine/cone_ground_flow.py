"""Fresh ground-texture motion, with historical projection scope kept explicit.

No command-derived odometry and no drive output. Texture loss is an invalid
measurement, not positive travel and not a reason to stop the owning controller.
"""
import cv2
import numpy as np
import math

from .autonomy_profile import GroundProjection
from .brake_parking import numeric
from ..runtime_timing import VISION_MAX_AGE_S


def valid_motion(motion, now):
    """Validate at producer AND controller boundaries; malformed flow isn't travel."""
    keys=('captured_s','previous_s','dt_s','delta_x_m','delta_y_m','yaw_rad','speed_mps','rms_m','points')
    if not motion.get('valid') or not numeric(now) or any(not numeric(motion.get(k)) for k in keys):
        return False
    dt=motion['dt_s']
    return (0<=now-motion['captured_s']<=VISION_MAX_AGE_S and .01<=dt<=VISION_MAX_AGE_S
            and abs(motion['captured_s']-motion['previous_s']-dt)<1e-6
            and 0<=motion['rms_m']<=.012 and motion['points']>=12
            and math.hypot(motion['delta_x_m'],motion['delta_y_m'])/dt<=.9
            and abs(motion['yaw_rad'])/dt<=1.5
            and abs(motion['delta_y_m']/dt-motion['speed_mps'])<1e-6)


def project_ground_pairs(projection,before,after):
    """Batch the matrix work, retaining the exact measured-polygon gate."""
    before=np.asarray(before,np.float32).reshape(-1,2)
    after=np.asarray(after,np.float32).reshape(-1,2)
    inside=np.array([cv2.pointPolygonTest(projection.poly,tuple(a),False)>=0
                     and cv2.pointPolygonTest(projection.poly,tuple(b),False)>=0
                     for a,b in zip(before,after)],bool)
    before=before[inside];after=after[inside]
    qa=np.column_stack((before,np.ones(len(before))))@projection.h.T
    qb=np.column_stack((after,np.ones(len(after))))@projection.h.T
    valid=(np.abs(qa[:,2])>=1e-9)&(np.abs(qb[:,2])>=1e-9)
    qa=qa[valid,:2]/qa[valid,2,None];qb=qb[valid,:2]/qb[valid,2,None]
    finite=np.isfinite(qa).all(axis=1)&np.isfinite(qb).all(axis=1)
    return qa[finite],qb[finite]


def ground_features(grey,mask):
    # Padding preserves the default 3x3 gradient/block neighbourhood at the
    # masked boundary, while avoiding feature response over the entire image.
    x,y,w,h=cv2.boundingRect(mask)
    if w==0 or h==0:return None
    x0=max(0,x-4);y0=max(0,y-4);x1=min(grey.shape[1],x+w+4);y1=min(grey.shape[0],y+h+4)
    points=cv2.goodFeaturesToTrack(grey[y0:y1,x0:x1],100,.015,6,mask=mask[y0:y1,x0:x1])
    if points is not None:points+=np.array([x0,y0],np.float32)
    return points


class GroundTextureMotion:
    def __init__(self,ground):
        self.projection=GroundProjection(ground,require_verified=False)
        self.previous=None;self.latest=None;self.last_id=None;self.last_t=None;self.last_good=None

    def update(self,image,row):
        t=row['captured_s'];fid=row.get('frame_id')
        if type(fid) is not int or not numeric(t):return {'valid':False,'reason':'frame_id_missing','captured_s':t}
        if fid==self.last_id and t==self.last_t:return self.latest
        if self.last_t is not None and t<=self.last_t:
            return {'valid':False,'reason':'out_of_order_capture','captured_s':t}
        reset=self.last_id is not None and fid<=self.last_id
        if reset:self.previous=None;self.last_good=None
        self.last_id,self.last_t=fid,t
        if (not isinstance(image,np.ndarray) or image.ndim!=3 or image.shape[2]!=3
                or list(image.shape[1::-1])!=self.projection.camera['image_size']):
            self.previous=None;self.latest={'valid':False,'reason':'image_shape_changed','captured_s':t}
            return self.latest
        grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        mask=np.zeros(grey.shape,np.uint8)
        cv2.fillConvexPoly(mask,self.projection.poly.astype(np.int32),255)
        context=row.get('road_context') or {};scope='historical_polygon'
        if context.get('valid') and len(context.get('left_inner',[]))>=6 and len(context.get('right_inner',[]))>=6:
            corridor=np.zeros_like(mask)
            polygon=np.asarray(context['left_inner']+context['right_inner'][::-1],np.int32)
            cv2.fillPoly(corridor,[polygon],255);mask=cv2.bitwise_and(mask,corridor)
            scope='observed_corridor'
        for track in row.get('tracks',[]):
            if not track.get('visible'):continue
            x1,y1,x2,y2=map(lambda v:int(round(v)),track['bbox_xyxy'])
            cv2.rectangle(mask,(x1-5,y1-5),(x2+5,y2+5),0,-1)
        points=ground_features(grey,mask)
        old=self.previous;self.previous=(grey,points,t,mask)
        bad={'valid':False,'captured_s':t,'reason':'camera_stream_reset' if reset else 'insufficient_ground_texture','reference_only':True}
        self.latest=bad
        bad['detected_points']=0 if points is None else len(points)
        if old is None:return bad
        if old[1] is None or points is None:return bad
        if not .01<=t-old[2]<=VISION_MAX_AGE_S:
            bad['reason']='capture_interval_out_of_range';return bad
        previous,p,old_t,old_mask=old;dt=t-old_t
        initial=None;flags=0
        if self.last_good and 0<=old_t-self.last_good['captured_s']<=.3:
            # Initial search location from the previous *measured* ground flow.
            # It is only a pixel matcher prior: forward/backward photometric and
            # rigid-ground checks below still decide whether motion is valid.
            last=self.last_good;ratio=dt/last['dt_s'];yaw=last['yaw_rad']*ratio
            co,si=math.cos(yaw),math.sin(yaw)
            rotation=np.array([[co,-si],[si,co]])
            delta=np.array([last['delta_x_m'],last['delta_y_m']])*ratio
            body=np.eye(3);body[:2,:2]=rotation;body[:2,2]=-rotation@delta
            transform=np.linalg.inv(self.projection.h)@body@self.projection.h
            initial=cv2.perspectiveTransform(p,transform)
            if np.isfinite(initial).all():flags=cv2.OPTFLOW_USE_INITIAL_FLOW
            else:initial=None
        q,ok,error=cv2.calcOpticalFlowPyrLK(previous,grey,p,initial,winSize=(21,21),maxLevel=3,flags=flags)
        if q is None or ok is None or error is None or not np.isfinite(q).all():return bad
        back,back_ok,_=cv2.calcOpticalFlowPyrLK(grey,previous,q,None,winSize=(21,21),maxLevel=3)
        if back is None or back_ok is None:return bad
        good=(ok[:,0]>0)&(back_ok[:,0]>0)&(error[:,0]<25)&(np.linalg.norm(back[:,0]-p[:,0],axis=1)<1.2)
        bad['matched_points']=int(good.sum())
        a,b=p[:,0][good],q[:,0][good]
        pixels=np.rint(b).astype(int)
        keep=(pixels[:,0]>=0)&(pixels[:,0]<grey.shape[1])&(pixels[:,1]>=0)&(pixels[:,1]<grey.shape[0])
        a,b,pixels=a[keep],b[keep],pixels[keep]
        keep=mask[pixels[:,1],pixels[:,0]]>0
        before,after=project_ground_pairs(self.projection,a[keep],b[keep])
        bad['projected_points']=len(before)
        if len(before)<12:
            bad['reason']='insufficient_projected_matches';return bad
        before=np.asarray(before);after=np.asarray(after)
        matrix,inliers=cv2.estimateAffinePartial2D(before,after,method=cv2.RANSAC,ransacReprojThreshold=.012,maxIters=150)
        if matrix is None or not np.isfinite(matrix).all() or inliers is None or np.mean(inliers)<.75:
            bad['reason']='inconsistent_ground_motion';return bad
        scale=math.hypot(matrix[0,0],matrix[1,0])
        if not .97<=scale<=1.03:
            bad['reason']='ground_scale_changed';bad['scale']=scale;return bad
        rotation=matrix[:,:2]/scale;delta=-rotation.T@matrix[:,2]
        yaw=math.atan2(rotation[1,0],rotation[0,0])
        residual=before@matrix[:,:2].T+matrix[:,2]-after
        rms=float(np.sqrt(np.mean(np.sum(residual[inliers[:,0]>0]**2,axis=1))))
        if not math.isfinite(rms) or rms>.012 or np.linalg.norm(delta)/dt>.9 or abs(yaw/dt)>1.5:
            bad['reason']='ground_motion_limit';return bad
        self.latest={'valid':True,'captured_s':t,'previous_s':old_t,'dt_s':dt,
            'delta_x_m':float(delta[0]),'delta_y_m':float(delta[1]),'yaw_rad':yaw,
            'speed_mps':float(delta[1]/dt),'rms_m':rms,'points':int(inliers.sum()),
            'reference_only':True,'floor_scope':scope,'reason':'measured_pixel_flow_historical_ground_scale'}
        self.last_good=self.latest
        return self.latest


def select_motion(motions,now):
    diagnostics=[{k:m[k] for k in ('camera','captured_s','reason','detected_points','matched_points','projected_points','scale','rms_m','points') if k in m}
                 for m in motions]
    good=[m for m in motions if valid_motion(m,now)]
    if not good:return {'valid':False,'captured_s':now,'reason':'ground_texture_motion_unavailable','camera_measurements':diagnostics}
    newest=max(m['captured_s'] for m in good)
    # Only compare same-time, similar-duration intervals; asynchronous cameras
    # need not report identical displacements in different capture intervals.
    current=[m for m in good if newest-m['captured_s']<.03]
    for i,a in enumerate(current):
        for b in current[i+1:]:
            if abs(a['previous_s']-b['previous_s'])>=.03 or abs(a['dt_s']-b['dt_s'])>=.02:continue
            dv=math.hypot(a['delta_x_m']/a['dt_s']-b['delta_x_m']/b['dt_s'],a['speed_mps']-b['speed_mps'])
            dyaw=abs(a['yaw_rad']/a['dt_s']-b['yaw_rad']/b['dt_s'])
            if dv>.12 or dyaw>.35:
                return {'valid':False,'captured_s':newest,'reason':'ground_motion_camera_disagreement','camera_measurements':diagnostics}
    observed=[m for m in current if m.get('floor_scope')=='observed_corridor']
    if observed:current=observed
    return {**min(current,key=lambda m:m['rms_m']+.0001/max(1,m['points'])),'camera_measurements':diagnostics}
