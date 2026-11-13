"""Bounded initial alignment before a metric bay centre has been acquired.

Current image edges guide alignment; they never assert entrance crossing.
This phase has its own deadline and cannot refresh a missing full observation
by repeatedly tracking only its far edge.
"""
import numpy as np


class ParkingApproach:
    BRIDGE_S=.25

    def __init__(self,view,front,timeout_s):
        self.view,self.front,self.timeout_s=view,front,timeout_s
        self.reset()

    def reset(self):
        self.started=self.full_at=self.last_error=None
        self.full_quad=None;self.last_region=None
        self.distance_bound=None
        self.frame=self.capture=None
        self.report={}

    def remaining(self,now):
        return self.timeout_s if self.started is None else max(0.,self.timeout_s-(now-self.started))

    def region(self,q,line):
        """Classify against the actual polygon, not a horizontal pixel row."""
        if line is not None:
            # Includes braking/alignment space ahead of the front of the car.
            return 'partial_with_distance' if line['distance_m']>self.front+.35+self.view.error else 'too_close'
        points=[p for x in np.linspace(q[0],q[1],60) if (p:=self.view.point(x)) is not None]
        if points:
            # A few points are insufficient for a pose but can bound forward
            # clearance during entry into the calibrated area.
            return 'partial_with_clearance' if min(p[1] for p in points)>self.front+.35+self.view.error else 'too_close'
        # The upper polygon edge separates far ground from the measured patch.
        # Its slope matters when the selected bay lies to either side.
        poly=self.view.poly;mid=poly.mean(axis=0)
        edges=list(zip(poly,np.roll(poly,-1,axis=0)))
        a,b=min(edges,key=lambda e:float((e[0][1]+e[1][1])/2))
        normal=np.array([-(b-a)[1],(b-a)[0]],float)
        normal/=np.linalg.norm(normal)
        if (mid-a)@normal<0:normal=-normal
        # Only the entrance segment in front of the measured patch can decide
        # far/near. An endpoint beyond its lateral extent may cross the infinite
        # extension of this sloping edge while the visible entrance is still far.
        tangent=(b-a)/np.linalg.norm(b-a)
        positions=(q[:2]-a)@tangent;delta=positions[1]-positions[0]
        if abs(delta)>1e-6:
            limits=sorted((-positions[0]/delta,(np.linalg.norm(b-a)-positions[0])/delta))
            lo,hi=max(0.,limits[0]),min(1.,limits[1])
            if hi>lo:
                clipped=q[0]+np.array([lo,hi])[:,None]*(q[1]-q[0])
                if np.max((clipped-a)@normal)<0:return 'far'
        return 'lateral_or_near_outside'

    def measured_distance_bound(self,q,line):
        if line is not None:return line['distance_m']
        points=[p for x in np.linspace(q[0],q[1],60) if (p:=self.view.point(x)) is not None]
        return min(p[1] for p in points) if points else None

    @staticmethod
    def image_error(geometry):
        checks=geometry.get('envelope_checks') or []
        if len(checks)!=2 or any(c.get('center_px') is None for c in checks):return None
        offsets=[((c['left_px']+c['right_px'])/2-c['center_px'])/
                 max(10.,(c['right_px']-c['left_px'])/2) for c in checks]
        return float(np.clip(.25*offsets[0]+.75*offsets[1],-.28,.28))

    def plan(self,q,near,geometry,white,line,now,sample):
        if self.started is None:self.started=now
        fid,t=sample.get('frame_id'),sample.get('captured_s')
        flow=sample.get('parking_ground_motion') or {}
        linked=(flow.get('valid') is True and flow.get('frame_id')==fid
                and flow.get('captured_s')==t and self.frame is not None
                and flow.get('previous_frame_id')==self.frame
                and flow.get('previous_captured_s')==self.capture
                and t is not None and 0<t-self.capture<=.5)
        self.frame,self.capture=fid,t
        self.report={'remaining_s':self.remaining(now),'bridge_used':False,
                     'full_boundary_age_s':None if self.full_at is None else now-self.full_at}
        if self.remaining(now)<=0:
            return self.wait('初次车位定位超时，仍未建立可靠车位位置')
        if q is not None and geometry.get('availability')!='clear':
            return self.wait('当前停车区空闲证据不足')
        recent=(self.full_at is not None and 0<now-self.full_at<=self.BRIDGE_S
                and self.last_region in ('far','partial_with_clearance','partial_with_distance'))
        range_ok=(recent and (self.last_region=='far' or self.distance_bound is not None
                  and self.distance_bound-1.5*(now-self.full_at)>self.front+.35+self.view.error))
        if q is not None and near:
            region=self.region(q,line);self.report['range_region']=region
            if region=='too_close':
                return self.wait('入口已经接近，完整定位与制动余量不足，等待重新对准')
            if region=='lateral_or_near_outside':
                return self.wait('入口位于标定有效范围的侧方或近端，缺少可用定位边线')
            error=self.image_error(geometry)
            if error is None:return self.wait('入口图像方向不足')
            self.full_at=now;self.full_quad=q.copy();self.last_error=error;self.last_region=region
            self.distance_bound=self.measured_distance_bound(q,line)
            return self.forward(error,'按当前入口方向低速对准，正在建立车位位置')
        if recent:
            # Three current edges can bridge a missing entrance. A far-only
            # observation additionally needs a matching current ground flow
            # or the currently observed road corridor.
            partial=q is not None and not near and geometry.get('availability')=='clear'
            far=geometry.get('far_edge_px')
            # Crossing the calibration's far edge must not itself disable a
            # short bridge. Preserve the current range evidence, subtracting
            # the same 1.5 m/s motion bound used by ground-flow validation.
            far_only=(q is None and (linked or white.get('observed') is True)
                      and geometry.get('source')=='current_tracked_far_edge'
                      and far is not None and range_ok)
            if partial or far_only:
                current=q[[3,2]] if partial else np.asarray(far,float)
                old=self.full_quad[[3,2]]
                ratio=np.linalg.norm(current[1]-current[0])/max(1.,np.linalg.norm(old[1]-old[0]))
                if .8<=ratio<=1.25 and np.max(np.linalg.norm(current-old,axis=1))<=30:
                    error=self.image_error(geometry) if partial else self.last_error
                    if error is not None:
                        self.report['bridge_used']=True
                        return self.forward(error,'同一车位当前边线仍可见，短时衔接入口识别')
        if self.full_at is None and white.get('observed') and now-self.started<3.:
            return {'stage':'seek','error':0.,'allowed':True,'reason':'沿当前道路低速寻找完整入口'}
        return self.wait('当前入口与连续边线依据不足，等待重新识别目标车位')

    @staticmethod
    def wait(reason):return {'stage':'locate_entry','allowed':False,'reason':reason}

    @staticmethod
    def forward(error,reason):return {'stage':'approach','allowed':True,'error':error,'reason':reason}
