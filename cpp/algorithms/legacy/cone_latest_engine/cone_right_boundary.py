"""Right-stripe distance from the retained secondary-camera calibration.

This is a planar estimate, not measured tyre clearance. A single observed
right stripe suffices; a pair need not supply a complete vehicle pose. Never
turn image-row fractions, cached lines or another camera's pixels into metres.
"""
import math

import numpy as np

from .cone_local_flow import ground
from .cone_timing import frame_fresh


def clamp(value,low,high):return max(low,min(high,value))


class RightBoundaryRecovery:
    enter_m=.50
    release_m=.55

    def __init__(self,settings):
        self.s=settings
        geometry=settings['geometry']
        self.center_y=(geometry['rear_axle_to_front_m']-geometry['rear_overhang_m'])/2
        self.left_reserve=geometry['vehicle_width_m']/2+.10
        self.active=False
        self.last_capture=None
        self.report={}

    def _fit(self,points):
        try:
            pixels=np.asarray(points,float)
            calibration=self.s['ground']
            if (calibration.get('image_size')!=self.s['image_size']
                    or calibration.get('coordinate_frame')!='rear_axle_x_right_y_forward'):
                return None
            if pixels.ndim!=2 or pixels.shape[1]!=2 or len(pixels)<6:return None
            if not np.isfinite(pixels).all() or np.ptp(pixels[:,1])<self.s['image_size'][1]*.09:return None
            h=np.asarray(calibration['homography'],float)
            if h.shape!=(3,3) or not np.isfinite(h).all():return None
            p=ground(h,pixels)
            if p is None:return None
            keep=(p[:,1]>.3)&(p[:,1]<3.)&(np.abs(p[:,0])<2.)
            p=p[keep];pixels=pixels[keep]
            if len(p)<6 or np.ptp(p[:,1])<.25 or p[:,1].min()>1.5:return None
            a,b=np.polyfit(p[:,1],p[:,0],1)
            residual=float(np.max(np.abs(p[:,0]-a*p[:,1]-b)))
            if residual>.035 or abs(a)>2.:return None
            distance=float((a*self.center_y+b)/math.hypot(1.,a))
            if not math.isfinite(distance):return None
            # Report extrapolation honestly. The retained homography's check
            # polygon is narrower than the track; do not mark this verified.
            polygon=np.asarray(calibration.get('valid_polygon_px',[]),float)
            inside=0
            if polygon.ndim==2 and polygon.shape[1]==2 and len(polygon)>=3:
                for point in pixels:
                    edge=np.roll(polygon,-1,axis=0)-polygon
                    offset=point-polygon
                    cross=edge[:,0]*offset[:,1]-edge[:,1]*offset[:,0]
                    inside+=bool(np.all(cross>=-1e-6) or np.all(cross<=1e-6))
            return dict(signed_distance_m=distance,fit_residual_m=residual,
                support_points=len(p),calibration_polygon_points=inside,
                extrapolated=inside<len(p),slope=float(a))
        except (KeyError,TypeError,ValueError,np.linalg.LinAlgError):return None

    def observe(self,nav,now):
        course=nav.get('local_course') or {}
        candidates=[]
        for row in course.get('image_lane',[]):
            candidates.append((row,row.get('right'),row.get('left')))
        for row in course.get('single_heading',[]):
            anchor=row.get('paired_observed_s')
            if (row.get('side')=='right' and isinstance(anchor,(int,float))
                    and 0<=now-anchor<=5.):
                candidates.append((row,row.get('points'),None))
        report=dict(active=False,available=False,applied=False,enter_m=self.enter_m,
            release_m=self.release_m,reference='vehicle_geometric_center_to_right_inner_edge',
            center_y_m=self.center_y,metric_verified=False,source='retained_secondary_planar_calibration',
            reason='no_current_calibrated_right_stripe')
        candidates=[c for c in candidates if c[0].get('camera')=='secondary'
                    and frame_fresh(c[0].get('captured_s'),now) and not c[0].get('line_held')]
        candidates.sort(key=lambda c:c[0]['captured_s'],reverse=True)
        for row,right,left in candidates:
            captured=row['captured_s']
            if self.last_capture is not None and captured<self.last_capture:continue
            fitted=self._fit(right)
            if fitted is None:continue
            distance=fitted['signed_distance_m']
            if not -.30<=distance<=self.s['geometry']['lane_width_m']+.30:continue
            # Do not carry a latched 50--55 cm state across a feedback outage.
            if self.last_capture is None or captured-self.last_capture>.5:self.active=False
            # Homography/least-squares roundoff differs across CPU libraries.
            # A nanometre tolerance stabilises exact thresholds, not sensor noise.
            if distance<self.enter_m-1e-9:self.active=True
            elif distance>=self.release_m-1e-9:self.active=False
            self.last_capture=captured
            left_fit=self._fit(left) if left is not None else None
            left_distance=-left_fit['signed_distance_m'] if left_fit else None
            left_limited=left_distance is not None and left_distance<=self.left_reserve
            strength=(.12+.38*clamp((self.enter_m-distance)/.20,0.,1.) if distance<self.enter_m
                else .12*clamp((self.release_m-distance)/.05,0.,1.)) if self.active else 0.
            report.update(fitted,available=True,active=self.active,captured_s=captured,
                right_distance_m=distance,left_distance_m=left_distance,left_limited=left_limited,
                left_command=-strength,reason='below_right_margin' if self.active else 'right_margin_clear')
            break
        if not report['available'] and (self.last_capture is None or not frame_fresh(self.last_capture,now)):
            self.active=False
        self.report=report
        return report

    @property
    def prefer_left(self):
        return self.report.get('active',False) and not self.report.get('left_limited',False)

    def constrain(self,command,*,left_allowed,left_risk=0.):
        report=self.report
        if not report.get('active'):return command
        before=command
        if report.get('left_limited') or left_risk>0:
            report['reason']='left_boundary_limits_right_margin_recovery'
        elif left_allowed:
            command=min(command,report['left_command'])
            report['reason']='gentle_left_right_margin_recovery'
        else:
            # A near cone/retained passing corridor cannot be crossed just to
            # satisfy the boundary preference. Unwind outward steering first.
            command=min(command,0.)
            report['reason']='unwind_right_without_crossing_cone'
        report.update(applied=abs(command-before)>1e-9,unconstrained_command=before,
            constrained_command=command,left_allowed=left_allowed)
        return command
