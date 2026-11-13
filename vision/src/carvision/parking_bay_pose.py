"""Fit a rectangular bay to current, range-checked far and side observations."""
import numpy as np


class ParkingBayPose:
    def __init__(self,view,width,length):
        self.view,self.width,self.length=view,width,length
        self.reason='no_current_bay_landmarks'

    def entrance(self,q,line):
        """Locate a partly calibrated entrance using a current adjacent side.

        Only pixels inside the existing calibration are projected. The unseen
        centre is a rectangular-bay estimate with explicit width uncertainty.
        A cut standing in for a missing entrance must never call this method.
        """
        self.reason='entrance_side_outside_calibration'
        if line is None:return None
        normal=np.asarray(line['normal']);tangent=np.array([normal[1],-normal[0]])
        offsets={};residual=0.
        for name,a,b in [('left',0,3),('right',1,2)]:
            corner=self.view.point(q[a]);side=self.view.segment(q[a],q[b])
            if corner is None or side is None:continue
            vector=side[-1]-side[0];vector/=np.linalg.norm(vector)
            lateral=float(corner@tangent)
            deviation=float(np.max(np.abs(side@tangent-lateral)))
            if vector@normal<.96 or deviation>.025:continue
            if abs(float(corner@normal)-line['distance_m'])>.025:continue
            offsets[name]=lateral;residual=max(residual,deviation)
        if not offsets:return None
        if len(offsets)==2:
            if abs(offsets['right']-offsets['left']-self.width)>.06:
                self.reason='bay_width_disagrees';return None
            lateral=(offsets['right']+offsets['left'])/2
        else:
            lateral=(offsets['left']+self.width/2 if 'left' in offsets
                     else offsets['right']-self.width/2)
        # A 6 cm trial width tolerance contributes 3 cm centre uncertainty.
        # It is not a verified error bound; it follows the pose into clearance
        # and motion budgets instead of being discarded after acquisition.
        error=self.view.error+max(.01,residual)+(.03 if len(offsets)==1 else 0.)
        if error>.08:self.reason='entrance_position_uncertain';return None
        center=normal*line['distance_m']+tangent*lateral
        self.reason=None
        return {**line,'center_m':center.tolist(),'error_estimate_m':error,
                'source':'current_entrance_side_estimate','visible_sides':sorted(offsets),
                'configured_width_m':self.width,'geometry_assumption_verified':False,
                'configured_width_tolerance_m':.06,
                'entrance_observed':True,'centre_inferred':True}

    def measure(self,landmarks,fid,t):
        self.reason='no_current_bay_landmarks'
        if not isinstance(landmarks,dict):return None
        if (landmarks.get('source')!='current_bay_landmarks' or landmarks.get('frame_id')!=fid
                or landmarks.get('captured_s')!=t or landmarks.get('far_observed') is not True):return None
        def project(edge):
            try:p=np.asarray(edge,dtype=float)
            except (ValueError,TypeError):return None
            if p.shape!=(2,2) or not np.isfinite(p).all():return None
            return self.view.segment(*p)
        far=project(landmarks.get('far_edge_px'))
        if far is None:self.reason='far_edge_outside_calibration';return None
        sides={}
        for name,edge in (landmarks.get('side_edges_px') or {}).items():
            if name not in ('left','right') or landmarks.get('side_support',{}).get(name,0)<.72:continue
            points=project(edge)
            if points is not None:sides[name]=points
        if not sides:self.reason='side_edge_outside_calibration';return None
        # Fit perpendicular edges by minimizing the worst boundary residual.
        # Otherwise a long, densely sampled side dominates least squares and
        # gives a short but current far edge almost no say in the orientation.
        # All samples remain inside the calibrated polygon. Limits below stay
        # absolute; neither a wider fit tolerance nor stale pixels are used.
        centered_far=far-far.mean(axis=0)
        centered_sides=np.concatenate([p-p.mean(axis=0) for p in sides.values()])
        def fit(angles):
            normals=np.column_stack((np.sin(angles),np.cos(angles)))
            tangents=np.column_stack((np.cos(angles),-np.sin(angles)))
            residual=np.maximum(np.max(np.abs(centered_far@normals.T),axis=0),
                                np.max(np.abs(centered_sides@tangents.T),axis=0))
            return float(angles[int(np.argmin(residual))])
        limit=float(np.arccos(.7));angle=fit(np.linspace(-limit,limit,401))
        angle=fit(np.linspace(max(-limit,angle-.004),min(limit,angle+.004),41))
        normal=np.array([np.sin(angle),np.cos(angle)])
        tangent=np.array([normal[1],-normal[0]])
        far_d=float(np.mean(far@normal));offsets={k:float(np.mean(v@tangent)) for k,v in sides.items()}
        residuals=[far@normal-far_d]+[v@tangent-offsets[k] for k,v in sides.items()]
        residual=np.concatenate(residuals);rms=float(np.sqrt(np.mean(residual**2)))
        maximum=float(np.max(np.abs(residual)))
        if rms>.025 or maximum>.06:self.reason='bay_edges_not_rectangular';return None
        if len(offsets)==2:
            measured_width=offsets['right']-offsets['left']
            if abs(measured_width-self.width)>.10:self.reason='bay_width_disagrees';return None
            lateral=(offsets['right']+offsets['left'])/2
        else:
            measured_width=None
            lateral=offsets['left']+self.width/2 if 'left' in offsets else offsets['right']-self.width/2
        distance=far_d-self.length
        center=normal*distance+tangent*lateral
        error=self.view.error+max(.01,maximum)+(.01 if len(sides)==1 else 0.)
        if error>.08:self.reason='bay_position_uncertain';return None
        self.reason=None
        return {'center_m':center.tolist(),'normal':normal.tolist(),'distance_m':distance,
            'error_estimate_m':error,'source':'current_far_side_bay_pose','physical_pose_verified':False,
            'far_distance_m':far_d,'visible_sides':sorted(sides),'fit_rms_m':rms,
            'measured_width_m':measured_width,'configured_width_m':self.width,'configured_length_m':self.length,
            'entrance_observed':False,'geometry_assumption_verified':False}
