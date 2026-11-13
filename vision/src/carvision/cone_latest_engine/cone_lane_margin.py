"""Image-space lane-margin constraints; not a calibrated vehicle pose."""
from .brake_parking import numeric
from .cone_timing import frame_fresh


def clamp(value,low,high):return max(low,min(high,value))


def center_margin(settings):
    # Nominal vehicle half-width plus a 10 cm planning reserve. Image fractions
    # and the historical track width cannot certify real tyre clearance.
    return clamp((settings['geometry']['vehicle_width_m']/2+.10)/settings['geometry']['lane_width_m'],.18,.35)


class LaneMarginGuard:
    preview_s=.35
    soft_band=.12
    heading_limit=.04

    def __init__(self,settings):
        self.margin=center_margin(settings)
        self.history={}

    def constrain(self,command,geometry,now,*,clearance_side=None):
        report={'applied':False,'position_verified':False,'reason':'no_current_lane_feedback'}
        if not geometry or geometry.get('feedback_weight',0)<=0:
            return command,report
        if geometry.get('heading_only'):
            return self._single_edge(command,geometry,now,report)
        camera=geometry['camera'];captured=geometry['lane_observed_s']
        near=geometry['near_ego_fraction'];heading=geometry['heading_correction_fraction']
        if not all(numeric(v) for v in (captured,near,heading)) or not 0<=now-captured<=1.:
            return command,report
        velocity=0.
        previous=self.history.get(camera)
        if geometry.get('fresh_lane') and frame_fresh(captured,now):
            if previous and captured==previous[0]:
                velocity=previous[2]
            elif previous and 0<captured-previous[0]<=.25:
                raw=clamp((near-previous[1])/(captured-previous[0]),-1.,1.)
                velocity=.6*raw+.4*previous[2]
            if previous is None or captured>previous[0]:
                self.history[camera]=(captured,near,velocity)
        projected=near+clamp(velocity*self.preview_s,-.20,.20)
        left_risk=clamp((self.margin+self.soft_band-min(near,projected))/self.soft_band,0.,1.)
        right_risk=clamp((max(near,projected)-(1-self.margin-self.soft_band))/self.soft_band,0.,1.)
        weight=geometry['feedback_weight'];before=command;reasons=[]
        response=geometry.get('heading_command',clamp(8*heading,-.65,.65)*weight)
        # Begin unwinding the outward request before a hard boundary response.
        # This preserves the selected cone side while reducing the approach
        # toward the line; a genuinely inward correction below still wins.
        risk=left_risk if command<0 else right_risk
        if clearance_side and risk>0:
            command*=1-risk
            reasons.append('early_outward_taper')
        # Do not allow the lateral cone term to keep increasing an already
        # excessive outward heading. This does not declare the cone passed.
        # Heading alone cannot reverse an unfinished cone manoeuvre in the
        # middle of the lane. Current/predicted boundary proximity still wins.
        check_heading=clearance_side is None or max(left_risk,right_risk)>.3
        if check_heading and heading < -self.heading_limit:
            recovery=min(0.,response)
            if command>recovery:command=recovery;reasons.append('outward_right_heading')
        elif check_heading and heading>self.heading_limit:
            recovery=max(0.,response)
            if command<recovery:command=recovery;reasons.append('outward_left_heading')
        if right_risk>0 and (heading<=.015 or velocity>.04):
            inward=min(-.15*weight,response)
            ceiling=1-right_risk+right_risk*inward
            if command>ceiling:command=ceiling;reasons.append('right_line_margin')
        if left_risk>0 and (heading>=-.015 or velocity<-.04):
            inward=max(.15*weight,response)
            floor=-1+left_risk+left_risk*inward
            if command<floor:command=floor;reasons.append('left_line_margin')
        report.update(applied=abs(command-before)>1e-9,reason='+'.join(reasons) or 'inside_margin',
            camera=camera,lane_observed_s=captured,near_fraction=near,
            estimated_fraction_rate_s=velocity,preview_fraction=projected,
            center_margin_fraction=self.margin,left_risk=left_risk,right_risk=right_risk,
            unconstrained_command=before,constrained_command=command)
        return command,report

    def _single_edge(self,command,geometry,now,report):
        side=geometry.get('observed_edge_side');gap=geometry.get('edge_clearance_fraction')
        captured=geometry.get('captured_s',geometry['lane_observed_s'])
        if (side not in ('left','right') or not numeric(gap)
                or geometry.get('edge_row_fraction',0)<.65 or not frame_fresh(captured,now)):
            return command,{**report,'reason':'single_stripe_direction_only'}
        key=(geometry['camera'],side);previous=self.history.get(key);velocity=0.
        if previous and captured==previous[0]:velocity=previous[2]
        elif previous and 0<captured-previous[0]<=.25:
            velocity=.6*clamp((gap-previous[1])/(captured-previous[0]),-1.,1.)+.4*previous[2]
        if previous is None or captured>previous[0]:self.history[key]=(captured,gap,velocity)
        predicted=gap+clamp(velocity*self.preview_s,-.20,.20)
        risk=clamp((.30-min(gap,predicted))/.20,0.,1.)
        before=command;response=geometry.get('heading_command',0.)
        inward_sign=1 if side=='left' else -1
        if risk>0:
            if command*inward_sign<0:command*=1-risk
            inward=inward_sign*max(.25,min(.65,inward_sign*response))
            limit=(1-risk)*(-inward_sign)+risk*inward
            command=max(command,limit) if side=='left' else min(command,limit)
            if risk>.15 and response*inward_sign>0:
                command=max(command,response) if side=='left' else min(command,response)
        report.update(applied=abs(command-before)>1e-9,reason='current_single_edge_margin',
            camera=geometry['camera'],lane_observed_s=captured,observed_edge_side=side,
            edge_clearance_fraction=gap,preview_edge_clearance_fraction=predicted,
            left_risk=risk if side=='left' else 0.,right_risk=risk if side=='right' else 0.,
            unconstrained_command=before,constrained_command=command)
        return command,report
