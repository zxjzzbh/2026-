"""Choose cone corridors by free space, with a 20 cm right-hand preference."""
import math

import numpy as np

from .brake_parking import numeric
from .cone_timing import frame_fresh

TIE_DISTANCE_M=.20


def side_spaces(nav,target,settings,now):
    """Scale a current same-camera cross-section by the operator's track width.

    These are estimates, not newly verified metric calibration. Never compare
    another camera's pixels or infer a second boundary from a single stripe.
    """
    camera=nav.get('source_camera');captured=nav.get('captured_s')
    width=settings['geometry']['lane_width_m']
    if not numeric(width) or width<=0 or not frame_fresh(captured,now):return None
    def result(left,right,observed):
        if not all(numeric(v) and v>=0 for v in (left,right)):return None
        return dict(left_gap_m=left*width,right_gap_m=right*width,lane_width_m=width,
                    observed_s=observed,camera=camera,metric_verified=False,
                    source='current_same_camera_edges_scaled_by_track_width')
    if (not nav.get('line_missing') and not nav.get('line_held') and not nav.get('single_edge_observed')
            and nav.get('lane_bounds_px') and numeric(target.get('fraction')) and numeric(target.get('half_width'))):
        return result(target['fraction']-target['half_width'],1-target['fraction']-target['half_width'],captured)
    w,h=settings['image_size'];x=target.get('image_fraction');y=target.get('y_fraction')
    radius=target.get('half_image_width')
    if not all(numeric(v) for v in (x,y,radius)):return None
    for row in (nav.get('local_course') or {}).get('image_lane',[]):
        if row.get('camera')!=camera or not frame_fresh(row.get('captured_s'),now):continue
        edges=[np.asarray(row.get(side),float) for side in ('left','right')]
        if any(v.ndim!=2 or v.shape[1]!=2 or len(v)<6 or not np.isfinite(v).all() for v in edges):continue
        low=max(v[:,1].min() for v in edges);high=min(v[:,1].max() for v in edges)
        if high-low<h*.09 or not low-h*.10<=y*h<=high+h*.10:continue
        fits=[np.polyfit(v[:,1],v[:,0],1) for v in edges]
        if any(np.max(np.abs(np.polyval(f,v[:,1])-v[:,0]))>8 for f,v in zip(fits,edges)):continue
        left,right=[float(np.polyval(f,y*h)) for f in fits];span=right-left
        if not w*.15<span<w*1.8:continue
        measured=result(((x-radius)*w-left)/span,(right-(x+radius)*w)/span,row['captured_s'])
        if measured:return measured
    return None


def choose_corridor(choices,spaces):
    """Return the wider feasible corridor; ties below 20 cm prefer right."""
    by_side={}
    for choice in choices:
        _,side,left,right,_=choice
        previous=by_side.get(side)
        if previous is None or (right-left,-choice[0])>(previous[3]-previous[2],-previous[0]):
            by_side[side]=choice
    report={'threshold_m':TIE_DISTANCE_M,'metric_verified':False,
            'available_sides':sorted(by_side),'left_gap_m':None,'right_gap_m':None}
    if spaces:
        report.update(spaces)
        difference=spaces['left_gap_m']-spaces['right_gap_m']
        tie=abs(difference)<TIE_DISTANCE_M and not math.isclose(abs(difference),TIE_DISTANCE_M,abs_tol=1e-9)
        preferred='right' if tie or difference<=0 else 'left'
        report.update(difference_m=abs(difference),reason='under_20cm_prefer_right' if tie else 'larger_gap')
    else:
        # Do not apply a centimetre threshold to raw image fractions.
        left=by_side.get('left');right=by_side.get('right')
        a=left[3]-left[2] if left else -1.;b=right[3]-right[2] if right else -1.
        preferred='left' if a>b+1e-9 else 'right'
        report.update(difference_m=None,reason='image_space_only_no_centimetre_comparison')
    report['preferred_side']=preferred
    if not by_side:return None,report
    chosen=by_side.get(preferred)
    if chosen is None:
        chosen=next(iter(by_side.values()))
        report['reason']='preferred_side_has_no_vehicle_corridor'
    report['selected_side']=chosen[1]
    return chosen,report


class ConeRouteChoice:
    """Separate a preview, a confirmed route and issued steering on that route.

    A lock never makes a missing corridor feasible. Metric exclusions remain
    attached to this physical target until new measurements reopen the side;
    an image-only fallback cannot erase them.
    """
    def __init__(self,settings):
        self.required_gap_m=settings['geometry']['vehicle_width_m']+.07
        self.reset()

    def reset(self):
        self.side=None;self.confirmed=False;self.committed=False
        self.metric_choice=False;self.blocked=set()
        self.pending=None;self.capture=None;self.hits=0

    @staticmethod
    def close(target,camera):
        return bool(target.get('image_close') or target.get('near_field_continuation')
                    or target.get('y_fraction',0)>=(.90 if camera=='primary' else .75))

    def choose(self,choices,spaces,*,camera,captured,target,ego,held=False,applied=0.,prefer_left=False):
        if spaces and not held:
            for side in ('left','right'):
                gap=spaces[side+'_gap_m']
                if gap<self.required_gap_m:self.blocked.add(side)
                elif gap>=self.required_gap_m+.02:self.blocked.discard(side)
        choices=[c for c in choices if c[1] not in self.blocked]
        preferred,report=choose_corridor(choices,spaces)
        # A boundary recovery is stronger than the ordinary 20 cm tie rule,
        # but it may select only an existing, physically feasible left route.
        inward=next((c for c in choices if c[1]=='left'),None) if prefer_left and not held else None
        boundary_switch=inward is not None and self.side!='left'
        if inward is not None:
            preferred=inward
            report['reason']='right_boundary_prefer_feasible_left'
        report['boundary_preference']='left' if inward is not None else None
        same=next((c for c in choices if c[1]==self.side),None)
        if self.side is not None and same is None:
            self.committed=False;self.confirmed=False;self.metric_choice=False
        if same and self.confirmed and applied*(-1 if self.side=='left' else 1)>.15:
            # Actual issued steering starts the manoeuvre even when its first
            # observations used image space. Later metric tie preferences must
            # not reverse a still-feasible route already being executed.
            self.committed=True
        report.update(required_gap_m=self.required_gap_m,blocked_sides=sorted(self.blocked),
                      preference_reason=report['reason'])

        def result(choice,reason):
            report.update(selected_side=choice[1] if choice else None,
                locked=bool(choice and self.committed),provisional=not self.confirmed,
                confirmation_hits=self.hits,candidate_side=self.pending,
                metric_route=self.metric_choice,reason=reason)
            return choice,report

        if preferred is None:
            self.committed=False
            self.pending=None;self.hits=0;self.capture=None
            return result(None,'no_current_vehicle_corridor')
        if held:
            return result(same,'held_observation_cannot_choose_new_route')
        if same and self.confirmed and not boundary_switch and (self.committed or self.metric_choice and spaces is None):
            return result(same,'retain_feasible_confirmed_route')

        candidate=preferred[1]
        # Only distinct frames from one camera confirm a changed preference.
        key=(camera,captured)
        if candidate!=self.pending or self.capture and self.capture[0]!=camera:
            self.pending=candidate;self.hits=0;self.capture=None
        if key!=self.capture:
            self.capture=key;self.hits+=1
        if self.hits<2:
            if self.side is None:
                # A first usable observation can start a provisional
                # response immediately. It does not acquire a side lock.
                self.side=candidate
                return result(preferred,'provisional_current_corridor')
            if same and candidate==self.side:
                return result(same,'provisional_current_corridor')
            # Never send the old side into a corridor that disappeared. A
            # provisional opposite preference gets a short heading-only frame.
            return result(same if self.confirmed and same else None,'confirming_current_corridor')

        crossing=(ego-target['x'])*(preferred[4]-target['x'])<0
        if self.close(target,camera) and crossing and candidate!=self.side:
            # At the bumper, switching to the other side may cross the cone's
            # nose. Reject that trajectory without inventing the old corridor.
            self.committed=False
            return result(None,'near_cone_crossing_not_supported')
        changed=candidate!=self.side
        if changed:self.committed=False;self.metric_choice=False
        self.side=candidate;self.confirmed=True
        self.metric_choice|=spaces is not None
        sign=-1 if candidate=='left' else 1
        clear=preferred[2]<=ego<=preferred[3]
        if applied*sign>.15 or spaces and clear and self.close(target,camera):
            self.committed=True
        return result(preferred,'confirmed_new_route' if changed else report['reason'])
