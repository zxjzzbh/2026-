"""Physical-target continuity and bounded alignment during a cone manoeuvre.

Aliases share identity only, never camera coordinates. Near-side observations
allow gradual heading correction; they do not certify rear-body passage.
"""
from .brake_parking import numeric


class ConeAliases:
    def __init__(self):self.reset()

    def reset(self):
        self.ids=set()
        self.views={}

    def remember(self,camera,target,captured):
        self.ids.add((camera,target['id']))
        self.views[camera]=(captured,dict(target))

    def match(self,rows,camera,captured,hint=None):
        if not self.views or captured-max(v[0] for v in self.views.values())>5.:
            return None,None
        known=[r for r in rows if (camera,r['id']) in self.ids]
        if len(known)==1:return known[0],'known_view_alias_retained'
        if len(known)>1:return None,None
        hint=hint or {}
        if ((hint.get('from_camera'),hint.get('from_id')) in self.ids
                and hint.get('to_camera')==camera):
            candidate=next((r for r in rows if r['id']==hint.get('to_id')),None)
            if candidate is not None:return candidate,'paired_view_alias_retained'
        previous=self.views.get(camera)
        if previous and 0<=captured-previous[0]<=.8:
            old=previous[1]
            matches=[r for r in rows if all(numeric(c.get(k)) for c in (r,old)
                     for k in ('image_fraction','y_fraction'))
                     and abs(r['image_fraction']-old['image_fraction'])<=.14
                     and -.08<=r['y_fraction']-old['y_fraction']<=.25]
            if len(matches)==1:return matches[0],'same_view_reassociated'
        return None,None


class ConeReturnGuard:
    """Limit countersteering before sustained near-side observations.

The early 20% steering allowance can arrest outward yaw without permitting
    the old 65% return across an approaching cone. Current close-boundary
    recovery can override this limit when stronger inward steering is needed.
"""
    def __init__(self):self.reset()

    def reset(self):
        self.key=None;self.captured=None;self.clear_hits=0
        self.near_side_seen=False;self.limit=.20;self.report={}

    def observe(self,target,camera,domain,captured,now,side,*,fresh,clear):
        key=(camera,domain,side)
        if target is not None and fresh and side in ('left','right'):
            if key!=self.key:
                if self.key and side!=self.key[2]:self.near_side_seen=False
                self.key=key;self.captured=None;self.clear_hits=0
                self.limit=.20
            if self.captured is None or captured>self.captured:
                self.clear_hits=self.clear_hits+1 if clear else 0
                self.captured=captured
                y=target.get('y_fraction')
                start=.90 if camera=='primary' else .85
                close=max(0.,min(1.,(y-start)/(.99-start))) if numeric(y) else 0.
                distance=target.get('reference_distance_m')
                if numeric(distance) and distance>.50:close=0.
                self.limit=.20+.25*close if self.clear_hits>=2 else .20
                if self.clear_hits>=2 and numeric(y) and y>=(.94 if camera=='primary' else .90):
                    self.near_side_seen=True
        age=None if self.captured is None else max(0.,now-self.captured)
        # Old observations cannot progressively unlock more return steering.
        limit=self.limit if age is not None and age<=.8 else .20
        self.report=dict(counter_limit=limit,near_side_observed=self.near_side_seen,
            current_clear_hits=self.clear_hits,observation_age_s=age,
            body_passed=False,applied=False,reason='bounded_alignment_before_body_passage')
        return self.report

    @staticmethod
    def current_boundary_close(margin,side):
        # A lookahead fraction changing during yaw is not by itself proof of
        # present body clearance. Preserve strong correction for a CURRENT
        # close boundary, not just the image-velocity preview.
        if margin.get('observed_edge_side')==side:
            gap=margin.get('edge_clearance_fraction')
            return numeric(gap) and gap<=.20
        q=margin.get('near_fraction');reserve=margin.get('center_margin_fraction')
        if not numeric(q) or not numeric(reserve):return False
        return q<=reserve+.03 if side=='left' else q>=1-reserve-.03

    def constrain(self,command,side,margin=None,*,boundary_override=False):
        if side not in ('left','right'):return command
        sign=-1 if side=='left' else 1
        before=command
        override=boundary_override or self.current_boundary_close(margin or {},side)
        if command*sign<0 and not override:
            command=sign*max(command*sign,-self.report.get('counter_limit',.20))
        self.report.update(applied=abs(command-before)>1e-9,
                           current_boundary_override=override,
                           unconstrained_command=before,constrained_command=command)
        return command
