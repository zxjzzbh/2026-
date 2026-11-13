"""Follow a currently visible stripe during a turn without renewing old width."""
import numpy as np

from ..cone_corridor import Boundary,fit_boundary,measured_corridor,_check_pair
from .cone_timing import FRAME_MAX_AGE_S


class TurnEdgeRecovery:
    max_width_age_s=1.
    max_heading_age_s=5.

    def __init__(self):
        self.paired=None
        self.previous=None
        self.last_capture=None
        self.current_heading=None
        self.tracked=None

    def update(self,context,shape,captured_s):
        self.current_heading=None
        if (captured_s is None or self.last_capture is not None
                and not 0<captured_s-self.last_capture<FRAME_MAX_AGE_S):
            self.paired=self.previous=None
            self.tracked=None
        self.last_capture=captured_s
        if captured_s is None:return None
        if context.get('valid'):
            try:model=measured_corridor(context,shape)
            except ValueError:return None
            self.paired=(captured_s,model)
            self.previous=(captured_s,model,None)
            self.tracked=None
            return None
        if not self.paired or not self.previous or not context.get('ground_supported'):return None
        age=captured_s-self.paired[0]
        dt=captured_s-(self.tracked[0] if self.tracked else self.previous[0])
        if not 0<age<=self.max_heading_age_s or not 0<dt<FRAME_MAX_AGE_S:return None
        h,w=shape[:2];old=self.previous[1];last_side=self.tracked[2] if self.tracked else None;matches=[]
        for candidate in context.get('boundary_candidates',[])[:12]:
            for side,previous,other,key in (
                    ('left',old.left,old.right,'right_edge'),('right',old.right,old.left,'left_edge')):
                if last_side is not None and side!=last_side:continue
                if self.tracked:previous=self.tracked[1]
                try:current=fit_boundary(candidate.get(key,[]),shape,residual_limit=max(3.,w*.01))
                except ValueError:continue
                low,high=max(current.low,previous.low),min(current.high,previous.high)
                # A real accepted pair may have only 9% common image height.
                # Direction tracking needs overlap with THAT observed stripe,
                # not a new complete pair on every frame of the turn.
                if high-low<h*.09:continue
                ys=np.linspace(low,high,7);shifts=current.at(ys)-previous.at(ys)
                shift=float(np.median(shifts));variation=float(np.max(np.abs(shifts-shift)))
                # Allow a current stripe to move with the turning camera. The
                # gate is per fresh observation, not displacement from an old pair.
                if abs(shift)>min(w*.18,max(24.,w*dt)) or variation>w*.04:continue
                score=abs(shift)/w+variation/w+current.residual/w
                matches.append((score,side,current,variation))
        if not matches:return None
        matches.sort(key=lambda v:v[0]);best=matches[0]
        # Threshold passes can yield several versions of the SAME stripe.
        # Merge those geometrically, but reject genuinely competing identities.
        for other in matches[1:]:
            low=max(best[2].low,other[2].low);high=min(best[2].high,other[2].high)
            if (other[1]!=best[1] or high-low<h*.09
                    or np.max(np.abs(best[2].at(np.linspace(low,high,5))-other[2].at(np.linspace(low,high,5))))>8):
                if other[0]-best[0]<.06:return None
        stripe=best[2];ys=np.linspace(stripe.low,stripe.high,13)
        self.tracked=(captured_s,stripe,best[1])
        self.current_heading={'captured_s':captured_s,'paired_observed_s':self.paired[0],
            'side':best[1],'points':np.column_stack((stripe.at(ys),ys)).tolist()}
        # The observed stripe can still provide direction after the width
        # reference expires. Never promote its predicted mate to a real edge.
        if age>self.max_width_age_s:return None
        previous,other=(old.left,old.right) if best[1]=='left' else (old.right,old.left)
        low,high=max(stripe.low,old.low),min(stripe.high,old.high)
        predicted=Boundary(stripe.coefficients+other.coefficients-previous.coefficients,
                           h,low,high,stripe.residual)
        left,right=(stripe,predicted) if best[1]=='left' else (predicted,stripe)
        try:model=_check_pair(left,right,shape,observed=False)
        except ValueError:return None
        ys=np.linspace(low,high,7)
        if np.any(model.left.at(ys)<-w*.35) or np.any(model.right.at(ys)>w*1.35):return None
        model.state='single';model.prediction_age_s=age
        model.uncertainty=min(.3,.04+.15*age+best[3]/w)
        self.previous=(captured_s,model,best[1])
        return model
