"""Bounded image-space white-edge corridors for cone turns.

Only current paint pixels are observations. A missing side may borrow a recent
same-camera *width model*, never an unchanged historical pixel corridor.
These fits are image guidance, not metric clearance or vehicle pose feedback.
"""
from dataclasses import dataclass

import cv2
import numpy as np


SINGLE_MAX_AGE_S = .25


@dataclass
class Boundary:
    coefficients: np.ndarray
    height: int
    low: float
    high: float
    residual: float

    def at(self, y):
        return np.polyval(self.coefficients, np.asarray(y) / self.height)


@dataclass
class Corridor:
    left: Boundary
    right: Boundary
    low: float
    high: float
    state: str = 'paired'
    prediction_age_s: float = 0.
    uncertainty: float = 0.

    def bounds(self, y):
        y=float(y)
        if not np.isfinite(y) or y<self.low-1e-9 or y>self.high+1e-9:
            raise ValueError('目标高度不在白线实际观测范围')
        y=min(self.high,max(self.low,y))
        return [float(self.left.at(y)), float(self.right.at(y))]


def fit_boundary(points, shape, *, residual_limit=8., minimum_span=None):
    h, w = shape[:2]
    a = np.asarray(points, float)
    if (a.ndim != 2 or a.shape[1] != 2 or len(a) < 6
            or not np.isfinite(a).all()):
        raise ValueError('白线点不足')
    a = a[np.argsort(a[:, 1])]
    if np.any(np.diff(a[:, 1]) <= 0):
        raise ValueError('白线采样高度重复或倒序')
    span = float(np.ptp(a[:, 1]))
    if span < (h * .10 if minimum_span is None else minimum_span):
        raise ValueError('白线可见范围不足')
    if np.max(np.diff(a[:, 1])) > h * .06:
        raise ValueError('白线采样存在过大空缺')
    coefficients = np.polyfit(a[:, 1] / h, a[:, 0], 2)
    residual = float(np.max(np.abs(np.polyval(coefficients, a[:, 1] / h) - a[:, 0])))
    if residual > residual_limit:
        raise ValueError('白线曲线残差过大')
    ys = np.array([a[0, 1], a[-1, 1]])
    slopes = np.polyval(np.polyder(coefficients), ys / h) / h
    # The previous straight detector allowed slopes up to six. Do not impose
    # opposite slope signs: both edges can lean to one side during a turn.
    if np.max(np.abs(slopes)) > 6 or abs(float(slopes[1] - slopes[0])) > 4:
        raise ValueError('白线曲率或局部斜率异常')
    return Boundary(coefficients, h, float(a[0, 1]), float(a[-1, 1]), residual)


def _check_pair(left, right, shape, *, short=False, observed=True):
    h, w = shape[:2]
    low, high = max(left.low, right.low), min(left.high, right.high)
    if high - low < h * (.09 if short else .10):
        raise ValueError('左右白线共同可见范围不足')
    ys = np.linspace(low, high, 9)
    lx, rx = left.at(ys), right.at(ys)
    widths = rx - lx
    if np.any(widths <= w * .15) or np.any(widths >= w * 1.8):
        raise ValueError('白线宽度异常')
    if observed and (np.any(lx < 0) or np.any(rx >= w)):
        raise ValueError('实测白线曲线越出画面')
    # Perspective width can change with height, but cannot abruptly form a
    # bulge between otherwise matching sides. No left/right centre split.
    linear_width = np.linspace(widths[0], widths[-1], len(widths))
    if np.max(np.abs(widths - linear_width)) > max(8., w * .06):
        raise ValueError('左右白线曲率不一致')
    return Corridor(left, right, low, high)


def measured_corridor(context, shape):
    if not context.get('valid'):
        raise ValueError('当前双边白线未确认')
    short = bool(context.get('boundary_diagnostics', {}).get('short_span_checked'))
    minimum_span = shape[0] * (.09 if short else .10)
    left = fit_boundary(context.get('left_inner', []), shape, minimum_span=minimum_span)
    right = fit_boundary(context.get('right_inner', []), shape, minimum_span=minimum_span)
    return _check_pair(left, right, shape, short=short)


def recover_single(context, shape, captured_s, paired, *, now_s=None):
    """Associate one current stripe to a recent actual pair, without refreshing it."""
    if not paired or not context.get('turn_aware') or not context.get('ground_supported'):
        raise ValueError('无可信同路单边恢复参考')
    reference_s, old = paired
    age = max(captured_s, captured_s if now_s is None else now_s) - reference_s
    if not 0 < age <= SINGLE_MAX_AGE_S + 1e-9:
        raise ValueError('单边恢复超过 0.25 秒')
    h, w = shape[:2]
    matches = []
    for candidate in context.get('boundary_candidates', [])[:12]:
        for side, previous, key in (('left', old.left, 'right_edge'), ('right', old.right, 'left_edge')):
            try:
                current = fit_boundary(candidate.get(key, []), shape, residual_limit=max(3., w * .01))
            except ValueError:
                continue
            low, high = max(current.low, old.low), min(current.high, old.high)
            if high - low < h * .10:
                continue
            ys = np.linspace(low, high, 5)
            width = old.right.at(ys) - old.left.at(ys)
            shifts = current.at(ys) - previous.at(ys)
            shift = float(np.median(shifts))
            if (abs(shift) > max(18., float(np.min(width)) * .18)
                    or np.max(np.abs(shifts - shift)) > max(6., w * .025)):
                continue
            other = old.right if side == 'left' else old.left
            # Reconstruct width relative to the *current* observed edge. The
            # old side itself is not copied into the new image.
            coefficients = current.coefficients + other.coefficients - previous.coefficients
            predicted = Boundary(coefficients, h, low, high, current.residual)
            left, right = (current, predicted) if side == 'left' else (predicted, current)
            try:
                model = _check_pair(left, right, shape, observed=False)
            except ValueError:
                continue
            ys2 = np.linspace(model.low, model.high, 5)
            if np.any(model.left.at(ys2) < -w * .25) or np.any(model.right.at(ys2) > w * 1.25):
                continue
            model.state = 'single'
            model.prediction_age_s = age
            model.uncertainty = min(.20, .03 + .08 * age / SINGLE_MAX_AGE_S
                + float(np.max(np.abs(shifts - shift))) / float(np.min(width)))
            matches.append((side, model))
    # More than one plausible stripe/identity is not a single trustworthy edge.
    if len(matches) != 1:
        raise ValueError('当前单边身份不唯一或未匹配')
    return matches[0][1]


def _paint_paths(mask, config, *, diagnostics=None):
    """Associate measured paint runs from near ground towards the horizon.

    Each current run retains at most two smooth histories, rather than letting
    the first far-background path consume a nearer tape run. All stored points
    are current mask observations; skipped rows never receive predicted pixels.
    """
    h, w = mask.shape
    step = max(2, round(h / 120))
    minimum, maximum = max(2, round(w * config.min_run_fraction)), round(w * config.max_run_fraction)
    active, finished = [], []
    stats={'row_run_count':0,'association_comparisons':0,'association_fit_attempts':0,
        'peak_active_paths':0,'association_budget_exhausted':False,
        'max_total_runs':256,'max_association_comparisons':8192,'max_association_fit_attempts':192}
    if diagnostics is not None:diagnostics.update(stats)
    def exhausted():
        stats['association_budget_exhausted']=True
        if diagnostics is not None:diagnostics.update(stats)
        return []
    def smooth(path):
        # One small least-squares solve evaluates both paint edges together.
        # This is an association prefilter only; every returned candidate and
        # pair still goes through the original fit_boundary/_check_pair below.
        points=np.asarray(path,float);ys=points[:,0]
        scale=float(np.ptp(ys))/2.;z=(ys-float(np.mean(ys)))/scale
        design=np.column_stack((z*z,z,np.ones(len(z))))
        edges=points[:,1:]-np.array([0.,1.])
        coefficients=np.linalg.lstsq(design,edges,rcond=None)[0]
        if np.max(np.abs(design@coefficients-edges))>max(3.,w*.012):return False
        slopes=(2.*z[[0,-1],None]*coefficients[0]+coefficients[1])/scale
        return bool(np.max(np.abs(slopes))<=6 and np.max(np.abs(slopes[1]-slopes[0]))<=4)
    def rank(item):
        path,cost=item
        return (len(path),path[0][0]-path[-1][0],-cost/max(1,len(path)-1),-path[0][1])
    def same(first,second):
        a={y:(l+r)/2 for y,l,r in first}
        b={y:(l+r)/2 for y,l,r in second}
        common=a.keys()&b.keys()
        if len(common)<max(6,min(len(a),len(b))*.5):return False
        errors=sorted(abs(a[y]-b[y]) for y in common)
        return errors[len(errors)//2]<=max(2.5,w*.006) and errors[round((len(errors)-1)*.9)]<=4.
    def trim_finished():
        unique=[]
        for item in sorted(finished,key=rank,reverse=True):
            if not any(same(item[0],old[0]) for old in unique):unique.append(item)
            if len(unique)==32:break
        finished[:]=unique
    rows=range(round(h * config.roi_top), round(h * .90), step)
    for y in reversed(rows):
        row = (mask[max(0, y - 1):min(h, y + 2)] > 0).mean(axis=0) >= .5
        changes = np.diff(np.pad(row.astype(np.int8), (1, 1)))
        runs = [(int(a), int(b)) for a, b in zip(np.where(changes == 1)[0], np.where(changes == -1)[0])
                if minimum <= b - a <= maximum]
        stats['row_run_count']+=len(runs)
        if len(runs)>24 or stats['row_run_count']>stats['max_total_runs']:return exhausted()
        keep = []
        for path,cost in active:
            if path[-1][0]-y<=h*.06:keep.append((path,cost))
            else:finished.append((path,cost))
        active = keep
        proposals=[];extended=set()
        for a,b in runs:
            links=[]
            for i,(path,cost) in enumerate(active):
                if stats['association_comparisons']>=stats['max_association_comparisons']:return exhausted()
                stats['association_comparisons']+=1
                last=path[-1];dy=last[0]-y;center=(last[1]+last[2])/2
                slope=0.
                if len(path)>=2:
                    previous=path[max(0,len(path)-4)]
                    slope=(center-(previous[1]+previous[2])/2)/(last[0]-previous[0])
                distance=abs((a+b)/2-(center-slope*dy))
                if (distance<=max(7.,6*dy)
                        and max(b-a,last[2]-last[1])<=2.5*min(b-a,last[2]-last[1])):
                    candidate=(path+[(y,a,b)],cost+distance/max(2.,b-a)+(dy/step-1)*.25)
                    links.append((rank(candidate),i,candidate))
            accepted=0
            for _,i,item in sorted(links,key=lambda value:value[0],reverse=True)[:4]:
                path,cost=item
                if len(path)>=8 and path[0][0]-y>=h*.09:
                    if stats['association_fit_attempts']>=stats['max_association_fit_attempts']:return exhausted()
                    stats['association_fit_attempts']+=1
                    if not smooth(path):
                        finished.append(active[i]);continue
                proposals.append(item);extended.add(i);accepted+=1
                if accepted==2:break
            # Independent starts remain possible when all longer histories
            # were background or a stripe genuinely first enters the image.
            proposals.append(([(y,a,b)],0.))
        proposals.extend(item for i,item in enumerate(active) if i not in extended)
        unique=[];seen=set()
        for item in sorted(proposals,key=rank,reverse=True):
            key=tuple(item[0])
            if key not in seen:unique.append(item);seen.add(key)
            if len(unique)==32:break
        active=unique
        stats['peak_active_paths']=max(stats['peak_active_paths'],len(active))
        if len(finished)>64:trim_finished()
    finished.extend(active)
    trim_finished()
    if diagnostics is not None:diagnostics.update(stats)
    return [list(reversed(path)) for path,cost in finished[:24]]


def curve_inner_edges(image, obstacle_mask, config, *, prior=None):
    h, w = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, config.white_v_min), (179, config.white_s_max, 255))
    mask[obstacle_mask > 0] = 0
    mask[:round(h * config.roi_top)] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    candidates = []
    association={}
    paths=_paint_paths(mask,config,diagnostics=association)
    paint_checked=paint_rejected=paint_no_flank_rows=0;paint_support=[]
    for path in paths:
        if len(path) < 8:
            continue
        left = [[float(a), float(y)] for y, a, b in path]
        right = [[float(b - 1), float(y)] for y, a, b in path]
        try:
            fit_boundary(left, image.shape, residual_limit=max(3., w * .012), minimum_span=h * .09)
            fit_boundary(right, image.shape, residual_limit=max(3., w * .012), minimum_span=h * .09)
        except ValueError:
            continue
        # A low-S/V mask can cut a narrow strip out of a smooth floor glare.
        # Require a current bright ridge relative to BOTH observed flanks,
        # rather than treating every threshold-shaped strip as white tape.
        points=np.asarray(path,int);ys,a,b=points[:,0],points[:,1],points[:,2]-1
        offsets=np.arange(int(np.max(b-a))+1)
        inside_x=a[:,None]+offsets
        inside=np.where(inside_x<=b[:,None],hsv[ys[:,None],np.clip(inside_x,0,w-1),2],np.nan)
        contrasts=[]
        for side in (-1,1):
            xs=(a if side<0 else b)[:,None]+side*np.arange(3,8)
            valid=(xs>=0)&(xs<w)
            values=np.where(valid,hsv[ys[:,None],np.clip(xs,0,w-1),2],np.nan)
            # A stripe at the image edge has only one observed floor flank.
            median=np.full(len(path),-np.inf)
            present=valid.any(axis=1)
            median[present]=np.nanmedian(values[present],axis=1)
            contrasts.append(median)
        flank=np.maximum(*contrasts);observed=np.isfinite(flank)
        paint_no_flank_rows+=int(np.count_nonzero(~observed))
        contrast=np.where(observed,np.nanmedian(inside,axis=1)-flank,-np.inf)
        support=float(np.mean(observed&(contrast>=10.)))
        paint_checked+=1
        if support<.70:
            paint_rejected+=1;continue
        paint_support.append({'supported_row_fraction':support,
                              'median_contrast_v':float(np.median(contrast))})
        candidates.append({'left_edge': left, 'right_edge': right})
        if len(candidates) == 12:
            break
    pairs = []
    for i, first in enumerate(candidates):
        for second in candidates[i + 1:]:
            # Determine left/right by their actual common rows, not image centre.
            one = {p[1]: (l[0], p[0]) for l, p in zip(first['left_edge'], first['right_edge'])}
            two = {p[1]: (l[0], p[0]) for l, p in zip(second['left_edge'], second['right_edge'])}
            ys = sorted(one.keys() & two.keys())
            if len(ys) < 8 or ys[-1] - ys[0] < h * .09:
                continue
            if np.median([sum(one[y]) - sum(two[y]) for y in ys]) > 0:
                one, two = two, one
            left = [[one[y][1], y] for y in ys]
            right = [[two[y][0], y] for y in ys]
            widths = np.asarray(right)[:, 0] - np.asarray(left)[:, 0]
            if np.any(widths <= w * config.min_width_fraction) or np.any(widths >= w * config.max_width_fraction):
                continue
            span = ys[-1] - ys[0];density = len(ys) / (span / max(2, round(h / 120)) + 1)
            short = span < h * .12
            if density < (.8 if short else .65) or (short and len(ys) < 10):
                continue
            try:
                lc = fit_boundary(left, image.shape, residual_limit=max(3., w * .012), minimum_span=h * .09)
                rc = fit_boundary(right, image.shape, residual_limit=max(3., w * .012), minimum_span=h * .09)
                _check_pair(lc, rc, image.shape, short=short)
            except ValueError:
                continue
            if short:
                # Preserve the extra evidence for a very short observed band.
                if widths[-1] - widths[0] < span * .5 or max(lc.residual, rc.residual) > max(2., w * .005):
                    continue
            score = (len(ys), span, -max(lc.residual, rc.residual))
            pairs.append((score,left,right,short,max(lc.residual,rc.residual),lc,rc))
    diagnostics = {'detector': 'turn_aware_curves', 'candidate_count': len(candidates),
        'support_rows': 0, 'roi_top': config.roi_top, 'white_v_min': config.white_v_min,
        'white_s_max': config.white_s_max, 'max_candidates': 12,
        'fit_residual_limit_px':max(3.,w*.012),
        'association_policy':'near_to_far_bounded_rows','max_active_paths':32,
        'max_runs_per_row':24,'max_branches_per_run':2,'max_pair_combinations':66,
        'path_count':len(paths),'pair_combinations':len(candidates)*(len(candidates)-1)//2,
        'valid_pair_count':len(pairs),'prior_used':False,'prior_reason':'not_provided',**association}
    diagnostics.update(paint_evidence_policy='current_bright_ridge_both_flanks',
        paint_min_contrast_v=10.,paint_min_supported_row_fraction=.70,
        paint_paths_checked=paint_checked,paint_paths_rejected=paint_rejected,
        paint_rows_without_flanks=paint_no_flank_rows,paint_candidate_support=paint_support)
    if association.get('association_budget_exhausted'):
        return [],[],mask,{**diagnostics,'reason':'row_association_budget_exhausted'},[]
    if not pairs:
        return [], [], mask, {**diagnostics, 'reason': 'no_supported_curve_pair'}, candidates
    best=max(pairs,key=lambda pair:pair[0])
    if prior is not None:
        diagnostics['prior_reason']='invalid_prior'
        try:
            old_left=fit_boundary(prior['left_inner'],image.shape,
                residual_limit=max(3.,w*.012),minimum_span=h*.09)
            old_right=fit_boundary(prior['right_inner'],image.shape,
                residual_limit=max(3.,w*.012),minimum_span=h*.09)
            old=_check_pair(old_left,old_right,image.shape,
                short=min(old_left.high,old_right.high)-max(old_left.low,old_right.low)<h*.12)
            qualified=[]
            for pair in pairs:
                # History only ranks similarly strong CURRENT, fully accepted
                # pairs. A clearly stronger current observation takes over.
                if pair[0][0]<best[0][0]-1 or pair[0][1]<best[0][1]-max(2,round(h/120)):continue
                low=max(old.low,pair[5].low,pair[6].low)
                high=min(old.high,pair[5].high,pair[6].high)
                if high-low<h*.09:continue
                ys=np.linspace(low,high,9)
                error=float(np.mean(np.maximum(np.abs(pair[5].at(ys)-old_left.at(ys)),
                                                np.abs(pair[6].at(ys)-old_right.at(ys)))))
                qualified.append((error,pair))
            diagnostics['prior_reason']='no_common_observed_band'
            if qualified:
                error,choice=min(qualified,key=lambda item:(item[0],tuple(-v for v in item[1][0])))
                if len(qualified)>1:
                    best=choice;diagnostics['prior_used']=True
                    diagnostics['prior_reason']='ranked_similar_current_pairs'
                else:diagnostics['prior_reason']='only_one_qualified_current_pair'
                diagnostics['prior_match_error_px']=error
        except (KeyError,TypeError,ValueError,IndexError):pass
    _, left, right, short, residual,_,_ = best
    diagnostics.update(support_rows=len(left), short_span_checked=short,
        fit_max_residual_px=residual, common_span_px=left[-1][1] - left[0][1], reason='observed_curve_pair')
    return left, right, mask, diagnostics, candidates
