"""Configured track context and observed WHITE INNER edges; no metric guessing."""
import cv2
import numpy as np
import copy
from dataclasses import replace

from .config import load_config
from .lane import LaneDetector
from .cone_boundaries import adaptive_inner_edges
from .cone_corridor import curve_inner_edges,fit_boundary,measured_corridor
from .cone_vision import obstacle_color
from .brake_output import numeric

# Legacy surfaces keep three additional passes. Indoor turn guidance may make
# one final 160 candidate pass; saved settings and geometric checks stay fixed.
WHITE_THRESHOLD_FALLBACKS = (220, 200, 180)


def red_mask(image):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    return cv2.bitwise_or(cv2.inRange(hsv,(0,55,35),(15,255,255)),
                         cv2.inRange(hsv,(165,55,35),(179,255,255)))


def blue_ground_mask(image):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv,(90,55,35),(135,255,255))


def inner_edge(mask,x,y,direction):
    """Move from the observed paint run towards the driving corridor."""
    h,w=mask.shape;yi=int(round(y));xi=int(round(x))
    if not 0<=yi<h:return None
    row=(mask[max(0,yi-1):min(h,yi+2)]>0).mean(axis=0)>=.5
    indices=np.flatnonzero(row[max(0,xi-8):min(w,xi+9)])+max(0,xi-8)
    if not len(indices):return None
    cursor=int(min(indices,key=lambda q:abs(q-xi)))
    while 0<=cursor<w and row[cursor]:cursor+=direction
    if not 0<=cursor<w:return None
    return [float(cursor),float(y)]


class ConeRoadContext:
    def __init__(self,settings,camera='secondary',*,thorough_boundary_search=False,turn_aware=False):
        self.thorough_boundary_search=bool(thorough_boundary_search)
        self.turn_aware=bool(turn_aware)
        self.camera=camera;self._previous_frame=None;self._pair_prior=None
        self.s=settings;config=load_config(None).lane
        # The generic preview threshold also includes pale green floor glare.
        # In the cone corridor that merges the left tape with the floor, so
        # it is discarded as an over-wide paint run. Require brighter, less
        # saturated paint here; leave the manual/traffic preview untouched.
        # Each fixed camera has its own ground ROI; cabinet/window edges above
        # that ROI must never veto the other camera as a white-line hazard.
        threshold=settings.get('scene',{}).get('white_v_min',200)
        if type(threshold) is not int or not 0<=threshold<=255:raise ValueError('白线亮度阈值必须为 0–255 的整数')
        config=replace(config,white_v_min=threshold,white_s_max=70,
                       roi_top=.60 if camera=='primary' else .38)
        if settings.get('scene',{}).get('surface')=='indoor_white_line_track':
            # In the fixed H65 view the nearby white edges leave the image
            # above the car bonnet. Fit the observed mid-ground, not the bonnet.
            # Metric projection still requires >=6 calibrated edge points,
            # >=0.3 m span, measured width and the original fit error limits.
            config=replace(config,sample_top=.50,sample_bottom=.76,lookahead_y=.63)
            if camera=='primary':
                # The lower camera sees the track later in the image. Keep its
                # own ROI and the original paint-width/inner-edge checks rather
                # than treating broad floor glare as a white boundary.
                config=replace(config,sample_top=.60,sample_bottom=.85,lookahead_y=.70)
            if self.turn_aware:
                # The two fixed views see the same tape at different heights.
                # These ROIs exclude the windows, without cropping the low
                # camera's actual mid-ground paint out of the image.
                roi=.38 if camera=='primary' else .53
                config=replace(config,roi_top=roi,sample_top=max(config.sample_top,roi))
        self.lane=LaneDetector(config)

    def _history(self,image,frame_id,captured_s):
        shape=tuple(image.shape[:2]);previous=self._previous_frame
        size=self.s.get('image_size')
        expected=None if not isinstance(size,(list,tuple)) or len(size)!=2 else tuple(reversed(size))
        metadata=(type(frame_id) is int and frame_id>=0 and numeric(captured_s)
                  and (expected is None or shape==expected))
        if not metadata:
            self._previous_frame=self._pair_prior=None
            return None,False,'missing_or_invalid_frame_metadata'
        current=(frame_id,captured_s,shape)
        increasing=(previous is None or frame_id>previous[0] and captured_s>previous[1])
        continuous=(previous is None or shape==previous[2] and captured_s-previous[1]<=.25)
        if not increasing or not continuous:
            self._pair_prior=None
            # A camera restart starts a new epoch. Its first frame earns no
            # history, but must not leave later frames locked behind old IDs.
            self._previous_frame=current
            return None,False,'nonincreasing_or_discontinuous_frame'
        self._previous_frame=current
        prior=self._pair_prior
        usable=bool(prior and prior['shape']==shape and prior['camera']==self.camera
                    and prior['frame_id']<frame_id and 0<captured_s-prior['captured_s']<=.25)
        if not usable:self._pair_prior=None
        return copy.deepcopy(prior) if usable else None,True,'current_frame'

    @staticmethod
    def _candidate_models(candidate,shape):
        try:
            return tuple(fit_boundary(candidate[key],shape,minimum_span=shape[0]*.09)
                         for key in ('left_edge','right_edge'))
        except (ValueError,KeyError,TypeError):return None

    def _merge_candidates(self,results,shape,prior):
        """Rank/deduplicate measured stripes, never union them into a road."""
        prior_models=None
        if prior:
            try:prior_models=tuple(fit_boundary(prior[key],shape)
                                   for key in ('left_inner','right_inner'))
            except (ValueError,KeyError,TypeError):pass
        ranked=[]
        for result,threshold in results:
            if (not result.get('ground_supported')
                    or result.get('boundary_diagnostics',{}).get('association_budget_exhausted')):continue
            for candidate in result.get('boundary_candidates',[])[:12]:
                models=self._candidate_models(candidate,shape)
                if models is None:continue
                left,right=models;span=min(left.high,right.high)-max(left.low,right.low)
                distance=float('inf')
                if prior_models:
                    for current,previous in ((right,prior_models[0]),(left,prior_models[1])):
                        low,high=max(current.low,previous.low),min(current.high,previous.high)
                        if high-low>=shape[0]*.09:
                            ys=np.linspace(low,high,5)
                            distance=min(distance,float(np.max(np.abs(current.at(ys)-previous.at(ys)))))
                ranked.append(((distance,-span,-len(candidate['left_edge']),-threshold),candidate,models))
        chosen=[]
        for _,candidate,models in sorted(ranked,key=lambda row:row[0]):
            duplicate=False
            for _,old in chosen:
                low=max(models[0].low,models[1].low,old[0].low,old[1].low)
                high=min(models[0].high,models[1].high,old[0].high,old[1].high)
                if high-low<shape[0]*.09:continue
                ys=np.linspace(low,high,5)
                if max(float(np.max(np.abs(a.at(ys)-b.at(ys)))) for a,b in zip(models,old))<=3.:
                    duplicate=True;break
            if not duplicate:chosen.append((copy.deepcopy(candidate),models))
            if len(chosen)>=12:break
        return [candidate for candidate,_ in chosen]

    def inspect(self,image,blue_mask,*,frame_id=None,captured_s=None):
        prior,history_ok,history_reason=(self._history(image,frame_id,captured_s)
            if self.turn_aware else (None,False,'legacy_current_image_only'))
        configured=self.lane.config.white_v_min
        fallbacks=WHITE_THRESHOLD_FALLBACKS
        if (self.turn_aware and self.s.get('scene',{}).get('surface')=='indoor_white_line_track'
                and configured>=160):fallbacks=fallbacks+(160,)
        fallbacks=tuple(value for value in fallbacks if value<configured)
        result=self._inspect_threshold(image,blue_mask,self.lane,prior=prior)
        candidates=[(result,configured)]
        attempts=[{'white_v_min':configured,'valid':result['valid'],
                   'white_lines_detected':result['white_lines_detected'],'reason':result['reason']}]
        effective=configured
        single_candidate=None
        # A detected pair rejected by the ground check must stay rejected;
        # lowering brightness is a remedy for missing paint, not missing road.
        if not result['white_lines_detected']:
            for threshold in fallbacks:
                lane=LaneDetector(replace(self.lane.config,white_v_min=threshold))
                candidate=self._inspect_threshold(image,blue_mask,lane,prior=prior)
                candidates.append((candidate,threshold))
                attempts.append({'white_v_min':threshold,'valid':candidate['valid'],
                                 'white_lines_detected':candidate['white_lines_detected'],
                                 'reason':candidate['reason']})
                if candidate['valid']:
                    result=candidate;effective=threshold;break
                if (self.turn_aware and candidate.get('boundary_candidates')
                        and not candidate.get('boundary_diagnostics',{}).get('association_budget_exhausted')
                        and candidate.get('ground_supported')):
                    single_candidate=(candidate,threshold)
            if not result['valid'] and single_candidate:
                result,effective=single_candidate
        if self.turn_aware:
            result['boundary_candidates']=self._merge_candidates(candidates,image.shape,prior)
            result['boundary_diagnostics'].update(merged_candidate_count=len(result['boundary_candidates']),
                candidate_thresholds=[threshold for item,threshold in candidates if item.get('boundary_candidates')
                    and not item.get('boundary_diagnostics',{}).get('association_budget_exhausted')],
                history_prior_available=prior is not None,history_reason=history_reason)
            saved=False
            if history_ok and result['valid']:
                try:measured_corridor(result,image.shape)
                except ValueError:pass
                else:self._pair_prior={'camera':self.camera,'shape':tuple(image.shape[:2]),
                    'frame_id':frame_id,'captured_s':captured_s,
                    'left_inner':copy.deepcopy(result['left_inner']),
                    'right_inner':copy.deepcopy(result['right_inner'])}
                saved=self._pair_prior is not None and self._pair_prior['frame_id']==frame_id
            result['boundary_diagnostics']['history_pair_saved']=saved
        result['boundary_diagnostics'].update(configured_white_v_min=configured,
            effective_white_v_min=effective,threshold_fallback_used=effective!=configured,
            threshold_attempts=attempts,max_threshold_attempts=1+len(fallbacks))
        return result

    def _inspect_threshold(self,image,blue_mask,detector,*,prior=None):
        """Select the explicit cone model, then apply the shared ground check."""
        if self.turn_aware:
            left,right,white,diagnostics,candidates=curve_inner_edges(image,blue_mask,detector.config,prior=prior)
            if diagnostics.get('association_budget_exhausted'):
                left,right,candidates=[],[],[]
            enough=bool(left)
            failure_reason=diagnostics['reason']
        else:
            road=image.copy();road[blue_mask>0]=0
            lane,white=detector.detect(road)
            candidates=[]
            left=[];right=[]
            if lane.valid:
                for lp,rp in zip(lane.left,lane.right):
                    l=inner_edge(white,*lp,1);r=inner_edge(white,*rp,-1)
                    if l and r and l[0]<r[0]:left.append(l);right.append(r)
            enough=lane.valid and len(left)>=max(6,len(lane.left)*.65)
            diagnostics={'detector':'fixed_band','baseline_reason':lane.reason,'support_rows':len(left),
                         'roi_top':detector.config.roi_top,'white_v_min':detector.config.white_v_min,
                         'white_s_max':detector.config.white_s_max}
            if not enough or self.thorough_boundary_search:
                baseline_diagnostics=diagnostics
                adaptive_left,adaptive_right,adaptive_white,diagnostics=adaptive_inner_edges(
                    image,blue_mask,detector.config,complete_small_sets=self.thorough_boundary_search)
                diagnostics['baseline_reason']=lane.reason
                if adaptive_left:
                    left,right,white=adaptive_left,adaptive_right,adaptive_white;enough=True
                elif enough:
                    diagnostics={**baseline_diagnostics,'adaptive_failure':diagnostics['reason']}
            failure_reason=lane.reason if not lane.valid else 'white_inner_edges_incomplete'
        red=red_mask(image)
        region=np.zeros(image.shape[:2],np.uint8)
        if enough:
            cv2.fillPoly(region,[np.asarray(left+right[::-1],np.int32)],255)
            support=(region>0)&(blue_mask==0)&(white==0)
        else:
            support=np.zeros(image.shape[:2],bool)
            support[int(image.shape[0]*.55):]=True
            support&=(blue_mask==0)&(white==0)
        ratio=float(np.mean(red[support]>0)) if support.any() else 0.
        threshold=self.s.get('scene',{}).get('red_ground_min_fraction',.15)
        red_supported=ratio>=threshold
        surface=self.s.get('scene',{}).get('surface','red_running_track')
        ground_color={'red_running_track':'red','blue_running_track':'blue'}.get(surface)
        blue_ratio=(float(np.mean(blue_ground_mask(image)[support]>0))
                    if ground_color=='blue' and support.any() else 0.)
        ground_ratio=blue_ratio if ground_color=='blue' else ratio
        ground_supported=ground_color is None or ground_ratio>=threshold
        valid=bool(enough and ground_supported)
        reason=failure_reason
        if valid:reason='white_inner_edges_on_'+ground_color+'_track' if ground_color else 'white_inner_edges_on_indoor_track'
        elif enough and ground_color:reason=ground_color+'_ground_not_confirmed'
        return {'valid':valid,'white_lines_detected':bool(enough),'red_ground_supported':red_supported,
                'boundary_diagnostics':diagnostics,
                'red_ground_required':ground_color=='red',
                'ground_color':ground_color,'ground_supported':ground_supported,
                'ground_fraction':ground_ratio,'blue_ground_fraction':blue_ratio,
                'obstacle_color':obstacle_color(self.s),
                'red_ground_fraction':ratio,'left_inner':left if enough else [],'right_inner':right if enough else [],
                'reason':reason,
                'boundary_policy':'whole_vehicle_inside_white_inner_edges',
                'metric_boundary_verified':False,
                **({'turn_aware':True,'boundary_candidates':candidates} if self.turn_aware else {})}

    @staticmethod
    def enrich(detections,image,context):
        red=red_mask(image);h,w=red.shape
        color=context.get('ground_color','red' if context.get('red_ground_required',True) else None)
        ground=blue_ground_mask(image) if color=='blue' else red
        for row in detections:
            x1,y1,x2,y2=row['bbox_xyxy'];pad=max(3,round((x2-x1)*.25))
            patch=red[max(0,y2):min(h,y2+pad),max(0,x1-pad):min(w,x2+pad)]
            row['red_ground_near_base']=float(np.mean(patch>0)) if patch.size else 0.
            patch=ground[max(0,y2):min(h,y2+pad),max(0,x1-pad):min(w,x2+pad)]
            row['ground_near_base']=float(np.mean(patch>0)) if patch.size else 0.
            row['boundary_relation']='unknown'
            # Top-view/occluded bases have no verified contact point. Do not
            # pretend a bounding-box point is a calibrated ground position.
            if context['valid'] and row['ground_contact_usable']:
                left,right=context['left_inner'],context['right_inner'];x,y=row['foot_pixel']
                if left[0][1]<=y<=left[-1][1]:
                    lo=float(np.interp(y,[p[1] for p in left],[p[0] for p in left]))
                    hi=float(np.interp(y,[p[1] for p in right],[p[0] for p in right]))
                    row['boundary_relation']='inside' if lo<x<hi else 'outside'
            row['scene_supported']=row['boundary_relation']=='inside' and (
                color is None or row['ground_near_base']>=.15)
        return detections
