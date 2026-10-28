"""Supervised image-feedback trial using historical, NOT verified, estimates.

No new calibration or verified flags. White-line proximity steers inward;
body-clearance, obstacle and lease guards still stop motion. Ordered image
evidence permits estimated pass handoff; insufficient evidence needs review.
"""

from .steering import LEFT_US, RIGHT_US, radius_settings
import copy
import math
import json
from pathlib import Path

import cv2
import numpy as np

from .autonomy_profile import GroundProjection
from .brake_parking import numeric
from .cone_trial_output import validate_limits
from .cone_corridor import SINGLE_MAX_AGE_S,measured_corridor,recover_single
from .cone_evidence import StablePairedLane

PROFILE='legacy_low_speed'
DEFAULT_STEERING_GAIN=1.5
LINE_RECOVERY_S=.4


def _identity_evidence(track):
    revision=track.get('identity_revision')
    if type(revision) is int and revision>=0:
        result={'identity_revision':revision,'identity_recovered':track.get('identity_recovered') is True}
        if isinstance(track.get('identity_recovery'),dict):
            result['identity_recovery']=copy.deepcopy(track['identity_recovery'])
        return result
    return {}


def steering_gain(value):
    if not numeric(value) or not 1.<=value<=3.:
        raise ValueError('绕桶转向增益仅允许 1.0–3.0')
    return float(value)


def trial_settings(settings,payload=None):
    payload=payload or {};s=copy.deepcopy(settings);g=s['geometry'];ground=s['ground']
    if not ground.get('historical_verified') or not ground.get('reuse_source'):
        raise ValueError('请先载入已有 GitHub 标定参考；本入口不要求重新采集')
    GroundProjection(ground,require_verified=False)
    for key in ('lane_width_m','vehicle_width_m','vehicle_length_m','rear_axle_to_front_m','rear_overhang_m','cone_base_width_m'):
        if not numeric(g.get(key)) or g[key]<=0:raise ValueError('已有车身尺寸缺失：'+key)
    if g['lane_width_m']<=g['vehicle_width_m']+.24:raise ValueError('白线内宽不足以进行该低速试验')
    if 'test_burst_s' in payload:raise ValueError('旧分段测试页面已过期，请刷新后使用持续前进入口')
    forward=payload.get('test_forward_us',1600);duration=payload.get('test_run_s',60.)
    left=payload.get('test_left_us',LEFT_US);right=payload.get('test_right_us',RIGHT_US)
    gain=steering_gain(payload.get('test_steering_gain',DEFAULT_STEERING_GAIN))
    validate_limits(right,left)
    if type(forward) is not int or not 1501<=forward<=1625:raise ValueError('测试油门仅允许 1501–1625 μs')
    if not numeric(duration) or not 5<=duration<=120:raise ValueError('本轮运行时限仅允许 5–120 秒')
    s['driving']=radius_settings(s.get('driving',{}));s['driving']['forward_us']=forward
    s['low_speed_test']={'drive_mode':'continuous','settle_s':.55,'max_run_s':duration,'reaction_horizon_s':.25,
        'left_us':left,'right_us':right,'steering_gain':gain,'turn_margin_m':.03,
        'reported_braking_m':.17,'stopping_budget_m':.27,'reported_turn_radius_m':12.03,
        'reported_displacement_m':.63,'reported_drive_s':1.5,'estimate_verified':False,
        'tail_powered_limit_s':1.0,'clearance_m':.05,'boundary_soft_fraction':.06,
        'boundary_release_fraction':.04,'boundary_hard_margin_m':.03}
    return s


class LowSpeedNavigator:
    def __init__(self,settings,camera='secondary'):
        self.s=settings;self.latest=None;self.last_id=None
        self.near_history={}
        self.outside_since=None;self.outside_frames=0;self.last_boundary_s=None
        self.camera=camera
        self.projection=GroundProjection(settings['ground'],require_verified=False)
        self.paired_corridor=None;self.last_captured_s=None;self.last_turn_aware=None

    def _visual_cones(self,row,w,h):
        """Current confirmed same-view pixels, never a projected ground point.

        This list may continue an already locked cone beyond the ground ROI.
        It does not authorize acquiring a new target or claiming a clear path.
        """
        output=[]
        for track in row.get('tracks',[]):
            identity=track.get('id')
            if (track.get('visible') is not True or track.get('confirmed') is not True
                    or track.get('seen_s')!=row['captured_s']
                    or not isinstance(identity,str) or not identity.startswith(self.camera+'-')):
                continue
            try:
                box=np.asarray(track.get('bbox_xyxy',[]),float)
                foot=np.asarray(track.get('foot_pixel',[]),float)
            except (TypeError,ValueError):continue
            if (box.shape!=(4,) or foot.shape!=(2,) or not np.isfinite(box).all()
                    or not np.isfinite(foot).all()):continue
            x1,y1,x2,y2=box;x,y=foot
            if not (0<=x1<x2<=w and 0<=y1<y2<=h and 0<=x<w and 0<=y<h):continue
            reference=None;H=self.projection.h
            if abs(H[0,0])>=1e-8:
                value=float(-(H[0,1]*y+H[0,2])/H[0,0]/w)
                if math.isfinite(value) and 0<=value<=1:reference=value
            output.append({'id':identity,'image_fraction':float(x/w),
                'half_image_width':float((x2-x1)/(2*w)),
                'bbox_xyxy':list(track['bbox_xyxy']),'y_fraction':float(y/h),
                'source_camera':self.camera,'frame_id':row['frame_id'],'captured_s':row['captured_s'],
                'clipped':bool(track.get('clipped')),'current_pixel':True,'confirmed':True,
                'image_reference_fraction':reference,
                'image_reference_source':'historical_X0_image_reference',**_identity_evidence(track)})
        output.sort(key=lambda item:item['y_fraction'],reverse=True)
        return output

    def update(self,image,row,now):
        fid=row['frame_id'];t=row['captured_s'];h,w=image.shape[:2]
        turn_aware=bool(row['road_context'].get('turn_aware'))
        if self.last_turn_aware is not None and turn_aware!=self.last_turn_aware:
            self.last_id=None;self.latest=None;self.last_captured_s=None
            self.paired_corridor=None;self.near_history={}
            self.outside_since=None;self.outside_frames=0;self.last_boundary_s=None
        self.last_turn_aware=turn_aware
        repeated=fid==self.last_id
        if repeated and not turn_aware:return self.latest
        if ((self.last_id is not None and fid<self.last_id)
                or (self.last_captured_s is not None and t<self.last_captured_s)):
            self.paired_corridor=None;self.near_history={}
            self.outside_since=None;self.outside_frames=0
        self.last_id=fid
        self.last_captured_s=t
        result={'experimental':True,'captured_s':t,'frame_id':fid,'ready':False,'action':'stop',
                'phase':'test_observation','hardware_output':False,'field_verified':False,
                'cones':[],'uncertain':[],'reason':'等待左右白线',
                'line_state':'missing','boundary_observed':False,'prediction_age_s':None,
                'boundary_uncertainty_fraction':None}
        if turn_aware:
            result['visual_cones']=[]
            result['tracking_diagnostics']=copy.deepcopy(row.get('tracking_diagnostics',{}))
            result['obstacle_diagnostics']=[]
            result['obstacle_diagnostics_omitted']=0
            result['boundary_diagnostics']=copy.deepcopy(row['road_context'].get('boundary_diagnostics',{}))
            result['boundary_diagnostics']['road_reason']=row['road_context'].get('reason')
        def fail(reason):
            result['boundary_failure_reason']=reason
            if turn_aware:
                for key in ('lane_bounds_px','lane_rows','ego_fraction','ego_near_fraction',
                            'ego_far_fraction','ego_range','heading_error'):
                    result.pop(key,None)
                result.update(line_state='missing',boundary_observed=False,line_held=False)
            if not result.get('boundary_danger') and 0<=now-t<=.25:
                # Losing segmentation is not losing the camera. Keep current
                # cone pixels, but never label image fractions as lane space.
                result.update(self.image_guidance(row,w,h),line_missing=True,guard='image_only_boundary_unknown')
                result.update(ready=True,action='visual_trial_ready')
                self.outside_since=None;self.outside_frames=0
            self.latest={**result,'reason':reason};return self.latest
        if not 0<=now-t<=.25:return fail('第二路图像过期')
        if repeated:
            if self.latest and self.latest.get('line_state')=='single':
                age=now-self.latest['line_observed_s']
                if age>SINGLE_MAX_AGE_S+1e-9:return fail('单边恢复超过 0.25 秒')
                self.latest=copy.deepcopy(self.latest)
                delta=max(0.,age-self.latest['prediction_age_s'])
                self.latest.update(prediction_age_s=age,line_age_s=age,
                    boundary_uncertainty_fraction=min(.20,self.latest['boundary_uncertainty_fraction']+.08*delta/SINGLE_MAX_AGE_S))
                self.latest['half_envelope']=self.latest['base_half_envelope']+self.latest['boundary_uncertainty_fraction']
                if self.latest['half_envelope']>=.5:
                    result['boundary_danger']=True
                    return fail('单侧白线预测不确定度占满车体净空，停止')
            return self.latest
        if turn_aware:result['visual_cones']=self._visual_cones(row,w,h)
        road=row['road_context']
        model=None
        if turn_aware:
            try:
                model=measured_corridor(road,image.shape)
            except ValueError as pair_error:
                try:model=recover_single(road,image.shape,t,self.paired_corridor,now_s=now)
                except ValueError as single_error:
                    return fail(str(pair_error)+'；'+str(single_error))
            y0,y1=model.low,model.high
            corridor=model.bounds
            result.update(line_state=model.state,boundary_observed=model.state=='paired',
                line_held=model.state=='single',prediction_age_s=model.prediction_age_s,
                boundary_uncertainty_fraction=model.uncertainty,
                line_age_s=model.prediction_age_s,
                line_observed_s=t if model.state=='paired' else self.paired_corridor[0])
            result['boundary_diagnostics'].update(model_observed_y_range=[float(y0),float(y1)],
                model_observation_only=True)
        else:
            if not road.get('valid'):return fail('白线暂未确认，继续低速前进搜索')
            short_pair=(road.get('boundary_diagnostics') or {}).get('short_span_checked') is True
            minimum_span=h*(.09 if short_pair else .10)
            edges=[]
            for key in ('left_inner','right_inner'):
                points=np.asarray(road.get(key,[]),float)
                if points.ndim!=2 or points.shape[0]<6 or points.shape[1]!=2 or not np.isfinite(points).all():
                    return fail('白线点不足')
                fit=np.polyfit(points[:,1],points[:,0],1)
                if np.ptp(points[:,1])<minimum_span or np.max(np.abs(np.polyval(fit,points[:,1])-points[:,0]))>8:
                    return fail('白线形状不稳定，仅支持直道低速试验')
                edges.append((fit,points[:,1].min(),points[:,1].max()))
            y0=max(p[1] for p in edges);y1=min(p[2] for p in edges)
            if y1-y0<minimum_span:return fail('左右白线共同可见范围不足')
            def corridor(y):return [float(np.polyval(e[0],y)) for e in edges]
            result.update(line_state='paired',boundary_observed=True,prediction_age_s=0.,
                boundary_uncertainty_fraction=0.)
        if turn_aware:
            # Curve validity and target lane coordinates end at the actual
            # joint observations. Current pixels remain in visual_cones.
            low,high=y0,y1
        else:
            # Preserve the legacy non-cone stage's original straight policy.
            low=max(h*.4,y0-h*.10);high=min(h*.78,y1+h*.14)
            if self.camera=='primary':low=max(h*.55,y0-h*.03);high=min(h*.97,y1+h*.20)
        g=self.s['geometry'];margin=.05+g['vehicle_length_m']*.10+.27*math.sin(math.radians(3))
        half=(g['vehicle_width_m']/2+margin)/g['lane_width_m']
        if turn_aware:
            result['base_half_envelope']=half
            half+=model.uncertainty
            if half>=.5:
                result['boundary_danger']=True
                return fail('白线预测不确定度占满车体净空，停止')
        centres=[]
        for y in (y0,(y0+y1)/2,y1):
            left,right=corridor(y);width=right-left
            if width<=0:
                return fail('边界线形相交，暂用图像引导继续观察')
            if not w*.15<width<w*1.8:return fail('白线宽度异常，继续图像引导')
            # Historical X=0 line is a nominal camera reference, not feedback.
            H=self.projection.h
            if abs(H[0,0])<1e-8:return fail('旧标定中心参考无效')
            centre=float(-(H[0,1]*y+H[0,2])/H[0,0])
            q=(centre-left)/width;centres.append(q)
        result.update(ego_fraction=float(np.mean(centres)),half_envelope=half,
                      ego_near_fraction=float(centres[-1]),ego_far_fraction=float(centres[0]),
                      heading_error=float(centres[-1]-centres[0]),
                      boundary_position_source='nearest_observed_row',
                      ego_range=[min(centres),max(centres)],
                      lane_bounds_px=[corridor(y0),corridor(y1)],lane_rows=[y0,y1],
                      guard='single_observed_edge_width_prediction' if model and model.state=='single' else 'fresh_white_lines_and_image_envelope',metric_boundary_verified=False)
        hard=(g['vehicle_width_m']/2+.03)/g['lane_width_m']
        # Far rows describe heading/lookahead, not where the car body is.
        # Use the nearest observed row for inward correction; a reserve-margin
        # warning is not a measured collision or a reason to latch the run off.
        near=centres[-1]
        result['boundary_margin_warning']=near<=hard or near>=1-hard
        if not 0<near<1:
            if self.outside_since is None or self.last_boundary_s is None or t-self.last_boundary_s>.25:
                self.outside_since=t;self.outside_frames=0
            self.outside_frames+=1
        else:self.outside_since=None;self.outside_frames=0
        self.last_boundary_s=t
        result['boundary_outside_frames']=self.outside_frames
        if self.outside_frames>=3 and t-self.outside_since>=.15:
            result['boundary_danger']=True
            return fail('近处参考中心连续位于白线外，结束本轮')
        soft=half+.06
        correction='right' if near<soft else 'left' if near>1-soft else None
        result['boundary_correction']=correction
        if model and model.state=='paired':self.paired_corridor=(t,model)
        result['uncertain_cones']=[]
        self.near_history={key:value for key,value in self.near_history.items() if 0<=t-value['captured_s']<=2.}
        def obstacle_reason(track,reason,y):
            if turn_aware:
                if len(result['obstacle_diagnostics'])<16:
                    result['obstacle_diagnostics'].append({'id':track['id'],'reason':reason,
                        'foot_y_px':float(y),'observed_y_range':[float(y0),float(y1)]})
                else:result['obstacle_diagnostics_omitted']+=1
        for track in row['tracks']:
            if not track.get('visible'):continue
            x,y=track['foot_pixel'];x1,b1,x2,b2=track['bbox_xyxy']
            history=self.near_history.get(track['id'])
            outside=not low<=y<=high
            lost_ground=((outside if turn_aware else y>high)
                or track.get('clipped') or not track.get('ground_contact_usable'))
            if (history and lost_ground and y/h>=history['entry']['y_fraction']-.03
                    and history['entry'].get('exit_band') and history['entry']['y_fraction']-history['first_y']>=.06):
                # Same tracked cone moved down/out of the ground ROI. Its new
                # clipped pixels are not a new object directly in front, and
                # must not be projected into an invented under-car distance.
                historical={key:value for key,value in history['entry'].items() if key!='pass_evidence'}
                result['cones'].append({**historical,'bbox_xyxy':track['bbox_xyxy'],
                    'image_fraction':float(x/w),'half_image_width':float((x2-x1)/w/2),
                    'y_fraction':min(1.,y/h),'near_field_continuation':True,
                    'image_close':False,'other_close':False,'reference_distance_m':None,
                    'position_source':'last_valid_ground_observation'})
                obstacle_reason(track,'historical_lane_continuation',y)
                continue
            if outside:
                obstacle_reason(track,'out_of_observed_band',y)
                continue
            if turn_aware:
                try:left,right=corridor(y)
                except ValueError:
                    obstacle_reason(track,'invalid_observed_slice',y)
                    continue
            else:left,right=corridor(y)
            width=right-left
            if width<=0:
                if turn_aware:
                    obstacle_reason(track,'invalid_observed_slice',y)
                    continue
                return fail('白线相交')
            # Ignore only objects wholly outside the observed/short-extended
            # corridor; ambiguous blue shapes remain visible as candidates.
            if x2<left or x1>right:
                obstacle_reason(track,'outside_observed_corridor',y)
                continue
            entry={'id':track['id'],'fraction':(x-left)/width,'half_width':max((x2-x1)/width/2,g['cone_base_width_m']/g['lane_width_m']/2),
                   'image_fraction':float(x/w),'half_image_width':float((x2-x1)/w/2),
                   'y_fraction':float(y/h),'near':bool(y>=max(y1*.94,h*.60)),'bbox_xyxy':track['bbox_xyxy'],
                   'image_close':bool(y/h>=(.95 if self.camera=='primary' else .70)),
                   'other_close':bool(y/h>=(.95 if self.camera=='primary' else .65)),
                   'exit_band':bool(y/h>=max(.90 if self.camera=='primary' else .70,high/h-.045)),
                   'reference_distance_m':None,**_identity_evidence(track)}
            if self.camera=='primary':entry['near']=bool(y/h>=.90)
            if (track.get('kind','cone_candidate')!='cone_candidate' or not track.get('confirmed')
                    or not track.get('ground_contact_usable') or track.get('clipped')):
                obstacle_reason(track,'unconfirmed_or_unusable_ground_contact',y)
                result['uncertain'].append(track['id'])
                if track.get('kind')=='cone_candidate' or track.get('confirmed'):
                    result['uncertain_cones'].append({**entry,'provisional':True})
                continue
            if (turn_aware and model.state=='paired' and track.get('visible') is True
                    and track.get('confirmed') is True and track.get('seen_s')==t
                    and track['id'].startswith(self.camera+'-')):
                H=self.projection.h
                reference=-(H[0,1]*y+H[0,2])/H[0,0]
                entry['pass_evidence']={'target_id':track['id'],'camera':self.camera,
                    'frame_id':row['frame_id'],'captured_s':t,
                    'reference_fraction_at_foot':float((reference-left)/width),
                    'cone_fraction_at_foot':float(entry['fraction']),
                    'cone_half_width_fraction':float(entry['half_width']),
                    'foot_y_fraction':float(y/h),'within_observed_band':True,
                    'reference_source':'historical_X0_same_foot_row_nominal',
                    'physical_tail_verified':False}
            try:entry['reference_distance_m']=self.projection.point([x,y])[1]-g['rear_axle_to_front_m']
            except ValueError:pass  # no metric extrapolation beyond old polygon
            result['cones'].append(entry)
            obstacle_reason(track,'current_observed_lane_position',y)
            self.near_history[track['id']]={'captured_s':t,'entry':copy.deepcopy(entry),
                'first_y':min(history['first_y'],entry['y_fraction']) if history else entry['y_fraction']}
        result['cones'].sort(key=lambda c:c['y_fraction'],reverse=True)
        result.update(ready=True,action='visual_trial_ready',
                      reason='白线已确认；旧数据低速测试视图' if not result['uncertain'] else '蓝色候选未确认，持续低速观察避让')
        self.latest=result;return result

    def image_guidance(self,row,w,h):
        cones=[];uncertain=[]
        for track in row.get('tracks',[]):
            if not track.get('visible'):continue
            x,y=track['foot_pixel'];x1,_,x2,_=track['bbox_xyxy'];q=y/h
            history=self.near_history.get(track['id'])
            recent=history and 0<=row['captured_s']-history['captured_s']<=2.
            continuation=bool(recent and history['entry'].get('exit_band')
                and history['entry']['y_fraction']-history['first_y']>=.06
                and q>=history['entry']['y_fraction']-.03)
            low,high=(.55,.97) if self.camera=='primary' else (.4,.78)
            if not low<=q<=high and not continuation:continue
            if track.get('kind','cone_candidate')!='cone_candidate' and not continuation:
                uncertain.append(track['id']);continue
            if not track.get('confirmed'):
                uncertain.append(track['id'])
                if track.get('kind')!='cone_candidate':continue
            cones.append({'id':track['id'],'image_fraction':float(x/w),'half_image_width':float((x2-x1)/w/2),
                'bbox_xyxy':track['bbox_xyxy'],'y_fraction':min(1.,q),'near':q>=(.90 if self.camera=='primary' else .60),
                'near_field_continuation':continuation,'provisional':not track.get('confirmed'),
                'position_source':'current_image_only','reference_distance_m':None,**_identity_evidence(track)})
        cones.sort(key=lambda c:c['y_fraction'],reverse=True)
        return {'cones':cones,'uncertain':uncertain}

    def draw(self,image):
        out=image.copy();row=self.latest or {}
        cv2.putText(out,'LOW-SPEED TEST / HISTORICAL ESTIMATES',(8,40),cv2.FONT_HERSHEY_SIMPLEX,.34,(0,220,255),1)
        if row.get('lane_bounds_px'):
            for bounds,y in zip(row['lane_bounds_px'],row['lane_rows']):
                left,right=bounds;pad=(right-left)*row['half_envelope']
                cv2.line(out,(round(left+pad),round(y)),(round(right-pad),round(y)),(0,220,255),2)
        return out


class DualLowSpeedNavigator:
    """Choose a coherent view; never paste one camera's pixels into the other.

    Turn-aware cones use current paired pixels, or one currently observed edge
    and a same-camera width prediction for at most .25s. The legacy context in
    other stages keeps its original acceptance and memory policy.
    """
    hold_s=.8

    def __init__(self,settings,primary_ground=None):
        self.s=settings;self.latest=None;self.selected='secondary';self.memory={};self.results={}
        self.avoidance_target=None
        self.secondary_recovery=StablePairedLane(duration=.15,frames=3)
        self.views={'secondary':LowSpeedNavigator(settings)}
        if primary_ground is None:
            path=Path(__file__).parents[2]/'configs/cone-primary-ground-reference.json'
            primary_ground=json.loads(path.read_text(encoding='utf-8'))
        original=copy.deepcopy(primary_ground)
        if original.get('image_size')!=[640,480] or original.get('historical_reference_only') is not True:
            raise ValueError('第一路旧标定规格不匹配')
        resize=np.array([[4/3,0,1/6],[0,4/3,1/6],[0,0,1.]])
        shift=np.eye(3);shift[1,2]=settings['geometry']['rear_axle_to_front_m']-original['frontmost_to_rear_axle_m']
        original['homography']=(shift@np.asarray(original['homography'])@resize).tolist()
        original['valid_polygon_px']=((np.asarray(original['valid_polygon_px'])+.5)*.75-.5).tolist()
        original.update(image_size=[480,360],verified=False)
        first=copy.deepcopy(settings);first['ground']=original
        self.views['primary']=LowSpeedNavigator(first,'primary')

    def set_avoidance_target(self,camera,identifier):
        """Optional avoidance hint; clearing it restores normal lane selection."""
        if camera is None and identifier is None:
            self.avoidance_target=None;return
        if (camera not in self.views or not isinstance(identifier,str)
                or not 1<=len(identifier)<=128 or not identifier.startswith(camera+'-')):
            raise ValueError('avoidance target must belong to the specified camera')
        self.avoidance_target=(camera,identifier)

    def _avoidance_source(self,current,rows,now):
        if self.avoidance_target is None:return None
        from .cone_guidance import strict_visual
        camera,identifier=self.avoidance_target
        nav=current.get(camera);row=rows.get(camera)
        if (not nav or not row or nav.get('boundary_danger') or nav.get('ready') is False
                or nav.get('action')=='stop' or nav.get('source_camera')!=camera
                or nav.get('frame_id')!=row.get('frame_id')
                or nav.get('captured_s')!=row.get('captured_s')):return None
        if strict_visual(nav,identifier,now,image_size=self.s['image_size']) is not None:return camera
        return None

    def update_views(self,images,rows,now):
        current={};fresh=[];held=[];danger=[]
        for camera in ('secondary','primary'):
            row=rows.get(camera)
            if not row or camera not in images or not 0<=now-row['captured_s']<=.25:continue
            observed=bool(row['road_context'].get('valid'));data=row;memory=self.memory.get(camera)
            turn_aware=bool(row['road_context'].get('turn_aware'))
            if turn_aware:
                self.memory.pop(camera,None)
            elif not observed and memory and 0<=now-memory['captured_s']<=self.hold_s:
                data={**row,'road_context':memory['context']}
            nav=self.views[camera].update(images[camera],data,now)
            if nav is None:continue
            nav=copy.deepcopy(nav);has_line=bool(nav.get('lane_bounds_px'))
            if nav.get('boundary_danger'):
                danger.append(camera)
                current[camera]={**nav,'source_camera':camera,'line_sources':[camera],'line_held':False}
                continue
            if turn_aware:
                observed=bool(nav.get('line_state')=='paired' and nav.get('boundary_observed')
                    and has_line and not nav.get('line_missing'))
                if observed:fresh.append(camera)
                elif nav.get('line_state')=='single' and has_line:held.append(camera)
                nav.update(source_camera=camera,line_held=nav.get('line_state')=='single')
                current[camera]=nav
                continue
            if observed and has_line:
                self.memory[camera]={'captured_s':row['captured_s'],'context':copy.deepcopy(row['road_context'])}
                fresh.append(camera)
            elif has_line and memory and now-memory['captured_s']<=self.hold_s:held.append(camera)
            nav.update(source_camera=camera,line_held=not observed,
                line_observed_s=row['captured_s'] if observed else memory['captured_s'] if memory else None,
                line_age_s=0. if observed else now-memory['captured_s'] if memory else None)
            current[camera]=nav
        self.results=current
        secondary_stable=self.secondary_recovery.update(current.get('secondary'),now)
        locked=self._avoidance_source(current,rows,now)
        choices=fresh or held
        if not choices and danger:
            self.latest=current[danger[0]];return self.latest
        choices=choices or list(current)
        if not choices:
            t=max((r['captured_s'] for r in rows.values()),default=-1e9)
            self.latest={'experimental':True,'ready':False,'action':'stop','captured_s':t,
                'phase':'camera_missing','reason':'两路相机图像中断，停止',
                'cones':[],'uncertain':[],'line_sources':[],'line_held':False,'hardware_output':False}
            return self.latest
        source=self.selected if self.selected in choices else choices[0]
        # Primary is a temporary fallback. Return only after independent,
        # current paired secondary observations, while retaining any healthy
        # view immediately if the fallback itself has lost usable white lines.
        if locked is not None:
            source=locked
        elif self.selected=='primary' and 'secondary' in fresh and secondary_stable:
            source='secondary'
        self.selected=source;result=current[source]
        corrections={current[c].get('boundary_correction') for c in fresh}
        if len(corrections)>1:
            result['boundary_disagreement']='双摄纠偏不一致，沿用当前有效视角'
        if danger:result['boundary_disagreement']=('另一视角边界异常，沿用当前目标视角' if locked is not None
            else '另一视角边界异常，采用当前可靠白线')
        result['line_sources']=fresh
        result['reason']=('第一路' if source=='primary' else '第二路')+('白线未恢复，保持低速图像引导' if result.get('line_missing')
            else '当前单侧白线已匹配，短时预测走廊宽度' if result.get('line_state')=='single'
            else '白线短暂漏检，沿用最近边界持续前进' if result['line_held'] else '白线已确认，可进行低速试验')
        self.latest=result;return result

    def draw_camera(self,camera,image):
        return self.views[camera].draw(image)


class LowSpeedController:
    def __init__(self,settings,now,*,stop_on_complete=True,initialized=False):
        self.s=settings;self.options=settings['low_speed_test'];self.started=now
        self.arm_until=now+(0. if initialized else 3.)
        self.phase='arming';self.deadline=self.arm_until;self.target=None;self.side=None;self.first_side=None
        self.target_fraction=.5;self.near_seen=False;self.powered_s=0.;self.tail_powered=0.;self.count=0
        self.drive_started=None;self.last_step=now;self.wait_reason='';self.review_pending=False
        self.phase_after_brake='observing';self.next_after=None;self.selected_pulse=1610
        self.last_observation_t=None;self.await_since=None
        self.line_missing_since=None;self.held_powered_s=0.;self.source_camera=None;self.last_target_fraction=None
        self.boundary_turn=None;self.auto_count=0;self.manual_count=0;self.passed_ids=set()
        self.target_aliases=set();self.pass_min_y=None;self.pass_last_y=None;self.pass_last_s=None;self.pass_gate_hits=0
        self.return_started=None;self.centred_since=None
        self.approach_frames=0;self.escape_exit_hits=0;self.escape_turn=None;self.possible_contact=False
        self.stop_on_complete=stop_on_complete;self.line_search_side=None;self.line_search_id=None

    def progress(self):
        return {'operator_confirmed_count':self.manual_count,'estimated_pass_count':self.auto_count,
                'passed_count':self.count,'target_id':self.target,'side':self.side,
                'can_confirm_pass':self.phase=='review','powered_s':self.powered_s,
                'source_camera':self.source_camera,'held_powered_s':self.held_powered_s,
                'left_us':self.options['left_us'],'right_us':self.options['right_us'],
                'boundary_correction':self.boundary_turn,'pass_gate_frames':self.pass_gate_hits,
                'near_field_escape':self.escape_turn,'possible_contact':self.possible_contact,
                'line_missing_since':self.line_missing_since,'line_loss_policy':'bounded_recovery_then_stop',
                'line_recovery_s':LINE_RECOVERY_S,
                'field_verified':False,'automatic_pass_count':True,'pass_estimate_unverified':True}

    def confirm_pass(self):
        if self.phase!='review':raise ValueError('当前未进入车尾通过确认阶段')
        self.manual_count+=1
        self._passed()
        self.phase='complete' if self.count>=2 else 'observing';self.await_since=None

    def _passed(self):
        self.passed_ids.update(self.target_aliases)
        self.count+=1;self.target=None;self.near_seen=False;self.tail_powered=0.;self.review_pending=False
        self.source_camera=None;self.last_target_fraction=None
        self.target_aliases=set();self.pass_min_y=self.pass_last_y=self.pass_last_s=None;self.pass_gate_hits=0
        self.approach_frames=0;self.escape_exit_hits=0;self.escape_turn=None

    def _pass_observation(self,target,nav,sign):
        t=nav['captured_s']
        if t==self.pass_last_s:return
        self.approach_frames+=1
        y=target['y_fraction'];self.pass_min_y=y if self.pass_min_y is None else min(self.pass_min_y,y)
        exit_evidence=(target.get('exit_band',False)
                       and y-self.pass_min_y>=.06)
        self.escape_exit_hits=self.escape_exit_hits+1 if exit_evidence else 0
        if target.get('near_field_continuation') or nav.get('line_held'):
            self.pass_last_s=t;self.pass_last_y=y
            return  # Preserve, but do not invent, previously observed clearance.
        core=(self.s['geometry']['vehicle_width_m']/2+self.options['clearance_m'])/self.s['geometry']['lane_width_m']+target['half_width']
        qualifies=(not nav.get('line_held') and target.get('exit_band',False)
            and y-self.pass_min_y>=.06 and sign*(nav['ego_fraction']-target['fraction'])>core
            and (self.pass_last_y is None or y>=self.pass_last_y-.02))
        self.pass_gate_hits=self.pass_gate_hits+1 if qualifies else 0
        self.pass_last_y=y;self.pass_last_s=t

    def _brake(self,now,reason,after='observing'):
        self.phase='braking';self.deadline=now+.12;self.phase_after_brake=after;self.wait_reason=reason

    def _guard_navigation(self,nav,now):
        """Shared trial/race gate; fresh camera pixels do not prove a clear lane."""
        if nav.get('boundary_danger') or nav.get('ready') is False or nav.get('action')=='stop':
            raise ValueError(nav.get('reason') or '避障导航要求停止')
        missing=bool(nav.get('line_missing') or not nav.get('lane_bounds_px') or not numeric(nav.get('ego_fraction')))
        if not missing and not nav.get('line_held'):
            self.line_missing_since=None
        elif now>=self.arm_until:
            age=nav.get('line_age_s')
            since=now-max(0.,age) if numeric(age) else now
            if self.line_missing_since is None:self.line_missing_since=since
            else:self.line_missing_since=min(self.line_missing_since,since)
            if now-self.line_missing_since>=LINE_RECOVERY_S-1e-9:
                raise ValueError('白线丢失超过 0.4 秒恢复预算，停止并重新准备')
        return missing

    def _guard_near_clearance(self,nav,steer):
        """Reject an image-space near-obstacle conflict before either output path."""
        if not numeric(nav.get('ego_fraction')):return
        ego=nav['ego_fraction'];shift=self.options['turn_margin_m']/self.s['geometry']['lane_width_m']
        predicted=ego+(shift if steer<1610 else -shift if steer>1610 else 0)
        for cone in nav.get('cones',[])+nav.get('uncertain_cones',[]):
            if cone.get('near_field_continuation') or not numeric(cone.get('fraction')):continue
            if not cone.get('image_close',cone.get('y_fraction',0.)>=.70):continue
            core=(self.s['geometry']['vehicle_width_m']/2+self.options['clearance_m'])/self.s['geometry']['lane_width_m']+cone['half_width']
            if min(abs(cone['fraction']-ego),abs(cone['fraction']-predicted))<=core:
                raise ValueError('纠偏方向与近处锥桶净空冲突，主动刹车')

    def step(self,nav,now):
        neutral={'action':'neutral','esc_us':1500,'steering_us':1610,'phase':self.phase,'reason':self.wait_reason,'experimental':True}
        if now<self.last_step:raise ValueError('控制时钟倒退，结束本轮')
        if now-self.started>=self.options['max_run_s']:raise ValueError(f"本轮 {self.options['max_run_s']:g} 秒运行时限已到，已停车")
        if self.phase=='driving':
            elapsed=now-self.last_step;self.powered_s+=elapsed
            if self.review_pending:self.tail_powered+=elapsed
            if nav and nav.get('line_held'):self.held_powered_s+=elapsed
        self.last_step=now
        if not nav or not 0<=now-nav.get('captured_s',-1e9)<=.25:raise ValueError('引导画面中断，已结束试验')
        if not nav.get('experimental'):raise ValueError('低速测试画面尚未就绪')
        # Confirmation and completion are stationary latches. A segmentation
        # gap must never turn them into a resumable lane-following state.
        if self.phase=='complete':return {**neutral,'phase':'complete','reason':'两桶通过流程已结束，保持停车'}
        if self.phase=='review':return {**neutral,'phase':'review','reason':'已停车：确认车尾确实通过当前桶后，再继续'}
        if self.phase in ('braking','settling') and self.phase_after_brake in ('review','complete'):
            # Once a terminal/review stop begins, a line gap cannot convert it
            # into the resumable waiting-lines state and restart the vehicle.
            if self.phase=='braking':
                if now<self.deadline:return {**neutral,'action':'brake','esc_us':1400,'phase':'braking','reason':self.wait_reason}
                self.phase='settling';self.deadline=now+self.options['settle_s']
            if now<self.deadline:return {**neutral,'phase':'settling','reason':self.wait_reason}
            self.phase=self.phase_after_brake
            return {**neutral,'phase':self.phase,'reason':self.wait_reason}
        missing=self._guard_navigation(nav,now)
        if not nav.get('line_held'):self.held_powered_s=0.
        if now<self.arm_until:return {**neutral,'reason':'3 秒中位初始化'}
        if self.phase=='arming':self.phase='observing'
        if self.phase=='braking':
            if now<self.deadline:return {**neutral,'action':'brake','esc_us':1400,'phase':'braking','reason':self.wait_reason}
            self.phase='settling';self.deadline=now+self.options['settle_s'];self.next_after=nav['captured_s']
        if self.phase=='settling':
            if now<self.deadline:return {**neutral,'phase':'settling','reason':'中位观察，等待新鲜白线与锥桶画面'}
            self.phase=self.phase_after_brake
        if self.phase=='review':return {**neutral,'phase':'review','reason':'已停车：确认车尾确实通过当前桶后，再继续'}
        if self.phase=='complete':return {**neutral,'phase':'complete','reason':'两桶视觉估计通过并回正，测试结束'}
        self.await_since=None
        if missing:return self._without_lines(nav,now,neutral)
        self.line_search_id=self.line_search_side=None
        cones=[c for c in nav['cones'] if c['id'] not in self.passed_ids]
        ego=nav['ego_fraction'];half=nav['half_envelope'];reserved=.03
        if self.count>=2:
            if not self.stop_on_complete:
                self.target_fraction=.5
                result=self._forward(nav,now,neutral,'两桶视觉估计通过，连续衔接下一任务')
                self.phase='complete';return {**result,'phase':'complete'}
            if self.return_started is None:self.return_started=now
            self.target_fraction=.5
            if abs(ego-.5)<.05 and not nav.get('boundary_correction') and not nav.get('line_held'):
                if self.centred_since is None:self.centred_since=now
                if now-self.centred_since>=.3:
                    self._brake(now,'两桶视觉估计通过，已回归车道',after='complete')
                    return {**neutral,'action':'brake','esc_us':1400,'phase':'braking','reason':self.wait_reason}
            else:self.centred_since=None
            return self._forward(nav,now,neutral,'两桶视觉估计通过，低速回归车道中心')
        if self.target is None and not cones:
            self.target_fraction=.5
            return self._forward(nav,now,neutral,'沿白线持续低速前进，搜索蓝色锥桶')
        if self.target is None:
            target=cones[0];choices=[]
            if self.count and target['y_fraction']>(.88 if nav.get('source_camera')=='primary' else .65):
                raise ValueError('新目标首次出现已在近处，不能确认第二桶身份')
            for side,sign in [('left',-1),('right',1)]:
                target_x=target['fraction']+sign*(half+target['half_width']+reserved)
                if half+reserved<=target_x<=1-half-reserved:choices.append((abs(target_x-ego),side,target_x))
            if not choices:raise ValueError('锥桶与白线之间间隙不足，保持停车')
            # The rules require avoiding both cones, not a mandatory S-turn.
            # Prefer switching sides when feasible; do not reject the only
            # clear route just because both cones happen to share one side.
            preferred=[choice for choice in choices if choice[1]!=self.first_side]
            choices=preferred or choices
            _,self.side,self.target_fraction=min(choices)
            if self.first_side is None:self.first_side=self.side
            self.target=target['id']
            self.target_aliases.add(self.target)
        source=nav.get('source_camera')
        if self.source_camera and source and source!=self.source_camera and self.target is not None:
            # Reassociate only a unique similar track-relative position. IDs
            # from the two cameras never count as two passes of one cone.
            candidates=[c for c in cones if self.last_target_fraction is not None and abs(c['fraction']-self.last_target_fraction)<.18]
            if len(candidates)!=1:raise ValueError('白线已切换摄像头，但锥桶对应关系不明确，停车核对')
            self.target=candidates[0]['id']
            self.target_aliases.add(self.target)
            self.pass_min_y=self.pass_last_y=self.pass_last_s=None;self.pass_gate_hits=0
            self.approach_frames=0;self.escape_exit_hits=0;self.escape_turn=None
        self.source_camera=source
        target=next((c for c in cones if c['id']==self.target),None)
        if target:
            continuation=target.get('near_field_continuation',False)
            if not continuation:self.review_pending=False;self.tail_powered=0.
            self.last_target_fraction=target['fraction']
            self.near_seen|=target['near']
            sign=-1 if self.side=='left' else 1
            self._pass_observation(target,nav,sign)
            if continuation:
                self.review_pending=True;self.escape_turn=None
                if self.tail_powered>=self.options['tail_powered_limit_s'] and self.pass_gate_hits>=3:
                    self.auto_count+=1;self._passed();return self.step(nav,now)
            else:
                self.target_fraction=target['fraction']+sign*(half+target['half_width']+reserved)
                if not half+reserved<=self.target_fraction<=1-half-reserved:raise ValueError('绕行侧没有白线内的通路，停止试验')
            # The .17m report is not a live measurement. Add .10m reserve;
            # only use distances inside the historical polygon as extra stops.
            distance=target.get('reference_distance_m')
            close=(numeric(distance) and 0<=distance<=self.options['stopping_budget_m']) or target.get('image_close',target['y_fraction']>=.70)
            if close and abs(target['fraction']-ego)<half+target['half_width']+reserved:
                # Planning includes a larger swing reserve. Do not call that
                # entire buffer an unavoidable collision: allow a turn AWAY
                # only when the current body + 5cm clearance is already clear.
                core=(self.s['geometry']['vehicle_width_m']/2+self.options['clearance_m'])/self.s['geometry']['lane_width_m']+target['half_width']
                away=sign*(self.target_fraction-ego)>.005 and sign*(ego-target['fraction'])>0
                if abs(target['fraction']-ego)<=core or not away:
                    approached=(self.phase=='driving' and self.approach_frames>=3
                                and target['y_fraction']-self.pass_min_y>=.06 and target.get('exit_band',False))
                    if not approached:raise ValueError('首次出现的近处锥桶净空不明确，主动刹车')
                    # For the same cone already being approached/avoided,
                    # continue a low-speed turn away instead of latching stop
                    # on the near-image band. This is not collision clearance.
                    self.escape_turn=self.side;self.possible_contact=True
                else:self.escape_turn=None
        elif not self.near_seen:
            self.target_fraction=.5;self.escape_turn=None
            return self._forward(nav,now,neutral,'目标暂时漏检，保持低速搜索，不增加通过计数')
        else:
            # Keep the passing-side offset while the rear clears. Automatic
            # handoff requires several distinct near-exit observations with
            # approach and side clearance, then bounded powered continuation.
            # It remains an estimate, not measured odometry or a verified pass.
            self.review_pending=True
            if self.tail_powered>=self.options['tail_powered_limit_s']:
                if self.pass_gate_hits>=3:
                    self.auto_count+=1;self._passed()
                    return self.step(nav,now)
                if self.phase!='braking':self._brake(now,'目标离开近处视野，停车确认车尾',after='review')
                return {**neutral,'action':'brake','esc_us':1400,'phase':'braking','reason':self.wait_reason}
        for other in cones:
            if other['id']==self.target:continue
            if other.get('other_close',other['y_fraction']>=.65) and abs(other['fraction']-ego)<half+other['half_width']+reserved:
                raise ValueError('另一锥桶进入近处行驶通道，主动刹车')
        return self._forward(nav,now,neutral,'持续低速绕桶：从'+('左' if self.side=='left' else '右')+'侧通过；白线持续检查')

    def _without_lines(self,nav,now,neutral):
        """Current pixels steer around blue cones while lane position is unknown.

        Never invent lane bounds or pass evidence from elapsed line-loss time.
        An old full-lock steering command is not held indefinitely in the dark.
        """
        self._guard_navigation(nav,now)
        cones=[c for c in nav.get('cones',[]) if c['id'] not in self.passed_ids]
        target=next((c for c in cones if c['id']==self.target),None)
        if self.target is not None and self.near_seen and (not target or target.get('near_field_continuation')):
            self.review_pending=True
            if self.tail_powered>=self.options['tail_powered_limit_s'] and self.pass_gate_hits>=3:
                self.auto_count+=1;self._passed()
        elif target:self.review_pending=False;self.tail_powered=0.
        # Prefer the active same-camera target; otherwise use the closest
        # current blue object as a local avoidance cue, not a counted passage.
        visible=next((c for c in cones if c['id']==self.target),None) or (cones[0] if cones else None)
        steer=1610
        if visible and numeric(visible.get('image_fraction')):
            x=visible['image_fraction'];width=visible.get('half_image_width',0.)
            if self.line_search_id!=visible['id']:
                self.line_search_id=visible['id']
                self.line_search_side=self.side if visible['id']==self.target else ('left' if x>=.5 else 'right')
            # Recenter once the cone is in the outer image band / below ROI.
            # These thresholds are pixels, not a claimed physical clearance.
            if .25-width<x<.75+width and not visible.get('near_field_continuation'):
                steer=self.options['left_us'] if self.line_search_side=='left' else self.options['right_us']
        else:self.line_search_id=self.line_search_side=None
        self.boundary_turn=None;self.phase='driving'
        if self.count>=2 and not self.stop_on_complete:self.phase='complete'
        return {**neutral,'action':'search','esc_us':self.s['driving']['forward_us'],'steering_us':steer,
                'phase':self.phase,'reason':'白线未恢复，保持低速'+('向左避桶' if steer>1610 else '向右避桶' if steer<1610 else '前进搜索'),
                'boundary_correction':None,'boundary_observed':False,'line_search':True}

    def _forward(self,nav,now,neutral,reason):
        ego=nav['ego_fraction'];half=nav['half_envelope'];reserved=.03
        difference=self.target_fraction-ego
        right=self.options['right_us']
        left=self.options['left_us']
        steer=1610 if abs(difference)<.025 else right if difference>0 else left
        if self.escape_turn:
            steer=left if self.escape_turn=='left' else right
            reason='同一锥桶已进入近处视野，保持低速向'+('左' if self.escape_turn=='left' else '右')+'避让驶离'
        near=nav.get('ego_near_fraction',ego)
        lo=hi=near;soft=half+self.options['boundary_soft_fraction']
        requested=nav.get('boundary_correction')
        if requested:self.boundary_turn=requested
        elif lo<soft:self.boundary_turn='right'
        elif hi>1-soft:self.boundary_turn='left'
        elif self.boundary_turn=='right' and lo>=soft+self.options['boundary_release_fraction']:self.boundary_turn=None
        elif self.boundary_turn=='left' and hi<=1-soft-self.options['boundary_release_fraction']:self.boundary_turn=None
        if self.boundary_turn:
            steer=self.options['right_us'] if self.boundary_turn=='right' else self.options['left_us']
            reason=('接近左白线，保持低速向右纠偏' if self.boundary_turn=='right' else '接近右白线，保持低速向左纠偏')
        elif nav.get('uncertain'):
            reason='蓝色候选未确认，保持低速观察'
            for cone in nav.get('uncertain_cones',[]):
                if cone.get('near') and abs(cone['fraction']-ego)<half+cone['half_width']:
                    steer=left if cone['fraction']>=ego else right
                    reason='蓝色候选进入前方，保持低速向'+('左' if steer>1610 else '右')+'避让'
                    break
        # New endpoints have no measured radius. Use an extra image-space
        # boundary reserve rather than pretending the old 12m radius applies.
        self._guard_near_clearance(nav,steer)
        if self.phase!='driving':
            if self.next_after is not None and nav['captured_s']<=self.next_after:
                return {**neutral,'phase':'observing','reason':'等待制动后的新相机帧'}
            self.phase='driving';self.deadline=None
        return {**neutral,'action':'search','esc_us':self.s['driving']['forward_us'],'steering_us':steer,
                'phase':'driving','reason':reason,'boundary_correction':self.boundary_turn}
