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

PROFILE='legacy_low_speed'


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
    validate_limits(right,left)
    if type(forward) is not int or not 1501<=forward<=1600:raise ValueError('测试油门仅允许 1501–1600 μs')
    if not numeric(duration) or not 5<=duration<=120:raise ValueError('本轮运行时限仅允许 5–120 秒')
    s['driving']=radius_settings(s.get('driving',{}));s['driving']['forward_us']=forward
    s['low_speed_test']={'drive_mode':'continuous','settle_s':.55,'max_run_s':duration,'reaction_horizon_s':.25,
        'left_us':left,'right_us':right,'turn_margin_m':.03,
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

    def update(self,image,row,now):
        fid=row['frame_id'];t=row['captured_s'];h,w=image.shape[:2]
        if fid==self.last_id:return self.latest
        self.last_id=fid
        result={'experimental':True,'captured_s':t,'frame_id':fid,'ready':False,'action':'stop',
                'phase':'test_observation','hardware_output':False,'field_verified':False,
                'cones':[],'uncertain':[],'reason':'等待左右白线'}
        def fail(reason):
            if not result.get('boundary_danger') and 0<=now-t<=.25:
                # Losing segmentation is not losing the camera. Keep current
                # cone pixels, but never label image fractions as lane space.
                result.update(self.image_guidance(row,w,h),line_missing=True,guard='image_only_boundary_unknown')
                result.update(ready=True,action='visual_trial_ready')
                self.outside_since=None;self.outside_frames=0
            self.latest={**result,'reason':reason};return self.latest
        if not 0<=now-t<=.25:return fail('第二路图像过期')
        road=row['road_context']
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
        # Short image-space extension of observed straight edges, not extension
        # of the old metric calibration polygon. The bonnet is excluded.
        low=max(h*.4,y0-h*.10);high=min(h*.78,y1+h*.14)
        if self.camera=='primary':low=max(h*.55,y0-h*.03);high=min(h*.97,y1+h*.20)
        g=self.s['geometry'];margin=.05+g['vehicle_length_m']*.10+.27*math.sin(math.radians(3))
        half=(g['vehicle_width_m']/2+margin)/g['lane_width_m']
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
                      ego_near_fraction=float(centres[-1]),boundary_position_source='nearest_observed_row',
                      ego_range=[min(centres),max(centres)],
                      lane_bounds_px=[corridor(y0),corridor(y1)],lane_rows=[y0,y1],
                      guard='fresh_white_lines_and_image_envelope',metric_boundary_verified=False)
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
        result['uncertain_cones']=[]
        self.near_history={key:value for key,value in self.near_history.items() if 0<=t-value['captured_s']<=2.}
        for track in row['tracks']:
            if not track.get('visible'):continue
            x,y=track['foot_pixel'];x1,b1,x2,b2=track['bbox_xyxy']
            history=self.near_history.get(track['id'])
            lost_ground=(y>high or track.get('clipped') or not track.get('ground_contact_usable'))
            if (history and lost_ground and y/h>=history['entry']['y_fraction']-.03
                    and history['entry'].get('exit_band') and history['entry']['y_fraction']-history['first_y']>=.06):
                # Same tracked cone moved down/out of the ground ROI. Its new
                # clipped pixels are not a new object directly in front, and
                # must not be projected into an invented under-car distance.
                result['cones'].append({**history['entry'],'bbox_xyxy':track['bbox_xyxy'],
                    'image_fraction':float(x/w),'half_image_width':float((x2-x1)/w/2),
                    'y_fraction':min(1.,y/h),'near_field_continuation':True,
                    'image_close':False,'other_close':False,'reference_distance_m':None,
                    'position_source':'last_valid_ground_observation'})
                continue
            if not low<=y<=high:continue  # sky/background and fixed car bonnet
            left,right=corridor(y);width=right-left
            if width<=0:return fail('白线相交')
            # Ignore only objects wholly outside the observed/short-extended
            # corridor; an overlapping ambiguous blue shape always stops.
            if x2<left or x1>right:continue
            entry={'id':track['id'],'fraction':(x-left)/width,'half_width':max((x2-x1)/width/2,g['cone_base_width_m']/g['lane_width_m']/2),
                   'image_fraction':float(x/w),'half_image_width':float((x2-x1)/w/2),
                   'y_fraction':float(y/h),'near':bool(y>=max(y1*.94,h*.60)),'bbox_xyxy':track['bbox_xyxy'],
                   'image_close':bool(y/h>=(.95 if self.camera=='primary' else .70)),
                   'other_close':bool(y/h>=(.95 if self.camera=='primary' else .65)),
                   'exit_band':bool(y/h>=max(.90 if self.camera=='primary' else .70,high/h-.045)),
                   'reference_distance_m':None}
            if self.camera=='primary':entry['near']=bool(y/h>=.90)
            if not track.get('confirmed') or not track.get('ground_contact_usable') or track.get('clipped'):
                result['uncertain'].append(track['id'])
                if track.get('kind')=='cone_candidate' or track.get('confirmed'):
                    result['uncertain_cones'].append({**entry,'provisional':True})
                continue
            try:entry['reference_distance_m']=self.projection.point([x,y])[1]-g['rear_axle_to_front_m']
            except ValueError:pass  # no metric extrapolation beyond old polygon
            result['cones'].append(entry)
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
            if not track.get('confirmed'):
                uncertain.append(track['id'])
                if track.get('kind')!='cone_candidate':continue
            cones.append({'id':track['id'],'image_fraction':float(x/w),'half_image_width':float((x2-x1)/w/2),
                'bbox_xyxy':track['bbox_xyxy'],'y_fraction':min(1.,q),'near':q>=(.90 if self.camera=='primary' else .60),
                'near_field_continuation':continuation,'provisional':not track.get('confirmed'),
                'position_source':'current_image_only','reference_distance_m':None})
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

    At least one fresh safe corridor suffices. Brief segmentation loss may use
    a same-camera corridor for .8s, always with new images and current cones.
    After the corridor expires, current image guidance keeps low-speed motion;
    boundary position is explicitly unknown until an edge pair is recovered.
    """
    hold_s=.8

    def __init__(self,settings,primary_ground=None):
        self.s=settings;self.latest=None;self.selected='secondary';self.memory={};self.results={}
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

    def update_views(self,images,rows,now):
        current={};fresh=[];held=[];danger=[]
        for camera in ('secondary','primary'):
            row=rows.get(camera)
            if not row or camera not in images or not 0<=now-row['captured_s']<=.25:continue
            observed=bool(row['road_context'].get('valid'));data=row;memory=self.memory.get(camera)
            if not observed and memory and 0<=now-memory['captured_s']<=self.hold_s:
                data={**row,'road_context':memory['context']}
            nav=self.views[camera].update(images[camera],data,now)
            if nav is None:continue
            nav=copy.deepcopy(nav);has_line=bool(nav.get('lane_bounds_px'))
            if nav.get('boundary_danger'):
                danger.append(camera)
                current[camera]={**nav,'source_camera':camera,'line_sources':[camera],'line_held':False}
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
        self.selected=source;result=current[source]
        corrections={current[c].get('boundary_correction') for c in fresh}-{None}
        if len(corrections)>1:
            result['boundary_disagreement']='双摄纠偏不一致，沿用当前有效视角'
        elif corrections:result['boundary_correction']=next(iter(corrections))
        if danger:result['boundary_disagreement']='另一视角边界异常，采用当前可靠白线'
        result['line_sources']=fresh
        result['reason']=('第一路' if source=='primary' else '第二路')+('白线未恢复，保持低速图像引导' if result.get('line_missing')
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
                'line_missing_since':self.line_missing_since,'line_loss_policy':'continue_low_speed',
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
        if nav.get('boundary_danger'):raise ValueError(nav.get('reason','白线保护触发'))
        missing=bool(nav.get('line_missing') or not nav.get('lane_bounds_px') or 'ego_fraction' not in nav)
        if missing or nav.get('line_held'):
            if self.line_missing_since is None:self.line_missing_since=now
        else:self.line_missing_since=None
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
                if self.tail_powered>=self.options['tail_powered_limit_s'] and self.escape_exit_hits>=3:
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
                if max(self.pass_gate_hits,self.escape_exit_hits)>=3:
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
        cones=[c for c in nav.get('cones',[]) if c['id'] not in self.passed_ids]
        target=next((c for c in cones if c['id']==self.target),None)
        if self.target is not None and self.near_seen and (not target or target.get('near_field_continuation')):
            self.review_pending=True
            if self.tail_powered>=self.options['tail_powered_limit_s'] and max(self.pass_gate_hits,self.escape_exit_hits)>=3:
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
        shift=self.options['turn_margin_m']/self.s['geometry']['lane_width_m']
        predicted=ego+(shift if steer<1610 else -shift if steer>1610 else 0)
        # Image-space reserve violations request inward steering; only the
        # navigator's persistent nearest-row departure flag ends the run.
        # A white-line correction is not allowed to steer into a nearby cone.
        for cone in nav['cones']:
            if cone.get('near_field_continuation'):continue
            if not cone.get('image_close',cone['y_fraction']>=.70):continue
            if cone['id']==self.target and self.escape_turn and (
                    self.escape_turn=='left' and steer>1610 or self.escape_turn=='right' and steer<1610):
                continue
            core=(self.s['geometry']['vehicle_width_m']/2+self.options['clearance_m'])/self.s['geometry']['lane_width_m']+cone['half_width']
            if min(abs(cone['fraction']-ego),abs(cone['fraction']-predicted))<=core:
                raise ValueError('纠偏方向与近处锥桶净空冲突，主动刹车')
        if self.phase!='driving':
            if self.next_after is not None and nav['captured_s']<=self.next_after:
                return {**neutral,'phase':'observing','reason':'等待制动后的新相机帧'}
            self.phase='driving';self.deadline=None
        return {**neutral,'action':'search','esc_us':self.s['driving']['forward_us'],'steering_us':steer,
                'phase':'driving','reason':reason,'boundary_correction':self.boundary_turn}
