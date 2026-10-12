"""Supervised image-feedback trial using historical, NOT verified, estimates.

No new calibration or verified flags. White-line guards run on every frame;
forward output is continuous until a hazard, operator review or session limit.
An operator, not image disappearance or elapsed time, confirms each pass.
"""
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
    left=payload.get('test_left_us',1690);right=payload.get('test_right_us',1550)
    validate_limits(right,left)
    if type(forward) is not int or not 1501<=forward<=1600:raise ValueError('测试油门仅允许 1501–1600 μs')
    if not numeric(duration) or not 5<=duration<=120:raise ValueError('本轮运行时限仅允许 5–120 秒')
    s.setdefault('driving',{})['forward_us']=forward
    s['low_speed_test']={'drive_mode':'continuous','settle_s':.55,'max_run_s':duration,'reaction_horizon_s':.25,
        'left_us':left,'right_us':right,'turn_margin_m':.03,
        'reported_braking_m':.17,'stopping_budget_m':.27,'reported_turn_radius_m':12.03,
        'reported_displacement_m':.63,'reported_drive_s':1.5,'estimate_verified':False,
        'tail_powered_limit_s':1.0,'clearance_m':.05}
    return s


class LowSpeedNavigator:
    def __init__(self,settings,camera='secondary'):
        self.s=settings;self.latest=None;self.last_id=None
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
            self.latest={**result,'reason':reason};return self.latest
        if not 0<=now-t<=.25:return fail('第二路图像过期')
        road=row['road_context']
        if not road.get('valid'):return fail('左右白线未同时确认，保持停车')
        edges=[]
        for key in ('left_inner','right_inner'):
            points=np.asarray(road.get(key,[]),float)
            if points.ndim!=2 or points.shape[0]<6 or points.shape[1]!=2 or not np.isfinite(points).all():
                return fail('白线点不足')
            fit=np.polyfit(points[:,1],points[:,0],1)
            if np.ptp(points[:,1])<h*.10 or np.max(np.abs(np.polyval(fit,points[:,1])-points[:,0]))>8:
                return fail('白线形状不稳定，仅支持直道低速试验')
            edges.append((fit,points[:,1].min(),points[:,1].max()))
        y0=max(p[1] for p in edges);y1=min(p[2] for p in edges)
        if y1-y0<h*.10:return fail('左右白线共同可见范围不足')
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
            if not w*.15<width<w*1.8:return fail('白线宽度异常')
            # Historical X=0 line is a nominal camera reference, not feedback.
            H=self.projection.h
            if abs(H[0,0])<1e-8:return fail('旧标定中心参考无效')
            centre=float(-(H[0,1]*y+H[0,2])/H[0,0])
            q=(centre-left)/width;centres.append(q)
            if not half+.03<q<1-half-.03:
                result['boundary_danger']=True
                return fail('接近白线保护边界，已禁止前进')
        result.update(ego_fraction=float(np.mean(centres)),half_envelope=half,
                      lane_bounds_px=[corridor(y0),corridor(y1)],lane_rows=[y0,y1],
                      guard='fresh_white_lines_and_image_envelope',metric_boundary_verified=False)
        for track in row['tracks']:
            if not track.get('visible'):continue
            x,y=track['foot_pixel'];x1,b1,x2,b2=track['bbox_xyxy']
            if not low<=y<=high:continue  # sky/background and fixed car bonnet
            left,right=corridor(y);width=right-left
            if width<=0:return fail('白线相交')
            # Ignore only objects wholly outside the observed/short-extended
            # corridor; an overlapping ambiguous blue shape always stops.
            if x2<left or x1>right:continue
            if not track.get('confirmed') or not track.get('ground_contact_usable') or track.get('clipped'):
                result['uncertain'].append(track['id']);continue
            entry={'id':track['id'],'fraction':(x-left)/width,'half_width':max((x2-x1)/width/2,g['cone_base_width_m']/g['lane_width_m']/2),
                   'y_fraction':float(y/h),'near':bool(y>=max(y1*.94,h*.60)),'bbox_xyxy':track['bbox_xyxy'],
                   'image_close':bool(y/h>=(.95 if self.camera=='primary' else .70)),
                   'other_close':bool(y/h>=(.95 if self.camera=='primary' else .65)),
                   'reference_distance_m':None}
            if self.camera=='primary':entry['near']=bool(y/h>=.90)
            try:entry['reference_distance_m']=self.projection.point([x,y])[1]-g['rear_axle_to_front_m']
            except ValueError:pass  # no metric extrapolation beyond old polygon
            result['cones'].append(entry)
        result['cones'].sort(key=lambda c:c['y_fraction'],reverse=True)
        result.update(ready=not result['uncertain'],action='visual_trial_ready' if not result['uncertain'] else 'stop',
                      reason='白线已确认；旧数据低速测试视图' if not result['uncertain'] else '通道中存在未确认蓝色物体，保持停车')
        self.latest=result;return result

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
    The controller separately limits powered travel on that memory to .15s.
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
        current={};fresh=[];held=[]
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
                self.latest={**nav,'source_camera':camera,'line_sources':[camera],'line_held':False}
                return self.latest
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
        if not choices:
            t=max((r['captured_s'] for r in rows.values()),default=-1e9)
            self.latest={'experimental':True,'ready':False,'action':'stop','captured_s':t,
                'phase':'waiting_lines','reason':'两路白线持续未确认，制动等待恢复',
                'cones':[],'uncertain':[],'line_sources':[],'line_held':False,'hardware_output':False}
            return self.latest
        source=self.selected if self.selected in choices else choices[0]
        self.selected=source;result=current[source]
        result['line_sources']=fresh
        result['reason']=('第一路' if source=='primary' else '第二路')+('白线短暂漏检，限时沿用最近边界' if result['line_held'] else '白线已确认，可进行低速试验')
        self.latest=result;return result

    def draw_camera(self,camera,image):
        return self.views[camera].draw(image)


class LowSpeedController:
    def __init__(self,settings,now):
        self.s=settings;self.options=settings['low_speed_test'];self.started=now
        self.phase='arming';self.deadline=now+3.;self.target=None;self.side=None;self.first_side=None
        self.target_fraction=.5;self.near_seen=False;self.powered_s=0.;self.tail_powered=0.;self.count=0
        self.drive_started=None;self.last_step=now;self.wait_reason='';self.review_pending=False
        self.phase_after_brake='observing';self.next_after=None;self.selected_pulse=1610
        self.last_observation_t=None;self.await_since=None
        self.line_missing_since=None;self.held_powered_s=0.;self.source_camera=None;self.last_target_fraction=None

    def progress(self):
        return {'operator_confirmed_count':self.count,'target_id':self.target,'side':self.side,
                'can_confirm_pass':self.phase=='review','powered_s':self.powered_s,
                'source_camera':self.source_camera,'held_powered_s':self.held_powered_s,
                'left_us':self.options['left_us'],'right_us':self.options['right_us'],
                'field_verified':False,'automatic_pass_count':False}

    def confirm_pass(self):
        if self.phase!='review':raise ValueError('当前未进入车尾通过确认阶段')
        self.count+=1;self.target=None;self.near_seen=False;self.tail_powered=0.;self.review_pending=False
        self.source_camera=None;self.last_target_fraction=None
        self.phase='complete' if self.count>=2 else 'observing';self.await_since=None

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
        if self.phase=='complete':return {**neutral,'phase':'complete','reason':'两桶均已人工确认通过，测试结束'}
        if self.phase=='review':return {**neutral,'phase':'review','reason':'已停车：确认车尾确实通过当前桶后，再继续'}
        if nav.get('boundary_danger'):raise ValueError(nav.get('reason','白线保护触发'))
        if not nav.get('lane_bounds_px') or 'ego_fraction' not in nav:
            if self.line_missing_since is None:self.line_missing_since=now
            if now-self.line_missing_since>3:raise ValueError('两路白线持续未恢复，结束试验')
            if self.phase=='driving':self._brake(now,'两路白线暂失，制动等待恢复',after='waiting_lines')
            if self.phase=='braking' and now<self.deadline:
                return {**neutral,'action':'brake','esc_us':1400,'phase':'braking','reason':self.wait_reason}
            if self.phase=='braking':self.phase='settling';self.deadline=now+self.options['settle_s']
            if self.phase=='settling' and now<self.deadline:
                return {**neutral,'phase':'settling','reason':'保持中位，等待白线恢复'}
            self.phase='waiting_lines'
            return {**neutral,'phase':'waiting_lines','reason':'等待任一路白线恢复，本轮暂不结束'}
        self.line_missing_since=None
        if self.phase=='waiting_lines':self.phase='observing'
        if not nav.get('line_held'):self.held_powered_s=0.
        if now<self.started+3:return {**neutral,'reason':'3 秒中位初始化'}
        if self.phase=='arming':self.phase='observing'
        if self.phase=='braking':
            if now<self.deadline:return {**neutral,'action':'brake','esc_us':1400,'phase':'braking','reason':self.wait_reason}
            self.phase='settling';self.deadline=now+self.options['settle_s'];self.next_after=nav['captured_s']
        if self.phase=='settling':
            if now<self.deadline:return {**neutral,'phase':'settling','reason':'中位观察，等待新鲜白线与锥桶画面'}
            self.phase=self.phase_after_brake
        if self.phase=='review':return {**neutral,'phase':'review','reason':'已停车：确认车尾确实通过当前桶后，再继续'}
        if nav.get('line_held') and self.held_powered_s>=.15:
            if self.phase=='driving':
                self._brake(now,'丢线期间有限前进已到，制动等待')
                return {**neutral,'action':'brake','esc_us':1400,'phase':'braking','reason':self.wait_reason}
            return {**neutral,'phase':'observing','reason':'等待任一路新白线，不延长丢线前进时间'}
        if nav.get('uncertain'):
            if self.phase=='driving':
                self._brake(now,'蓝色障碍暂未确认，制动等待')
                return {**neutral,'action':'brake','esc_us':1400,'phase':'braking','reason':self.wait_reason}
            if self.await_since is None:self.await_since=now
            if now-self.await_since>3:raise ValueError('蓝色障碍持续不明确，结束本轮')
            return {**neutral,'phase':'observing','reason':'等待蓝色障碍连续确认'}
        self.await_since=None
        cones=nav['cones'];ego=nav['ego_fraction'];half=nav['half_envelope'];reserved=.03
        if self.target is None and not cones:
            self.target_fraction=.5
            return self._forward(nav,now,neutral,'沿白线持续低速前进，搜索蓝色锥桶')
        if self.target is None:
            target=cones[0];choices=[]
            for side,sign in [('left',-1),('right',1)]:
                if self.first_side and side==self.first_side:continue
                target_x=target['fraction']+sign*(half+target['half_width']+reserved)
                if half+reserved<=target_x<=1-half-reserved:choices.append((abs(target_x-ego),side,target_x))
            if not choices:raise ValueError('锥桶与白线之间间隙不足，保持停车')
            _,self.side,self.target_fraction=min(choices)
            if self.first_side is None:self.first_side=self.side
            self.target=target['id']
        source=nav.get('source_camera')
        if self.source_camera and source and source!=self.source_camera and self.target is not None:
            # Reassociate only a unique similar track-relative position. IDs
            # from the two cameras never count as two passes of one cone.
            candidates=[c for c in cones if self.last_target_fraction is not None and abs(c['fraction']-self.last_target_fraction)<.18]
            if len(candidates)!=1:raise ValueError('白线已切换摄像头，但锥桶对应关系不明确，停车核对')
            self.target=candidates[0]['id']
        self.source_camera=source
        target=next((c for c in cones if c['id']==self.target),None)
        if target:
            self.last_target_fraction=target['fraction']
            self.near_seen|=target['near']
            sign=-1 if self.side=='left' else 1
            self.target_fraction=target['fraction']+sign*(half+target['half_width']+reserved)
            if not half+reserved<=self.target_fraction<=1-half-reserved:raise ValueError('绕行侧接近白线，停止试验')
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
                    raise ValueError('近处锥桶侵入车身净空或转向不能远离，主动刹车')
        elif not self.near_seen:raise ValueError('目标提前丢失，结束本轮；不会算作已通过')
        else:
            # Limited continuation follows fresh white lines, never an open-loop
            # steering timer. It is an estimate; the operator confirms the tail.
            self.review_pending=True
            if self.tail_powered>=self.options['tail_powered_limit_s']:
                if self.phase!='braking':self._brake(now,'目标离开近处视野，停车确认车尾',after='review')
                return {**neutral,'action':'brake','esc_us':1400,'phase':'braking','reason':self.wait_reason}
        for other in cones:
            if other['id']==self.target:continue
            if other.get('other_close',other['y_fraction']>=.65) and abs(other['fraction']-ego)<half+other['half_width']+reserved:
                raise ValueError('另一锥桶进入近处行驶通道，主动刹车')
        return self._forward(nav,now,neutral,'持续低速绕桶：从'+('左' if self.side=='left' else '右')+'侧通过；白线持续检查')

    def _forward(self,nav,now,neutral,reason):
        ego=nav['ego_fraction'];half=nav['half_envelope'];reserved=.03
        difference=self.target_fraction-ego
        right=self.options['right_us'] if self.target else 1570
        left=self.options['left_us'] if self.target else 1650
        steer=1610 if abs(difference)<.025 else right if difference>0 else left
        # New endpoints have no measured radius. Use an extra image-space
        # boundary reserve rather than pretending the old 12m radius applies.
        shift=self.options['turn_margin_m']/self.s['geometry']['lane_width_m']
        predicted=ego+(shift if steer<1610 else -shift if steer>1610 else 0)
        if not half+reserved<predicted<1-half-reserved:raise ValueError('预计靠近白线，主动刹车')
        if self.phase!='driving':
            if self.next_after is not None and nav['captured_s']<=self.next_after:
                return {**neutral,'phase':'observing','reason':'等待制动后的新相机帧'}
            self.phase='driving';self.deadline=None
        return {**neutral,'action':'search','esc_us':self.s['driving']['forward_us'],'steering_us':steer,
                'phase':'driving','reason':reason}
