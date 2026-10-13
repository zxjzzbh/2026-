"""Full race orchestration using existing task controllers; no hardware I/O.

Explicit handoff starts at the blue board. Independent trials are unchanged.
Visual cone passage remains estimated; parking needs measured world feedback.
Payment is reported by the operator, never performed by the vehicle software.
"""

from .steering import LEFT_US, RIGHT_US
import copy
import json
import math
from pathlib import Path

from .brake_parking import BrakeParkingController, numeric, load_settings as load_brake
from .traffic_driving import TrafficDrivingController, load_settings as load_traffic
from .cone_low_speed import LowSpeedController, trial_settings
from .cone_planning import load_settings as load_cones
from .cone_navigation import reuse_legacy_ground
from .departure import DepartureSequence
from .parking_bay import load_settings as load_parking, readiness as parking_readiness
from .parking_mission import ParkingMission

STAGES=('arming','wait_board','crosswalk','traffic','cones','parking_search','parking_entry','payment','complete')
LABELS={'idle':'手动遥控／待切换','arming':'自动模式初始化','wait_board':'等待发车挡板移开',
        'crosswalk':'斑马线停车与播报','traffic':'红绿灯停车通行','cones':'两桶避障',
        'parking_search':'寻找空闲停车半区','parking_entry':'自动入位与停稳',
        'payment':'30 秒扫码缴费','complete':'流程结束','fault':'已停止，需重新准备','cancelled':'已退出自动模式'}


def validate_flow(s):
    s=copy.deepcopy(s)
    if s.get('schema_version')!=1:raise ValueError('总流程配置版本无效')
    for k,lo,hi in [('forward_us',1501,1600),('left_us',1650,LEFT_US),('right_us',RIGHT_US,1570)]:
        if type(s.get(k)) is not int or not lo<=s[k]<=hi:raise ValueError('总流程参数越界：'+k)
    for k,lo,hi in [('max_run_s',60,600),('max_stage_s',20,180),('parking_search_s',5,60)]:
        if not numeric(s.get(k)) or not lo<=s[k]<=hi:raise ValueError('总流程参数越界：'+k)
    for k,value in [('crosswalk_hold_s',10),('payment_window_s',30),('max_frame_age_s',.25),('page_lease_s',.5),('arming_s',3)]:
        if not numeric(s.get(k)) or s[k]!=value:raise ValueError('本版比赛流程固定参数：'+k)
    if s.get('school_team_speech')!='我是山东大学启航队的智能车，请为我加油':raise ValueError('学校队伍播报未匹配')
    return s


def load_bundle(root,drive=None):
    root=Path(root);configs=root/'vision/configs'
    flow_path=configs/'race-flow.local.json'
    flow=validate_flow(json.loads((flow_path if flow_path.exists() else configs/'race-flow.json').read_text(encoding='utf-8-sig')))
    brake=load_brake(configs/'brake-parking.json');traffic=load_traffic(configs/'traffic-driving.json')
    def local(name):
        path=configs/(name+'.local.json')
        return json.loads(path.read_text(encoding='utf-8-sig')) if path.exists() else None
    saved=local('brake-parking')
    if saved:brake['brake_trigger_distance_m']=saved['brake_trigger_distance_m']
    saved=local('traffic-driving')
    if saved:
        for key in ('red_confirm_percent','max_approach_s'):
            if key in saved:traffic[key]=saved[key]
    cones=load_cones(configs/('cone-trial.local.json' if local('cone-trial') else 'cone-trial.json'))
    parking=load_parking(configs/('parking-bay.local.json' if local('parking-bay') else 'parking-bay.json'))
    departure=local('departure') or {'min_blue_percent':3.,'clear_s':1.}
    # Use exactly the settings already loaded by the real console wrappers.
    node=drive;seen=set()
    mapping={'BrakeTrialDrive':'brake','TrafficTrialDrive':'traffic','ConeObserverDrive':'cones','ParkingBayDrive':'parking','DepartureTrialDrive':'departure'}
    values={'brake':brake,'traffic':traffic,'cones':cones,'parking':parking,'departure':departure}
    while node is not None and id(node) not in seen:
        seen.add(id(node));name=type(node).__name__
        if name in mapping:values[mapping[name]]=copy.deepcopy(node.settings)
        node=getattr(node,'drive',None)
    if not values['cones']['ground'].get('historical_verified') and not values['cones']['ground'].get('verified'):
        reference=json.loads((configs/'cone-legacy-ground-candidate.json').read_text(encoding='utf-8'))
        values['cones']=reuse_legacy_ground(values['cones'],reference)
    return prepare_bundle({'flow':flow,**values})


def prepare_bundle(bundle):
    b=copy.deepcopy(bundle);f=validate_flow(b['flow']);b['flow']=f
    b['brake']['hold_s']=10.;b['brake']['speech_text']=f['school_team_speech']
    b['brake']['max_approach_s']=f['max_stage_s']
    b['brake']['pwm']['search_us']=b['brake']['pwm']['creep_us']=min(1575,f['forward_us'])
    b['brake']['brake_release_s']=max(.1,b['brake']['brake_release_s'])
    b['traffic']['pwm']['search_us']=b['traffic']['pwm']['creep_us']=f['forward_us']
    b['cones']=trial_settings(b['cones'],{'test_forward_us':f['forward_us'],'test_left_us':f['left_us'],
                                        'test_right_us':f['right_us'],'test_run_s':min(120,f['max_stage_s'])})
    geometry=b['cones']['geometry']
    for target,source in [('lane_width_m','lane_width_m'),('vehicle_width_m','vehicle_width_m'),
                          ('front_extent_m','rear_axle_to_front_m'),('rear_extent_m','rear_overhang_m')]:
        if b['parking']['perception_verified'] and abs(b['parking'][target]-geometry[source])>.005:
            raise ValueError('停车与车辆尺寸配置不一致：'+target)
        b['parking'][target]=geometry[source]
    return b


def parking_blockers(b):
    missing=list(parking_readiness(b['parking'])['missing']);d=b['cones'].get('driving',{})
    names={'steering_verified':'转向档位与曲率对应关系','odometry_verified':'连续位姿反馈有效性',
           'steering_radius_right_cm':'1350 右转半径','steering_radius_left_cm':'1870 左转半径',
           'visual_error_bound_m':'光流位姿误差上限','visual_verified_distance_m':'光流有效行程'}
    for k in ('steering_verified','odometry_verified'):
        if d.get(k) is not True:missing.append(names[k])
    for k in ('steering_radius_right_cm','steering_radius_left_cm','visual_error_bound_m','visual_verified_distance_m'):
        if not numeric(d.get(k)) or abs(d[k])<=0:missing.append(names[k])
    return list(dict.fromkeys(missing))


class FlowTrafficController(TrafficDrivingController):
    """Reuse the light/stop vote machine after the shared ESC initialization.

    Full-route search has the full-stage time budget; the independently exposed
    traffic trial keeps its original 10-second search and 3-second arming.
    """
    def __init__(self,settings,now,stage_limit):
        super().__init__(settings)
        self.s=copy.deepcopy(self.s);self.s['max_approach_s']=stage_limit
        self.s['max_wait_green_s']=min(18.,self.s['max_wait_green_s'])
        self.started=self.approach_at=now;self.phase='approach'
        self.reason='沿赛道接近红绿灯，等待实际灯色'


class RaceFlowController:
    def __init__(self,bundle,now):
        self.b=prepare_bundle(bundle);self.s=self.b['flow'];self.stage='arming';self.started=self.entered=now
        self.reason='保持中位 3 秒，随后等待蓝色发车挡板';self.last_t=None;self.last_seq=None
        self.controller=None;self.departure=DepartureSequence(self.b['departure'].get('clear_confirm_s',1.))
        self.guide=None;self.last_module_id=None;self.last_intent=None;self.events=[];self.transitions=[]
        self.parking=ParkingMission(self.b['parking']);self.parking_wait=None
        self.payment_started=None;self.paid=False;self.paid_in_time=False;self.last_drive=False;self.audio_wait=None;self.stationary_since=None

    def _go(self,stage,now,reason):
        self.transitions.append({'from':self.stage,'to':stage,'at_s':now,'reason':reason})
        self.stage=stage;self.entered=now;self.reason=reason;self.last_module_id=None;self.last_intent=None
        self.guide=LowSpeedController(self.b['cones'],now-3.)
        if stage=='crosswalk':self.controller=BrakeParkingController(self.b['brake'],'straight')
        elif stage=='traffic':self.controller=FlowTrafficController(self.b['traffic'],now,self.s['max_stage_s'])
        elif stage=='cones':self.controller=LowSpeedController(self.b['cones'],now-3.)

    def _out(self,reason=None,intent=None,events=None):
        raw=intent or {'action':'neutral','esc_us':1500,'steering_us':1610}
        if reason:self.reason=reason
        result={**raw,'phase':self.stage,'stage_label':LABELS[self.stage],'reason':self.reason,
                'events':events or [],'hardware_output':False,'field_verified':False}
        if result['esc_us']>1500:
            result['action']='creep' if result['esc_us']==self.b['brake']['pwm']['creep_us'] else 'search'
        self.last_drive=result['esc_us']>1500
        return result

    def fail(self,reason):
        self.stage='fault';return self._out(reason)

    def report_paid(self,now):
        if self.stage!='payment':raise ValueError('尚未停入车位并进入缴费阶段')
        self.paid=True;self.paid_in_time=0<=now-self.payment_started<=30
        self._go('complete',now,'已记录队员付款声明；支付平台到账结果未自动核验')

    def _guided(self,nav,now,base=None):
        if not nav:raise ValueError('缺少新鲜白线引导')
        # Reuse the existing lane-loss and inward-correction behavior without
        # selecting cone targets before the traffic stage has finished.
        lane={**nav,'cones':[]}
        steer=self.guide.step(lane,now)
        if steer['action'] not in ('search','creep'):return steer
        # Apply the same near-obstacle clearance guard to the real observations.
        steer=self.guide._forward(nav,now,steer,'沿赛道低速前进')
        return {**(base or steer),'steering_us':steer['steering_us']}

    def step(self,sample,now,audio=None):
        audio=audio or {}
        if self.stage in ('complete','fault'):return self._out()
        if self.stage=='payment':
            if now-self.payment_started>=30:self._go('complete',now,'30 秒缴费窗口结束，车辆保持停止')
            return self._out()
        if not numeric(now) or self.last_t is not None and now<self.last_t:return self.fail('控制时钟倒退')
        self.last_t=now
        if now-self.started>self.s['max_run_s']:return self.fail('总流程超时，已停止')
        if now-self.entered>self.s['max_stage_s']:return self.fail('当前任务超时，已停止')
        if not isinstance(sample,dict) or not numeric(sample.get('captured_s')) or not 0<=now-sample['captured_s']<=.25:
            return self.fail('相机帧过期或中断')
        if type(sample.get('sequence')) is not int:return self.fail('相机批次编号无效')
        if sample['sequence']<0 or self.last_seq is not None and sample['sequence']<self.last_seq:
            return self.fail('相机批次编号倒退，结束本轮')
        if sample.get('sequence')==self.last_seq:return None
        self.last_seq=sample.get('sequence')
        motion=sample.get('motion') or {}
        if self.stage not in ('arming','wait_board') and motion.get('valid') is True and 0<=now-motion.get('captured_s',-1e9)<=.25:
            if motion.get('moving') is True:self.stationary_since=None
            elif motion.get('moving') is False:
                if self.stationary_since is None:self.stationary_since=now
                if now-self.stationary_since>=19:return self.fail('自动阶段连续停止接近 20 秒，结束本轮并保持停止')
        try:
            if self.stage=='arming':
                if now-self.started>=3:self._go('wait_board',now,'等待先看到蓝色挡板，再确认移开')
                return self._out()
            if self.stage=='wait_board':
                ready=self.departure.update(sample.get('board',{}),now)
                if ready:
                    self._go('crosswalk',now,'挡板移开已确认，开始自主循迹')
                    return self._out(events=[{'event':'speak','id':'departure','text':'我发车啦'}])
                return self._out(self.departure.reason)
            if self.stage in ('crosswalk','traffic'):
                module=sample.get(self.stage);nav=sample.get('navigation')
                if not module or not 0<=now-module.get('captured_s',-1e9)<=.25:raise ValueError('任务观测过期')
                if module['frame_id']!=self.last_module_id:
                    self.last_module_id=module['frame_id'];self.last_intent=self.controller.step(module,now)
                    events=[{**e,'id':self.stage} for e in self.last_intent.get('events',[]) if e.get('event')=='speak']
                else:events=[]
                decision=copy.deepcopy(self.last_intent)
                if decision['phase']=='fault':raise ValueError(decision['reason'])
                if decision['phase']=='done':
                    if self.stage=='crosswalk':
                        if audio.get('crosswalk')=='failed':raise ValueError('队伍播报发送失败')
                        if audio.get('crosswalk')!='command_sent':
                            if self.audio_wait is None:self.audio_wait=now
                            if now-self.audio_wait>3:raise ValueError('队伍播报发送未确认，停止本轮')
                            return self._out('已停够 10 秒，等待播报发送结果',events=events)
                        self._go('traffic',now,'斑马线任务完成，接近红绿灯')
                    else:self._go('cones',now,'实际绿灯达标，继续行驶并开始两桶避障')
                    return self._out(events=events)
                if decision['esc_us']>1500:decision=self._guided(nav,now,decision)
                return self._out(decision['reason'],decision,events)
            if self.stage=='cones':
                decision=self.controller.step(sample.get('navigation'),now)
                if decision['phase']=='review':raise ValueError('锥桶通过证据不足；全流程停止，不以人工跳过冒充自主通过')
                if decision['phase']=='complete':
                    self._go('parking_search',now,'两桶通过流程结束，寻找停车区')
                    return self._out()
                return self._out(decision['reason'],decision)
            if self.stage=='parking_search':
                view=sample.get('parking_view') or {}
                if view.get('layout_found'):
                    self._go('parking_entry',now,'已发现停车区，制动并核对空闲半区')
                    self.parking_wait=now
                    return self._out(intent={'action':'brake','esc_us':1400,'steering_us':1610})
                if now-self.entered>self.s['parking_search_s']:raise ValueError('停车区搜索超时')
                return self._out('沿赛道寻找停车区',self._guided(sample.get('navigation'),now))
            if self.stage=='parking_entry':
                # Maintain the original finite brake length; do not hold reverse.
                if now-self.parking_wait<.12:return self._out(intent={'action':'brake','esc_us':1400,'steering_us':1610})
                missing=parking_blockers(self.b)
                if now-self.parking_wait>=18 and (missing or sample.get('parking') is None):
                    raise ValueError('停车入位反馈持续未就绪，结束本轮并保持停止')
                if missing:return self._out('停车入位等待有效反馈：'+', '.join(missing))
                parking=sample.get('parking')
                if parking is None:return self._out('等待停车区世界坐标、位姿和速度反馈')
                if parking['frame_id']==self.last_module_id:return self._out()
                self.last_module_id=parking['frame_id'];decision=self.parking.step(parking,now)
                if decision['phase']=='fault':raise ValueError(decision['reason'])
                if decision['phase']=='parked':
                    self.payment_started=now;self._go('payment',now,'正确半区内已取得停稳反馈，30 秒内由队员扫码支付 0.01 元')
                    return self._out(events=[{'event':'payment_window_opened','amount_cents':1}])
                if decision['action']=='follow_path':
                    d=self.b['cones']['driving'];curvature=decision['target_curvature_1pm']
                    endpoint=RIGHT_US if curvature>0 else LEFT_US
                    radius=abs(d['steering_radius_right_cm' if curvature>0 else 'steering_radius_left_cm'])/100
                    ratio=abs(curvature)*radius
                    if ratio>1:raise ValueError('入位转向需求超过既有实测档位')
                    steer=round(1610+ratio*(endpoint-1610))
                    return self._out(decision['reason'],{'action':'creep','esc_us':self.b['brake']['pwm']['creep_us'],'steering_us':steer})
                # A repeated brake request is bounded by the shared actuator.
                return self._out(decision['reason'],{'action':'brake' if decision['action']=='brake' else 'neutral',
                    'esc_us':1400 if decision['action']=='brake' else 1500,'steering_us':1610})
            raise ValueError('未知比赛阶段')
        except (ValueError,KeyError,TypeError) as exc:return self.fail(str(exc))

    def status(self,now):
        return {'stage':self.stage,'stage_label':LABELS[self.stage],'reason':self.reason,'transitions':list(self.transitions),
                'cone_progress':self.controller.progress() if self.stage=='cones' else None,
                'parking_missing':parking_blockers(self.b),'payment_remaining_s':None if self.payment_started is None else max(0,30-(now-self.payment_started)),
                'operator_reported_paid':self.paid,'reported_within_window':self.paid_in_time,'provider_verified':False,
                'field_verified':False,'rules_crosswalk_hold_s':10,'manual_interventions':0}
