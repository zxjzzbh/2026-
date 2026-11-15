"""Full race orchestration using existing task controllers; no hardware I/O.

Explicit handoff starts at the blue board. Independent trials are unchanged.
Visual cone passage remains estimated; parking needs measured world feedback.
Payment is reported by the operator, never performed by the vehicle software.
"""

from .steering import LEFT_US, RIGHT_US
import copy
from .runtime_timing import VISION_MAX_AGE_S, MOTION_MAX_AGE_S, MOTION_RECOVERY_S, RACE_CONTROL_WINDOW_S
import json
import math
from pathlib import Path

from .brake_parking import (BrakeParkingController, numeric, load_settings as load_brake,
                            crosswalk_calibrated_settings)
from .traffic_driving import TrafficDrivingController, load_settings as load_traffic, approach_settings
from .cone_low_speed import trial_settings
from .cone_continuous import ContinuousConeController
from .cone_evidence import StablePairedLane, paired_lane
from .lane_following import LaneFollower, LaneRecoveryRequired, measurements
from .handoff import StillnessWindow
from .cone_planning import load_settings as load_cones
from .cone_navigation import reuse_legacy_ground
from .departure import DEFAULT_MIN_BLUE_PERCENT,DEFAULT_CLEAR_CONFIRM_S,DepartureSequence
from .parking_bay import load_settings as load_parking
from .parking_task import ParkingTask
from .parking_speech import TEAM_PARKING_PHRASE
from .parking_control import parking_blockers

STAGES=('arming','wait_board','crosswalk','traffic','cones','parking_search','parking_entry','payment','complete')
LABELS={'handoff':'正在交接控制权','idle':'手动遥控／待切换','arming':'自动模式初始化','wait_board':'等待发车挡板移开',
        'crosswalk':'斑马线停车与播报','traffic':'红绿灯停车通行','cones':'两桶避障',
        'parking_search':'寻找空闲停车半区','parking_entry':'自动入位与停稳',
        'payment':'30 秒扫码缴费','complete':'流程结束','fault':'已停止，需重新准备','cancelled':'已退出自动模式'}


def continuous_stillness(motion, now):
    motion = motion or {}
    stamp, since = motion.get('captured_s'), motion.get('continuous_still_since_s')
    if (motion.get('valid') is True and motion.get('moving') is False
            and numeric(stamp) and numeric(since) and since <= stamp
            and 0 <= now-stamp <= MOTION_MAX_AGE_S):
        return since, stamp
    return None


def lane_edge_continuity(previous, current):
    """Compare both inner edges at a shared image row in the same camera."""
    old_rows, old_bounds = previous
    rows, bounds = current
    if not all(isinstance(v, (tuple, list)) and len(v) == 2 for v in (old_rows, old_bounds, rows, bounds)):
        return False
    if not all(numeric(v) for v in (*old_rows, *rows)):
        return False
    row = min(old_rows[1], rows[1])
    if row < max(old_rows[0], rows[0]) or old_rows[1] <= old_rows[0] or rows[1] <= rows[0]:
        return False

    def at(y_rows, x_bounds):
        if not all(isinstance(pair, (tuple, list)) and len(pair) == 2
                   and all(numeric(x) for x in pair) and pair[1] > pair[0] for pair in x_bounds):
            return None
        fraction = (row-y_rows[0]) / (y_rows[1]-y_rows[0])
        return [start + fraction*(end-start) for start, end in zip(*x_bounds)]

    old = at(old_rows, old_bounds)
    new = at(rows, bounds)
    if old is None or new is None:
        return False
    tolerance = .25 * (old[1]-old[0])
    return all(abs(a-b) <= tolerance for a, b in zip(old, new))


def validate_flow(s):
    s=copy.deepcopy(s)
    if s.get('max_frame_age_s')==.25:s['max_frame_age_s']=VISION_MAX_AGE_S
    if s.get('schema_version')!=1:raise ValueError('总流程配置版本无效')
    for k,lo,hi in [('forward_us',1570,1600),('left_us',1650,LEFT_US),('right_us',RIGHT_US,1570)]:
        if type(s.get(k)) is not int or not lo<=s[k]<=hi:raise ValueError('总流程参数越界：'+k)
    for k,lo,hi in [('max_run_s',60,600),('max_stage_s',20,180),('parking_search_s',5,60)]:
        if not numeric(s.get(k)) or not lo<=s[k]<=hi:raise ValueError('总流程参数越界：'+k)
    for k,value in [('crosswalk_hold_s',10),('payment_window_s',30),('max_frame_age_s',VISION_MAX_AGE_S),('page_lease_s',.5),('arming_s',3)]:
        if not numeric(s.get(k)) or s[k]!=value:raise ValueError('本版比赛流程固定参数：'+k)
    if s.get('school_team_speech')=='我是山东大学启航队的智能车，请为我加油':
        s['school_team_speech']=TEAM_PARKING_PHRASE
    if s.get('school_team_speech')!=TEAM_PARKING_PHRASE:raise ValueError('学校队伍播报未匹配')
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
        if 'forward_us' in saved:
            traffic['pwm']['search_us']=traffic['pwm']['creep_us']=saved['forward_us']
        for key in ('red_confirm_percent','max_approach_s'):
            if key in saved:traffic[key]=saved[key]
    cones=load_cones(configs/('cone-trial.local.json' if local('cone-trial') else 'cone-trial.json'))
    parking=load_parking(configs/('parking-bay.local.json' if local('parking-bay') else 'parking-bay.json'))
    departure=local('departure') or {'min_blue_percent':DEFAULT_MIN_BLUE_PERCENT,'clear_confirm_s':DEFAULT_CLEAR_CONFIRM_S}
    # The application passes explicit task settings; no wrapper/class-name walk.
    values={'brake':brake,'traffic':traffic,'cones':cones,'parking':parking,'departure':departure}
    supplied=getattr(drive,'task_settings',{}) if drive is not None else {}
    supplied=supplied() if callable(supplied) else supplied
    for name,settings in (supplied or {}).items():
        if name in values:values[name]=copy.deepcopy(settings)
    values['brake']=crosswalk_calibrated_settings(values['brake'],root)
    if not values['cones']['ground'].get('historical_verified') and not values['cones']['ground'].get('verified'):
        reference=json.loads((configs/'cone-legacy-ground-candidate.json').read_text(encoding='utf-8'))
        values['cones']=reuse_legacy_ground(values['cones'],reference)
    return prepare_bundle({'flow':flow,**values})


def prepare_bundle(bundle):
    b=copy.deepcopy(bundle);f=validate_flow(b['flow']);b['flow']=f
    b['brake']['max_frame_age_s']=VISION_MAX_AGE_S;b['brake']['actuator_lease_s']=VISION_MAX_AGE_S
    b['brake']['hold_s']=10.;b['brake']['speech_text']=f['school_team_speech']
    b['brake']['max_approach_s']=f['max_stage_s']
    b['brake']['pwm']['search_us']=f['forward_us']
    b['brake']['pwm']['creep_us']=min(1575,f['forward_us'])
    b['brake']['brake_release_s']=max(.1,b['brake']['brake_release_s'])
    b['traffic']=approach_settings(b['traffic'])
    b['cones']=trial_settings(b['cones'],{'test_forward_us':f['forward_us'],'test_left_us':f['left_us'],
                                        'test_right_us':f['right_us'],'test_run_s':min(120,f['max_stage_s'])})
    geometry=b['cones']['geometry']
    for target,source in [('lane_width_m','lane_width_m'),('vehicle_width_m','vehicle_width_m'),
                          ('front_extent_m','rear_axle_to_front_m'),('rear_extent_m','rear_overhang_m')]:
        if b['parking']['perception_verified'] and abs(b['parking'][target]-geometry[source])>.005:
            raise ValueError('停车与车辆尺寸配置不一致：'+target)
        b['parking'][target]=geometry[source]
    return b

class FlowTrafficController(TrafficDrivingController):
    """Reuse the light/stop vote machine after the shared ESC initialization.

    Full-route search has the full-stage time budget; the independently exposed
    traffic trial keeps its original 10-second search and 3-second arming.
    """
    def __init__(self,settings,now,stage_limit,*,external_cadence=False):
        super().__init__(settings, control_interval_s=None if external_cadence else VISION_MAX_AGE_S)
        self.s=copy.deepcopy(self.s);self.s['max_approach_s']=stage_limit
        self.s['max_wait_green_s']=min(18.,self.s['max_wait_green_s'])
        self.started=self.approach_at=now;self.phase='approach'
        self.reason='沿赛道接近红绿灯，等待实际灯色'

    def recover_control(self, now):
        self.last_t = None
        self.no_progress_since = None
        self.votes.reset_green()
        self.stop_motion.begin_brake(preserve_slide=True)
        self.stop_motion.begin_neutral(now)
        self.stop_since = self.neutral_hold_since = None
        self.release_until = now + self.s['brake_release_s'] + .02
        self.recovery_motion_capture = None
        if self.phase in ('motion_recovery', 'approach'):
            self.phase = 'approach'
        elif self.phase in ('brake_pulse', 'settle'):
            self.phase = 'settle'
            self.brake_at = now


class RaceFlowController:
    def __init__(self,bundle,now,*,initialized=False,recover_gaps=False):
        self.b=prepare_bundle(bundle);self.s=self.b['flow'];self.stage='arming';self.started=self.entered=now
        self.reason='保持中位 3 秒，随后等待蓝色发车挡板';self.last_t=None;self.last_seq=None
        self.controller=None;self.departure=DepartureSequence(self.b['departure'].get('clear_confirm_s',DEFAULT_CLEAR_CONFIRM_S))
        self.guide=None;self.last_module_id=None;self.last_intent=None;self.events=[];self.transitions=[]
        self.parking=ParkingTask(self.b['parking'],self.b['cones'],require_ready=False,
                                 max_control_gap_s=RACE_CONTROL_WINDOW_S);self.parking_wait=None
        self.payment_started=None;self.paid=False;self.paid_in_time=False;self.last_drive=False;self.audio_wait=None;self.stationary_since=None
        self.passed_cone_ids=set()
        self.vision_recovery_started=None
        self.vision_recovery_needs_lane=False
        self.vision_recovery_brake_until=0.
        self.vision_recovery_hits=0
        self.vision_recovery_frame=None
        self.recover_gaps = recover_gaps
        self.control_pause = None
        self.control_stillness = StillnessWindow()
        self.control_evidence_seen = None
        self.control_resume_frame = None
        self.autonomy_started = None
        self.observed_stationary_s = None
        self.lane_reference = None
        self.lane_recovery = StablePairedLane(max_gap_s=RACE_CONTROL_WINDOW_S)
        self.lane_resume_after = None
        self.crosswalk_moved_at = None
        if initialized:
            self._go('wait_board',now,'已确认共用执行器零油门及连续静止，等待蓝色挡板')

    def _go(self,stage,now,reason):
        self.transitions.append({'from':self.stage,'to':stage,'at_s':now,'reason':reason})
        self.stage=stage;self.entered=now;self.reason=reason;self.last_module_id=None;self.last_intent=None
        if stage=='crosswalk':self.autonomy_started=now
        if self.guide is None:self.guide=LaneFollower(self.b['cones'],now)
        self.guide.transition(stage)
        if stage=='crosswalk':self.controller=BrakeParkingController(self.b['brake'],'straight',first_detection_stop=True,crosswalk=True,control_interval_s=None if self.recover_gaps else VISION_MAX_AGE_S)
        elif stage=='traffic':self.controller=FlowTrafficController(self.b['traffic'],now,self.s['max_stage_s'],external_cadence=self.recover_gaps)
        elif stage=='cones':self.controller=ContinuousConeController(self.b['cones'],now,initialized=True,handoff_after_two=True)

    def frame_age_limit(self):
        if self.stage=='crosswalk' and self.b['brake'].get('crosswalk_calibration') is not None:
            return max(self.s['max_frame_age_s'],self.b['brake'].get('crosswalk_max_frame_age_s',0.))
        return self.s['max_frame_age_s']

    def control_evidence(self, sample):
        """Identify the actual stage input instead of the newest unrelated camera."""
        sample = sample or {}
        stage = self.stage
        if stage == 'crosswalk' and self.recover_gaps and self.controller.phase == 'done':
            row = sample.get('navigation')
            limit = VISION_MAX_AGE_S
        elif stage in ('crosswalk', 'traffic'):
            row = sample.get(stage)
            limit = self.frame_age_limit()
        elif stage == 'parking_entry':
            row = sample.get('parking')
            limit = self.b['parking']['max_frame_age_s']
        else:
            row = sample.get('navigation') if stage in ('cones', 'parking_search') else sample
            limit = self.frame_age_limit()
        row = row or {}
        fid = row.get('frame_id', sample.get('sequence'))
        return (row.get('source_camera'), fid, row.get('captured_s')), limit

    def forward_evidence_until(self, sample, previous_stage):
        sample = sample or {}
        nav = sample.get('navigation') or {}
        lane_stamp = nav.get('captured_s')
        if self.stage == 'parking_entry':
            (_, _, stamp), limit = self.control_evidence(sample)
            pose_stamp = ((sample.get('parking') or {}).get('pose') or {}).get('captured_s')
            return min(stamp, pose_stamp) + limit if numeric(stamp) and numeric(pose_stamp) else None
        if not numeric(lane_stamp):
            return None
        if self.stage == previous_stage and self.stage in ('crosswalk', 'traffic'):
            (_, _, stamp), limit = self.control_evidence(sample)
            if not numeric(stamp):
                return None
            return min(lane_stamp + VISION_MAX_AGE_S, stamp + limit)
        return lane_stamp + VISION_MAX_AGE_S

    def _crosswalk_control_pause(self, now):
        task = self.controller
        if task.phase == 'target_recovery':
            task.recover_target_control(now)
            return
        task.last_t = None
        task.metric_history.clear()
        task.no_progress_since = task.stop_since = task.hold_since = None
        if task.phase in ('brake_pulse', 'settle', 'hold'):
            task.phase = 'settle'
            task.brake_started = now
            task.release_until = now + task.s['brake_release_s']
        elif task.seen_at is not None:
            task.seen_at = now

    def _control_recovery(self, sample, now, interrupted, gap, *, motion=None, lane_lost=False):
        identity, limit = self.control_evidence(sample)
        _, fid, stamp = identity
        fresh = type(fid) is int and numeric(stamp) and 0 <= now-stamp <= limit
        # Capture age answers whether this observation is usable. Time between
        # decisions is supervised by the sender, not inferred from skipped
        # camera frames (which are normal under processing load).
        if self.control_pause is None and not (interrupted or gap or not fresh or lane_lost):
            if self.control_resume_frame is not None:
                stage, resumed = self.control_resume_frame
                if stage == self.stage and stamp <= resumed[2]:
                    return self._out('控制更新恢复，等待下一张不同画面')
                self.control_resume_frame = None
            self.control_evidence_seen = (self.stage, identity)
            return None
        if self.control_pause is None:
            needs_lane = lane_lost or self.last_drive or self.stage in ('cones', 'parking_search')
            cause = ('lane_missing' if lane_lost else 'sender_deadline' if interrupted else
                     'lane_frame_stale' if gap and fresh else 'task_frame_stale')
            self.control_pause = {'at': now, 'until': now + (.35 if self.last_drive and not interrupted else 0.),
                                  'hits': 0, 'cadence_hits':0, 'frame': None, 'needs_lane': needs_lane, 'lane_lost': lane_lost,
                                  'cause': cause}
            self.control_stillness.reset()
            self.lane_recovery.reset()
            if self.guide is not None:
                self.guide.suspend()
        pause = self.control_pause
        nav = (sample or {}).get('navigation') or {}
        if (pause['lane_lost'] and nav.get('boundary_danger')
                and numeric(nav.get('captured_s')) and 0 <= now-nav['captured_s'] <= VISION_MAX_AGE_S):
            return self.fail(nav.get('reason') or '巡线边界要求停止')
        lane_ready = True
        if pause['lane_lost']:
            lane_ready = self._same_lane_again(nav, now, pause['at'])
        if self.stage in ('crosswalk', 'traffic', 'cones', 'parking_search'):
            lane_t = nav.get('captured_s')
            if pause['needs_lane']:
                fresh = fresh and paired_lane(nav) and numeric(lane_t) and 0 <= now-lane_t <= VISION_MAX_AGE_S
        motion = motion if motion is not None else (sample or {}).get('motion') or {}
        parking_still = None
        if self.stage == 'parking_entry' and fresh:
            parking = sample['parking']
            speed = parking.get('speed_mps')
            pose = parking.get('pose') or {}
            pose_stamp = pose.get('captured_s')
            settings = self.b['parking']
            pose_valid = (pose.get('verified') is True
                          and all(numeric(pose.get(key)) for key in
                                  ('x_m','y_m','yaw_rad','error_bound_m','captured_s'))
                          and 0 <= now-pose_stamp <= settings['max_frame_age_s']
                          and numeric(settings.get('maximum_pose_error_m'))
                          and 0 <= pose['error_bound_m'] <= settings['maximum_pose_error_m']
                          and parking.get('coordinate_frame') == 'parking_road_world'
                          and parking.get('obstacle_region_verified') is True
                          and numeric(speed) and numeric(settings.get('max_speed_mps'))
                          and abs(speed) <= settings['max_speed_mps'])
            independent_stamp = motion.get('captured_s')
            moving_independently = (motion.get('valid') is True and motion.get('moving') is True
                                    and numeric(independent_stamp)
                                    and 0 <= now-independent_stamp <= MOTION_MAX_AGE_S)
            moving_pose = (pose.get('verified') is True and numeric(pose_stamp)
                           and 0 <= now-pose_stamp <= settings['max_frame_age_s']
                           and numeric(speed) and abs(speed) > settings['stopped_speed_mps'])
            if moving_independently or moving_pose:
                stamps = ([independent_stamp] if moving_independently else []) + ([pose_stamp] if moving_pose else [])
                pause['parking_motion_conflict_after'] = max(
                    max(stamps), pause.get('parking_motion_conflict_after', -math.inf))
            fresh = fresh and pose_valid and not moving_independently and not moving_pose
            if fresh:
                parking_still = continuous_stillness(motion, now)
                conflict = pause.get('parking_motion_conflict_after')
                if parking_still is not None and conflict is not None and parking_still[0] <= conflict:
                    parking_still = None
            # Slow parking calculations cannot reconstruct a continuous stop
            # from isolated poses. Preserve the independent sampler's original
            # interval, but corroborate it with current valid parking feedback.
            motion = {'captured_s': pose_stamp, 'valid': fresh, 'moving': not fresh}
        previous = pause['frame']
        if fresh and stamp >= pause['at'] and (previous is None or stamp > previous[2] and identity != previous):
            timely = previous is not None and identity[0]==previous[0] and stamp-previous[2]<=RACE_CONTROL_WINDOW_S
            rest = parking_still if self.stage=='parking_entry' else continuous_stillness(motion,now)
            bridges = (previous is not None and rest is not None and identity[0]==previous[0]
                       and rest[0]<=previous[2] and rest[1]>=stamp)
            if previous is not None and not timely and not bridges:pause['hits'] = 0
            # Continuous independent rest may connect sparse observations, but
            # cannot prove the planner can sustain forward control. Require two
            # timely decisions too; otherwise resuming would stop every cycle.
            pause['cadence_hits'] = pause['cadence_hits']+1 if timely else 1
            pause['frame'] = identity
            pause['hits'] += 1
        elif not fresh:
            pause['hits'] = 0
            pause['cadence_hits'] = 0
            pause['frame'] = None
        motion_stamp = motion.get('captured_s')
        pause['task_age_s'] = now-stamp if numeric(stamp) else None
        pause['lane_age_s'] = now-nav['captured_s'] if numeric(nav.get('captured_s')) else None
        stopped = self.stage in ('arming', 'wait_board') or self.control_stillness.update(
            motion if numeric(motion_stamp) and motion_stamp >= pause['at'] else None, now)
        still = parking_still if self.stage == 'parking_entry' else continuous_stillness(motion, now)
        if still is not None and still[1]-max(still[0], pause['until']) >= .3-1e-9:
            stopped = True
        if stopped and fresh and lane_ready and pause['hits']>=2 and pause['cadence_hits']>=2 and now>=pause['until']+.1:
            if self.stage == 'wait_board':
                self.departure.restart_confirmation()
            elif self.stage == 'crosswalk':
                self._crosswalk_control_pause(now)
            elif self.stage == 'traffic':
                self.controller.recover_control(now)
            elif self.stage == 'cones':
                self.controller.recover_control(now)
            elif self.stage == 'parking_entry':
                self.parking.recover_control()
            self.control_evidence_seen = (self.stage, identity)
            self.control_resume_frame = (self.stage, identity)
            if pause['lane_lost']:
                self.lane_resume_after = (self.lane_recovery.identity[1], self.lane_recovery.last)
            self.control_pause = None
            return self._out('控制更新恢复，下一帧重新判断当前任务',
                             {'action':'neutral','esc_us':1500,'steering_us':1610,'control_recovered':True})
        braking = now < pause['until']
        reason = ('巡线暂失，停车等待连续白线确认' if pause['lane_lost'] else
                  '控制更新中断，保持当前任务并等待新鲜反馈')
        if stopped and fresh and lane_ready and pause['hits']>=2 and pause['cadence_hits']<2:
            reason='停稳和当前目标已重新确认，等待识别处理间隔恢复后再继续'
        return self._out(reason,
                         {'action':'brake' if braking else 'neutral','esc_us':1400 if braking else 1500,
                          'steering_us':1610,'control_recovering':True})

    def accept_early_stop(self,event,now):
        if self.stage != 'traffic':return None
        intent=self.controller.accept_early_stop(event,now)
        if intent is None:return None
        if intent.get('phase')=='fault':return self.fail(intent['reason'])
        self.last_intent=intent
        return self._out(intent['reason'],intent)

    def _out(self,reason=None,intent=None,events=None):
        raw=intent or {'action':'neutral','esc_us':1500,'steering_us':1610}
        if reason:self.reason=reason
        result={**raw,'phase':self.stage,'stage_label':LABELS[self.stage],'reason':self.reason,
                'events':events or [],'hardware_output':False,'field_verified':False}
        pause = getattr(self, 'control_pause', None)
        if pause is not None:
            result['recovery_diagnostics'] = {key:pause.get(key) for key in
                ('cause','at','hits','cadence_hits','task_age_s','lane_age_s','needs_lane','lane_lost')}
        if self.stage in ('arming','wait_board'):
            result['departure_diagnostics']=self.departure.diagnostics()
        cone_limited=result['action'].startswith('cone_')
        if result['esc_us']>1500 and not result['action'].startswith('parking_') and not cone_limited:
            result['action']=('traffic_approach' if self.stage=='traffic' else
                              'creep' if result['esc_us']==self.b['brake']['pwm']['creep_us'] else 'search')
        self.last_drive=result['esc_us']>1500
        if self.guide is not None and result.get('fresh_control') is not False:
            self.guide.observe_output(result['steering_us'],self.last_t if self.last_t is not None else self.entered)
        return result

    def fail(self,reason):
        self.stage='fault';return self._out(reason)

    def report_paid(self,now):
        if self.stage!='payment':raise ValueError('尚未停入车位并进入缴费阶段')
        self.paid=True;self.paid_in_time=0<=now-self.payment_started<=30
        self._go('complete',now,'已记录队员付款声明；支付平台到账结果未自动核验')

    def _guided(self,nav,now,base=None):
        # Lane feedback supplies direction and a forward ceiling only. It has
        # no cone ownership, ESC arming or authority to override a task brake.
        decision=dict(base or {'action':'search','esc_us':self.s['forward_us']})
        if decision['esc_us']<=1500:
            self.guide.suspend()
            return decision
        if not nav:
            if self.recover_gaps:
                raise LaneRecoveryRequired('缺少新鲜相机引导')
            raise ValueError('缺少新鲜相机引导')
        if self.recover_gaps and (not numeric(nav.get('captured_s'))
                                 or not 0<=now-nav['captured_s']<=VISION_MAX_AGE_S):
            raise LaneRecoveryRequired('巡线画面暂时失效，等待新鲜白线')
        if self.lane_resume_after is not None:
            if nav.get('boundary_danger'):
                raise ValueError(nav.get('reason') or '巡线边界要求停止')
            if not self._matches_lane_reference(nav, now):
                raise LaneRecoveryRequired('重新起步前白线不一致')
            confirmed_frame, confirmed_time = self.lane_resume_after
            if nav['frame_id'] <= confirmed_frame or nav['captured_s'] <= confirmed_time:
                return {**decision, 'action':'neutral', 'esc_us':1500, 'steering_us':1610,
                        'reason':'等待下一张不同的白线画面后再起步'}
            self.lane_resume_after = None
        steer=self.guide.update(nav,now)
        if self.recover_gaps and steer.get('fresh_control') and paired_lane(nav):
            self.lane_reference = (nav['source_camera'], *measurements(nav)[:2], nav.get('frame_id'),
                                   (nav.get('lane_rows'), nav.get('lane_bounds_px')))
        result={**decision,**steer}
        pulse=min(decision['esc_us'],steer['forward_limit_us'])
        result['esc_us']=pulse
        if pulse==1500:
            result.update(action='neutral',steering_us=1610,
                reason='边界外向旧舵撤回或等待当前白线，保持中位')
        elif pulse!=decision['esc_us']:
            result['action']='cone_'+str(pulse)  # Existing frozen 1570/1580 profiles.
        if steer.get('line_search') and pulse>1500:
            result['reason']='巡线暂失，在有限恢复预算内保持有界方向'
        return result

    def _matches_lane_reference(self, nav, now):
        stamp = nav.get('captured_s')
        if (not paired_lane(nav) or nav.get('source_camera') not in ('primary','secondary')
                or type(nav.get('frame_id')) is not int or nav['frame_id'] < 0
                or not numeric(stamp) or not 0 <= now-stamp <= VISION_MAX_AGE_S
                or nav.get('boundary_margin_warning') or nav.get('action')=='stop'):
            return False
        try:
            near, far, _ = measurements(nav)
        except ValueError:
            return False
        half = nav.get('half_envelope')
        if not (numeric(half) and 0 < half < .5 and half < near < 1-half and 0 < far < 1):
            return False
        if self.lane_reference is None:
            return True
        camera, old_near, old_far, old_frame, old_edges = self.lane_reference
        return (nav['source_camera'] == camera and (old_frame is None or nav['frame_id'] > old_frame)
                and max(abs(near-old_near), abs(far-old_far)) <= .25
                and lane_edge_continuity(old_edges, (nav.get('lane_rows'), nav.get('lane_bounds_px'))))

    def _same_lane_again(self, nav, now, since):
        matches = self._matches_lane_reference(nav, now) and nav['captured_s'] >= since
        previous = self.lane_recovery.identity
        if matches and previous is not None and previous[0] == nav['source_camera']:
            matches = nav['frame_id'] >= previous[1]
        return self.lane_recovery.update(nav if matches else None, now)

    def _hold_crosswalk(self, sample, now, motion):
        task = self.controller
        if task.phase != 'hold' or task.preliminary_stop:
            return None
        still = continuous_stillness(motion, now)
        if still is None:
            if (motion and motion.get('valid') is True and motion.get('moving') is True
                    and numeric(motion.get('captured_s')) and 0 <= now-motion['captured_s'] <= MOTION_MAX_AGE_S):
                if self.crosswalk_moved_at is None:
                    self.crosswalk_moved_at = now
                if now-self.crosswalk_moved_at > task.s['brake_timeout_s']:
                    return self.fail('斑马线停车期间重新检测到移动，未恢复停稳')
                decision = task.brake_during_hold(now)
                return self._out(decision['reason'], decision)
            return self._out('斑马线保持中位，等待连续停稳反馈')
        if self.crosswalk_moved_at is not None or still[0] > task.hold_since:
            stripe = (sample or {}).get('crosswalk')
            if (still[1]-still[0] < task.s['stop_confirm_s']-1e-9
                    or not task.confirms_stop_reference(stripe, now) or stripe['captured_s'] < still[0]):
                return self._out('停稳反馈曾中断，保持中位并重新确认停车位置')
            task.hold_since = now
            self.crosswalk_moved_at = None
        elapsed = max(0., still[1]-task.hold_since)
        remaining = max(0., task.s['hold_s']-elapsed)
        task.last_t = now
        task.reason = '斑马线保持中位，按连续停稳反馈计时'
        if remaining <= 1e-9:
            task.phase, task.reason = 'done', '停车流程完成，保持停止'
            self.control_evidence_seen = None  # The next input is a lane frame, not a zebra frame.
        decision = task.output()
        decision['hold_remaining_s'] = remaining
        self.last_intent = decision
        return self._out(task.reason, decision)

    def step(self,sample,now,audio=None,*,interrupted=False,motion=None):
        audio=audio or {}
        if self.stage in ('complete','fault'):return self._out()
        if self.stage=='payment':
            if now-self.payment_started>=30:self._go('complete',now,'30 秒缴费窗口结束，车辆保持停止')
            return self._out()
        if not numeric(now) or self.last_t is not None and now<self.last_t:return self.fail('控制时钟倒退')
        self.last_t=now
        if now-self.started>self.s['max_run_s']:return self.fail('总流程超时，已停止')
        if now-self.entered>self.s['max_stage_s']:return self.fail('当前任务超时，已停止')
        motion=motion if motion is not None else (sample or {}).get('motion')
        if self.recover_gaps and self.stage not in ('arming','wait_board'):
            still = continuous_stillness(motion, now)
            self.observed_stationary_s = None
            if still is not None:
                start = max(still[0], self.autonomy_started if self.autonomy_started is not None else self.entered)
                self.observed_stationary_s = max(0., still[1]-start)
                if self.observed_stationary_s > 20:
                    return self.fail('自动阶段连续静止超过 20 秒，结束本轮并保持停止')
            if self.stage == 'crosswalk':
                hold = self._hold_crosswalk(sample, now, motion)
                if hold is not None:return hold
        if self.recover_gaps:
            recovery = self._control_recovery(sample,now,interrupted,False,motion=motion)
            if recovery is not None:
                return recovery
        stamp=(sample or {}).get('captured_s')
        fresh=numeric(stamp) and 0<=now-stamp<=self.frame_age_limit()
        nav=(sample or {}).get('navigation')
        if self.vision_recovery_needs_lane or self.last_drive and nav is not None:
            t=(nav or {}).get('captured_s');fresh=fresh and numeric(t) and 0<=now-t<=VISION_MAX_AGE_S
        if self.recover_gaps and not fresh:
            return self._control_recovery(sample,now,interrupted,True,motion=motion)
        if not fresh or self.vision_recovery_started is not None:
            recovering=self._recover_vision(sample,now,fresh)
            if recovering is not None:return recovering

        if type(sample.get('sequence')) is not int:return self.fail('相机批次编号无效')
        if sample['sequence']<0 or self.last_seq is not None and sample['sequence']<self.last_seq:
            return self.fail('相机批次编号倒退，结束本轮')
        if sample.get('sequence')==self.last_seq:return None
        self.last_seq=sample.get('sequence')
        motion = motion or {}
        # The independent sampler may have advanced while vision was working.
        # Give the task the same original-timestamp motion row used by recovery;
        # never refresh image or motion timestamps to the decision time.
        if self.stage in ('crosswalk','traffic') and isinstance(sample.get(self.stage),dict):
            sample={**sample,self.stage:{**sample[self.stage],'motion':dict(motion)}}
        if not self.recover_gaps and self.stage not in ('arming','wait_board') and motion.get('valid') is True and 0<=now-motion.get('captured_s',-1e9)<=.25:
            if motion.get('moving') is True:self.stationary_since=None
            elif motion.get('moving') is False:
                if self.stationary_since is None:self.stationary_since=now
                if now-self.stationary_since>=19:return self.fail('自动阶段连续停止接近 20 秒，结束本轮并保持停止')
        try:return self._step_stage(sample,now,audio)
        except LaneRecoveryRequired as exc:
            if not self.recover_gaps:return self.fail(str(exc))
            return self._control_recovery(sample,now,interrupted,False,motion=motion,lane_lost=True)
        except (ValueError,KeyError,TypeError) as exc:return self.fail(str(exc))

    def _recover_vision(self,sample,now,fresh):
        if self.vision_recovery_started is None:
            self.vision_recovery_started=now
            self.vision_recovery_needs_lane=self.last_drive
            self.vision_recovery_brake_until=now+(.35 if self.last_drive else 0.)
            self.vision_recovery_hits=0;self.vision_recovery_frame=None
            if self.guide:self.guide.suspend()
        if now-self.vision_recovery_started>=MOTION_RECOVERY_S:
            return self.fail('画面持续失效，恢复等待超时，保持停止')
        if fresh:
            identity=(sample.get('sequence'),sample.get('captured_s'))
            previous=self.vision_recovery_frame
            ordered=(type(identity[0]) is int and identity[0]>=0
                     and (self.last_seq is None or identity[0]>self.last_seq)
                     and (previous is None or identity[0]>previous[0] and identity[1]>previous[1]))
            if ordered:
                self.vision_recovery_frame=identity;self.vision_recovery_hits+=1
            else:fresh=False
        else:self.vision_recovery_hits=0
        if fresh and self.vision_recovery_hits>=2 and now>=self.vision_recovery_brake_until+.1:
            self.vision_recovery_started=None
            self.vision_recovery_needs_lane=False
            if self.guide:self.guide.suspend()
            return None
        braking=now<self.vision_recovery_brake_until
        return self._out('画面短暂失效，停车等待连续新画面恢复',
            {'action':'brake' if braking else 'neutral','esc_us':1400 if braking else 1500,'steering_us':1610})

    def _advance(self,stage,now,reason,sample,audio,events=None):
        # Dispatch the next task in the same camera batch. No neutral pulse
        # or repeated ESC initialization between moving mission stages.
        self._go(stage,now,reason)
        if stage in ('crosswalk','traffic') and not sample.get(stage):
            # The cached reader processes only the active task each batch.
            # Bridge to its first observation with live lane/image guidance,
            # not a neutral pulse or fabricated traffic/crosswalk observation.
            intent=self._guided(sample.get('navigation'),now)
            if intent['action'] in ('search','creep') and intent['esc_us']>1500:
                intent['esc_us']=self.b['brake' if stage=='crosswalk' else 'traffic']['pwm']['search_us']
            result=self._out(reason,intent)
        else:result=self._step_stage(sample,now,audio)
        result['events']=(events or [])+result.get('events',[])
        return result

    def _step_stage(self,sample,now,audio):
        if self.stage=='arming':
            if now-self.started>=3:self._go('wait_board',now,'等待先看到蓝色挡板，再确认移开')
            return self._out()
        if self.stage=='wait_board':
            ready=self.departure.update(sample.get('board',{}),now)
            if ready:
                return self._advance('crosswalk',now,'挡板移开已确认，前进并寻找斑马线',sample,audio)
            return self._out(self.departure.reason)
        if self.stage == 'crosswalk' and self.recover_gaps and self.controller.phase == 'done':
            return self._leave_crosswalk(sample, now, audio)
        if self.stage in ('crosswalk','traffic'):
            module=sample.get(self.stage);nav=sample.get('navigation')
            if not module or not 0<=now-module.get('captured_s',-1e9)<=self.frame_age_limit():raise ValueError('任务观测过期')
            new_frame = module['frame_id'] != self.last_module_id
            if new_frame:
                self.last_module_id=module['frame_id'];self.last_intent=self.controller.step(module,now)
                events=[{**e,'id':self.stage} for e in self.last_intent.get('events',[]) if e.get('event')=='speak']
            else:events=[]
            decision=copy.deepcopy(self.last_intent)
            if decision['phase']=='fault':raise ValueError(decision['reason'])
            if decision['phase']=='done':
                if self.stage=='crosswalk':
                    return self._leave_crosswalk(sample, now, audio, events)
                return self._advance('cones',now,'实际绿灯达标，继续行驶并开始两桶避障',sample,audio,events)
            decision=self._guided(nav,now,decision)
            if self.recover_gaps and not new_frame:
                decision['fresh_control'] = False
            return self._out(decision['reason'],decision,events)
        if self.stage=='cones':
            decision=self.controller.step(sample.get('navigation'),now)
            if decision['phase']=='review':raise ValueError('锥桶通过证据不足；全流程停止，不以人工跳过冒充自主通过')
            if decision['phase']=='complete':
                self.passed_cone_ids.update(self.controller.passed_ids)
                return self._advance('parking_search',now,'两桶通过流程结束，寻找停车区',sample,audio)
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
            self.last_module_id=parking['frame_id'];decision=self.parking.decide(parking,now)
            if decision['phase']=='fault':raise ValueError(decision['reason'])
            if decision['phase']=='parked':
                self.payment_started=now;self._go('payment',now,'正确半区内已取得停稳反馈，30 秒内由队员扫码支付 0.01 元')
                return self._out(events=[{'event':'payment_window_opened','amount_cents':1}])
            intent=self.parking.intent(decision,now,parking['captured_s'])
            return self._out(decision['reason'],intent)
        raise ValueError('未知比赛阶段')

    def _leave_crosswalk(self, sample, now, audio, events=None):
        if audio.get('crosswalk') == 'failed':
            raise ValueError('队伍播报发送失败')
        if audio.get('crosswalk') != 'command_sent':
            if self.audio_wait is None:
                self.audio_wait = now
            if now-self.audio_wait > 3:
                raise ValueError('队伍播报发送未确认，停止本轮')
            return self._out('已停够 10 秒，等待播报发送结果', events=events)
        nav = sample.get('navigation') or {}
        if self.recover_gaps:
            if nav.get('boundary_danger'):
                raise ValueError(nav.get('reason') or '巡线边界要求停止')
            if (not paired_lane(nav) or not numeric(nav.get('captured_s'))
                    or not 0 <= now-nav['captured_s'] <= VISION_MAX_AGE_S):
                return self._out('已停够 10 秒，保持中位并等待新鲜白线', events=events)
        return self._advance('traffic', now, '斑马线任务完成，接近红绿灯', sample, audio, events)

    def status(self,now):
        return {'stage':self.stage,'stage_label':LABELS[self.stage],'reason':self.reason,'transitions':list(self.transitions),
                'cone_progress':self.controller.progress() if self.stage=='cones' else None,
                'parking_missing':parking_blockers(self.b),'payment_remaining_s':None if self.payment_started is None else max(0,30-(now-self.payment_started)),
                'operator_reported_paid':self.paid,'reported_within_window':self.paid_in_time,'provider_verified':False,
                'observed_stationary_s':self.observed_stationary_s,
                'field_verified':False,'rules_crosswalk_hold_s':10,'manual_interventions':0}
