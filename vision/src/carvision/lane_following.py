"""Shared image-feedback steering, with no hardware or mission transitions.

Centreline feedback follows the general structure in sana-u/smartCar's
line.cpp. Multi-row heading, timing, gains and PWM are this vehicle's local
policy, not copied pixel thresholds or measured wheel angles.
"""
from .brake_parking import numeric
from .cone_evidence import paired_lane
from .cone_response import combine_guidance
from .steering import CENTER_US,ImagePD


STEERING_RATE_US_S=600.


def measurements(nav):
    near=nav.get('ego_near_fraction',nav.get('ego_fraction'))
    far=nav.get('ego_far_fraction',near)
    if not numeric(near) or not numeric(far):
        raise ValueError('当前白线缺少有效方向参考')
    heading=nav.get('heading_error',near-far)
    if not numeric(heading):raise ValueError('当前白线方向无效')
    return near,far,heading


class SteeringController:
    """One PD and endpoint mapping shared by normal and obstacle guidance."""
    def __init__(self,right_us,left_us):
        self.right,self.left=right_us,left_us
        self.pd=ImagePD(kp=2.5,kd=.08)

    def request(self,error,captured_s,*,context,gain=1.,boundary_turn=None):
        command=self.pd.update(error,captured_s,context=context,gain=gain)
        if boundary_turn=='right':command=max(command,min(1.,error*2.5))
        elif boundary_turn=='left':command=min(command,max(-1.,error*2.5))
        endpoint=self.right if command>=0 else self.left
        return CENTER_US+abs(command)*(endpoint-CENTER_US)


def advance_steering(previous,requested,dt,*,boundary_turn=None,near=None,half=None,warning=False):
    """Withdraw an urgent outward demand before permitting more forward power.

    Ordinary soft warnings retain the regular slew. Neutral withdrawal is a
    command policy, not a claim that the physical wheels have reached centre.
    """
    if not all(numeric(value) for value in (previous,requested,dt)) or dt<0:
        raise ValueError('方向输出时间或数值无效')
    outward=(boundary_turn=='right' and previous>CENTER_US+1
             or boundary_turn=='left' and previous<CENTER_US-1)
    critical=bool(warning or numeric(near) and numeric(half) and (near<half or near>1-half))
    if outward and critical:
        return float(CENTER_US),True
    change=STEERING_RATE_US_S*min(.25,dt)
    return previous+max(-change,min(change,requested-previous)),False


class LaneFollower:
    """Fresh-frame normal lane following. Gain is always one.

    The task controller owns throttle/brake decisions. Returned forward limits
    can only reduce a positive task request; they never replace a red brake.
    There is no ESC initialization or cone ownership in this class.
    """
    def __init__(self,settings,now,*,recovery_s=.4):
        options=settings['low_speed_test']
        self.control=SteeringController(options['right_us'],options['left_us'])
        self.actual=float(CENTER_US);self.last_now=now;self.last_result=None
        self.frames={};self.last_pair=None;self.missing_since=None
        self.recovery_s=recovery_s;self.context=None

    def transition(self,context):
        if self.context!=context:
            self.context=context;self.control.pd.reset()

    def suspend(self):
        """End moving-lane history during a task-requested stationary wait.

        Keep frame identities so a cached pre-stop pair cannot restart motion.
        Ordinary lane recovery/neutral decisions must not call this method.
        """
        self.last_pair=self.missing_since=self.last_result=None
        self.control.pd.reset()

    def observe_output(self,steering,now):
        if numeric(steering) and numeric(now) and now>=self.last_now:
            self.actual=float(steering);self.last_now=now

    def update(self,nav,now):
        if not isinstance(nav,dict) or not numeric(now) or now<self.last_now:
            raise ValueError('巡线输入或控制时钟无效')
        captured=nav.get('captured_s')
        if not numeric(captured) or not 0<=now-captured<=.25:
            raise ValueError('实时巡线画面中断')
        if nav.get('ready') is False or nav.get('boundary_danger') or nav.get('action')=='stop':
            raise ValueError(nav.get('reason') or '巡线边界要求停止')
        source=nav.get('source_camera');frame=nav.get('frame_id',captured)
        if source not in ('primary','secondary') or 'frame_id' in nav and (type(frame) is not int or frame<0):
            raise ValueError('巡线相机来源或帧号无效')
        if self.missing_since is not None and now-self.missing_since>=self.recovery_s-1e-9:
            raise ValueError(f'巡线丢失超过 {self.recovery_s:g} 秒恢复预算')
        old=self.frames.get(source)
        if old and (captured<old[1] or frame<old[0]):raise ValueError('巡线相机帧倒退')
        if old and (frame==old[0] or captured==old[1]):
            return {**self.last_result,'fresh_control':False} if self.last_result else {
                'steering_us':CENTER_US,'forward_limit_us':1500,'fresh_control':False}
        self.frames[source]=(frame,captured)
        dt=now-self.last_now;self.last_now=now
        paired=paired_lane(nav)
        age=nav.get('prediction_age_s',nav.get('line_age_s'))
        single=(nav.get('line_state')=='single' and numeric(age) and 0<=age<=.25
                and not nav.get('line_missing') and bool(nav.get('lane_bounds_px')))
        if paired:self.last_pair=captured;self.missing_since=None
        elif self.missing_since is None:
            self.missing_since=self.last_pair if self.last_pair is not None else captured
        if self.missing_since is not None and now-self.missing_since>=self.recovery_s-1e-9:
            raise ValueError(f'巡线丢失超过 {self.recovery_s:g} 秒恢复预算')
        quality='paired' if paired else 'single' if single else 'missing'
        boundary=None;withdraw=False;heading=0.;lateral=None;lookahead=None
        if paired or single:
            near,far,heading=measurements(nav)
            lateral=.5-near;lookahead=.5-far
            error,_=combine_guidance(lateral,heading,False)
            half=nav.get('base_half_envelope',nav['half_envelope']) if single else nav['half_envelope']
            soft=half+.06
            correction=nav.get('boundary_correction') if paired else None
            if near<soft or correction=='right':
                error=max(error,.08+soft-near);boundary='right'
            elif near>1-soft or correction=='left':
                error=min(error,-.08-(near-(1-soft)));boundary='left'
            requested=self.control.request(error,captured,context=(self.context,source,quality),boundary_turn=boundary)
            if single:requested=max(CENTER_US-80,min(CENTER_US+80,requested))
            self.actual,withdraw=advance_steering(self.actual,requested,dt,boundary_turn=boundary,
                near=near,half=half,warning=nav.get('boundary_margin_warning',False))
            limit=1570 if single or abs(requested-CENTER_US)>=156 or abs(heading)>=.15 else (
                1580 if abs(requested-CENTER_US)>=65 or abs(heading)>=.07 else 1600)
            reference='current_multi_row_lane' if paired else 'bounded_single_edge_prediction'
        else:
            # Unknown remains unknown; a bounded old request is not a newly
            # observed centreline, and cannot create cone passage evidence.
            requested=max(CENTER_US-80,min(CENTER_US+80,self.actual))
            self.actual=float(requested)
            limit=1570 if self.last_pair is not None else 1500
            reference='bounded_previous_request' if self.last_pair is not None else 'no_observed_lane'
        if withdraw:limit=1500
        if limit==1500:self.actual=float(CENTER_US)
        self.last_result={'steering_us':round(self.actual),'requested_steering_us':round(requested),
            'effective_steering_gain':1.,'fresh_control':True,'forward_limit_us':limit,
            'boundary_output_withdrawal':withdraw,'boundary_correction':boundary,
            'line_state':quality,'line_search':not paired,'boundary_observed':paired,
            'lane_reference_source':reference,'lateral_error':lateral,'heading_error':heading,
            'lookahead_error':lookahead,'steering_policy':'shared_multi_row_pd',
            'wheel_angle_verified':False}
        return dict(self.last_result)
