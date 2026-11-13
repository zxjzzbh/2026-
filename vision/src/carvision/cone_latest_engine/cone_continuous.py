"""Single supervised image controller; projections never switch the driver."""
from .brake_parking import numeric
from .cone_low_speed import LowSpeedController
from .cone_image_guidance import ImageConeGuidance
from .cone_timing import frame_fresh,frame_error
from .cone_power import output_request


class ContinuousConeController(LowSpeedController):
    def __init__(self,settings,now,*,initialized=False,handoff_after_two=False):
        super().__init__(settings,now,stop_on_complete=False,initialized=initialized)
        self.guidance=ImageConeGuidance(settings)
        self.guidance_state=None;self.last_output=None
        self.actual_steering=1610.;self.handoff_after_two=handoff_after_two

    def progress(self):
        g=self.guidance_state or {}
        return {'passed_count':0,'estimated_pass_count':0,'operator_confirmed_count':0,
                'target_id':g.get('target_id'),'side':g.get('side'),'can_confirm_pass':False,
                'manual_stop_only':True,'route_pause_active':False,
                'steering_policy':'continuous_image_guidance','image_guidance':g or None,
                'steering_gain':self.options.get('steering_gain',1.),
                'left_us':self.options['left_us'],'right_us':self.options['right_us'],
                'route_risk':g.get('route_risk',False),'route_replans':g.get('replans',0),
                'boundary_correction':g.get('boundary_correction'),
                'travel_evidence_valid':False,'motion_source':'image_only',
                'estimated_pose':None,'field_verified':False,
                'count_pending_reason':'本图像调试入口不把目标出画当成实际通过；由操作者结束本轮'}

    def step(self,nav,now):
        if not numeric(now) or now<self.last_step:raise ValueError('控制时钟异常，结束本轮')
        captured=(nav or {}).get('captured_s')
        if not frame_fresh(captured,now):raise ValueError(frame_error(captured,now))
        if not nav.get('experimental'):raise ValueError('持续绕桶画面格式不匹配')
        dt=min(.5,max(0.,now-self.last_step));self.last_step=now
        if now<self.arm_until:
            self.guidance_state=self.guidance.prime(nav,now)
            return {'action':'neutral','esc_us':1500,'steering_us':1610,
                    'phase':'arming','reason':'3 秒中位初始化','experimental':True}
        self.phase='driving'
        g=self.guidance.update(nav,now);self.guidance_state=g
        if g['stop_required']:raise ValueError(g['reason'])
        command=g['command']
        endpoint=self.options['right_us'] if command>=0 else self.options['left_us']
        requested=1610+abs(command)*(endpoint-1610)
        # All phases share the same output ramp, including camera/target changes.
        delta=600*min(.15,dt if self.last_output else .1)
        self.actual_steering+=max(-delta,min(delta,requested-self.actual_steering))
        forward=self.s['driving']['forward_us']
        action,throttle=output_request(forward,g['power_scale'])
        if action=='neutral':self.actual_steering=1610.;requested=1610
        span=(1610-self.options['right_us']) if self.actual_steering<=1610 else (self.options['left_us']-1610)
        self.guidance.applied_command=(1610-self.actual_steering)/span
        self.last_output={'action':action,'esc_us':throttle,
            'steering_us':round(self.actual_steering),'requested_steering_us':round(requested),
            'phase':'driving','reason':g['reason'],'steering_phase':g['phase'],
            'steering_policy':'continuous_image_guidance','experimental':True,'field_verified':False,
            'route_pause':False,'route_risk':g['route_risk'],'target_fraction':g['target_fraction'],
            'boundary_correction':g['boundary_correction'],'lane_margin':g['lane_margin'],
            'power_scale':g['power_scale'],'control_conflict':g['control_conflict']}
        return dict(self.last_output)
