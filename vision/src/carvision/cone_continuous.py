"""Operator-ended cone trial: image guidance, no vision-triggered stop states.

Perception is not calibrated collision proof. An uncertain route is reported as
best effort, never as verified clearance. Camera/lease/output faults still stop
the owning session. This controller has no hardware I/O.
"""
from .brake_parking import numeric
from .cone_low_speed import LowSpeedController,DEFAULT_STEERING_GAIN,steering_gain
from .steering import CENTER_US, ImagePD


class ContinuousConeController(LowSpeedController):
    def __init__(self,settings,now,*,initialized=False,handoff_after_two=False):
        super().__init__(settings,now,stop_on_complete=False,initialized=initialized)
        self.steering_gain=steering_gain(self.options.get('steering_gain',DEFAULT_STEERING_GAIN))
        self.handoff_after_two=handoff_after_two
        self.actual_steering=1610.;self.requested_steering=1610
        self.filtered_target=.5;self.guidance_source=None;self.guidance_at=None
        self.unverified_passes=0;self.best_effort=False
        self.recentering=False;self.recenter_reason=None
        self.pd=ImagePD(kp=1/.12,kd=.18)
        self.last_pass_side=None;self.last_output=None

    def progress(self):
        return {**super().progress(),'manual_stop_only':not self.handoff_after_two,'can_confirm_pass':False,
                'max_run_s':None,'unverified_passes':self.unverified_passes,
                'target_fraction':self.filtered_target,'requested_steering_us':self.requested_steering,
                'output_steering_us':round(self.actual_steering),'steering_gain':self.steering_gain,
                'best_effort_route':self.best_effort,'steering_recentering':self.recentering,
                'steering_center_us':CENTER_US,'recenter_reason':self.recenter_reason,
                'steering_policy':'locked_cone_edge_pd','pd_error':self.pd.error,'pd_derivative':self.pd.derivative}

    def _begin_recenter(self,reason):
        # Retiring an avoidance target must retire its lateral command too.
        # Actual S3 output still returns through the existing slew limiter.
        self.side=None;self.target_fraction=.5;self.filtered_target=.5
        self.recentering=True;self.recenter_reason=reason
        self.pd.reset()

    def _forget_target(self):
        self.target=None;self.side=None;self.target_aliases=set();self.last_target_fraction=None
        self.near_seen=False;self.review_pending=False;self.tail_powered=0.
        self.pass_min_y=self.pass_last_y=self.pass_last_s=None
        self.pass_gate_hits=self.escape_exit_hits=self.approach_frames=0
        self._begin_recenter('当前避让目标已释放，转向回中')

    def _passed(self):
        self.last_pass_side=self.side
        super()._passed()
        self._begin_recenter('当前锥桶通过估计已完成，转向回中')

    @staticmethod
    def _positions(cone,half):
        reserve=.03;space=half+cone['half_width']+reserve;lo,hi=half+reserve,1-half-reserve
        return [(side,x) for side,x in [('left',cone['fraction']-space),('right',cone['fraction']+space)] if lo<=x<=hi]

    def _lane_target(self,cones,nav):
        half=nav['half_envelope'];ego=nav['ego_fraction']
        target=next((c for c in cones if c['id']==self.target),None)
        if target is None:
            return self.target_fraction if self.target and self.review_pending else .5
        if target.get('near_field_continuation'):return self.target_fraction
        choices=self._positions(target,half)
        if half+.03<=ego<=1-half-.03 and abs(ego-target['fraction'])>=half+target['half_width']+.03:
            choices.append(('left' if ego<target['fraction'] else 'right',ego))
        # Lock one side for the whole tracked cone. Prefer an alternating side
        # for the next cone only when that side has an observed usable target.
        if self.side is not None:
            choices=[p for p in choices if p[0]==self.side]
        elif self.last_pass_side:
            preferred='right' if self.last_pass_side=='left' else 'left'
            alternating=[p for p in choices if p[0]==preferred]
            if alternating:choices=alternating
        others=[c for c in cones if c['id']!=self.target and not c.get('near_field_continuation')]
        upcoming=others[0] if others else None
        def score(choice):
            side,x=choice
            change=.09 if self.side and side!=self.side else 0.
            next_choices=self._positions(upcoming,half) if upcoming else []
            if upcoming and abs(x-upcoming['fraction'])>=half+upcoming['half_width']+.03:
                next_choices.append(('stay',x))
            future=min((abs(y-x)+.2*abs(y-.5) for _,y in next_choices),default=abs(x-.5))
            return abs(x-self.filtered_target)+.2*abs(x-ego)+change+.3*future
        if choices:
            self.side,self.target_fraction=min(choices,key=score)
            return self.target_fraction
        # No confirmed gap on the locked side: retain direction and report the
        # uncertainty rather than switching across the cone on the next frame.
        self.best_effort=True;lo,hi=half+.03,1-half-.03
        if self.side is None:self.side='left' if target['fraction']>=ego else 'right'
        sign=-1 if self.side=='left' else 1
        desired=target['fraction']+sign*(half+target['half_width']+.03)
        self.target_fraction=max(lo,min(hi,desired))
        return self.target_fraction

    def step(self,nav,now):
        if not numeric(now) or now<self.last_step:raise ValueError('控制时钟异常，结束本轮')
        dt=now-self.last_step
        if not nav or not numeric(nav.get('captured_s')) or not 0<=now-nav['captured_s']<=.25:
            raise ValueError('实时相机画面中断，结束本轮')
        if not nav.get('experimental'):raise ValueError('持续绕桶画面格式不匹配')
        captured=nav['captured_s']
        lane=bool(nav.get('lane_bounds_px') and numeric(nav.get('ego_fraction')) and not nav.get('line_missing'))
        source=nav.get('source_camera');mode=(source,lane)
        if mode==self.guidance_source and self.guidance_at is not None and captured<self.guidance_at:
            raise ValueError('相机时间倒退，结束本轮')
        if mode==self.guidance_source and captured==self.guidance_at and self.last_output is not None:
            return dict(self.last_output)
        if self.phase=='driving':
            self.powered_s+=dt
            if self.review_pending:self.tail_powered+=dt
        self.last_step=now
        if now<self.arm_until:
            return {'action':'neutral','esc_us':1500,'steering_us':1610,'phase':'arming','reason':'3 秒中位初始化','experimental':True}
        self.phase='driving';self.best_effort=False
        previous_mode=self.guidance_source
        if self.guidance_source!=mode:
            self.guidance_source=mode
        cones=[c for c in nav.get('cones',[]) if c['id'] not in self.passed_ids
               and not c.get('provisional') and (lane and numeric(c.get('fraction')) or not lane and numeric(c.get('image_fraction')))]
        cones.sort(key=lambda c:c.get('y_fraction',0.),reverse=True)
        if self.source_camera and source!=self.source_camera and self.target:
            candidates=[c for c in cones if lane and self.last_target_fraction is not None
                        and not c.get('near_field_continuation')
                        and abs(c['fraction']-self.last_target_fraction)<.18]
            if len(candidates)==1:
                self.target=candidates[0]['id'];self.target_aliases.add(self.target)
                self.pass_min_y=self.pass_last_y=self.pass_last_s=None;self.pass_gate_hits=self.escape_exit_hits=0
            else:self._forget_target()  # Association loss is not a stop or a pass.
        self.source_camera=source
        fresh_targets=[c for c in cones if not c.get('near_field_continuation')]
        if self.target is None and fresh_targets:
            self.target=fresh_targets[0]['id'];self.target_aliases={self.target}
            # A newly observed obstacle may require immediate avoidance. A
            # cached near-field edge alone must never restart the old turn.
            self.recentering=False;self.recenter_reason=None
        target=next((c for c in cones if c['id']==self.target),None)
        if target and lane:
            self.last_target_fraction=target['fraction'];self.near_seen|=target.get('near',False)
            if self.side is None:self._lane_target(cones,nav)
            self._pass_observation(target,nav,-1 if self.side=='left' else 1)
        if self.target and (target is None or target.get('near_field_continuation')):
            self.review_pending=True
            if self.tail_powered>=self.options['tail_powered_limit_s']:
                alternating=self.last_pass_side is None or self.side!=self.last_pass_side
                if self.near_seen and self.pass_gate_hits>=3 and alternating:
                    self.auto_count+=1;self._passed()
                else:
                    self.unverified_passes+=1;self._forget_target()
                cones=[c for c in cones if c['id'] not in self.passed_ids]
        elif target:self.review_pending=False;self.tail_powered=0.

        reason='持续低速循迹，等待人工停止'
        if lane:
            self.line_missing_since=None;desired=self._lane_target(cones,nav);ego=nav['ego_fraction']
            if self.target:reason='锁定绕行侧，跟随锥桶边缘目标'
        else:
            if self.line_missing_since is None:self.line_missing_since=now
            desired=.5;ego=.5
            visible=next((c for c in cones if c['id']==self.target),None) or (cones[0] if cones else None)
            if visible and not visible.get('near_field_continuation'):
                x=visible['image_fraction'];half=visible.get('half_image_width',.03)
                if self.side is None:self.side='left' if x>=.5 else 'right'
                # Normalized image fallback; no fixed pixel offset or claim
                # of physical clearance when the white corridor is missing.
                desired=max(.1,min(.9,x+(-1 if self.side=='left' else 1)*(half+.15)))
            elif visible and visible.get('near_field_continuation'):
                desired=self.filtered_target if previous_mode==mode else .5
            reason='白线未确认，持续低速搜索和图像避让'
        if self.best_effort:reason='可用间隙未明确，保持低速向较宽一侧尝试避让'

        # Follow the edge target immediately. Filter the derivative, not the
        # target, so a newly visible cone does not incur the old 0.18 s lag.
        self.filtered_target=desired
        error=self.filtered_target-ego
        self.boundary_turn=None
        if lane:
            near=nav.get('ego_near_fraction',ego);soft=nav['half_envelope']+self.options['boundary_soft_fraction']
            if near<soft or nav.get('boundary_correction')=='right':
                error=max(error,.08+soft-near);self.boundary_turn='right'
            elif near>1-soft or nav.get('boundary_correction')=='left':
                error=min(error,-.08-(near-(1-soft)));self.boundary_turn='left'
            if self.boundary_turn:reason='白线预警，持续低速向'+('右' if self.boundary_turn=='right' else '左')+'纠偏'
        command=self.pd.update(error,captured,context=(mode,self.target),gain=self.steering_gain)
        if self.boundary_turn=='right':command=max(command,min(1.,self.steering_gain*error/.12))
        elif self.boundary_turn=='left':command=min(command,max(-1.,self.steering_gain*error/.12))
        endpoint=self.options['right_us'] if command>=0 else self.options['left_us']
        requested=1610+abs(command)*(endpoint-1610);self.requested_steering=round(requested)
        # Honor elapsed time at 5 Hz as well as 20 Hz. The first output remains
        # bounded; stale camera input and the independent actuator lease stop.
        steering_phase='boundary_correction' if self.boundary_turn else 'avoiding' if self.target else 'lane_follow' if lane else 'straight_search'
        if self.recentering and not self.boundary_turn:
            requested=CENTER_US;self.requested_steering=CENTER_US
            steering_phase='recentering';reason=self.recenter_reason
        change=1200*(.1 if self.last_output is None else max(0.,min(.25,dt)))
        self.actual_steering+=max(-change,min(change,requested-self.actual_steering))
        if self.recentering and not self.boundary_turn and abs(self.actual_steering-CENTER_US)<.5:
            self.actual_steering=float(CENTER_US);self.recentering=False
            self.filtered_target=ego
            self.pd.reset()
        self.guidance_at=captured
        if self.handoff_after_two and self.count>=2:self.phase='complete'
        self.last_output={'action':'search','esc_us':self.s['driving']['forward_us'],'steering_us':round(self.actual_steering),
                'phase':self.phase,'reason':reason,'experimental':True,'manual_stop_only':not self.handoff_after_two,
                'boundary_correction':self.boundary_turn,'best_effort_route':self.best_effort,
                'steering_phase':steering_phase,'requested_steering_us':self.requested_steering,
                'steering_policy':'locked_cone_edge_pd',
                'line_search':not lane,'boundary_observed':lane and not nav.get('line_held')}
        return dict(self.last_output)
