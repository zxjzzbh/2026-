"""Operator-ended cone trial: image guidance, no vision-triggered stop states.

Perception is not calibrated collision proof. An uncertain route is reported as
best effort, never as verified clearance. Camera/lease/output faults still stop
the owning session. This controller has no hardware I/O.
"""
import math

from .brake_parking import numeric
from .cone_low_speed import LowSpeedController,DEFAULT_STEERING_GAIN,steering_gain


class ContinuousConeController(LowSpeedController):
    def __init__(self,settings,now,*,initialized=False,handoff_after_two=False):
        super().__init__(settings,now,stop_on_complete=False,initialized=initialized)
        self.steering_gain=steering_gain(self.options.get('steering_gain',DEFAULT_STEERING_GAIN))
        self.handoff_after_two=handoff_after_two
        self.actual_steering=1610.;self.requested_steering=1610
        self.filtered_target=.5;self.guidance_source=None;self.guidance_at=None
        self.unverified_passes=0;self.best_effort=False

    def progress(self):
        return {**super().progress(),'manual_stop_only':not self.handoff_after_two,'can_confirm_pass':False,
                'max_run_s':None,'unverified_passes':self.unverified_passes,
                'target_fraction':self.filtered_target,'requested_steering_us':self.requested_steering,
                'output_steering_us':round(self.actual_steering),'steering_gain':self.steering_gain,
                'best_effort_route':self.best_effort}

    def _forget_target(self):
        self.target=None;self.side=None;self.target_aliases=set();self.last_target_fraction=None
        self.near_seen=False;self.review_pending=False;self.tail_powered=0.
        self.pass_min_y=self.pass_last_y=self.pass_last_s=None
        self.pass_gate_hits=self.escape_exit_hits=self.approach_frames=0

    @staticmethod
    def _positions(cone,half):
        reserve=.03;space=half+cone['half_width']+reserve;lo,hi=half+reserve,1-half-reserve
        return [(side,x) for side,x in [('left',cone['fraction']-space),('right',cone['fraction']+space)] if lo<=x<=hi]

    def _lane_target(self,cones,nav):
        half=nav['half_envelope'];ego=nav['ego_fraction']
        target=next((c for c in cones if c['id']==self.target),None)
        if target is None:return .5
        if target.get('near_field_continuation'):return self.target_fraction
        choices=self._positions(target,half)
        if half+.03<=ego<=1-half-.03 and abs(ego-target['fraction'])>=half+target['half_width']+.03:
            choices.append(('left' if ego<target['fraction'] else 'right',ego))
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
        # No proven open gap: keep trying the widest image-space gap instead
        # of stopping on a geometric estimate. Do not claim it is collision-free.
        self.best_effort=True;lo,hi=half+.03,1-half-.03
        candidates=[lo+(hi-lo)*i/12 for i in range(13)]
        self.target_fraction=max(candidates,key=lambda x:min(abs(x-c['fraction'])-c['half_width'] for c in cones))
        self.side='left' if self.target_fraction<target['fraction'] else 'right'
        return self.target_fraction

    def step(self,nav,now):
        if not numeric(now) or now<self.last_step:raise ValueError('控制时钟异常，结束本轮')
        dt=now-self.last_step
        if not nav or not numeric(nav.get('captured_s')) or not 0<=now-nav['captured_s']<=.25:
            raise ValueError('实时相机画面中断，结束本轮')
        if not nav.get('experimental'):raise ValueError('持续绕桶画面格式不匹配')
        if self.phase=='driving':
            self.powered_s+=dt
            if self.review_pending:self.tail_powered+=dt
        self.last_step=now
        if now<self.arm_until:
            return {'action':'neutral','esc_us':1500,'steering_us':1610,'phase':'arming','reason':'3 秒中位初始化','experimental':True}
        self.phase='driving';self.best_effort=False
        lane=bool(nav.get('lane_bounds_px') and numeric(nav.get('ego_fraction')) and not nav.get('line_missing'))
        source=nav.get('source_camera');mode=(source,lane)
        if self.guidance_source!=mode:
            self.filtered_target=nav['ego_fraction'] if lane else .5
            self.guidance_source=mode;self.guidance_at=None
        cones=[c for c in nav.get('cones',[]) if c['id'] not in self.passed_ids
               and not c.get('provisional') and (lane and numeric(c.get('fraction')) or not lane and numeric(c.get('image_fraction')))]
        cones.sort(key=lambda c:c.get('y_fraction',0.),reverse=True)
        if self.source_camera and source!=self.source_camera and self.target:
            candidates=[c for c in cones if lane and self.last_target_fraction is not None
                        and abs(c['fraction']-self.last_target_fraction)<.18]
            if len(candidates)==1:
                self.target=candidates[0]['id'];self.target_aliases.add(self.target)
                self.pass_min_y=self.pass_last_y=self.pass_last_s=None;self.pass_gate_hits=self.escape_exit_hits=0
            else:self._forget_target()  # Association loss is not a stop or a pass.
        self.source_camera=source
        if self.target is None and cones:
            self.target=cones[0]['id'];self.target_aliases={self.target}
        target=next((c for c in cones if c['id']==self.target),None)
        if target and lane:
            self.last_target_fraction=target['fraction'];self.near_seen|=target.get('near',False)
            if self.side is None:self._lane_target(cones,nav)
            self._pass_observation(target,nav,-1 if self.side=='left' else 1)
        if self.target and (target is None or target.get('near_field_continuation')):
            self.review_pending=True
            if self.tail_powered>=self.options['tail_powered_limit_s']:
                if self.near_seen and max(self.pass_gate_hits,self.escape_exit_hits)>=3:
                    self.auto_count+=1;self._passed()
                else:
                    self.unverified_passes+=1;self._forget_target()
                cones=[c for c in cones if c['id'] not in self.passed_ids]
        elif target:self.review_pending=False;self.tail_powered=0.

        reason='持续低速循迹，等待人工停止'
        if lane:
            self.line_missing_since=None;desired=self._lane_target(cones,nav);ego=nav['ego_fraction']
            if self.target:reason='持续绕桶，结合前后目标选择可用侧'
        else:
            if self.line_missing_since is None:self.line_missing_since=now
            desired=.5;ego=.5
            visible=next((c for c in cones if c['id']==self.target),None) or (cones[0] if cones else None)
            if visible and not visible.get('near_field_continuation'):
                x=visible['image_fraction'];half=visible.get('half_image_width',.03)
                if self.side is None:self.side='left' if x>=.5 else 'right'
                if .25-half<x<.75+half:desired=.28 if self.side=='left' else .72
            reason='白线未确认，持续低速搜索和图像避让'
        if self.best_effort:reason='可用间隙未明确，保持低速向较宽一侧尝试避让'

        # Target filtering uses elapsed time between distinct observations;
        # repeated cached frames do not repeatedly amplify the command.
        captured=nav['captured_s']
        if captured!=self.guidance_at:
            observed_dt=.08 if self.guidance_at is None else max(0.,min(.25,captured-self.guidance_at))
            alpha=1-math.exp(-observed_dt/.18)
            self.filtered_target+=alpha*(desired-self.filtered_target);self.guidance_at=captured
        error=self.filtered_target-ego
        self.boundary_turn=None
        if lane:
            near=nav.get('ego_near_fraction',ego);soft=nav['half_envelope']+self.options['boundary_soft_fraction']
            if near<soft or nav.get('boundary_correction')=='right':
                error=max(error,.08+soft-near);self.boundary_turn='right'
            elif near>1-soft or nav.get('boundary_correction')=='left':
                error=min(error,-.08-(near-(1-soft)));self.boundary_turn='left'
            if self.boundary_turn:reason='白线预警，持续低速向'+('右' if self.boundary_turn=='right' else '左')+'纠偏'
        ratio=0. if abs(error)<.018 else min(1.,self.steering_gain*abs(error)/.12)
        endpoint=self.options['right_us'] if error>0 else self.options['left_us']
        requested=1610+ratio*(endpoint-1610);self.requested_steering=round(requested)
        change=1200*max(0.,min(.1,dt))
        self.actual_steering+=max(-change,min(change,requested-self.actual_steering))
        if self.handoff_after_two and self.count>=2:self.phase='complete'
        return {'action':'search','esc_us':self.s['driving']['forward_us'],'steering_us':round(self.actual_steering),
                'phase':self.phase,'reason':reason,'experimental':True,'manual_stop_only':not self.handoff_after_two,
                'boundary_correction':self.boundary_turn,'best_effort_route':self.best_effort,
                'line_search':not lane,'boundary_observed':lane and not nav.get('line_held')}
