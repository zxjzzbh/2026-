"""Turn-aware obstacle guidance; image evidence is not metric verification.

The steering state below is the last requested PWM, not wheel-angle feedback.
Lower PWM setpoints are conservative trial settings, not measured speeds.
"""
from .brake_parking import numeric
from .cone_low_speed import LowSpeedController,DEFAULT_STEERING_GAIN,steering_gain
from .cone_evidence import paired_lane, StablePairedLane
from .cone_response import combine_guidance,current_visual,visual_goal,blocking_replacement
from .cone_maneuver import TargetGap,ObservedLaneRecovery
from .cone_mission import ConeMissionLedger
from .cone_passage import PassageEvidence
from .cone_route import assess_route
from .steering import CENTER_US
from .lane_following import SteeringController,advance_steering
from .cone_guidance import CurrentTargetEvidence,GuidanceBudgets,strict_visual,TARGET_ONLY_MAX_S,NO_GUIDANCE_MAX_S
import copy


OBSTACLE_LINE_RECOVERY_S=NO_GUIDANCE_MAX_S
DEFAULT_STOP_FOR_CONES=False
TARGET_REACQUIRE_HOLD_S=.3


class ContinuousConeController(LowSpeedController):
    def __init__(self,settings,now,*,initialized=False,handoff_after_two=False,
                 stop_for_cones=DEFAULT_STOP_FOR_CONES):
        if type(stop_for_cones) is not bool:raise ValueError('锥桶接触停车开关必须为布尔值')
        super().__init__(settings,now,stop_on_complete=False,initialized=initialized)
        self.stop_for_cones=stop_for_cones
        self.steering_gain=steering_gain(self.options.get('steering_gain',DEFAULT_STEERING_GAIN))
        self.handoff_after_two=handoff_after_two
        self.actual_steering=1610.;self.requested_steering=1610
        self.filtered_target=.5;self.guidance_source=None;self.guidance_at=None
        self.unverified_passes=0;self.best_effort=False
        self.recentering=False;self.recenter_reason=None
        self.control=SteeringController(self.options['right_us'],self.options['left_us'])
        self.pd=self.control.pd
        self.last_pass_side=None;self.last_output=None
        self.lane_evidence=StablePairedLane(duration=.15,frames=3)
        self.return_evidence=StablePairedLane(duration=.3,frames=3)
        self.paired_stable=False;self.return_confirmed=False
        self.has_forward_command=False;self.last_observed_pair=None
        self.source_frames={};self.current_line_state='missing'
        self.target_space='none';self.target_handoff_reason=None;self.target_handoffs=0
        self.retired_unverified_ids=[];self.soft_boundary_override=None
        self.target_reacquire_until={}
        self.target_gap=TargetGap();self.lane_recovery=ObservedLaneRecovery()
        self.maneuver_phase='search';self.effective_steering_gain=1.
        self.target_identity_revision=0;self.unverified_recovery=False
        self.route_assessment=None;self.paired_recovery_continuation=False
        self.mission=ConeMissionLedger();self.passage=PassageEvidence(settings)
        self.pass_evidence={};self.last_pass_rejection=None
        self.boundary_output_withdrawal=False
        self.target_evidence=CurrentTargetEvidence(settings['image_size'])
        self.guidance_budgets=GuidanceBudgets()
        self.target_observation={};self.guidance_budget={};self.guidance_mode='waiting_for_lane'

    def _target_diagnostics(self):
        sources={'lane':'current_lane','image':'current_pixels','lane_follow':'fresh_lane_only',
                 'pending':'target_pending','none':'no_current_target'}
        return {'target_id':self.target,'target_space':self.target_space,
                'target_source':sources[self.target_space],'target_camera':self.source_camera,
                'target_handoff_reason':self.target_handoff_reason,'target_handoffs':self.target_handoffs,
                'retired_unverified_ids':list(self.retired_unverified_ids),
                'target_reacquire_blocked_ids':sorted(self.target_reacquire_until),
                'target_reacquire_hold_s':TARGET_REACQUIRE_HOLD_S,
                'soft_boundary_override':self.soft_boundary_override,
                'maneuver_phase':self.maneuver_phase,
                'effective_steering_gain':self.effective_steering_gain,
                'steering_gain_scope':'active_cone_avoidance',
                'target_identity_revision':self.target_identity_revision,
                'paired_recovery_continuation':self.paired_recovery_continuation,
                'route_assessment':self.route_assessment,
                'passage_evidence':copy.deepcopy(self.pass_evidence),
                'last_pass_rejection':self.last_pass_rejection,
                'boundary_output_withdrawal':self.boundary_output_withdrawal,
                'guidance_mode':self.guidance_mode,
                'guidance_budget':dict(self.guidance_budget),
                'target_observation':dict(self.target_observation),
                **self.mission.diagnostics(),
                **self.target_gap.diagnostics(self.last_step)}

    def progress(self):
        return {**super().progress(),'line_recovery_s':OBSTACLE_LINE_RECOVERY_S,
                'line_recovery_scope':'no_reliable_road_or_target',
                'target_only_limit_s':TARGET_ONLY_MAX_S,'target_only_limit_verified':False,
                'cone_contact_stop_enabled':self.stop_for_cones,
                **self._target_diagnostics(),
                'manual_stop_only':not self.handoff_after_two,'can_confirm_pass':False,
                'max_run_s':None,'unverified_passes':self.unverified_passes,
                'target_fraction':self.filtered_target,'requested_steering_us':self.requested_steering,
                'output_steering_us':round(self.actual_steering),'steering_gain':self.steering_gain,
                'best_effort_route':self.best_effort,'steering_recentering':self.recentering,
                'steering_center_us':CENTER_US,'recenter_reason':self.recenter_reason,
                'steering_policy':'shared_multi_row_pd','pd_error':self.pd.error,'pd_derivative':self.pd.derivative,
                'line_state':self.current_line_state,'paired_stable':self.paired_stable,
                'paired_frames':self.lane_evidence.count,'lane_return_confirmed':self.return_confirmed,
                'tail_clearance_verified':False,'pass_evidence_kind':'image_sequence_estimate'}

    @staticmethod
    def _line_state(nav):
        if paired_lane(nav):return 'paired'
        age=nav.get('prediction_age_s',nav.get('line_age_s'))
        if (nav.get('line_state')=='single' and not nav.get('line_missing')
                and numeric(age) and 0<=age<=.25 and numeric(nav.get('ego_fraction'))):
            return 'single'
        return 'missing'

    def _guard_navigation(self,nav,now):
        """Validate road observations separately from current target feedback."""
        if nav.get('boundary_danger') or nav.get('ready') is False or nav.get('action')=='stop':
            raise ValueError(nav.get('reason') or '避障导航要求停止')
        self.current_line_state=self._line_state(nav)
        self.paired_stable=self.lane_evidence.update(nav,now)
        if self.current_line_state=='paired':
            self.last_observed_pair=nav['captured_s']
        self.target_observation=self.target_evidence.update(nav,self.target,self.side,now)
        if self.has_forward_command:
            self.guidance_budget=self.guidance_budgets.update(nav,now,
                paired_stable=self.paired_stable,single_valid=self.current_line_state=='single',
                target_stable=self.target_observation.get('target_stable',False),
                target_current_valid=self.target_observation.get('current_valid',False))
            self.line_missing_since=self.guidance_budget.get('none_since_s')
            self.guidance_mode=self.guidance_budget['mode']
            if self.guidance_budget.get('expired'):
                if self.guidance_budget.get('reason')=='target_only_episode_deadline':
                    raise ValueError(f'仅目标引导超过 {TARGET_ONLY_MAX_S:g} 秒试验上限，停止并重新准备')
                raise ValueError(f'道路和锥桶均无可靠引导超过 {OBSTACLE_LINE_RECOVERY_S:g} 秒，停止并重新准备')
        return self.current_line_state!='paired'

    @staticmethod
    def _measurements(nav):
        near=nav.get('ego_near_fraction',nav.get('ego_fraction',.5))
        near=near if numeric(near) else .5
        far=nav.get('ego_far_fraction',near)
        far=far if numeric(far) else near
        heading=nav.get('heading_error',near-far)
        return near,far,heading if numeric(heading) else 0.

    @staticmethod
    def _small_distant_target(target):
        if not target or target.get('y_fraction',1.)>=.72:return False
        box=target.get('bbox_xyxy')
        if isinstance(box,(list,tuple)) and len(box)==4 and all(numeric(v) for v in box):
            return box[2]-box[0]<=28 and box[3]-box[1]<=36
        width=target.get('half_image_width')
        return numeric(width) and width<=.03

    def _neutral(self,phase,reason,fresh=True):
        self.phase=phase;self.actual_steering=float(CENTER_US);self.requested_steering=CENTER_US
        self.target_space='none';self.soft_boundary_override=None
        self.effective_steering_gain=1.;self.paired_recovery_continuation=False
        self.boundary_output_withdrawal=False
        self.pd.reset()
        self.last_output={'action':'neutral','esc_us':1500,'steering_us':CENTER_US,
            'phase':phase,'reason':reason,'experimental':True,'fresh_control':fresh,
            'line_state':self.current_line_state,'paired_stable':self.paired_stable,
            'boundary_observed':self.current_line_state=='paired',
            'cone_contact_stop_enabled':self.stop_for_cones,
            **self._target_diagnostics(),
            'manual_stop_only':not self.handoff_after_two,'tail_clearance_verified':False}
        return dict(self.last_output)

    def _begin_recenter(self,reason):
        # Retiring an avoidance target must retire its lateral command too.
        # Actual S3 output still returns through the existing slew limiter.
        self.side=None;self.target_fraction=.5;self.filtered_target=.5
        self.recentering=True;self.recenter_reason=reason
        self.return_confirmed=False;self.return_evidence.reset()
        self.maneuver_phase='return';self.unverified_recovery=False
        self.target_gap=TargetGap();self.lane_recovery=ObservedLaneRecovery()
        self.pd.reset()
        self.target_observation={}

    def _forget_target(self):
        self._retire_unverified()
        self.target=None;self.side=None;self.target_aliases=set();self.last_target_fraction=None
        self.near_seen=False;self.review_pending=False;self.tail_powered=0.
        self.pass_min_y=self.pass_last_y=self.pass_last_s=None
        self.pass_gate_hits=self.escape_exit_hits=self.approach_frames=0
        # A timeout is not proof that the car tail passed the cone.
        self.target_fraction=.5;self.filtered_target=.5;self.recentering=False
        self.unverified_recovery=True;self.maneuver_phase='unverified_recovery'
        self.return_confirmed=False;self.return_evidence.reset();self.pd.reset()
        self.target_gap=TargetGap();self.lane_recovery=ObservedLaneRecovery()
        self.passage.reset('unverified_target_retired');self.pass_evidence={}
        self.target_observation={}

    def _passed(self):
        if not self.mission.commit_estimate(self.source_camera,self.target,self.pass_evidence,self.last_step):
            self.last_pass_rejection='no_registered_current_secondary_visual_estimate'
            return False
        self.last_pass_side=self.side
        # No other camera/ID inherits this target's observed passage.
        self.target_aliases={self.target}
        super()._passed()
        self._begin_recenter('当前锥桶通过估计已完成，转向回中')
        self.passage.reset('estimated_pass_committed');self.pass_evidence={}
        return True

    def confirm_pass(self):
        raise ValueError('持续自主避障仅接受当前视觉通过证据，不能人工增加计数')

    def _adopt_target(self,target,reason):
        """New ownership of a target never inherits an old cone's pass proof."""
        if self.target is not None:
            self._retire_unverified();self.target_fraction=.5
        self.target=target['id'];self.target_aliases={self.target};self.side=None
        self.last_target_fraction=None;self.near_seen=False;self.review_pending=False;self.tail_powered=0.
        self.pass_min_y=self.pass_last_y=self.pass_last_s=None
        self.pass_gate_hits=self.escape_exit_hits=self.approach_frames=0
        self.recentering=False;self.recenter_reason=None
        self.return_confirmed=False;self.return_evidence.reset();self.pd.reset()
        self.target_handoff_reason=reason
        self.target_identity_revision=target.get('identity_revision',0)
        self.target_gap=TargetGap();self.lane_recovery=ObservedLaneRecovery()
        self.unverified_recovery=False;self.maneuver_phase='avoid'
        self.passage.reset('new_target');self.pass_evidence={};self.last_pass_rejection=None
        self.target_observation={}

    def _retire_unverified(self):
        if self.target and self.target not in self.passed_ids and self.target not in self.retired_unverified_ids:
            self.retired_unverified_ids.append(self.target)
            self.retired_unverified_ids=self.retired_unverified_ids[-8:]

    @staticmethod
    def _positions(cone,half):
        reserve=.03;space=half+cone['half_width']+reserve;lo,hi=half+reserve,1-half-reserve
        return [(side,x) for side,x in [('left',cone['fraction']-space),('right',cone['fraction']+space)] if lo<=x<=hi]

    def _lane_target(self,cones,nav):
        half=nav['half_envelope'];ego=self._measurements(nav)[0]
        target=next((c for c in cones if c['id']==self.target),None)
        # A disappeared/clipped target may retain independent pass evidence,
        # but its historical lane fraction is no longer a motion target.
        if target is None or target.get('near_field_continuation'):return .5
        choices=self._positions(target,half)
        if half+.03<=ego<=1-half-.03 and abs(ego-target['fraction'])>=half+target['half_width']+.03:
            choices.append(('left' if ego<target['fraction'] else 'right',ego))
        # Keep the current cone's side; select a new cone from the current
        # position instead of forcing an S-turn across the next obstacle.
        if self.side is not None:
            choices=[p for p in choices if p[0]==self.side]
        others=[c for c in cones if c['id']!=self.target and not c.get('near_field_continuation')]
        upcoming=others[0] if others else None
        def score(choice):
            side,x=choice
            change=.09 if self.side and side!=self.side else 0.
            next_choices=self._positions(upcoming,half) if upcoming else []
            if upcoming and abs(x-upcoming['fraction'])>=half+upcoming['half_width']+.03:
                next_choices.append(('stay',x))
            future=min((abs(y-x)+.2*abs(y-.5) for _,y in next_choices),default=abs(x-.5))
            previous=self.target_fraction if self.side is not None and numeric(self.target_fraction) else ego
            return abs(x-ego)+.2*abs(x-previous)+change+.3*future
        if choices:
            self.side,self.target_fraction=min(choices,key=score)
            self.route_assessment=assess_route(self.s,nav,target,self.side,self.target_fraction)
            return self.target_fraction
        if self.stop_for_cones:
            raise ValueError('锥桶与白线之间没有确认可行的绕行间隙，停止并重新准备')
        # Contact continuation is an explicit policy, not evidence of clearance.
        # Retain this cone/side and keep the target inside the observed lane;
        # neither a blocked gap nor this fallback constitutes a passed cone.
        lo,hi=half+.03,1-half-.03
        if lo>hi:raise ValueError('白线内没有可容纳当前车体包络的目标，停止并重新准备')
        previous=self.target_fraction if numeric(self.target_fraction) else ego
        self.target_fraction=max(lo,min(hi,previous));self.best_effort=True
        self.route_assessment=assess_route(self.s,nav,target,self.side,self.target_fraction)
        return self.target_fraction

    def _observe_identity(self,target,visual):
        entry=visual or target or {}
        revision=entry.get('identity_revision',self.target_identity_revision)
        if type(revision) is int and revision>=0 and revision!=self.target_identity_revision:
            # A recovered/revised target cannot inherit the old identity's
            # two-frame road continuation or derivative. Mission records and
            # passage evidence have their own independent provenance rules.
            self.lane_recovery=ObservedLaneRecovery();self.pd.reset()
        if type(revision) is int and revision>self.target_identity_revision:
            # PassageEvidence validates unique recovery provenance. It may
            # freeze a previously earned candidate, never an old exit timer.
            self.target_identity_revision=revision
            self.review_pending=False;self.tail_powered=0.

    def _observe_mission(self,target,nav,now):
        visual=strict_visual(nav,self.target,now,image_size=self.s['image_size']) if self.target else None
        credible=bool(target and self.current_line_state=='paired'
            and target.get('id')==self.target and not target.get('near_field_continuation')
            and not target.get('provisional') and target.get('confirmed') is not False and visual
            and any(item.get('id')==self.target and item.get('reason')=='current_observed_lane_position'
                    for item in nav.get('obstacle_diagnostics',[])))
        self.mission.observe(nav.get('source_camera'),self.target,nav.get('frame_id'),
            nav.get('captured_s'),now,credible=credible)

    def _pass_observation(self,target,nav,sign):
        """Current same-foot-row proof plus later current pixel exit only."""
        if nav.get('source_camera')!=self.mission.camera:
            self.pass_evidence={'target_id':self.target,'source_camera':nav.get('source_camera'),
                'frame_id':nav.get('frame_id'),'captured_s':nav.get('captured_s'),
                'gate_hits':0,'near_seen':False,'visual_pass_candidate':False,
                'visual_estimate_ready':False,'tail_verified':False,
                'phase':'auxiliary_guidance','reason':'primary_camera_does_not_count_task_cones'}
        else:
            self.pass_evidence=self.passage.observe(target,nav,self.side,self.last_step)
        self.pass_gate_hits=self.pass_evidence.get('gate_hits',0)
        self.near_seen=self.pass_evidence.get('near_seen',False)

    def _guidance(self,nav,cones,target,lane,now):
        near,far,heading=self._measurements(nav)
        visual=strict_visual(nav,self.target,now,image_size=self.s['image_size']) if self.target else None
        current_lane=bool(lane and target and not target.get('near_field_continuation'))
        self._observe_identity(target,visual)
        pending=self.target_gap.update(self.target,bool(current_lane or visual),nav,now,self.requested_steering)
        reason='新鲜双线循迹，按偏移与方向共同纠正'
        if lane:
            desired=self._lane_target(cones,nav);ego=near
            self.target_space='lane' if current_lane else 'lane_follow'
            if current_lane:reason='锁定绕行侧，跟随锥桶边缘目标'
            if self.best_effort:reason='未确认绕行间隙，按用户要求保持白线内目标继续；不视为通过'
        else:
            desired=ego=.5;heading=0.;self.target_space='none'
            reason='道路和目标引导暂失，在1秒恢复预算内降低油门并撤去旧舵'
        if visual and not current_lane:
            desired,ego,self.side=visual_goal(visual,self.side)
            self.target_space='image';reason='同一目标当前像素延续；不沿用历史车道坐标'
        elif pending:
            desired=ego;heading=0.;self.target_space='pending'
            reason='近场目标短暂待关联，限时撤去旧转向；尚未确认通过'
        if self.target_space in ('lane','image'):
            self.maneuver_phase='avoid'
        elif pending:
            self.maneuver_phase='target_pending'
        elif self.target:
            self.maneuver_phase='tail_confirm' if self.pass_gate_hits>=3 and self.near_seen else 'unverified_recovery'
        elif self.recentering:
            self.maneuver_phase='return'
        else:
            self.maneuver_phase='unverified_recovery' if self.unverified_recovery else 'search'
        return desired,ego,heading,current_lane,visual,reason

    def step(self,nav,now):
        if not numeric(now) or now<self.last_step:raise ValueError('控制时钟异常，结束本轮')
        dt=now-self.last_step
        if not nav or not numeric(nav.get('captured_s')) or not 0<=now-nav['captured_s']<=.25:
            raise ValueError('实时相机画面中断，结束本轮')
        if not nav.get('experimental'):raise ValueError('持续绕桶画面格式不匹配')
        source=nav.get('source_camera');captured=nav['captured_s'];fid=nav.get('frame_id',captured)
        if 'frame_id' in nav and type(fid) is not int:
            raise ValueError('相机帧号无效，结束本轮')
        old=self.source_frames.get(source)
        if old is not None and (captured<old[1] or fid<old[0]):
            raise ValueError('相机时间或帧号倒退，结束本轮')
        duplicate=old is not None and (fid==old[0] or captured==old[1])
        self._guard_navigation(nav,now)
        if duplicate:
            # Callers must not send this cached intent again or renew motion.
            return {**(self.last_output or self._neutral('waiting_for_lane','等待新鲜双线')),
                    'fresh_control':False,'reason':'等待本视角新帧；不重发缓存运动指令'}
        self.source_frames[source]=(fid,captured)
        lane=self.current_line_state!='missing' and bool(nav.get('lane_bounds_px')) and numeric(nav.get('ego_fraction'))
        reliable=self.current_line_state=='paired' and self.paired_stable
        mode=(source,self.current_line_state)
        if (self.last_output or {}).get('esc_us',1500)>1500:
            self.powered_s+=min(.25,dt)
            if self.review_pending and reliable:self.tail_powered+=min(.25,dt)
        self.last_step=now
        if now<self.arm_until:
            return self._neutral('arming','3 秒中位初始化')
        if not self.has_forward_command and not reliable:
            return self._neutral('waiting_for_lane','等待同一视角连续3个新鲜双线帧且持续0.15秒，尚未前进')
        self.phase='driving';self.best_effort=False
        if self.guidance_source!=mode:
            self.guidance_source=mode
            self.pd.reset();self.return_evidence.reset()
        cones=[c for c in nav.get('cones',[]) if c['id'] not in self.passed_ids
               and not c.get('provisional') and c.get('confirmed') is not False
               and lane and numeric(c.get('fraction'))]
        cones.sort(key=lambda c:c.get('y_fraction',0.),reverse=True)
        if self.source_camera and source!=self.source_camera and self.target:
            # Normalized fractions in two cameras are not physical identity.
            # Guidance may switch, but neither pass samples nor aliases cross.
            self._forget_target()
        self.source_camera=source
        # Only an actual handoff starts this short hold. Reappearance and
        # duplicate frames cannot extend it; a new third blocker stays eligible.
        self.target_reacquire_until={key:until for key,until in self.target_reacquire_until.items() if now+1e-9<until}
        eligible=[c for c in cones if c['id'] not in self.target_reacquire_until]
        fresh_targets=[c for c in eligible if not c.get('near_field_continuation')]
        target=next((c for c in cones if c['id']==self.target),None)
        if self.target and lane:
            goals=[self.target_fraction]
            if target is None:goals.append(.5)
            replacement=blocking_replacement(eligible,self.target,target,goals,nav['half_envelope'])
            if replacement:
                previous=self.target;self.target_handoffs+=1
                self.target_reacquire_until[previous]=now+TARGET_REACQUIRE_HOLD_S
                self._adopt_target(replacement,'当前确认目标占用旧目标通道：'+str(previous)+' -> '+str(replacement['id']))
        if self.target is None and fresh_targets:
            self._adopt_target(fresh_targets[0],'新鲜车道目标')
        target=next((c for c in cones if c['id']==self.target),None)
        self._observe_identity(target,current_visual(nav,self.target) if self.target else None)
        self._observe_mission(target,nav,now)
        if target and reliable and not target.get('near_field_continuation'):
            self.last_target_fraction=target['fraction']
            if self.side is None:self._lane_target(cones,nav)
        if self.target:
            # A marker identifies an existing lock; it supplies no invented
            # lane coordinates/gates when only current pixels remain.
            self._pass_observation(target or {'id':self.target},nav,-1 if self.side=='left' else 1)
        if self.target and self.pass_evidence.get('visual_estimate_ready') is True:
            if self._passed():
                self.auto_count+=1
            else:
                self.unverified_passes+=1;self._forget_target()
            cones=[c for c in cones if c['id'] not in self.passed_ids]
        elif self.target and (target is None or target.get('near_field_continuation')):
            self.review_pending=True
            # One second is only an observation/retirement bound. It cannot
            # manufacture a distance travelled or an estimated passed cone.
            timed_out=self.pass_evidence.get('phase')=='unverified'
            no_candidate=not self.pass_evidence.get('visual_pass_candidate')
            current_pixel=strict_visual(nav,self.target,now,image_size=self.s['image_size'])
            if current_pixel is None and (timed_out or (no_candidate and reliable and self.tail_powered>=self.options['tail_powered_limit_s'])):
                self.unverified_passes+=1;self._forget_target()
                cones=[c for c in cones if c['id'] not in self.passed_ids]
        elif target:self.review_pending=False;self.tail_powered=0.

        target=next((c for c in cones if c['id']==self.target),None)
        desired,ego,heading,current_lane,visual,reason=self._guidance(nav,cones,target,lane,now)
        near,far,_=self._measurements(nav)

        # Follow the edge target immediately. Filter the derivative, not the
        # target, so a newly visible cone does not incur the old 0.18 s lag.
        self.filtered_target=None if self.target_space=='pending' else desired
        avoiding=self.target_space in ('lane','image')
        error,heading_compensation=combine_guidance(desired-ego,heading,avoiding)
        self.boundary_turn=None;self.soft_boundary_override=None
        if lane:
            # Uncertainty still enlarges path clearance and limits movement;
            # it must not alone reverse the soft steering warning.
            single=self.current_line_state=='single'
            soft_half=nav.get('base_half_envelope',nav['half_envelope']) if single else nav['half_envelope']
            soft=soft_half+self.options['boundary_soft_fraction']
            correction=None if single else nav.get('boundary_correction')
            if single and nav.get('boundary_correction') and soft<=near<=1-soft:
                self.soft_boundary_override='single_uncertainty_only_warning_ignored; base_half_envelope_used'
            if near<soft or correction=='right':
                error=max(error,.08+soft-near);self.boundary_turn='right'
            elif near>1-soft or correction=='left':
                error=min(error,-.08-(near-(1-soft)));self.boundary_turn='left'
            if self.boundary_turn:reason='白线预警，降低油门向'+('右' if self.boundary_turn=='right' else '左')+'纠偏'
        self.effective_steering_gain=self.steering_gain if avoiding and not self.boundary_turn else 1.
        requested=self.control.request(error,captured,context=(mode,self.target,self.target_space),
            gain=self.effective_steering_gain,boundary_turn=self.boundary_turn)
        trusted_main=bool(current_lane and self.current_line_state=='paired' and visual
            and any(item.get('id')==self.target and item.get('reason')=='current_observed_lane_position'
                    for item in nav.get('obstacle_diagnostics',[])))
        small_target=self._small_distant_target(target) and not trusted_main
        if small_target and not self.boundary_turn:
            requested=max(CENTER_US+.45*(self.options['right_us']-CENTER_US),
                min(CENTER_US+.45*(self.options['left_us']-CENTER_US),requested))
        self.requested_steering=round(requested)
        if self.target_space=='pending' and not self.boundary_turn:
            requested=self.target_gap.request(now)
        # Honor elapsed time at 5 Hz as well as 20 Hz. The first output remains
        # bounded; stale camera input and the independent actuator lease stop.
        steering_phase='boundary_correction' if self.boundary_turn else 'avoiding' if avoiding else 'lane_follow' if lane else 'straight_search'
        mission_ready=self.mission.completion_ready()
        returning=(not self.target and (not self.unverified_recovery or mission_ready)
                   and (self.recentering or self.handoff_after_two and self.count>=2))
        if returning and not self.boundary_turn:
            steering_phase='return_to_lane';reason='桶通过仅为图像估计，连续核对横向位置和道路方向后回道'
        degraded=not reliable
        continuation,limit=self.lane_recovery.limit(nav,now,self.target,
            bool(trusted_main and not small_target),reliable,requested,self.boundary_turn)
        self.paired_recovery_continuation=bool(degraded and continuation)
        target_feedback=bool(self.target and self.target_space in ('lane','image')
            and self.target_observation.get('target_stable') and visual)
        if degraded:
            # Road confidence and target confidence are independent. Current,
            # stable pixels can sustain a known avoidance side without a
            # fabricated lane; only absent/untrusted targets use the small
            # search demand. Keep the existing PWM endpoints and slew limit.
            if target_feedback and not self.boundary_turn:
                limit=max(CENTER_US-self.options['right_us'],self.options['left_us']-CENTER_US)
                if self.current_line_state=='single':
                    uncertainty=nav.get('boundary_uncertainty_fraction',0.)
                    uncertainty=uncertainty if numeric(uncertainty) else .2
                    limit*=max(.5,min(1.,1.-2.*uncertainty))
                reason=('单侧白线辅助，当前锥桶画面持续引导绕行' if self.current_line_state=='single'
                    else '双线暂失，当前锥桶画面持续引导绕行；5秒试验上限')
            requested=max(CENTER_US-limit,min(CENTER_US+limit,requested))
            # Withdraw an old full-lock demand on the first degraded frame.
            # This bounds the requested command, not physical servo motion.
            if not continuation and not target_feedback:
                self.actual_steering=max(CENTER_US-80,min(CENTER_US+80,self.actual_steering))
            self.return_evidence.reset();self.return_confirmed=False
            steering_phase=('single_line_avoid' if self.current_line_state=='single' else 'target_only_avoid') if target_feedback else 'limited_recovery'
        self.requested_steering=round(requested)
        self.actual_steering,self.boundary_output_withdrawal=advance_steering(
            self.actual_steering,requested,dt,boundary_turn=self.boundary_turn,
            near=near if lane else None,half=soft_half if lane else None,
            warning=nav.get('boundary_margin_warning',False))
        if lane and self.stop_for_cones:
            self._guard_near_clearance(nav,requested)
            self._guard_near_clearance(nav,self.actual_steering)
        centered=(reliable and not self.target and not self.boundary_turn
                  and abs(near-.5)<=.05 and abs(heading)<=.05)
        if returning and centered:
            self.return_confirmed=self.return_evidence.update(nav,now)
        elif returning:
            self.return_evidence.reset();self.return_confirmed=False
        if returning and self.return_confirmed:
            self.recentering=False;self.recenter_reason=None
        self.guidance_at=captured
        if self.handoff_after_two and self.count>=2 and mission_ready and returning and self.return_confirmed:self.phase='complete'
        forward=self.s['driving']['forward_us'];turn=max(abs(requested-CENTER_US),abs(self.actual_steering-CENTER_US))
        cap=1570 if degraded or self.target_space=='pending' or turn>=156 or abs(heading)>=.15 else 1580 if turn>=65 or abs(heading)>=.07 else forward
        pulse=min(forward,cap)
        action='search' if pulse==forward else 'cone_'+str(pulse)
        if self.boundary_output_withdrawal:
            cap=pulse=1500;action='neutral';steering_phase='boundary_withdrawal'
            reason='边界迫近，撤去外向旧舵并暂缓前进'
        self.has_forward_command=True
        self.last_output={'action':action,'esc_us':pulse,'steering_us':round(self.actual_steering),
                'phase':self.phase,'reason':reason,'experimental':True,'manual_stop_only':not self.handoff_after_two,
                'boundary_correction':self.boundary_turn,'best_effort_route':self.best_effort,
                'steering_phase':steering_phase,'requested_steering_us':self.requested_steering,
                'steering_policy':'shared_multi_row_pd',
                'line_search':degraded,'boundary_observed':self.current_line_state=='paired',
                'line_state':self.current_line_state,'paired_stable':self.paired_stable,
                'target_feedback_steering':target_feedback,
                'heading_error':heading,'lateral_error':desired-ego,'small_target_limited':small_target,
                'fresh_control':True,'forward_limit_us':cap,'lane_return_confirmed':self.return_confirmed,
                'cone_contact_stop_enabled':self.stop_for_cones,
                **self._target_diagnostics(),
                'heading_compensation':heading_compensation,
                'tail_clearance_verified':False,'pass_evidence_kind':'image_sequence_estimate'}
        return dict(self.last_output)
