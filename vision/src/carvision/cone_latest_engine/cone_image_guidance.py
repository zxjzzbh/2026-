"""Continuous image cone guidance with feasibility-first route selection. Author: 13246.

Cone goals remain camera/lane fractions; a separate calibrated right-stripe
estimate limits outward steering. Neither supplies a cone-passage count.
"""
import math

from .brake_parking import numeric
from .cone_image_geometry import image_lane_geometry
from .cone_ground_flow import valid_motion
from .cone_lane_margin import LaneMarginGuard, center_margin
from .cone_timing import frame_fresh
from .cone_steering_policy import RoadHeadingControl, ConeHandoff, ApproachPreview, approach_command, passing_command
from .cone_side_choice import side_spaces, ConeRouteChoice
from .cone_right_boundary import RightBoundaryRecovery
from .cone_maneuver_state import ConeAliases, ConeReturnGuard


def clamp(value,low=-1.,high=1.):return max(low,min(high,value))


class ImageConeGuidance:
    missing_hold_s=.35
    reassociate_s=.8

    def __init__(self,settings):
        self.s=settings;self.gain=settings['low_speed_test'].get('steering_gain',1.)
        self.target_id=self.side=None;self.phase='search';self.replans=0
        self.last_seen=None;self.last_camera=self.last_domain=None;self.last_target=None
        self.last_captures={};self.last_avoid=0.
        self.corridor_memory=None;self.motion_capture=None;self.observed_speed=None
        self.no_direction_since=None;self.recenter_aligned_hits=0;self.recenter_capture=None
        self.lane_margin=LaneMarginGuard(settings)
        self.right_boundary=RightBoundaryRecovery(settings)
        self.last_heading=None;self.last_heading_s=None
        self.identity_transition='none'
        self.heading_control=RoadHeadingControl()
        self.approach_preview=ApproachPreview()
        self.near_clearance_until=None
        self.handoff=ConeHandoff();self.maneuver_started=False
        self.side_choice=None
        self.route_choice=ConeRouteChoice(settings)
        self.applied_command=0.
        self.aliases=ConeAliases();self.return_guard=ConeReturnGuard()

    def _advance_corridor(self,nav,now):
        """Retire a steering corridor from new flow; never increment pass count."""
        motion=(nav.get('local_course') or {}).get('motion') or {}
        if valid_motion(motion,now) and (self.motion_capture is None or motion['captured_s']>self.motion_capture):
            fraction=1. if self.motion_capture is None else min(1.,(motion['captured_s']-self.motion_capture)/motion['dt_s'])
            self.motion_capture=motion['captured_s'];self.observed_speed=max(0.,motion['speed_mps'])
            if self.corridor_memory:
                self.corridor_memory['remaining_m']=max(0.,self.corridor_memory['remaining_m']-max(0.,motion['delta_y_m'])*fraction)
                self.corridor_memory['motion_samples']+=1

    def _corridor_alive(self,now):
        m=self.corridor_memory
        if m is None:return False
        if m['motion_samples']>=3 and m['remaining_m']<=0:return False
        return max(0.,now-m['captured_s'])<m['estimated_limit_s']

    @staticmethod
    def _lane_contract(nav):
        return (not nav.get('line_missing') and nav.get('guard')!='image_only_boundary_unknown'
                and bool(nav.get('lane_bounds_px')) and numeric(nav.get('ego_fraction')))

    @staticmethod
    def _weight(geometry):
        if not geometry:return 0.
        if geometry.get('heading_only'):return .85
        if geometry.get('single_edge_observed'):return .6*clamp(1.-geometry.get('line_age_s',1.),0.,1.)
        if not geometry.get('line_held'):return 1.
        return clamp((.35-geometry.get('line_age_s',.8))/.20,0.,1.)

    def _candidates(self,nav,lane):
        result=[]
        for raw in nav.get('cones',[]):
            x=raw.get('fraction' if lane else 'image_fraction')
            radius=raw.get('half_width' if lane else 'half_image_width',.03)
            if not numeric(x) or not numeric(radius) or radius<0:continue
            result.append({**raw,'x':float(x),'radius':float(radius)})
        return result

    def _select(self,rows,camera,domain,captured,now,is_new,view_handover=None):
        confirmed=[c for c in rows if not c.get('provisional')]
        age=math.inf if self.last_seen is None else max(0.,now-self.last_seen)
        old=self.last_target
        self.identity_transition='retained'
        candidate=next((c for c in confirmed if c.get('id')==self.target_id),None)
        continuity=candidate is not None and camera==self.last_camera
        hint=view_handover or {}
        if candidate is None:
            candidate,identity=self.aliases.match(confirmed,camera,captured,hint)
            if candidate is not None:
                continuity=True;self.identity_transition=identity
        # During a camera handover retain the manoeuvre side, not the old
        # camera's pixel coordinates. This is bounded continuity, not pass proof.
        if candidate is None and confirmed and old and camera!=self.last_camera and age<=self.reassociate_s:
            near=[c for c in confirmed if c.get('image_close') or c.get('exit_band') or c.get('y_fraction',0)>=.90]
            if len(near)==1:
                candidate=near[0];continuity=True;self.identity_transition='view_handover_side_retained'
        if candidate is None and confirmed:
            # A bottom-edge disappearance alone cannot declare the rear of
            # the vehicle clear. Keep the current side through the short gap.
            if old is None or age>self.reassociate_s:candidate=confirmed[0]
        if candidate is not None:
            if not continuity:
                if self.maneuver_started:self.handoff.begin(now)
                self.maneuver_started=True
                self.side=None;self.phase='avoiding'
                self.side_choice=None
                self.route_choice.reset()
                self.aliases.reset();self.return_guard.reset()
                self.corridor_memory=None;self.identity_transition='new_target'
            elif camera!=self.last_camera or candidate['id']!=self.target_id:
                # An obsolete target's tail may never be resurrected after a
                # newly observed target has taken over.
                self.corridor_memory=None
            self.target_id=candidate['id']
            if is_new:
                self.last_seen=max(captured,self.last_seen if self.last_seen is not None else captured)
                self.last_camera=camera;self.last_domain=domain
                self.last_target=dict(candidate)
                self.aliases.remember(camera,candidate,captured)
            return candidate,False
        # A reidentified provisional box can support the prior manoeuvre only
        # briefly; it never starts a target or renews its confirmation lifetime.
        if self.last_target and age<=self.missing_hold_s:
            if camera==self.last_camera and domain==self.last_domain:
                old=self.last_target
                nearby=[c for c in rows if numeric(c.get('image_fraction')) and numeric(old.get('image_fraction'))
                        and abs(c['image_fraction']-old['image_fraction'])<=.14
                        and abs(c.get('y_fraction',0)-old.get('y_fraction',0))<=.16]
                candidate=min(nearby,key=lambda c:abs(c['image_fraction']-old['image_fraction'])) if nearby else dict(old)
                return {**candidate,'id':self.target_id},True
            return None,True  # Keep the side, never reuse another coordinate domain.
        if age>self.reassociate_s and not self._corridor_alive(now):
            self.target_id=None;self.side=None;self.last_target=None
        return None,False

    def prime(self,nav,now):
        """Observe during existing neutral arming; never retain motion progress."""
        result=self.update(nav,now)
        self.applied_command=0.;self.route_choice.committed=False
        self.maneuver_started=False;self.handoff=ConeHandoff()
        self.corridor_memory=None;self.near_clearance_until=None
        self.return_guard.reset();self.approach_preview=ApproachPreview()
        self.no_direction_since=None;self.recenter_aligned_hits=0;self.recenter_capture=None
        return {**result,'preparing_only':True,'stop_required':False}

    def update(self,nav,now):
        space_nav=nav  # Keep raw same-camera fractions separate from ground conversions.
        camera=nav.get('source_camera');captured=nav.get('captured_s')
        fresh=frame_fresh(captured,now)
        is_new=bool(fresh and captured>self.last_captures.get(camera,-math.inf))
        if is_new:self.last_captures[camera]=captured
        self._advance_corridor(nav,now)
        geometry=image_lane_geometry(nav,self.s,now=now) if fresh else None
        self.right_boundary.observe(nav if fresh else {},now)
        weight=self._weight(geometry)
        lane=self._lane_contract(nav)
        held_age=nav.get('line_age_s')
        if nav.get('line_held') and (not numeric(held_age) or held_age>=.35):lane=False
        # Reject the old mixed contract even if a producer retained lane fields.
        if lane and any(not c.get('provisional') and not numeric(c.get('fraction')) for c in nav.get('cones',[])):lane=False
        mapped=False
        # If one camera sees the cone and the other sees the lane, translate
        # only through their existing common rear-axle ground references.
        # Never subtract one camera's pixel fraction from the other's lane.
        if not lane and geometry and not geometry.get('single_edge_observed') and numeric(geometry.get('body_ego_fraction')):
            width=geometry.get('ground_lane_width_estimate_m');slope=geometry.get('ground_lane_slope')
            observed={(p.get('camera'),p.get('alias')):p for p in (nav.get('local_course') or {}).get('cones',[])
                      if frame_fresh(p.get('captured_s'),now)}
            confirmed=[c for c in nav.get('cones',[]) if not c.get('provisional')]
            if confirmed and numeric(width) and width>0 and numeric(slope) and all((camera,c['id']) in observed for c in confirmed):
                converted=[]
                for c in confirmed:
                    p=observed[(camera,c['id'])]
                    if not numeric(p.get('x')) or not numeric(p.get('y')):break
                    q=geometry['body_ego_fraction']+(p['x']-slope*p['y'])/width
                    converted.append({**c,'fraction':q,'half_width':self.s['geometry']['cone_base_width_m']/self.s['geometry']['lane_width_m']/2})
                if len(converted)==len(confirmed):nav={**nav,'cones':converted};lane=True;mapped=True
        domain='lane_fraction' if lane else 'image_fraction'
        compatible=bool(lane and geometry and not geometry.get('single_edge_observed')
                        and (geometry.get('nav_goal_compatible') or mapped))
        position=geometry.get('body_ego_fraction') if geometry else None
        if not numeric(position):position=geometry['near_ego_fraction'] if geometry else None
        ego=(position if compatible else nav.get('ego_near_fraction',nav.get('ego_fraction',.5))) if lane else .5
        if not numeric(ego):ego=.5
        half=(self.s['geometry']['vehicle_width_m']/2+.035)/self.s['geometry']['lane_width_m'] if lane else .15
        if not numeric(half) or not 0<half<.5:half=.15
        rows=self._candidates(nav,lane) if fresh else []
        pending=next((c['id'] for c in rows if not c.get('provisional') and c['id']!=self.target_id),None)
        target,held=self._select(rows,camera,domain,captured,now,is_new,nav.get('target_view_handover'))
        recovering=self.handoff.active
        self.handoff.update(geometry)
        if recovering and not self.handoff.active and not self.route_choice.committed:
            # The route selector keeps uncommitted preferences reviewable; the
            # handoff timer must not erase exclusions or force a fresh lock.
            self.identity_transition='handoff_route_reselected'
        if target is not None and not held and is_new and target.get('exit_band'):
            self.near_clearance_until=captured+self.reassociate_s
        near_hold=bool(target is None and self.side is not None and self.near_clearance_until is not None
                       and now<self.near_clearance_until)
        retained=target is None and not held and (self._corridor_alive(now) or near_hold)
        tail=retained and self.return_guard.near_side_seen
        if retained and self.corridor_memory:
            self.side=self.corridor_memory['side'];self.target_id=self.corridor_memory['target_id']
        edge=max(half+.03,center_margin(self.s)) if lane else half+.03
        lo,hi=edge,1-edge;goal=ego;route_risk=False;route_wait=False;lateral_clear=False
        heading=geometry['heading_correction_fraction'] if geometry else 0.
        geometry_near=geometry['near_ego_fraction'] if geometry else None
        boundary=None
        if geometry and not geometry.get('heading_only'):
            body_half=self.s['geometry']['vehicle_width_m']/self.s['geometry']['lane_width_m']/2
            edge=body_half+.06
            if geometry_near<edge:boundary='right'
            elif geometry_near>1-edge:boundary='left'
        if target is not None:
            relevant=[c for c in rows if not c.get('provisional') and
                      (c.get('id')==target['id'] or c.get('near') or c.get('y_fraction',0)>=target.get('y_fraction',1)-.08)]
            if not any(c.get('id')==target['id'] for c in relevant):relevant.insert(0,target)
            free=[(lo,hi)] if lo<=hi else []
            for c in relevant:
                a,b=c['x']-half-c['radius']-.06,c['x']+half+c['radius']+.06;remaining=[]
                for left,right in free:
                    if b<left or a>right:remaining.append((left,right));continue
                    if left<a:remaining.append((left,min(a,right)))
                    if right>b:remaining.append((max(b,left),right))
                free=remaining
            choices=[]
            for left,right in free:
                side='left' if right<target['x'] else 'right'
                nearest=clamp(ego,left,right)
                choices.append((abs(nearest-ego)+.08*abs(nearest-.5),side,left,right,nearest))
            space_target=next((c for c in space_nav.get('cones',[]) if c['id']==target['id']),target)
            spaces=side_spaces(space_nav,space_target,self.s,now) if not held else None
            choice,self.side_choice=self.route_choice.choose(choices,spaces,camera=camera,captured=captured,
                target=target,ego=ego,held=held,applied=self.applied_command,
                prefer_left=self.right_boundary.prefer_left)
            route_wait=choice is None
            if choice is not None:
                _,side,left,right,goal=choice
            else:
                # Continue bounded road-heading feedback while checking the
                # route. Missing intervals are never fabricated at a lane edge.
                side=None;left=right=goal=ego;route_risk=True
            if self.side is not None and self.side!=side:self.replans+=1
            if side!=self.side:self.corridor_memory=None
            self.side=side
            lateral_clear=not route_wait and left<=ego<=right
            if not lateral_clear and not route_risk:
                # Move only as far as needed to clear the cone, plus a small
                # interior reserve; the corridor midpoint can hug the white line.
                inset=min(.04,(right-left)*.25)
                goal=clamp(ego,left+inset,right-inset)
            if route_wait:
                self.phase='route_observation'
            elif lateral_clear:
                self.phase='passing'
                goal=ego  # Side clearance permits unwinding, never body-pass proof.
            else:
                self.phase='hold_target' if held else 'avoiding'
        elif tail:
            self.phase='clearance_tail'
            # Retain the chosen passing side, never replay an old lateral goal
            # after the vehicle has aligned. Current heading is the only tail cue.
            goal=ego
        elif held or retained:
            self.phase='hold_side'
        else:
            self.phase='recenter' if geometry and weight>0 else 'search'
            goal=.5

        clearance_required=bool(target is not None and self.phase in ('avoiding','hold_target'))
        passing=self.phase in ('passing','hold_side','clearance_tail')
        self.return_guard.observe(target,camera,domain,captured,now,self.side,
            fresh=bool(target is not None and not held and is_new),clear=lateral_clear)
        road_command=self.heading_control.update(geometry)*weight
        if geometry and weight>0:
            self.no_direction_since=None
            if geometry.get('fresh_lane') or geometry.get('single_edge_observed'):
                observed=geometry.get('captured_s',captured)
                if self.last_heading_s is None or observed>self.last_heading_s:
                    self.last_heading=road_command;self.last_heading_s=observed
        elif self.no_direction_since is None:
            self.no_direction_since=now
        direction_age=0. if self.no_direction_since is None else max(0.,now-self.no_direction_since)

        # Approach uses one side-consistent lateral request in either coordinate
        # domain. The old secondary-camera y=.60 gate halved it too early.
        if clearance_required:
            remaining,settling=self.approach_preview.update((camera,domain,self.target_id),captured,goal-ego,
                enabled=bool(compatible and geometry and geometry.get('fresh_lane') and not held))
            lateral=self.gain*(6.*remaining if compatible else remaining/.12)
            self.last_avoid=clamp(lateral,-.75,.75)
            command=approach_command(self.last_avoid,road_command,settling=settling)
            self.recenter_aligned_hits=0;self.recenter_capture=None
        elif passing:
            command=passing_command(road_command,self.side)
            self.recenter_aligned_hits=0;self.recenter_capture=None
        elif route_wait:
            command=road_command
            self.recenter_aligned_hits=0;self.recenter_capture=None
        elif geometry and weight>0:
            capture=(geometry['camera'],geometry['lane_observed_s'])
            if capture!=self.recenter_capture:
                self.recenter_capture=capture
                self.recenter_aligned_hits=(self.recenter_aligned_hits+1
                    if geometry.get('fresh_lane') and abs(heading)<.025 else 0)
            self.phase='align';command=road_command
            if self.recenter_aligned_hits>=3 and not geometry.get('heading_only'):
                self.phase='recenter'
                command=clamp(2*(.5-geometry_near)*weight+road_command,-.50,.50)
        else:
            self.recenter_aligned_hits=0;self.recenter_capture=None
            self.phase='search';command=0.

        if not geometry or weight<=0:
            if not clearance_required and self.last_heading_s is not None:
                age=max(0.,now-self.last_heading_s)
                command=passing_command(self.last_heading,self.side)*clamp(1-age/.6,0.,1.)
        self.handoff.advance(now,target,camera,fresh_target=bool(target is not None and not held and fresh))
        if self.handoff.active and geometry and weight>0:
            recovery=self.handoff.recovery(geometry,road_command,weight)
            # Keep observing the next target during recovery. An inward cone
            # avoidance request may help; an outward one may not restart yaw.
            if not clearance_required or command*recovery<0 or abs(recovery)<.05:
                command=recovery
            else:
                command=clamp(command,-.65,.65)
            self.phase='transition_align'
        elif self.handoff.active:
            age=math.inf if self.last_heading_s is None else max(0.,now-self.last_heading_s)
            command=passing_command(self.last_heading or 0.,self.side)*clamp(1-age/.6,0.,1.)
            self.phase='transition_align'
        # This is a short maneuver-continuity guard, not a claim of measured
        # body passage. No pixels, elapsed time or output PWM increment a count.
        direction_limit=5. if (clearance_required or route_wait and target is not None) and not self.handoff.active else 1.
        needs_stop=direction_age>=direction_limit
        if route_wait and target is not None:
            # Do not turn into a side rejected by current clearance evidence.
            blocked=self.side_choice.get('blocked_sides',[])
            available=self.side_choice.get('available_sides',[])
            if command>0 and ('right' in blocked or available==['left']):command=0.
            if command<0 and ('left' in blocked or available==['right']):command=0.
        command,margin_report=self.lane_margin.constrain(command,
            {**geometry,'feedback_weight':weight,'heading_command':road_command} if geometry else None,now,
            clearance_side=self.side if clearance_required or passing else None)
        if clearance_required or passing:
            command=self.return_guard.constrain(command,self.side,margin_report,
                boundary_override=self.side=='right' and self.right_boundary.report.get('active',False))
        left_allowed=(target is not None and not held and not route_wait and self.side=='left'
                      or target is None and not held and not passing)
        command=self.right_boundary.constrain(command,left_allowed=left_allowed,
            left_risk=margin_report.get('left_risk',0.))
        conflict=bool(clearance_required and command*(-1 if self.side=='left' else 1)<0)
        if conflict:
            self.phase='boundary_recovery';command=clamp(command,-.65,.65)
        elif passing:
            # Boundary proximity may require an inward correction, but never
            # unlock a full endpoint turn when a near cone leaves the picture.
            command=clamp(command,-.65,.65)
        else:
            limit=.75 if clearance_required and self.phase!='transition_align' else .65
            command=clamp(command,-limit,limit)
        power_scale=.75 if abs(command)>.5 else 1.
        if passing or self.phase in ('boundary_recovery','transition_align','route_observation') or not geometry or weight<=0:
            power_scale=min(power_scale,.65)
        if geometry and (geometry.get('line_held') or geometry.get('heading_only')):
            power_scale=min(power_scale,.65)
        if self.right_boundary.report.get('active'):power_scale=min(power_scale,.65)
        if needs_stop:self.phase='guidance_lost';command=0.
        if target is not None and self.side is not None and not held and is_new and compatible and geometry and geometry.get('fresh_lane'):
            distance=target.get('reference_distance_m')
            distance=distance if numeric(distance) and 0<=distance<=2.5 else .7
            budget=distance+self.s['geometry']['vehicle_length_m']+.09
            speed=self.observed_speed if self.observed_speed is not None and self.observed_speed>.1 else .25
            self.corridor_memory={'camera':geometry['camera'],'goal':float(goal),'side':self.side,
                'target_id':self.target_id,'captured_s':captured,'remaining_m':budget,'motion_samples':0,
                'estimated_limit_s':clamp(budget/max(.15,speed*.8),.6,6.)}
        elif not retained and not held and target is None:self.corridor_memory=None
        reason={'avoiding':'沿已选侧连续绕桶','hold_target':'目标短时缺测，保留同侧避让',
                'route_observation':'确认当前可通行侧，保持道路方向；不沿失效通道继续转向',
                'hold_side':'目标短时缺测，保留同桶绕行侧并限制反向回正','align':'用统一转向控制对齐道路',
                'passing':'侧向已让开，保持通过方向，尚未确认整车通过',
                'clearance_tail':'近桶出画后保持可通过走廊并回正，不立即切回桶前',
                'recenter':'按近行位置与道路方向平缓回中','search':'道路方向暂缺，限时寻找白线',
                'transition_align':'持续观察下一桶，先收住外偏方向并恢复绕行空间',
                'boundary_recovery':'接近白线，低速收转向并对齐道路，保留当前锥桶目标',
                'guidance_lost':'缺少可用道路方向，已结束恢复；需重新定位后再开始'}[self.phase]
        return {'stop_required':needs_stop,'direction_missing_s':direction_age,'lane_margin':margin_report,
                'right_boundary':dict(self.right_boundary.report),
                'return_guard':dict(self.return_guard.report),
                'target_aliases':[list(a) for a in sorted(self.aliases.ids)],
                'power_scale':power_scale,'control_conflict':conflict,'identity_transition':self.identity_transition,
                'command':clamp(command),'target_id':self.target_id,'side':self.side,'target_fraction':float(goal),
                'coordinate_domain':domain,'phase':self.phase,'boundary_correction':boundary,
                'handoff_recovery':self.handoff.active,'handoff_reason':self.handoff.reason,
                'approach_preview':self.approach_preview.report if clearance_required else None,'pending_target_id':pending,
                'side_choice':self.side_choice,
                'cone_coordinates_from_shared_ground':mapped,
                'route_risk':route_risk,'replans':self.replans,
                'corridor_memory':dict(self.corridor_memory) if self.corridor_memory else None,
                'geometry':{**geometry,'feedback_weight':weight} if geometry else None,'reason':reason}
