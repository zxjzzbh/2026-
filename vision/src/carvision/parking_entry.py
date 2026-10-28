"""Image-guided parking entry trial; pure computation, no hardware/network I/O.

Uses separately configurable image slowdown/stop rows for the far yellow edge.
Old camera rows remain a labelled, unverified transfer fallback. Neither an
image row nor lower PWM proves distance, speed or whole-vehicle containment.
The historical homography supplies an image centreline only.
"""
import copy
import math

import cv2
import numpy as np

from .brake_parking import BrakeParkingController, numeric
from .parking_visual import resolve_visual_options
from .parking_control import PARKING_STEERING_RATE_US_S
from .steering import CENTER_US, LEFT_US, RIGHT_US, ImagePD, from_offset

# Recovery is stronger than the old single-frame stopped check. Three new
# stationary observations must also span 200 ms after brake release.
BOUNDARY_RECOVERY_STOP_S = .2
BOUNDARY_RECOVERY_STOP_FRAMES = 3
BAY_SUPPORT_ACQUIRE_MARGIN = .025


class ParkingEntryController:
    """Create at operator start; call step once per fresh 480x360 secondary frame.

    ``sample`` contains fresh/frame_id/captured_s/image_size, ``parking`` from
    detect_parking, and ``motion`` from GroundMotionEstimate. Recommended or
    default sides may be selected. The trial may approach before its full
    boundary appears, but only an observed associated far line can trigger
    geometric parking. No metric calibration or free-space claim is invented.
    """
    def __init__(self, brake_settings, selected_side='right', *, forward_us=1575,
                 steering_gain=1., vehicle_width_m=.295, front_extent_m=.335,
                 rear_extent_m=.085, zone_length_m=1., half_width_m=.625,
                 ground=None, require_white_boundary=False, visual_options=None):
        if selected_side not in ('left', 'right'):
            raise ValueError('selected_side must be left or right')
        if type(forward_us) is not int or not 1501 <= forward_us <= 1625:
            raise ValueError('parking forward PWM must be 1501..1625 us')
        if not numeric(steering_gain) or not .25 <= steering_gain <= 3:
            raise ValueError('invalid parking steering gain')
        dimensions=(vehicle_width_m, front_extent_m, rear_extent_m, zone_length_m, half_width_m)
        if not all(numeric(v) and v > 0 for v in dimensions):
            raise ValueError('vehicle and bay dimensions must be positive')
        if vehicle_width_m + .04 >= half_width_m or front_extent_m + rear_extent_m + .04 >= zone_length_m:
            raise ValueError('vehicle does not fit the supplied bay dimensions')
        self.s=copy.deepcopy(brake_settings)
        if self.s.get('camera') != 'secondary' or self.s.get('image_size') != [480, 360] or self.s.get('required_esc_mode') != 'F/B':
            raise ValueError('requires existing secondary 480x360 F/B brake settings')
        if ([r.get('distance_m') for r in self.s.get('references', [])] != [.65, .5, .25]
                or not all(numeric(r.get('far_edge_y_normalized')) for r in self.s['references'])
                or not all(a['far_edge_y_normalized'] < b['far_edge_y_normalized'] for a,b in zip(self.s['references'], self.s['references'][1:]))):
            raise ValueError('preserve the existing 65/50/25 cm camera references')
        p=self.s.get('pwm', {})
        if (p.get('neutral_us') != 1500 or p.get('steering_center_us') != CENTER_US
                or type(p.get('brake_us')) is not int or not 1300 <= p['brake_us'] < 1500
                or not numeric(self.s.get('brake_pulse_s')) or not 0 < self.s['brake_pulse_s'] <= .3
                or type(self.s.get('max_brake_pulses')) is not int or not 1 <= self.s['max_brake_pulses'] <= 3):
            raise ValueError('invalid finite F/B brake settings')
        for name in ('brake_release_s','brake_timeout_s','stop_confirm_s','max_frame_age_s','no_progress_s'):
            if not numeric(self.s.get(name)) or self.s[name] <= 0:
                raise ValueError('invalid '+name)
        if self.s['max_frame_age_s'] > .25:
            raise ValueError('freshness must remain at most 250 ms')
        self.s.update(arming_s=3., hold_s=.5,
                      max_approach_s=30., speech_text='停车停稳，请确认车辆已进入停车区')
        self.visual=resolve_visual_options(visual_options,self.s,forward_us)
        self.s['pwm'].update(search_us=forward_us, creep_us=self.visual['slow_forward_us'])
        self.brake_reference={'distance_m':None,'far_edge_y_normalized':self.visual['stop_line_y_normalized'],
                              'measured':False,'source':self.visual['reference_source']}
        self.side, self.gain=selected_side, float(steering_gain)
        self.width_ratio=vehicle_width_m/half_width_m
        self.clearance_ratio=.02/half_width_m
        self.vehicle_length=front_extent_m+rear_extent_m
        self.track_width=half_width_m*2
        self.body_width=vehicle_width_m
        self.zone_length=zone_length_m
        self.center_h=None
        if ground is not None and ground.get('homography') is not None:
            h=np.asarray(ground['homography'], dtype=float)
            if (ground.get('image_size') != [480,360] or h.shape != (3,3)
                    or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-12 or abs(h[0,0]) < 1e-12):
                raise ValueError('invalid secondary image-centre reference')
            self.center_h=h
        self.phase='arming'
        self.reason='保持中位初始化 3 秒'
        self.started=self.last_now=self.last_capture=self.last_frame=None
        self.last_poly=self.last_poly_at=None
        self.last_far_edge=self.last_far_at=None
        self.last_poly_near_observed=False
        self.has_full_quad=False
        self.geometry_source=None
        self.near_boundary_observed=False
        self.core=None
        self.steering=float(CENTER_US)
        self.was_driving=False
        self.abort_until=None
        self.geometry={}
        self.cached_geometry=False
        self.geometry_age=None
        self.last_core_output={}
        self.seek_no_progress_since=None
        self.guidance_target_px=None
        self.retarget_count=0
        self.retarget_reason=None
        self.target_change=None
        self.aligned_frames=0
        self.countersteer_limited=False
        self.countersteer_active=False
        self.boundary={'observed':False,'correction':None}
        self.boundary_breach_frames=0
        # The live session always enables this. False is retained only for
        # isolated historical geometry replays without a road observation.
        self.require_white_boundary=bool(require_white_boundary)
        self.pd=ImagePD(kp=1.,kd=.12)
        self.guidance_phase='seek'
        self.seek_since=None
        self.last_white_at=self.last_white=None
        self.last_boundary_risk=None
        self.boundary_wait_since=self.boundary_brake_until=None
        self.boundary_paused_driving=False
        self.boundary_wait_cause=None
        self.recovery_stop_since=None;self.recovery_stop_frames=0
        self.bay_support_active=False
        self.bay_since=self.bay_at=None;self.bay_frames=0

    def _result(self, esc_us=1500, events=None):
        out={**self.last_core_output, 'schema_version':1, 'phase':self.phase,
             'action':'brake' if esc_us < 1500 else (
                 'creep' if esc_us==self.s['pwm']['creep_us'] else 'search') if esc_us > 1500 else 'neutral',
             'esc_us':esc_us, 'throttle_us':esc_us,
             'steering_us':CENTER_US if esc_us == 1500 else round(self.steering),
             'reason':self.reason, 'events':events or [], 'selected_side':self.side,
             'done':self.phase == 'done', 'active':self.phase not in ('done','fault'),
             'brake_trigger':self.brake_reference, 'field_verified':False,
             'actual_pose_not_verified':True, 'parking_zone_verified':False,
             'physical_stop_verified':False, 'tuning_verified':False,
             'whole_vehicle_inside_verified':False, 'metric_pose':None,
             'cached_geometry':self.cached_geometry, 'geometry_age_s':self.geometry_age,
             'geometry_source':self.geometry_source,
             'guidance_target_px':self.guidance_target_px,
             'guidance_phase':self.guidance_phase,
             'steering_policy':'bay_center_heading_pd',
             'steering_rate_us_s':PARKING_STEERING_RATE_US_S,
             'logic_revision':'parking_closed_loop_v3',
             'diagnostic_only':True,'formal_task':False,'payment_ready':False,
             'completion_authorizes_payment':False,
             'slow_forward_us':self.visual['slow_forward_us'],
             'slowdown_available':self.visual['slow_forward_us']<self.s['pwm']['search_us'],
             'slow_line_y_normalized':self.visual['slow_line_y_normalized'],
             'seek_timeout_s':self.visual['seek_timeout_s'],
             'max_entry_s':self.visual['max_run_s'],
             'boundary_wait_elapsed_s':None if self.boundary_wait_since is None else self.last_now-self.boundary_wait_since,
             'boundary_wait_cause':self.boundary_wait_cause,
             'recovery_stop_frames':self.recovery_stop_frames,
             'recovery_stop_confirm_s':BOUNDARY_RECOVERY_STOP_S,
             'near_boundary_observed':self.near_boundary_observed,
             'far_boundary_fresh':not self.cached_geometry and self.last_far_at == self.last_now,
             'target_acquired':self.last_far_edge is not None,
             'far_tracking_age_s':None if self.last_far_at is None else self.last_now-self.last_far_at,
             'retarget_count':self.retarget_count,'retarget_reason':self.retarget_reason,
             'target_change':self.target_change,'countersteer_limited':self.countersteer_limited,
             'countersteer_active':self.countersteer_active,
             'opposite_correction_limit_us':self.visual['countersteer_limit_us'],
             'white_boundary':copy.deepcopy(self.boundary),
             'geometry':copy.deepcopy(self.geometry),
             'centerline_source':'historical_ground_image_reference' if self.center_h is not None else 'image_midpoint',
             'reference_transfer_verified':False}
        # Interpolated crosswalk distances are not parking distances.
        out['distance_estimate_m']=None
        out['closing_speed_estimate_mps']=None
        return out

    def _abort(self, reason, now):
        self.reason=reason
        if self.was_driving and numeric(now):
            self.phase='safety_brake'
            self.abort_until=now+self.s['brake_pulse_s']
            self.was_driving=False
            return self._result(self.s['pwm']['brake_us'])
        self.phase='fault'
        return self._result()

    def _center_x(self, y):
        if self.center_h is None:
            return 240.
        x=float(-(self.center_h[0,1]*y+self.center_h[0,2])/self.center_h[0,0])
        return x if math.isfinite(x) and 0 <= x < 480 else None

    @staticmethod
    def _valid_quad(corners):
        try:
            q=np.asarray(corners, dtype=np.float32)
        except (ValueError,TypeError):
            return None
        if (q.shape != (4,2) or not np.isfinite(q).all() or (q < 0).any()
                or (q[:,0] >= 480).any() or (q[:,1] >= 360).any()
                or not cv2.isContourConvex(q) or cv2.contourArea(q) < 600):
            return None
        # Detector ordering: near-left, near-right, far-right, far-left.
        if (q[1,0]-q[0,0] < 20 or q[2,0]-q[3,0] < 15
                or q[0,1]-q[3,1] < 8 or q[1,1]-q[2,1] < 8
                or abs(q[2,1]-q[3,1]) > 54):
            return None
        return q

    def _associated(self, q, now, guide=False):
        """Match only to the latest *observed* far edge, never a predicted row."""
        if self.last_poly is None:
            return not guide
        dt=now-self.last_poly_at
        if dt > .4 and not guide:
            # A freshly observed full half-bay re-establishes geometry after
            # seeking; partial guides still need recent target association.
            return True
        if not 0 < dt <= .4:
            return False
        before=self.last_poly[2:]; after=q[2:]
        old_center=before.mean(axis=0); new_center=after.mean(axis=0)
        dy=float(new_center[1]-old_center[1])
        old_width=float(before[0,0]-before[1,0])
        new_width=float(after[0,0]-after[1,0])
        # Broad enough for low-speed perspective expansion and steering, but
        # reject jumping a lane width or relabelling the near entrance edge.
        center_limit=min(55., max(28.,240.*dt))
        endpoint_limit=min(65., max(35.,300.*dt))
        if (not -10. <= dy <= min(45., max(18.,240.*dt))
                or np.linalg.norm(new_center-old_center) > center_limit
                or np.max(np.linalg.norm(after-before,axis=1)) > endpoint_limit
                or not .72 <= new_width/old_width <= 1.38):
            return False
        if guide and self.last_poly_near_observed:
            old_near=float(self.last_poly[:2,1].mean())
            old_depth=old_near-float(old_center[1])
            if old_depth > 35 and abs(float(new_center[1])-old_near) < max(12.,.15*old_depth):
                return False
        return True

    def _polygon(self, parking, now):
        hint=parking.get('direction_hint') or {}
        slots=[s for s in parking.get('slots', []) if isinstance(s, dict) and s.get('id') == self.side]
        if any(s.get('availability') == 'blocked' for s in slots):
            return None, True, None, False
        complete=[]
        for slot in slots:
            q=self._valid_quad(slot.get('corners_px'))
            if q is not None and self._associated(q,now):
                complete.append(q)
        if len(complete) == 1:
            return complete[0], False, 'complete_quad', True
        # A real observed full quad can initialise a trial even when the stricter
        # two-bay pairing fails. A default side alone cannot label an arbitrary
        # shape: the blue-board hint must independently agree with that side.
        if not complete and hint.get('suggested_slot') == self.side and hint.get('blocked_side') != self.side:
            candidates=[]
            for c in parking.get('entry_candidates', []):
                if (not isinstance(c,dict) or c.get('id') != self.side
                        or c.get('candidate_only') is not True or c.get('availability') != 'clear'):
                    continue
                q=self._valid_quad(c.get('corners_px'))
                if q is not None and self._associated(q,now):
                    candidates.append(q)
            if len(candidates) == 1:
                return candidates[0],False,'visible_candidate',True
        # After the actual entrance edge leaves the image, the ground U shape
        # supplies current side edges plus the actual yellow far edge. Its near
        # section is an ROI cut, not a newly observed entrance or passed wheel.
        guides=[]
        if self.has_full_quad and self.last_poly_at is not None and now-self.last_poly_at <= .4:
            for g in parking.get('entry_guides', []):
                if (not isinstance(g,dict) or g.get('near_boundary_observed') is not False
                        or g.get('far_boundary_observed') is not True
                        or g.get('availability') == 'blocked' or g.get('id') not in (None,self.side)):
                    continue
                q=self._valid_quad(g.get('corners_px'))
                if q is not None and self._associated(q,now,guide=True):
                    guides.append(q)
        if len(guides) == 1:
            return guides[0],False,'tracked_far_guide',False
        return None,False,None,False

    def _retarget_if_blocked(self,parking,now):
        """Prefer the other half, without confusing screen side with bay ID.

        Once a real target is acquired, a coarse image-half hint alone cannot
        relabel it when the camera turns. Explicit bay occupancy can replan.
        Before acquisition a board cue may correct the initial choice once.
        """
        blocked={s.get('id') for s in parking.get('slots',[]) if isinstance(s,dict) and s.get('availability')=='blocked'}
        if {'left','right'}<=blocked:return '两侧入口均有遮挡，当前没有可进入半区'
        other='right' if self.side=='left' else 'left';hint=parking.get('direction_hint') or {}
        explicit=self.side in blocked
        overlap=False
        if self.last_poly is not None and now-self.last_poly_at<=.4:
            left,right=float(self.last_poly[0,0]),float(self.last_poly[1,0]);span=right-left
            near=float(self.last_poly[:2,1].mean())
            for board in hint.get('boards',[]):
                box=board.get('bbox_px') if isinstance(board,dict) else None
                if not isinstance(box,list) or len(box)!=4 or not all(numeric(v) for v in box):continue
                cx=(box[0]+box[2])/2
                if abs(box[3]-near)<29 and left+.2*span<cx<right-.2*span:
                    overlap=True;break
        coarse=(not self.has_full_quad and self.retarget_count==0 and hint.get('blocked_side')==self.side
                and hint.get('suggested_slot')==other and other not in blocked)
        if not explicit and not coarse and not overlap:return None
        if other in blocked:return '目标入口与另一侧均有遮挡，当前没有可进入半区'
        if self.last_far_edge is not None and float(self.last_far_edge[:,1].mean())/359.>=self.brake_reference['far_edge_y_normalized']:
            return '已到远端停车位置，先刹停，不能此时横跨停车半区'
        before=self.side;self.side=other;self.retarget_count+=1
        self.retarget_reason=('已见原目标入口遮挡，改向另一半区' if explicit or overlap else '按入口挡板修正初始选侧')
        self.target_change={'from':before,'to':other,'reason':self.retarget_reason,'captured_s':self.last_capture}
        # Keep throttle continuity, steering slew and the original start time.
        # Never reuse the old half's far edge as the new half's parking line.
        self.last_poly=self.last_poly_at=self.last_far_edge=self.last_far_at=None
        self.last_poly_near_observed=False;self.has_full_quad=False
        self.geometry_source=None;self.near_boundary_observed=False
        self.geometry={};self.geometry_age=None;self.cached_geometry=False
        self.core=None;self.last_core_output={};self.guidance_target_px=None
        self.seek_no_progress_since=None;self.aligned_frames=0;self.phase='seek'
        self.bay_since=self.bay_at=None;self.bay_frames=0
        self.pd.reset()
        return None

    def _steer(self,desired,dt,aligned=False):
        """Fresh same-bay evidence enables bounded correction before alignment."""
        correction=self.visual['countersteer_limit_us'] if self._bay_stable() else 0
        lower=CENTER_US-correction if self.side=='left' else RIGHT_US
        upper=LEFT_US if self.side=='left' else CENTER_US+correction
        limited=max(lower,min(upper,desired));self.countersteer_limited=limited!=desired
        correction=self.boundary.get('correction')
        if correction:
            # White-line recovery takes priority over the selected-side limit.
            # Reuse the cone controller's bounded inward steering response.
            near=self.boundary['near_fraction'];soft=self.boundary['soft_fraction']
            error=(.08+soft-near) if correction=='right' else (-.08-(near-(1-soft)))
            limited=from_offset(max(-1.,min(1.,error/.12)))
            self.countersteer_limited=False
        self.countersteer_active=(limited<CENTER_US if self.side=='left' else limited>CENTER_US)
        delta=(1200. if correction else PARKING_STEERING_RATE_US_S)*dt
        self.steering=max(float(RIGHT_US),min(float(LEFT_US),self.steering+max(-delta,min(delta,limited-self.steering))))

    def _white_guard(self,parking,now):
        road=parking.get('road_context') or {}
        previous_breach=self.boundary_breach_frames;self.boundary_breach_frames=0
        self.boundary={'observed':False,'correction':None,'metric_boundary_verified':False,
                       'reason':'双侧白线未连续确认'}
        if road.get('white_lines_detected') is not True:
            self.boundary_breach_frames=0;return False
        edges=[]
        for key in ('left_inner','right_inner'):
            try:points=np.asarray(road.get(key,[]),dtype=float)
            except (ValueError,TypeError):
                self.boundary['reason']='白线坐标无效';return False
            if (points.ndim!=2 or points.shape[1]!=2 or len(points)<6 or not np.isfinite(points).all()
                    or np.ptp(points[:,1])<32 or (points<0).any()
                    or (points[:,0]>=480).any() or (points[:,1]>=360).any()):
                self.boundary['reason']='白线有效点或纵向覆盖不足';return False
            fit=np.polyfit(points[:,1],points[:,0],1)
            if np.max(np.abs(np.polyval(fit,points[:,1])-points[:,0]))>8:
                self.boundary['reason']='白线点不构成稳定边界';return False
            edges.append((fit,float(points[:,1].min()),float(points[:,1].max())))
        lo=max(v[1] for v in edges);y=min(v[2] for v in edges)
        if y-lo<32:
            self.boundary['reason']='左右白线共同可见范围不足';return False
        left,right=[float(np.polyval(v[0],y)) for v in edges];width=right-left;center=self._center_x(y)
        if center is None or not 72<width<864:
            self.boundary['reason']='白线宽度或车身图像参考无效';return False
        near=(center-left)/width
        soft=(self.body_width/2+.04+self.vehicle_length*.1)/self.track_width+.03
        hard=(self.body_width/2+.02)/self.track_width
        correction='right' if near<soft else 'left' if near>1-soft else None
        self.boundary_breach_frames=previous_breach+1 if near<hard or near>1-hard else 0
        self.boundary={'observed':True,'correction':correction,'near_fraction':near,'soft_fraction':soft,
            'hard_fraction':hard,'row_px':y,'left_px':left,'right_px':right,
            'breach_frames':self.boundary_breach_frames,'metric_boundary_verified':False,
            'reason':'图像边界有效；近场车身包络仍未实测验证'}
        self.last_white_at=now;self.last_white=copy.deepcopy(self.boundary)
        if self.boundary_breach_frames:self.last_boundary_risk=now
        return self.boundary_breach_frames>=3

    def _bay_evidence(self,now):
        g=self.geometry
        usable=(self.last_far_at==now and self.geometry_source in
                ('complete_quad','visible_candidate','tracked_far_guide')
                and g.get('availability')=='clear'
                and numeric(g.get('lateral_error')) and numeric(g.get('heading_error')))
        if not usable:
            self.bay_since=self.bay_at=None;self.bay_frames=0;return
        if self.bay_at is None or not 0<now-self.bay_at<=.25:
            self.bay_since=now;self.bay_frames=0
        self.bay_at=now;self.bay_frames+=1

    def _bay_stable(self):
        return self.bay_frames>=3 and self.bay_at-self.bay_since>=.12

    def _visible_edge_risk(self,parking):
        """A currently seen dangerous edge vetoes use of the other evidence."""
        road=parking.get('road_context') or {}
        for key in ('left_inner','right_inner'):
            try:points=np.asarray(road.get(key,[]),dtype=float)
            except (ValueError,TypeError):continue
            if (points.ndim!=2 or points.shape[1]!=2 or len(points)<6 or not np.isfinite(points).all()
                    or np.ptp(points[:,1])<32 or (points<0).any()
                    or (points[:,0]>=480).any() or (points[:,1]>=360).any()):continue
            if self.last_white is not None:
                y=float(points[:,1].max())
                width=self.last_white['right_px']-self.last_white['left_px']
                margin=width*self.last_white['hard_fraction']
            else:
                checks=[check for check in self.geometry.get('envelope_checks',[])
                        if float(points[:,1].min())<=check['row_px']<=float(points[:,1].max())]
                if not checks:continue
                check=max(checks,key=lambda c:c['row_px']);y=check['row_px'];margin=check['envelope_half_width_px']
            center=self._center_x(y)
            if center is None:continue
            fit=np.polyfit(points[:,1],points[:,0],1);edge=float(np.polyval(fit,y))
            if np.max(np.abs(np.polyval(fit,points[:,1])-points[:,0]))>8:continue
            if key=='left_inner' and center-edge<margin:return True
            if key=='right_inner' and edge-center<margin:return True
        return False

    def _boundary_gate(self,parking,sample,now):
        """Fuse fresh bay geometry; otherwise pause without cancelling a blink."""
        # Heading belongs to steering and the final stop gate. It cannot make
        # a currently contained, observed bay disappear during alignment.
        observed_bay=(self.last_far_at==now and self.geometry.get('availability')=='clear'
                      and numeric(self.geometry.get('heading_error')))
        inside=self.geometry.get('lateral_envelope_estimate_inside') is True
        margins=[]
        for check in self.geometry.get('envelope_checks',[]):
            span=check['right_px']-check['left_px'];center=check['center_px']
            if center is not None and span>0:
                half=check['envelope_half_width_px']
                margins.append(min(center-check['left_px']-half,check['right_px']-center-half)/span)
        clearance=min(margins) if len(margins)==2 else None
        margin_ok=numeric(clearance) and (clearance>0 if self.bay_support_active else clearance>=BAY_SUPPORT_ACQUIRE_MARGIN)
        fresh_bay=observed_bay and inside and margin_ok
        risk=self._visible_edge_risk(parking)
        recent_risk=self.last_boundary_risk is not None and now-self.last_boundary_risk<.5
        supported=(not self.require_white_boundary or self.boundary['observed'])
        mode='white_lines' if self.boundary['observed'] else 'historical_replay'
        if not supported and not risk and not recent_risk:
            if fresh_bay and self._bay_stable():supported=True;mode='current_bay_envelope'
            elif (fresh_bay and self.last_white_at is not None
                    and now-self.last_white_at<=self.visual['boundary_grace_s']):
                supported=True;mode='recent_white_and_current_bay'
        evidence_usable=bool(supported)
        cause=('bay_envelope_outside' if observed_bay and not inside else
               'bay_margin_pending' if observed_bay and not margin_ok else 'boundary_evidence_missing')
        self.boundary.update(evidence_usable=evidence_usable,bay_clearance_fraction=clearance,
                             support_source=mode if supported else 'waiting')
        if supported and self.boundary_wait_since is not None and self.boundary_paused_driving:
            motion=sample.get('motion') or {}
            stamp=motion.get('captured_s',sample.get('captured_s'))
            feedback_ok=(motion.get('valid') is True and type(motion.get('moving')) is bool
                         and numeric(stamp) and 0<=now-stamp<=self.s['max_frame_age_s'])
            released=now>=(self.boundary_brake_until if self.boundary_brake_until is not None else now)+self.s['brake_release_s']
            stopped=feedback_ok and motion['moving'] is False
            if released and stopped:
                if self.recovery_stop_since is None:self.recovery_stop_since=now
                self.recovery_stop_frames+=1
            else:
                self.recovery_stop_since=None;self.recovery_stop_frames=0
            confirmed=(self.recovery_stop_since is not None
                       and self.recovery_stop_frames>=BOUNDARY_RECOVERY_STOP_FRAMES
                       and now-self.recovery_stop_since>=BOUNDARY_RECOVERY_STOP_S-1e-9)
            if not confirmed:
                supported=False
                cause=('stop_feedback_invalid' if not feedback_ok else
                       'still_moving' if not stopped else 'stop_confirmation_pending')
        elif not supported:
            self.recovery_stop_since=None;self.recovery_stop_frames=0
        self.boundary['usable']=bool(supported)
        if supported:
            self.boundary_wait_since=self.boundary_brake_until=None;self.boundary_paused_driving=False
            self.boundary_wait_cause=None;self.recovery_stop_since=None;self.recovery_stop_frames=0
            self.bay_support_active=mode in ('current_bay_envelope','recent_white_and_current_bay')
            if mode not in ('white_lines','historical_replay'):
                self.boundary['reason']='由连续新鲜车位边界补充白线；低输出修正，未作米制验证'
            return None
        if not fresh_bay:self.bay_support_active=False
        self.boundary_wait_cause=cause
        if self.boundary_wait_since is None:
            self.boundary_wait_since=now;self.boundary_paused_driving=self.was_driving
            self.boundary_brake_until=now+(self.s['brake_pulse_s'] if self.was_driving else 0.)
            self.core=None;self.last_core_output={};self.pd.reset()
        if now-self.boundary_wait_since>=self.visual['boundary_wait_s']:
            reason={'stop_feedback_invalid':'停车边界已恢复，但缺少有效停稳反馈，结束入位',
                    'still_moving':'停车边界已恢复，但仍检测到移动，结束入位',
                    'stop_confirmation_pending':'停车边界已恢复，但连续停稳确认不足，结束入位',
                    'bay_margin_pending':'当前车位包络余量不足以首次确认边界，结束入位',
                    'bay_envelope_outside':'当前车位车体包络已越过边界，结束入位',
                    'boundary_evidence_missing':'缺少当前有效车位和白线边界，结束入位'}[cause]
            return self._abort(reason,now)
        self.phase='boundary_wait';self.guidance_phase='boundary_wait'
        self.reason='停车边界暂不明确，暂停等待新鲜边界和停稳反馈'
        if now<self.boundary_brake_until:return self._result(self.s['pwm']['brake_us'])
        self.steering=float(CENTER_US);self.was_driving=False
        return self._result()

    def _geometry(self, q):
        far=float(np.mean(q[2:,1])); near=float(np.mean(q[:2,1]))
        def bounds(y):
            left=float(q[3,0]+(y-q[3,1])*(q[0,0]-q[3,0])/(q[0,1]-q[3,1]))
            right=float(q[2,0]+(y-q[2,1])*(q[1,0]-q[2,0])/(q[1,1]-q[2,1]))
            return left,right
        # Far/mid and near image envelopes prevent turning toward a bay whose
        # centreline crosses the divider. These remain image-space checks.
        rows=[far+(near-far)*.2, far+(near-far)*.8]
        checks=[]
        for y in rows:
            left,right=bounds(y); center=self._center_x(y); span=right-left
            margin=span*(self.width_ratio/2+self.clearance_ratio)
            checks.append({'row_px':y, 'left_px':left, 'right_px':right,
                           'center_px':center, 'envelope_half_width_px':margin,
                           'inside':center is not None and left+margin <= center <= right-margin})
        target_y=far+(near-far)*.4
        left,right=bounds(target_y); center=self._center_x(target_y)
        lateral=None if center is None else ((left+right)/2-center)/max(10.,(right-left)/2)
        offsets=[None if c['center_px'] is None else
                 ((c['left_px']+c['right_px'])/2-c['center_px'])/max(10.,(c['right_px']-c['left_px'])/2)
                 for c in checks]
        heading=None if None in offsets else offsets[0]-offsets[1]
        error=None if lateral is None or heading is None else lateral+.45*heading
        aligned=(error is not None and all(c['inside'] for c in checks)
                 and abs(lateral)<.12 and abs(heading)<.12)
        return {'far_edge_y_normalized':far/359., 'near_edge_y_normalized':near/359.,
                'lateral_envelope_estimate_inside':all(c['inside'] for c in checks),
                'envelope_checks':checks, 'steering_error':error,
                'lateral_error':lateral,'heading_error':heading,'image_aligned':aligned,
                'actual_pose_not_verified':True}

    def _seek(self, sample, parking, now, dt):
        """Keep a low forward command without inventing a missing far edge.

        A current board supplies a continuously updated target in its opposite
        half. Without a visible target, unwind to centre under a bounded
        search deadline. Current same-bay geometry can authorize correction.
        """
        if self.last_far_at is not None and now-self.last_far_at>.6:
            return self._abort('已锁定停车目标后远端线跟踪丢失，刹停防止越过车位',now)
        if self.seek_since is None:self.seek_since=now
        if now-self.seek_since>=self.visual['seek_timeout_s']:
            return self._abort('寻位超时，仍无可靠停车目标，结束入位',now)
        self.phase='seek'
        self.guidance_phase='seek'
        self.core=None  # No stale crosswalk seen_at/last_t timer during seeking.
        self.last_core_output={}
        self.geometry_age=None if self.last_far_at is None else now-self.last_far_at
        self.cached_geometry=self.last_far_edge is not None and self.geometry_age <= .4
        hint=parking.get('direction_hint') or {}
        self.guidance_target_px=None
        if self.cached_geometry:
            desired=self.steering
            source='cached_heading'
            self.pd.reset()
        else:
            # With no current target evidence, unwind gently to straight;
            # do not keep an old quarter-turn indefinitely.
            bias=0.
            if hint.get('suggested_slot') == self.side:
                boards=[]
                for board in hint.get('boards',[]):
                    box=board.get('bbox_px') if isinstance(board,dict) else None
                    if (isinstance(board,dict) and board.get('side') == ('left' if self.side == 'right' else 'right')
                            and isinstance(box,list) and len(box)==4 and all(numeric(v) for v in box)
                            and 0<=box[0]<box[2]<=480 and 0<=box[1]<box[3]<=360):
                        boards.append(box)
                if boards:
                    x1,y1,x2,y2=max(boards,key=lambda b:(b[2]-b[0])*(b[3]-b[1]))
                    target=min(479.,x2+(x2-x1)/2) if self.side == 'right' else max(0.,x1-(x2-x1)/2)
                    center=self._center_x(y2)
                    if center is not None:
                        bias=max(-1.,min(1.,(target-center)/max(20.,(x2-x1)/2)))
                        self.guidance_target_px=[target,y2]
            source='blue_board_seek' if self.guidance_target_px is not None else 'selected_side_seek'
            desired=from_offset(self.pd.update(bias,sample['captured_s'],
                context=(self.side,source),gain=self.gain))
        self._steer(desired,dt)
        self.geometry_source=source
        self.near_boundary_observed=False
        self.geometry={'far_edge_y_normalized':None,'near_edge_y_normalized':None,
                       'lateral_envelope_estimate_inside':None,'envelope_checks':[],
                       'actual_pose_not_verified':True,'near_boundary_observed':False,
                       'far_boundary_fresh':False,'geometry_source':source}
        motion=sample.get('motion') or {}
        if motion.get('valid') is True and motion.get('moving') is False:
            if self.seek_no_progress_since is None:self.seek_no_progress_since=now
            if now-self.seek_no_progress_since >= self.s['no_progress_s']:
                return self._abort('持续前进输出但未观察到移动；检查电调与起步输出',now)
        else:
            self.seek_no_progress_since=None
        self.reason='向已选'+('左' if self.side == 'left' else '右')+'侧低速接近；边界未完整，继续寻找实际远端黄线'
        if source=='cached_heading':self.reason+='；短时保持上一帧方向'
        elif source=='blue_board_seek':self.reason+='；持续跟随挡板旁的空位目标修正转向'
        else:self.reason+='；目标暂失，逐步减小转向低速寻找'
        if self.boundary.get('correction'):self.reason+='；白线优先向内纠偏'
        self.was_driving=True
        return self._result(self.s['pwm']['creep_us'])

    def _tracked_geometry(self,parking,now):
        """Associate measured optical-flow far points; do not invent a bay box."""
        if self.last_far_edge is None or not 0<now-self.last_far_at<=.4:return None
        candidates=[];previous=self.last_far_edge;dt=now-self.last_far_at
        for line in parking.get('tracked_far_lines',[]):
            if not isinstance(line,dict):continue
            if (line.get('id')!=self.side or line.get('source')!='optical_flow_far_edge'
                    or line.get('frame_id')!=self.last_frame or line.get('captured_s')!=self.last_capture
                    or type(line.get('tracked_points')) is not int or line['tracked_points']<6
                    or not numeric(line.get('marking_support')) or line['marking_support']<.65):continue
            try:edge=np.asarray(line.get('edge_px'),dtype=np.float32)
            except (TypeError,ValueError):continue
            if (edge.shape!=(2,2) or not np.isfinite(edge).all() or (edge<0).any()
                    or (edge[:,0]>=480).any() or (edge[:,1]>=358).any()
                    or edge[1,0]-edge[0,0]<20 or abs(float(edge[1,1]-edge[0,1]))>54):continue
            old_width=float(previous[1,0]-previous[0,0]);width=float(edge[1,0]-edge[0,0])
            dy=float(edge[:,1].mean()-previous[:,1].mean())
            if (not .72<=width/old_width<=1.38 or not -10<=dy<=min(45.,max(18.,240.*dt))
                    or np.max(np.linalg.norm(edge-previous,axis=1))>min(65.,max(35.,300.*dt))):continue
            candidates.append(edge)
        if len(candidates)!=1:return None
        edge=candidates[0];y=float(edge[:,1].mean());center=self._center_x(y)
        if center is None:return None
        self.last_far_edge,self.last_far_at=edge.copy(),now
        self.geometry_source='optical_flow_far_edge';self.near_boundary_observed=False;self.geometry_age=0.
        error=(float(edge[:,0].mean())-center)/max(10.,float(edge[1,0]-edge[0,0])/2)
        if abs(error)<.10:error=0.
        return {'far_edge_y_normalized':y/359.,'near_edge_y_normalized':None,
                'steering_error':error,'lateral_error':error,'heading_error':None,'image_aligned':False,
                'lateral_envelope_estimate_inside':None,'envelope_checks':[],
                'actual_pose_not_verified':True}

    def step(self, sample, now):
        self.target_change=None
        if self.phase in ('done','fault'):
            return self._result()
        if self.phase == 'safety_brake':
            if numeric(now) and self.last_now is not None and 0 < now-self.last_now <= .25 and now < self.abort_until:
                self.last_now=now
                return self._result(self.s['pwm']['brake_us'])
            self.phase='fault'
            return self._result()
        if not numeric(now) or self.last_now is not None and not 0 < now-self.last_now <= .25:
            # After a missed control deadline the output lease, not a new
            # unbounded pulse, must return to neutral.
            self.phase='fault'; self.reason='停车控制间隔失效'
            return self._result()
        dt=.1 if self.last_now is None else now-self.last_now
        self.last_now=now
        if self.started is None:
            self.started=now
        frame_id, captured=sample.get('frame_id'), sample.get('captured_s')
        if (sample.get('fresh') is not True or sample.get('image_size') != [480,360]
                or type(frame_id) is not int or not numeric(captured)
                or not 0 <= now-captured <= self.s['max_frame_age_s']
                or self.last_frame is not None and frame_id <= self.last_frame
                or self.last_capture is not None and captured <= self.last_capture):
            return self._abort('停车画面过期、重复或分辨率改变', now)
        self.last_frame,self.last_capture=frame_id,captured
        if now-self.started < 3.:
            return self._result()
        if self.phase == 'arming':
            self.phase='seek'
        parking=sample.get('parking') or {}
        braking=self.phase in ('brake_pulse','settle','hold')
        if not braking:
            if now-self.started-3.>=self.visual['max_run_s']:
                return self._abort('停车入位总时限已到，结束本轮',now)
            if self._white_guard(parking,now):return self._abort('连续白线观测显示车身边缘风险，刹停防止继续越线',now)
            obstruction=self._retarget_if_blocked(parking,now)
            if obstruction:return self._abort(obstruction,now)
            q, blocked, source, near_observed=self._polygon(parking,now)
            if blocked:return self._abort('重新选区后入口仍被遮挡，停止进入',now)
            self.cached_geometry=False
            if q is not None:
                self.guidance_target_px=None
                self.last_poly,self.last_poly_at=q.copy(),now
                self.last_poly_near_observed=near_observed
                self.has_full_quad=self.has_full_quad or near_observed
                self.geometry_source=source
                self.near_boundary_observed=near_observed
                self.geometry_age=0.
                self.last_far_edge,self.last_far_at=q[[3,2]].copy(),now
                self.geometry=self._geometry(q)
                collection=parking.get('slots',[]) if source=='complete_quad' else parking.get('entry_guides',[])
                clear=(source=='visible_candidate' or any(
                    isinstance(item,dict) and item.get('id') in (self.side,None)
                    and item.get('availability')=='clear'
                    and (observed:=self._valid_quad(item.get('corners_px'))) is not None
                    and np.allclose(observed,q) for item in collection))
                self.geometry['availability']='clear' if clear else 'unknown'
            else:
                tracked=self._tracked_geometry(parking,now)
                if tracked is None:
                    self.bay_since=self.bay_at=None;self.bay_frames=0
                    pause=self._boundary_gate(parking,sample,now)
                    if pause is not None:return pause
                    return self._seek(sample,parking,now,dt)
                self.geometry=tracked
            self.seek_since=None
            self.seek_no_progress_since=None
            self.geometry.update(geometry_source=self.geometry_source,
                                 near_boundary_observed=self.near_boundary_observed,
                                 far_boundary_fresh=not self.cached_geometry)
            y=self.geometry['far_edge_y_normalized']
            if self.geometry['steering_error'] is None:
                return self._abort('历史中心参考不在当前画面内',now)
            self._bay_evidence(now)
            pause=self._boundary_gate(parking,sample,now)
            if pause is not None:return pause
            if y >= self.brake_reference['far_edge_y_normalized'] and (
                    self.geometry['lateral_envelope_estimate_inside'] is False or
                    (numeric(self.geometry.get('heading_error')) and abs(self.geometry['heading_error'])>.15)):
                return self._abort('已到远端停车参考但车身横向尚未对齐，停止而不越过远边',now)
            self.guidance_phase='enter' if self.geometry.get('image_aligned',False) else (
                'align' if self.geometry['lateral_envelope_estimate_inside'] else 'turn_in')
            desired=from_offset(self.pd.update(self.geometry['steering_error'],captured,
                context=(self.side,self.geometry_source),gain=self.gain))
            self._steer(desired,dt,self.geometry.get('image_aligned',False))
            if self.core is None:
                self.core=BrakeParkingController(self.s, path_mode='straight',
                    image_brake_y=self.visual['stop_line_y_normalized'],
                    image_creep_y=self.visual['slow_line_y_normalized'])
        core_sample={**sample, 'candidate':True,
                     'far_edge_y_normalized':self.geometry.get('far_edge_y_normalized')}
        result=self.core.step(core_sample,now)
        if result['phase'] == 'fault':
            return self._abort(result['reason'],now)
        self.last_core_output=result
        self.phase=result['phase']
        if self.phase in ('search','creep'):
            self.phase='approach'
        esc=result['esc_us']
        if esc>1500 and (self.guidance_phase in ('align','enter')
                or self.countersteer_active
                or self.boundary.get('support_source') in ('current_bay_envelope','recent_white_and_current_bay')):
            esc=min(esc,self.visual['slow_forward_us'])
        self.was_driving=esc > 1500
        if esc > 1500:
            side='左' if self.side=='left' else '右'
            self.reason={'turn_in':'跟随车位中心目标，低速转入'+side+'半区',
                         'align':'结合远近中心偏差回正，对齐'+side+'车位',
                         'enter':'车位图像已对齐，低速接近实际远端黄线'}[self.guidance_phase]
            if self.cached_geometry:
                self.reason+='；短时沿用边界 '+str(round(self.geometry_age,2))+' 秒'
            elif self.geometry_source == 'tracked_far_guide':
                self.reason+='；入口近线已出画，跟随实时侧边与远端黄线'
            elif self.geometry_source == 'optical_flow_far_edge':
                self.reason+='；完整框缺失，持续跟踪已锁定远端黄线'
            if self.boundary.get('correction'):self.reason+='；白线优先向内纠偏'
        elif self.phase == 'brake_pulse':
            self.guidance_phase='brake'
            self.reason='实际远端黄线到达停车图像停止参考，发出有限 F/B 制动'
        elif self.phase == 'done':
            self.guidance_phase='stopped'
            self.reason='画面连续停稳候选成立；实际四轮入位尚未验证'
        else:
            self.reason=result['reason']
        return self._result(esc,result.get('events'))
