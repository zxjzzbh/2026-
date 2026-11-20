"""Autonomous race orchestration. Pure perception/policy; no hardware access."""
from dataclasses import asdict, replace
from pathlib import Path
import sys
import time
import cv2

import config as C
from barrier import BlueBarrierGate
from cone_solo_logic import Avoider, Params
from cone_relaxed_logic import RelaxedAvoider
from fusion_logic import FusionController, MissionPolicy, Command
from traffic_light import TrafficLightDetector, LightResult

sys.path.append(str(Path(__file__).resolve().parent/'parking_modules'))
from parking_side_exit_policy import SideExitParkingPolicy
from parking_side_exit_perception import SideExitPerception


class RaceBarrierGate:
    """Original temporal gate, observing upper image to exclude blue floor."""
    def __init__(self, roi):
        self.roi=roi
        self.gate=BlueBarrierGate()

    def update(self, frame):
        h,w=frame.shape[:2]
        x0,y0,x1,y1=self.roi
        return self.gate.update(frame[int(y0*h):int(y1*h),int(x0*w):int(x1*w)])


class RaceAvoider(Avoider):
    """Confirm real passes; give uncertain cone recovery to ordinary lane following."""
    def _finish_avoid(self, boxes, mask, dt):
        completed=self.target_id if self.pass_confirmed else None
        out=super()._finish_avoid(boxes,mask,dt)
        out.completed_target_id=completed
        return out

    def halt(self, reason, boxes=None, mask=None):
        if self.fault:
            return super().halt(self.fault, boxes, mask)
        if (not self.p.protection_stop and reason in
                ('TARGET_UNCERTAIN', 'PASS_UNCONFIRMED', 'NO_AVOID_PROGRESS')):
            # 不确定的桶不算通过；退出绕桶反打，交回普通巡线及其丢线重获逻辑。
            self.pass_confirmed = False
            result = self._finish_avoid(boxes or [], mask, 0.)
            result.handoff_turn_us = None
            result.reason = reason + '_LANE_FOLLOW'
            result.lane_follow_fallback = True
            return result
        return super().halt(reason, boxes, mask)


def parking_policy(cfg):
    return SideExitParkingPolicy(dict(
        stop_duty=C.STOP_DUTY,low_duty=cfg['parking_duty'],
        brake_duty=C.PARK_MANUAL_BRAKE_DUTY,brake_seconds=C.CROSS_FR_BRAKE_TIME,
        steer_center_us=C.STEER_CENTER_US,steer_sign=C.STEER_SIGN,
        steer_min_us=C.STEER_MIN_US,steer_max_us=C.STEER_MAX_US,
        max_turn_us=cfg['parking_turn_us'],kp_us=cfg['parking_kp_us'],
        pass_seconds=cfg['parking_pass_seconds']))


class RacePolicy(MissionPolicy):
    RED_STOP_WAIT_SECONDS=5.5
    CONE_PARK_TIMEOUT_SECONDS=7.5
    STOPPED=MissionPolicy.STOPPED+('LIGHT_BRAKE','LIGHT_WAIT','PARK_SETTLE','PARK_WAIT')

    def __init__(self, settings, cfg):
        super().__init__(settings)
        self.cfg=cfg
        crosswalk_duty=cfg.get('crosswalk_drive_duty',settings.cruise_duty)
        self.crosswalk_settings=replace(settings,cruise_duty=crosswalk_duty,
                                       approach_duty=cfg.get('crosswalk_drive_duty',settings.approach_duty))
        self.parking_search_settings = replace(settings, cruise_duty=cfg['parking_duty'])
        self.cone_settings = replace(settings, cruise_duty=cfg.get('cone_cruise_duty', settings.cruise_duty))
        self.order=cfg['stage_order']
        self.stage=self.order[0]
        self.light=LightResult()
        self.signal=None
        self.light_last_seen=None
        self.light_fault=''
        self.traffic_mode=cfg.get('traffic_mode','green')
        if self.traffic_mode not in ('green','red-timer'):
            raise ValueError('Invalid traffic mode')
        self.traffic_red_seen=False
        self.traffic_wait_end=None
        self.traffic_wait_applied=False
        self.completed_cones=set()
        self.first_cone_completed_at=None
        self.attempted_cones=set()
        self.parking_observation={}
        self.parking=parking_policy(cfg)
        self.settle_end=None
        self.transition_reason='START_CROSSWALK'
        self.unconfirmed_cones_at_parking=0
        self.parking_alignment_hits=0
        self.clock=None  # Runtime injects monotonic; offline tests use supplied time.

    def advance(self, now, reason):
        self.stage=self.order[self.order.index(self.stage)+1]
        self.transition_reason=reason
        self.state,self.owner='REACQUIRE','none'
        self.lane_hits=0
        self.reacquire_since=now
        self.cone_since=None
        self.reason=''
        return self._command(C.STOP_DUTY,reason)

    def _light_brake(self, now, reason, fault=False):
        self.state,self.owner='LIGHT_BRAKE','none'
        self.brake_end=now+C.CROSS_FR_BRAKE_TIME
        self.light_fault=reason if fault else ''
        return self._command(C.CROSS_FR_BRAKE_DUTY,reason)

    def update(self, now, lane, cone, cross):
        if self.state=='HALT': return self.halt(self.reason)
        if self.stage=='QR_CAPTURE':
            return Command('DONE','none',C.STEER_CENTER_US,C.STOP_DUTY,'PARKED_QR_READY')
        if self.stage=='CROSSWALK':
            # A lost confirmed crossing is a fault, never a completed ten-second stop.
            if self.state == 'BRAKE' and self.reason and now >= self.brake_end:
                return self.halt(self.reason)
            if self.state not in self.STOPPED and getattr(cross, 'brake_requested', False):
                fault = getattr(cross, 'fault_reason', '')
                if self.state == 'REACQUIRE' and getattr(cross, 'confirmed', False):
                    return self.halt('CROSSWALK_START_TOO_CLOSE')
                if fault:
                    self.reason = fault
            settings=self.s
            try:
                self.s=self.crosswalk_settings
                cmd=super().update(now,lane,cone,cross)
            finally:
                self.s=settings
            if cmd.state=='BRAKE':
                cmd.owner='crosswalk_saved'
                cmd.duty=10600
            if self.cross_done and cmd.state!='HALT':
                return self.advance(now,'CROSSWALK_HOLD_COMPLETE')
            return cmd
        if self.stage=='TRAFFIC':
            return self._traffic(now,lane,cone)
        if self.stage=='CONES':
            if cone.target_id is not None: self.attempted_cones.add(cone.target_id)
            completed=getattr(cone,'completed_target_id',None)
            if completed is not None:
                self.completed_cones.add(completed)
                if self.first_cone_completed_at is None:
                    self.first_cone_completed_at=now
            settings = self.s
            try:
                self.s = self.cone_settings
                cmd = super().update(now,lane,cone,None)
            finally:
                self.s = settings
            if cmd.state in ('HALT','FAULT_BRAKE'): return cmd
            if cone.state=='STRAIGHT' and abs(cone.turn_us)<=.5:
                if len(self.completed_cones)>=self.cfg['expected_cones']:
                    return self.advance(now,'TWO_CONES_CONFIRMED')
                # A true parking entrance can end a field run with an unconfirmed
                # cone, but cannot skip an outstanding traffic-light stage.
                if (self.order[self.order.index('CONES')+1]=='PARK_SEARCH'
                        and self.attempted_cones and self.parking_observation.get('observation_ready')
                        and self.parking_observation.get('valid')):
                    self.unconfirmed_cones_at_parking=max(0,self.cfg['expected_cones']-len(self.completed_cones))
                    return self.advance(now,'PARK_ENTRANCE_WITH_UNCONFIRMED_CONES')
            # Deadline starts on confirmed maneuver completion, never detection.
            # Do not bypass a remaining traffic stage or a latched fault.
            if (self.CONE_PARK_TIMEOUT_SECONDS is not None
                    and self.first_cone_completed_at is not None
                    and now-self.first_cone_completed_at>=self.CONE_PARK_TIMEOUT_SECONDS
                    and self.order[self.order.index('CONES')+1]=='PARK_SEARCH'):
                self.unconfirmed_cones_at_parking=max(0,self.cfg['expected_cones']-len(self.completed_cones))
                return self.advance(now,'FIRST_CONE_PARK_TIMEOUT')
            return cmd
        if self.stage=='PARK_SEARCH':
            # A latched motion fault must finish braking, never become parking.
            if self.state=='FAULT_BRAKE':
                return super().update(now,lane,cone,None)
            # This is the parking detector's upright board above an entrance,
            # not generic blue pixels (the track and cones may also be blue).
            board_seen=(self.parking_observation.get('board_visible') is True
                        or bool(self.parking_observation.get('board_conflict')
                                and self.parking_observation.get('board_boxes')))
            entrance_ready=(self.parking_observation.get('valid') and
                            self.parking_observation.get('observation_ready'))
            aligned=(entrance_ready and lane.valid and getattr(lane,'heading_valid',False)
                     and abs(getattr(lane,'heading_error_px',float('inf')))<=10)
            self.parking_alignment_hits=self.parking_alignment_hits+1 if aligned else 0
            if board_seen or self.parking_alignment_hits>=3:
                self.stage='PARK_SETTLE'
                self.transition_reason=('BLUE_BOARD_YELLOW_WHITE_HANDOFF' if board_seen
                                        else 'PARK_ENTRANCE_CONFIRMED')
                self.state,self.owner='BRAKE','none'
                self.steer=C.STEER_CENTER_US
                self.brake_end=now+C.CROSS_FR_BRAKE_TIME
                return self._command(C.PARK_MANUAL_BRAKE_DUTY,'PARK_ENTRY_SETTLE_BRAKE')
            settings = self.s
            try:
                self.s = self.parking_search_settings
                return super().update(now,lane,cone,None)
            finally:
                self.s = settings
        if self.stage=='PARK_SETTLE':
            if now<self.brake_end:
                return self._command(C.PARK_MANUAL_BRAKE_DUTY,'PARK_ENTRY_SETTLE_BRAKE')
            self.state='PARK_SETTLE'
            if self.settle_end is None:
                self.settle_end=now+self.cfg['parking_settle_seconds']
            if now>=self.settle_end:
                self.stage,self.state='PARKING','PARK_WAIT'
                self.transition_reason='PARK_STATIONARY_BASELINE'
            return self._command(C.STOP_DUTY,'PARK_STATIONARY_BASELINE')
        if self.stage=='PARKING':
            if self.clock is not None: now=self.clock()
            request=self.parking.update(now,self.parking_observation)
            self.state,self.owner=request['state'],'parking'
            self.steer=request['steer_us']
            if self.state=='HALT': return self.halt(request['reason'])
            if self.state=='DONE':
                self.stage='QR_CAPTURE'
                self.transition_reason='PARK_MOTION_COMPLETE'
                return Command('DONE','none',C.STEER_CENTER_US,C.STOP_DUTY,'PARKED_QR_READY')
            if self.state=='WAIT': self.state='PARK_WAIT'
            driver_state='BRAKE' if self.state in ('BRAKE','FAULT_BRAKE') else self.state
            return Command(driver_state,'parking',request['steer_us'],request['duty'],request['reason'])
        return self.halt('UNKNOWN_RACE_STAGE')

    def _traffic(self, now, lane, cone):
        if self.signal is not None:
            return self._field_traffic(now,lane,cone)
        light=self.light
        seen=light.visible and light.color in ('RED','YELLOW','GREEN')
        green=seen and light.color=='GREEN' and light.confirmed
        if self.traffic_mode=='red-timer' and seen and light.color=='RED':
            self.traffic_red_seen=True
        if self.state=='LIGHT_BRAKE':
            if now<self.brake_end: return self._command(C.CROSS_FR_BRAKE_DUTY,'LIGHT_BRAKE')
            if self.light_fault: return self.halt(self.light_fault)
            self.state,self.owner='LIGHT_WAIT','none'
            if self.traffic_red_seen: return self._wait_red_timer(now)
            return self._command(C.STOP_DUTY,'WAIT_FRESH_GREEN')
        if self.state=='LIGHT_WAIT':
            if self.traffic_red_seen: return self._wait_red_timer(now)
            if green: return self.advance(now,'GREEN_CONFIRMED')
            return self._command(C.STOP_DUTY,'WAIT_FRESH_GREEN')
        if seen and light.color in ('RED','YELLOW'):
            return self._light_brake(now,light.color+'_SEEN')
        if green: return self.advance(now,'GREEN_CONFIRMED')
        if seen: self.light_last_seen=now
        elif self.light_last_seen is not None and now-self.light_last_seen>=.4:
            return self._light_brake(now,'LIGHT_LOST',True)
        command=super().update(now,lane,cone,None)
        # Approach the lights slowly to leave room for braking.
        if command.duty>C.STOP_DUTY:
            command.duty=min(command.duty,self.cfg.get('traffic_approach_duty',12300))
        return command

    def _field_traffic(self, now, lane, cone):
        signal=self.signal.step(self.light,now)
        if signal.reason in ('GREEN_CONFIRMED', 'RED_OFF_CONFIRMED', 'RED_WAIT_9S_FALLBACK'):
            return self.advance(now,signal.reason)
        if signal.state in ('LIGHT_BRAKE','LIGHT_WAIT'):
            self.state=signal.state
            self.owner='traffic_saved' if signal.state=='LIGHT_BRAKE' else 'none'
            if signal.state=='LIGHT_WAIT': self.steer=C.STEER_CENTER_US
            duty = 10600 if signal.state == 'LIGHT_BRAKE' else signal.duty
            return self._command(duty,signal.reason)
        command=super().update(now,lane,cone,None)
        if command.duty>C.STOP_DUTY: command.duty=signal.duty
        return command

    def _wait_red_timer(self, now):
        if self.traffic_wait_end is None:
            self.traffic_wait_end=now+self.RED_STOP_WAIT_SECONDS
        if now>=self.traffic_wait_end:
            return self.advance(now,'RED_TIMER_COMPLETE')
        return self._command(C.STOP_DUTY,'WAIT_RED_TIMER')

    def mark_applied(self, command, applied_at):
        if self.signal is not None and self.stage=='TRAFFIC':
            self.signal.applied(command,applied_at)
        if (command.state=='LIGHT_WAIT' and command.reason=='WAIT_RED_TIMER'
                and not self.traffic_wait_applied):
            # Anchor the wait to the applied neutral command, after braking.
            self.traffic_wait_end=applied_at+self.RED_STOP_WAIT_SECONDS
            self.traffic_wait_applied=True
        if self.stage=='PARKING' and command.state=='BRAKE':
            self.parking.brake_started_at=applied_at
        elif command.state in ('BRAKE','FAULT_BRAKE','LIGHT_BRAKE'):
            self.brake_end=applied_at+(.25 if command.owner in ('crosswalk_saved','traffic_saved') else
                C.CROSS_FR_BRAKE_TIME if command.state in ('BRAKE','LIGHT_BRAKE') else C.BRAKE_TIME)
        if command.state=='HOLD': self.hold_end=applied_at+self.s.hold_seconds
        if command.state=='PARK_SETTLE':
            self.settle_end=applied_at+self.cfg['parking_settle_seconds']


class RaceController(FusionController):
    def __init__(self, settings, cfg, light_cfg, **kwargs):
        p=Params(color=cfg['cone_color'],roi_top=cfg['cone_roi_top'],front_x=.3888,
                 max_turn_us=cfg['cone_turn_us'],min_turn_us=cfg['cone_min_turn_us'],
                 padding=cfg['cone_padding'],hold_peak_turn=True,slew_us_s=1800,
                 pass_seconds=.2,lane_guidance=True,lane_margin_ratio=cfg['cone_lane_margin'],
                 recovery_turn_us=130,protection_stop=cfg['cone_protection'])
        super().__init__(settings,cone_params=p,**kwargs)
        cone_mode=cfg.get('cone_mode','original')
        if cone_mode not in ('original','relaxed'): raise ValueError('Invalid cone mode')
        self.cone=(RelaxedAvoider if cone_mode=='relaxed' else RaceAvoider)(self.cone.p)
        self.policy=RacePolicy(settings,cfg)
        self.traffic=TrafficLightDetector(light_cfg)
        self.light_frame=False
        self.traffic_timing_ms = None
        self.light_cfg=light_cfg
        self.light_result=LightResult()
        self.parking_perception=SideExitPerception(front_x=self.policy.parking.cfg['front_x'])
        self.parking_observation={}

    def cones_enabled(self): return self.policy.stage=='CONES'
    def cross_enabled(self): return self.policy.stage=='CROSSWALK'

    def update_extra(self, frame, frame_id, captured_at, now):
        if self.policy.stage=='CONES':
            # An entrance found before a maneuver is not evidence after it.
            self.parking_observation={}
            self.policy.parking_observation={}
        if self.policy.stage=='TRAFFIC':
            self.light_frame=True
            light_started = time.perf_counter()
            self.light_result=self.traffic.update(frame)
            if self.policy.signal is not None:
                self.policy.signal.observe_lamp(frame,self.light_result)
            self.traffic_timing_ms = {
                'light_ms': round((time.perf_counter() - light_started) * 1000, 3)
            }
            self.policy.light=self.light_result
        parking_probe=(self.policy.stage=='CONES' and self.policy.attempted_cones
                       and (self.cone_result is None or self.cone_result.state=='STRAIGHT')
                       and self.policy.order[-2]=='CONES')
        if self.policy.stage in ('PARK_SEARCH','PARKING') or parking_probe:
            if (self.policy.stage=='PARKING' and self.policy.settle_end is not None
                    and captured_at<=self.policy.settle_end):
                self.policy.parking_observation={}
                return
            self.parking_observation=self.parking_perception.update(
                frame,frame_id,captured_at,self.policy.parking.selected_side)
            self.policy.parking_observation=self.parking_observation

    def step(self, *args):
        self.traffic_timing_ms = None
        self.light_frame=False
        stage,state=self.policy.stage,self.policy.state
        cmd=super().step(*args)
        if cmd is not None and self.policy.state=='LIGHT_WAIT' and state!='LIGHT_WAIT':
            if self.policy.signal is None: self.traffic.reset_confirmation()
            self.light_result.confirmed=False
        if stage!='PARKING' and self.policy.stage=='PARKING':
            self.parking_perception=SideExitPerception(front_x=self.policy.parking.cfg['front_x'])
            self.parking_observation={}
            self.policy.parking_observation={}
        return cmd

    def extra_event(self):
        observed=self.light_frame and self.policy.signal is not None
        fields=(dict(light_candidates=self.light_result.candidates,source=self.light_result.source,
                     color=self.light_result.color,light_state=self.policy.signal.state) if observed else {})
        if self.traffic_timing_ms is not None:
            fields['traffic_timing_ms'] = dict(self.traffic_timing_ms)
        if self.policy.stage == 'CROSSWALK' and hasattr(self.cross_result, 'distance_cm'):
            fields.update(cross_distance_cm=self.cross_result.distance_cm,
                          cross_confirmed=self.cross_result.confirmed,
                          cross_trigger_cm=self.cross.trigger_cm)
            if hasattr(self.cross,'reference_edge'):
                fields.update(cross_trigger_cm=None,cross_trigger_mode='sampled_image_edge',
                              cross_reference_edge=self.cross.reference_edge,
                              cross_reference_truth_cm=self.cross.reference_truth_cm)
        return dict(mission_stage=self.policy.stage,
                    transition_reason=self.policy.transition_reason,
                    traffic=asdict(self.light_result if self.light_frame else LightResult()),parking=self.parking_observation,
                    traffic_mode=self.policy.traffic_mode,
                    lane_mode_config=self.policy.cfg.get('lane_mode','original'),
                    cone_mode=self.policy.cfg.get('cone_mode','original'),
                    cone_target_relink=getattr(self.cone,'last_target_relink',None),
                    guidance_source=('YELLOW_WHITE_PARKING' if self.policy.stage in
                                     ('PARK_SETTLE','PARKING','QR_CAPTURE') else 'WHITE_LANE'),
                    traffic_wait_end=self.policy.traffic_wait_end,
                    traffic_red_wait_started_at=getattr(self.policy.signal,'red_wait_started_at',None),
                    traffic_red_wait_deadline=getattr(self.policy.signal,'red_wait_deadline',None),
                    completed_cones=sorted(self.policy.completed_cones),
                    first_cone_completed_at=self.policy.first_cone_completed_at,
                    attempted_cones=sorted(self.policy.attempted_cones),
                    unconfirmed_cones_at_parking=self.policy.unconfirmed_cones_at_parking,
                    parking_verified=False,**fields)

    def draw_extra(self, frame):
        out=frame.copy()
        if self.light_frame:
            out=self.traffic.draw(out,self.light_result,self.light_cfg)
            for color,(x,y,w,h) in getattr(self.light_result,'candidates',()):
                cv2.rectangle(out,(x,y),(x+w-1,y+h-1),(0,0,255) if color=='RED' else (0,255,0),2)
        cv2.putText(out,self.policy.stage,(4,106),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,255),1)
        return out
