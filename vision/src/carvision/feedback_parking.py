"""Metric low-speed parking decisions; brake is NOT a reverse PWM alias.

This module emits speed/brake intentions only. It deliberately has no hardware
adapter: the current S4 direction API cannot yet provide a calibrated slow-speed
setpoint or a verified non-reversing brake. Never map `brake` to 1300 us by guess.
"""
import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass
class ParkingLimits:
    search_speed_mps: float = .08
    creep_speed_mps: float = .03
    creep_zone_m: float = .40
    target_distance_m: float = .15
    position_margin_m: float = .02
    speed_tolerance_mps: float = .01
    stationary_confirmation_s: float = .4
    hold_s: float = 3
    maximum_feedback_age_s: float = .25
    command_lease_s: float = .10
    maximum_search_s: float = 10

    def validate(self):
        if any(not number(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('limits must be positive finite numbers')
        if not self.creep_speed_mps <= self.search_speed_mps <= .15:
            raise ValueError('initial parking speed goals must be within 0.15 m/s')
        if not 0 < self.target_distance_m < .3 or self.position_margin_m >= self.target_distance_m:
            raise ValueError('invalid parking position goal/margin')
        if self.hold_s != 3 or self.command_lease_s > .25:
            raise ValueError('parking hold is 3 seconds; command lease cannot exceed 250 ms')
        return self


def readiness(calibration, limits=None):
    limits=(limits or ParkingLimits()).validate()
    missing=[]
    for key in ('esc_model','esc_running_mode','speed_feedback_source','brake_evidence'):
        if not isinstance(calibration.get(key),str) or not calibration[key].strip():missing.append(key)
    for key in ('speed_feedback_verified','low_speed_control_verified','distance_feedback_verified',
                'active_brake_verified','brake_does_not_reverse_verified'):
        if calibration.get(key) is not True:missing.append(key)
    for key in ('braking_deceleration_lower_bound_mps2','brake_response_upper_bound_s','minimum_stable_speed_mps'):
        if not number(calibration.get(key)) or calibration[key]<=0:missing.append(key)
    minimum=calibration.get('minimum_stable_speed_mps')
    if number(minimum) and minimum>limits.creep_speed_mps:missing.append('creep_speed_below_measured_stable_range')
    bounds=calibration.get('distance_verified_range_m')
    if (not isinstance(bounds,list) or len(bounds)!=2 or not all(number(v) for v in bounds)
            or not 0<=bounds[0]<limits.target_distance_m-limits.position_margin_m
            or bounds[1]<limits.creep_zone_m):
        missing.append('distance_coverage_including_final_stop_zone')
    return {'ready':not missing,'missing':missing,'hardware_output':False,'limits':asdict(limits)}


class FeedbackParkingController:
    def __init__(self, calibration, limits=None):
        self.calibration=calibration
        self.limits=(limits or ParkingLimits()).validate()
        self.phase='search'
        self.last_t=self.started=self.stationary_since=self.hold_since=None
        self.seen=False
        self.fault=None
        self.announced=False
        self.final_creep=False

    def _intent(self,action,reason,target_speed=0,brake=0,**extra):
        if target_speed>0 and brake>0:raise ValueError('traction and brake cannot be requested together')
        return {'schema':'feedback-parking-intent-1','phase':self.phase,'action':action,'reason':reason,
                'target_speed_mps':target_speed,'brake_demand':brake,'valid_for_s':self.limits.command_lease_s,
                'hardware_output':False,'reverse_requested':False,**extra}

    def _fault(self,reason,speed=None):
        self.fault=self.fault or reason
        self.phase='fault'
        # A hardware adapter must implement bounded verified forward braking;
        # it may not reinterpret this as a reverse-traction command.
        if number(speed) and speed>self.limits.speed_tolerance_mps and self.calibration.get('active_brake_verified') is True:
            return self._intent('brake',self.fault,brake=1)
        return self._intent('neutral',self.fault)

    def step(self,observation,now):
        r=readiness(self.calibration,self.limits)
        if not r['ready']:
            self.phase='blocked'
            return self._intent('neutral','calibration_required',missing=r['missing'])
        if not isinstance(observation,dict) or not number(now):raise ValueError('timestamped feedback required')
        v=observation.get('speed_mps')
        if self.fault:return self._fault(self.fault,v)
        c=self.limits
        if self.last_t is not None and not 0<now-self.last_t<=c.maximum_feedback_age_s:
            return self._fault('control_update_gap',v)
        self.last_t=now
        if self.started is None:self.started=now
        stamp=observation.get('speed_captured_s')
        if (observation.get('speed_valid') is not True or not number(v) or not number(stamp)
                or not 0<=now-stamp<=c.maximum_feedback_age_s):
            return self._fault('measured_speed_unavailable',v)
        if v < -c.speed_tolerance_mps:return self._fault('unexpected_reverse_motion')
        if observation.get('camera_ok') is not True:return self._fault('camera_unavailable',v)
        if self.phase=='done':return self._intent('neutral','hold_complete_remain_stopped')

        d=observation.get('distance_m')
        dstamp=observation.get('distance_captured_s')
        distance_ok=(observation.get('distance_valid') is True and number(d) and number(dstamp)
                     and 0<=now-dstamp<=c.maximum_feedback_age_s)
        if not distance_ok:
            if self.seen or observation.get('crosswalk_visible') is True:
                return self._fault('target_distance_unavailable',v)
            if observation.get('search_region_valid') is not True:
                return self._fault('search_region_unverified',v)
            if now-self.started>c.maximum_search_s:return self._fault('search_time_limit',v)
            if v>c.search_speed_mps+c.speed_tolerance_mps:
                return self._intent('brake','search_speed_exceeded',brake=min(1,(v-c.search_speed_mps)/c.search_speed_mps))
            self.phase='search'
            return self._intent('set_speed','slow_forward_search',target_speed=c.search_speed_mps)
        if d<0:return self._fault('crosswalk_overrun',v)
        bounds=self.calibration['distance_verified_range_m']
        if not bounds[0]<=d<=bounds[1]:return self._fault('distance_outside_verified_range',v)
        self.seen=True
        a=self.calibration['braking_deceleration_lower_bound_mps2']
        delay=self.calibration['brake_response_upper_bound_s']+max(now-stamp,now-dstamp)
        remaining=max(0,d-c.target_distance_m-c.position_margin_m)
        # v*delay + v^2/(2*a) <= remaining, using a measured conservative bound.
        envelope=max(0,math.sqrt((a*delay)**2+2*a*remaining)-a*delay)
        cap=c.creep_speed_mps if d<=c.creep_zone_m or self.final_creep else c.search_speed_mps
        target=min(cap,envelope)
        stopping=max(0,v)*delay+max(0,v)**2/(2*a)+c.position_margin_m
        if self.phase in ('braking','settle','hold') or d<=c.target_distance_m+stopping or target<self.calibration['minimum_stable_speed_mps']:
            if abs(v)>c.speed_tolerance_mps:
                self.stationary_since=self.hold_since=None
                self.phase='braking'
                return self._intent('brake','reduce_measured_speed_to_zero',brake=1,estimated_stopping_distance_m=stopping)
            if d>=.3:
                # Stronger-than-expected braking may stop early. Re-approach
                # only at the verified creep speed; never jump to cruise again.
                self.final_creep=True
                self.phase='creep'
                self.stationary_since=self.hold_since=None
                return self._intent('set_speed','stopped_early_resume_creep',target_speed=c.creep_speed_mps)
            if self.stationary_since is None:self.stationary_since=now
            if now-self.stationary_since<c.stationary_confirmation_s:
                self.phase='settle'
                return self._intent('neutral','confirm_continuous_stationary_feedback')
            if self.hold_since is None:self.hold_since=now
            events=[]
            if not self.announced:
                self.announced=True;events=[{'event':'speak','text':'我停车了啊'}]
            if now-self.hold_since>=c.hold_s:
                self.phase='done'
                return self._intent('neutral','hold_complete_remain_stopped',events=events)
            self.phase='hold'
            return self._intent('neutral','stationary_three_second_hold',events=events,
                                hold_remaining_s=max(0,c.hold_s-(now-self.hold_since)))
        self.stationary_since=self.hold_since=None
        self.phase='creep' if d<=c.creep_zone_m else 'approach'
        if v>target+c.speed_tolerance_mps:
            return self._intent('brake','measured_speed_above_distance_speed_limit',brake=min(1,(v-target)/max(cap,.01)),
                                speed_limit_mps=target,estimated_stopping_distance_m=stopping)
        return self._intent('set_speed','closed_loop_low_speed_approach',target_speed=target,
                            estimated_stopping_distance_m=stopping)


def check_file(path):
    data=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    return readiness(data['calibration'],ParkingLimits(**data['limits']))
