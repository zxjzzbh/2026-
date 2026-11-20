#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Guide between longitudinal tapes; stop 0.3 seconds after board side exit.

Hardware-free; Python 3.7+. Image exit is not proof of physical clearance or
four-wheel parking. The passage tolerates a missing entrance crossbar, but
requires fresh yellow/white longitudinal guidance with only a brief dropout.
"""

from parking_bypass_policy import BypassTrialPolicy, _finite


DEFAULT_SIDE_EXIT_SECONDS = 0.3
DEFAULT_SIDE_EXIT_SLEW_US_S = 600.0


class SideExitParkingPolicy(BypassTrialPolicy):
    """WAIT -> ALIGN -> PASS_TIMED -> BRAKE -> DONE.

    The perception producer owns side_exit_confirmed: it must establish three
    consecutive fresh frames of departure on the previously locked board side,
    with the board's near edge previously outside the front image window.
    This policy does not equate arbitrary disappearance with side exit.
    """

    ALIGN_GEOMETRY_HOLD_SECONDS = 0.25
    ALIGN_LEFT_HEADING_DEADBAND = 0.02
    ALIGN_LEFT_RELIEF_KP_US = 500.0
    ALIGN_LEFT_RELIEF_MAX_US = 40.0

    def __init__(self, config_dict=None):
        supplied = {} if config_dict is None else dict(config_dict)
        pass_seconds = supplied.pop("pass_seconds", DEFAULT_SIDE_EXIT_SECONDS)
        pass_slew_us_s = supplied.pop('pass_slew_us_s', DEFAULT_SIDE_EXIT_SLEW_US_S)
        if not _finite(pass_seconds) or not 0 < pass_seconds <= 4.0:
            raise ValueError("pass_seconds must be finite and in (0, 4]")
        if not _finite(pass_slew_us_s) or pass_slew_us_s <= 0:
            raise ValueError('pass_slew_us_s must be finite and positive')
        supplied.setdefault("low_duty", 12800)
        supplied.setdefault("max_turn_us", 140)
        supplied.setdefault("kp_us", 700)
        if not _finite(supplied["low_duty"]) or supplied["low_duty"] > 12800:
            raise ValueError("Side-exit passage cannot exceed duty 12800")
        supplied.update(max_seconds=0, align_timeout_seconds=0,
                        creep_seconds=float(pass_seconds))
        super().__init__(supplied)
        self.cfg["pass_seconds"] = float(pass_seconds)
        self.cfg['pass_slew_us_s'] = float(pass_slew_us_s)
        self.cfg.update(lane_center_kp_us=450.0, lane_heading_kp_us=500.0,
                        lane_max_turn_us=100.0, lane_hold_seconds=0.20,
                        lane_loss_confirm_frames=3)

    def reset(self):
        super().reset()
        self.pass_started_at = None
        self.last_align_geometry_at = None
        self.last_lane_at = None
        self.last_lane_target = None
        self.lane_engaged = False
        self.lane_loss_started_at = None
        self.lane_loss_frames = 0
        self.lane_loss_last_frame = None

    def _start_brake(self, now, reason, fault=True):
        # End lane steering immediately when braking begins, including
        # a fault during the passage. DONE/HALT and hardware stop also center.
        if self.state == 'PASS_TIMED' or self.lane_engaged:
            self.steer = float(self.cfg['steer_center_us'])
        return super()._start_brake(now, reason, fault)

    def _remember_align_geometry(self, observation, result):
        if (result['state'] == 'ALIGN' and isinstance(observation, dict)
                and observation.get('geometry_valid') is True
                and observation.get('valid') is True
                and observation.get('observation_ready') is True):
            self.last_align_geometry_at = observation['captured_at']
        return result

    def _result(self, now):
        result = super()._result(now)
        if self.state == "PASS_TIMED":
            result["duty"] = int(round(self.cfg["low_duty"]))
        if self.state == "DONE" and self.pass_started_at is not None:
            self.reason = "SIDE_EXIT_STOPPED_NOT_PARKING_VERIFIED"
            result["reason"] = self.reason
        return result

    def _pass_observation_problem(self, now, observation):
        """Only freshness and explicit obstacle conflicts gate timed passage."""
        if not isinstance(observation, dict):
            return "OBSERVATION_MISSING"
        if observation.get("board_conflict") is True:
            return "BOARD_REENTERED_OR_CONFLICT"
        captured = observation.get("captured_at")
        if not _finite(captured):
            return "CAPTURE_TIME_INVALID"
        if captured > now + 0.025:
            return "CAPTURE_TIME_IN_FUTURE"
        if now - captured > self.cfg["max_observation_age"]:
            return "FRAME_EXPIRED"
        frame_id = observation.get("frame_id")
        if (isinstance(frame_id, bool) or not isinstance(frame_id, (str, int))
                or (isinstance(frame_id, str) and not frame_id)):
            return "FRAME_ID_INVALID"
        if observation.get("blocked") == "BOTH":
            return "BOTH_SIDES_BLOCKED"
        if observation.get("blocked") == self.selected_side:
            return "SELECTED_SIDE_BLOCKED"
        if frame_id == self.last_frame_id:
            if captured != self.last_captured_at:
                return "REPEATED_FRAME_TIMESTAMP_CHANGED"
            if now - self.last_fresh_at > self.cfg["max_observation_age"]:
                return "NO_FRESH_FRAME"
            return None
        if frame_id in self.seen_frame_ids:
            return "FRAME_REPLAYED"
        if self.last_captured_at is not None:
            if captured <= self.last_captured_at:
                return "CAPTURE_TIME_NOT_ADVANCING"
            if captured - self.last_captured_at > self.cfg["max_observation_age"]:
                return "FRESH_FRAME_GAP_TOO_LONG"
        if (isinstance(frame_id, int) and isinstance(self.last_frame_id, int)
                and frame_id <= self.last_frame_id):
            return "FRAME_ID_NOT_ADVANCING"
        return None

    def _record_pass_frame(self, now, observation):
        if observation["frame_id"] != self.last_frame_id:
            self.last_frame_id = observation["frame_id"]
            self.last_captured_at = observation["captured_at"]
            self.last_fresh_at = now
            self.seen_frame_ids.add(self.last_frame_id)

    def _observe_lane(self, now, observation):
        if not isinstance(observation, dict):
            return False
        guide = observation.get('lane_guide') or {}
        stamp = guide.get('captured_at')
        center, heading = guide.get('center_x'), guide.get('heading_error')
        if (guide.get('valid') is not True or guide.get('side') != self.selected_side
                or not _finite(stamp) or stamp != observation.get('captured_at')
                or not 0 <= now-stamp <= self.cfg['max_observation_age']
                or not _finite(center) or not 0 <= center <= 1
                or not _finite(heading) or abs(heading) > .25):
            return False
        if self.last_lane_at is not None and stamp < self.last_lane_at:
            return False
        correction = (self.cfg['lane_center_kp_us']*(center-self.cfg['front_x'])
                      + self.cfg['lane_heading_kp_us']*heading)
        limit = self.cfg['lane_max_turn_us']
        correction = max(-limit, min(limit, correction))
        self.last_lane_target = self.cfg['steer_center_us'] + self.cfg['steer_sign']*correction
        self.last_lane_at = stamp
        self.lane_loss_started_at = None
        self.lane_loss_frames = 0
        self.lane_loss_last_frame = None
        return True

    def _lane_steering(self, now, dt, observation):
        fresh = self._observe_lane(now, observation)
        self.lane_engaged = True
        if self.last_lane_at is None:
            return self._start_brake(now, 'LONGITUDINAL_LANES_LOST')
        if not fresh:
            # The caller already checked camera freshness/replay/conflicts.
            # Count distinct bad images, not control-loop polls. Measure the
            # bad-image interval using capture times, excluding processing lag.
            fid = observation['frame_id']
            captured = observation['captured_at']
            if fid != self.lane_loss_last_frame:
                if self.lane_loss_started_at is None:
                    self.lane_loss_started_at = captured
                self.lane_loss_frames += 1
                self.lane_loss_last_frame = fid
            if (self.lane_loss_frames >= self.cfg['lane_loss_confirm_frames']
                    and captured-self.lane_loss_started_at >= self.cfg['lane_hold_seconds']):
                return self._start_brake(now, 'LONGITUDINAL_LANES_LOST')
            self.reason = 'LONGITUDINAL_LANES_BRIEF_HOLD'
            return self._result(now)
        desired = self.last_lane_target
        step = self.cfg['pass_slew_us_s'] * min(max(dt, 0.0), 0.15)
        self.steer += max(-step, min(step, desired - self.steer))
        self.steer = max(self.cfg["steer_min_us"], min(self.cfg["steer_max_us"], self.steer))
        self.reason = ('SIDE_EXIT_LANE_GUIDED_PASS' if self.state == 'PASS_TIMED'
                       else 'FRONT_CLEAR_LANE_GUIDANCE')
        return self._result(now)

    def update(self, now, observation):
        if not _finite(now):
            raise ValueError("now must be a finite monotonic timestamp")
        if (self.state in ("WAIT", "BRAKE", "FAULT_BRAKE", "DONE", "HALT")
                or (self.last_now is not None and now < self.last_now)):
            # WAIT still requires the original fresh/valid/ready side checks.
            return self._remember_align_geometry(
                observation, super().update(now, observation))

        lane_fresh = self._observe_lane(now, observation)

        if self.state == "ALIGN" and not (
                isinstance(observation, dict)
                and observation.get("side_exit_confirmed") is True
                and observation.get("frame_id") != self.last_frame_id):
            # Start longitudinal guidance as soon as the tracked board clears
            # the front image window, BEFORE confirmed disappearance. Do not
            # start/restart the 0.3-second timer here.
            if (isinstance(observation, dict)
                    and observation.get('tracking_valid') is True
                    and observation.get('front_clear') is True):
                dt = 0.0 if self.last_now is None else now-self.last_now
                self.last_now = now
                problem = self._pass_observation_problem(now, observation)
                if problem is not None:
                    return self._start_brake(now, problem)
                self._record_pass_frame(now, observation)
                return self._lane_steering(now, dt, observation)
            if (isinstance(observation, dict)
                    and observation.get('geometry_valid') is False
                    and observation.get('tracking_valid') is True
                    and observation.get('board_visible') is True
                    and observation.get('front_clear') is not True
                    and observation.get('open_side') == self.selected_side
                    and self.last_align_geometry_at is not None):
                # Bridge only brief ground-line flicker with a live board track.
                # Hold the last steering command; never steer from cached pixels.
                self.last_now = now
                problem = self._pass_observation_problem(now, observation)
                if problem is not None:
                    return self._start_brake(now, problem)
                if now - self.last_align_geometry_at > self.ALIGN_GEOMETRY_HOLD_SECONDS:
                    return self._start_brake(now, 'ALIGN_GEOMETRY_LOSS_PERSISTED')
                self._record_pass_frame(now, observation)
                self.reason = 'ALIGN_BRIEF_GEOMETRY_HOLD'
                return self._result(now)
            # Visual alignment alone no longer starts a stopping countdown.
            aligned_observation = observation
            if isinstance(observation, dict):
                aligned_observation = dict(observation, clear_of_board=False)
                # Reduce accumulated left yaw before front-clear takeover.
                # Use only this frame's confirmed pair; never reverse the
                # avoidance turn or change the original logged observation.
                target = observation.get('target_x')
                if (lane_fresh and self.selected_side == 'LEFT'
                        and self.cfg['steer_sign'] == -1
                        and _finite(target) and 0 <= target < self.cfg['front_x']):
                    heading = observation['lane_guide']['heading_error']
                    relief = min(self.ALIGN_LEFT_RELIEF_MAX_US,
                                 self.ALIGN_LEFT_RELIEF_KP_US * max(
                                     0.0, heading-self.ALIGN_LEFT_HEADING_DEADBAND))
                    turn = min(self.cfg['max_turn_us'],
                               self.cfg['kp_us']*(self.cfg['front_x']-target))
                    aligned_observation['target_x'] = (
                        self.cfg['front_x']-max(0.0, turn-relief)/self.cfg['kp_us'])
            self.reason = "ALIGN_TO_SELECTED_EMPTY_SIDE"
            return self._remember_align_geometry(
                aligned_observation, super().update(now, aligned_observation))

        dt = 0.0 if self.last_now is None else now - self.last_now
        self.last_now = now
        if (self.state == "PASS_TIMED"
                and now - self.pass_started_at >= self.cfg["pass_seconds"]):
            return self._start_brake(now, "SIDE_EXIT_TIMED_PASS_COMPLETE", fault=False)
        problem = self._pass_observation_problem(now, observation)
        if problem is not None:
            return self._start_brake(now, problem)
        self._record_pass_frame(now, observation)
        if self.state == "ALIGN":
            self.state = "PASS_TIMED"
            self.pass_started_at = now
            # Keep the established output elapsed field and runner timer alias.
            self.creep_started_at = now
        return self._lane_steering(now, dt, observation)
