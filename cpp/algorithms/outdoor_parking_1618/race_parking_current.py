"""Adapt the standalone outdoor parking guidance to the full race lifecycle."""
import config as C
from lane_follow import LaneFollower
from standalone_parking_20261119.parking_only_test import (
    BoardSidePerception, EntryLineTracker, OuterWhiteTracker, install_parking_guidance,
    drive_command, limit_left_parking_speed, measure_step, prepare_step_timing, steering_snapshot,
)

ORIGINAL_FRAME_AGE = C.MAX_FRAME_AGE_SECONDS
PARKING_STAGES = ('PARK_SEARCH', 'PARK_SETTLE', 'PARKING')


def sync_parking_age(ctl):
    age = .300 if ctl.policy.stage in PARKING_STAGES + ('QR_CAPTURE',) else ORIGINAL_FRAME_AGE
    C.MAX_FRAME_AGE_SECONDS = age
    ctl.s.max_age = age if ctl.policy.stage in PARKING_STAGES else ctl.original_max_age


def install_new_parking(ctl):
    ctl.original_max_age = ctl.s.max_age
    install_parking_guidance(ctl, board_min_saturation=75, left_pink=True, left_delay=.6)
    original_step, original_event = ctl.step, ctl.extra_event

    def step(*args):
        sync_parking_age(ctl)
        was_parking = ctl.policy.stage in PARKING_STAGES
        if was_parking:
            # The standalone searches with the original lane detector, including
            # when the preceding race stages used the optional relaxed detector.
            if type(ctl.lane) is not LaneFollower:
                ctl.lane = LaneFollower()
            prepare_step_timing(ctl)
            cmd = measure_step(ctl, 'controller_total_ms', original_step, *args)
        else:
            cmd = original_step(*args)
        sync_parking_age(ctl)
        return cmd

    def event():
        result = original_event()
        if ctl.policy.stage in PARKING_STAGES or ctl.policy.stage == 'QR_CAPTURE':
            result['guidance_source'] = 'OUTER_WHITE_ENTRY_YELLOW'
            result['entry_delay_started_at'] = ctl.policy.parking.entry_delay_started_at
            result['entry_delay_deadline'] = ctl.policy.parking.entry_delay_deadline
            result['parking_timing_ms'] = dict(ctl.step_timing)
            result['parking_version'] = 'standalone_parking_20261119'
        return result

    ctl.step, ctl.extra_event = step, event
    return ctl


def parking_driver_command(ctl, cmd):
    if ctl.policy.stage not in PARKING_STAGES:
        return cmd
    # The full race owns the driver. Only translate the command here.
    return drive_command(None, cmd, None)
