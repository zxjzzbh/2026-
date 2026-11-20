"""Optional relaxed lane geometry, without changing the original global config.

Uses the same observed tape candidates, steering gains, speed and loss guards.
Select with race_master_run.py --lane-mode relaxed (also no-traffic wrapper).
"""
from types import SimpleNamespace
from lane_follow import LaneFollower


class RelaxedLaneFollower(LaneFollower):
    def __init__(self):
        super().__init__()
        self.limits = SimpleNamespace(
            LANE_PAIR_MIN_WIDTH_RATIO=.12,
            LANE_PAIR_MIN_BOTTOM_WIDTH_RATIO=.28,
            LANE_VP_MAX_Y_RATIO=.90,
            LANE_CENTER_MAX_JUMP_PX=42.,
            LANE_WIDTH_MAX_REL_JUMP=.45,
        )
        self.vp_height_tolerance_ratio = .10
