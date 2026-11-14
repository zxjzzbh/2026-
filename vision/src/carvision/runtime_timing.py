"""Capture-age budgets; page ownership and physical evidence are separate."""

VISION_MAX_AGE_S = .5
LANE_HISTORY_MAX_GAP_S = .5
MOTION_MAX_AGE_S = .25
MOTION_RECOVERY_S = 1.5

# The local sender keeps the 500 ms actuator lease alive at 50 ms cadence.
# This is a computation window for already accepted race decisions, not a
# longer capture-age allowance. Every task checks real evidence at acceptance;
# parking planning also reserves travel during this bounded decision window.
RACE_CONTROL_WINDOW_S = .75
