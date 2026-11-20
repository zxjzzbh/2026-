"""Shared pulse for calibrated crosswalk stops and the straight brake test.

This duration is an operator tuning choice, not a measured stopping distance.
The existing actuator deadlines and maximum of three braking pulses remain.
"""

from math import isfinite

CROSSWALK_BRAKE_PULSE_S = .35
MAX_BRAKE_PULSE_S = .50


def brake_cleanup_window_s(pulse_s, release_s=0.):
    """Bounded cleanup grace; it never changes an existing pulse deadline."""
    if (type(pulse_s) not in (int, float) or not isfinite(pulse_s)
            or not 0 < pulse_s <= MAX_BRAKE_PULSE_S
            or type(release_s) not in (int, float) or not isfinite(release_s)
            or not 0 <= release_s <= .3):
        raise ValueError('invalid finite brake cleanup timing')
    return max(.35, pulse_s + release_s + .05)
