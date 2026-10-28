"""Current manual endpoints; historical no-opt-in profiles remain unchanged.

The current span is 260 us on both sides of 1610 us. Legacy span=180 remains
a profile selector for older launchers. Symmetric PWM does not imply equal angles.
"""
from .steering_limits import STEERING_LEFT_US, STEERING_CENTER_US, STEERING_RIGHT_US

REQUESTED_CENTER_US = STEERING_CENTER_US
REQUESTED_SPAN_US = 260
LEGACY_PROFILE_SELECTOR_US = 180
REQUESTED_MIN_US = STEERING_RIGHT_US
REQUESTED_MAX_US = STEERING_LEFT_US
CONTINUOUS_GAIN_PERCENT = 50  # Operator-requested manual sensitivity, 2026-10-14.


def manual_target(center_us, endpoint_us, gain_percent):
    """Scale directional input only; the raw S3 envelope stays unchanged."""
    if (type(center_us) is not int or type(endpoint_us) is not int
            or type(gain_percent) is not int or not 0 <= gain_percent <= 100):
        raise ValueError('manual steering gain must be an integer from 0 to 100')
    return center_us + round((endpoint_us-center_us)*gain_percent/100)


def manual_endpoints(center_us, span_us=0):
    if type(span_us) is not int or span_us not in (0, LEGACY_PROFILE_SELECTOR_US, REQUESTED_SPAN_US):
        raise ValueError('unsupported manual steering span')
    if span_us:
        if type(center_us) is not int or center_us != REQUESTED_CENTER_US:
            raise ValueError('current manual endpoints require the requested 1610-us center')
        return REQUESTED_MAX_US, REQUESTED_MIN_US
    return 1750, 1550
