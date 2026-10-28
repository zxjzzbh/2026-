"""Manual throttle endpoints supplied by the operator on 2026-10-14.

Percentage is an input setting, not measured speed. Endpoint and intermediate
points are not marked field verified. Autonomous/brake settings are unchanged.
"""
NEUTRAL_US = 1500
FORWARD_MAX_US = 2000
REVERSE_MIN_US = 1000
DEFAULT_FORWARD_PERCENT = 15  # existing 1575-us forward point
DEFAULT_REVERSE_PERCENT = 40  # existing 1300-us reverse point
ENDPOINT_SOURCE = 'operator supplied 2000 forward, 1500 neutral, 1000 reverse on 2026-10-14'


def forward_pulse(percent):
    if type(percent) is not int or not 0 <= percent <= 100:
        raise ValueError('前进油门比例必须是0到100的整数')
    return NEUTRAL_US + round((FORWARD_MAX_US - NEUTRAL_US) * percent / 100)


def valid_forward_pulse(pulse):
    return type(pulse) is int and NEUTRAL_US <= pulse <= FORWARD_MAX_US


def reverse_pulse(percent):
    if type(percent) is not int or not 0 <= percent <= 100:
        raise ValueError('倒车油门比例必须是0到100的整数')
    return NEUTRAL_US - round((NEUTRAL_US - REVERSE_MIN_US) * percent / 100)


def valid_manual_pulse(pulse):
    return type(pulse) is int and REVERSE_MIN_US <= pulse <= FORWARD_MAX_US
