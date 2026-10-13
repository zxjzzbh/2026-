"""Shared S3 endpoints supplied by the team on 2026-10-13 (microseconds).

These are output endpoints, not a measured wheel-angle or turn-radius model.
Historical calibration records keep their original pulse labels.
"""
import math

RIGHT_US=1350
CENTER_US=1610
LEFT_US=1870

RADIUS_ALIASES={'steering_radius_right_cm':'steering_radius_1350_cm',
                'steering_radius_left_cm':'steering_radius_1870_cm'}


def radius_settings(values):
    """Accept both names of these same endpoints; never reuse 1570/1650 data."""
    result=dict(values)
    for key,alias in RADIUS_ALIASES.items():
        if result.get(key) is not None and result.get(alias) is not None and result[key]!=result[alias]:
            raise ValueError('conflicting steering radius aliases: '+key)
        result[key]=result.get(key) if result.get(key) is not None else result.get(alias)
        result[alias]=result[key]
    return result


def from_offset(offset):
    """Positive image/path error means steer right; zero means straight."""
    if type(offset) not in (int,float) or not math.isfinite(offset):raise ValueError('invalid steering offset')
    offset=max(-1.,min(1.,offset))
    endpoint=RIGHT_US if offset>=0 else LEFT_US
    return round(CENTER_US+abs(offset)*(endpoint-CENTER_US))
