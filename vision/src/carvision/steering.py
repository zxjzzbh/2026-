"""Shared S3 endpoints supplied by the team on 2026-10-13 (microseconds).

These are output endpoints, not a measured wheel-angle or turn-radius model.
Historical calibration records keep their original pulse labels.
"""
import math

RIGHT_US=1350
CENTER_US=1610
LEFT_US=1870


def from_offset(offset):
    """Positive image/path error means steer right; zero means straight."""
    if type(offset) not in (int,float) or not math.isfinite(offset):raise ValueError('invalid steering offset')
    offset=max(-1.,min(1.,offset))
    endpoint=RIGHT_US if offset>=0 else LEFT_US
    return round(CENTER_US+abs(offset)*(endpoint-CENTER_US))
