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


class ImagePD:
    """Bounded, time-based image feedback with a filtered derivative.

    Target/camera changes reset derivative history. Cached frames cannot add
    corrections. These normalized gains are trial values, not wheel calibration.
    """
    def __init__(self, kp=1., kd=.12, *, deadband=.018, derivative_tau=.06):
        if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0
               for v in (kp, kd, deadband, derivative_tau)):
            raise ValueError('invalid image PD settings')
        self.kp, self.kd = kp, kd
        self.deadband, self.derivative_tau = deadband, derivative_tau
        self.reset()

    def reset(self):
        self.captured_s = self.error = self.context = None
        self.derivative = self.output = 0.

    def update(self, error, captured_s, *, context=None, gain=1.):
        if any(type(v) not in (int, float) or not math.isfinite(v)
               for v in (error, captured_s, gain)) or gain <= 0:
            raise ValueError('invalid image PD observation')
        same_context=context==self.context
        if same_context and self.captured_s is not None and captured_s < self.captured_s:
            raise ValueError('image PD observation time moved backwards')
        if same_context and self.captured_s == captured_s:
            return self.output
        dt = None if self.captured_s is None else captured_s-self.captured_s
        if dt is None or not .001 <= dt <= .25 or context != self.context:
            self.derivative = 0.
        else:
            derivative = max(-2., min(2., (error-self.error)/dt))
            alpha = 1. if self.derivative_tau == 0 else 1-math.exp(-dt/self.derivative_tau)
            self.derivative += alpha*(derivative-self.derivative)
        self.error, self.captured_s, self.context = error, captured_s, context
        self.output = 0. if abs(error) < self.deadband else max(-1., min(1., gain*(self.kp*error+self.kd*self.derivative)))
        return self.output
