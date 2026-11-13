"""Parking-only stop hysteresis with current ground-motion corroboration."""
import math


class ParkingStopEvidence:
    def __init__(self):
        self.stationary=False

    def update(self,sample,*,enabled):
        motion=dict(sample.get('motion') or {})
        if not enabled or motion.get('valid') is not True or type(motion.get('moving')) is not bool:
            self.stationary=False
            return motion
        flow=sample.get('parking_ground_motion') or {}
        slow=False;clearly_moving=False
        try:
            dt=sample['captured_s']-flow['previous_captured_s']
            current=(flow.get('valid') is True and flow.get('frame_id')==sample['frame_id']
                     and flow.get('captured_s')==sample['captured_s'] and 0<dt<=.5)
            speed=float(flow['translation_m'])/dt
            rotation=flow['transform']
            yaw_rate=abs(math.atan2(rotation[1][0],rotation[0][0]))/dt
            if current and math.isfinite(speed) and speed>=0:
                slow=speed<=.03 and yaw_rate<=.04
                clearly_moving=speed>.06 or yaw_rate>.08
        except (KeyError,TypeError,ValueError,IndexError,ZeroDivisionError):pass
        pixel=motion.get('pixel_motion_per_100ms')
        # Enter stationary only from the original stop signal. Hold it over
        # small pixel jitter only with independent, fresh low ground motion.
        hold=(self.stationary and slow and type(pixel) in (float,int) and 0<=pixel<=.6)
        stopped=(motion['moving'] is False or hold) and not clearly_moving
        self.stationary=stopped
        return {**motion,'moving':not stopped,'parking_stop_hysteresis':bool(hold and motion['moving']),
                'ground_slow_corroborated':slow}
