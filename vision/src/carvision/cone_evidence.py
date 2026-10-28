"""Fresh, same-camera lane evidence shared by obstacle preparation/control."""
from .brake_parking import numeric


def paired_lane(nav):
    if not isinstance(nav, dict):
        return False
    return bool(nav.get('lane_bounds_px') and numeric(nav.get('ego_fraction'))
                and not nav.get('line_missing') and not nav.get('line_held')
                and not nav.get('boundary_danger') and nav.get('ready') is not False
                and nav.get('line_state', 'paired') == 'paired'
                and nav.get('boundary_observed', True))


class StablePairedLane:
    def __init__(self, duration=.15, frames=3):
        self.duration, self.frames = duration, frames
        self.reset()

    def reset(self):
        self.first = self.last = self.identity = self.camera = None
        self.count = 0

    def update(self, nav, now):
        t = (nav or {}).get('captured_s')
        if not paired_lane(nav) or not numeric(t) or not 0 <= now-t <= .25:
            self.reset()
            return False
        camera = nav.get('source_camera')
        identity = (camera, nav.get('frame_id', t))
        if self.last is not None and (camera != self.camera or t < self.last or t-self.last > .25):
            self.reset()
        if identity != self.identity and (self.last is None or t > self.last):
            if self.first is None:
                self.first = t
            self.last, self.identity, self.camera = t, identity, camera
            self.count += 1
        return self.ready(now)

    def ready(self, now):
        return bool(self.first is not None and self.last is not None
                    and 0 <= now-self.last <= .25 and self.count >= self.frames
                    and self.last-self.first >= self.duration-1e-9)
