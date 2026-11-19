"""Bounded continuity policies; remembered requests are not position feedback."""
from .brake_parking import numeric
from .steering import CENTER_US


TARGET_GAP_S = .25


class TargetGap:
    def __init__(self):
        self.identifier = self.camera = self.first_missing_s = None
        self.offset = 0.

    def update(self, identifier, current, nav, now, last_request):
        camera = nav.get('source_camera')
        if identifier != self.identifier or camera != self.camera:
            self.identifier, self.camera = identifier, camera
            self.first_missing_s = None
            self.offset = 0.
        if identifier is None or current:
            self.first_missing_s = None
            self.offset = 0.
            return False
        if self.first_missing_s is None:
            # Include the processing age of the first missing frame. Further
            # missing/duplicate frames cannot move this deadline forward.
            self.first_missing_s = nav['captured_s']
            self.offset = max(-80., min(80., last_request-CENTER_US))
        return now < self.first_missing_s+TARGET_GAP_S-1e-9

    def request(self, now):
        remaining = max(0., self.first_missing_s+TARGET_GAP_S-now)
        return CENTER_US+self.offset*min(1., remaining/TARGET_GAP_S)

    def diagnostics(self, now):
        deadline = None if self.first_missing_s is None else self.first_missing_s+TARGET_GAP_S
        return {'target_missing_since_s': self.first_missing_s,
                'target_pending_deadline_s': deadline,
                'target_pending_remaining_s': max(0., deadline-now) if deadline is not None else 0.,
                'target_pending_limit_s': TARGET_GAP_S}


class ObservedLaneRecovery:
    """Two continuous current pairs may smoothly resume one known demand.

    This never resets StablePairedLane or the one-second recovery budget.
    True single/missing, a target change or a boundary correction cannot use it.
    """
    def __init__(self):
        self.previous = self.trusted = None

    @staticmethod
    def signature(nav, target):
        near = nav.get('ego_near_fraction', nav.get('ego_fraction'))
        far = nav.get('ego_far_fraction', near)
        half = nav.get('half_envelope')
        if not all(numeric(value) for value in (near, far, half)):
            return None
        return {'camera': nav.get('source_camera'), 'target': target,
                'captured_s': nav['captured_s'], 'near': near, 'far': far, 'half': half}

    def limit(self, nav, now, target, current_lane, reliable, requested, boundary_turn):
        safe = (current_lane and nav.get('line_state', 'paired') == 'paired'
                and nav.get('boundary_observed', True) and not nav.get('line_held')
                and not nav.get('line_missing') and not boundary_turn
                and not nav.get('boundary_margin_warning') and not nav.get('boundary_correction'))
        current = self.signature(nav, target) if safe else None
        old = self.previous
        self.previous = current
        if reliable and current:
            self.trusted = {**current, 'offset': requested-CENTER_US}
            return False, 80.
        trusted = self.trusted
        continuous = (current and old and trusted
            and current['camera'] == old['camera'] == trusted['camera']
            and current['target'] == old['target'] == trusted['target']
            and 0 < current['captured_s']-old['captured_s'] <= .25
            and 0 <= now-trusted['captured_s'] < 1.
            and abs(current['near']-old['near']) <= .10
            and abs(current['far']-old['far']) <= .12
            and abs(current['half']-old['half']) <= .04
            and (requested-CENTER_US)*trusted['offset'] > 0)
        if not continuous:
            return False, 80.
        return True, min(260., abs(trusted['offset']))
