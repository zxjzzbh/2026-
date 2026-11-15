"""Shared measured/estimated space preference for the production controller.

The centimetre tie is an explicitly labelled same-slice width estimate. The
50 cm boundary warning uses the retained planar camera mapping, never a lane
fraction presented as a calibrated distance.
"""
from .cone_latest_engine.cone_side_choice import choose_corridor, side_spaces
from .cone_latest_engine.cone_right_boundary import RightBoundaryRecovery


class ProductionRouteChoice:
    def __init__(self, settings):
        self.settings = settings
        self.boundary = RightBoundaryRecovery(settings)
        self.key = self.pending = self.capture = None
        self.hits = 0
        self.report = {}

    def choose(self, positions, target, nav, now, side, ego):
        key = (nav.get('source_camera'), target['id'], target.get('identity_revision', 0))
        if key != self.key or side is None:
            self.pending = self.capture = None
            self.hits = 0
        self.key = key
        half = nav['half_envelope']
        lo, hi = half+.03, 1-half-.03
        space = half+target['half_width']+.03
        choices = [(abs(x-ego), direction,
                    lo if direction == 'left' else target['fraction']+space,
                    target['fraction']-space if direction == 'left' else hi, x)
                   for direction, x in positions]
        spaces = side_spaces(nav, target, self.settings, now)
        choice, report = choose_corridor(choices, spaces)
        self.report = report
        same = next((row for row in choices if row[1] == side), None)
        left = next((row for row in choices if row[1] == 'left'), None)
        if self.boundary.prefer_left and left is None:
            self.pending = self.capture = None
            self.hits = 0
            report.update(reason='right_boundary_no_feasible_left', waiting=True)
            return None
        if self.boundary.prefer_left and left is not None:
            choice = left
            report['preference_reason'] = 'calibrated_right_boundary_below_50cm'
        if same is not None and not (self.boundary.prefer_left and side == 'right'):
            self.pending = self.capture = None
            self.hits = 0
            report.update(selected_side=side, reason='retain_current_feasible_side', waiting=False)
            return same
        if choice is None:
            self.pending = self.capture = None
            self.hits = 0
            report.update(reason='no_current_vehicle_corridor', waiting=True)
            return None
        if side is None:
            report.update(selected_side=choice[1], waiting=False)
            return choice
        # A failed old corridor never remains the forward target. Confirm a
        # replacement from distinct frames, without jumping across a near cone.
        crossing = (ego-target['fraction'])*(choice[4]-target['fraction']) < 0
        close = (target.get('near') or target.get('image_close')
                 or target.get('y_fraction', 0) >= (.90 if nav.get('source_camera') == 'primary' else .75))
        if crossing and close:
            report.update(reason='near_cone_crossing_not_supported', waiting=True)
            return None
        stamp = (nav.get('source_camera'), nav.get('frame_id'), nav.get('captured_s'))
        if self.pending != choice[1]:
            self.pending, self.capture, self.hits = choice[1], None, 0
        if stamp != self.capture:
            self.capture = stamp
            self.hits += 1
        report.update(confirmation_hits=self.hits, selected_side=choice[1])
        if self.hits < 2:
            report.update(reason='confirm_replacement_corridor', waiting=True)
            return None
        report.update(reason='confirmed_replacement_corridor', waiting=False)
        return choice
