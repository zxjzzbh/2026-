"""Measured-view selection from crosswalk_observation.CalibratedTrialCamera."""
CAMERAS=("primary","secondary")
class CrosswalkSelection:
    def _danger(self, row):
        return row['candidate'] and (row['near_limit_reached']
            or row['metric_valid'] and row['distance_estimate_m'] <= self.trigger)
    def _select(self, primary_changed):
        self.distance_disagreement = None
        if len(self.rows)==1:
            # One calibrated current view can observe/brake. Do not invent
            # agreement or distance averaging with an unavailable camera.
            self.active_source=next(iter(self.rows))
            self.confirmed_primary_frames,self.confirmation_id=0,None
            if self.active_source=='primary' and self.rows['primary']['near_limit_reached']:
                self.primary_locked=True
            return
        primary, secondary = self.rows['primary'], self.rows['secondary']
        if primary['metric_valid'] and secondary['metric_valid']:
            delta = abs(primary['distance_estimate_m'] - secondary['distance_estimate_m'])
            if delta > .05 + 1e-9:
                # The views are not target-associated or simultaneous. Record
                # their native measurements, never average them or abort for
                # this difference alone. Each selected view keeps its own table.
                self.distance_disagreement = {
                    'threshold_m': .05, 'difference_m': delta,
                    'primary_distance_m': primary['distance_estimate_m'],
                    'secondary_distance_m': secondary['distance_estimate_m'],
                    'capture_skew_ms': abs(primary['captured_s'] - secondary['captured_s']) * 1000,
                    'captured_s': {camera: self.rows[camera]['captured_s'] for camera in CAMERAS},
                    'raw_frame_ids': {camera: self.rows[camera]['raw_frame_id'] for camera in CAMERAS},
                    'target_association_verified': False,
                    'policy': 'one_source_without_distance_averaging',
                }
        if primary['near_limit_reached']:
            self.active_source, self.primary_locked = 'primary', True
            return
        if secondary['near_limit_reached']:
            self.active_source = 'secondary'
            self.confirmed_primary_frames, self.confirmation_id = 0, None
            return
        if self.primary_locked:
            # Primary remains preferred, but cannot hide a secondary stop
            # request. This override is a real observation, not a fused value.
            if self._danger(secondary) and (not self._danger(primary)
                    or secondary['distance_estimate_m'] < primary['distance_estimate_m']):
                self.active_source = 'secondary'
            else:
                self.active_source = ('secondary' if not primary['candidate']
                                      and secondary['candidate'] else 'primary')
            return
        if self._danger(primary) and (not self._danger(secondary)
                or primary['distance_estimate_m'] <= secondary['distance_estimate_m']):
            self.active_source, self.primary_locked = 'primary', True
            return
        if self._danger(secondary):
            self.active_source = 'secondary'
            self.confirmed_primary_frames, self.confirmation_id = 0, None
            return
        self.active_source = 'secondary'
        if primary['candidate'] != secondary['candidate']:
            # One recognized view suffices; the other need not recognize a row.
            self.active_source = 'primary' if primary['candidate'] else 'secondary'
            self.confirmed_primary_frames, self.confirmation_id = 0, None
            return
        lo, hi = self.profile['overlap_range_m']
        measured_overlap = (primary['metric_valid'] and secondary['metric_valid']
            and lo <= primary['distance_estimate_m'] <= hi
            and lo <= secondary['distance_estimate_m'] <= hi
            and primary['distance_estimate_m'] <= .5)
        if not measured_overlap:
            self.confirmed_primary_frames, self.confirmation_id = 0, None
        elif primary_changed and primary['frame_id'] != self.confirmation_id:
            self.confirmation_id = primary['frame_id']
            self.confirmed_primary_frames += 1
            if self.confirmed_primary_frames >= 2:
                self.active_source, self.primary_locked = 'primary', True
