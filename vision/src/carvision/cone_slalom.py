"""Ordered, alternating pass evidence from measured poses; no hardware output."""
import copy
import math


class SlalomProgress:
    def __init__(self, settings, cones, sides, pose, road_heading, captured_s):
        self.s = settings
        self.cones = copy.deepcopy(cones)
        self.sides = dict(sides)
        self.heading = road_heading
        self.previous = dict(pose)
        self.last_t = captured_s
        self.gates = []
        self.passed = []
        self.fault = None

    def coordinates(self, pose, cone):
        dx, dy = pose['x_m'] - cone['x_m'], pose['y_m'] - cone['y_m']
        co, si = math.cos(self.heading), math.sin(self.heading)
        return co * dx - si * dy, si * dx + co * dy

    def update(self, pose, captured_s, observed_world):
        if self.fault:
            return
        dt = captured_s - self.last_t
        travel = math.hypot(pose['x_m'] - self.previous['x_m'], pose['y_m'] - self.previous['y_m'])
        error = max(pose['error_bound_m'], self.previous['error_bound_m'])
        speed = self.s.get('driving', {}).get('max_speed_mps', .2)
        if not 0 < dt <= self.s['max_frame_age_s'] or travel > 1.5 * speed * dt + 2 * error:
            self.fault = 'slalom_pose_discontinuity'
            return
        # Fixed cones and a fixed road frame prevent camera yaw / ID reacquisition
        # from swapping the meaning of left and right midway through the run.
        tolerance = 2 * (self.s['ground']['max_error_m'] + self.s['motion']['maximum_tracking_error_m'])
        anchors = {c['id']: c for c in self.cones}
        for key, current in observed_world.items():
            if key not in anchors:
                self.fault = 'slalom_unexpected_cone'
                return
            anchor = anchors[key]
            if math.hypot(current['x_m'] - anchor['x_m'], current['y_m'] - anchor['y_m']) > tolerance:
                self.fault = 'slalom_cone_position_changed'
                return
        g = self.s['geometry']
        margin = (g['clearance_m'] + self.s['motion']['maximum_tracking_error_m']
                  + self.s['ground']['max_error_m'] + .02)
        for index, cone in enumerate(self.cones):
            key = cone['id']
            before_x, before_y = self.coordinates(self.previous, cone)
            x, y = self.coordinates(pose, cone)
            sign = -1 if self.sides[key] == 'left' else 1
            if before_y < 0 <= y and key not in self.gates:
                crossing_x = before_x + (x - before_x) * (-before_y / (y - before_y))
                if index != len(self.gates):
                    self.fault = 'slalom_wrong_order'
                    return
                if sign * crossing_x < g['vehicle_width_m'] / 2 + cone['radius_m'] + margin:
                    self.fault = 'slalom_wrong_side_or_clearance'
                    return
                self.gates.append(key)
            relative_yaw = pose['yaw_rad'] - self.heading
            rear = g['rear_overhang_m'] * math.cos(relative_yaw) + g['vehicle_width_m'] / 2 * abs(math.sin(relative_yaw))
            front = g['rear_axle_to_front_m'] * math.cos(relative_yaw) + g['vehicle_width_m'] / 2 * abs(math.sin(relative_yaw))
            # Do not switch to the other side while the body still overlaps a cone.
            if -front - cone['radius_m'] - margin <= y <= rear + cone['radius_m'] + margin:
                if sign * x <= cone['radius_m'] + margin:
                    self.fault = 'slalom_wrong_side_or_clearance'
                    return
            if y > rear + cone['radius_m'] + margin and key not in self.passed:
                if key not in self.gates or index != len(self.passed):
                    self.fault = 'slalom_pass_without_ordered_side_evidence'
                    return
                self.passed.append(key)
        self.previous = dict(pose)
        self.last_t = captured_s

    def status(self):
        return {'required': True,
                'order': [{'id': c['id'], 'side': self.sides[c['id']]} for c in self.cones],
                'gate_verified_ids': list(self.gates), 'passed_ids': list(self.passed),
                'complete': len(self.passed) == 2 and self.fault is None,
                'fault': self.fault}
