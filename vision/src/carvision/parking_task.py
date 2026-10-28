"""One measured parking core for competition and standalone metric entry.

The caller owns arming, camera sampling, command leases and payment. This
module owns the mission and its measured PWM/steering adapter; it opens no I/O.
"""
import copy

from .parking_bay import number, validate
from .parking_control import parking_intent, parking_readiness, PARKING_STEERING_RATE_US_S
from .parking_mission import ParkingMission
from .steering import CENTER_US, ImagePD, from_offset


class ParkingTask:
    def __init__(self, parking, cones, *, require_ready=True):
        self.s = validate(parking)
        self.cones = copy.deepcopy(cones)
        self.readiness = parking_readiness({'parking': self.s, 'cones': self.cones})
        if require_ready and not self.readiness['ready']:
            raise ValueError('路径入库缺少：'+'、'.join(self.readiness['missing']))
        self.mission = ParkingMission(self.s)
        self.pd = ImagePD(kp=1., kd=.08)
        self.steering = float(CENTER_US)
        self.control_at = None

    @property
    def selected(self):
        return self.mission.selected

    @property
    def path(self):
        return self.mission.path

    def decide(self, world, now):
        if not self.readiness['ready']:
            return {'action': 'stop', 'phase': 'not_ready', 'reason': '停车实测标定未就绪',
                    'missing': list(self.readiness['missing']), 'selected_slot': None,
                    'payment_ready': False, 'hardware_output': False}
        return self.mission.step(world, now)

    def intent(self, decision, now, captured_s=None):
        if not self.readiness['ready'] and decision['action'] == 'follow_path':
            raise ValueError('停车实测标定未就绪，禁止路径输出')
        output = parking_intent(decision, self.s)
        if decision['action'] == 'follow_path':
            stamp = now if captured_s is None else captured_s
            if not number(now) or not number(stamp):
                raise ValueError('停车控制需要有效单调时间')
            signed_ratio = (CENTER_US-output['steering_us'])/260.
            command = self.pd.update(signed_ratio, stamp, context=self.selected)
            requested = from_offset(command)
            rows = self.s['motion_calibration']['curvature_pwm']
            requested = max(min(r['pwm_us'] for r in rows), min(max(r['pwm_us'] for r in rows), requested))
            dt = min(.25, max(0., now-(self.control_at if self.control_at is not None else now-.1)))
            change = PARKING_STEERING_RATE_US_S*dt
            self.steering += max(-change, min(change, requested-self.steering))
            output.update(steering_us=round(self.steering), steering_policy='verified_path_pd',
                          requested_steering_us=requested)
        else:
            self.pd.reset()
            self.steering = float(CENTER_US)
        self.control_at = now
        output.update(phase=decision['phase'], selected_side=decision.get('selected_slot'),
                      payment_ready=decision.get('payment_ready', False),
                      diagnostic_only=False, formal_task=True, task_core='metric_parking_v1')
        return output

    def step(self, world, now):
        decision = self.decide(world, now)
        return self.intent(decision, now, world.get('captured_s') if isinstance(world, dict) else None)
