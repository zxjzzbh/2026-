"""One measured parking core for competition and standalone metric entry.

The caller owns arming, camera sampling, command leases and payment. This
module owns the mission and its measured PWM/steering adapter; it opens no I/O.
"""
import copy

from .parking_bay import number, validate
from .parking_control import parking_intent, parking_readiness, curvature_for_pwm, PARKING_STEERING_RATE_US_S
from .parking_mission import ParkingMission
from .steering import CENTER_US
from .runtime_timing import RACE_CONTROL_WINDOW_S


class ParkingTask:
    def __init__(self, parking, cones, *, require_ready=True, max_control_gap_s=RACE_CONTROL_WINDOW_S):
        self.s = validate(parking)
        self.cones = copy.deepcopy(cones)
        self.readiness = parking_readiness({'parking': self.s, 'cones': self.cones})
        if require_ready and not self.readiness['ready']:
            raise ValueError('路径入库缺少：'+'、'.join(self.readiness['missing']))
        self.mission = ParkingMission(self.s,max_control_gap_s=max_control_gap_s)
        self.steering = float(CENTER_US)
        self.control_at = None
        self.localization_epoch = None

    @property
    def selected(self):
        return self.mission.selected

    @property
    def path(self):
        return self.mission.path

    def recover_control(self):
        self.mission.recover_control()
        self.steering = float(CENTER_US)
        self.control_at = None

    def decide(self, world, now):
        if not self.readiness['ready']:
            return {'action': 'stop', 'phase': 'not_ready', 'reason': '停车实测标定未就绪',
                    'missing': list(self.readiness['missing']), 'selected_slot': None,
                    'payment_ready': False, 'hardware_output': False}
        epoch=world.get('localization_epoch') if isinstance(world,dict) else None
        if type(epoch) is int and epoch>=0:
            if self.localization_epoch is not None and epoch<self.localization_epoch:
                raise ValueError('停车定位版本倒退')
            if epoch>0 and epoch!=self.localization_epoch:
                self.recover_control()
            self.localization_epoch=epoch
        return self.mission.step(world, now)

    def intent(self, decision, now, captured_s=None):
        if not self.readiness['ready'] and decision['action'] == 'follow_path':
            raise ValueError('停车实测标定未就绪，禁止路径输出')
        output = parking_intent(decision, self.s)
        if decision['action'] == 'follow_path':
            stamp = now if captured_s is None else captured_s
            if not number(now) or not number(stamp):
                raise ValueError('停车控制需要有效单调时间')
            requested = output['steering_us']
            dt = min(.25, max(0., now-(self.control_at if self.control_at is not None else now-.1)))
            change = PARKING_STEERING_RATE_US_S*dt
            steering = round(self.steering+max(-change,min(change,requested-self.steering)))
            curvature = curvature_for_pwm(steering,self.s)
            rejected = self.mission.check_final_command(curvature,output['command_speed_mps'],now,stamp)
            if rejected is not None:
                target_curvature = output['target_curvature_1pm']
                decision = rejected
                output = parking_intent(decision,self.s)
                output.update(rejected_steering_us=steering,rejected_command_curvature_1pm=curvature,
                              rejected_target_curvature_1pm=target_curvature)
                self.steering = float(CENTER_US)
            else:
                self.steering = float(steering)
                output.update(steering_us=steering, steering_policy='calibrated_path_slew',
                              requested_steering_us=requested,command_curvature_1pm=curvature)
        else:
            self.steering = float(CENTER_US)
        self.control_at = now
        output.update(phase=decision['phase'], selected_side=decision.get('selected_slot'),
                      payment_ready=decision.get('payment_ready', False),
                      diagnostic_only=False, formal_task=True, task_core='metric_parking_v1')
        return output

    def step(self, world, now):
        decision = self.decide(world, now)
        return self.intent(decision, now, world.get('captured_s') if isinstance(world, dict) else None)
