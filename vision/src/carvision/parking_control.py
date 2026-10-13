"""Shared metric parking -> bounded PWM adapter, without hardware access.

Only explicitly verified, monotonic measurement tables are interpolated.
Symmetric endpoints do NOT imply symmetric angles or a linear curvature model.
The independent actuator retains its own heartbeat, lease and finite F/B brake.
"""
import copy

import numpy as np

from .parking_bay import number, readiness, validate
from .parking_mission import ParkingMission
from .steering import CENTER_US, LEFT_US, RIGHT_US, radius_settings


def calibration_tables(settings):
    calibration = settings.get('motion_calibration') or {}
    if not isinstance(calibration, dict) or calibration.get('verified') is not True:
        raise ValueError('停车速度/PWM及曲率/PWM实测表尚未验证')
    if not isinstance(calibration.get('source'), str) or not calibration['source'].strip():
        raise ValueError('停车运动标定缺少实测记录来源')
    tables = []
    for name, key in (('speed_pwm', 'speed_mps'), ('curvature_pwm', 'curvature_1pm')):
        rows = calibration.get(name)
        if not isinstance(rows, list) or not (2 if name == 'speed_pwm' else 3)<=len(rows)<=32:
            raise ValueError('停车运动标定表缺失：'+name)
        if any(not isinstance(r, dict) or not number(r.get(key)) or type(r.get('pwm_us')) is not int for r in rows):
            raise ValueError('停车运动标定表格式无效：'+name)
        values = [r[key] for r in rows]
        pulses = [r['pwm_us'] for r in rows]
        if any(b <= a for a, b in zip(values, values[1:])):
            raise ValueError('停车运动标定横坐标必须严格递增：'+name)
        if name == 'speed_pwm':
            if (any(not 1501 <= v <= 1625 for v in pulses) or values[0] <= 0
                    or values[0] != settings['final_speed_mps'] or values[-1] != settings['max_speed_mps']
                    or any(b <= a for a, b in zip(pulses, pulses[1:]))):
                raise ValueError('停车速度表应覆盖末段/接近速度，PWM递增且在1501–1625内')
        else:
            if (not values[0] < 0 < values[-1] or any(not RIGHT_US <= v <= LEFT_US for v in pulses)
                    or any(b >= a for a, b in zip(pulses, pulses[1:]))
                    or not any(r[key] == 0 and r['pwm_us'] == CENTER_US for r in rows)):
                raise ValueError('停车曲率表须左负右正、PWM递减，并包含0曲率/1610中位')
        tables.append((values, pulses))
    return tables


def parking_blockers(bundle):
    s = validate(bundle['parking'])
    missing = list(readiness(s)['missing'])
    d = radius_settings(bundle['cones'].get('driving', {}))
    for key, label in (('fixed_camera_confirmed', '第二路当前固定姿态'),
                       ('steering_verified', '当前端点转向实测'),
                       ('odometry_verified', '连续位姿反馈有效性')):
        if d.get(key) is not True:
            missing.append(label)
    for key, label, sign in (('steering_radius_right_cm', '1350右转半径', 1),
                             ('steering_radius_left_cm', '1870左转半径', -1),
                             ('visual_error_bound_m', '光流位姿误差上限', 1),
                             ('visual_verified_distance_m', '光流有效行程', 1)):
        if not number(d.get(key)) or d[key]*sign <= 0:
            missing.append(label)
    if (number(d.get('visual_error_bound_m')) and number(s.get('maximum_pose_error_m'))
            and d['visual_error_bound_m'] > s['maximum_pose_error_m']):
        missing.append('光流误差大于停车位姿预算')
    try:
        tables = calibration_tables(s)
        radius=s.get('minimum_turn_radius_m')
        if number(radius) and 1/radius > min(-tables[1][0][0],tables[1][0][-1])+1e-9:
            missing.append('规划转弯半径超出左右曲率表共同覆盖范围')
        radii = (d.get('steering_radius_left_cm'), d.get('steering_radius_right_cm'))
        if all(number(v) and v != 0 for v in radii):
            values = tables[1][0]
            if values[0] < 100/radii[0]-1e-9 or values[-1] > 100/radii[1]+1e-9:
                missing.append('曲率表超出当前端点实测能力')
    except ValueError as exc:
        missing.append(str(exc))
    return list(dict.fromkeys(missing))


def parking_intent(decision, settings):
    """Never substitute an old fixed throttle for the requested metric speed."""
    output = {'action': 'neutral', 'esc_us': 1500, 'steering_us': CENTER_US,
              'reason': decision['reason'], 'parking_decision': copy.deepcopy(decision),
              'hardware_output': False}
    if decision['action'] == 'follow_path':
        speed, curvature = decision['target_speed_mps'], decision['target_curvature_1pm']
        if not number(speed) or not settings['final_speed_mps']<=speed<=settings['max_speed_mps']:
            raise ValueError('停车目标速度超出配置上限或末段低速')
        tables = calibration_tables(settings)
        for value, (xs, ys) in zip((speed, curvature), tables):
            if not number(value) or not xs[0]-1e-9 <= value <= xs[-1]+1e-9:
                raise ValueError('入位指令超出实测速度或曲率表，禁止外推')
        # Select a measured speed at or below the target; no assumed PWM/speed
        # interpolation through the ESC's start dead zone. Steering uses the
        # verified intermediate curvature samples.
        index=max(i for i,v in enumerate(tables[0][0]) if v<=speed+1e-9)
        throttle=tables[0][1][index]
        steering=round(float(np.interp(curvature,*tables[1])))
        output.update(action='parking_'+str(throttle), esc_us=throttle, steering_us=steering,
                      target_speed_mps=speed, command_speed_mps=tables[0][0][index],
                      target_curvature_1pm=curvature)
    elif decision['action'] == 'brake':
        output.update(action='brake', esc_us=1400)
    return output


class MetricParkingController:
    """Same mission for standalone entry and the full race; explicit start only."""
    def __init__(self, parking, cone):
        self.s = validate(parking)
        missing = parking_blockers({'parking': self.s, 'cones': cone})
        if missing:
            raise ValueError('路径入库缺少：'+'、'.join(missing))
        self.mission = ParkingMission(self.s)
        self.started = None
        self.phase = 'arming'

    def step(self, sample, now):
        if self.started is None:
            self.started = now
        if self.phase in ('done', 'fault'):
            return {'action': 'neutral', 'esc_us': 1500, 'steering_us': CENTER_US,
                    'phase': self.phase, 'reason': '本轮已结束', 'hardware_output': False}
        if now-self.started < 3.:
            return {'action': 'neutral', 'esc_us': 1500, 'steering_us': CENTER_US,
                    'phase': 'arming', 'reason': '保持中位初始化3秒', 'hardware_output': False}
        world = sample.get('parking_world')
        if not isinstance(world, dict):
            self.phase = 'fault'
            return {'action': 'brake' if self.mission.selected else 'neutral',
                    'esc_us': 1400 if self.mission.selected else 1500, 'steering_us': CENTER_US,
                    'phase': self.phase, 'reason': '没有新鲜米制车位/位姿反馈，结束本轮重新准备',
                    'hardware_output': False}
        decision = self.mission.step(world, now)
        output = parking_intent(decision, self.s)
        self.phase = 'done' if decision['phase'] == 'parked' else decision['phase']
        output.update(phase=self.phase, selected_side=decision['selected_slot'],
                      field_verified=False, whole_vehicle_inside_verified=decision.get('four_wheels_inside', False),
                      actual_pose_not_verified=False, physical_stop_verified=decision['phase']=='parked',
                      payment_ready=decision['payment_ready'])
        return output
