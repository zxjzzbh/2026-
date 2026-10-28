"""Shared metric parking -> bounded PWM adapter, without hardware access.

Only explicitly verified, monotonic measurement tables are interpolated.
Symmetric endpoints do NOT imply symmetric angles or a linear curvature model.
The independent actuator retains its own heartbeat, lease and finite F/B brake.
"""
import copy

import numpy as np

from .parking_bay import number, readiness, validate
from .steering import CENTER_US, LEFT_US, RIGHT_US, radius_settings

PARKING_STEERING_RATE_US_S = 600.


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


def parking_readiness(bundle):
    """Explain existing gates; never manufacture calibration or verified flags."""
    missing = parking_blockers(bundle)
    requirements = {
        '第二路固定姿态的近场地面标定': ('camera', '在当前相机安装姿态和480×360图像下测量地面点，并用独立点核验投影误差'),
        '停车区/入口遮挡识别的现场验证': ('perception', '分别录制左/右挡板、空闲半区、遮挡与反光场景，核对检测与占用结果'),
        '实测转弯半径': ('motion', '在当前左右舵机端点分别低速转弯，测量可重复达到的半径'),
        '当前低速的实测制动距离': ('motion', '以停车接近速度重复制动，记录最长停车距离和指令响应延迟'),
        '持续位姿反馈的误差上限': ('localization', '沿已测距离和角度运行，用独立测量核对连续位姿误差'),
        '第二路当前固定姿态': ('camera', '核对相机固定姿态、分辨率及有效投影区域与当前标定一致'),
        '当前端点转向实测': ('motion', '核验1350/1610/1870输出对应的实际转向方向及响应'),
        '连续位姿反馈有效性': ('localization', '验证运动、静止、遮挡与丢帧情况下的位姿反馈和失效判定'),
        '1350右转半径': ('motion', '测量1350输出的右转半径，使用正厘米值'),
        '1870左转半径': ('motion', '测量1870输出的左转半径，使用负厘米值'),
        '光流位姿误差上限': ('localization', '在实际停车地面与距离范围内测量光流累计误差上限'),
        '光流有效行程': ('localization', '记录光流误差仍满足预算的最长连续行程'),
        '停车速度/PWM及曲率/PWM实测表尚未验证': ('motion', '记录低速档实际速度和左右中间转向档曲率，注明原始实测记录来源'),
    }
    labels = {'camera': '相机与地面标定', 'perception': '停车区与挡板识别',
              'motion': '转向、速度与制动实测', 'localization': '连续位姿反馈',
              'consistency': '几何与配置一致性'}
    items = []
    for reason in missing:
        group, measurement = requirements.get(reason, ('consistency', '核对配置与已记录实测数据，修复报告中的不一致'))
        items.append({'reason': reason, 'group': group, 'requires_measurement': group != 'consistency',
                      'required_evidence': measurement})
    groups = [{'id': key, 'label': label, 'ready': not any(x['group'] == key for x in items),
               'items': [x for x in items if x['group'] == key]}
              for key, label in labels.items()]
    return {'ready': not missing, 'missing': missing, 'groups': groups, 'items': items,
            'hardware_output_enabled': False, 'calibration_modified': False}


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
        from .parking_task import ParkingTask
        self.task = ParkingTask(parking, cone)
        self.s = self.task.s
        self.mission = self.task.mission
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
        output = self.task.step(world, now)
        decision = output['parking_decision']
        self.phase = 'done' if decision['phase'] == 'parked' else decision['phase']
        output.update(phase=self.phase, selected_side=decision['selected_slot'],
                      field_verified=False, whole_vehicle_inside_verified=decision.get('four_wheels_inside', False),
                      actual_pose_not_verified=False, physical_stop_verified=decision['phase']=='parked',
                      payment_ready=decision['payment_ready'])
        return output
