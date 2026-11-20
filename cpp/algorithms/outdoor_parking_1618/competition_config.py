"""读取、检查和保存比赛参数；图像中的停车位置需要在实车上标定。"""

import json
import math
import os
from pathlib import Path


DEFAULT_PATH = Path(__file__).resolve().parent / 'competition_config.json'
POSE_KEYS = (
    'yaw_gpio',
    'pitch_gpio',
    'yaw_us',
    'pitch_us',
    'frame_width',
    'frame_height',
    'camera_index',
)


def validate_camera_pose(pose, calibrated=None):
    """检查保存的云台和相机参数；提供标定记录时也核对该记录，不一致时报错。"""
    import config as C

    actual = dict(
        yaw_gpio=C.YAW_GPIO,
        pitch_gpio=C.PITCH_GPIO,
        frame_width=C.CAMERA_WIDTH,
        frame_height=C.CAMERA_HEIGHT,
        camera_index=C.CAMERA_INDEX,
    )
    for key, value in actual.items():
        if pose.get(key) != value:
            raise ValueError('实际相机/GPIO配置与camera_pose.json不一致：' + key)
    if calibrated is not None:
        if any(pose.get(key) != calibrated.get(key) for key in POSE_KEYS):
            raise ValueError(
                '云台/相机与红绿灯停车标定不一致，请重新运行competition_tune.py'
            )


def defaults():
    """返回一份新的默认比赛配置；位置标定值初始为空，不能直接用于行驶。"""
    return {
        'version': 1,
        'school': '',
        'team': '',
        'announcement_file': '/home/pi/src26/zebra/nihao_nihao.wav',
        'announcement_note': '暂用你好你好；正式比赛替换为学校、队名和加油词',
        'expected_cones': 2,
        'traffic': {
            'roi': [0.10, 0.00, 0.90, 0.60],
            'housing_v_max': 95,
            'aspect_min': 2.3,
            'aspect_max': 5.5,
            'min_width_px': 18,
            'min_height_px': 6,
            'min_dark_fraction': 0.25,
            's_min': 100,
            'v_min': 150,
            'red_h': [[0, 12], [165, 179]],
            'yellow_h': [[15, 38]],
            'green_h': [[40, 95]],
            'min_lit_pixels': 4,
            'min_lit_fraction': 0.025,
            'confirm_frames': 3,
            'green_frames': 4,
            'approach_width': None,
            'stop_width': None,
            'approach_duty': 12600,
            'wait_timeout': 18.0,
            'lost_seconds': 0.40,
            'minimum_stop_seconds': 0.50,
            'calibrated': False,
            'position_note': '灯壳宽/图宽；在触发传感器后的1米停车区内标定制动点，不能从灯尺寸直接当作米数',
        },
        'parking': {
            'roi': [0.03, 0.30, 0.97, 0.995],
            'yellow_low': [18, 90, 100],
            'yellow_high': [40, 255, 255],
            'blue_low': [90, 80, 60],
            'blue_high': [124, 255, 255],
            'line_min_width': 0.25,
            'line_aspect_min': 3.0,
            'board_min_area': 0.006,
            'confirm_frames': 3,
            'entry_pass_y': None,
            'stop_y': None,
            'calibrated': False,
            'duty': 12600,
            'front_x': 0.47,
            'kp_us': 300.0,
            'max_turn_us': 90.0,
            'lost_seconds': 0.40,
            'max_seconds': 30.0,
            'end_max_jump': 0.10,
            'position_note': '入口黄线接近画面底部但仍可见时采entry；车位远端黄线到达预定制动点时采stop；需实测四轮入位且不越界',
        },
        'camera_pose': None,
    }


def _merge(base, values, path=''):
    """把文件中的配置逐层写入默认配置；遇到未知参数或错误层级时报错。"""
    for key, value in values.items():
        if key not in base:
            raise ValueError('未知配置项: ' + path + key)
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError('配置应为对象: ' + path + key)
            _merge(base[key], value, path + key + '.')
        else:
            base[key] = value


def load(path=DEFAULT_PATH):
    """读取指定文件并补齐默认参数，检查后返回配置；文件不存在时使用默认值。"""
    cfg = defaults()
    if Path(path).exists():
        _merge(cfg, json.loads(Path(path).read_text(encoding='utf-8')))
    validate(cfg)
    return cfg


def validate(cfg, drive=False):
    """检查配置的数值和标定条件；准备行驶时还要求两项位置标定都已完成。"""

    def finite(value):
        """递归检查一个配置值及其子项，拒绝无穷大和无效数字。"""
        if isinstance(value, dict):
            for item in value.values():
                finite(item)
        elif isinstance(value, (list, tuple)):
            for item in value:
                finite(item)
        elif isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError('配置数值必须有限')

    finite(cfg)
    if (
        not isinstance(cfg['expected_cones'], int)
        or not 1 <= cfg['expected_cones'] <= 10
    ):
        raise ValueError('expected_cones需为1到10的整数')
    if cfg['camera_pose'] is not None and not isinstance(cfg['camera_pose'], dict):
        raise ValueError('camera_pose需为对象或null')
    for name in ('traffic', 'parking'):
        section = cfg[name]
        x0, y0, x1, y1 = section['roi']
        if not (0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1):
            raise ValueError(name + ' ROI无效')
        if (
            not isinstance(section['confirm_frames'], int)
            or not 1 <= section['confirm_frames'] <= 30
        ):
            raise ValueError(name + '确认帧数无效')
        if not 0 < section['lost_seconds'] <= 2:
            raise ValueError(name + '.lost_seconds无效')

    traffic = cfg['traffic']
    parking = cfg['parking']
    if (
        not 1 <= traffic['green_frames'] <= 30
        or int(traffic['green_frames']) != traffic['green_frames']
    ):
        raise ValueError('绿灯确认帧数无效')
    if (
        not 0 < traffic['aspect_min'] < traffic['aspect_max']
        or traffic['min_width_px'] < 3
        or traffic['min_height_px'] < 3
    ):
        raise ValueError('灯壳尺寸范围无效')
    if (
        not 0 < traffic['wait_timeout'] < 20
        or not 0 <= traffic['minimum_stop_seconds'] < traffic['wait_timeout']
    ):
        raise ValueError('红灯等待超时需小于20秒，且不能超时放行')
    for key in ('s_min', 'v_min', 'housing_v_max'):
        if not 0 <= traffic[key] <= 255:
            raise ValueError('HSV范围无效')
    for key in ('red_h', 'yellow_h', 'green_h'):
        if not traffic[key] or any(
            not 0 <= low <= high <= 179 for low, high in traffic[key]
        ):
            raise ValueError('色相范围无效')
    if (
        not 0 < traffic['min_lit_fraction'] < 1
        or traffic['min_lit_pixels'] < 1
        or not 0 < traffic['min_dark_fraction'] < 1
    ):
        raise ValueError('灯亮度面积范围无效')
    for low, high in (('yellow_low', 'yellow_high'), ('blue_low', 'blue_high')):
        if (
            len(parking[low]) != 3
            or len(parking[high]) != 3
            or any(
                not 0 <= lower <= upper <= maximum
                for lower, upper, maximum in zip(
                    parking[low], parking[high], (179, 255, 255)
                )
            )
        ):
            raise ValueError('停车颜色范围无效')
    if (
        not 0 < parking['line_min_width'] < 1
        or parking['line_aspect_min'] <= 1
        or not 0 < parking['board_min_area'] < 1
    ):
        raise ValueError('停车形状条件无效')
    if (
        not 0.15 <= parking['front_x'] <= 0.85
        or not 0 < parking['max_turn_us'] <= 130
        or parking['kp_us'] <= 0
        or parking['max_seconds'] <= 0
        or not 0 < parking['end_max_jump'] <= 0.2
    ):
        raise ValueError('停车控制参数无效')
    if not all(
        12000 < duty <= 16400 for duty in (traffic['approach_duty'], parking['duty'])
    ):
        raise ValueError('行驶PWM需在12000和16400之间')
    if (
        traffic['calibrated']
        or traffic['approach_width'] is not None
        or traffic['stop_width'] is not None
    ):
        approach, stop = traffic['approach_width'], traffic['stop_width']
        if approach is not None and not 0 < approach < 1:
            raise ValueError('灯接近宽度无效')
        if stop is not None and not 0 < stop < 1:
            raise ValueError('灯停车宽度无效')
        if approach is not None and stop is not None and not approach < stop:
            raise ValueError('灯接近宽度必须小于制动宽度')
        if traffic['calibrated'] and (approach is None or stop is None):
            raise ValueError('红绿灯缺少位置标定')
    for key in ('entry_pass_y', 'stop_y'):
        if parking[key] is not None and not 0 < parking[key] < 1:
            raise ValueError('停车位置比例无效')
        if parking['calibrated'] and parking[key] is None:
            raise ValueError('停车区缺少位置标定')
    if drive and not (traffic['calibrated'] and parking['calibrated']):
        raise ValueError(
            '先运行competition_tune.py完成红绿灯和停车区位置标定，再启用--drive'
        )


def save(cfg, path=DEFAULT_PATH):
    """检查配置后写入临时文件，再替换目标文件，并确保文件内容已交给磁盘保存。"""
    validate(cfg)
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    with temp.open('w', encoding='utf-8') as output:
        output.write(json.dumps(cfg, ensure_ascii=False, indent=2) + '\n')
        output.flush()
        os.fsync(output.fileno())
    temp.replace(path)
    if os.name == 'posix':
        directory = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
