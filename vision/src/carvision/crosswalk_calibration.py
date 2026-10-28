"""Independent, output-free calibration for fixed near/far crosswalk views.

Distances mean vehicle-front to the nearest paper edge. The near-edge feature
is median(y + height - 1) / (image_height - 1) for the actual paper row; it
measures that same edge without assuming a paper length. The legacy top/far
feature additionally requires a declared common paper specification. Each
camera keeps its own measured table; pixels are never appended to the other
camera's table. These validation checks assess declared calibration
evidence, not the physical accuracy of a measurement or the car's braking.

A missing file selects no dual profile (legacy compatibility). A present,
enabled profile must be complete and verified, or loading raises. Disabled
drafts may hold provisional measurements, but cannot supply estimates for
motion. This module reads JSON only; it opens no camera, network or actuator.
"""
from bisect import bisect_left
import copy
from datetime import date
import json
import math
from pathlib import Path
import re


CAMERAS = ('primary', 'secondary')
IMAGE_SIZE = [480, 360]
DISTANCE_REFERENCE = 'vehicle_front_to_nearest_paper_edge_m'
IMAGE_FEATURE = 'paper_row_top_y_normalized'
IMAGE_FEATURE_NEAR = 'paper_row_near_edge_y_normalized'
_EPS = 1e-9
_UNSET = object()


class CalibrationError(ValueError):
    """An explicit dual configuration cannot be used with these blockers."""

    def __init__(self, errors):
        self.errors = list(errors)
        super().__init__('双摄斑马线标定不可用于行驶：' + '；'.join(self.errors))


def _number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        # A JSON/Python integer can be larger than a finite physical float.
        return False


def _text(value):
    return isinstance(value, str) and bool(value.strip())


def _captured_date(value):
    if not isinstance(value, str) or not re.fullmatch(r'\d{4}-\d{2}-\d{2}', value):
        return False
    try:
        date.fromisoformat(value)
    except ValueError:
        return False
    return True


def _camera_report(camera, value, errors, coordinate_key):
    """Return only structurally usable points, preserving provenance per view."""
    prefix = 'cameras.' + camera
    if not isinstance(value, dict):
        errors.append(prefix + ': 缺少独立相机标定')
        return []
    if not _text(value.get('identity')):
        errors.append(prefix + '.identity: 缺少物理摄像头身份')
    image_size = value.get('image_size')
    if (not isinstance(image_size, list) or len(image_size) != 2
            or any(type(n) is not int for n in image_size) or image_size != IMAGE_SIZE):
        errors.append(prefix + '.image_size: 必须为未经裁剪的480×360画面')
    if value.get('verified') is not True:
        errors.append(prefix + '.verified: 当前相机测量尚未核验')
    if not _captured_date(value.get('captured_on')):
        errors.append(prefix + '.captured_on: 需要有效的YYYY-MM-DD采集日期')
    pose = value.get('pose')
    if not isinstance(pose, dict):
        errors.append(prefix + '.pose: 缺少当前固定安装姿态')
    else:
        if pose.get('fixed') is not True or pose.get('verified') is not True:
            errors.append(prefix + '.pose: 需要核验当前固定姿态，旧姿态不能自动继承')
        if not _text(pose.get('description')):
            errors.append(prefix + '.pose.description: 缺少可复核的姿态说明')
        for key in ('pan_us', 'tilt_us'):
            if key in pose and (not _number(pose[key]) or not 500 <= pose[key] <= 2500):
                errors.append(prefix + '.pose.' + key + ': 姿态脉宽须为500–2500微秒')
    references = value.get('references')
    if not isinstance(references, list):
        errors.append(prefix + '.references: 实测点必须是列表')
        return []
    if len(references) < 2:
        errors.append(prefix + '.references: 同一相机至少需要两个独立实测点，单点不能插值')
    reference_errors_before = len(errors)
    points = []
    for index, row in enumerate(references):
        label = prefix + '.references[' + str(index) + ']'
        if not isinstance(row, dict):
            errors.append(label + ': 实测点格式无效')
            continue
        distance, y = row.get('distance_m'), row.get(coordinate_key)
        other_key = ('far_edge_y_normalized' if coordinate_key == 'near_edge_y_normalized'
                     else 'near_edge_y_normalized')
        if other_key in row:
            errors.append(label + ': 近边/远边坐标不能混表或同时填写')
        if coordinate_key == 'near_edge_y_normalized' and row.get('bottom_clipped') is True:
            errors.append(label + ': 近边已被画面裁断，不能作为可测量近边点')
        if not _number(distance) or distance <= 0:
            errors.append(label + '.distance_m: 车头到纸近边距离须为正有限米数')
        if not _number(y) or not 0 < y < 1:
            errors.append(label + '.' + coordinate_key + ': 白块边缘坐标须在0与1之间')
        if row.get('measured') is not True:
            errors.append(label + '.measured: 疑似值或推算值不能代替实测点')
        if not _text(row.get('evidence')):
            errors.append(label + '.evidence: 缺少测量/图像证据定位')
        if _number(distance) and distance > 0 and _number(y) and 0 < y < 1:
            points.append(row)
    if len(points) == len(references) and len(points) >= 2:
        ds = [row['distance_m'] for row in points]
        ys = [row[coordinate_key] for row in points]
        distances_increase = all(a < b for a, b in zip(ds, ds[1:]))
        distances_decrease = all(a > b for a, b in zip(ds, ds[1:]))
        pixels_increase = all(a < b for a, b in zip(ys, ys[1:]))
        pixels_decrease = all(a > b for a, b in zip(ys, ys[1:]))
        if not ((distances_increase and pixels_decrease)
                or (distances_decrease and pixels_increase)):
            errors.append(prefix + '.references: 距离和图像边缘坐标须严格反向单调，不能重复或折返')
    # Do not derive an apparently valid overlap from a surviving subset of a
    # malformed, unmeasured or noninvertible table. Keep the errors in a draft.
    return points if len(errors) == reference_errors_before else []


def validate_profile(data, *, require_ready=False):
    """Return a deep-copied profile with ``ready`` and human-readable errors.

    Required schema: profile_id/enabled/verified, the explicit distance and
    image-feature strings exported above, two cameras (identity/image_size/pose/captured_on/
    verified/references), and overlap (verified/reference_distance_m/evidence).
    Each reference needs distance_m/measured/evidence plus exactly its chosen
    near_edge_y_normalized or far_edge_y_normalized coordinate. Only the old
    far feature needs paper_spec (id/width_m/height_m/same_spec_confirmed).
    IMAGE_FEATURE_NEAR explicitly defines median strip bottom, including the
    last pixel, normalized by image_height-1; a clipped edge is not measurable.

    Disabled incomplete drafts return their blockers. Enabled incomplete
    profiles always raise, including when require_ready is False. Setting
    require_ready=True also rejects disabled drafts. No input is modified.
    """
    if not isinstance(data, dict):
        raise CalibrationError(['profile: 双摄标定须为JSON对象'])
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise CalibrationError(['schema_version: 仅支持独立双摄标定版本1'])
    if type(data.get('enabled')) is not bool:
        raise CalibrationError(['enabled: 必须显式填写true或false'])
    result = copy.deepcopy(data)
    errors = []
    if not _text(result.get('profile_id')):
        errors.append('profile_id: 缺少本次独立标定编号')
    if result.get('verified') is not True:
        errors.append('verified: 整体双摄标定尚未核验')
    if result.get('distance_reference') != DISTANCE_REFERENCE:
        errors.append('distance_reference: 必须声明车头到纸近边距离，不能使用相机距离')
    feature = result.get('image_feature')
    if feature not in (IMAGE_FEATURE, IMAGE_FEATURE_NEAR):
        errors.append('image_feature: 必须显式声明同排白块top远边或median bottom近边坐标')
    coordinate_key = ('near_edge_y_normalized' if feature == IMAGE_FEATURE_NEAR
                      else 'far_edge_y_normalized')
    paper = result.get('paper_spec')
    if feature == IMAGE_FEATURE and not isinstance(paper, dict):
        errors.append('paper_spec: 缺少两摄共同使用的纸张规格')
    elif feature == IMAGE_FEATURE:
        if not _text(paper.get('id')):
            errors.append('paper_spec.id: 缺少纸张/排列规格编号')
        for key in ('width_m', 'height_m'):
            if not _number(paper.get(key)) or paper[key] <= 0:
                errors.append('paper_spec.' + key + ': 纸张尺寸须为正有限米数')
        if paper.get('same_spec_confirmed') is not True:
            errors.append('paper_spec.same_spec_confirmed: 未确认两摄使用同规格纸和相同排列')
    cameras = result.get('cameras')
    if not isinstance(cameras, dict):
        errors.append('cameras: 需要primary和secondary独立标定表')
        cameras = {}
    if set(cameras) - set(CAMERAS):
        errors.append('cameras: 只允许primary和secondary，不接受未知摄像头替代')
    points = {camera: _camera_report(camera, cameras.get(camera), errors, coordinate_key)
              for camera in CAMERAS}
    identities = [cameras.get(camera, {}).get('identity')
                  for camera in CAMERAS if isinstance(cameras.get(camera), dict)]
    if len(identities) == 2 and _text(identities[0]) and identities[0] == identities[1]:
        errors.append('cameras.identity: 两个角色不能使用同一物理摄像头身份')
    overlap_range = None
    common = []
    if all(len(points[camera]) >= 2 for camera in CAMERAS):
        ranges = [(min(p['distance_m'] for p in points[camera]),
                   max(p['distance_m'] for p in points[camera])) for camera in CAMERAS]
        lo, hi = max(r[0] for r in ranges), min(r[1] for r in ranges)
        if hi - lo > _EPS:
            overlap_range = [float(lo), float(hi)]
        else:
            errors.append('overlap.range: 两摄实测范围必须有非零共同覆盖，端点相接也不够')
        common = sorted({float(a['distance_m']) for a in points['primary']
                         for b in points['secondary']
                         if a.get('measured') is True and b.get('measured') is True
                         and abs(a['distance_m'] - b['distance_m']) <= _EPS})
        if not common:
            errors.append('overlap.common_point: 两摄必须各自实测至少一个相同车头距离，不能拼接不同相机的y表')
    else:
        errors.append('overlap.range: 两摄均至少有两点后才能核对交接覆盖')
    overlap = result.get('overlap')
    if not isinstance(overlap, dict):
        errors.append('overlap: 缺少独立重叠确认记录')
    else:
        if overlap.get('verified') is not True:
            errors.append('overlap.verified: 重叠交接尚未核验')
        if not _text(overlap.get('evidence')):
            errors.append('overlap.evidence: 缺少同位置双摄重叠证据')
        value = overlap.get('reference_distance_m')
        if (not _number(value) or not any(abs(value - d) <= _EPS for d in common)
                or overlap_range is None or not overlap_range[0] <= value <= overlap_range[1]):
            errors.append('overlap.reference_distance_m: 重叠确认距离须在双方覆盖内且双方均有对应实测点')
    result['calibration_ready'] = not errors
    result['overlap_ready'] = not errors
    if not result['enabled']:
        errors.append('enabled: 当前为禁用草稿，不可用于运动')
    result['errors'] = errors
    result['ready'] = not errors
    result['overlap_range_m'] = overlap_range
    result['common_measured_distances_m'] = common
    result['reference_key'] = coordinate_key
    if (result['enabled'] or require_ready) and errors:
        raise CalibrationError(errors)
    return result


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON field: ' + key)
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError('non-finite JSON constant: ' + value)


def _finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('non-finite JSON number')
    return number


def load_profile(path, *, require_ready=False):
    """Load optional JSON; only an absent file can select legacy behavior."""
    if path is None:
        if require_ready:
            raise CalibrationError(['profile: 未提供双摄标定文件'])
        return None
    try:
        profile_path = Path(path)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(['profile: 双摄标定文件路径无效']) from exc
    try:
        content = profile_path.read_text(encoding='utf-8-sig')
    except FileNotFoundError:
        # A dangling link is a broken configured file, not an absent optional
        # profile. It must not silently switch a deployment back to legacy.
        if profile_path.is_symlink():
            raise CalibrationError(['profile: 已配置的标定符号链接目标不存在']) from None
        if require_ready:
            raise CalibrationError(['profile: 双摄标定文件不存在']) from None
        return None
    except (OSError, UnicodeError, ValueError) as exc:
        raise CalibrationError(['profile: 无法读取双摄标定文件：' + str(exc)]) from exc
    try:
        data = json.loads(content, object_pairs_hook=_unique_object,
                          parse_constant=_invalid_constant, parse_float=_finite_float)
    except (ValueError, TypeError) as exc:
        raise CalibrationError(['profile: 双摄标定JSON无效']) from exc
    return validate_profile(data, require_ready=require_ready)


def _interpolate(xs, ys, value):
    if value < xs[0] or value > xs[-1]:
        raise CalibrationError(['estimate.range: 超出该摄像头实测范围，禁止外推或借用另一相机标定'])
    index = bisect_left(xs, value)
    if index < len(xs) and value == xs[index]:
        return float(ys[index]), True
    left = index - 1
    fraction = (value - xs[left]) / (xs[index] - xs[left])
    return float(ys[left] + fraction * (ys[index] - ys[left])), False


def _camera(profile, camera):
    # Intentionally validate a fresh copy each time: callers may cache a plain
    # dict and later edit it. A cached/forged ready=True cannot authorize a
    # changed table. Tables contain a handful of points; no file I/O occurs.
    validated = validate_profile(profile, require_ready=True)
    if camera not in CAMERAS:
        raise CalibrationError(['camera: 仅支持primary或secondary'])
    return validated, validated['cameras'][camera]


def estimate_distance(profile, camera, y=_UNSET, *, far_edge_y_normalized=_UNSET,
                      near_edge_y_normalized=_UNSET):
    """Estimate metres from one table without extrapolation or edge conversion.

    ``y`` is the coordinate declared by image_feature. The legacy far keyword
    and the explicit near keyword remain supported, but must match the chosen
    feature and cannot be combined. Plain cached dicts are always revalidated.
    """
    validated, view = _camera(profile, camera)
    coordinate_key = validated['reference_key']
    arguments = [('y', y), ('far_edge_y_normalized', far_edge_y_normalized),
                 ('near_edge_y_normalized', near_edge_y_normalized)]
    supplied = [(key, value) for key, value in arguments if value is not _UNSET]
    if len(supplied) != 1 or supplied[0][0] not in ('y', coordinate_key):
        raise CalibrationError(['estimate.y: 须且只能提供当前标定声明的一个近边或远边坐标'])
    y = supplied[0][1]
    if not _number(y):
        raise CalibrationError(['estimate.y: 图像边缘坐标须为有限数值'])
    rows = sorted(view['references'], key=lambda p: p[coordinate_key])
    distance, measured = _interpolate([r[coordinate_key] for r in rows],
                                      [r['distance_m'] for r in rows], y)
    return {'profile_id': validated['profile_id'], 'camera_id': camera,
            'camera_identity': view['identity'], 'distance_m': distance,
            'image_feature': validated['image_feature'], coordinate_key: float(y),
            'y_normalized': float(y), 'measured': measured,
            'source': 'dual_camera_measured_point' if measured else 'dual_camera_in_range_interpolation',
            'valid_distance_range_m': [min(r['distance_m'] for r in rows), max(r['distance_m'] for r in rows)],
            'extrapolated': False, 'physical_stop_verified': False}


def reference_y(profile, camera, distance_m):
    """Inverse lookup of a trigger inside one camera's independently measured range."""
    validated, view = _camera(profile, camera)
    coordinate_key = validated['reference_key']
    if not _number(distance_m):
        raise CalibrationError(['estimate.distance_m: 参考距离须为有限数值'])
    rows = sorted(view['references'], key=lambda p: p['distance_m'])
    y, measured = _interpolate([r['distance_m'] for r in rows],
                               [r[coordinate_key] for r in rows], distance_m)
    return {'profile_id': validated['profile_id'], 'camera_id': camera,
            'camera_identity': view['identity'], 'distance_m': float(distance_m),
            'image_feature': validated['image_feature'], coordinate_key: y,
            'y_normalized': y, 'measured': measured,
            'source': 'dual_camera_measured_point' if measured else 'dual_camera_in_range_interpolation',
            'extrapolated': False, 'physical_stop_verified': False}
