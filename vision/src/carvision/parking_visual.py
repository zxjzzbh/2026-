"""Validated visual-trial settings; image rows are never metric calibration."""
import copy
import math


DEFAULTS = {
    'slow_forward_us': 1550,
    'seek_timeout_s': 5.,
    'max_run_s': 30.,
    'boundary_grace_s': .25,
    'boundary_wait_s': 1.2,
    'countersteer_limit_us': 80,
    'slow_line_y_normalized': None,
    'stop_line_y_normalized': None,
}


def validate_visual_options(value=None):
    if value is None:
        value = {}
    if not isinstance(value, dict) or set(value)-set(DEFAULTS):
        raise ValueError('视觉停车参数格式无效')
    result = {**DEFAULTS, **copy.deepcopy(value)}
    for key, lo, hi in (
        ('slow_forward_us', 1501, 1625), ('seek_timeout_s', 1., 10.),
        ('max_run_s', 5., 60.), ('boundary_grace_s', .05, .25),
        ('boundary_wait_s', .4, 2.), ('countersteer_limit_us', 20, 100),
    ):
        number = result[key]
        if type(number) not in (int, float) or not math.isfinite(number) or not lo <= number <= hi:
            raise ValueError('视觉停车参数越界：'+key)
    for key in ('slow_forward_us', 'countersteer_limit_us'):
        if type(result[key]) is not int:
            raise ValueError('视觉停车参数须为整数：'+key)
    if result['seek_timeout_s'] > result['max_run_s']:
        raise ValueError('寻位时限不能超过总入位时限')
    slow, stop = (result[key] for key in ('slow_line_y_normalized', 'stop_line_y_normalized'))
    if (slow is None) != (stop is None):
        raise ValueError('自定义减速线和停止线须一起设置或一起留空')
    if slow is not None:
        if (any(type(x) not in (int, float) or not math.isfinite(x) for x in (slow, stop))
                or not .3 <= slow <= stop-.02 or not stop <= .95):
            raise ValueError('视觉减速线须在停止线之前至少2%画面高度，范围30%–95%')
    return result


def resolve_visual_options(value, brake, forward_us):
    result = validate_visual_options(value)
    # A lower operator-selected approach setting remains the absolute ceiling.
    result['slow_forward_us'] = min(forward_us, result['slow_forward_us'])
    if forward_us>1501 and result['slow_forward_us']>=forward_us:
        result['slow_forward_us']=max(1501,forward_us-10)
    custom = result['stop_line_y_normalized'] is not None
    if not custom:
        result['slow_line_y_normalized'] = brake['references'][0]['far_edge_y_normalized']
        result['stop_line_y_normalized'] = brake['references'][1]['far_edge_y_normalized']
    if result['slow_line_y_normalized'] >= result['stop_line_y_normalized']:
        raise ValueError('停车图像减速线必须早于停止线')
    result['reference_source'] = 'operator_parking_image_reference' if custom else 'legacy_secondary_reference_transfer'
    result['reference_transfer_verified'] = False
    return result
