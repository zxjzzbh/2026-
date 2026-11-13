"""Forward-only power requests above the measured motor dead band."""
import math

MIN_FORWARD_US = 1570  # Operator's measured minimum on this vehicle.


def validate_forward(forward):
    if type(forward) is not int or not MIN_FORWARD_US <= forward <= 1625:
        raise ValueError('前进油门仅允许 1570–1625 μs；低于 1570 无法带动车辆')
    return forward


def reduced_pulses(forward):
    validate_forward(forward)
    return sorted({MIN_FORWARD_US, min(1580, round(MIN_FORWARD_US+(forward-MIN_FORWARD_US)*.75))})


def output_request(forward,scale):
    validate_forward(forward)
    if isinstance(scale,bool) or not isinstance(scale,(int,float)) or not math.isfinite(scale):
        raise ValueError('无效的前进动力请求')
    if scale<=0:return 'neutral',1500
    if scale>=1:return 'search',forward
    allowed=[p for p in reduced_pulses(forward) if p<=MIN_FORWARD_US+(forward-MIN_FORWARD_US)*scale+1e-6]
    pulse=max(allowed)
    return 'cone_'+str(pulse),pulse


def with_reduced_power(actuator_settings):
    return {**actuator_settings,'cone_forward_us':reduced_pulses(actuator_settings['pwm']['search_us'])}
