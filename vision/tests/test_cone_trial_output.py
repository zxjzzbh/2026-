"""Actual output bounds and brake timing, with recorded writes only."""
import pytest

from carvision.brake_output import PulseGuard
from carvision.cone_trial_output import ConePulseGuard,ConeTrialActuator,validate_limits


def settings():
    return {'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':1600,'creep_us':1600,'brake_us':1400},
            'brake_pulse_s':.12,'brake_release_s':.1,'actuator_lease_s':.2}


def command(seq,t,steer=1690,action='search'):
    return {'sequence':seq,'sent_s':t,'steering_us':steer,'action':action,
            'esc_us':{'search':1600,'neutral':1500,'brake':1400}[action]}


def test_actual_cone_pulses_are_transmitted_and_original_guard_stays_narrow():
    writes=[];guard=ConePulseGuard(settings(),lambda *p:writes.append(p),mode='F/B')
    guard.accept(command(0,0),0)
    assert writes[-2:]==[(3,1690),(4,1600)]
    guard.accept(command(1,.05,1550),.05)
    assert writes[-2:]==[(3,1550),(4,1600)]
    old=PulseGuard(settings(),lambda *p:None,mode='F/B',forward_limit_us=1625)
    with pytest.raises(ValueError,match='steering'):old.accept(command(0,0),0)


@pytest.mark.parametrize('pulse',[1549,1691,1751,1430,1790,True,1600.5])
def test_actual_output_never_exceeds_selected_bounds(pulse):
    writes=[];guard=ConePulseGuard(settings(),lambda *p:writes.append(p),mode='F/B')
    with pytest.raises(ValueError):guard.accept(command(0,0,pulse),0)
    assert all(ch in (3,4) and (ch!=3 or value==1610) for ch,value in writes)


def test_watchdog_applies_finite_brake_then_centers_expanded_steering():
    writes=[];guard=ConePulseGuard(settings(),lambda *p:writes.append(p),mode='F/B')
    guard.accept(command(0,0),0);guard.tick(.201)
    assert guard.fault and guard.esc==1400
    guard.tick(.322)
    assert writes[-2:]==[(4,1500),(3,1610)]


def test_repeated_braking_does_not_extend_pulse_and_duplicate_command_stops():
    writes=[];guard=ConePulseGuard(settings(),lambda *p:writes.append(p),mode='F/B')
    guard.accept(command(0,0),0)
    guard.accept(command(1,.05,1690,'brake'),.05)
    guard.accept(command(2,.10,1690,'brake'),.10)
    guard.tick(.171);assert guard.esc==1500
    with pytest.raises(ValueError):guard.accept(command(2,.18),.18)
    assert guard.fault


@pytest.mark.parametrize('right,left',[(1550,1750),(1570,1650),(1560,1690)])
def test_supported_steering_limits(right,left):
    validate_limits(right,left)
    guard=ConePulseGuard(settings(),lambda *p:None,mode='F/B',right_us=right,left_us=left)
    guard.accept(command(0,0,left),0)
    guard.accept(command(1,.05,right),.05)


def test_nonhardware_actuator_reports_actual_command_not_timing_placeholder():
    actuator=ConeTrialActuator(settings(),run=False)
    result=actuator.send({'action':'search','steering_us':1690,'esc_us':1600})
    assert not result['hardware_output'] and result['command']['steering_us']==1690
    assert result['steering_limits_us']==[1550,1690]
    with pytest.raises(ValueError):actuator.send({'action':'search','steering_us':1750,'esc_us':1600})
    assert actuator.closed


def test_invalid_neutral_or_forward_profile_cannot_move_steering():
    writes=[];guard=ConePulseGuard(settings(),lambda *p:writes.append(p),mode='F/B')
    with pytest.raises(ValueError):guard.accept(command(0,0,1690,'neutral'),0)
    assert all(ch!=3 or value==1610 for ch,value in writes)
    bad=settings();bad['pwm']['search_us']=1625
    with pytest.raises(ValueError):ConePulseGuard(bad,lambda *p:None,mode='F/B')
