"""Actual output bounds and brake timing, with recorded writes only."""
import pytest

from carvision.brake_output import PulseGuard
from carvision.cone_trial_output import ConePulseGuard,ConeTrialActuator,validate_limits


def settings():
    return {'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':1600,'creep_us':1600,'brake_us':1400},
            'brake_pulse_s':.12,'brake_release_s':.1,'actuator_lease_s':.2}


def command(seq,t,steer=1870,action='search'):
    return {'sequence':seq,'sent_s':t,'steering_us':steer,'action':action,
            'esc_us':{'search':1600,'neutral':1500,'brake':1400}[action]}


def test_actual_cone_and_task_guards_share_new_endpoints():
    writes=[];guard=ConePulseGuard(settings(),lambda *p:writes.append(p),mode='F/B')
    guard.accept(command(0,0),0)
    assert writes[-2:]==[(3,1870),(4,1600)]
    guard.accept(command(1,.05,1350),.05)
    assert writes[-2:]==[(3,1350),(4,1600)]
    old=PulseGuard(settings(),lambda *p:None,mode='F/B',forward_limit_us=1625)
    old.accept(command(0,0),0);assert not old.fault


@pytest.mark.parametrize('pulse',[1349,1871,2000,True,1600.5])
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


@pytest.mark.parametrize('right,left',[(1350,1870),(1550,1750),(1570,1650),(1560,1690)])
def test_supported_steering_limits(right,left):
    validate_limits(right,left)
    guard=ConePulseGuard(settings(),lambda *p:None,mode='F/B',right_us=right,left_us=left)
    guard.accept(command(0,0,left),0)
    guard.accept(command(1,.05,right),.05)


def test_nonhardware_actuator_reports_actual_command_not_timing_placeholder():
    actuator=ConeTrialActuator(settings(),run=False)
    result=actuator.send({'action':'search','steering_us':1870,'esc_us':1600})
    assert not result['hardware_output'] and result['command']['steering_us']==1870
    assert result['steering_limits_us']==[1350,1870]
    with pytest.raises(ValueError):actuator.send({'action':'search','steering_us':1871,'esc_us':1600})
    assert actuator.closed


def test_invalid_neutral_or_forward_profile_cannot_move_steering():
    writes=[];guard=ConePulseGuard(settings(),lambda *p:writes.append(p),mode='F/B')
    with pytest.raises(ValueError):guard.accept(command(0,0,1690,'neutral'),0)
    assert all(ch!=3 or value==1610 for ch,value in writes)
    bad=settings();bad['pwm']['search_us']=1625
    with pytest.raises(ValueError):ConePulseGuard(bad,lambda *p:None,mode='F/B')


def test_serial_s3_packets_accept_new_endpoints_without_widening_gimbal_range():
    import struct
    from carvision.rasadapter5 import RasAdapter
    from carvision.brake_output import write_parking_pulse
    packets=[];adapter=RasAdapter.__new__(RasAdapter);adapter.send=lambda fn,data:packets.append((fn,data))
    for pulse in (1350,1610,1870):write_parking_pulse(adapter,3,pulse)
    assert [struct.unpack('<BHBBH',data) for fn,data in packets]==[(1,20,1,3,p) for p in (1350,1610,1870)]
    assert all(fn==4 for fn,_ in packets)
    for channel in (1,2):
        with pytest.raises(ValueError):adapter.set_position(channel,1350)
    with pytest.raises(ValueError):adapter.set_position(3,1871)


def test_selected_narrower_range_is_still_enforced():
    guard=ConePulseGuard(settings(),lambda *p:None,mode='F/B',right_us=1550,left_us=1690)
    with pytest.raises(ValueError):guard.accept(command(0,0,1870),0)
