"""Reproduce the reported reverse jerk using the real controller and guardian."""
import pytest
from carvision.manual_drive import ManualDrive, DriveError
from test_ground_held_drive import Clock, Hardware, command
from carvision.pi5_pwm import Pi5ContinuousBackend
from test_ground_continuous_drive import continuous_review


def ready():
    clock=Clock();backend=Hardware(clock)
    backend.manual_continuous_reviewed=True
    backend.manual_session_limit_s=backend.manual_motion_hold_max_s=None
    backend.reverse_available=True
    backend.reverse_start_mode='direct_after_neutral'
    drive=ManualDrive(backend,clock=clock,settling_s=0,session_s=None,
                      ground_held=True,ground_continuous=True)
    drive.request('enable',{'client':'ground-test','bench_ready':True});drive.tick()
    backend.pwm.calls.clear()
    return drive,backend,clock


def test_stationary_held_lt_never_inserts_a_brake_pause():
    drive,backend,clock=ready()
    for seq in range(1,201):
        clock.now=(seq-1)*.02
        command(drive,seq,'reverse','center');drive.tick()
        assert backend.pwm.pulse==1300, f'unwanted neutral at {clock.now:.2f}s'
    assert drive.stage=='reverse'
    assert not any(c[0]=='neutral' for c in backend.pwm.calls)
    assert all(pulse==1300 and duration<=.2 for pulse,duration in backend.segments)


def test_repeated_reverse_has_no_artificial_second_start_delay():
    drive,backend,clock=ready()
    command(drive,1,'reverse');drive.tick()
    clock.now=.1;drive.request('stop',{'stop_source':'gamepad_release'})
    assert backend.pwm.pulse==1500
    clock.now=.2;command(drive,2,'stop','center')
    clock.now=.22;command(drive,3,'reverse');drive.tick()
    assert drive.stage=='reverse' and backend.pwm.pulse==1300


def test_forward_to_reverse_waits_at_neutral_without_a_reverse_kick():
    drive,backend,clock=ready()
    command(drive,1,'forward');drive.tick()
    clock.now=.1;drive.request('stop',{'stop_source':'gamepad_release'})
    command(drive,2,'stop','center')
    backend.pwm.calls.clear()
    for seq in range(3,59):
        clock.now=.12+(seq-3)*.02
        command(drive,seq,'reverse','center');drive.tick()
        assert backend.pwm.pulse==1500
    assert not any(c[0]=='motion' for c in backend.pwm.calls)
    clock.now=1.31;command(drive,60,'reverse');drive.tick()
    assert backend.pwm.pulse==1300


def test_idle_time_already_elapsed_is_not_waited_a_second_time():
    drive,backend,clock=ready()
    command(drive,1,'forward');drive.tick()
    clock.now=.1;drive.request('stop',{'stop_source':'gamepad_release'})
    for step in range(1,71):clock.now=.1+step*.02;drive.tick()
    command(drive,2,'stop','center');command(drive,3,'reverse');drive.tick()
    assert drive.stage=='reverse' and backend.pwm.pulse==1300


def test_reverse_straight_start_waits_for_commanded_center_after_a_turn():
    drive,backend,clock=ready()
    drive.steering_us=1750;drive.last_steering_change=0
    backend.pwm.calls.clear()
    for seq in range(1,9):
        clock.now=seq*.025
        command(drive,seq,'reverse','center');drive.tick()
        assert backend.pwm.pulse==1500
    for seq in range(9,30):
        clock.now=seq*.025
        command(drive,seq,'reverse','center');drive.tick()
    assert drive.steering_us==drive.steering_center_us
    assert backend.pwm.pulse==1300
    first_motion=next(i for i,c in enumerate(backend.pwm.calls) if c[0]=='motion')
    assert ('turn',drive.steering_center_us) in backend.pwm.calls[:first_motion]


@pytest.mark.parametrize('action',['stop','emergency'])
def test_reverse_release_or_emergency_is_immediate(action):
    drive,backend,clock=ready()
    command(drive,1,'reverse');drive.tick()
    clock.now=.03;drive.request(action,{'stop_source':'gamepad_release'})
    assert backend.pwm.pulse==1500
    clock.now=.04
    if action=='emergency':
        with pytest.raises(DriveError):command(drive,2,'reverse')
    else:command(drive,2,'reverse');drive.tick()
    assert backend.pwm.pulse==1500


def test_reverse_input_loss_still_latches_at_500ms():
    drive,backend,clock=ready()
    command(drive,1,'reverse');drive.tick()
    for step in range(1,25):clock.now=step*.02;drive.tick()
    assert backend.pwm.pulse==1300
    clock.now=.5;drive.tick()
    assert drive.mode=='emergency' and backend.pwm.pulse==1500


def test_controller_stall_still_trips_independent_guardian():
    drive,backend,clock=ready()
    command(drive,1,'reverse');drive.tick()
    clock.now=.26;backend.guard.tick()
    assert backend.guard.fault and backend.pwm.pulse==1500


def test_direct_profile_requires_the_observed_single_pulse_behavior(tmp_path):
    review={**continuous_review(),'reverse_start_mode':'direct_after_neutral'}
    with pytest.raises(ValueError,match='operator observation'):
        Pi5ContinuousBackend(tmp_path,tmp_path/'out','current',review)
    review['single_pulse_reverse_observed_by_user']=True
    assert Pi5ContinuousBackend(tmp_path,tmp_path/'out','current',review).reverse_start_mode=='direct_after_neutral'
    with pytest.raises(ValueError):
        Pi5ContinuousBackend(tmp_path,tmp_path/'out','current',{**review,'reverse_start_mode':'guess'})


def test_real_backend_establishes_center_before_reverse_and_keeps_it(tmp_path):
    review={**continuous_review(),'reverse_start_mode':'direct_after_neutral',
            'single_pulse_reverse_observed_by_user':True,'steering_center_us':1610}
    backend=Pi5ContinuousBackend(tmp_path,tmp_path/'out','current',review)
    backend.process=object()
    calls=[];backend.rpc=lambda action,**payload:calls.append((action,payload))
    clock=Clock()
    drive=ManualDrive(backend,clock=clock,settling_s=0,session_s=None,
                      ground_held=True,ground_continuous=True)
    drive.request('enable',{'client':'ground-test','bench_ready':True});drive.tick();calls.clear()
    for seq in range(1,31):
        clock.now=(seq-1)*.02
        command(drive,seq,'reverse','center');drive.tick()
    actuator=[c for c in calls if c[0] in ('steering','motion','neutral','steering_idle')]
    assert actuator[0]==('steering',{'pulse':1610})
    assert actuator[1][0]=='motion' and actuator[1][1]['pulse']==1300
    assert all(c[0]=='motion' for c in actuator[1:])
    assert backend.steering_active and drive.steering_us==1610
