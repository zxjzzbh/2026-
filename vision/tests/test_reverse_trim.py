"""Reverse-only calibration must not change forward/stop or bypass ownership."""
import pytest
from carvision.manual_drive import DriveError, ManualDrive
from carvision.pi5_pwm import Pi5ContinuousBackend
from test_direct_reverse import ready
from test_ground_held_drive import Clock, command
from test_ground_continuous_drive import continuous_review


def apply(drive,seq,offset,client='ground-test'):
    return drive.request('reverse_trim',{'client':client,'sequence':seq,'offset_us':offset})


def test_parameter_application_does_not_start_motor_or_change_forward_center():
    drive,backend,_=ready()
    original=drive.steering_center_us
    state=apply(drive,1,5)
    assert state['reverse_trim_us']==5
    assert state['reverse_steering_center_us']==original+5
    assert state['steering_center_us']==original
    assert state['motor_direction']=='stop' and backend.segments==[]
    assert not any(c[0] in ('motion','turn') for c in backend.pwm.calls)


@pytest.mark.parametrize('offset',[5,-5,30,-30])
def test_only_reverse_center_uses_the_offset_and_release_restores_base(offset):
    drive,backend,clock=ready();base=drive.steering_center_us
    apply(drive,1,offset)
    for seq in range(2,53):
        clock.now=(seq-2)*.025;command(drive,seq,'reverse','center');drive.tick()
    assert drive.steering_us==base+offset and backend.pwm.pulse==1300
    drive.request('stop',{'stop_source':'gamepad_release'})
    for n in range(1,31):clock.now=1.25+n*.025;drive.tick()
    assert drive.steering_us==base and backend.pwm.pulse==1500
    command(drive,54,'stop','center');command(drive,55,'forward','center');drive.tick()
    assert drive.steering_us==base and backend.pwm.pulse==1575


@pytest.mark.parametrize('turn,target',[('left',1750),('right',1550)])
def test_explicit_reverse_turns_keep_existing_endpoints(turn,target):
    drive,_,clock=ready();apply(drive,1,15)
    for seq in range(2,35):
        clock.now=(seq-2)*.025;command(drive,seq,'reverse',turn);drive.tick()
    assert drive.steering_us==target


@pytest.mark.parametrize('value',[31,-31,True,1.5,'5',None])
def test_invalid_trim_is_rejected_without_changes(value):
    drive,backend,_=ready()
    with pytest.raises(DriveError):apply(drive,1,value)
    assert drive.reverse_trim_us==0 and backend.segments==[]


def test_other_pages_old_sequences_and_moving_adjustments_are_rejected():
    drive,_,_=ready()
    with pytest.raises(DriveError):apply(drive,1,5,'other-client')
    apply(drive,1,5)
    with pytest.raises(DriveError):apply(drive,1,-5)
    command(drive,2,'reverse','center')
    with pytest.raises(DriveError):apply(drive,3,10)
    assert drive.reverse_trim_us==5


def test_trim_does_not_remove_disconnect_or_emergency_protection():
    drive,backend,clock=ready();apply(drive,1,5)
    command(drive,2,'reverse','center')
    for n in range(1,25):clock.now=n*.02;drive.tick()
    clock.now=.5;drive.tick()
    assert drive.mode=='emergency' and backend.pwm.pulse==1500
    with pytest.raises(DriveError):apply(drive,3,10)


def test_trimmed_center_reaches_real_backend_before_reverse_motion(tmp_path):
    review={**continuous_review(),'reverse_start_mode':'direct_after_neutral',
            'single_pulse_reverse_observed_by_user':True,'steering_center_us':1610}
    backend=Pi5ContinuousBackend(tmp_path,tmp_path/'out','current',review)
    backend.process=object();calls=[]
    backend.rpc=lambda action,**payload:calls.append((action,payload))
    clock=Clock();drive=ManualDrive(backend,clock=clock,settling_s=0,session_s=None,
                                   ground_held=True,ground_continuous=True)
    drive.request('enable',{'client':'ground-test','bench_ready':True});drive.tick()
    apply(drive,1,5);calls.clear()
    for seq in range(2,30):
        clock.now=(seq-2)*.025;command(drive,seq,'reverse','center');drive.tick()
    actuator=[c for c in calls if c[0] in ('steering','motion','steering_idle')]
    assert actuator[0]==('steering',{'pulse':1615})
    assert all(c[0]=='motion' and c[1]['pulse']==1300 for c in actuator[1:])


def test_unmeasured_session_trim_is_not_silently_persisted():
    drive,_,_=ready();apply(drive,1,10)
    fresh,_,_=ready()
    assert fresh.reverse_trim_us==0
