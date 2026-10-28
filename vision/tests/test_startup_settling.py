"""A shorter neutral hold must neither queue old input nor weaken stops."""
from pathlib import Path
import pytest
from carvision.manual_drive import ManualDrive, DriveError, CONTINUOUS_STARTUP_SETTLING_S
from test_ground_held_drive import Clock, Hardware, command


def setup():
    clock=Clock();backend=Hardware(clock)
    backend.manual_continuous_reviewed=True
    backend.manual_session_limit_s=backend.manual_motion_hold_max_s=None
    drive=ManualDrive(backend,clock=clock,settling_s=CONTINUOUS_STARTUP_SETTLING_S,
                      session_s=None,ground_held=True,ground_continuous=True)
    drive.request('enable',{'client':'ground-test','bench_ready':True})
    return drive,backend,clock


def test_three_seconds_of_neutral_then_ready_without_queued_motion():
    drive,backend,clock=setup()
    assert drive.status()['startup_settling_s']==3
    for step in range(150):
        clock.now=step*.02;drive.tick()
        assert drive.mode=='settling' and backend.pwm.pulse==1500
        if step in (0,75,149):
            with pytest.raises(DriveError):command(drive,step+1,'forward')
    clock.now=3.;drive.tick()
    assert drive.mode=='enabled' and drive.status()['ready_in_s']==0
    assert backend.pwm.pulse==1500 and backend.segments==[]
    command(drive,200,'forward');drive.tick()
    assert backend.pwm.pulse==1575


def test_emergency_during_short_initialization_never_arms_later():
    drive,backend,clock=setup()
    for step in range(51):clock.now=step*.02;drive.tick()
    drive.request('emergency',{'stop_source':'keyboard_space'})
    for step in range(51,201):clock.now=step*.02;drive.tick()
    assert drive.mode=='emergency' and backend.pwm.pulse==1500
    assert backend.segments==[]
    with pytest.raises(DriveError):command(drive,201,'reverse')


def test_countdown_report_matches_current_profile_without_changing_s3_or_legacy(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1]/'tools'))
    from serve_boot_test import initial_status
    assert initial_status({'ground_continuous_requested_by_user':True})['startup_settling_s']==3
    assert initial_status({'ground_continuous_requested_by_user':True,'steering_pwm_mode':True})['startup_settling_s']==1
    assert initial_status({})['startup_settling_s']==12
