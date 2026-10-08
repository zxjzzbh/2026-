"""Manual ground control has fresh-input, physical-preparation and hard bounds."""
import hashlib
import json

import pytest
from carvision.manual_drive import DriveError, ManualDrive, SimulationBackend
from carvision.pi5_pwm import GuardState, Pi5GroundHeldBackend


class Clock:
    now=0.0
    def __call__(self): return self.now


class PWM:
    def __init__(self): self.pulse=1500; self.calls=[]
    def motion(self,pulse): self.pulse=pulse; self.calls.append(('motion',pulse))
    def neutral(self): self.pulse=1500; self.calls.append(('neutral',))
    def steering_idle(self): pass
    def turn(self,pulse): self.calls.append(('turn',pulse))


class Hardware:
    hardware_output=True
    ground_held_reviewed=True
    combined_trial=True
    steering_center_us=1715
    reverse_available=False
    def __init__(self,clock):
        self.pwm=PWM(); self.guard=GuardState(self.pwm,clock=clock); self.segments=[]
    def check(self): self.guard.request({'action':'check'})
    def motion(self,pulse,seconds):
        self.segments.append((pulse,seconds))
        self.guard.request({'action':'motion','pulse':pulse,'seconds':seconds})
    def neutral(self): self.guard.request({'action':'neutral'})
    def steering(self,pulse): self.guard.request({'action':'steering','pulse':pulse})
    def close(self): self.pwm.neutral()


def ready():
    clock=Clock(); backend=Hardware(clock)
    drive=ManualDrive(backend,clock=clock,settling_s=0,session_s=180,ground_held=True)
    drive.request('enable',{'client':'ground-test','bench_ready':True}); drive.tick()
    return drive,backend,clock


def command(drive,seq,motor='forward',turn='center'):
    return drive.request('command',{'client':'ground-test','sequence':seq,'motor':motor,'steering':turn})


def test_ground_mode_never_reuses_simulation_or_unreviewed_backend_or_unbounded_session():
    with pytest.raises(ValueError): ManualDrive(SimulationBackend(),session_s=180,ground_held=True)
    backend=Hardware(Clock()); backend.ground_held_reviewed=False
    with pytest.raises(ValueError): ManualDrive(backend,session_s=180,ground_held=True)
    with pytest.raises(ValueError): ManualDrive(Hardware(Clock()),session_s=None,ground_held=True)


def test_fresh_commands_hold_low_gear_beyond_three_seconds_and_motor_release_preserves_turn():
    drive,backend,clock=ready()
    for seq in range(1,202):
        clock.now=(seq-1)*.05; command(drive,seq,'forward','left'); drive.tick()
    assert drive.pulse==backend.pwm.pulse==1575
    assert all(pulse==1575 and .02<=seconds<=.2 for pulse,seconds in backend.segments)
    clock.now+=.05; command(drive,202,'stop','left'); drive.tick()
    assert backend.pwm.pulse==1500 and drive.turn=='left'
    clock.now+=.05; command(drive,203,'stop','center'); drive.tick()
    assert drive.turn=='center'


def test_status_checks_cannot_renew_driver_input_or_restart_after_lost_commands():
    drive,backend,clock=ready(); command(drive,1); drive.tick()
    for moment in (.05,.1,.15,.21): clock.now=moment; drive.tick(); drive.status()
    assert drive.mode=='emergency' and backend.pwm.pulse==1500
    with pytest.raises(DriveError): command(drive,2)


def test_hardware_owner_stops_even_when_control_thread_stalls():
    drive,backend,clock=ready(); command(drive,1); drive.tick()
    clock.now=.26; backend.guard.tick()
    assert backend.pwm.pulse==1500 and backend.guard.fault


def test_ground_emergency_latches_and_unreviewed_reverse_never_outputs_reverse():
    drive,backend,clock=ready(); command(drive,1); drive.tick()
    drive.request('emergency',{'stop_source':'keyboard_space'})
    assert backend.pwm.pulse==1500
    with pytest.raises(DriveError): command(drive,2)
    drive.request('reset',{}); drive.request('enable',{'client':'ground-test','bench_ready':True}); drive.tick()
    with pytest.raises(DriveError): command(drive,1,'reverse')
    assert all(pulse==1575 for pulse,_ in backend.segments)


def test_hold_and_session_bounds_cannot_be_extended_by_messages_or_reset():
    drive,backend,clock=ready()
    for seq in range(1,1204):
        clock.now=(seq-1)*.05; command(drive,seq); drive.tick()
    assert drive.blocked and backend.pwm.pulse==1500
    drive.request('emergency',{}); drive.request('reset',{})
    drive.request('enable',{'client':'ground-test','bench_ready':True}); drive.tick()
    assert drive.deadline==180
    for step in range(1204,3601):
        clock.now=step*.05; drive.tick()
    assert drive.mode=='expired' and backend.pwm.pulse==1500


def review():
    return dict(model='Raspberry Pi 5 Model B Rev 1.0',boot_id='current',
        wheels_raised_confirmed_by_user=False,wheels_on_ground_confirmed_by_user=True,
        esc_off_confirmed_by_user=True,ground_held_requested_by_user=True,
        clear_area_confirmed_by_user=True,spotter_can_cut_power_confirmed_by_user=True,
        neutral_physically_verified=True,stop_physically_verified=True,
        keyboard_stop_physically_verified=True,prior_pi5_ground_low_speed_physically_verified=True,
        prior_pi5_steering_physically_verified=True,continuous_ground_driving_enabled=True,
        keyboard_stop_evidence_boot_id='current',esc_backend='rasadapter5a_uart',esc_channel=4,
        steering_backend='rasadapter5a_uart',steering_channel=3,steering_center_us=1715)


def test_missing_ground_preparation_stop_or_spotter_rejects_before_any_output(tmp_path):
    valid=review(); backend=Pi5GroundHeldBackend(tmp_path,tmp_path/'run','current',valid)
    assert not backend.reverse_available
    for field in ('wheels_on_ground_confirmed_by_user','esc_off_confirmed_by_user',
                  'spotter_can_cut_power_confirmed_by_user','keyboard_stop_physically_verified',
                  'prior_pi5_ground_low_speed_physically_verified','prior_pi5_steering_physically_verified'):
        with pytest.raises(ValueError): Pi5GroundHeldBackend(tmp_path,tmp_path/'run','current',{**valid,field:False})
    with pytest.raises(ValueError): Pi5GroundHeldBackend(tmp_path,tmp_path/'run','different-boot',valid)


def test_changed_keyboard_stop_evidence_is_rejected_before_guardian_start(tmp_path,monkeypatch):
    valid=review(); valid['baseline_record']='baseline.json'
    (tmp_path/'baseline.json').write_text('{}')
    raw=json.dumps({'boot_id':'current','keyboard_stop_physically_verified':True,
                   'key_release_during_forward_output_events':[{'motor_pulse_us':1575}]}).encode()
    (tmp_path/'stop.json').write_bytes(raw)
    valid.update(keyboard_stop_evidence_record='stop.json',keyboard_stop_evidence_sha256='0'*64)
    monkeypatch.setattr('carvision.pi5_pwm.inspect_pi5',lambda:{'boot_id':'current','power_flags':0})
    monkeypatch.setattr('carvision.pi5_pwm.subprocess.Popen',lambda *a,**k:pytest.fail('must not start guardian'))
    backend=Pi5GroundHeldBackend(tmp_path,tmp_path/'run','current',valid)
    with pytest.raises(ValueError,match='observation'): backend.open()


def test_long_route_requires_reviewed_limits_and_retains_input_expiry(tmp_path):
    valid=review()
    with pytest.raises(ValueError):
        Pi5GroundHeldBackend(tmp_path,tmp_path/'out','current',{**valid,'manual_session_limit_s':600,'hold_limit_s':600})
    valid.update(manual_session_limit_s=600,hold_limit_s=600,long_route_requested_by_user=True)
    backend=Pi5GroundHeldBackend(tmp_path,tmp_path/'out','current',valid)
    assert backend.manual_session_limit_s==600 and backend.manual_motion_hold_max_s==600
    clock=Clock();hardware=Hardware(clock)
    hardware.manual_session_limit_s=600;hardware.manual_motion_hold_max_s=600
    drive=ManualDrive(hardware,clock=clock,settling_s=0,session_s=600,ground_held=True)
    drive.request('enable',{'client':'ground-test','bench_ready':True});drive.tick()
    for seq in range(1,1402):
        clock.now=(seq-1)*.05;command(drive,seq);drive.tick()
    assert drive.pulse==1575 and drive.status()['motion_hold_max_s']==600
    clock.now+=.21;drive.tick()
    assert drive.mode=='emergency' and hardware.pwm.pulse==1500
    for step in range(1406,12001):clock.now=step*.05;drive.tick()
    assert drive.mode=='expired' and hardware.pwm.pulse==1500


def test_late_enable_keeps_long_route_inside_owner_deadline_and_reset_cannot_extend_it():
    clock=Clock();hardware=Hardware(clock)
    hardware.manual_session_limit_s=600;hardware.manual_motion_hold_max_s=600
    drive=ManualDrive(hardware,clock=clock,settling_s=12,session_s=600,preparation_s=120,ground_held=True)
    for step in range(2400):clock.now=step*.05;drive.tick()
    drive.request('enable',{'client':'ground-test','bench_ready':True})
    original_deadline=drive.deadline
    assert original_deadline<=732 and original_deadline<800
    drive.request('emergency',{});drive.request('reset',{})
    drive.request('enable',{'client':'ground-test','bench_ready':True})
    assert drive.deadline==original_deadline
