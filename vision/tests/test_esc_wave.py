import json
import struct

import pytest

from carvision.esc_wave import DmaEsc
from carvision.esc_load_probe import LoadProbeEsc
from carvision.manual_hardware import validate_state


class Clock:
    now = 0
    def __call__(self): return self.now


class Driver(DmaEsc):
    def __init__(self):
        self.time = Clock()
        super().__init__(8890, clock=self.time, sleep=lambda s: None)
        self.calls = []
        self.next_id = 0
        self.connection = object()
    def command(self, cmd, p1=0, p2=0, data=b''):
        self.calls.append((cmd, p1, p2, data))
        if cmd == 49:
            self.next_id += 1
            return self.next_id-1
        return 1 if cmd == 32 else 0
    def close(self): self.connection = None


class ProbeDriver(LoadProbeEsc, Driver):
    pass


def test_load_probe_queues_neutral_without_python_and_inherits_verified_stop():
    d = ProbeDriver(); d.prepare()
    waves = [struct.unpack('<6I', x[3]) for x in d.calls if x[0] == 28]
    assert waves[-1] == (8192, 0, 1600, 0, 8192, 18400)
    d.start_brief_command(1600, .8)
    assert d.calls[-1] == (93, 0, 0, bytes((255, 0, d.waves[1600], 255, 1, 40, 0,
                                          255, 0, d.waves[1500], 255, 1, 184, 11)))
    for method in ('_cancel', '_block', 'neutral', 'check', 'shutdown'):
        assert getattr(LoadProbeEsc, method) is getattr(DmaEsc, method)
    d.neutral(); d.time.now = 46; d.check()
    assert d.calls[-1][3] == bytes((255, 0, d.waves[1500], 255, 1, 184, 11))
    assert d.shutdown() == [] and (0, 13, 0, b'') in d.calls


@pytest.mark.parametrize('pulse,seconds', [(1625,.8),(2000,.8),(1600.0,.8),
    (1600,True),(1600,.801),(1600,.001),(1600,float('nan'))])
def test_load_probe_rejects_any_larger_or_unbounded_step(pulse, seconds):
    d = ProbeDriver(); d.prepare(); calls = list(d.calls)
    with pytest.raises(ValueError): d.start_brief_command(pulse, seconds)
    assert d.calls == calls


def test_motion_has_dma_bounded_duration_and_neutral_without_python_wakeup():
    d = Driver(); d.prepare()
    waves = [struct.unpack('<6I', x[3]) for x in d.calls if x[0] == 28]
    assert waves == [(8192, 0, width, 0, 8192, 20000-width) for width in (1500,1575,1300)]
    for pulse, duration in ((1575,.8), (1300,.3)):
        d.start_brief_command(pulse,duration)
        assert d.calls[-1] == (93,0,0,bytes((255,0,d.waves[pulse],255,1,int(duration*50),0,
                                          255,0,d.waves[1500],255,1,184,11)))
        assert d.calls[-3][:3] == (33,0,0) and d.calls[-2][:3] == (4,13,0)


@pytest.mark.parametrize('pulse,seconds', [(1000,.8),(2000,.8),(True,.8),(1300,0),
    (1575,.801),(1575,float('nan')),(1575,float('inf')),(1575,True),(1575,.001)])
def test_unobserved_or_unbounded_commands_never_reach_gpio(pulse,seconds):
    d = Driver(); d.prepare(); calls = list(d.calls)
    with pytest.raises(ValueError): d.start_brief_command(pulse,seconds)
    assert d.calls == calls


def test_emergency_neutral_replaces_motion_chain_and_refresh_never_replays_motion():
    d = Driver(); d.prepare(); d.start_brief_command(1575,.8)
    d.neutral()
    assert d.calls[-3][0] == 33 and d.calls[-2][:3] == (4,13,0)
    assert d.calls[-1][3] == bytes((255,0,0,255,1,184,11))
    d.time.now=46; d.check()
    assert d.calls[-1][3] == bytes((255,0,0,255,1,184,11))
    assert d.shutdown() == []
    assert d.calls[-4:] == [(33,0,0,b''),(4,13,0,b''),(0,13,0,b''),(27,0,0,b'')]
    assert d.connection is None and not d.claimed


def test_cleanup_releases_pin_even_if_restoring_neutral_fails():
    d=Driver();d.prepare()
    command=d.command
    def failing(cmd,p1=0,p2=0,data=b''):
        if cmd == 93: raise RuntimeError('failed neutral')
        return command(cmd,p1,p2,data)
    d.command=failing
    assert d.shutdown() == ['failed neutral']
    assert (0,13,0,b'') in d.calls and d.connection is None


def test_cancel_waits_for_high_pulse_to_finish_before_stopping_wave():
    d=Driver();d.prepare();command=d.command;levels=iter((1,1,0))
    def level(cmd,p1=0,p2=0,data=b''):
        if cmd == 3:
            d.calls.append((cmd,p1,p2,data));return next(levels)
        return command(cmd,p1,p2,data)
    d.command=level;d.neutral()
    assert [c[0] for c in d.calls[-6:]] == [3,3,3,33,4,93]


def test_wire_protocol_handles_fragmented_signed_reply_and_extensions():
    class Connection:
        def __init__(self): self.parts=[b'\0'*7,b'\0'*5,struct.pack('<i',-5)]
        def sendall(self,data): self.sent=data
        def recv(self,n): return self.parts.pop(0)
    d=DmaEsc(8890);d.connection=Connection()
    with pytest.raises(RuntimeError,match='failed: -5'):d.command(93,data=b'abc')
    assert d.connection.sent == struct.pack('<4I',93,0,0,3)+b'abc'


def test_stop_revalidation_exception_is_forward_only_scoped_and_fresh():
    from pathlib import Path
    s=json.loads((Path(__file__).parents[1]/'configs/bench-calibration.json').read_text(encoding='utf-8'))
    s.pop('dma_combined_revalidation_review',None)
    s['motor_stop_review']['unresolved_persistent_rotation']=True
    s['parking_protection_review']=dict(requested_by_user=True,servo_signals_disabled=True,
        forward_only=True,latest_restart_confirmed_manual=True,fresh_wheels_raised_confirmed=True,
        fresh_esc_off_confirmed=True,baseline_power_flags=0,baseline_record='baseline.json',boot_id='new-boot')
    s['dma_stop_revalidation_review']=dict(requested_by_user=True,neutral_idle_physically_passed=True,
        signal_backend='dma_finite_wave',neutral_result_record='neutral.json',neutral_result_sha256='hash',
        timing_comparison_record='timing.json',fresh_esc_off_confirmed=True,fresh_wheels_raised_confirmed=True,
        servo_signals_disabled=True,baseline_power_flags=0,baseline_record='baseline.json',boot_id='new-boot')
    validate_state(s,parking_protection=True)
    for scope in ({},{'motor_only':True},{'combined':True},{'steering_only':True}):
        with pytest.raises(ValueError,match='persistent'):validate_state(s,**scope)
    for field,value in [('neutral_idle_physically_passed',False),('neutral_result_sha256',''),
        ('fresh_esc_off_confirmed',False),('baseline_power_flags',0x50000),('boot_id','old-boot')]:
        original=s['dma_stop_revalidation_review'][field];s['dma_stop_revalidation_review'][field]=value
        with pytest.raises(ValueError,match='persistent'):validate_state(s,parking_protection=True)
        s['dma_stop_revalidation_review'][field]=original


def test_joint_retry_requires_new_physical_stop_and_retains_ordinary_drive_lock():
    from pathlib import Path
    s=json.loads((Path(__file__).parents[1]/'configs/bench-calibration.json').read_text(encoding='utf-8'))
    s.pop('dma_stop_revalidation_review',None)
    s['dma_combined_revalidation_review']=dict(requested_by_user=True,neutral_idle_physically_passed=True,
        signal_backend='dma_finite_wave',neutral_result_record='neutral.json',neutral_result_sha256='n',
        space_result_record='space.json',space_result_sha256='s',space_stop_physically_verified=True,
        fresh_esc_off_confirmed=True,fresh_wheels_raised_confirmed=True,gimbal_signals_disabled=True,
        baseline_power_flags=0,baseline_record='baseline.json',boot_id='new',continuous_ground_driving_enabled=False)
    validate_state(s,combined=True)
    for scope in ({},{'motor_only':True},{'steering_only':True}):
        with pytest.raises(ValueError,match='persistent'):validate_state(s,**scope)
    for field,value in [('space_stop_physically_verified',False),('space_result_sha256',''),
        ('fresh_esc_off_confirmed',False),('baseline_power_flags',0x50000),('continuous_ground_driving_enabled',True)]:
        old=s['dma_combined_revalidation_review'][field];s['dma_combined_revalidation_review'][field]=value
        with pytest.raises(ValueError,match='persistent'):validate_state(s,combined=True)
        s['dma_combined_revalidation_review'][field]=old


def test_ground_short_trial_keeps_fault_lock_and_requires_exact_finite_scope():
    from pathlib import Path
    s=json.loads((Path(__file__).parents[1]/'configs/bench-calibration.json').read_text(encoding='utf-8'))
    s.pop('dma_stop_revalidation_review',None);s.pop('dma_combined_revalidation_review',None)
    s['wheels_raised_confirmed']=False
    s['ground_short_review']=dict(requested_by_user=True,fresh_esc_off_confirmed=True,
        clear_area_confirmed=True,operator_switch_in_reach=True,manual_restart_confirmed=True,
        gimbal_signals_disabled=True,baseline_power_flags=0,baseline_record='baseline.json',boot_id='current',
        signal_backend='dma_finite_wave',signal_implementation_sha256='wave-hash',
        neutral_idle_physically_passed=True,neutral_result_record='neutral.json',neutral_result_sha256='n',
        space_stop_physically_verified=True,space_result_record='space.json',space_result_sha256='s',
        space_evidence_boot_id='previous-verified',motion_pulse_max_s=.8,active_session_s=60,
        continuous_ground_driving_enabled=False)
    validate_state(s,ground_short_trial=True)
    s['ground_short_review']['forward_pulse_us'] = 1600
    with pytest.raises(ValueError, match='explicit finite load probe'): validate_state(s,ground_short_trial=True)
    s['ground_short_review']['forward_load_probe'] = dict(requested_by_user=True,
        previous_us=1575,target_us=1600,esc_off_confirmed=True,signal_check_record='check.json',
        signal_check_sha256='signal-hash',implementation_sha256='probe-hash')
    validate_state(s,ground_short_trial=True)
    s['ground_short_review']['forward_pulse_us'] = 1625
    with pytest.raises(ValueError, match='explicit finite load probe'): validate_state(s,ground_short_trial=True)
    s['ground_short_review']['forward_pulse_us'] = 1575
    s['ground_short_review']['manual_restart_confirmed']=False
    s['ground_short_review']['supply_fix_confirmed_by_user']=True
    validate_state(s,ground_short_trial=True)
    s['ground_short_review']['supply_fix_confirmed_by_user']=False
    with pytest.raises(ValueError,match='fresh clear area'):validate_state(s,ground_short_trial=True)
    s['ground_short_review']['manual_restart_confirmed']=True
    for scope in ({},{'combined':True},{'motor_only':True},{'steering_only':True}):
        with pytest.raises(ValueError,match='persistent'):validate_state(s,**scope)
    for field,value in [('clear_area_confirmed',False),('fresh_esc_off_confirmed',False),
        ('manual_restart_confirmed',False),('baseline_power_flags',0x50000),('blocked_by_auto_restart',True),
        ('space_stop_physically_verified',False),('signal_implementation_sha256',''),
        ('motion_pulse_max_s',1),('active_session_s',180),('continuous_ground_driving_enabled',True)]:
        old=s['ground_short_review'].get(field);s['ground_short_review'][field]=value
        with pytest.raises(ValueError,match='fresh clear area'):validate_state(s,ground_short_trial=True)
        s['ground_short_review'][field]=old


def test_ground_trial_controller_expires_after_one_fixed_session():
    from carvision.manual_drive import ManualDrive, SimulationBackend
    class Backend(SimulationBackend):
        hardware_output=True
        ground_short_trial=True
        combined_trial=True
    clock=Clock();drive=ManualDrive(Backend(),clock=clock,settling_s=0,session_s=60,preparation_s=120)
    drive.request('enable',{'client':'ground-client','bench_ready':True});drive.tick()
    assert drive.status()['ground_short_trial'] and drive.status()['test_session_s']==60
    clock.now=61;drive.tick()
    assert drive.status()['mode']=='expired'


def test_raised_load_probe_isolated_after_restart_and_never_unlocks_ground(tmp_path):
    import hashlib
    from pathlib import Path
    from types import SimpleNamespace
    from carvision.manual_hardware import PiBenchBackend
    s=json.loads((Path(__file__).parents[1]/'configs/bench-calibration.json').read_text(encoding='utf-8'))
    s['motor_stop_review']['unresolved_persistent_rotation']=True
    s['restart_review']['further_motion_paused']=True
    s['ground_short_review']['blocked_by_auto_restart']=True
    src=Path(__file__).parents[1]/'src/carvision'
    s['raised_load_probe_review']=dict(requested_by_user=True,fresh_wheels_raised_confirmed=True,
        fresh_esc_off_confirmed=True,servo_signals_disabled=True,baseline_power_flags=0,
        baseline_record='fresh.json',boot_id='current',signal_backend='dma_finite_wave',
        signal_implementation_sha256=hashlib.sha256((src/'esc_wave.py').read_bytes()).hexdigest(),
        load_implementation_sha256=hashlib.sha256((src/'esc_load_probe.py').read_bytes()).hexdigest(),
        neutral_idle_physically_passed=True,neutral_result_record='neutral.json',neutral_result_sha256='n',
        forward_pulse_us=1600,previous_forward_pulse_us=1575,motion_pulse_max_s=.8,
        active_session_s=60,continuous_ground_driving_enabled=False)
    validate_state(s,raised_load_probe=True)
    b=PiBenchBackend(tmp_path,tmp_path,'current',s,raised_load_probe=True)
    assert b.motor_only and b.motor_available and not b.steering_available and not b.reverse_available
    assert b.forward_pulse_us==1600 and b.load_probe and not b.ground_short_trial
    calls=[];b.motor=SimpleNamespace(start_brief_command=lambda *args:calls.append(args))
    b.motion(1600,.8);assert calls==[(1600,.8)]
    with pytest.raises(ValueError,match='forward-only'):b.motion(1300,.3)
    with pytest.raises(ValueError):b.motion(1600,.801)
    with pytest.raises(ValueError):validate_state(s,ground_short_trial=True)
    for field,value in [('fresh_wheels_raised_confirmed',False),('fresh_esc_off_confirmed',False),
                        ('baseline_power_flags',0x50000),('forward_pulse_us',1625),('motion_pulse_max_s',1)]:
        old=s['raised_load_probe_review'][field];s['raised_load_probe_review'][field]=value
        with pytest.raises(ValueError,match='fresh raised wheels'):validate_state(s,raised_load_probe=True)
        s['raised_load_probe_review'][field]=old
    assert s['ground_short_review']['blocked_by_auto_restart'] and s['restart_review']['further_motion_paused']
