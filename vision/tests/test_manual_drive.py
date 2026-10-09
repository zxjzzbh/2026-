import json
from pathlib import Path
import re
import socket
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from carvision.manual_drive import ManualDrive, PausedDrive, DriveError, SimulationBackend, WorkerDrive
from carvision.manual_hardware import PiBenchBackend, validate_state
from carvision.web_preview import PreviewState, make_server


class Clock:
    now = 0.0
    def __call__(self):
        return self.now


class Backend(SimulationBackend):
    def __init__(self, clock):
        self.clock, self.calls, self.failed = clock, [], False
    def motion(self, pulse, seconds):
        self.calls.append(('motion', self.clock(), pulse, seconds))
    def neutral(self):
        self.calls.append(('neutral', self.clock()))
    def steering(self, pulse):
        self.calls.append(('steering', self.clock(), pulse))
    def check(self):
        if self.failed:
            raise RuntimeError('undervoltage')
    def close(self):
        self.calls.append(('close', self.clock()))


def prepared():
    clock = Clock()
    backend = Backend(clock)
    drive = ManualDrive(backend, clock=clock, settling_s=0)
    drive.request('enable', {'client': 'test-client', 'bench_ready': True})
    drive.tick()
    return drive, backend, clock


def command(drive, sequence, motor='forward', steering='center', client='test-client'):
    return drive.request('command', dict(client=client, sequence=sequence, motor=motor, steering=steering))


def state_for_isolated_review_test():
    state = json.loads((Path(__file__).parents[1]/'configs/bench-calibration.json').read_text(encoding='utf-8'))
    # Test each review gate separately from a newly reported deployment fault.
    state.pop('motor_stop_review', None)
    state.pop('dma_stop_revalidation_review', None)
    state.pop('dma_combined_revalidation_review', None)
    return state


def test_paused_dashboard_rejects_enable_and_motion_without_simulation():
    drive = PausedDrive('树莓派本次开机记录了欠压，实车测试暂停。')
    server = make_server(('127.0.0.1', 0), PreviewState(), drive=drive)
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base) as response:
            page = response.read().decode()
        key = re.search(r'const key="([a-f0-9]+)"', page).group(1)
        for action in ('enable', 'command', 'speed', 'reset'):
            req = Request(base+'/api/drive/'+action, method='POST',
                          data=json.dumps({'client': 'test-client', 'bench_ready': True,
                                           'sequence': 0, 'motor': 'forward',
                                           'steering': 'left', 'speed_percent': 20}).encode(),
                          headers={'Content-Type': 'application/json', 'X-Drive-Key': key})
            with pytest.raises(HTTPError) as error:
                urlopen(req)
            assert error.value.code == 400
            assert '欠压' in json.load(error.value)['error']
        with urlopen(base+'/api/drive/status') as response:
            status = json.load(response)
        assert status['controls_paused'] and status['mode'] == 'paused'
        assert not status['hardware_output'] and not status['continuous_simulation']
        assert not status['motor_available'] and not status['steering_available']
        assert drive.request('emergency', {}) == drive.status()
        assert drive.request('stop', {}) == drive.status()
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=2); drive.close()


def test_long_hold_never_repeats_finite_forward_and_release_allows_next_trial():
    drive, backend, clock = prepared()
    for seq in range(25):
        clock.now = seq*.1
        command(drive, seq)
        drive.tick()
    motions = [c for c in backend.calls if c[0]=='motion']
    assert len(motions)==1 and motions[0][2:]==(1575, .8)
    assert drive.status()['release_required']
    command(drive, 25, 'stop')
    command(drive, 26)
    drive.tick()
    assert len([c for c in backend.calls if c[0]=='motion'])==2


def test_increased_ground_point_still_stops_on_space_and_cannot_restart_by_heartbeat():
    clock = Clock(); backend = Backend(clock)
    backend.hardware_output = True; backend.ground_short_trial = True
    backend.forward_pulse_us = 1600; backend.load_probe = True
    drive = ManualDrive(backend, clock=clock, settling_s=0, session_s=60)
    drive.request('enable', {'client':'test-client','bench_ready':True}); drive.tick()
    command(drive, 0); drive.tick()
    assert [c for c in backend.calls if c[0]=='motion'][-1][2:] == (1600,.8)
    assert drive.status()['load_probe'] and drive.status()['forward_pulse_us']==1600
    clock.now = .1; drive.request('emergency',{'stop_source':'keyboard_space'})
    count = len([c for c in backend.calls if c[0]=='motion'])
    with pytest.raises(DriveError): command(drive, 1)
    drive.tick()
    assert drive.status()['mode']=='emergency' and drive.status()['motor_pulse_us']==1500
    assert len([c for c in backend.calls if c[0]=='motion'])==count


def test_reverse_keeps_the_observed_neutral_gap_and_never_sends_forward():
    drive, backend, clock = prepared()
    for seq in range(26):
        clock.now = seq*.1
        command(drive, seq, 'reverse')
        drive.tick()
    motions = [c for c in backend.calls if c[0]=='motion']
    assert len(motions)==2
    assert all(c[2]==1300 for c in motions)
    assert motions[0][3] <= .3 and motions[1][3] <= .8
    assert motions[1][1] - (motions[0][1]+motions[0][3]) >= 1.2
    assert drive.status()['motor_pulse_us']==1500


def test_reverse_status_distinguishes_brake_wait_and_actual_reverse_output():
    drive, backend, clock = prepared()
    for seq in range(17):
        clock.now=seq*.1
        command(drive,seq,'reverse');drive.tick()
        status=drive.status()
        if seq<3:
            assert status['motor_stage']=='brake'
            assert status['reverse_wait_remaining_s']>1
        elif seq<15:
            assert status['motor_stage']=='neutral_gap'
            assert status['motor_pulse_us']==1500
            assert status['reverse_wait_remaining_s']==pytest.approx(1.5-clock.now)
        else:
            assert status['motor_stage']=='reverse'
            assert status['reverse_wait_remaining_s']==0
    drive.request('stop',{})
    assert drive.status()['motor_stage'] is None
    assert drive.status()['reverse_wait_remaining_s']==0


def test_reverse_neutral_gap_starts_after_hardware_ack_and_release_cancels_it():
    clock=Clock()
    class DelayedBackend(Backend):
        def neutral(self):
            super().neutral()
            self.clock.now += .04
    backend=DelayedBackend(clock)
    drive=ManualDrive(backend,clock=clock,settling_s=0)
    drive.request('enable',{'client':'test-client','bench_ready':True});drive.tick()
    command(drive,1,'reverse');drive.tick()
    neutral_ack=None
    for seq in range(2,40):
        clock.now += .05
        command(drive,seq,'reverse');drive.tick()
        if drive.stage=='neutral_gap' and neutral_ack is None:
            neutral_ack=clock.now
        if drive.stage=='reverse':
            motion=[x for x in backend.calls if x[0]=='motion'][-1]
            assert motion[1]-neutral_ack >= 1.2
            assert motion[3]<=.8
            break
    else:
        pytest.fail('reverse stage never started')
    assert len([x for x in backend.calls if x[0]=='motion'])==2

    drive.request('stop',{});command(drive,40,'stop')
    command(drive,41,'reverse');drive.tick()
    clock.now += .1
    drive.request('stop',{})
    assert drive.status()['reverse_cancelled']
    assert drive.status()['motor_pulse_us']==1500
    count=len([x for x in backend.calls if x[0]=='motion'])
    clock.now += 2;drive.tick()
    assert len([x for x in backend.calls if x[0]=='motion'])==count


@pytest.mark.parametrize('motor,turn', [('forward','left'),('forward','right'),('reverse','left'),('reverse','right')])
def test_combined_chord_can_join_motor_to_turn_without_repeating_or_changing_direction(motor,turn):
    drive,backend,clock=prepared();backend.combined_trial=True
    command(drive,1,'stop',turn);drive.tick()
    clock.now=.07
    command(drive,2,motor,turn);drive.tick()
    assert not drive.blocked and drive.motor==motor and drive.turn==turn
    for seq in range(3,70):
        clock.now += .05
        command(drive,seq,motor,turn);drive.tick()
    motions=[x for x in backend.calls if x[0]=='motion']
    assert len(motions)==(2 if motor=='reverse' else 1)
    assert all(x[2]==(1300 if motor=='reverse' else 1575) and 0<x[3]<=.8 for x in motions)
    assert drive.status()['motor_pulse_us']==1500 and drive.blocked


def test_late_motor_addition_and_forward_reverse_change_still_stop_combined_trial():
    drive,backend,clock=prepared();backend.combined_trial=True
    command(drive,1,'stop','left');drive.tick()
    clock.now=.15;command(drive,2,'stop','left');drive.tick()
    clock.now=.25;command(drive,3,'reverse','left');drive.tick()
    assert drive.blocked and not [x for x in backend.calls if x[0]=='motion']
    command(drive,4,'stop');command(drive,5,'forward','left');drive.tick()
    count=len([x for x in backend.calls if x[0]=='motion'])
    clock.now += .05;command(drive,6,'reverse','left');drive.tick()
    assert drive.blocked and drive.pulse==1500
    assert len([x for x in backend.calls if x[0]=='motion'])==count


def test_lost_control_messages_stop_and_latch_before_next_reverse_stage():
    drive, backend, clock = prepared()
    command(drive, 1, 'reverse');drive.tick()
    clock.now=.201;drive.tick()
    assert drive.status()['mode']=='emergency'
    clock.now=1.6
    with pytest.raises(DriveError):
        command(drive, 2, 'reverse')
    assert len([c for c in backend.calls if c[0]=='motion'])==1
    drive.request('reset', {})
    assert drive.status()['mode']=='disabled'


def test_stop_rejects_delayed_motion_until_a_new_release_and_sequence():
    drive, backend, clock = prepared()
    command(drive, 1);drive.tick()
    drive.request('stop', {})
    command(drive, 2);drive.tick()
    assert drive.status()['motor_pulse_us']==1500
    assert len([c for c in backend.calls if c[0]=='motion'])==1
    command(drive, 3, 'stop')
    with pytest.raises(DriveError, match='sequence'):
        command(drive, 2)
    command(drive, 4);drive.tick()
    assert len([c for c in backend.calls if c[0]=='motion'])==2


def test_timeout_reason_survives_late_release_and_is_returned_to_operator():
    drive, backend, clock = prepared()
    command(drive, 1); drive.tick()
    clock.now = .201; drive.tick()
    reason = drive.reason
    assert '控制消息超时' in reason
    drive.request('stop', {'stop_source': 'window_blur'})
    assert drive.mode == 'emergency' and drive.reason == reason
    assert drive.pulse == 1500
    with pytest.raises(DriveError, match='控制消息超时'):
        command(drive, 2)
    drive.request('reset', {})
    assert drive.mode == 'disabled'


def test_release_confirmation_after_urgent_stop_does_not_expire_an_idle_lease():
    drive,backend,clock=prepared()
    command(drive,1);drive.tick()
    drive.request('stop',{'stop_source':'key_release'})
    command(drive,2,'stop');drive.tick()
    clock.now += 1;drive.tick()
    assert drive.status()['mode']=='enabled' and drive.expires is None
    assert drive.pulse==1500 and len([c for c in backend.calls if c[0]=='motion'])==1


def test_emergency_and_bad_direction_cannot_be_cleared_by_ordinary_commands():
    drive, backend, clock = prepared()
    command(drive, 1);drive.tick()
    with pytest.raises(DriveError, match='direction'):
        command(drive, 2, 'full-speed')
    assert drive.status()['mode']=='emergency'
    with pytest.raises(DriveError):
        command(drive, 3)
    assert backend.calls[-1][0]=='neutral'


@pytest.mark.parametrize('action,source', [('emergency','keyboard_space'),('stop','window_blur')])
def test_stop_audit_distinguishes_a_protection_request_from_finite_motion_expiry(action,source):
    drive,backend,clock=prepared();events=[];drive.event_sink=events.append
    command(drive,1);drive.tick()
    for seq in range(2,5):
        clock.now += .1;command(drive,seq);drive.tick()
    drive.request(action,{'stop_source':source})
    request=next(e for e in events if e['event']=='safety_request')
    assert request['input_source']==source and request['motor_pulse_us']==1575
    assert request['monotonic_s']<.8
    assert events[-1]['event']=='neutral' and events[-1]['monotonic_s']<.8
    assert drive.pulse==1500 and drive.blocked
    assert len([c for c in backend.calls if c[0]=='motion'])==1
    if action=='emergency':
        with pytest.raises(DriveError): command(drive,5)


def test_other_page_cannot_take_control_and_simulation_has_no_hardware_output():
    drive, backend, clock = prepared()
    with pytest.raises(DriveError, match='另一页面'):
        command(drive, 1, client='other-client')
    assert not drive.status()['hardware_output']
    assert not [c for c in backend.calls if c[0]=='motion']


def test_steering_is_rate_limited_and_returns_to_center_after_stop():
    drive, backend, clock = prepared()
    command(drive, 1, 'stop', 'left')
    for i in range(15):
        clock.now=i*.02
        command(drive, i+2, 'stop', 'left');drive.tick();drive.tick()
    pulses=[c[2] for c in backend.calls if c[0]=='steering']
    assert max(pulses)<=1750
    assert all(abs(a-b)<=10 for a,b in zip([1650]+pulses,pulses))
    drive.request('stop', {})
    for i in range(20):
        clock.now += .021;drive.tick()
    assert drive.status()['steering_pulse_us']==1650


def test_power_failure_and_session_deadline_close_the_backend():
    drive, backend, clock = prepared()
    command(drive, 1);drive.tick()
    backend.failed=True;drive.tick()
    assert drive.status()['mode']=='fault' and backend.calls[-1][0]=='close'
    drive, backend, clock = prepared()
    clock.now=121;drive.tick()
    assert drive.status()['mode']=='expired' and backend.calls[-1][0]=='close'
    with pytest.raises(DriveError):
        drive.request('enable', {'client':'test-client','bench_ready':True})


def test_preparation_does_not_consume_active_interval_or_allow_repeated_extensions():
    clock = Clock(); backend = Backend(clock)
    drive = ManualDrive(backend, clock=clock, settling_s=12,
                        session_s=180, preparation_s=600)
    clock.now = 590; drive.tick()
    assert drive.status()['mode'] == 'disabled'
    assert drive.status()['session_phase'] == 'preparing'
    assert not [c for c in backend.calls if c[0] == 'motion']
    drive.request('enable', {'client':'test-client','bench_ready':True})
    clock.now = 602; drive.tick()
    assert drive.status()['mode'] == 'enabled'
    assert drive.status()['session_remaining_s'] == 180
    original_deadline = drive.deadline
    drive.request('emergency', {})
    drive.request('reset', {})
    clock.now = 610
    drive.request('enable', {'client':'test-client','bench_ready':True})
    assert drive.deadline == original_deadline
    clock.now = original_deadline; drive.tick()
    assert drive.status()['mode'] == 'expired'
    assert backend.calls[-1][0] == 'close'


def test_preparation_expiry_and_fault_both_prevent_late_enable():
    for fault in (False, True):
        clock = Clock(); backend = Backend(clock)
        drive = ManualDrive(backend, clock=clock, preparation_s=600)
        clock.now = 601 if not fault else 20
        backend.failed = fault
        drive.tick()
        assert drive.status()['mode'] == ('fault' if fault else 'expired')
        assert backend.calls[-1][0] == 'close'
        with pytest.raises(DriveError):
            drive.request('enable', {'client':'test-client','bench_ready':True})


def test_startup_rejects_unverified_profile_and_unresolved_restart():
    state = state_for_isolated_review_test()
    state.setdefault('manual_bench_power_review', {})['further_motion_paused']=False
    state.setdefault('restart_review', {})['further_motion_paused']=False
    validate_state(state)
    state['esc']['negative_test_us']=1000
    with pytest.raises(ValueError, match='observed'):
        validate_state(state)
    state['esc']['negative_test_us']=1300
    state['restart_review']['further_motion_paused']=True
    with pytest.raises(ValueError, match='restart'):
        validate_state(state)


def test_startup_rejects_an_unresolved_bench_undervoltage():
    state = state_for_isolated_review_test()
    state['manual_bench_power_review']={'further_motion_paused':True}
    with pytest.raises(ValueError, match='undervoltage'):
        validate_state(state)


def test_motor_only_review_preserves_full_test_pause_and_requires_explicit_scope():
    state = state_for_isolated_review_test()
    state['manual_bench_power_review']={'further_motion_paused':True}
    state.pop('motor_only_review', None)
    state.setdefault('restart_review', {})['further_motion_paused']=False
    with pytest.raises(ValueError, match='undervoltage'):
        validate_state(state, motor_only=True)
    state['motor_only_review']={'requested_by_user':True,'servo_signals_disabled':True,'baseline_power_flags':0}
    validate_state(state, motor_only=True)
    assert state['manual_bench_power_review']['further_motion_paused']
    with pytest.raises(ValueError, match='undervoltage'):
        validate_state(state)
    state['motor_only_review']['baseline_power_flags']=0x50000
    with pytest.raises(ValueError, match='undervoltage'):
        validate_state(state, motor_only=True)


def test_motor_only_control_rejects_turning_without_commanding_any_servo():
    drive, backend, clock = prepared()
    backend.steering_available=False
    assert drive.status()['steering_available'] is False
    with pytest.raises(DriveError, match='steering disabled'):
        command(drive, 1, 'forward', 'left')
    assert drive.status()['mode']=='emergency'
    assert not [c for c in backend.calls if c[0] in ('motion','steering')]


def test_steering_only_review_allows_only_scoped_retry_and_keeps_full_pause():
    state = state_for_isolated_review_test()
    state['manual_bench_power_review'] = {'further_motion_paused': True}
    state['restart_review'] = {'further_motion_paused': True}
    state.pop('steering_only_review', None)
    with pytest.raises(ValueError, match='fresh isolated'):
        validate_state(state, steering_only=True)
    state['steering_only_review'] = dict(requested_by_user=True,
        motor_and_gimbal_signals_disabled=True, center_power_review_passed=True,
        center_mechanical_review_passed=True,
        fresh_wheels_raised_confirmed=True, fresh_esc_off_confirmed=True,
        baseline_power_flags=0)
    validate_state(state, steering_only=True)
    assert state['manual_bench_power_review']['further_motion_paused']
    assert state['restart_review']['further_motion_paused']
    with pytest.raises(ValueError, match='undervoltage'):
        validate_state(state)
    state['steering_only_review']['fresh_esc_off_confirmed'] = False
    with pytest.raises(ValueError, match='fresh isolated'):
        validate_state(state, steering_only=True)


def test_combined_review_requires_same_boot_isolated_results_and_keeps_full_pause():
    state = state_for_isolated_review_test()
    state['manual_bench_power_review'] = {'further_motion_paused': True}
    state['restart_review'] = {'further_motion_paused': True}
    state.pop('combined_review', None)
    with pytest.raises(ValueError, match='combined trial requires'):
        validate_state(state, combined=True)
    for name in ('motor_only_review', 'steering_only_review'):
        state[name] = dict(keyboard_control_verified=True, power_flags_after=0,
                           boot_unchanged=True, boot_id='reviewed-boot')
    state['combined_review'] = dict(requested_by_user=True, gimbal_signals_disabled=True,
        fresh_wheels_raised_confirmed=True, fresh_esc_off_confirmed=True,
        baseline_power_flags=0, baseline_record='fresh-baseline.json', boot_id='reviewed-boot')
    validate_state(state, combined=True)
    with pytest.raises(ValueError, match='undervoltage'):
        validate_state(state)
    state['steering_only_review']['boot_id'] = 'old-boot'
    with pytest.raises(ValueError, match='combined trial requires'):
        validate_state(state, combined=True)
    assert state['manual_bench_power_review']['further_motion_paused']
    assert state['restart_review']['further_motion_paused']


def test_parking_review_keeps_historical_pauses_and_requires_current_preparation(tmp_path):
    state = state_for_isolated_review_test()
    state['manual_bench_power_review'] = {'further_motion_paused': True}
    state['restart_review'] = {'further_motion_paused': True}
    state['parking_protection_review'] = dict(requested_by_user=True, servo_signals_disabled=True,
        forward_only=True, latest_restart_confirmed_manual=True, fresh_wheels_raised_confirmed=True,
        fresh_esc_off_confirmed=True, baseline_power_flags=0,
        baseline_record='fresh-baseline.json', boot_id='current-boot')
    validate_state(state, parking_protection=True)
    backend = PiBenchBackend(tmp_path, tmp_path, 'current-boot', state, parking_protection=True)
    assert backend.motor_only and not backend.steering_available and not backend.reverse_available
    with pytest.raises(ValueError, match='forward-only'):
        backend.motion(1300, .8)
    assert backend.motor is None
    for key, value in [('baseline_power_flags', 0x50000), ('fresh_esc_off_confirmed', False),
                       ('latest_restart_confirmed_manual', False), ('baseline_record', None)]:
        original = state['parking_protection_review'][key]
        state['parking_protection_review'][key] = value
        with pytest.raises(ValueError, match='parking protection requires'):
            validate_state(state, parking_protection=True)
        state['parking_protection_review'][key] = original
    with pytest.raises(ValueError, match='undervoltage'):
        validate_state(state)
    assert state['restart_review']['further_motion_paused']
    with pytest.raises(ValueError, match='only one'):
        validate_state(state, parking_protection=True, combined=True)


@pytest.mark.parametrize('motor,turn', [('reverse', 'center'), ('forward', 'left')])
def test_parking_controller_rejects_other_outputs_before_motion(motor, turn):
    drive, backend, clock = prepared()
    backend.parking_protection_trial = True
    backend.reverse_available = backend.steering_available = False
    with pytest.raises(DriveError):
        command(drive, 1, motor, turn)
    assert not [call for call in backend.calls if call[0] in ('motion', 'steering')]
    assert drive.status()['mode'] == 'emergency'
    assert drive.status()['parking_protection_trial'] and not drive.status()['reverse_available']


@pytest.mark.parametrize('scope', [{}, {'motor_only': True}, {'steering_only': True},
                                  {'combined': True}, {'parking_protection': True}])
def test_reported_persistent_rotation_blocks_every_real_trial_scope(scope):
    state = state_for_isolated_review_test()
    state['motor_stop_review'] = {'unresolved_persistent_rotation': True}
    with pytest.raises(ValueError, match='persistent wheel rotation'):
        validate_state(state, **scope)


def continuous_prepared():
    clock = Clock(); backend = Backend(clock)
    drive = ManualDrive(backend, clock=clock, settling_s=0, session_s=None,
                        continuous_simulation=True)
    drive.request('enable', {'client': 'test-client', 'bench_ready': True}); drive.tick()
    return drive, backend, clock


def test_continuous_simulation_speed_changes_without_extending_the_control_lease():
    drive, backend, clock = continuous_prepared()
    for i in range(40):
        clock.now = i*.05; command(drive, i); drive.tick()
    assert drive.status()['mode'] == 'enabled' and drive.status()['motor_pulse_us'] == 1515
    drive.request('speed', {'client':'test-client', 'speed_percent':60}); drive.tick()
    assert drive.status()['motor_pulse_us'] == 1545
    assert len([c for c in backend.calls if c[0]=='motion']) > 1
    assert all(c[3] <= .3 for c in backend.calls if c[0]=='motion')
    clock.now += .201; drive.tick()
    assert drive.status()['mode'] == 'emergency' and drive.status()['motor_pulse_us'] == 1500
    with pytest.raises(DriveError): command(drive, 50)


def test_continuous_axes_release_and_late_motor_join_are_independent():
    drive, backend, clock = continuous_prepared()
    for i in range(30):
        clock.now=i*.05; command(drive, i, 'stop', 'left'); drive.tick()
    command(drive, 30, 'forward', 'left'); drive.tick()
    assert drive.status()['motor_pulse_us'] == 1515
    clock.now += .05; command(drive, 31, 'forward', 'center'); drive.tick()
    assert drive.status()['motor_direction'] == 'forward' and not drive.status()['release_required']
    clock.now += .05; command(drive, 32, 'stop', 'left'); drive.tick()
    assert drive.status()['motor_pulse_us'] == 1500 and drive.status()['steering_direction'] == 'left'
    drive.request('emergency', {'stop_source':'keyboard_space'})
    with pytest.raises(DriveError): command(drive, 33)


def test_continuous_reverse_preserves_wait_and_keeps_only_simulated_output():
    drive, backend, clock = continuous_prepared()
    drive.request('speed', {'client':'test-client', 'speed_percent':40})
    for i in range(100):
        clock.now = i*.05; command(drive, i, 'reverse', 'right'); drive.tick()
    assert drive.status()['motor_stage'] == 'reverse'
    assert drive.status()['motor_pulse_us'] == 1420 and not drive.status()['hardware_output']
    stages=[c for c in backend.calls if c[0]=='motion']
    assert stages[0][2] == 1300 and stages[1][1] - (stages[0][1]+stages[0][3]) >= 1.2
    drive.request('speed', {'client':'test-client', 'speed_percent':0})
    assert drive.status()['motor_pulse_us'] == 1500 and drive.status()['release_required']


@pytest.mark.parametrize('value', [-1,101,True,20.5,None,'40'])
def test_speed_request_rejects_invalid_values_without_changing_speed(value):
    drive, backend, clock = continuous_prepared()
    with pytest.raises(DriveError):
        drive.request('speed', {'client':'test-client', 'speed_percent':value})
    assert drive.status()['speed_percent'] == 20


def test_zero_speed_never_enters_reverse_and_other_client_cannot_adjust_speed():
    drive, backend, clock = continuous_prepared()
    drive.request('speed', {'client':'test-client', 'speed_percent':0})
    command(drive, 1, 'stop'); command(drive, 2, 'reverse'); drive.tick()
    assert not [c for c in backend.calls if c[0]=='motion']
    with pytest.raises(DriveError, match='another page'):
        drive.request('speed', {'client':'other-client', 'speed_percent':80})


def test_continuous_path_cannot_be_enabled_for_any_real_hardware_backend():
    backend=Backend(Clock()); backend.hardware_output=True
    with pytest.raises(ValueError, match='never drive real hardware'):
        ManualDrive(backend, continuous_simulation=True)
    assert backend.calls == []


@pytest.mark.parametrize('motor,turn', [('forward','left'),('reverse','right')])
def test_combined_hold_preserves_finite_motor_stages_and_returns_center(motor, turn):
    clock = Clock(); backend = Backend(clock)
    backend.combined_trial = True; backend.steering_active = True
    backend.steering_idle = lambda: backend.calls.append(('idle', clock()))
    drive = ManualDrive(backend, clock=clock, settling_s=0)
    drive.request('enable', {'client':'test-client','bench_ready':True})
    drive.tick()
    assert not [c for c in backend.calls if c[0]=='idle']
    for seq in range(65):
        clock.now=seq*.05
        command(drive, seq, motor, turn); drive.tick()
    motions=[c for c in backend.calls if c[0]=='motion']
    assert len(motions)==(1 if motor=='forward' else 2)
    assert all(c[2]==(1575 if motor=='forward' else 1300) and 0<c[3]<=.8 for c in motions)
    pulses=[c[2] for c in backend.calls if c[0]=='steering']
    assert pulses and all(1550<=p<=1750 for p in pulses)
    assert all(abs(a-b)<=10 for a,b in zip([1650]+pulses,pulses))
    assert drive.status()['release_required'] and drive.status()['motor_pulse_us']==1500
    assert drive.status()['steering_pulse_us']==1650
    assert [c for c in backend.calls if c[0]=='idle']


def test_steering_only_rejects_motor_before_output_and_releases_after_center():
    drive, backend, clock = prepared()
    backend.motor_available = False
    with pytest.raises(DriveError, match='motor disabled'):
        command(drive, 1, 'forward', 'left')
    assert not [c for c in backend.calls if c[0] in ('motion', 'steering')]
    assert drive.status()['mode'] == 'emergency'

    drive, backend, clock = prepared()
    backend.motor_available = False
    backend.steering_idle = lambda: backend.calls.append(('idle', clock(), drive.steering_us))
    for i in range(15):
        clock.now = i * .021
        command(drive, i, 'stop', 'left'); drive.tick()
    assert drive.status()['steering_pulse_us'] == 1750
    assert not [c for c in backend.calls if c[0] == 'motion']
    drive.request('stop', {})
    for _ in range(12):
        clock.now += .021; drive.tick()
    assert drive.status()['steering_pulse_us'] == 1650
    assert not [c for c in backend.calls if c[0] == 'idle' and c[1] > 0]
    for _ in range(12):
        clock.now += .021; drive.tick()
    idle = [c for c in backend.calls if c[0] == 'idle']
    assert idle and all(c[2] == 1650 for c in idle)
    assert drive.status()['motor_pulse_us'] is None


def test_steering_only_hardware_startup_never_loads_motor_or_holds_servo(tmp_path, monkeypatch):
    state = state_for_isolated_review_test()
    state['steering_only_review'] = dict(requested_by_user=True,
        motor_and_gimbal_signals_disabled=True, center_power_review_passed=True,
        center_mechanical_review_passed=True,
        fresh_wheels_raised_confirmed=True, fresh_esc_off_confirmed=True,
        baseline_power_flags=0, boot_id='reviewed-boot')
    backend = PiBenchBackend(tmp_path, tmp_path, 'reviewed-boot', state, steering_only=True)
    def read_bytes(path):
        assert path.as_posix() == '/proc/device-tree/model', 'motor helper must never be read'
        return b'Raspberry Pi 4 Model B'
    original_read = Path.read_text
    monkeypatch.setattr(Path, 'read_bytes', read_bytes)
    monkeypatch.setattr(Path, 'read_text', lambda path, **kw: 'reviewed-boot'
        if path.as_posix() == '/proc/sys/kernel/random/boot_id' else original_read(path, **kw))
    monkeypatch.setattr(backend, 'power', lambda: 0)
    monkeypatch.setattr(backend, 'pins', lambda: '\n'.join(f'{p}: // GPIO{p} = input' for p in (12,13,17,27)))
    class Probe:
        def __enter__(self): return self
        def __exit__(self, *a): pass
        def bind(self, address): pass
    monkeypatch.setattr('carvision.manual_hardware.socket.socket', Probe)
    class Process:
        pid = 456
        def poll(self): return None
        def wait(self, **kw): return 0
    monkeypatch.setattr('carvision.manual_hardware.subprocess.Popen', lambda *a, **kw: Process())
    class Reply:
        def __init__(self, code): self.returncode = code
    monkeypatch.setattr('carvision.manual_hardware.subprocess.run',
        lambda cmd, **kw: Reply(1 if cmd[0] == 'pgrep' else 0))
    class Driver:
        def __init__(self, *a): self.pulse = 0; self.calls = []; self.released = False
        def open(self): pass
        def close(self): pass
        def mode(self, pin): return 0
        def set_servo(self, pin, pulse): self.calls.append((pin, pulse)); self.pulse = pulse; self.released = False
        def servo_pulse(self, pin):
            if self.released: raise RuntimeError('PI_NOT_SERVO after release')
            return self.pulse
        def release(self, pin): self.calls.append(('input', pin)); self.released = True
    monkeypatch.setattr('carvision.manual_hardware.CalibrationDriver', Driver)
    backend.open()
    assert backend.motor is None and backend.client.calls == []
    assert not backend.steering_active
    with pytest.raises(RuntimeError, match='motor output disabled'):
        backend.motion(1575, .8)
    backend.steering(1660)
    backend.steering(1650)
    backend.steering_idle()
    assert backend.client.calls[-2:] == [(12, 0), ('input', 12)]
    assert not backend.steering_active
    backend.close()
    result = json.loads((tmp_path/'result.json').read_text())
    assert result['motor_signal_pin_preserved'] and result['other_pins_preserved']
    assert result['pins_are_inputs'] and result['cleanup_errors'] == []


def test_http_control_key_origin_and_manual_page_do_not_change_preview_defaults():
    drive, backend, clock = prepared()
    preview=PreviewState()
    server=make_server(('127.0.0.1',0),preview,drive=drive)
    thread=threading.Thread(target=server.serve_forever,kwargs={'poll_interval':.01},daemon=True)
    thread.start()
    base=f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base) as response:
            page=response.read().decode()
        key=re.search(r'const key="([a-f0-9]+)"',page).group(1)
        session=re.search(r'pageSession="([a-f0-9]+)"',page).group(1)
        assert '架空驾驶短测' in page and 'pointercancel' in page and 'pagehide' in page
        def post(key, origin):
            request=Request(base+'/api/drive/emergency',data=b'{}',method='POST',
                            headers={'Content-Type':'application/json','X-Drive-Key':key,'Origin':origin})
            return urlopen(request)
        for bad_key, origin in [('wrong',base),(key,'http://untrusted.example')]:
            with pytest.raises(HTTPError) as error:
                post(bad_key,origin)
            assert error.value.code==403
            assert json.load(error.value)['code']=='control_page_rejected'
        with post(key,base) as response:
            assert json.load(response)['mode']=='emergency'
        with urlopen(base+'/api/drive/status') as response:
            status=json.load(response)
            assert status['hardware_output'] is False
            assert status['control_session_id']==session
    finally:
        preview.finish();server.shutdown();server.server_close();thread.join(timeout=2);drive.close()


def test_previous_page_cannot_enable_a_restarted_control_service():
    drives=[ManualDrive(settling_s=0, session_s=None) for _ in range(2)]
    previews=[PreviewState(),PreviewState()]
    servers=[make_server(('127.0.0.1',0),p,drive=d) for p,d in zip(previews,drives)]
    threads=[threading.Thread(target=s.serve_forever,kwargs={'poll_interval':.01},daemon=True) for s in servers]
    for t in threads:t.start()
    try:
        pages=[urlopen(f'http://127.0.0.1:{s.server_port}').read().decode() for s in servers]
        sessions=[re.search(r'pageSession="([a-f0-9]+)"',page).group(1) for page in pages]
        assert sessions[0]!=sessions[1]
        old_key=re.search(r'const key="([a-f0-9]+)"',pages[0]).group(1)
        base=f'http://127.0.0.1:{servers[1].server_port}'
        request=Request(base+'/api/drive/enable',data=json.dumps({'client':'old-page-client','bench_ready':True}).encode(),method='POST',
                        headers={'Content-Type':'application/json','X-Drive-Key':old_key,'Origin':base})
        with pytest.raises(HTTPError) as error:urlopen(request)
        assert error.value.code==403
        assert json.load(error.value)['code']=='control_page_rejected'
        assert drives[1].status()['mode']=='disabled'
        with urlopen(base+'/api/drive/status') as response:assert json.load(response)['control_session_id']==sessions[1]
    finally:
        for p,s,t,d in zip(previews,servers,threads,drives):
            p.finish();s.shutdown();s.server_close();t.join(timeout=2);d.close()


def test_lost_motor_only_worker_preserves_scope_and_does_not_claim_a_live_pulse(tmp_path, monkeypatch):
    worker=WorkerDrive(tmp_path/'missing.sock',steering_available=False)
    def unavailable(*_):
        raise ConnectionRefusedError('worker ended')
    monkeypatch.setattr(worker,'request',unavailable)
    worker.last_status.update({'motor_pulse_us':1575,'session_remaining_s':0})
    status=worker.status()
    assert status['mode']=='expired' and not status['worker_available']
    assert status['steering_available'] is False
    assert status['motor_pulse_us'] is None


@pytest.mark.skipif(not hasattr(socket, 'AF_UNIX'), reason='Pi worker uses Unix sockets')
def test_ended_worker_has_readable_error_and_cannot_be_reenabled(tmp_path):
    worker = WorkerDrive(tmp_path/'ended.sock', steering_available=False)
    worker.last_status.update(mode='disabled', session_phase='preparing', session_remaining_s=0)
    for action in ('emergency', 'reset', 'enable'):
        with pytest.raises(DriveError, match='控制进程已停止') as error:
            worker.request(action, {'client': 'expired-page', 'bench_ready': True})
        assert 'No such file' not in str(error.value)
    status = worker.status()
    assert status['mode'] == 'expired'
    assert status['worker_available'] is False
    assert status['motor_pulse_us'] is None
