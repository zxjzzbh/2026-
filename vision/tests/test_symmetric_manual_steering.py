"""Offline-only checks of requested endpoints; no physical angle claim."""
import json
import struct
from pathlib import Path

import pytest

from carvision.manual_drive import DriveError
from carvision.manual_steering import manual_endpoints
from carvision.pi5_pwm import Pi5BenchBackend, RasAdapterPWM
from carvision.rasadapter5 import RasAdapter
from test_steering_pwm import Steering, ready, tune


class SymmetricSteering(Steering):
    steering_center_us = 1610
    manual_steering_span_us = 260
    steering_single_target = True


def test_requested_endpoints_are_larger_on_both_sides_and_match_operator_observations():
    left, right = manual_endpoints(1610, 260)
    assert left == 1870 and right == 1350
    assert left - 1610 == 1610 - right == 260
    assert left > 1750 and right < 1550
    assert manual_endpoints(1610) == (1750, 1550)


@pytest.mark.parametrize('center,span', [(1650,260), (1610,261), (1610,True), (1610,260.0)])
def test_unrequested_envelopes_rejected(center, span):
    with pytest.raises(ValueError):
        manual_endpoints(center, span)


@pytest.mark.parametrize('direction,pulse', [('left',1870), ('right',1350)])
def test_keyboard_uses_full_symmetric_endpoints_and_release_recenters(direction, pulse):
    drive, backend, clock = ready(SymmetricSteering())
    drive.request('command', dict(client='pwm-test', sequence=1, motor='stop', steering=direction))
    drive.tick()
    assert backend.calls[-1] == pulse
    clock.now = .04
    drive.request('command', dict(client='pwm-test', sequence=2, motor='stop', steering='center'))
    drive.tick()
    assert backend.calls[-1] == 1610
    state = drive.status()
    assert (state['steering_left_us'], state['steering_right_us']) == (1870,1350)
    assert state['manual_endpoints_field_verified'] is False


@pytest.mark.parametrize('pulse', [1350, 1549, 1610, 1751, 1870])
def test_raw_s3_allows_requested_envelope_without_motor_output(pulse):
    drive, backend, _ = ready(SymmetricSteering())
    tune(drive, 1, pulse)
    assert backend.calls[-1] == pulse
    assert drive.status()['motor_pulse_us'] is None
    assert drive.status()['steering_center_us'] == 1610


@pytest.mark.parametrize('pulse', [1349, 1871, True, 1350.0, None])
def test_raw_s3_rejects_outside_requested_envelope(pulse):
    drive, backend, _ = ready(SymmetricSteering())
    with pytest.raises(DriveError):
        tune(drive, 1, pulse)
    assert backend.calls == []


@pytest.mark.parametrize('ending', ['stop', 'emergency', 'timeout', 'two_seconds'])
def test_expanded_raw_target_keeps_stop_lease_and_two_second_limit(ending):
    drive, backend, clock = ready(SymmetricSteering())
    tune(drive, 1, 1350)
    if ending == 'two_seconds':
        for n in range(2,21):
            clock.now = (n-1)*.1
            tune(drive,n,1350)
        clock.now = 2.01
    elif ending == 'timeout':
        clock.now = .21
    else:
        clock.now = .04
        drive.request(ending,{})
    drive.tick()
    assert backend.calls[-1] == 1610
    assert not drive.status()['steering_pwm_active']


def test_transport_extension_is_manual_s3_only():
    board = RasAdapter()
    sent = []
    board.send = lambda function,payload: sent.append((function,payload))
    for pulse in (1350,1870):
        board.set_manual_steering(3,pulse,.3)
    assert sent == [(4,struct.pack('<BHBBH',1,300,1,3,p)) for p in (1350,1870)]
    for channel,pulse in [(1,1350),(2,1870),(4,1500),(3,1349),(3,1871),(3,True)]:
        with pytest.raises(ValueError):
            board.set_manual_steering(channel,pulse)
    for pulse in (1350,1870):
        board.set_position(3,pulse)
    assert len(sent) == 4


def test_guardian_scope_requires_explicit_manual_selection():
    default = RasAdapterPWM(motor=False,channel=3,center_us=1610)
    default.board.send = lambda *args: None
    default.turn(1350)
    with pytest.raises(ValueError):
        default.turn(1349)
    manual = RasAdapterPWM(motor=False,channel=3,center_us=1610,symmetric_manual=True)
    sent = []
    manual.board.send = lambda *args: sent.append(args)
    manual.turn(1350)
    manual.turn(1870)
    assert len(sent) == 2
    with pytest.raises(ValueError):
        manual.turn(1871)
    for channel,center in [(2,1610),(3,1650)]:
        with pytest.raises(ValueError):
            RasAdapterPWM(motor=False,channel=channel,center_us=center,symmetric_manual=True)


def test_review_opt_in_reaches_backend_and_rejects_wrong_scope(tmp_path):
    review = dict(model='Raspberry Pi 5 Model B Rev 1.0',boot_id='boot',
                  wheels_raised_confirmed_by_user=True,esc_off_confirmed_by_user=True,
                  steering_backend='rasadapter5a_uart',steering_channel=3,
                  steering_center_us=1610,manual_steering_span_us=260)
    backend = Pi5BenchBackend(tmp_path,tmp_path/'out','boot',review,steering_only=True)
    assert backend.manual_steering_span_us == 260
    for change in ({'steering_channel':2},{'steering_center_us':1650}):
        with pytest.raises(ValueError):
            Pi5BenchBackend(tmp_path,tmp_path/'out','boot',{**review,**change},steering_only=True)


def test_profile_keeps_unverified_endpoints_and_center():
    profile = json.loads((Path(__file__).parents[1]/'deployment/boot-test-template.json').read_text(encoding='utf-8'))
    assert profile['steering_center_us'] == 1610
    assert profile['manual_steering_span_us'] == 260
    assert profile['manual_endpoints_field_verified'] is False


@pytest.mark.parametrize('direction,pulse', [('left',1870), ('right',1350)])
def test_normal_ramp_reaches_endpoints_without_overshoot(direction, pulse):
    backend = SymmetricSteering()
    backend.steering_single_target = False
    drive, backend, clock = ready(backend)
    for seq in range(1,31):
        clock.now = seq*.025
        drive.request('command',dict(client='pwm-test',sequence=seq,motor='stop',steering=direction))
        drive.tick()
    assert backend.calls[-1] == pulse
    assert all(1350 <= p <= 1870 for p in backend.calls)
    assert all(abs(a-b) <= 10 for a,b in zip([1610]+backend.calls,backend.calls))


def test_new_span_is_forwarded_to_the_separate_guardian(tmp_path, monkeypatch):
    review = dict(model='Raspberry Pi 5 Model B Rev 1.0',boot_id='boot',
                  wheels_raised_confirmed_by_user=True,esc_off_confirmed_by_user=True,
                  steering_backend='rasadapter5a_uart',steering_channel=3,
                  steering_center_us=1610,manual_steering_span_us=260,
                  baseline_record='baseline.json')
    (tmp_path/'baseline.json').write_text(json.dumps(dict(
        boot_id='boot',duration_s=30,all_power_flags_zero=True,boot_unchanged=True,
        samples=[dict(boot_id='boot',power_flags=0)]*30)))
    backend = Pi5BenchBackend(tmp_path,tmp_path/'out','boot',review,steering_only=True)
    monkeypatch.setattr('carvision.pi5_pwm.inspect_pi5',lambda:dict(boot_id='boot',power_flags=0))
    calls = []
    monkeypatch.setattr('carvision.pi5_pwm.subprocess.Popen',lambda command,**kw:calls.append(command))
    monkeypatch.setattr(backend,'read_reply',lambda **kw:None)
    monkeypatch.setattr(backend,'check',lambda:None)
    try:
        backend.open()
    finally:
        backend.log.close()
    assert '--symmetric-manual-steering' in calls[0]
    assert calls[0][calls[0].index('--steering-channel')+1] == '3'
    assert '--motor' not in calls[0] and '--esc-channel' not in calls[0]
