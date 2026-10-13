"""Parking session authority, neutral preparation and independent F/B guard."""
import copy
import json
from pathlib import Path

import pytest

from carvision.parking_entry_run import ParkingEntryRun, parking_base_idle
from carvision.cone_trial_output import ConePulseGuard


CONFIG = Path(__file__).resolve().parents[1] / 'configs'


def configurations():
    return tuple(json.loads((CONFIG / filename).read_text(encoding='utf-8-sig')) for filename in (
        'parking-bay.json', 'brake-parking.json', 'cone-trial.json'))


def base():
    return {'mode': 'disabled', 'boot_test_phase': 'esc_off', 'setup_token': 'page-setup',
            'hardware_output': False, 'worker_started': False, 'pending_hardware_start': False}


def preparation(**changes):
    return {'setup_token': 'page-setup', 'client': 'page', 'esc_off': True,
            'fb_mode_confirmed': True, 'selected_slot': 'right', **changes}


def command(run, sequence, **changes):
    return {'client': 'page', 'run_token': run.token, 'trial_sequence': sequence, **changes}


def sample(t, frame=None, **changes):
    # This deliberately unconfirmed right bay still provides an actual quad.
    return {'fresh': True, 'camera_id': 'secondary', 'frame_id': frame if frame is not None else round(t * 1000),
            'captured_s': t, 'image_size': [480, 360],
            'parking': {'slots': [{'id': 'right', 'availability': 'unknown',
                                  'corners_px': [[110, 310], [380, 310], [330, 170], [150, 170]]}]},
            'motion': {'valid': True, 'moving': False}, **changes}


class Actuator:
    run = False

    def __init__(self):
        self.commands = []
        self.closed = False

    def send(self, intent):
        self.commands.append(copy.deepcopy(intent))
        return {'hardware_output': False}

    def close(self):
        self.closed = True


def prepared(tmp_path=None, **changes):
    actuator = Actuator()
    run = ParkingEntryRun(lambda _: actuator, tmp_path)
    run.prepare(*configurations(), base(), preparation(**changes), 0.)
    run.tick(sample(.1), .1)
    return run, actuator


def start(run, now=.2):
    run.command('parking_trial_start', command(run, 0, fb_mode_confirmed=True,
                esc_on_static=True, field_ready=True), now)


def test_prepare_and_start_are_explicit_neutral_only_then_single_three_second_arming(tmp_path):
    opened = []
    actuator = Actuator()
    def factory(settings):
        opened.append(copy.deepcopy(settings))
        return actuator
    run = ParkingEntryRun(factory, tmp_path)
    p, b, c = configurations()
    originals = copy.deepcopy((p, b, c))
    run.prepare(p, b, c, base(), preparation(), 0.)
    assert not opened
    run.tick(sample(.1), .1)
    assert opened[0]['pwm'] == {'neutral_us': 1500, 'steering_center_us': 1610,
                               'search_us': 1575, 'creep_us': 1575, 'brake_us': 1400}
    assert opened[0]['actuator_lease_s'] == .2 and opened[0]['brake_pulse_s'] == .12
    assert actuator.commands[-1]['action'] == 'neutral'
    assert run.status(base(), .15)['can_start']
    start(run)
    for i in range(1, 35):
        t = .2 + i * .1
        run.command('parking_trial_keepalive', command(run, i), t)
        run.tick(sample(t), t)
        assert run.active, run.reason
        if i <= 30:
            assert actuator.commands[-1]['esc_us'] == 1500
    assert actuator.commands[-1]['action'] == 'search'
    assert actuator.commands[-1]['esc_us'] == 1575
    assert (p, b, c) == originals
    report = run.status(base(), 3.6)
    assert report['field_verified'] is False and report['parking_zone_verified'] is False
    assert report['actual_pose_not_verified'] is True
    run.cancel('operator stopped')
    assert actuator.closed and not run.active and run.token is None


@pytest.mark.parametrize('other', [
    {'mode': 'manual'}, {'boot_test_phase': 'ready'}, {'hardware_output': True},
    {'worker_started': True}, {'pending_hardware_start': True},
    {'crosswalk_trial': {'active': True}}, {'traffic_trial': {'active': True}},
    {'departure_trial': {'active': True}}, {'race_flow': {'active': True}},
    {'cone_trial': {'avoidance': {'active': True}}},
    {'cone_trial': {'short_test': {'active': True}}},
    {'parking_trial': {'active': True}}, {'parking_payment': {'motion_locked': True}},
])
def test_other_output_owners_block_preparation(other):
    state = {**base(), **other}
    run = ParkingEntryRun(lambda _: pytest.fail('must not acquire output'))
    assert not parking_base_idle(state)
    assert not run.status(state, 0.)['can_prepare']
    with pytest.raises(ValueError, match='先结束'):
        run.prepare(*configurations(), state, preparation(), 0.)


def test_parking_observation_is_not_a_motor_owner():
    assert parking_base_idle({**base(), 'parking_bay': {'active': True},
                              'cone_trial': {'observing': True}})


@pytest.mark.parametrize('change', [
    {'setup_token': 'old'}, {'client': ''}, {'esc_off': False}, {'fb_mode_confirmed': False},
    {'selected_slot': 'middle'}, {'forward_us': 1500}, {'forward_us': 1626},
    {'forward_us': True}, {'forward_us': 1575.5}, {'steering_gain': 0},
    {'steering_gain': float('nan')},
])
def test_invalid_prepare_has_no_session_or_hardware(change):
    run = ParkingEntryRun(lambda _: pytest.fail('must not acquire output'))
    with pytest.raises(ValueError):
        run.prepare(*configurations(), base(), preparation(**change), 0.)
    assert not run.active


@pytest.mark.parametrize('forward', [1501, 1575, 1625])
def test_selected_forward_pulse_is_snapshotted_and_bounded(forward):
    run, actuator = prepared(forward_us=forward)
    assert run.s['brake']['pwm']['search_us'] == forward
    assert run.status(base(), .1)['forward_us'] == forward
    assert actuator.commands[-1]['esc_us'] == 1500


@pytest.mark.parametrize('change', [{'client': 'foreign'}, {'run_token': 'wrong'},
                                    {'trial_sequence': True}, {'trial_sequence': -1}])
def test_owner_and_sequence_gate_start(change):
    run, actuator = prepared()
    with pytest.raises(ValueError):
        run.command('parking_trial_start', command(run, 0, fb_mode_confirmed=True,
                    esc_on_static=True, field_ready=True, **change), .2)
    assert run.controller is None and all(c['esc_us'] == 1500 for c in actuator.commands)


def test_repeat_keepalive_cannot_extend_lease_and_late_start_is_invalid():
    run, actuator = prepared()
    run.command('parking_trial_keepalive', command(run, 0), .2)
    deadline = run.lease_until
    with pytest.raises(ValueError):
        run.command('parking_trial_keepalive', command(run, 0), .3)
    assert run.lease_until == deadline
    old_token = run.token
    run.command('parking_trial_cancel', command(run, 1), .4)
    with pytest.raises(ValueError):
        run.command('parking_trial_start', {'client': 'page', 'run_token': old_token,
                    'trial_sequence': 2, 'fb_mode_confirmed': True,
                    'esc_on_static': True, 'field_ready': True}, .45)
    assert actuator.closed and run.token is None


@pytest.mark.parametrize('case', ['page_lease', 'camera_age', 'camera_failure', 'wrong_camera', 'wrong_size'])
def test_stale_camera_or_page_releases_actuator_and_records_failed_sample(tmp_path, case):
    run, actuator = prepared(tmp_path)
    bad = sample(.2, diagnostic_token={'run_token': 'never-record-this'})
    now = .2
    if case == 'page_lease':
        now = .8
    elif case == 'camera_age':
        bad['captured_s'] = -.1
    elif case == 'camera_failure':
        bad['fresh'] = False
    elif case == 'wrong_camera':
        bad['camera_id'] = 'primary'
    elif case == 'wrong_size':
        bad['image_size'] = [640, 480]
    run.tick(bad, now)
    assert run.phase == 'fault' and not run.active and actuator.closed
    summary = json.loads((run.folder / 'summary.json').read_text(encoding='utf-8'))
    assert summary['failure_sample']['frame_id'] == bad['frame_id']
    assert 'never-record-this' not in (run.folder / 'summary.json').read_text(encoding='utf-8')
    assert all('run_token' not in f.read_text(encoding='utf-8') for f in run.folder.iterdir())


def test_cancel_during_slow_initialization_releases_returned_handle():
    actuator = Actuator()
    run = ParkingEntryRun()
    def factory(settings):
        run.cancel('operator cancelled while preparing')
        return actuator
    run.factory = factory
    run.prepare(*configurations(), base(), preparation(), 0.)
    run.tick(sample(.1), .1)
    assert actuator.closed and not actuator.commands and not run.active


def test_initialization_waits_for_real_frame_without_opening_output():
    opened = []
    run = ParkingEntryRun(lambda _: opened.append(True))
    run.prepare(*configurations(), base(), preparation(), 0.)
    run.tick(None, .1)
    assert run.active and not opened
    for i in range(1, 53):
        t = i * .1
        run.command('parking_trial_keepalive', command(run, i), t)
        run.tick(None, t)
        if not run.active:
            break
    assert not opened and run.phase == 'fault'


def test_done_closes_output_without_claiming_verified_parking():
    run, actuator = prepared()
    class Finished:
        def step(self, sample, now):
            return {'phase': 'done', 'reason': 'image stop estimate', 'action': 'neutral',
                    'esc_us': 1500, 'steering_us': 1610}
    run.controller = Finished()
    run.tick(sample(.2), .2)
    result = run.status(base(), .2)
    assert not result['active'] and result['phase'] == 'done' and actuator.closed
    assert not result['parking_zone_verified'] and result['actual_pose_not_verified']


def test_output_error_closes_session():
    run, actuator = prepared()
    def failing(intent):
        raise RuntimeError('independent output lease expired')
    actuator.send = failing
    run.tick(sample(.2), .2)
    assert run.phase == 'fault' and actuator.closed
    assert 'lease expired' in run.reason


def test_independent_guard_caps_brake_duration_even_with_repeated_frames():
    run, _ = prepared()
    writes = []
    guard = ConePulseGuard(run.s['brake'], lambda channel, pulse: writes.append((channel, pulse)), mode='F/B')
    def send(seq, t, action, pulse, steering=1610):
        guard.accept({'sequence': seq, 'sent_s': t, 'action': action,
                      'esc_us': pulse, 'steering_us': steering}, t)
    send(0, 0., 'search', 1575, 1350)
    send(1, .1, 'brake', 1400, 1870)
    send(2, .15, 'brake', 1400, 1870)
    send(3, .23, 'brake', 1400, 1870)
    assert guard.brake_until is None and guard.esc == 1500
    assert (3, 1350) in writes and (3, 1870) in writes
    send(4, .3, 'search', 1575)
    guard.tick(.51)
    assert guard.fault == 'command lease expired' and guard.esc == 1400
    guard.tick(.64)
    assert guard.esc == 1500


def test_missing_start_confirmations_never_create_controller():
    run, _ = prepared()
    with pytest.raises(ValueError, match='场地就绪'):
        run.command('parking_trial_start', command(run, 0), .2)
    assert run.controller is None


def test_duplicate_fresh_frame_renews_output_without_accumulating_entry_evidence():
    run, actuator = prepared()
    class CountingController:
        def __init__(self):
            self.steps = 0
        def step(self, view, now):
            self.steps += 1
            return {'phase': 'approach', 'reason': 'steer to selected bay', 'action': 'search',
                    'esc_us': 1575, 'steering_us': 1500}
    controller = CountingController()
    run.controller = controller
    view = sample(.2)
    run.tick(view, .2)
    run.tick(view, .3)
    run.tick(view, .4)
    assert run.active and controller.steps == 1
    assert len(actuator.commands) == 4
    assert all(c['esc_us'] == 1575 for c in actuator.commands[1:])
    run.tick(view, .46)
    assert run.phase == 'fault' and actuator.closed and controller.steps == 1


def test_brake_duplicate_retransmits_without_restarting_controller_pulse():
    run, actuator = prepared()
    class Braking:
        def __init__(self):
            self.steps = 0
        def step(self, view, now):
            self.steps += 1
            return {'phase': 'brake_pulse', 'reason': 'finite F/B brake', 'action': 'brake',
                    'esc_us': 1400, 'steering_us': 1610}
    controller = Braking()
    run.controller = controller
    view = sample(.2)
    run.tick(view, .2)
    run.tick(view, .3)
    assert controller.steps == 1
    assert actuator.commands[-1]['action'] == 'brake'
    run.tick(sample(.4), .4)
    assert controller.steps == 2


def test_new_frame_with_regressed_capture_is_not_treated_as_repeat():
    run, actuator = prepared()
    start(run)
    run.tick(sample(.2), .2)
    run.tick(sample(.19, frame=201), .3)
    assert run.phase == 'fault' and actuator.closed


def test_blocked_bay_cannot_become_success_by_session_logic():
    run, actuator = prepared()
    start(run)
    for i in range(1, 36):
        t = .2 + .1 * i
        run.command('parking_trial_keepalive', command(run, i), t)
        view = sample(t)
        view['parking']['slots'][0]['availability'] = 'blocked'
        run.tick(view, t)
        if not run.active:
            break
    assert run.phase == 'fault' and actuator.closed
    assert not any(c['esc_us'] > 1500 for c in actuator.commands)
