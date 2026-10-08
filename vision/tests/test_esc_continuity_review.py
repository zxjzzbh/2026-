import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


folder = Path(__file__).parents[1] / 'tools'
sys.path.insert(0, str(folder))
spec = importlib.util.spec_from_file_location('esc_continuity_review', folder / 'esc_continuity_review.py')
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)


def test_preview_and_missing_preparation_do_not_access_hardware(monkeypatch):
    monkeypatch.setattr(review.bench, 'power_flags', lambda: pytest.fail('hardware access'))
    assert review.run(SimpleNamespace(run=False))['commands'] == []
    with pytest.raises(ValueError, match='preparation'):
        review.run(SimpleNamespace(run=True, bench_prepared=True, original_esc_and_battery=True,
                                   esc_off_confirmed=True, user_review_authorized=False))


def test_cancelled_operator_gate_never_runs_next_command(monkeypatch):
    monkeypatch.setattr(review, 'check_power', lambda initial: None)
    motor = Mock()
    with pytest.raises(RuntimeError, match='cancelled'):
        review.await_operator(motor, 'REVERSE', 0, reader=lambda: 'STOP')
    motor.start_brief_command.assert_not_called()


def test_operator_timeout_and_interrupt_stop_motion(monkeypatch):
    monkeypatch.setattr(review, 'check_power', lambda initial: None)
    ticks = iter([0, 31])
    with pytest.raises(RuntimeError, match='timed out'):
        review.await_operator(Mock(), 'ON', 0, reader=lambda: None, clock=lambda: next(ticks))
    motor = Mock()
    motor.wait.return_value = False
    with pytest.raises(RuntimeError, match='interrupted'):
        review.await_operator(motor, 'ON', 0, reader=lambda: None)
    motor.start_brief_command.assert_not_called()


def test_forward_observation_gate_blocks_reverse(monkeypatch):
    def gate(motor, token, flags):
        if token == 'REVERSE':
            raise RuntimeError('cancelled')
    monkeypatch.setattr(review, 'await_operator', gate)
    monkeypatch.setattr(review, 'hold', lambda *args: None)
    monkeypatch.setattr(review, 'check_power', lambda initial: None)
    motor = Mock()
    with pytest.raises(RuntimeError, match='cancelled'):
        review.review(motor, 0, [])
    motor.start_brief_command.assert_called_once_with(1575, .4)


def test_new_power_fault_prevents_command(monkeypatch):
    monkeypatch.setattr(review.bench, 'power_flags', lambda: 0x10000)
    motor = Mock()
    with pytest.raises(RuntimeError, match='undervoltage'):
        review.command(motor, 1400, .8, 0, [])
    motor.start_brief_command.assert_not_called()


@pytest.mark.parametrize('pulse,duration', [(1250, .8), (1400, 2), (1600, .4), (1400, .4)])
def test_larger_or_unplanned_commands_rejected(pulse, duration):
    motor = Mock()
    with pytest.raises(ValueError, match='fixed small commands'):
        review.command(motor, pulse, duration, 0, [])
    motor.start_brief_command.assert_not_called()


def test_retry_needs_same_boot_actual_forward_observation(tmp_path):
    result = {'boot_id_before': 'current', 'boot_id_after': 'current',
              'pins_are_inputs': True, 'cleanup_errors': [], 'undervoltage_detected': False,
              'commands': [{'requested_pulse_us': 1575, 'duration_s': .4}]}
    observation = {'source': 'user_message', 'boot_id': 'current',
                   'forward_movement_and_stop_confirmed': False}
    (tmp_path / 'result.json').write_text(json.dumps(result))
    (tmp_path / 'observation.json').write_text(json.dumps(observation))
    with pytest.raises(RuntimeError, match='actual user observation'):
        review.validate_observed_forward_retry(tmp_path, 'current')
    observation['forward_movement_and_stop_confirmed'] = True
    (tmp_path / 'observation.json').write_text(json.dumps(observation))
    review.validate_observed_forward_retry(tmp_path, 'current')
    with pytest.raises(RuntimeError, match='same-boot'):
        review.validate_observed_forward_retry(tmp_path, 'changed')
    (tmp_path / 'retry-consumed.json').write_text('{}')
    with pytest.raises(RuntimeError, match='already been consumed'):
        review.validate_observed_forward_retry(tmp_path, 'current')


def test_retry_does_not_wait_for_operator_but_rechecks_power_before_reverse(monkeypatch):
    monkeypatch.setattr(review, 'await_operator', lambda *args: pytest.fail('unexpected operator gate'))
    monkeypatch.setattr(review, 'hold', lambda *args: None)
    checks = iter([None, RuntimeError('new undervoltage')])
    def power(initial):
        value = next(checks)
        if value:
            raise value
    monkeypatch.setattr(review, 'check_power', power)
    motor = Mock()
    with pytest.raises(RuntimeError, match='undervoltage'):
        review.review(motor, 0, [], observed_forward_retry=True)
    motor.start_brief_command.assert_called_once_with(1575, .4)


def test_mode_review_requires_actual_labels_selection_and_power_off():
    state = {'esc': {'operating_mode_label': 'F/B/R',
                     'operating_mode_confirmed_by_user': True,
                     'operating_mode_label_verified_from_photo': True,
                     'powered_off_confirmed_by_user': True}}
    review.validate_mode_change(state, True, None)
    with pytest.raises(RuntimeError, match='photographed labels'):
        review.validate_mode_change(state, False, None)
    state['esc']['operating_mode_label'] = 'F/B'
    with pytest.raises(RuntimeError, match='photographed labels'):
        review.validate_mode_change(state, True, None)


def test_mode_review_never_moves_before_fresh_power_on_observation(monkeypatch):
    def cancelled(motor, token, flags, **kwargs):
        assert token == 'ON' and kwargs['timeout_s'] == 60
        raise RuntimeError('operator cancelled review')
    monkeypatch.setattr(review, 'await_operator', cancelled)
    motor = Mock()
    with pytest.raises(RuntimeError, match='cancelled'):
        review.review(motor, 0, [], mode_changed=True)
    motor.start_brief_command.assert_not_called()


def test_next_reverse_step_requires_actual_preceding_failure_and_same_boot():
    state = {'esc': {'last_mode_reverse_observation': {'pulse_us': 1400,
                     'source': 'user_message', 'reverse_rotated': False,
                     'forward_rotated': True, 'boot_id': 'current'}}}
    review.validate_reverse_step(state, 1350, True, 'current')
    for pulse, mode, boot in [(1300, True, 'current'), (1350, False, 'current'),
                              (1350, True, 'changed')]:
        with pytest.raises(RuntimeError, match='50-us step'):
            review.validate_reverse_step(state, pulse, mode, boot)
    state['esc']['last_mode_reverse_observation']['reverse_rotated'] = True
    with pytest.raises(RuntimeError, match='50-us step'):
        review.validate_reverse_step(state, 1350, True, 'current')


@pytest.mark.parametrize('pulse', [1250, 1375, 1500, 1000, True])
def test_unplanned_reverse_amplitudes_never_access_hardware(pulse):
    with pytest.raises(ValueError, match='bounded'):
        review.run(SimpleNamespace(run=True, reverse_pulse_us=pulse))


def test_reverse_only_never_sends_any_forward_pulse(monkeypatch):
    monkeypatch.setattr(review, 'hold', lambda *args: None)
    monkeypatch.setattr(review, 'check_power', lambda initial: None)
    monkeypatch.setattr(review, 'await_operator', lambda *args: pytest.fail('fresh on confirmation already recorded'))
    motor = Mock()
    trace = []
    review.review(motor, 0, trace, mode_changed=True, reverse_pulse=1300,
                  reverse_only=True, already_on=True)
    assert [p['requested_pulse_us'] for p in trace] == [1300, 1300]
    assert all(c.args[0] < 1500 for c in motor.start_brief_command.call_args_list)


def test_already_on_mode_review_needs_actual_on_preparation():
    state = {'esc': {'operating_mode_label': 'F/B/R',
                     'operating_mode_confirmed_by_user': True,
                     'operating_mode_label_verified_from_photo': True,
                     'powered_off_confirmed_by_user': False}}
    with pytest.raises(RuntimeError, match='user-confirmed'):
        review.validate_mode_change(state, False, None, already_on=True)
    state['esc']['latest_on_confirmation'] = {'on_and_static_confirmed': True}
    review.validate_mode_change(state, False, None, already_on=True)
