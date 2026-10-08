import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, call

import pytest


spec = importlib.util.spec_from_file_location('esc_bench_wrapper', Path(__file__).parents[1] / 'tools/bench_original_esc.py')
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


def driver():
    result = Mock()
    result.gpiochip_open.return_value = 7
    result.gpio_get_chip_info.return_value = [0, 58, 'gpiochip0', 'pinctrl-bcm2711']
    return result


def test_restoring_input_only_claims_esc_and_releases_handle():
    backend = driver()
    bench.restore_input(backend)
    assert backend.mock_calls == [call.gpiochip_open(0), call.gpio_get_chip_info(7),
                                  call.gpio_claim_input(7, 13), call.gpio_free(7, 13),
                                  call.gpiochip_close(7)]


def test_busy_esc_pin_is_preserved_and_handle_still_closed():
    backend = driver()
    backend.gpio_claim_input.side_effect = RuntimeError('busy')
    with pytest.raises(RuntimeError, match='busy'):
        bench.restore_input(backend)
    backend.gpio_free.assert_not_called()
    backend.gpiochip_close.assert_called_once_with(7)


def test_wrong_gpio_controller_is_not_modified():
    backend = driver()
    backend.gpio_get_chip_info.return_value[3] = 'other-controller'
    with pytest.raises(RuntimeError, match='controller'):
        bench.restore_input(backend)
    backend.gpio_claim_input.assert_not_called()
    backend.gpiochip_close.assert_called_once_with(7)


def test_handle_close_is_attempted_even_when_free_fails():
    backend = driver()
    backend.gpio_free.side_effect = RuntimeError('release failure')
    with pytest.raises(RuntimeError, match='release failure'):
        bench.restore_input(backend)
    backend.gpiochip_close.assert_called_once_with(7)


def test_dry_forward_and_unverified_reference_do_not_access_hardware(monkeypatch):
    monkeypatch.setattr(bench.subprocess, 'check_output', lambda *a, **k: pytest.fail('hardware access'))
    assert not bench.run(SimpleNamespace(phase='forward', run=False))['pulse_output']
    with pytest.raises(ValueError, match='reset reference'):
        bench.run(SimpleNamespace(phase='forward', run=True, original_esc_and_battery=True,
                                  bench_prepared=True, reference_verified=False))


@pytest.mark.parametrize('pulse', [1000, 1375, 1500, 1600, 2000, True, 1425.0])
def test_unobserved_larger_motor_commands_rejected_before_host_access(pulse):
    with pytest.raises(ValueError, match='bounded low-command'):
        bench.run(SimpleNamespace(phase='response', pulse_us=pulse, run=True))


def test_negative_response_preview_does_not_access_hardware():
    result = bench.run(SimpleNamespace(phase='response', pulse_us=1425, run=False))
    assert result['test_pulse_us'] == 1425 and not result['pulse_output']


def test_restart_blocks_motor_output_before_hardware_access(tmp_path, monkeypatch):
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'restart_review': {'further_motion_paused': True}}))
    monkeypatch.setattr(bench, 'CALIBRATION_STATE', state)
    monkeypatch.setattr(bench.subprocess, 'check_output',
                        lambda *a, **k: pytest.fail('hardware access after unexplained restart'))
    with pytest.raises(RuntimeError, match='unexplained restart'):
        bench.run(SimpleNamespace(phase='response', pulse_us=1425, run=True,
                                  original_esc_and_battery=True, bench_prepared=True,
                                  reference_verified=True))


@pytest.mark.parametrize('pulse', [None, 1400, 1425])
def test_reverse_sequence_preview_uses_only_small_negative_command(pulse):
    r = bench.run(SimpleNamespace(phase='reverse-sequence', pulse_us=pulse, run=False))
    assert r['test_pulse_us'] == (1425 if pulse is None else pulse)
    assert r['command_durations_s'] == [.3, .8]
    assert not r['pulse_output']


@pytest.mark.parametrize('pulse', [1525, 1575])
def test_reverse_sequence_cannot_repeat_forward_commands(pulse):
    with pytest.raises(ValueError, match='negative command'):
        bench.run(SimpleNamespace(phase='reverse-sequence', pulse_us=pulse, run=True))


def test_two_reverse_taps_preserve_a_neutral_gap(monkeypatch):
    monkeypatch.setattr(bench, 'power_flags', lambda: 0)
    motor = Mock()
    motor.wait.return_value = True
    bench.execute_responses(motor, 1425, True, 0)
    assert motor.mock_calls == [call.start_brief_command(1425, .3), call.wait(.35),
                               call.wait(.7), call.start_brief_command(1425, .8),
                               call.wait(pytest.approx(.85)), call.wait(1)]


def test_interrupted_neutral_gap_never_sends_second_reverse_tap(monkeypatch):
    monkeypatch.setattr(bench, 'power_flags', lambda: 0)
    motor = Mock()
    motor.wait.side_effect = [True, False]
    with pytest.raises(RuntimeError, match='before second reverse'):
        bench.execute_responses(motor, 1425, True, 0)
    motor.start_brief_command.assert_called_once_with(1425, .3)


def test_new_power_fault_during_sequence_blocks_second_tap(monkeypatch):
    readings = iter([0, 0x10000])
    monkeypatch.setattr(bench, 'power_flags', lambda: next(readings))
    motor = Mock()
    motor.wait.return_value = True
    with pytest.raises(RuntimeError, match='undervoltage'):
        bench.execute_responses(motor, 1425, True, 0)
    motor.start_brief_command.assert_called_once_with(1425, .3)


@pytest.mark.parametrize('phase', ['response', 'reverse-sequence'])
def test_unresolved_reverse_failure_blocks_only_reverse_tests(tmp_path, monkeypatch, phase):
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'esc': {'reverse_test_paused': True}}))
    monkeypatch.setattr(bench, 'CALIBRATION_STATE', state)
    with pytest.raises(RuntimeError, match='inspect ESC operating mode'):
        bench.run(SimpleNamespace(phase=phase, pulse_us=1425, run=True,
                                  original_esc_and_battery=True, bench_prepared=True,
                                  reference_verified=True))


def test_reverse_failure_does_not_block_forward_checks(tmp_path, monkeypatch):
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'esc': {'reverse_test_paused': True}}))
    monkeypatch.setattr(bench, 'CALIBRATION_STATE', state)
    monkeypatch.setattr(bench.sys, 'platform', 'win32')
    with pytest.raises(RuntimeError, match='requires the prepared'):
        bench.run(SimpleNamespace(phase='response', pulse_us=1575, run=True,
                                  original_esc_and_battery=True, bench_prepared=True,
                                  reference_verified=True))
