"""Bounded reference/response probe for the user-confirmed original car ESC.

Uses the already tested /home/pi/motor_test.py driver, not race calibration.
The original controller's 50 Hz, 1500 us reset is a reference to verify.
Run under timeout (35s reference, 8s forward/response), with the wheels raised.
"""

import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path


SOURCE_SHA256 = '454c557b00bfeb0d8351b5fa0becbadb990dfff0b4faa725e0d9093deeee3148'
PINS = '12,13,17,27'
CALIBRATION_STATE = Path(__file__).resolve().parents[1] / 'configs/bench-calibration.json'


def execute_responses(motor, pulse, reverse_sequence, initial_flags):
    durations = (.3, .8) if reverse_sequence else (.8,)
    for index, duration in enumerate(durations):
        flags = power_flags()
        if flags & 1 or (flags & 0x10000 and not initial_flags & 0x10000):
            raise RuntimeError('undervoltage before motor command')
        print(f'BRIEF_COMMAND_ACTIVE: {pulse} us for {round(duration*50)} cycles, then queued reset', flush=True)
        motor.start_brief_command(pulse, duration)
        if not motor.wait(duration + .05):
            raise RuntimeError('interrupted during motion')
        print('RESET_QUEUED: observing stop', flush=True)
        if index + 1 < len(durations):
            # Keep the queued neutral waveform continuous between both taps.
            if not motor.wait(.7):
                raise RuntimeError('interrupted before second reverse command')
    motor.wait(1)


def power_flags():
    return int(subprocess.check_output(['vcgencmd', 'get_throttled'], text=True, timeout=1).strip().split('=')[1], 16)


def restore_input(driver):
    # Releasing an lgpio output does not restore its direction on this kernel.
    # Claiming input respects kernel ownership; do not force pinctrl over a user.
    handle = driver.gpiochip_open(0)
    claimed = False
    try:
        if driver.gpio_get_chip_info(handle)[3] != 'pinctrl-bcm2711':
            raise RuntimeError('unexpected controller during cleanup')
        driver.gpio_claim_input(handle, 13)
        claimed = True
        driver.gpio_free(handle, 13)
        claimed = False
    finally:
        try:
            if claimed:
                driver.gpio_free(handle, 13)
        finally:
            driver.gpiochip_close(handle)


def run(args):
    pulse = 1575
    if args.phase in ('response', 'reverse-sequence'):
        pulse = getattr(args, 'pulse_us', None)
        if pulse is None:
            pulse = 1425 if args.phase == 'reverse-sequence' else 1575
    if type(pulse) is not int or pulse not in (1400, 1425, 1450, 1475, 1525, 1550, 1575):
        raise ValueError('response calibration only permits bounded low-command reference points')
    if args.phase == 'reverse-sequence' and pulse >= 1500:
        raise ValueError('reverse sequence requires a small negative command')
    result = dict(phase=args.phase, pulse_output=False, gpio=13, physical_pin=33,
                  reference_us=1500, frequency_hz=50, calibration_completed=False,
                  esc_model_confirmed=False, mechanical_movement_verified=False)
    if args.phase != 'reference':
        result['test_pulse_us'] = pulse
        result['command_durations_s'] = [.3, .8] if args.phase == 'reverse-sequence' else [.8]
        if args.phase == 'reverse-sequence':
            result['continuous_neutral_gap_s'] = .7
    if not args.run:
        return result
    if not args.original_esc_and_battery or not args.bench_prepared:
        raise ValueError('original ESC/battery and prepared raised-wheel test must be confirmed')
    if args.phase != 'reference' and not args.reference_verified:
        raise ValueError('observe a successful original reset reference before forward testing')
    if CALIBRATION_STATE.exists():
        state = json.loads(CALIBRATION_STATE.read_text(encoding='utf-8-sig'))
        if state.get('restart_review', {}).get('further_motion_paused'):
            raise RuntimeError('unexplained restart recorded; resolve restart review before further motion')
        if args.phase != 'reference' and pulse < 1500 and state.get('esc', {}).get('reverse_test_paused'):
            raise RuntimeError('reverse response unverified; inspect ESC operating mode before further reverse tests')
    if sys.platform != 'linux' or b'Raspberry Pi 4 Model B' not in Path('/proc/device-tree/model').read_bytes():
        raise RuntimeError('requires the prepared Raspberry Pi 4B')
    parent = [part.decode() for part in Path(f'/proc/{os.getppid()}/cmdline').read_bytes().split(b'\0') if part]
    duration = '35s' if args.phase == 'reference' else '8s'
    if not parent or Path(parent[0]).name != 'timeout' or duration not in parent or '--kill-after=1s' not in parent or '--signal=TERM' not in parent:
        raise RuntimeError('run under the required independent timeout')
    source = Path('/home/pi/motor_test.py')
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise RuntimeError('existing motor helper changed; inspect before testing')
    spec = importlib.util.spec_from_file_location('verified_motor_helper', source)
    motor_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(motor_module)
    before = subprocess.check_output(['pinctrl', 'get', PINS], text=True, timeout=1)
    if len(before.splitlines()) != 4 or any('= input' not in line for line in before.splitlines()):
        raise RuntimeError('reserved GPIO already has an output owner')
    flags = power_flags()
    if flags & 1:
        raise RuntimeError('current undervoltage; no output sent')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    motor = motor_module.MotorTest()
    try:
        motor.open()
        motor.set_pulse(1500)
        result['pulse_output'] = True
        result['source_sha256'] = SOURCE_SHA256
        print('RESET_REFERENCE_ACTIVE: 1500 us / 50 Hz on BCM13', flush=True)
        if args.phase == 'reference':
            # Human turns on the ESC during this window and watches the wheels.
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline and not motor.stop_requested:
                if power_flags() & 1:
                    raise RuntimeError('undervoltage during reference test')
                motor.wait(.25)
        else:
            if not motor.wait(2):
                raise RuntimeError('interrupted before motion')
            if power_flags() & 1:
                raise RuntimeError('undervoltage before motion')
            result['test_pulse_us'] = pulse
            result['motion_duration_s'] = .8
            execute_responses(motor, pulse, args.phase == 'reverse-sequence', flags)
        result['interrupted'] = motor.stop_requested
    except BaseException as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        result['cleanup_errors'] = motor.shutdown()
        # The motor helper relinquishes its kernel claim before this input claim.
        if motor.driver is not None:
            try:
                restore_input(motor.driver)
                result['input_restore_requested'] = True
            except Exception as exc:
                result['cleanup_errors'].append(f'input restore: {exc}')
        after = subprocess.check_output(['pinctrl', 'get', PINS], text=True, timeout=1)
        result['pins_before'], result['pins_after'] = before, after
        result['pins_are_inputs'] = len(after.splitlines()) == 4 and all('= input' in line for line in after.splitlines())
        result['other_pins_preserved'] = all(a == b for a, b in zip(before.splitlines(), after.splitlines()) if 'GPIO13 =' not in a)
        after_flags = power_flags()
        result['power_flags_before'], result['power_flags_after'] = hex(flags), hex(after_flags)
        result['undervoltage_detected'] = bool(after_flags & 1 or (after_flags & 0x10000 and not flags & 0x10000))
        result['note'] = 'Original ESC/battery and bench setup confirmed by human; reset and actual movement require observation.'
        temporary = output / 'result.json.tmp'
        with temporary.open('w', encoding='utf-8') as record:
            record.write(json.dumps(result, indent=2)+'\n')
            record.flush()
            os.fsync(record.fileno())
        temporary.replace(output / 'result.json')
        for directory in (output, output.parent):
            handle = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(handle)
            finally:
                os.close(handle)
        print(json.dumps(result, indent=2), flush=True)
    if result['cleanup_errors'] or not result['pins_are_inputs'] or not result['other_pins_preserved'] or result['undervoltage_detected']:
        raise RuntimeError('post-test check failed; inspect before further motion')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=('reference', 'forward', 'response', 'reverse-sequence'), required=True)
    parser.add_argument('--pulse-us', type=int)
    parser.add_argument('--output', required=True)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--original-esc-and-battery', action='store_true')
    parser.add_argument('--bench-prepared', action='store_true')
    parser.add_argument('--reference-verified', action='store_true')
    args = parser.parse_args()
    if not args.run:
        print(json.dumps(run(args), indent=2))
    else:
        run(args)


if __name__ == '__main__':
    main()
