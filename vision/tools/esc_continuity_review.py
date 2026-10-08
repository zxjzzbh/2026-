"""One operator-gated ESC review using only previously tried small signals.

Neutral stays active from switch-on through the forward and reverse checks.
This does not program the ESC or clear the persistent reverse-test pause.
"""

import argparse
import hashlib
import importlib.util
import json
import os
import select
import subprocess
import sys
import time
from pathlib import Path

import bench_original_esc as bench


def check_power(initial_flags):
    flags = bench.power_flags()
    if flags & 1 or (flags & 0x10000 and not initial_flags & 0x10000):
        raise RuntimeError('new undervoltage; review stopped')


def hold(motor, seconds, initial_flags):
    remaining = seconds
    while remaining > 0:
        check_power(initial_flags)
        step = min(.25, remaining)
        if not motor.wait(step):
            raise RuntimeError('review interrupted')
        remaining -= step


def read_operator():
    if select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline()
        if not line:
            raise RuntimeError('operator input closed')
        return line.strip()
    return None


def await_operator(motor, token, initial_flags, reader=read_operator, clock=time.monotonic, timeout_s=30):
    deadline = clock() + timeout_s
    print(f'WAIT_OPERATOR: {token}; neutral only; {timeout_s}-second deadline', flush=True)
    while clock() < deadline:
        check_power(initial_flags)
        reply = reader()
        if reply is not None:
            if reply != token:
                raise RuntimeError('operator cancelled review')
            return
        if not motor.wait(.1):
            raise RuntimeError('review interrupted while waiting')
    raise RuntimeError('operator confirmation timed out; no next motion')


def command(motor, pulse, duration, initial_flags, trace):
    if (pulse, duration) not in ((1575, .4), (1400, .3), (1400, .8),
                                 (1350, .3), (1350, .8), (1300, .3), (1300, .8)):
        raise ValueError('review only permits its three fixed small commands')
    check_power(initial_flags)
    trace.append({'requested_pulse_us': pulse, 'duration_s': duration})
    print(f'BRIEF_COMMAND: {pulse} us for {duration} s; neutral queued', flush=True)
    motor.start_brief_command(pulse, duration)
    hold(motor, duration + .05, initial_flags)


def review(motor, initial_flags, trace, observed_forward_retry=False, mode_changed=False, reverse_pulse=1400, reverse_only=False, already_on=False):
    if not observed_forward_retry and not already_on:
        if mode_changed:
            await_operator(motor, 'ON', initial_flags, timeout_s=60)
        else:
            await_operator(motor, 'ON', initial_flags)
    print('NEUTRAL_SETTLING: 12 seconds of continuous neutral before motion', flush=True)
    hold(motor, 12, initial_flags)
    if not reverse_only:
        if not observed_forward_retry and not mode_changed:
            await_operator(motor, 'FORWARD', initial_flags)
        command(motor, 1575, .4, initial_flags, trace)
        hold(motor, 2, initial_flags)
        print('FORWARD_FINISHED: neutral active; confirmed F/B/R mode review continues'
              if mode_changed else
              ('FORWARD_FINISHED: neutral active; previous actual movement/stop confirmed'
               if observed_forward_retry else
               'FORWARD_FINISHED: confirm actual forward movement and stop before REVERSE'), flush=True)
        if not observed_forward_retry and not mode_changed:
            await_operator(motor, 'REVERSE', initial_flags)
    else:
        print('REVERSE_ONLY: no forward command in this trial', flush=True)
    command(motor, reverse_pulse, .3, initial_flags, trace)
    hold(motor, 1.2, initial_flags)
    command(motor, reverse_pulse, .8, initial_flags, trace)
    hold(motor, 1, initial_flags)


def validate_observed_forward_retry(record_path, boot):
    folder = Path(record_path)
    if (folder / 'retry-consumed.json').exists():
        raise RuntimeError('this observed-forward retry has already been consumed')
    prior = json.loads((folder / 'result.json').read_text(encoding='utf-8'))
    observation = json.loads((folder / 'observation.json').read_text(encoding='utf-8'))
    if (prior.get('boot_id_before') != boot or prior.get('boot_id_after') != boot
            or not prior.get('pins_are_inputs') or prior.get('cleanup_errors')
            or prior.get('undervoltage_detected')
            or prior.get('commands') != [{'requested_pulse_us': 1575, 'duration_s': .4}]
            or observation.get('source') != 'user_message'
            or observation.get('boot_id') != boot
            or observation.get('forward_movement_and_stop_confirmed') is not True):
        raise RuntimeError('retry requires the same-boot clean forward trial and actual user observation')


def validate_mode_change(state, off_confirmed, retry, already_on=False):
    esc = state.get('esc', {})
    confirmed_power = (esc.get('latest_on_confirmation', {}).get('on_and_static_confirmed') is True
                       if already_on else esc.get('powered_off_confirmed_by_user') is True)
    if (not (off_confirmed or already_on) or retry or esc.get('operating_mode_label') != 'F/B/R'
            or esc.get('operating_mode_confirmed_by_user') is not True
            or esc.get('operating_mode_label_verified_from_photo') is not True
            or not confirmed_power):
        raise RuntimeError('mode review requires actual photographed labels and user-confirmed F/B/R selection with ESC off')


def validate_reverse_step(state, pulse, mode_changed, boot):
    if type(pulse) is not int or pulse not in (1400, 1350, 1300):
        raise ValueError('reverse review is bounded to 1400, 1350, 1300 us')
    if pulse == 1400:
        return
    previous = state.get('esc', {}).get('last_mode_reverse_observation', {})
    if (not mode_changed or previous.get('pulse_us') != pulse + 50
            or previous.get('source') != 'user_message'
            or previous.get('reverse_rotated') is not False
            or previous.get('forward_rotated') is not True
            or previous.get('boot_id') != boot):
        raise RuntimeError('a 50-us step requires the preceding same-boot observed F/B/R reverse failure and working forward response')


def save_record(folder, result):
    temporary = folder / 'result.json.tmp'
    with temporary.open('w', encoding='utf-8') as stream:
        stream.write(json.dumps(result, indent=2) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(folder / 'result.json')
    handle = os.open(folder, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(handle)
    finally:
        os.close(handle)


def run(args):
    reverse_pulse = getattr(args, 'reverse_pulse_us', 1400)
    if type(reverse_pulse) is not int or reverse_pulse not in (1400, 1350, 1300):
        raise ValueError('reverse review is bounded to 1400, 1350, 1300 us')
    result = {'phase': 'continuity_review', 'pulse_output': False,
              'mechanical_movement_verified': False, 'esc_model_confirmed': False,
              'neutral_us': 1500, 'settling_s': 12, 'commands': [], 'reverse_pulse_us': reverse_pulse,
              'persistent_reverse_pause_preserved': True}
    if not args.run:
        return result
    retry = getattr(args, 'observed_forward_retry', None)
    mode_changed = getattr(args, 'mode_change_review', False)
    reverse_only = getattr(args, 'reverse_only', False)
    on_static = getattr(args, 'esc_on_static_confirmed', False)
    switch_confirmed = bool(args.esc_off_confirmed) != bool(on_static)
    if not all((args.bench_prepared, args.original_esc_and_battery,
                switch_confirmed, args.user_review_authorized)):
        raise ValueError('fresh user preparation and this limited review authorization required')
    if bool(retry) != bool(on_static) and not (reverse_only and on_static and not retry):
        raise ValueError('already-on retry requires the observed-forward record')
    if reverse_only and (not mode_changed or retry):
        raise ValueError('reverse-only requires confirmed F/B/R mode review without forward retry')
    state = json.loads(bench.CALIBRATION_STATE.read_text(encoding='utf-8-sig'))
    if mode_changed:
        validate_mode_change(state, args.esc_off_confirmed, retry, already_on=on_static)
        result['user_confirmed_mode_change_review'] = True
        result['reverse_only'] = reverse_only
    if state.get('restart_review', {}).get('further_motion_paused'):
        raise RuntimeError('unresolved restart; no output')
    esc = state.get('esc', {})
    if not esc.get('reference_observed_static') or esc.get('positive_direction') != 'forward':
        raise RuntimeError('requires previously observed neutral and forward response')
    if sys.platform != 'linux' or b'Raspberry Pi 4 Model B' not in Path('/proc/device-tree/model').read_bytes():
        raise RuntimeError('requires the prepared Raspberry Pi 4B')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if not args.expected_boot_id or boot != args.expected_boot_id:
        raise RuntimeError('boot changed since preparation; no output')
    if reverse_only and on_static:
        confirmation = state.get('esc', {}).get('latest_on_confirmation', {})
        if confirmation.get('source') != 'user_message' or confirmation.get('boot_id') != boot:
            raise RuntimeError('already-on reverse review requires current-boot user preparation')
    validate_reverse_step(state, reverse_pulse, mode_changed, boot)
    if retry:
        validate_observed_forward_retry(retry, boot)
        result['observed_forward_retry'] = str(retry)
    parent = [p.decode() for p in Path(f'/proc/{os.getppid()}/cmdline').read_bytes().split(b'\0') if p]
    if not parent or Path(parent[0]).name != 'timeout' or not all(
            p in parent for p in ('95s', '--kill-after=1s', '--signal=TERM')):
        raise RuntimeError('independent 95-second timeout required')
    source = Path('/home/pi/motor_test.py')
    if hashlib.sha256(source.read_bytes()).hexdigest() != bench.SOURCE_SHA256:
        raise RuntimeError('motor helper changed; no output')
    before = subprocess.check_output(['pinctrl', 'get', bench.PINS], text=True, timeout=1)
    if len(before.splitlines()) != 4 or any('= input' not in s for s in before.splitlines()):
        raise RuntimeError('a reserved pin has an output owner')
    flags = bench.power_flags()
    if flags & 1:
        raise RuntimeError('current undervoltage; no output')
    folder = Path(args.output)
    folder.mkdir(parents=True, exist_ok=False)
    if retry:
        receipt = Path(retry) / 'retry-consumed.json'
        with receipt.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps({'output': str(folder), 'boot_id': boot}) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
    spec = importlib.util.spec_from_file_location('pinned_motor_helper', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    motor = module.MotorTest()
    result.update(boot_id_before=boot, power_flags_before=hex(flags), pins_before=before)
    try:
        motor.open()
        motor.set_pulse(1500)
        result['pulse_output'] = True
        print('NEUTRAL_ACTIVE: GPIO13 / physical 33 / 1500 us / 50 Hz', flush=True)
        review(motor, flags, result['commands'], observed_forward_retry=bool(retry), mode_changed=mode_changed, reverse_pulse=reverse_pulse, reverse_only=reverse_only, already_on=on_static)
        result['sequence_completed'] = True
    except BaseException as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        result['cleanup_errors'] = motor.shutdown()
        if motor.driver is not None:
            try:
                bench.restore_input(motor.driver)
            except Exception as exc:
                result['cleanup_errors'].append(str(exc))
        after = subprocess.check_output(['pinctrl', 'get', bench.PINS], text=True, timeout=1)
        result['pins_after'] = after
        result['pins_are_inputs'] = len(after.splitlines()) == 4 and all('= input' in s for s in after.splitlines())
        result['other_pins_preserved'] = all(a == b for a, b in zip(before.splitlines(), after.splitlines()) if 'GPIO13 =' not in a)
        after_flags = bench.power_flags()
        result['power_flags_after'] = hex(after_flags)
        result['undervoltage_detected'] = bool(after_flags & 1 or (after_flags & 0x10000 and not flags & 0x10000))
        result['boot_id_after'] = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        result['boot_unchanged'] = result['boot_id_after'] == boot
        result['source_sha256'] = bench.SOURCE_SHA256
        result['note'] = 'Requested waveforms recorded; actual movement requires separate human observation.'
        save_record(folder, result)
        print(json.dumps(result, indent=2), flush=True)
    if (result.get('error') or result['cleanup_errors'] or not result['pins_are_inputs']
            or not result['other_pins_preserved'] or result['undervoltage_detected']
            or not result['boot_unchanged']):
        raise RuntimeError('review stopped; inspect its record before another test')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--expected-boot-id')
    parser.add_argument('--reverse-pulse-us', type=int, default=1400)
    parser.add_argument('--observed-forward-retry')
    for name in ('run', 'bench-prepared', 'original-esc-and-battery', 'esc-off-confirmed', 'esc-on-static-confirmed', 'user-review-authorized', 'mode-change-review', 'reverse-only'):
        parser.add_argument('--' + name, action='store_true')
    args = parser.parse_args()
    result = run(args)
    if not args.run:
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
