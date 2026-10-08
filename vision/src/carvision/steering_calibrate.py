"""One observed steering calibration step. No ESC output or endpoint guessing.

The operator records each human observation separately in the state file.
Only a 50-us extension from the last accepted position is allowed. Every test
returns gradually to the current reference position and then releases BCM12.
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

from .gimbal import LocalPigpio
from .gimbal_bench import power_flags


def pulse_plan(center, previous, target):
    if any(type(v) is not int or not 1000 <= v <= 2000 for v in (center, previous, target)):
        raise ValueError('calibration reference must be an integer in 1000..2000 us')
    if abs(previous - target) > 50:
        raise ValueError('observe each step; maximum extension is 50 us')
    def ramp(start, end):
        values = []
        while start != end:
            start += max(-10, min(10, end-start))
            values.append((start, .02))
        return values
    return [(center, .2)] + ramp(center, target) + [(target, 2.0)] + ramp(target, center) + [(center, .2)]


class CalibrationDriver(LocalPigpio):
    def __init__(self, port, plan):
        super().__init__(port)
        self.allowed = {pulse for pulse, _ in plan} | {0}

    def set_servo(self, gpio, pulse):
        if type(gpio) is not int or gpio != 12 or type(pulse) is not int or pulse not in self.allowed:
            raise ValueError('only the validated BCM12 calibration plan is allowed')
        self.command(8, gpio, pulse)

    def servo_pulse(self, gpio):
        if type(gpio) is not int or gpio != 12:
            raise ValueError('only BCM12 readback is allowed')
        return self.command(84, gpio)

    def release(self, gpio):
        if type(gpio) is not int or gpio != 12:
            raise ValueError('only BCM12 release is allowed')
        self.command(0, gpio, 0)


def execute_plan(driver, plan, result, health_check=None):
    if any(driver.mode(pin) != 0 for pin in (12, 13, 17, 27)):
        raise RuntimeError('reserved pin is already in use as an output')
    result.update(pulse_off_confirmed=False, pin_input_confirmed=False)
    errors = []
    try:
        for pulse, duration in plan:
            if health_check:
                health_check()
            started = time.monotonic()
            driver.set_servo(12, pulse)
            result['hardware_output'] = True
            if driver.servo_pulse(12) != pulse:
                raise RuntimeError('calibration pulse not acknowledged')
            deadline = started + duration
            if health_check is None:
                time.sleep(max(0, deadline - time.monotonic()))
            else:
                while True:
                    health_check()
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    time.sleep(min(.1, remaining))
    finally:
        try:
            driver.set_servo(12, 0)
            if driver.servo_pulse(12) != 0:
                raise RuntimeError('pulse off not acknowledged')
            result['pulse_off_confirmed'] = True
        except Exception as exc:
            errors.append(str(exc))
        try:
            driver.release(12)
            if driver.mode(12) != 0:
                raise RuntimeError('input mode not acknowledged')
            result['pin_input_confirmed'] = True
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            raise RuntimeError('cleanup failed: ' + '; '.join(errors))


def run(args):
    state = json.loads(Path(args.state).read_text(encoding='utf-8-sig'))
    steering = state['steering']
    center, previous = steering['reference_us'], steering['last_accepted_us']
    plan = pulse_plan(center, previous, args.target_us)
    result = dict(hardware_output=False, esc_output=False, reference_us=center,
                  previous_accepted_us=previous, target_us=args.target_us,
                  duration_s=sum(duration for _, duration in plan),
                  mechanical_observation='pending', calibration_completed=False)
    result['motion_blocked_by_restart'] = bool(state.get('restart_review', {}).get('further_motion_paused'))
    result['center_review_retry'] = bool(getattr(args, 'center_review_retry', False))
    if not args.run:
        return result
    if result['motion_blocked_by_restart'] and not (result['center_review_retry'] and args.target_us == center):
        raise RuntimeError('unexplained restart recorded; resolve restart review before further motion')
    if not args.bench_prepared:
        raise ValueError('confirm raised wheels and steering wiring before output')
    if not state.get('steering_5v_pin_wiring_confirmed') or not state.get('wheels_raised_confirmed'):
        raise ValueError('missing confirmed bench setup')
    if sys.platform != 'linux' or b'Raspberry Pi 4 Model B' not in Path('/proc/device-tree/model').read_bytes():
        raise RuntimeError('requires prepared Pi 4B')
    subprocess.run(['sudo', '-n', 'true'], check=True)
    if subprocess.run(['pgrep', '-x', 'pigpiod'], capture_output=True).returncode != 1:
        raise RuntimeError('existing daemon or inventory failure; preserved')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 8890))
    flags = power_flags()
    if flags & 1:
        raise RuntimeError('current undervoltage; no pulse sent')
    if result['center_review_retry'] and flags & 0x10000:
        raise RuntimeError('center review requires a clean power baseline on this boot')
    boot_path = Path('/proc/sys/kernel/random/boot_id')
    boot_before = boot_path.read_text().strip()
    result.update(boot_id_before=boot_before, power_samples=[])
    def health_check():
        current = power_flags()
        result['power_samples'].append({'monotonic_s': time.monotonic(), 'flags': hex(current)})
        if current & 1 or (current & 0x10000 and not flags & 0x10000):
            raise RuntimeError('new undervoltage; steering output stopped')
        if boot_path.read_text().strip() != boot_before:
            raise RuntimeError('boot changed; steering output stopped')
    before = subprocess.check_output(['pinctrl', 'get', '12,13,17,27'], text=True)
    if len(before.splitlines()) != 4 or any('= input' not in line for line in before.splitlines()):
        raise RuntimeError('reserved GPIO already configured as output')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    driver = Path(__file__).resolve().parents[3] / 'drivers/pigpio'
    command = ['sudo', '-n', 'timeout', '--signal=TERM', '--kill-after=1s', '7s',
               'env', f'LD_LIBRARY_PATH={driver}', str(driver/'pigpiod'), '-g',
               '-l', '-f', '-n', '127.0.0.1', '-t', '1', '-x', '0x1000', '-p', '8890']
    client = CalibrationDriver(8890, plan)
    with (output/'driver.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 1.8
            while True:
                try:
                    client.open()
                    break
                except OSError:
                    client.close()
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError('calibration driver did not start')
                    time.sleep(.03)
            execute_plan(client, plan, result, health_check)
        except BaseException as exc:
            result['error'] = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            client.close()
            result['daemon_timeout_exit'] = process.wait(timeout=9)
            after_flags = power_flags()
            result['power_flags_before'], result['power_flags_after'] = hex(flags), hex(after_flags)
            result['boot_id_after'] = boot_path.read_text().strip()
            result['boot_unchanged'] = result['boot_id_after'] == boot_before
            result['undervoltage_detected'] = bool(after_flags & 1 or (after_flags & 0x10000 and not flags & 0x10000))
            after = subprocess.check_output(['pinctrl', 'get', '12,13,17,27'], text=True)
            result['pins_before'], result['pins_after'] = before, after
            result['pins_restored'] = before == after
            temporary = output/'result.json.tmp'
            with temporary.open('w', encoding='utf-8') as record:
                record.write(json.dumps(result, indent=2)+'\n')
                record.flush()
                os.fsync(record.fileno())
            temporary.replace(output/'result.json')
            for directory in (output, output.parent):
                handle = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(handle)
                finally:
                    os.close(handle)
    if (result['undervoltage_detected'] or not result['boot_unchanged']
            or not result['pins_restored'] or result['daemon_timeout_exit'] != 124):
        raise RuntimeError('post-test check failed; do not continue calibration')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state', required=True)
    parser.add_argument('--target-us', type=int, required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--bench-prepared', action='store_true')
    parser.add_argument('--center-review-retry', action='store_true',
                        help='after checking connections, permit only the current reference for restart review')
    print(json.dumps(run(parser.parse_args()), indent=2))


if __name__ == '__main__':
    main()
