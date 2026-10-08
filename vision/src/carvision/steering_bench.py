"""Initial steering response only, with a private, expiring BCM12 driver.

1500 us is an interface reference, not a measured straight-wheel position.
No ESC signal is emitted and no race calibration is changed. The initial
reference lasts 100 ms; observation stays within 1450..1550 us for two seconds.
"""

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

from .gimbal import LocalPigpio
from .gimbal_bench import power_flags


PINS = (12, 13, 17, 27)


class SteeringDriver(LocalPigpio):
    """Separate one-pin client; the cloud servo client remains restricted."""

    @staticmethod
    def check_pin(gpio):
        if type(gpio) is not int or gpio != 12:
            raise ValueError('steering reference only allows BCM12')

    def set_servo(self, gpio, pulse):
        self.check_pin(gpio)
        if type(pulse) is not int or (pulse != 0 and not 1450 <= pulse <= 1550):
            raise ValueError('initial steering pulse must be 1450..1550 us or off')
        self.command(8, gpio, pulse)

    def servo_pulse(self, gpio):
        self.check_pin(gpio)
        return self.command(84, gpio)

    def release(self, gpio):
        self.check_pin(gpio)
        self.command(0, gpio, 0)


def pulse_plan(observe=False):
    if type(observe) is not bool:
        raise ValueError('observation choice must be boolean')
    if not observe:
        return [(1500, .1)]
    return ([(1500, .2)] + [(pulse, .02) for pulse in range(1490, 1449, -10)]
            + [(1450, .35)] + [(pulse, .02) for pulse in range(1460, 1551, 10)]
            + [(1550, .45)] + [(pulse, .02) for pulse in range(1540, 1499, -10)]
            + [(1500, .6)])


def brief_reference(driver, observe=False, progress=None):
    plan = pulse_plan(observe)
    if any(driver.mode(pin) != 0 for pin in PINS):
        raise RuntimeError('reserved GPIO has an existing owner; no output sent')
    progress = {} if progress is None else progress
    progress.update(gpio=12, physical_pin=32, reference_us=1500,
                    requested_duration_s=sum(period for _, period in plan),
                    observation_mode=observe, hardware_output_attempted=True,
                    hardware_output=None, pulse_off_confirmed=False,
                    pin_input_confirmed=False, mechanical_movement_verified=False,
                    calibration_completed=False, esc_output=False)
    errors = []
    try:
        started = time.monotonic()
        for pulse, period in plan:
            step_started = time.monotonic()
            driver.set_servo(12, pulse)
            progress['hardware_output'] = True
            if driver.servo_pulse(12) != pulse:
                raise RuntimeError('requested steering pulse was not acknowledged')
            time.sleep(max(0, period - (time.monotonic() - step_started)))
        progress['measured_client_duration_s'] = time.monotonic() - started
    finally:
        try:
            driver.set_servo(12, 0)
            if driver.servo_pulse(12) != 0:
                raise RuntimeError('steering pulse-off was not acknowledged')
            progress['pulse_off_confirmed'] = True
        except Exception as exc:
            errors.append(str(exc))
        try:
            driver.release(12)
            if driver.mode(12) != 0:
                raise RuntimeError('steering GPIO did not return to input')
            progress['pin_input_confirmed'] = True
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            raise RuntimeError('steering cleanup failed: ' + '; '.join(errors))
    return progress


def daemon_command(driver, observe=False):
    pulse_plan(observe)
    return ['sudo', '-n', 'timeout', '--signal=TERM', '--kill-after=1s',
            '5s' if observe else '3s', 'env', f'LD_LIBRARY_PATH={driver}',
            str(driver / 'pigpiod'), '-g', '-l', '-f', '-n', '127.0.0.1',
            '-t', '1', '-x', '0x1000', '-p', '8890']


def run(args):
    plan = pulse_plan(args.observe)
    result = dict(hardware_output=False, esc_output=False, gpio=12, physical_pin=32,
                  pulse_plan=plan, calibration_completed=False,
                  requested_duration_s=sum(period for _, period in plan))
    if not args.run:
        return result
    if not args.confirm_5v_pin_wiring or not args.wheels_raised:
        raise ValueError('confirm actual steering 5V wiring and raised drive wheels')
    if sys.platform != 'linux' or b'Raspberry Pi 4 Model B' not in Path('/proc/device-tree/model').read_bytes():
        raise RuntimeError('this test is for the prepared Raspberry Pi 4B')
    subprocess.run(['sudo', '-n', 'true'], check=True)
    if subprocess.run(['pgrep', '-x', 'pigpiod'], capture_output=True).returncode != 1:
        raise RuntimeError('existing daemon or inventory error; nothing replaced')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 8890))
    before_flags = power_flags()
    if before_flags & 1:
        raise RuntimeError('Pi reports current undervoltage; no signal sent')
    before = subprocess.check_output(['pinctrl', 'get', '12,13,17,27'], text=True)
    if len(before.splitlines()) != 4 or any('= input' not in line for line in before.splitlines()):
        raise RuntimeError('reserved GPIO is not input; existing owner preserved')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    driver = Path(__file__).resolve().parents[3] / 'drivers/pigpio'
    client = SteeringDriver(8890)
    with (output / 'driver.log').open('w') as log:
        process = subprocess.Popen(daemon_command(driver, args.observe), stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 1.8
            while True:
                try:
                    client.open()
                    break
                except OSError:
                    client.close()
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError('private steering driver did not start')
                    time.sleep(.03)
            result['driver_version'] = client.version()
            brief_reference(client, args.observe, result)
        except BaseException as exc:
            result['error'] = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            client.close()
            result['daemon_timeout_exit'] = process.wait(timeout=7)
            result['power_flags_before'] = hex(before_flags)
            after_flags = power_flags()
            result['power_flags_after'] = hex(after_flags)
            result['undervoltage_detected'] = bool(after_flags & 1 or (after_flags & 0x10000 and not before_flags & 0x10000))
            after = subprocess.check_output(['pinctrl', 'get', '12,13,17,27'], text=True)
            result['pins_before'], result['pins_after'] = before, after
            result['pins_restored'] = before == after
            result['gpio_permission_mask'] = '0x1000'
            result['note'] = '5V wiring confirmed by human; supply capacity and actual movement are not measured.'
            (output / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    if result['undervoltage_detected'] or not result['pins_restored'] or result['daemon_timeout_exit'] != 124:
        raise RuntimeError('steering post-test check failed; inspect result before more motion')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--observe', action='store_true')
    parser.add_argument('--confirm-5v-pin-wiring', action='store_true')
    parser.add_argument('--wheels-raised', action='store_true')
    print(json.dumps(run(parser.parse_args()), indent=2))


if __name__ == '__main__':
    main()
