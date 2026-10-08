"""Single MG996R initial reference test; does not establish calibration.

1500 us is an interface reference, not a measured mechanical center. Only one
axis receives pulses, for 100 ms by default. Observation mode adds a 50 us
excursion and return, taking 600 ms or two seconds. A root-owned deadline
also stops the daemon if the client disappears. No wave or car PWM commands.
"""

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

from .gimbal import LocalPigpio, load_gimbal_config


def pulse_plan(observe=False, observe_seconds=.6):
    if type(observe) is not bool:
        raise ValueError('observation selection must be boolean')
    if type(observe_seconds) not in (int, float) or observe_seconds not in (.6, 2.0):
        raise ValueError('observation duration must be .6 or 2 seconds')
    if not observe:
        return [(1500, .1)]
    extra_hold = (observe_seconds - .6) / 2
    return ([(1500, .15)] + [(pulse, .02) for pulse in range(1510, 1551, 10)]
            + [(1550, .15 + extra_hold)] + [(pulse, .02) for pulse in range(1540, 1499, -10)]
            + [(1500, .1 + extra_hold)])


def brief_reference(driver, gpio, progress=None, observe=False, observe_seconds=.6):
    if type(gpio) is not int or gpio not in (17, 27):
        raise ValueError('initial reference is restricted to BCM17 or BCM27')
    plan = pulse_plan(observe, observe_seconds)
    if any(driver.mode(pin) != 0 for pin in (12, 13, 17, 27)):
        raise RuntimeError('reserved pins have an existing owner; no output was sent')
    progress = {} if progress is None else progress
    progress.update({'gpio': gpio, 'physical_pin': {17: 11, 27: 13}[gpio],
                     'reference_us': 1500, 'requested_duration_s': sum(period for _, period in plan),
                     'observation_mode': observe, 'maximum_requested_us': max(pulse for pulse, _ in plan),
                     'hardware_output_attempted': True, 'hardware_output': None,
                     'pulse_off_confirmed': False, 'pin_input_confirmed': False,
                     'mechanical_movement_verified': False, 'calibration_completed': False})
    errors = []
    try:
        started = time.monotonic()
        for pulse, period in plan:
            step_started = time.monotonic()
            driver.set_servo(gpio, pulse)
            progress['hardware_output'] = True
            if driver.servo_pulse(gpio) != pulse:
                raise RuntimeError('daemon did not acknowledge the requested pulse')
            progress['reference_acknowledged'] = True
            time.sleep(max(0, period - (time.monotonic() - step_started)))
        progress['measured_client_duration_s'] = time.monotonic() - started
    finally:
        # Read pulse-off while the GPIO still has the servo function. Releasing
        # it changes that function; GPW would correctly return NOT_SERVO (-93).
        try:
            driver.set_servo(gpio, 0)
            if driver.servo_pulse(gpio) != 0:
                raise RuntimeError('reference pulse-off was not acknowledged')
            progress['pulse_off_confirmed'] = True
        except Exception as exc:
            errors.append(str(exc))
        try:
            driver.release(gpio)
            if driver.mode(gpio) != 0:
                raise RuntimeError('reference pin input mode was not acknowledged')
            progress['pin_input_confirmed'] = True
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            raise RuntimeError('reference cleanup failed: ' + '; '.join(errors))
    return progress


def power_flags():
    value = subprocess.check_output(['vcgencmd', 'get_throttled'], text=True).strip()
    return int(value.split('=', 1)[1], 16)


def daemon_command(driver, gpio, port, duration=.1):
    if type(gpio) is not int or gpio not in (17, 27):
        raise ValueError('invalid reference axis')
    deadline = '5s' if duration > .6 else '3s'
    return ['sudo', '-n', 'timeout', '--signal=TERM', '--kill-after=1s', deadline,
            'env', f'LD_LIBRARY_PATH={driver}', str(driver / 'pigpiod'),
            '-g', '-l', '-f', '-n', '127.0.0.1', '-t', '1',
            '-x', hex(1 << gpio), '-p', str(port)]


def run(args):
    config = load_gimbal_config(args.gimbal_config)
    if args.axis not in ('pan', 'tilt'):
        raise ValueError('select one reference axis')
    gpio = {'pan': 17, 'tilt': 27}[args.axis]
    observe = getattr(args, 'observe', False)
    observe_seconds = getattr(args, 'observe_seconds', .6)
    plan = pulse_plan(observe, observe_seconds)
    if getattr(config, args.axis).model.upper() != 'MG996R':
        raise ValueError('this initial reference is limited to the identified MG996R')
    result = {'hardware_output': False, 'axis': args.axis, 'gpio': gpio,
              'reference_us': 1500, 'requested_duration_s': sum(period for _, period in plan),
              'observation_mode': observe, 'maximum_requested_us': max(pulse for pulse, _ in plan),
              'calibration_completed': False, 'full_control_ready': not config.missing()}
    if not args.run:
        return result
    if not args.confirm_5v_pin_wiring or not args.acknowledge_motion:
        raise ValueError('reference requires confirmed 5V-pin wiring and prepared motion test')
    if not config.daemon_compatibility_confirmed:
        raise ValueError('complete the read-only driver compatibility probe first')
    if sys.platform != 'linux' or b'Raspberry Pi 4 Model B' not in Path('/proc/device-tree/model').read_bytes():
        raise RuntimeError('reference is only for the prepared Raspberry Pi 4B')
    subprocess.run(['sudo', '-n', 'true'], check=True)
    existing = subprocess.run(['pgrep', '-x', 'pigpiod'], capture_output=True)
    if existing.returncode != 1:
        raise RuntimeError('existing daemon or inventory failure; no daemon was replaced')
    with socket.socket() as port_probe:
        port_probe.bind(('127.0.0.1', config.local_port))
    before_flags = power_flags()
    if before_flags & 1:
        raise RuntimeError('Pi currently reports undervoltage; no output was sent')
    before = subprocess.check_output(['pinctrl', 'get', '12,13,17,27'], text=True)
    if len(before.splitlines()) != 4 or any('= input' not in line for line in before.splitlines()):
        raise RuntimeError('reserved pins are not all inputs; existing owner was preserved')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[3]
    driver = root / 'drivers/pigpio'
    client = LocalPigpio(config.local_port)
    with (output / 'driver.log').open('w') as log:
        process = subprocess.Popen(daemon_command(driver, gpio, config.local_port, result['requested_duration_s']),
                                   stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 1.8
            while True:
                try:
                    client.open()
                    break
                except OSError:
                    client.close()
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError('private reference daemon failed to start')
                    time.sleep(.03)
            result['driver_version'] = client.version()
            brief_reference(client, gpio, progress=result, observe=observe, observe_seconds=observe_seconds)
        except BaseException as exc:
            result['error'] = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            client.close()
            # Wait for root's deadline; client signal handling cannot extend it.
            result['daemon_timeout_exit'] = process.wait(timeout=7)
            result['power_flags_before'] = hex(before_flags)
            after_flags = power_flags()
            result['power_flags_after'] = hex(after_flags)
            result['undervoltage_detected'] = bool(after_flags & 1 or (after_flags & 0x10000 and not before_flags & 0x10000))
            after = subprocess.check_output(['pinctrl', 'get', '12,13,17,27'], text=True)
            result['pins_before'] = before
            result['pins_after'] = after
            result['pins_restored'] = before == after
            result['gpio_permission_mask'] = hex(1 << gpio)
            result['note'] = 'Human confirmed 5V-pin wiring; voltage/current and actual motion are not measured by this test.'
            (output / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    if result['undervoltage_detected'] or not result['pins_restored'] or result['daemon_timeout_exit'] != 124:
        raise RuntimeError('post-test check failed; inspect result.json before further motion')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gimbal-config', required=True)
    parser.add_argument('--axis', choices=['pan', 'tilt'], required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--observe', action='store_true', help='one 50 us excursion and return over 600 ms')
    parser.add_argument('--observe-seconds', type=float, choices=[.6, 2.0], default=.6,
                        help='fixed observation duration; initial reference remains 100 ms')
    parser.add_argument('--confirm-5v-pin-wiring', action='store_true')
    parser.add_argument('--acknowledge-motion', action='store_true')
    args = parser.parse_args()
    print(json.dumps(run(args), indent=2))


if __name__ == '__main__':
    main()
