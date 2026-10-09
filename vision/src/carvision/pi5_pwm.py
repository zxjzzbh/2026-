"""Explicit Pi 5 bench PWM, owned by a separate, bounded guardian process.

RP1 hardware generates the pulse widths. The guardian enforces durations and
the command lease in software; it is not the Pi 4 finite DMA waveform driver.
No pins are changed by importing this module or by inspecting hardware.
"""

import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time

from .network_guard import interface_is_inactive


def inspect_pi5():
    from .pi_control import inspect_pwm_chips
    model = Path('/proc/device-tree/model').read_bytes().rstrip(b'\0').decode()
    pins = subprocess.check_output(['pinctrl', 'get', '12,13,17,27'], text=True, timeout=2)
    return {'model': model, 'pins': pins, 'pwm_chips': inspect_pwm_chips(),
            'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'power_flags': int(subprocess.check_output(['vcgencmd', 'get_throttled'],
                               text=True, timeout=2).strip().split('=')[1], 16),
            'hardware_output': False}


def validate_review(review, boot, *, neutral_only=False, motor_only=False,
                    steering_only=False, combined=False, ground_short=False, ground_held=False, ground_continuous=False):
    if sum((neutral_only, motor_only, steering_only, combined, ground_short, ground_held, ground_continuous)) != 1:
        raise ValueError('Pi 5 requires one explicit bench scope')
    if (review.get('model') != 'Raspberry Pi 5 Model B Rev 1.0'
            or review.get('boot_id') != boot
            or (not (ground_short or ground_held or ground_continuous) and review.get('wheels_raised_confirmed_by_user') is not True)
            or review.get('esc_off_confirmed_by_user') is not True):
        raise ValueError('fresh Pi 5, raised-wheel and ESC-off preparation required')
    if ground_short:
        required = ('ground_short_requested_by_user', 'wheels_on_ground_confirmed_by_user',
                    'clear_area_confirmed_by_user', 'power_switch_in_reach_confirmed_by_user',
                    'neutral_physically_verified', 'steering_physically_verified',
                    'stop_physically_verified', 'disconnect_stop_physically_verified')
        if (not all(review.get(k) is True for k in required)
                or review.get('continuous_ground_driving_enabled') is not False
                or review.get('esc_backend') != 'rasadapter5a_uart'
                or review.get('steering_backend') != 'rasadapter5a_uart'):
            raise ValueError('Pi 5 ground short trial requires fresh ground preparation and actual stop evidence')
    if ground_held:
        session_limit = review.get('manual_session_limit_s', 180)
        hold_limit = review.get('hold_limit_s', 60)
        if (type(session_limit) is not int or type(hold_limit) is not int
                or (session_limit, hold_limit) not in ((180, 60), (600, 600))
                or session_limit == 600 and review.get('long_route_requested_by_user') is not True):
            raise ValueError('long ground session requires explicit route authorization and bounded limits')
        required = ('ground_held_requested_by_user', 'wheels_on_ground_confirmed_by_user',
                    'clear_area_confirmed_by_user', 'spotter_can_cut_power_confirmed_by_user',
                    'neutral_physically_verified', 'stop_physically_verified',
                    'keyboard_stop_physically_verified', 'prior_pi5_ground_low_speed_physically_verified',
                    'prior_pi5_steering_physically_verified')
        if (not all(review.get(name) is True for name in required)
                or review.get('continuous_ground_driving_enabled') is not True
                or review.get('keyboard_stop_evidence_boot_id') != boot
                or review.get('esc_backend') != 'rasadapter5a_uart' or review.get('esc_channel') != 4
                or review.get('steering_backend') != 'rasadapter5a_uart' or review.get('steering_channel') != 3):
            raise ValueError('ground held control needs current key-release stop evidence, ground preparation and spotter')
    if ground_continuous:
        required = ('ground_continuous_requested_by_user', 'wheels_on_ground_confirmed_by_user',
                    'clear_area_confirmed_by_user', 'spotter_can_cut_power_confirmed_by_user',
                    'neutral_physically_verified', 'stop_physically_verified',
                    'disconnect_stop_physically_verified', 'steering_physically_verified',
                    'drivetrain_issue_resolved_reported_by_user', 'continuous_ground_driving_enabled',
                    'reverse_revalidation_prepared')
        if (not all(review.get(k) is True for k in required)
                or review.get('manual_session_limit_s', 0) is not None
                or review.get('hold_limit_s', 0) is not None
                or review.get('esc_backend') != 'rasadapter5a_uart' or review.get('esc_channel') != 4
                or review.get('steering_backend') != 'rasadapter5a_uart' or review.get('steering_channel') != 3):
            raise ValueError('continuous ground control requires explicit manual authorization, fresh neutral and ground preparation')
    if not neutral_only and not steering_only and review.get('neutral_physically_verified') is not True:
        raise ValueError('Pi 5 neutral must first be physically verified')
    if combined and not all(review.get(k) is True for k in
                            ('steering_physically_verified', 'stop_physically_verified')):
        raise ValueError('Pi 5 combined trial needs isolated steering and stop evidence')
    if review.get('forward_us', 1575) != 1575:
        raise ValueError('Pi 5 first trial uses only the previously observed 1575-us point')
    if review.get('steering_backend') == 'rasadapter5a_uart' and (steering_only or combined or ground_short or ground_held or ground_continuous):
        channel = review.get('steering_channel')
        if type(channel) is not int or not 1 <= channel <= 6:
            raise ValueError('the actual RasAdapter steering channel must be identified')
    if review.get('esc_backend') == 'rasadapter5a_uart' and not steering_only:
        if review.get('esc_channel') != 4:
            raise ValueError('the actual S4 ESC channel must be identified')
    if review.get('reverse_revalidation_prepared') is True and not all(
            review.get(k) is True for k in ('neutral_physically_verified', 'stop_physically_verified')):
        raise ValueError('bounded reverse revalidation requires current neutral and stop evidence')


class ScopedPWM:
    """Own only newly exported RP1 channels; preserve fan and other outputs."""
    def __init__(self, *, motor, steering):
        self.motor, self.steering = motor, steering
        self.chip = None
        self.channels = {}
        self.exported = []
        self.muxed = []
        self.lock_file = None

    def write(self, channel, name, value):
        (self.channels[channel] / name).write_text(str(value))

    def open(self):
        import fcntl
        from .pi_control import inspect_pwm_chips
        if b'Raspberry Pi 5 Model B' not in Path('/proc/device-tree/model').read_bytes():
            raise RuntimeError('RP1 bench backend requires Raspberry Pi 5')
        candidates = [c for c in inspect_pwm_chips()
                      if 'raspberrypi,rp1-pwm' in c['compatible']
                      and c['channels'] == 4 and c['export_writable']]
        if len(candidates) != 1:
            raise RuntimeError('one writable RP1 PWM controller is required')
        self.chip = Path(candidates[0]['path'])
        fd = os.open('/tmp/carvision-pi-pwm.lock', os.O_CREAT | os.O_RDWR |
                     getattr(os, 'O_NOFOLLOW', 0), 0o600)
        self.lock_file = os.fdopen(fd, 'w')
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            wanted = ([0] if self.steering else []) + ([1] if self.motor else [])
            # Inspect every selected channel/pin before modifying any of them.
            for channel in wanted:
                if (self.chip / f'pwm{channel}').exists():
                    raise RuntimeError('existing PWM owner preserved')
                pin = 12 + channel
                state = subprocess.check_output(['pinctrl', 'get', str(pin)], text=True, timeout=1)
                if f'GPIO{pin} = input' not in state and f'GPIO{pin} = none' not in state:
                    raise RuntimeError('existing pin owner preserved')
            for channel in wanted:
                (self.chip / 'export').write_text(str(channel))
                self.exported.append(channel)
                folder = self.chip / f'pwm{channel}'
                deadline = time.monotonic() + 2
                while not (folder / 'enable').exists() or not os.access(folder / 'enable', os.W_OK):
                    if time.monotonic() >= deadline:
                        raise RuntimeError('PWM channel permissions unavailable')
                    time.sleep(.01)
                self.channels[channel] = folder
                for name, value in [('enable', 0), ('duty_cycle', 0),
                                    ('period', 20_000_000), ('polarity', 'normal')]:
                    self.write(channel, name, value)
                self.write(channel, 'duty_cycle', (1650 if channel == 0 else 1500) * 1000)
                pin = 12 + channel
                self.muxed.append(pin)
                subprocess.run(['pinctrl', 'set', str(pin), 'a0', 'pd'], check=True, timeout=1)
                actual = subprocess.check_output(['pinctrl', 'get', str(pin)], text=True, timeout=1)
                if f'GPIO{pin} = PWM0_CHAN{channel}' not in actual:
                    raise RuntimeError('RP1 pin/channel mapping was not acknowledged')
            if self.motor:
                self.write(1, 'enable', 1)
        except BaseException:
            self.close()
            raise

    def neutral(self):
        if 1 in self.channels:
            self.write(1, 'duty_cycle', 1_500_000)

    def motion(self, pulse):
        if not self.motor or type(pulse) is not int or pulse not in (1575, 1300):
            raise ValueError('invalid isolated motor point')
        self.write(1, 'duty_cycle', pulse * 1000)

    def turn(self, pulse):
        if not self.steering or type(pulse) is not int or not 1550 <= pulse <= 1750:
            raise ValueError('invalid conservative steering point')
        self.write(0, 'duty_cycle', pulse * 1000)
        self.write(0, 'enable', 1)

    def steering_idle(self):
        if 0 in self.channels:
            self.write(0, 'enable', 0)

    def close(self):
        errors = []
        if 1 in self.channels:
            try:
                self.neutral()
                time.sleep(.42)
            except Exception as exc:
                errors.append(str(exc))
        for channel in self.channels:
            try:
                self.write(channel, 'enable', 0)
            except Exception as exc:
                errors.append(str(exc))
        for pin in self.muxed:
            try:
                subprocess.run(['pinctrl', 'set', str(pin), 'ip', 'pd'], check=True, timeout=1)
            except Exception as exc:
                errors.append(str(exc))
        for channel in self.exported:
            try:
                (self.chip / 'unexport').write_text(str(channel))
            except Exception as exc:
                errors.append(str(exc))
        self.channels.clear()
        self.exported.clear()
        self.muxed.clear()
        if self.lock_file:
            self.lock_file.close()
            self.lock_file = None
        return errors


class GuardState:
    """The hardware owner expires motion even if the HTTP/control parent stalls."""
    def __init__(self, pwm, *, clock=time.monotonic, neutral_only=False):
        self.pwm, self.clock, self.neutral_only = pwm, clock, neutral_only
        self.expires = clock() + 1.5  # bounded startup handshake
        self.motion_until = None
        self.fault = None

    def tick(self):
        now = self.clock()
        if self.motion_until is not None and now >= self.motion_until:
            self.pwm.neutral()
            self.motion_until = None
        if now >= self.expires and self.fault is None:
            self.pwm.neutral()
            self.pwm.steering_idle()
            self.motion_until = None
            self.fault = 'Pi 5 guardian command lease expired'

    def request(self, data):
        self.tick()
        action = data.get('action')
        if self.fault and action not in ('neutral', 'shutdown'):
            raise RuntimeError(self.fault)
        if action == 'motion':
            duration = data.get('seconds')
            pulse = data.get('pulse')
            if (self.neutral_only or type(duration) not in (int, float)
                    or not math.isfinite(duration) or not .02 <= duration <= .8
                    or type(pulse) is not int or pulse not in (1575, 1300)):
                raise ValueError('only finite, reviewed motor commands are allowed')
            # Count the limit from before the potentially slow sysfs write.
            self.motion_until = self.clock() + duration
            self.pwm.motion(pulse)
        elif action == 'neutral':
            self.pwm.neutral()
            self.motion_until = None
        elif action == 'steering':
            if self.neutral_only:
                raise ValueError('servos disabled for neutral-only trial')
            self.pwm.turn(data.get('pulse'))
        elif action == 'steering_idle':
            self.pwm.steering_idle()
        elif action not in ('check', 'shutdown'):
            raise ValueError('invalid PWM guardian request')
        self.expires = self.clock() + .25
        return {'ok': True, 'fault': self.fault}


class RasAdapterPWM(ScopedPWM):
    """Only the identified S3 steering and S4 ESC; S1/S2 never commanded."""
    def __init__(self, *, motor, channel=None, esc_channel=None, center_us=1650, single_target=False,
                 boundary_center_trial=False):
        # Keep the common ownership lock; do not export RP1 PWM channels.
        super().__init__(motor=False, steering=False)
        from .rasadapter5 import RasAdapter
        self.board = RasAdapter()
        if type(boundary_center_trial) is not bool:
            raise ValueError('boundary center trial requires an explicit boolean')
        if boundary_center_trial and (channel != 3 or center_us != 1550):
            raise ValueError('boundary center trial is only the requested S3 1550-us reference')
        if type(center_us) is not int or not (1550 < center_us < 1750
                                            or boundary_center_trial and center_us == 1550):
            raise ValueError('steering reference must be inside the observed range')
        self.center_us = center_us
        if type(single_target) is not bool or single_target and channel != 3:
            raise ValueError('single target interpolation requires the actual S3 channel')
        self.single_target = single_target
        if channel is not None and channel != 3:
            raise ValueError('actual steering is S3; cloud/gimbal channels are excluded')
        if motor and esc_channel != 4:
            raise ValueError('actual ESC is S4')
        self.channel = channel
        self.esc_channel = esc_channel if motor else None
        self.steering_commanded = False

    def open(self):
        super().open()
        try:
            self.board.open()
            # Discard pre-existing telemetry before matching fresh replies.
            self.board.receive(.02)
            for channel in (self.channel, self.esc_channel):
                if channel is not None:
                    for attempt in range(3):
                        try:
                            self.board.read_position(channel)  # read-only startup handshake
                            break
                        except RuntimeError:
                            if attempt == 2:
                                raise
            if self.channel is not None:
                self.turn(self.center_us)
                time.sleep(.12)
                if self.board.read_position(self.channel) != self.center_us:
                    raise RuntimeError('RasAdapter S3 center was not acknowledged')
            if self.esc_channel is not None:
                self.neutral()
                time.sleep(.03)
                if self.board.read_position(self.esc_channel) != 1500:
                    raise RuntimeError('RasAdapter S4 neutral was not acknowledged')
        except BaseException:
            self.close()
            raise

    def turn(self, pulse):
        if self.channel is None:
            raise ValueError('servos disabled for this RasAdapter scope')
        self.board.set_position(self.channel, pulse, .3 if self.single_target else .02)
        self.steering_commanded = True

    def motion(self, pulse):
        if self.esc_channel is None:
            raise ValueError('ESC disabled for this RasAdapter scope')
        self.board.set_esc(self.esc_channel, pulse)

    def neutral(self):
        if self.esc_channel is not None and self.board.fd is not None:
            self.board.set_esc(self.esc_channel, 1500)

    def steering_idle(self):
        if self.steering_commanded:
            # Manufacturer has no documented PWM unload command here. Keep a
            # center command rather than claim the MCU signal was disabled.
            self.board.set_position(self.channel, self.center_us, .1)

    def close(self):
        errors = []
        try:
            if self.esc_channel is not None and self.board.fd is not None:
                self.neutral()
                time.sleep(.42)
            self.steering_idle()
        except Exception as exc:
            errors.append(str(exc))
        self.board.close()
        errors.extend(super().close())
        return errors


def guard_main(args):
    pwm = (RasAdapterPWM(motor=args.motor, channel=args.steering_channel, esc_channel=args.esc_channel,
                         center_us=args.steering_center_us,
                         single_target=getattr(args, 'steering_single_target', False),
                         boundary_center_trial=getattr(args, 'steering_boundary_center_trial', False)) if args.steering_channel or args.esc_channel
           else ScopedPWM(motor=args.motor, steering=args.steering))
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    continuous = getattr(args, 'continuous_manual', False)
    deadline, buffer = None if continuous else time.monotonic() + 800, b''
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    def reply(data):
        print(json.dumps(data), flush=True)
    try:
        if boot != args.expected_boot_id:
            raise RuntimeError('Pi 5 boot changed before PWM startup')
        if int(subprocess.check_output(['vcgencmd', 'get_throttled'], text=True).split('=')[1], 16):
            raise RuntimeError('Pi 5 PWM requires clean current/historical power flags')
        pwm.open()
        state = GuardState(pwm, neutral_only=args.neutral_only)
        reply({'ok': True, 'ready': True})
        power_at = network_at = 0
        while not stopping and (deadline is None or time.monotonic() < deadline):
            state.tick()
            if state.fault:
                break
            if time.monotonic() >= power_at:
                flags = int(subprocess.check_output(['vcgencmd', 'get_throttled'], text=True, timeout=.2).split('=')[1], 16)
                if flags or Path('/proc/sys/kernel/random/boot_id').read_text().strip() != boot:
                    raise RuntimeError('Pi 5 power/boot changed')
                power_at = time.monotonic() + .25
            if continuous and time.monotonic() >= network_at:
                route = json.loads(subprocess.check_output(['ip', '-j', 'route', 'get', '1.1.1.1'], text=True, timeout=.2))[0]
                temperature = subprocess.check_output(['vcgencmd', 'measure_temp'], text=True, timeout=.2)
                if (route.get('dev') != 'usb0'
                        or Path('/sys/class/net/usb0/carrier').read_text().strip() != '1'
                        or any(not interface_is_inactive(n) for n in ('eth0', 'wlan0'))
                        or float(temperature.split('=')[1].split("'")[0]) >= 70):
                    raise RuntimeError('5G route or temperature changed; manual output stopped')
                network_at = time.monotonic()+2
            readable, _, _ = select.select([sys.stdin.fileno()], [], [], .01)
            if not readable:
                continue
            data = os.read(sys.stdin.fileno(), 4096)
            if not data:
                break  # controller exited; neutral and release in finally
            buffer += data
            if len(buffer) > 8192:
                raise ValueError('oversized PWM request')
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                request = json.loads(line)
                try:
                    reply(state.request(request))
                except Exception as exc:
                    pwm.neutral()
                    reply({'error': str(exc)})
                    stopping = True
                if request.get('action') == 'shutdown':
                    stopping = True
    except Exception as exc:
        reply({'error': str(exc)})
    finally:
        errors = pwm.close()
        if args.result:
            args.result.write_text(json.dumps({'cleanup_errors': errors,
                                  'boot_id': boot, 'signal_backend': 'rasadapter5a_uart' if args.esc_channel else 'rp1_hardware_pwm',
                                  'steering_backend': 'rasadapter5a_uart' if args.steering_channel else 'rp1_hardware_pwm',
                                  'steering_channel': args.steering_channel,
                                  'esc_backend': 'rasadapter5a_uart' if args.esc_channel else 'rp1_hardware_pwm',
                                  'esc_channel': args.esc_channel,
                                  'mcu_esc_holds_neutral_after_cleanup': bool(args.esc_channel),
                                  'mcu_steering_signal_disabled': False if args.steering_channel else None,
                                  'hardware_finite_wave': False}, indent=2) + '\n')


class Pi5BenchBackend:
    hardware_output = True
    signal_backend = 'rp1_hardware_pwm_guardian'
    forward_pulse_us = 1575
    ground_short_trial = False
    raised_load_probe = False
    load_probe = False

    def __init__(self, root, output, expected_boot, review, *, neutral_only=False,
                 motor_only=False, steering_only=False, combined=False, ground_short=False, ground_held=False, ground_continuous=False):
        validate_review(review, expected_boot, neutral_only=neutral_only,
                        motor_only=motor_only, steering_only=steering_only, combined=combined,
                        ground_short=ground_short, ground_held=ground_held, ground_continuous=ground_continuous)
        self.root, self.output, self.expected_boot = root, output, expected_boot
        self.review, self.neutral_only = review, neutral_only
        self.steering_backend = review.get('steering_backend', 'rp1_hardware_pwm')
        self.steering_center_us = review.get('steering_center_us', 1650)
        self.steering_boundary_center_trial = review.get('steering_boundary_center_trial_requested_by_user') is True
        if self.steering_boundary_center_trial and (
                self.steering_backend != 'rasadapter5a_uart' or review.get('steering_channel') != 3
                or self.steering_center_us != 1550 or neutral_only or motor_only):
            raise ValueError('boundary center trial requires the explicitly requested S3 1550-us reference')
        if type(self.steering_center_us) is not int or not (1550 < self.steering_center_us < 1750
                or self.steering_boundary_center_trial and self.steering_center_us == 1550):
            raise ValueError('invalid observed steering reference')
        if self.steering_center_us != 1650 and self.steering_backend != 'rasadapter5a_uart':
            raise ValueError('alternate steering reference is only supported by the UART board')
        self.esc_backend = review.get('esc_backend', 'rp1_hardware_pwm')
        if self.esc_backend == 'rasadapter5a_uart':
            self.signal_backend = 'rasadapter5a_uart_guardian'
        self.motor_available = not steering_only and not neutral_only
        self.steering_available = steering_only or combined or ground_short or ground_held or ground_continuous
        self.steering_pwm_available = (steering_only and self.steering_backend == 'rasadapter5a_uart'
                                       and review.get('steering_channel') == 3)
        self.steering_single_target = review.get('steering_single_target_verified') is True
        if self.steering_single_target and (self.steering_backend != 'rasadapter5a_uart' or not self.steering_available):
            raise ValueError('observed single target steering requires the S3 UART scope')
        self.steering_settle_s = .3 if self.steering_single_target else .2
        self.reverse_available = (review.get('reverse_physically_verified') is True
                                  or review.get('reverse_revalidation_prepared') is True) and not neutral_only
        self.ground_short_trial = ground_short
        self.ground_continuous_trial = ground_continuous
        self.ground_held_trial = ground_held or ground_continuous
        self.manual_session_limit_s = review.get('manual_session_limit_s', 180) if self.ground_held_trial else 180
        self.manual_motion_hold_max_s = review.get('hold_limit_s', 60) if self.ground_held_trial else 60
        self.combined_trial, self.parking_protection_trial = combined or ground_short or self.ground_held_trial, motor_only
        self.steering_active = False
        self.process = None
        self.log = None
        self.closed = False
        self.buffer = b''
        self.lock = threading.RLock()

    def read_reply(self, timeout=.7):
        deadline = time.monotonic() + timeout
        while b'\n' not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], max(0, remaining))[0]:
                raise RuntimeError('Pi 5 PWM guardian response timed out')
            data = os.read(self.process.stdout.fileno(), 4096)
            if not data:
                raise RuntimeError('Pi 5 PWM guardian stopped')
            self.buffer += data
        line, self.buffer = self.buffer.split(b'\n', 1)
        result = json.loads(line)
        if result.get('error') or result.get('fault'):
            raise RuntimeError(result.get('error') or result.get('fault'))
        return result

    def rpc(self, action, **values):
        with self.lock:
            if self.closed or self.process is None or self.process.poll() is not None:
                raise RuntimeError('Pi 5 PWM guardian unavailable')
            self.process.stdin.write((json.dumps({'action': action, **values}) + '\n').encode())
            self.process.stdin.flush()
            return self.read_reply()

    def open(self):
        audit = inspect_pi5()
        baseline_path = (self.root / self.review['baseline_record']).resolve()
        if not baseline_path.is_relative_to(self.root.resolve()):
            raise ValueError('Pi 5 baseline must belong to this workspace')
        baseline = json.loads(baseline_path.read_text())
        if self.steering_single_target:
            import hashlib
            evidence_path = (self.root/self.review['steering_single_target_evidence_record']).resolve()
            if not evidence_path.is_relative_to(self.root.resolve()):
                raise ValueError('single target evidence must belong to this workspace')
            raw = evidence_path.read_bytes()
            evidence = json.loads(raw)
            if (hashlib.sha256(raw).hexdigest() != self.review['steering_single_target_evidence_sha256']
                    or evidence.get('boot_id') != self.expected_boot
                    or evidence.get('right_turn_physically_verified') is not True
                    or evidence.get('returned_to_center_physically_verified') is not True
                    or evidence.get('target_us') != 1550
                    or evidence.get('center_us') != self.steering_center_us):
                raise ValueError('same-boot physical single target steering evidence is missing or changed')
        if self.ground_held_trial and not self.ground_continuous_trial:
            import hashlib
            evidence_path = (self.root/self.review['keyboard_stop_evidence_record']).resolve()
            if not evidence_path.is_relative_to(self.root.resolve()):
                raise ValueError('keyboard stop evidence must belong to this workspace')
            raw = evidence_path.read_bytes()
            evidence = json.loads(raw)
            if (hashlib.sha256(raw).hexdigest() != self.review['keyboard_stop_evidence_sha256']
                    or evidence.get('boot_id') != self.expected_boot
                    or evidence.get('keyboard_stop_physically_verified') is not True
                    or not evidence.get('key_release_during_forward_output_events')):
                raise ValueError('current keyboard stop observation is missing or changed')
        if (audit['boot_id'] != self.expected_boot or audit['power_flags'] != 0
                or baseline.get('boot_id') != self.expected_boot
                or baseline.get('duration_s', 0) < 30
                or not baseline.get('all_power_flags_zero') or not baseline.get('boot_unchanged')
                or len(baseline.get('samples', [])) < 30
                or any(x.get('power_flags') != 0 or x.get('boot_id') != self.expected_boot for x in baseline['samples'])):
            raise RuntimeError('fresh 30-second Pi 5 power baseline required')
        self.output.mkdir(parents=True, exist_ok=True)
        command = [sys.executable, '-m', 'carvision.pi5_pwm', '--guard',
                   '--expected-boot-id', self.expected_boot,
                   '--result', str(self.output / 'guardian-result.json')]
        if self.ground_continuous_trial:
            command.append('--continuous-manual')
        if not self.steering_available:
            command.append('--motor')
        else:
            command.append('--steering')
            if self.motor_available:
                command.append('--motor')
        if self.neutral_only:
            command.append('--neutral-only')
        if self.steering_available and self.steering_backend == 'rasadapter5a_uart':
            command.extend(['--steering-channel', str(self.review['steering_channel'])])
            command.extend(['--steering-center-us', str(self.steering_center_us)])
            if self.steering_boundary_center_trial:
                command.append('--steering-boundary-center-trial')
            if self.steering_single_target:
                command.append('--steering-single-target')
        if not self.steering_available or self.motor_available:
            if self.esc_backend == 'rasadapter5a_uart':
                command.extend(['--esc-channel', str(self.review['esc_channel'])])
        self.log = (self.output / 'guardian.log').open('w')
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log,
                                        env={**os.environ, 'PYTHONPATH': str(self.root / 'vision/src')},
                                        start_new_session=True, bufsize=0)
        self.read_reply(timeout=5)
        self.check()

    def motion(self, pulse, seconds):
        if not self.motor_available or pulse == 1300 and not self.reverse_available:
            raise RuntimeError('Pi 5 motor direction not reviewed')
        # Establish position holding before traction after startup/idle. A
        # retained UART target is not sufficient for an unloaded RP1 output.
        # Leave an active turn alone; its PWM already holds the chosen angle.
        if self.steering_available and not self.steering_active:
            self.steering(self.steering_center_us)
        self.rpc('motion', pulse=pulse, seconds=seconds)

    def neutral(self):
        if not self.closed and self.process is not None:
            self.rpc('neutral')

    def steering(self, pulse):
        if not self.steering_available:
            raise RuntimeError('Pi 5 steering disabled for this trial')
        self.rpc('steering', pulse=pulse)
        self.steering_active = True

    def steering_idle(self):
        if self.steering_active:
            self.rpc('steering_idle')
            self.steering_active = False

    def check(self):
        self.rpc('check')

    def close(self):
        if self.closed:
            return
        errors = []
        if self.process:
            try:
                self.rpc('shutdown')
            except Exception as exc:
                errors.append(str(exc))
            self.process.stdin.close()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                # TERM is handled by the owner; never force-kill live PWM.
                self.process.terminate()
                errors.append('PWM guardian cleanup delayed')
        self.closed = True
        if self.log:
            self.log.close()
        (self.output / 'result.json').write_text(json.dumps({
            'boot_id': self.expected_boot, 'signal_backend': self.signal_backend,
            'steering_backend': self.steering_backend,
            'esc_backend': self.esc_backend,
            'ground_short_trial': self.ground_short_trial,
            'continuous_ground_driving_enabled': self.ground_held_trial,
            'hardware_finite_wave': False, 'cleanup_errors': errors,
            'mechanical_results_require_user_observation': True}, indent=2) + '\n')


class Pi5RaisedHeldBackend(Pi5BenchBackend):
    """Three-second raised-wheel trials; no continuous ground permission."""
    raised_held_reviewed = True

    def __init__(self, root, output, expected_boot, review):
        if (review.get('raised_held_requested_by_user') is not True
                or review.get('keyboard_stop_physically_verified') is not True
                or review.get('continuous_ground_driving_enabled') is not False
                or review.get('esc_backend') != 'rasadapter5a_uart'
                or review.get('steering_backend') != 'rasadapter5a_uart'
                or review.get('steering_channel') != 3):
            raise ValueError('raised held trials require current keyboard stop evidence and S3/S4 preparation')
        super().__init__(root, output, expected_boot, review, combined=True)


class Pi5GroundHeldBackend(Pi5BenchBackend):
    """Explicit fixed-low-gear manual ground trial, separate from autonomy."""
    ground_held_reviewed = True

    def __init__(self, root, output, expected_boot, review):
        super().__init__(root, output, expected_boot, review, ground_held=True)
        # This first long-distance session has only current forward/stop
        # observations. A reverse review must be prepared separately.
        self.reverse_available = False


class Pi5ContinuousBackend(Pi5BenchBackend):
    """Operator-held manual input with no wall clock limit; guardian lease remains finite."""
    ground_held_reviewed = True
    manual_continuous_reviewed = True

    def __init__(self, root, output, expected_boot, review):
        super().__init__(root, output, expected_boot, review, ground_continuous=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--guard', action='store_true')
    parser.add_argument('--motor', action='store_true')
    parser.add_argument('--steering', action='store_true')
    parser.add_argument('--neutral-only', action='store_true')
    parser.add_argument('--continuous-manual', action='store_true')
    parser.add_argument('--steering-channel', type=int)
    parser.add_argument('--steering-center-us', type=int, default=1650)
    parser.add_argument('--steering-boundary-center-trial', action='store_true')
    parser.add_argument('--steering-single-target', action='store_true')
    parser.add_argument('--esc-channel', type=int)
    parser.add_argument('--expected-boot-id')
    parser.add_argument('--result', type=Path)
    options = parser.parse_args()
    if options.guard:
        if not options.expected_boot_id or not (options.motor or options.steering):
            parser.error('guardian requires expected boot and selected channels')
        if options.steering_single_target and options.steering_channel != 3:
            parser.error('single target steering requires the actual S3 channel')
        if options.steering_boundary_center_trial and (not options.steering
                or options.steering_channel != 3 or options.steering_center_us != 1550 or options.neutral_only):
            parser.error('boundary center trial requires S3 steering at exactly 1550 us')
        if options.continuous_manual and (not options.motor or not options.steering
                or options.neutral_only or options.steering_channel != 3 or options.esc_channel != 4):
            parser.error('continuous manual guardian requires the identified S3/S4 scope')
        guard_main(options)
    else:
        print(json.dumps(inspect_pi5(), indent=2))
