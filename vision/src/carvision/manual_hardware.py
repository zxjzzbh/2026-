"""Finite manual GPIO worker. Launched explicitly; never starts on boot.

The private PCM/DMA driver owns BCM13 and, when requested, BCM12.
BCM17/27 are not commanded here. Ordinary output remains locked after faults;
reviewed bench trials and a separately prepared finite ground trial have their own scope.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

from .manual_drive import ManualDrive, DriveError, worker_control_endpoint
from .steering_calibrate import CalibrationDriver
from .esc_wave import DmaEsc
from .esc_load_probe import LoadProbeEsc


def validate_state(state, *, motor_only=False, steering_only=False, combined=False, parking_protection=False, ground_short_trial=False, raised_load_probe=False):
    raised = state.get('raised_load_probe_review', {})
    raised_retry = raised_load_probe and all((
        raised.get('requested_by_user'), raised.get('fresh_wheels_raised_confirmed'),
        raised.get('fresh_esc_off_confirmed'), raised.get('servo_signals_disabled'),
        raised.get('baseline_power_flags') == 0, raised.get('baseline_record'), raised.get('boot_id'),
        raised.get('signal_backend') == 'dma_finite_wave', raised.get('signal_implementation_sha256'),
        raised.get('neutral_idle_physically_passed'), raised.get('neutral_result_record'),
        raised.get('neutral_result_sha256'), type(raised.get('forward_pulse_us')) is int, raised.get('forward_pulse_us') == 1600,
        raised.get('previous_forward_pulse_us') == 1575, raised.get('load_implementation_sha256'),
        raised.get('motion_pulse_max_s') == .8, raised.get('active_session_s') == 60,
        raised.get('continuous_ground_driving_enabled') is False,
    ))
    if raised_load_probe and not raised_retry:
        raise ValueError('raised load probe requires fresh raised wheels, ESC-off, clean power and exact one-step profile')
    ground = state.get('ground_short_review', {})
    ground_retry = ground_short_trial and all((
        not ground.get('blocked_by_auto_restart', False),
        ground.get('requested_by_user'), ground.get('fresh_esc_off_confirmed'),
        ground.get('clear_area_confirmed'), ground.get('operator_switch_in_reach'),
        (ground.get('manual_restart_confirmed') or ground.get('supply_fix_confirmed_by_user')),
        ground.get('gimbal_signals_disabled'),
        ground.get('baseline_power_flags') == 0, ground.get('baseline_record'), ground.get('boot_id'),
        ground.get('signal_backend') == 'dma_finite_wave', ground.get('signal_implementation_sha256'),
        ground.get('neutral_idle_physically_passed'), ground.get('neutral_result_record'),
        ground.get('neutral_result_sha256'), ground.get('space_stop_physically_verified'),
        ground.get('space_result_record'), ground.get('space_result_sha256'), ground.get('space_evidence_boot_id'),
        ground.get('motion_pulse_max_s') == .8, ground.get('active_session_s') == 60,
        ground.get('continuous_ground_driving_enabled') is False,
    ))
    if ground_short_trial and not ground_retry:
        raise ValueError('ground short trial needs fresh clear area, ESC-off, clean power and finite DMA stop evidence')
    if ground_short_trial and ground.get('forward_pulse_us', 1575) != 1575:
        probe = ground.get('forward_load_probe', {})
        if not all((type(ground.get('forward_pulse_us')) is int, ground.get('forward_pulse_us') == 1600, probe.get('requested_by_user'),
                    probe.get('previous_us') == 1575, probe.get('target_us') == 1600,
                    probe.get('esc_off_confirmed'), probe.get('signal_check_record'),
                    probe.get('signal_check_sha256'), probe.get('implementation_sha256'))):
            raise ValueError('increased forward point needs its explicit finite load probe review')
    joint = state.get('dma_combined_revalidation_review', {})
    dma_joint_retry = combined and all((
        joint.get('requested_by_user'), joint.get('neutral_idle_physically_passed'),
        joint.get('signal_backend') == 'dma_finite_wave', joint.get('neutral_result_record'),
        joint.get('neutral_result_sha256'), joint.get('space_result_record'),
        joint.get('space_result_sha256'), joint.get('space_stop_physically_verified'),
        joint.get('fresh_esc_off_confirmed'), joint.get('fresh_wheels_raised_confirmed'),
        joint.get('gimbal_signals_disabled'), joint.get('baseline_power_flags') == 0,
        joint.get('baseline_record'), joint.get('boot_id'),
        joint.get('continuous_ground_driving_enabled') is False,
    ))
    dma = state.get('dma_stop_revalidation_review', {})
    dma_retry = parking_protection and all((
        dma.get('requested_by_user'), dma.get('neutral_idle_physically_passed'),
        dma.get('signal_backend') == 'dma_finite_wave',
        dma.get('neutral_result_record'), dma.get('neutral_result_sha256'),
        dma.get('timing_comparison_record'), dma.get('fresh_esc_off_confirmed'),
        dma.get('fresh_wheels_raised_confirmed'), dma.get('servo_signals_disabled'),
        dma.get('baseline_power_flags') == 0, dma.get('baseline_record'), dma.get('boot_id'),
        dma.get('boot_id') == state.get('parking_protection_review', {}).get('boot_id'),
    ))
    if state.get('motor_stop_review', {}).get('unresolved_persistent_rotation') and not (dma_retry or dma_joint_retry or ground_retry or raised_retry):
        raise ValueError('unexpected persistent wheel rotation is unresolved; real output is paused')
    if sum((motor_only, steering_only, combined, parking_protection, ground_short_trial, raised_load_probe)) > 1:
        raise ValueError('select only one isolated trial scope')
    esc, steer = state.get('esc', {}), state.get('steering', {})
    steering_review = state.get('steering_only_review', {})
    steering_retry = steering_only and all((
        steering_review.get('requested_by_user'),
        steering_review.get('motor_and_gimbal_signals_disabled'),
        steering_review.get('center_power_review_passed'),
        steering_review.get('center_mechanical_review_passed'),
        steering_review.get('fresh_wheels_raised_confirmed'),
        steering_review.get('fresh_esc_off_confirmed'),
        steering_review.get('baseline_power_flags') == 0,
    ))
    if steering_only and not steering_retry:
        raise ValueError('steering-only trial requires a fresh isolated center and power review')
    review = state.get('combined_review', {})
    previous = [state.get(name, {}) for name in ('motor_only_review', 'steering_only_review')]
    combined_retry = combined and all((
        review.get('requested_by_user'), review.get('gimbal_signals_disabled'),
        review.get('fresh_wheels_raised_confirmed'), review.get('fresh_esc_off_confirmed'),
        review.get('baseline_power_flags') == 0, review.get('baseline_record'),
        review.get('boot_id'),
        all(r.get('keyboard_control_verified') and r.get('power_flags_after') == 0
            and r.get('boot_unchanged') and r.get('boot_id') == review.get('boot_id') for r in previous),
    ))
    if combined and not (combined_retry or dma_joint_retry):
        raise ValueError('combined trial requires current clean baseline and verified isolated trials on this boot')
    parking_review = state.get('parking_protection_review', {})
    parking_retry = parking_protection and all((
        parking_review.get('requested_by_user'), parking_review.get('servo_signals_disabled'),
        parking_review.get('forward_only'), (parking_review.get('latest_restart_confirmed_manual')
                                             or parking_review.get('latest_restart_requested_by_controller')),
        parking_review.get('fresh_wheels_raised_confirmed'), parking_review.get('fresh_esc_off_confirmed'),
        parking_review.get('baseline_power_flags') == 0, parking_review.get('baseline_record'),
        parking_review.get('boot_id'),
    ))
    if parking_protection and not parking_retry:
        raise ValueError('parking protection requires fresh forward-only physical and clean power review')
    if state.get('manual_bench_power_review', {}).get('further_motion_paused'):
        review = state.get('motor_only_review', {})
        if not (raised_retry or ground_retry or steering_retry or combined_retry or dma_joint_retry or parking_retry or motor_only and review.get('requested_by_user')
                and review.get('servo_signals_disabled')
                and review.get('baseline_power_flags') == 0):
            raise ValueError('bench undervoltage is unresolved; review the power supply before motion')
    if state.get('restart_review', {}).get('further_motion_paused') and not (raised_retry or ground_retry or steering_retry or combined_retry or dma_joint_retry or parking_retry):
        raise ValueError('restart investigation is unresolved')
    if (not (raised_retry or ground_retry or state.get('wheels_raised_confirmed')) or not state.get('esc_original_confirmed')
            or not state.get('battery_original_confirmed')
            or not esc.get('reference_observed_static') or esc.get('reference_us') != 1500
            or esc.get('positive_test_us') != 1575 or esc.get('positive_direction') != 'forward'
            or not esc.get('positive_stop_observed') or not esc.get('reverse_function_verified')
            or esc.get('negative_test_us') != 1300 or esc.get('negative_direction') != 'reverse'
            or not esc.get('reverse_stop_observed') or esc.get('operating_mode_label') != 'F/B/R'
            or not esc.get('operating_mode_confirmed_by_user')
            or not esc.get('operating_mode_label_verified_from_photo')
            or [steer.get(k) for k in ('left_us', 'center_us', 'right_us')] != [1750, 1650, 1550]):
        raise ValueError('requires the actual observed short-trial profile; no guessed endpoints')


class PiBenchBackend:
    hardware_output = True

    def __init__(self, root, output, expected_boot, state, *, motor_only=False, steering_only=False, combined=False, parking_protection=False, ground_short_trial=False, raised_load_probe=False):
        validate_state(state, motor_only=motor_only, steering_only=steering_only, combined=combined,
                       parking_protection=parking_protection, ground_short_trial=ground_short_trial, raised_load_probe=raised_load_probe)
        motor_only = motor_only or parking_protection or raised_load_probe
        self.motor_only, self.steering_available = motor_only, not motor_only
        self.steering_only, self.motor_available = steering_only, not steering_only
        self.ground_short_trial = ground_short_trial
        self.raised_load_probe = raised_load_probe
        self.combined_trial = combined or ground_short_trial
        self.parking_protection_trial = parking_protection
        self.signal_backend = DmaEsc.signal_backend
        self.dma_review = (state.get('raised_load_probe_review', {}) if raised_load_probe else
                          state.get('ground_short_review', {}) if ground_short_trial else
                          state.get('dma_stop_revalidation_review', {}) if parking_protection
                           else state.get('dma_combined_revalidation_review', {}) if combined else {})
        self.reverse_available = not (parking_protection or raised_load_probe)
        self.steering_active = False
        review_name = ('parking_protection_review' if parking_protection else 'combined_review' if combined
                       else 'steering_only_review' if steering_only else 'motor_only_review')
        self.review_boot = state.get(review_name, {}).get('boot_id')
        if self.dma_review:
            if (self.ground_short_trial or self.raised_load_probe) and hashlib.sha256(Path(__file__).with_name('esc_wave.py').read_bytes()).hexdigest() != self.dma_review['signal_implementation_sha256']:
                raise RuntimeError('DMA signal implementation changed since physical stop evidence')
            self.review_boot = self.dma_review.get('boot_id')
        self.forward_pulse_us = self.dma_review.get('forward_pulse_us', 1575) if ground_short_trial or raised_load_probe else 1575
        self.load_probe = self.forward_pulse_us == 1600
        self.root, self.output, self.expected_boot = root, output, expected_boot
        self.motor, self.client, self.process, self.log = None, None, None, None
        self.closed = False
        self.flags_before = self.flags_after = None
        self.pins_before = None
        self.last_power_check = 0

    def power(self):
        return int(subprocess.check_output(['vcgencmd', 'get_throttled'], text=True,
                                          timeout=1).strip().split('=')[1], 16)

    def pins(self):
        return subprocess.check_output(['pinctrl', 'get', '12,13,17,27'], text=True, timeout=1)

    def open(self):
        if b'Raspberry Pi 4 Model B' not in Path('/proc/device-tree/model').read_bytes():
            raise RuntimeError('requires the prepared Pi 4B')
        if Path('/proc/sys/kernel/random/boot_id').read_text().strip() != self.expected_boot:
            raise RuntimeError('boot changed since physical preparation')
        if self.dma_review:
            baseline_path = (self.root/self.dma_review['baseline_record']).resolve()
            if not baseline_path.is_relative_to(self.root.resolve()):
                raise RuntimeError('power baseline must belong to this workspace')
            baseline = json.loads(baseline_path.read_text())
            if baseline.get('boot_id') != self.expected_boot or baseline.get('power_flags') != 0:
                raise RuntimeError('fresh clean power baseline does not match this boot')
            if self.raised_load_probe:
                if (not baseline.get('wheels_raised_confirmed_by_user')
                        or baseline.get('duration_s', 0) < 30 or not baseline.get('all_power_flags_zero')
                        or not baseline.get('boot_unchanged') or len(baseline.get('samples', [])) < 30
                        or any(s.get('power_flags') != 0 or s.get('boot_id') != self.expected_boot for s in baseline['samples'])):
                    raise RuntimeError('raised load probe needs a verified fresh 30-second clean baseline')
                if hashlib.sha256(Path(__file__).with_name('esc_load_probe.py').read_bytes()).hexdigest() != self.dma_review['load_implementation_sha256']:
                    raise RuntimeError('raised load probe implementation changed after review')
            if self.ground_short_trial and self.dma_review.get('supply_fix_confirmed_by_user'):
                if (not baseline.get('supply_connections_resecured_confirmed_by_user')
                        or not baseline.get('all_power_flags_zero') or not baseline.get('boot_unchanged')
                        or baseline.get('duration_s', 0) < 30 or len(baseline.get('samples', [])) < 30
                        or any(s.get('power_flags') != 0 or s.get('boot_id') != self.expected_boot for s in baseline['samples'])):
                    raise RuntimeError('post-fix ground trial needs a verified 30-second clean power record')
            if self.load_probe and self.ground_short_trial:
                probe = self.dma_review['forward_load_probe']
                source = Path(__file__).with_name('esc_load_probe.py')
                if hashlib.sha256(source.read_bytes()).hexdigest() != probe['implementation_sha256']:
                    raise RuntimeError('load probe implementation changed after signal review')
                check_path = (self.root/probe['signal_check_record']).resolve()
                if not check_path.is_relative_to(self.root.resolve()):
                    raise RuntimeError('load signal review must belong to this workspace')
                raw = check_path.read_bytes(); check = json.loads(raw)
                if (hashlib.sha256(raw).hexdigest() != probe['signal_check_sha256']
                        or check.get('boot_id') != self.expected_boot or not check.get('passed')
                        or not check.get('esc_off_confirmed') or check.get('forward_pulse_us') != 1600
                        or check.get('neutral_pulse_us') != 1500 or check.get('power_flags_after') != 0
                        or check.get('cleanup_errors') or not check.get('pins_are_inputs')):
                    raise RuntimeError('finite load probe signal check is incomplete')
            evidence = (self.root/self.dma_review['neutral_result_record']).resolve()
            if not evidence.is_relative_to(self.root.resolve()):
                raise RuntimeError('neutral evidence must belong to this workspace')
            raw = evidence.read_bytes()
            neutral = json.loads(raw)
            if (hashlib.sha256(raw).hexdigest() != self.dma_review['neutral_result_sha256']
                    or not neutral.get('physical_neutral_idle_passed')
                    or neutral.get('nominal_pulse_us') != 1500
                    or neutral.get('forward_or_reverse_sent') is not False
                    or not neutral.get('esc_off_confirmed_after')
                    or neutral.get('cleanup_errors')):
                raise RuntimeError('physical DMA neutral evidence is incomplete or changed')
            if self.combined_trial:
                space_path = (self.root/self.dma_review['space_result_record']).resolve()
                if not space_path.is_relative_to(self.root.resolve()):
                    raise RuntimeError('space evidence must belong to this workspace')
                raw = space_path.read_bytes()
                space = json.loads(raw)
                if (hashlib.sha256(raw).hexdigest() != self.dma_review['space_result_sha256']
                        or not space.get('space_stop_physically_verified')
                        or not space.get('space_request_interrupts_active_motion')
                        or space.get('boot_id_after') != (self.dma_review['space_evidence_boot_id'] if self.ground_short_trial else self.expected_boot)
                        or space.get('power_flags_after') != 0 or space.get('cleanup_errors')
                        or not space.get('esc_off_confirmed_after')):
                    raise RuntimeError('current physical DMA stop evidence is incomplete or changed')
        self.pins_before = self.pins()
        if len(self.pins_before.splitlines()) != 4 or any('= input' not in s for s in self.pins_before.splitlines()):
            raise RuntimeError('another output owner is present')
        self.flags_before = self.power()
        if (self.ground_short_trial or self.raised_load_probe) and self.flags_before != 0:
            raise RuntimeError('ground short trial requires zero current and historical power flags')
        if self.flags_before & 1:
            raise RuntimeError('active undervoltage')
        if (self.motor_only or self.steering_only or self.combined_trial) and (self.flags_before & 0x10000 or self.review_boot != self.expected_boot):
            raise RuntimeError('isolated trial needs a fresh clean power baseline on this boot')
        if subprocess.run(['pgrep', '-x', 'pigpiod'], capture_output=True).returncode != 1:
            raise RuntimeError('existing driver preserved')
        if self.motor_available or self.steering_available:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 8890))
            subprocess.run(['sudo', '-n', 'true'], check=True, timeout=1)
            driver = self.root / 'drivers/pigpio'
            self.log = (self.output/'signal-driver.log').open('w')
            pin_mask = '0x1000' if self.steering_only else '0x2000' if self.motor_only else '0x3000'
            self.process = subprocess.Popen(
                ['sudo', '-n', 'timeout', '--signal=TERM', '--kill-after=1s',
                 '800s' if self.combined_trial or self.parking_protection_trial or self.raised_load_probe else '140s',
                 'env', f'LD_LIBRARY_PATH={driver}', str(driver/'pigpiod'), '-g', '-l', '-f',
                 '-n', '127.0.0.1', '-t', '1', '-x', pin_mask, '-p', '8890'],
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            if not self.motor_only:
                self.client = CalibrationDriver(8890, [(p, .02) for p in range(1550, 1751, 10)])
            if not self.steering_only:
                self.motor = (LoadProbeEsc if self.load_probe else DmaEsc)(8890)
            connection = self.client or self.motor
            deadline = time.monotonic() + 1.8
            while True:
                try:
                    connection.open()
                    break
                except OSError:
                    connection.close()
                    if time.monotonic() >= deadline or self.process.poll() is not None:
                        raise RuntimeError('steering driver did not start')
                    time.sleep(.03)
            if any(connection.mode(pin) != 0 for pin in (12, 13, 17, 27)):
                raise RuntimeError('pin ownership changed during startup')
        if not self.steering_only:
            if self.client:
                self.motor.open()
            self.motor.prepare()
            if self.client:
                self.steering(1650)
        if self.steering_only:
            print('STEERING_ONLY_READY: BCM12 idle; motor and gimbal signals disabled', flush=True)
            return
        print('BENCH_NEUTRAL_ACTIVE: motor 1500 us; '+
              ('all servo signals disabled' if self.motor_only else 'steering center 1650 us'), flush=True)

    def motion(self, pulse, seconds):
        if not self.motor_available:
            raise RuntimeError('motor output disabled for steering-only trial')
        if pulse not in (self.forward_pulse_us, 1300) or not 0 < seconds <= .8:
            raise ValueError('only observed finite commands are allowed')
        if (self.parking_protection_trial or self.raised_load_probe) and pulse != self.forward_pulse_us:
            raise ValueError('parking protection trial is forward-only')
        if self.closed:
            raise RuntimeError('hardware worker already closed')
        self.motor.start_brief_command(pulse, seconds)

    def neutral(self):
        if self.closed or self.motor is None or not self.motor.claimed:
            return
        self.motor.neutral()

    def steering(self, pulse):
        if self.closed:
            raise RuntimeError('hardware worker already closed')
        if self.motor_only:
            raise RuntimeError('servo output disabled for motor-only trial')
        self.client.set_servo(12, pulse)
        if self.client.servo_pulse(12) != pulse:
            raise RuntimeError('steering pulse not acknowledged')
        self.steering_active = True

    def steering_idle(self):
        if (self.steering_only or self.combined_trial) and self.steering_active and not self.closed:
            self.client.set_servo(12, 0)
            if self.client.servo_pulse(12) != 0:
                raise RuntimeError('steering pulse off not acknowledged')
            self.client.release(12)
            if self.client.mode(12) != 0:
                raise RuntimeError('steering idle release not acknowledged')
            self.steering_active = False

    def check(self):
        if self.closed:
            return
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('GPIO timing driver stopped')
        if self.motor:
            self.motor.check()
        if time.monotonic() - self.last_power_check >= .25:
            self.last_power_check = time.monotonic()
            self.flags_after = self.power()
            if self.flags_after & 1 or (self.flags_after & 0x10000 and not self.flags_before & 0x10000):
                raise RuntimeError('new undervoltage; stop bench motion')
            if Path('/proc/sys/kernel/random/boot_id').read_text().strip() != self.expected_boot:
                raise RuntimeError('boot changed; stop bench motion')

    def close(self):
        if self.closed:
            return
        self.closed = True
        errors = []
        if self.motor:
            errors.extend(self.motor.shutdown())
        if self.client:
            try:
                self.client.set_servo(12, 0)
                self.client.release(12)
                self.steering_active = False
            except Exception as exc:
                errors.append(str(exc))
            self.client.close()
        if self.process and self.process.poll() is None:
            # This is the process group created above; its GPIO driver is root.
            subprocess.run(['sudo', '-n', 'kill', '-TERM', '--', '-'+str(self.process.pid)],
                           check=False, timeout=1, capture_output=True)
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                errors.append('driver did not stop; independent timeout still applies')
        if self.log:
            self.log.close()
        after = self.pins()
        result = {'pins_before': self.pins_before, 'pins_after': after,
                  'pins_are_inputs': len(after.splitlines()) == 4 and all('= input' in s for s in after.splitlines()),
                  'other_pins_preserved': self.pins_before is not None and
                  [s for s in self.pins_before.splitlines() if 'GPIO17 =' in s or 'GPIO27 =' in s] ==
                  [s for s in after.splitlines() if 'GPIO17 =' in s or 'GPIO27 =' in s],
                  'power_flags_before': self.flags_before, 'power_flags_after': self.power(),
                  'boot_id_after': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                  'expected_boot_id': self.expected_boot, 'cleanup_errors': errors,
                  'motor_only': self.motor_only,
                  'steering_only': self.steering_only,
                  'combined_trial': self.combined_trial,
                  'ground_short_trial': self.ground_short_trial,
                  'forward_pulse_us': self.forward_pulse_us,
                  'load_probe': self.load_probe,
                  'raised_load_probe': self.raised_load_probe,
                  'parking_protection_trial': self.parking_protection_trial,
                  'signal_backend': self.signal_backend,
                  'motor_signal_pin_preserved': self.pins_before is not None and
                  [s for s in self.pins_before.splitlines() if 'GPIO13 =' in s] ==
                  [s for s in after.splitlines() if 'GPIO13 =' in s],
                  'servo_signal_pins_preserved': self.pins_before is not None and
                  [s for s in self.pins_before.splitlines() if 'GPIO13 =' not in s] ==
                  [s for s in after.splitlines() if 'GPIO13 =' not in s],
                  'mechanical_results_require_user_observation': True}
        p = self.output/'result.json.tmp'
        p.write_text(json.dumps(result, indent=2)+'\n')
        p.replace(self.output/'result.json')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--expected-boot-id', required=True)
    p.add_argument('--pi5-review', type=Path, help='fresh Pi 5 preparation/evidence; never reuse Pi 4 DMA reviews')
    preparation = p.add_mutually_exclusive_group()
    preparation.add_argument('--bench-prepared', action='store_true')
    preparation.add_argument('--ground-prepared', action='store_true')
    scope = p.add_mutually_exclusive_group()
    scope.add_argument('--motor-only', action='store_true')
    scope.add_argument('--steering-only', action='store_true')
    scope.add_argument('--combined', action='store_true')
    scope.add_argument('--parking-protection', action='store_true')
    scope.add_argument('--ground-short', action='store_true')
    scope.add_argument('--raised-load-probe', action='store_true')
    scope.add_argument('--raised-held', action='store_true', help='reviewed Pi 5 raised-wheel holds up to 3 seconds')
    scope.add_argument('--ground-held', action='store_true', help='reviewed fixed-low-gear manual ground session')
    args = p.parse_args()
    ground_mode = args.ground_short or args.ground_held
    if ground_mode and not args.ground_prepared or args.ground_prepared and not ground_mode:
        raise ValueError('ground short trial requires explicit ground preparation')
    if not (args.bench_prepared or args.ground_prepared):
        raise ValueError('fresh raised-wheel/ESC-off physical preparation required')
    parent = Path(f'/proc/{os.getppid()}/cmdline').read_bytes().split(b'\0')
    if args.raised_held and (not args.pi5_review or not args.bench_prepared):
        raise ValueError('raised held mode requires a Pi 5 review and raised-wheel preparation')
    if args.ground_held and not args.pi5_review:
        raise ValueError('ground held mode requires a separate Pi 5 ground review')
    extended = args.pi5_review or args.combined or args.parking_protection or args.ground_short or args.raised_load_probe or args.raised_held
    expected_timeout = b'810s' if extended else b'150s'
    if not parent or Path(os.fsdecode(parent[0])).name != 'timeout' or expected_timeout not in parent:
        raise RuntimeError('matching independent worker timeout required')
    state = json.loads((args.root/'vision/configs/bench-calibration.json').read_text())
    if not state['esc'].get('powered_off_confirmed_by_user'):
        raise ValueError('ESC must be confirmed off before startup')
    args.output.mkdir(parents=True, exist_ok=False)
    if args.pi5_review:
        from .pi5_pwm import Pi5BenchBackend, Pi5RaisedHeldBackend, Pi5GroundHeldBackend
        if args.parking_protection or args.raised_load_probe:
            raise ValueError('Pi 5 uses fresh isolated bench scopes; Pi 4 DMA/ground profiles cannot be reused')
        review = json.loads(args.pi5_review.read_text())
        if args.ground_held:
            backend = Pi5GroundHeldBackend(args.root, args.output, args.expected_boot_id, review)
        elif args.raised_held:
            backend = Pi5RaisedHeldBackend(args.root, args.output, args.expected_boot_id, review)
        else:
            backend = Pi5BenchBackend(args.root, args.output, args.expected_boot_id, review,
                                     motor_only=args.motor_only, steering_only=args.steering_only,
                                     combined=args.combined, ground_short=args.ground_short)
    else:
        backend = PiBenchBackend(args.root, args.output, args.expected_boot_id, state,
                                 motor_only=args.motor_only, steering_only=args.steering_only, combined=args.combined,
                                 parking_protection=args.parking_protection, ground_short_trial=args.ground_short, raised_load_probe=args.raised_load_probe)
    controller, listener = None, None
    endpoint = worker_control_endpoint(args.output)
    private_socket_folder = endpoint.parent if endpoint.parent != args.output.resolve() else None
    if private_socket_folder is not None:
        # Refuse a stale or unowned folder before opening any hardware. Only
        # this worker creates and removes its private, per-round IPC directory.
        private_socket_folder.mkdir(mode=0o700)
    stop = False
    def handle_stop(*_):
        nonlocal stop
        stop = True
    for name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
        signal.signal(getattr(signal, name), handle_stop)
    try:
        backend.open()
        def record(event):
            with (args.output/'events.jsonl').open('a') as stream:
                stream.write(json.dumps(event, ensure_ascii=False)+'\n')
        controller = ManualDrive(backend, event_sink=record, settling_s=1 if args.steering_only else 12,
                                 session_s=backend.manual_session_limit_s if args.ground_held else 60 if args.ground_short or args.raised_load_probe or args.raised_held else 120 if args.pi5_review else 180 if extended else 120,
                                 preparation_s=120 if args.ground_held and backend.manual_session_limit_s == 600 else 600 if args.pi5_review else 120 if args.ground_short or args.raised_load_probe else 600 if extended else None,
                                 raised_held=args.raised_held, ground_held=args.ground_held)
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        listener.bind(str(endpoint))
        os.chmod(endpoint, 0o600)
        listener.listen(8)
        listener.settimeout(.1)
        controller.start()
        while not stop and controller.status()['mode'] not in ('fault', 'expired'):
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            with connection:
                connection.settimeout(.2)
                try:
                    with connection.makefile('rb') as stream:
                        request = json.loads(stream.readline(4097))
                    action = request.get('action')
                    if action == 'shutdown':
                        stop = True
                        controller.emergency('控制服务退出')
                        status = controller.status()
                    elif action == 'status':
                        status = controller.status()
                    else:
                        status = controller.request(action, request.get('payload'))
                    reply = {'status': status}
                except (ValueError, OSError) as exc:
                    reply = {'error': str(exc)}
                try:
                    connection.sendall((json.dumps(reply, ensure_ascii=False)+'\n').encode())
                except OSError:
                    pass
    finally:
        mode_before_close = controller.status()['mode'] if controller else 'startup_failed'
        if controller:
            controller.close()
        else:
            backend.close()
        if listener:
            listener.close()
        endpoint.unlink(missing_ok=True)
        if private_socket_folder is not None:
            private_socket_folder.rmdir()
        (args.output/'trial-completion.json').write_text(json.dumps({
            'boot_id': args.expected_boot_id, 'mode_before_close': mode_before_close,
            'cleanup_completed': True}, indent=2)+'\n')


if __name__ == '__main__':
    main()
