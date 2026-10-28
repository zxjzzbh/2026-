"""Shared preparation-to-manual-worker construction for app and legacy launchers.

Importing this module starts no process and changes no hardware. Both entry
points use the same explicit preparation, worker arguments and cleanup owner.
"""
import os
from pathlib import Path
import subprocess
import time

from .boot_test import reviewed_manifest
from .manual_drive import (DeferredDrive, ManualDrive, RepeatableDrive,
                           worker_control_endpoint, CONTINUOUS_STARTUP_SETTLING_S)
from .manual_steering import manual_endpoints
from .worker_client import OwnedWorkerDrive


def repeated_drive(root, context, *, allow_autonomy_handoff=False):
    review = context['review']
    continuous = review.get('ground_continuous_requested_by_user') is True
    steering = review.get('steering_pwm_mode') is True
    worker = None
    number = 0

    def start_worker():
        nonlocal worker, number
        if worker is not None and worker.poll() is None:
            raise RuntimeError('上一轮控制进程尚未清理')
        reviewed_manifest(root, review)
        if Path('/proc/sys/kernel/random/boot_id').read_text().strip() != context['boot_id']:
            raise RuntimeError('开机状态已改变')
        number += 1
        output = Path(context['folder'])/('round-'+str(number).zfill(6))
        if output.exists():
            raise RuntimeError('本轮目录已存在')
        command = ['timeout', '--signal=TERM', '--kill-after=1s', '0s' if continuous else '810s', '/usr/bin/python3',
                   '-m', 'carvision.manual_hardware', '--root', str(root), '--output', str(output),
                   '--steering-only' if steering else '--ground-continuous' if continuous else '--ground-short',
                   '--bench-prepared' if steering else '--ground-prepared', '--expected-boot-id', context['boot_id'],
                   '--pi5-review', str(Path(context['folder'])/'review.json')]
        if allow_autonomy_handoff:
            command.append('--allow-autonomy-handoff')
        log = output.with_suffix('.worker.log').open('x')
        try:
            worker = subprocess.Popen(command, env={**os.environ, 'PYTHONPATH': str(root/'vision/src')},
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            endpoint = worker_control_endpoint(output)
            deadline = time.monotonic()+5
            while not endpoint.exists():
                if worker.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('控制进程未能启动，请关闭电调并检查本轮记录')
                time.sleep(.05)
            return OwnedWorkerDrive(endpoint, process=worker, worker_log=log, output=output,
                                    expected_boot=context['boot_id'], ground_short_trial=not (continuous or steering),
                                    ground_held=continuous,
                                    motor_available=not steering, steering_available=True)
        except BaseException:
            if worker is not None and worker.poll() is None:
                worker.terminate()
                worker.wait(timeout=5)
            log.close()
            raise

    wrapper = DeferredDrive if continuous else RepeatableDrive
    return wrapper(start_worker, initial_status(review))



def initial_status(review):
    continuous = review.get('ground_continuous_requested_by_user') is True
    steering = review.get('steering_pwm_mode') is True
    span = review.get('manual_steering_span_us', 0)
    left, right = manual_endpoints(review.get('steering_center_us', 1610), span)
    return dict(esc_confirmation_only=review.get('operator_confirmation_scope') == 'esc_state_only',
                startup_settling_s=1 if steering else CONTINUOUS_STARTUP_SETTLING_S if continuous else 12,
                ground_short_trial=not (continuous or steering), ground_held_trial=continuous,
                ground_continuous_trial=continuous, continuous_simulation=False, speed_adjustable=False,
                speed_percent=100, steering_available=True, motor_available=not steering, reverse_available=continuous,
                steering_pwm_available=steering, combined_trial=not steering, controls_paused=False, steering_center_us=review.get('steering_center_us', 1610),
                steering_pulse_us=None, steering_signal_active=False, steering_single_target=False,
                manual_steering_span_us=span, steering_left_us=left, steering_right_us=right,
                steering_pwm_min_us=right, steering_pwm_max_us=left,
                manual_endpoints_field_verified=False if span else None,
                test_session_s=None if continuous else 120 if steering else 60, motion_hold_max_s=None,
                lease_ms=round(1000*(ManualDrive.continuous_lease_s if continuous else ManualDrive.lease_s)),
                release_required=False)

