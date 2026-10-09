"""A boot waiting page with explicit ESC-off/on preparation and finite rounds."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

from carvision.boot_test import BootTestDrive, PiBootPreparation, reviewed_manifest
from carvision.config import load_config
from carvision.manual_drive import DeferredDrive, ManualDrive, RepeatableDrive, worker_control_endpoint
from carvision.web_preview import serve
from serve_bench_drive import OwnedWorkerDrive


def repeated_drive(root, context):
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
    return dict(ground_short_trial=not (continuous or steering), ground_held_trial=continuous,
                ground_continuous_trial=continuous, continuous_simulation=False, speed_adjustable=False,
                speed_percent=100, steering_available=True, motor_available=not steering, reverse_available=continuous,
                steering_pwm_available=steering, combined_trial=not steering, controls_paused=False, steering_center_us=review.get('steering_center_us', 1610),
                steering_pulse_us=None, steering_signal_active=False, steering_single_target=False,
                test_session_s=None if continuous else 120 if steering else 60, motion_hold_max_s=None,
                lease_ms=round(1000*(ManualDrive.continuous_lease_s if continuous else ManualDrive.lease_s)),
                release_required=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--template', type=Path)
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    root = args.root.resolve()
    template_path = args.template or root/'vision/deployment/boot-test-template.json'
    template = json.loads(template_path.read_text())
    if not (template.get('repeatable_ground_short_requested_by_user') is True
            or template.get('ground_continuous_requested_by_user') is True):
        raise ValueError('explicit repeatable trial authorization required')
    cameras = json.loads((root/'vision/configs/cameras.json').read_text())
    preparation = PiBootPreparation(root, template_path)
    profiles={'driving':initial_status(template), 'steering_pwm':initial_status({**template,
              'steering_pwm_mode':True,'ground_continuous_requested_by_user':False})}
    drive = BootTestDrive(lambda cancelled: preparation.prepare(cancelled, mode=drive.selected_mode),
                          preparation.confirm, lambda context: repeated_drive(root, context),
                          profiles['driving'], profiles=profiles)
    def finish(*_):
        raise SystemExit(0)
    for name in ('SIGTERM', 'SIGINT', 'SIGHUP'):
        signal.signal(getattr(signal, name), finish)
    preview = SimpleNamespace(camera=cameras['primary'], aux_camera=cameras.get('secondary'),
        host='0.0.0.0', port=args.port, width=640, height=480, fps=15, preview_fps=3,
        raw_preview_fps=15, secondary_raw_preview_fps=3, preview_width=480,
        jpeg_quality=55, stale_ms=1500, camera_format='MJPG', weights=None,
        classic_candidates=False, race_config=str(root/'vision/configs/race-2026.json'))
    try:
        serve(preview, load_config(None), drive=drive)
    finally:
        drive.close()


if __name__ == '__main__':
    main()
