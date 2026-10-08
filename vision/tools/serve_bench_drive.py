"""Camera page plus finite manual bench controls. Default is simulation.

Hardware mode requires a separately bounded GPIO worker and current physical
preparation. This never marks race calibration complete or runs at boot.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

from carvision.config import load_config
from carvision.manual_drive import DeferredDrive, RepeatableDrive, ManualDrive, PausedDrive, SimulationBackend, WorkerDrive, worker_control_endpoint
from carvision.web_preview import serve


class OwnedWorkerDrive(WorkerDrive):
    """A finished socket alone is insufficient proof to create another owner."""
    def __init__(self, path, *, process, worker_log, output, expected_boot, **options):
        super().__init__(path, **options)
        self.process, self.worker_log = process, worker_log
        self.output, self.expected_boot = output, expected_boot
        self.closed = False

    def status(self):
        state = super().status()
        normal = False
        if self.process.poll() == 0:
            try:
                completed = json.loads((self.output/'trial-completion.json').read_text())
                result = json.loads((self.output/'result.json').read_text())
                guardian = json.loads((self.output/'guardian-result.json').read_text())
                normal = (completed['boot_id'] == result['boot_id'] == guardian['boot_id'] == self.expected_boot
                          and completed['mode_before_close'] == 'expired'
                          and completed['cleanup_completed'] is True
                          and result['cleanup_errors'] == guardian['cleanup_errors'] == [])
            except (OSError, ValueError, KeyError):
                pass
        if normal:
            state.update(mode='expired', reason='本轮已结束，确认停稳后可开始下一轮',
                         motor_direction='stop', motor_pulse_us=1500, session_remaining_s=0)
        elif self.process.poll() is not None:
            state.update(mode='fault', reason='本轮未完成正常清理，请关闭电调后检查')
        return {**state, 'trial_completed_normally': normal}

    def close(self):
        if self.closed:
            return
        try:
            super().close()
        finally:
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=5)
            finally:
                self.worker_log.close()
                self.closed = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--camera-fps', type=int, default=10)
    parser.add_argument('--preview-fps', type=int, default=3)
    parser.add_argument('--raw-preview-fps', type=float, default=10)
    parser.add_argument('--secondary-raw-preview-fps', type=float, default=3)
    parser.add_argument('--classic-candidates', action='store_true',
                        help='show experimental race object cues for observation-only field checks')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--run', action='store_true')
    mode.add_argument('--controls-paused', action='store_true')
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument('--motor-only', action='store_true')
    scope.add_argument('--steering-only', action='store_true')
    scope.add_argument('--combined', action='store_true')
    scope.add_argument('--parking-protection', action='store_true')
    scope.add_argument('--ground-short', action='store_true')
    scope.add_argument('--raised-load-probe', action='store_true')
    scope.add_argument('--raised-held', action='store_true', help='reviewed raised-wheel keyboard holds, at most 3 seconds')
    scope.add_argument('--ground-held', action='store_true', help='reviewed low-gear continuous manual ground session')
    parser.add_argument('--repeatable-tests', action='store_true',
                        help='explicitly start another finite ground-short round after verified normal cleanup')
    preparation = parser.add_mutually_exclusive_group()
    preparation.add_argument('--bench-prepared', action='store_true')
    preparation.add_argument('--ground-prepared', action='store_true')
    parser.add_argument('--expected-boot-id')
    parser.add_argument('--pi5-review', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 5 <= args.camera_fps <= 30 or not 1 <= args.preview_fps <= 5:
        parser.error('camera FPS must be 5..30 and preview FPS must be 1..5')
    if not 1 <= args.raw_preview_fps <= 30 or not 1 <= args.secondary_raw_preview_fps <= 30:
        parser.error('raw preview FPS must be 1..30')
    if args.raised_held and (not args.run or not args.pi5_review or not args.bench_prepared):
        parser.error('raised held mode requires hardware mode, a Pi 5 review and raised-wheel preparation')
    if args.ground_held and (not args.run or not args.pi5_review or not args.ground_prepared):
        parser.error('ground held mode requires hardware mode, a separate Pi 5 review and ground preparation')
    if args.repeatable_tests and not (args.run and args.ground_short and args.pi5_review and args.ground_prepared):
        parser.error('repeatable tests require reviewed Pi 5 ground-short mode')
    if os.name == 'posix':
        def finish_service(*_):
            # Keep cleanup in finally, including the separately owned GPIO worker.
            raise SystemExit(0)
        for name in ('SIGTERM', 'SIGHUP'):
            signal.signal(getattr(signal, name), finish_service)
    root = args.root.resolve()
    cameras = json.loads((root/'vision/configs/cameras.json').read_text())
    worker, log = None, None
    round_number = 0
    drive = None
    try:
        if args.run:
            if not (args.ground_prepared if args.ground_short or args.ground_held else args.bench_prepared) or not args.expected_boot_id or args.output is None:
                raise ValueError('hardware mode needs current physical preparation, boot ID and new output folder')
            if os.name != 'posix' or args.output.exists():
                raise ValueError('Pi hardware mode requires a new output directory')
            args.output.parent.mkdir(parents=True, exist_ok=True)
            command = ['timeout', '--signal=TERM', '--kill-after=1s', '810s' if args.pi5_review or args.combined or args.parking_protection or args.ground_short or args.raised_load_probe else '150s', '/usr/bin/python3',
                       '-m', 'carvision.manual_hardware', '--root', str(root), '--output', str(args.output),
                       '--expected-boot-id', args.expected_boot_id, '--ground-prepared' if args.ground_short or args.ground_held else '--bench-prepared']
            if args.motor_only:
                command.append('--motor-only')
            if args.steering_only:
                command.append('--steering-only')
            if args.combined:
                command.append('--combined')
            if args.parking_protection:
                command.append('--parking-protection')
            if args.ground_short:
                command.append('--ground-short')
            if args.raised_load_probe:
                command.append('--raised-load-probe')
            if args.raised_held:
                command.append('--raised-held')
            if args.ground_held:
                command.append('--ground-held')
            if args.pi5_review:
                command.extend(['--pi5-review', str(args.pi5_review.resolve())])
            if args.repeatable_tests:
                review = json.loads(args.pi5_review.read_text())
                if review.get('repeatable_ground_short_requested_by_user') is not True:
                    raise ValueError('repeatable rounds require explicit user authorization')
            def start_worker():
                nonlocal worker, log, round_number
                if args.repeatable_tests:
                    if worker is not None and worker.poll() is None:
                        raise RuntimeError('previous hardware owner has not exited')
                    manifest_path = (root/review['software_checks_record']).resolve()
                    if not manifest_path.is_relative_to(root):
                        raise ValueError('software checks must belong to the workspace')
                    manifest = json.loads(manifest_path.read_text())
                    if manifest['pi_checks']['rc'] != 0:
                        raise ValueError('software checks failed')
                    for item in manifest['files']:
                        path = (root/item['path']).resolve()
                        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
                            raise ValueError('reviewed software changed')
                    round_number += 1
                    output = args.output/('round-'+str(round_number).zfill(6))
                    output.parent.mkdir(parents=True, exist_ok=True)
                else:
                    output = args.output
                if output.exists():
                    raise ValueError('hardware worker requires a new output folder')
                worker_command = list(command)
                worker_command[worker_command.index('--output')+1] = str(output)
                log = output.with_suffix('.worker.log').open('x')
                worker = subprocess.Popen(worker_command, env={**os.environ, 'PYTHONPATH': str(root/'vision/src')},
                                          stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                endpoint = worker_control_endpoint(output)
                deadline = time.monotonic()+5
                try:
                    while not endpoint.exists():
                        if worker.poll() is not None or time.monotonic()>deadline:
                            raise RuntimeError('hardware worker failed to start; inspect worker log')
                        time.sleep(.05)
                    options = dict(steering_available=not (args.motor_only or args.parking_protection or args.raised_load_probe),
                                          motor_available=not args.steering_only, parking_protection=args.parking_protection,
                                          ground_short_trial=args.ground_short, raised_load_probe=args.raised_load_probe,
                                          ground_held=args.ground_held)
                    control = (OwnedWorkerDrive(endpoint, process=worker, worker_log=log, output=output,
                                                expected_boot=args.expected_boot_id, **options)
                               if args.repeatable_tests else WorkerDrive(endpoint, **options))
                    if args.pi5_review:
                        control.last_status['reverse_available'] = json.loads(args.pi5_review.read_text()).get('reverse_physically_verified') is True
                    return control
                except BaseException:
                    if worker.poll() is None:
                        worker.terminate()
                        worker.wait(timeout=5)
                    raise
            if args.ground_held or args.repeatable_tests:
                review = json.loads(args.pi5_review.read_text())
                wrapper = RepeatableDrive if args.repeatable_tests else DeferredDrive
                drive = wrapper(start_worker, dict(ground_held_trial=args.ground_held,
                    ground_short_trial=args.ground_short, continuous_simulation=False,
                    speed_adjustable=False, speed_percent=100, steering_available=True, motor_available=True,
                    reverse_available=False, combined_trial=True, controls_paused=False,
                    steering_center_us=review.get('steering_center_us',1650),
                    steering_pulse_us=review.get('steering_center_us',1650), steering_signal_active=False,
                    steering_single_target=review.get('steering_single_target_verified') is True,
                    test_session_s=60 if args.repeatable_tests else review.get('manual_session_limit_s',180),
                    motion_hold_max_s=None if args.repeatable_tests else review.get('hold_limit_s',60),
                    lease_ms=200, release_required=False))
            else:
                drive = start_worker()
        elif args.controls_paused:
            state = json.loads((root/'vision/configs/bench-calibration.json').read_text())
            reason = state.get('latest_control_interruption', {}).get('display_reason')
            drive = PausedDrive(reason or '实车控制尚未启动，请保持电调关闭。')
        else:
            backend = SimulationBackend()
            state = json.loads((root/'vision/configs/bench-calibration.json').read_text())
            backend.real_output_paused_reason = ('曾出现未按键自行转动，停车保护复核前暂停实车输出。'
                if state.get('motor_stop_review', {}).get('unresolved_persistent_rotation') else '实车输出尚未启用。')
            drive = ManualDrive(backend, settling_s=1, session_s=None, continuous_simulation=True)
            drive.start()
        preview = SimpleNamespace(camera=cameras['primary'], aux_camera=cameras.get('secondary'),
            host='0.0.0.0', port=args.port, width=640, height=480,
            fps=args.camera_fps, preview_fps=args.preview_fps,
            raw_preview_fps=min(args.raw_preview_fps, args.camera_fps),
            secondary_raw_preview_fps=min(args.secondary_raw_preview_fps, args.camera_fps),
            preview_width=480, jpeg_quality=55, stale_ms=1500, camera_format='MJPG',
            weights=None, classic_candidates=args.classic_candidates,
            race_config=str(root/'vision/configs/race-2026.json'))
        serve(preview, load_config(None), drive=drive)
    finally:
        if drive:
            drive.close()
        if worker:
            try:
                worker.wait(timeout=3)
            except subprocess.TimeoutExpired:
                worker.terminate()
                worker.wait(timeout=3)
        if log:
            log.close()


if __name__ == '__main__':
    main()
