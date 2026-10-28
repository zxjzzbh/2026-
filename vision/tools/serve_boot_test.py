"""A boot waiting page with explicit ESC-off/on preparation and finite rounds."""
import argparse
import json
from pathlib import Path
import signal
from types import SimpleNamespace

from carvision.boot_test import BootTestDrive, PiBootPreparation
from carvision.config import load_config
from carvision.web_preview import serve
from carvision.manual_service import initial_status, repeated_drive
from carvision.worker_client import OwnedWorkerDrive


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
    preparation.start_monitor()
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
        host='0.0.0.0', port=args.port, width=640, height=480, fps=15, preview_fps=8,
        raw_preview_fps=15, secondary_raw_preview_fps=3, preview_width=480,
        jpeg_quality=55, stale_ms=1500, camera_format='MJPG', weights=None,
        classic_candidates=False, race_config=str(root/'vision/configs/race-2026.json'))
    try:
        serve(preview, load_config(None), drive=drive)
    finally:
        drive.close()
        preparation.close()


if __name__ == '__main__':
    main()
