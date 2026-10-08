"""Start/stop the Pi preview without a desktop or administrator privileges.

Bundle layout: <root>/.venv and <root>/vision. Not an auto-start service.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from carvision.cameras import camera_selector


def matching_process(pid, python):
    try:
        argv = (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
        return argv[:4] == [str(python).encode(), b'-m', b'carvision', b'serve']
    except FileNotFoundError:
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['start', 'stop', 'status'])
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--camera', type=camera_selector)
    parser.add_argument('--aux-camera', type=camera_selector)
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--classic-candidates', action='store_true', help='show experimental race-element cues')
    args = parser.parse_args()
    if os.name != 'posix' or not Path('/proc').exists():
        raise RuntimeError('This launcher is for the Linux target only')
    root = args.root.resolve()
    camera_config = root/'vision/configs/cameras.json'
    selected = json.loads(camera_config.read_text()) if camera_config.exists() else {'primary': 0, 'secondary': None}
    primary = args.camera if args.camera is not None else camera_selector(selected['primary'])
    secondary = args.aux_camera if args.aux_camera is not None else selected.get('secondary')
    python = root/'.venv/bin/python'
    runtime = root/'run'
    runtime.mkdir(exist_ok=True)
    pid_file = runtime/'preview.pid'
    pid = int(pid_file.read_text().strip()) if pid_file.exists() else None
    running = pid is not None and matching_process(pid, python)
    if args.action == 'status':
        print(json.dumps({'running': running, 'pid': pid if running else None,
                          'log': str(runtime/'preview.log')}))
        return
    if args.action == 'stop':
        if running:
            os.kill(pid, signal.SIGINT)
            for _ in range(50):
                if not matching_process(pid, python):
                    break
                time.sleep(.1)
            if matching_process(pid, python):
                raise RuntimeError('Preview did not stop; inspect log instead of killing other processes')
        pid_file.unlink(missing_ok=True)
        print('Preview stopped')
        return
    if running:
        print(f'Preview already running: PID {pid}')
        return
    if not python.is_file():
        raise RuntimeError(f'Missing environment: {python}')
    command = [str(python), '-m', 'carvision', 'serve', '--camera', str(primary),
               '--host', '0.0.0.0', '--port', str(args.port), '--width', '640', '--height', '480',
               '--fps', '30', '--preview-fps', '10', '--preview-width', '640']
    if secondary is not None:
        command += ['--aux-camera', str(camera_selector(secondary))]
    race_config = root/'vision/configs/race-2026.json'
    if race_config.is_file():
        command += ['--race-config', str(race_config)]
    if args.classic_candidates:
        command += ['--classic-candidates']
    with (runtime/'preview.log').open('ab') as log:
        process = subprocess.Popen(command, cwd=root/'vision', stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                   close_fds=True)
    time.sleep(1)
    if process.poll() is not None:
        raise RuntimeError(f'Preview exited with {process.returncode}; inspect {runtime / "preview.log"}')
    pid_file.write_text(str(process.pid)+'\n')
    print(json.dumps({'started': True, 'pid': process.pid, 'port': args.port,
                      'log': str(runtime/'preview.log'), 'auto_start_on_boot': False}))


if __name__ == '__main__':
    main()
