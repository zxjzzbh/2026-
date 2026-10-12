"""Deploy the real F/B parking console, with backup, checks and rollback.

Never starts a trial. Connection credentials stay in the existing handoff file.
"""
import argparse
import ast
import hashlib
import json
import shlex
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import paramiko

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'vision/src'))
from carvision.brake_controls import extend_controls

REMOTE = '/home/pi/smartcar-race-20261002'
PYTHON = REMOTE+'/.venv/bin/python'
UNIT = '/etc/systemd/system/smartcar-preview.service.d/50-crosswalk-trial.conf'
FILES = [
    'vision/src/carvision/brake_parking.py', 'vision/src/carvision/brake_output.py',
    'vision/src/carvision/brake_runtime.py', 'vision/src/carvision/brake_trial.py',
    'vision/src/carvision/brake_controls.py', 'vision/configs/brake-parking.json',
    'vision/tools/brake_parking.py', 'vision/tools/serve_crosswalk_trial.py',
    'vision/tests/test_brake_parking_program.py', 'vision/tests/test_brake_trial.py',
    'vision/src/carvision/parking_speech.py', 'vision/tools/speak_parking.py',
    'vision/tests/test_parking_speech.py',
]


def command(client, args, *, password=None, timeout=40):
    line = shlex.join(args)
    if password is not None:
        line = 'sudo -S -p '+shlex.quote('')+' '+line
    stdin, stdout, stderr = client.exec_command(line, timeout=timeout)
    if password is not None:
        stdin.write(password+'\n'); stdin.flush(); stdin.channel.shutdown_write()
    out, err = stdout.read().decode('utf-8', 'replace'), stderr.read().decode('utf-8', 'replace')
    if stdout.channel.recv_exit_status():
        raise RuntimeError(out+err)
    return out


def put_atomic(sftp, path, data):
    temporary = path+'.brake-update.tmp'
    with sftp.open(temporary, 'wb') as f:
        f.write(data)
    sftp.posix_rename(temporary, path)


def read(sftp, path):
    with sftp.open(path, 'rb') as f:
        return f.read()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--handoff', type=Path, required=True)
    parser.add_argument('--esc-off-confirmed', action='store_true', required=True)
    parser.add_argument('--esc-mode', choices=['F/B'], required=True)
    args = parser.parse_args()
    cfg = json.loads((args.handoff/'connection.json').read_text(encoding='utf-8-sig'))['ssh']
    record = ROOT/'run'/('brake-console-deploy-'+time.strftime('%Y%m%d-%H%M%S'))
    record.mkdir(parents=True, exist_ok=False)
    audit = {'deployed': False, 'motion_commands_sent': False, 'esc_off_declared': True,
             'esc_mode_declared': args.esc_mode, 'active_brake_verified': False}
    client = paramiko.SSHClient()
    client.load_host_keys(str(args.handoff/'attachments/known_hosts.smartcar'))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    changed = []
    unit_changed = False
    sftp = None
    try:
        client.connect(cfg['host'], port=cfg.get('port', 22), username=cfg['username'],
                       password=cfg['password'], look_for_keys=False, allow_agent=False, timeout=5)
        sftp = client.open_sftp()
        # Inspect the exact live script without executing its imports or code.
        source = read(sftp, REMOTE+'/vision/src/carvision/manual_controls.py')
        (record/'manual_controls.before.py').write_bytes(source)
        tree = ast.parse(source.decode('utf-8'))
        node = next(n.value for n in tree.body if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'SCRIPT' for t in n.targets))
        while isinstance(node, ast.Call):
            node = node.func.value
        controls = SimpleNamespace(SCRIPT=ast.literal_eval(node), HTML='')
        extend_controls(controls)  # Version-sensitive injection must match.
        unit_before = read(sftp, UNIT)
        (record/'service.before.conf').write_bytes(unit_before)
        before = command(client, ['curl', '--fail', '--silent', '--max-time', '3',
                                  'http://127.0.0.1:8080/api/drive/status'])
        status = json.loads(before)
        audit['before'] = {k: status.get(k) for k in ('mode', 'hardware_output', 'boot_test_phase')}
        # Backup both locally and on the Pi before replacing any program file.
        remote_backup = REMOTE+'/run/'+record.name
        command(client, ['mkdir', '-p', remote_backup])
        backup = {}
        for rel in FILES:
            try:
                data = read(sftp, REMOTE+'/'+rel)
            except FileNotFoundError:
                data = None
            backup[rel] = data
            if data is not None:
                dst = record/'before'/rel; dst.parent.mkdir(parents=True, exist_ok=True); dst.write_bytes(data)
                command(client, ['mkdir', '-p', remote_backup+'/before/'+str(Path(rel).parent).replace('\\', '/')])
                put_atomic(sftp, remote_backup+'/before/'+rel, data)
        put_atomic(sftp, remote_backup+'/service.before.conf', unit_before)
        for rel in FILES:
            put_atomic(sftp, REMOTE+'/'+rel, (ROOT/rel).read_bytes())
            changed.append(rel)
        tests = command(client, ['env', 'PYTHONPATH='+REMOTE+'/vision/src', PYTHON, '-m', 'pytest',
                                REMOTE+'/vision/tests/test_brake_parking_program.py',
                                REMOTE+'/vision/tests/test_brake_trial.py',
                                REMOTE+'/vision/tests/test_parking_speech.py', '-q'], timeout=60)
        audit['pi_software_tests'] = tests
        new_unit = ('[Service]\nExecStart=\nEnvironment=PYTHONPATH='+REMOTE+'/vision/src\n'
                    'ExecStart='+PYTHON+' '+REMOTE+'/vision/tools/serve_crosswalk_trial.py --root '+REMOTE+
                    ' --brake-config '+REMOTE+'/vision/configs/brake-parking.json\n')
        staging = remote_backup+'/service.new.conf'
        put_atomic(sftp, staging, new_unit.encode())
        unit_changed = True
        command(client, ['install', '-m', '644', staging, UNIT], password=cfg['password'])
        command(client, ['systemctl', 'daemon-reload'], password=cfg['password'])
        command(client, ['systemctl', 'restart', 'smartcar-preview.service'], password=cfg['password'])
        deadline = time.monotonic()+20
        while True:
            try:
                raw = command(client, ['curl', '--fail', '--silent', '--max-time', '2',
                                       'http://127.0.0.1:8080/api/drive/status'])
                status = json.loads(raw)
                if status.get('crosswalk_trial', {}).get('program') != 'fb-brake-v1':
                    raise RuntimeError('new console not active yet')
                break
            except Exception:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.5)
        if (status.get('hardware_output') or status.get('worker_started')
                or status['crosswalk_trial']['active'] or status.get('reverse_available') is not False):
            raise RuntimeError('post-deployment waiting state failed')
        audit.update(deployed=True, backup=remote_backup,
                     after={k: status.get(k) for k in ('mode', 'hardware_output', 'reverse_available', 'esc_mode', 'boot_test_phase')},
                     sha256={rel: hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() for rel in FILES})
        print(json.dumps(audit, ensure_ascii=False, indent=2))
    except Exception as exc:
        audit['error'] = f'{type(exc).__name__}: {exc}'
        if changed:
            try:
                for rel in reversed(changed):
                    if backup[rel] is None:
                        sftp.remove(REMOTE+'/'+rel)
                    else:
                        put_atomic(sftp, REMOTE+'/'+rel, backup[rel])
                if unit_changed:
                    command(client, ['install', '-m', '644', remote_backup+'/service.before.conf', UNIT], password=cfg['password'])
                    command(client, ['systemctl', 'daemon-reload'], password=cfg['password'])
                    command(client, ['systemctl', 'restart', 'smartcar-preview.service'], password=cfg['password'])
                audit['rolled_back'] = True
            except Exception as rollback:
                audit['rollback_error'] = str(rollback)
        print(json.dumps(audit, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1
    finally:
        client.close()
        (record/'deployment.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
