"""Update the idle car console, preserving manual control and parking files.

No prepare/start/drive HTTP calls. Refuses active or pending control sessions;
restarted service must expose an inert, unprepared traffic stage. All changed
files and the old systemd override are backed up before mutation.
"""
import argparse
import ast
import hashlib
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import paramiko

from deploy_brake_trial import command, put_atomic, read, REMOTE, PYTHON, UNIT, ROOT

sys.path.insert(0, str(ROOT/'vision/src'))
from carvision.brake_controls import extend_controls as extend_brake
from carvision.traffic_controls import extend_controls as extend_traffic

FILES = ['vision/src/carvision/'+n+'.py' for n in ('traffic_driving', 'traffic_trial', 'traffic_controls', 'traffic_signal', 'traffic_voting', 'brake_output')]+[
    'vision/configs/traffic-driving.json', 'vision/configs/traffic-signal.json',
    'vision/tools/serve_crosswalk_trial.py', 'vision/tests/test_traffic_driving.py',
    'vision/tests/test_traffic_trial.py', 'vision/tests/test_traffic_signal.py', 'vision/tests/test_traffic_voting.py', 'vision/tests/test_brake_parking_program.py']
PROTECTED = ['vision/src/carvision/'+n+'.py' for n in ('manual_controls', 'web_preview', 'brake_trial', 'brake_controls',
             'brake_parking', 'parking_speech', 'console_layout')]+[
    'vision/tools/serve_boot_test.py', 'vision/tools/speak_parking.py', 'vision/configs/brake-parking.json',
    'vision/configs/cameras.json', 'vision/configs/brake-parking.local.json', 'vision/configs/traffic-driving.local.json']


def idle(status):
    return (status.get('mode') == 'disabled' and status.get('boot_test_phase') == 'esc_off'
            and all(not status.get(k) for k in ('hardware_output', 'worker_started', 'pending_hardware_start'))
            and all(not (status.get(k) or {}).get('active') for k in ('crosswalk_trial', 'traffic_trial')))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--handoff', type=Path, required=True)
    args = parser.parse_args()
    cfg = json.loads((args.handoff/'connection.json').read_text(encoding='utf-8-sig'))['ssh']
    record = ROOT/'run'/('traffic-console-deploy-'+time.strftime('%Y%m%d-%H%M%S'))
    record.mkdir(parents=True, exist_ok=False)
    audit = {'deployed': False, 'motion_commands_sent': False, 'green_action': 'speech_only'}
    c = paramiko.SSHClient()
    c.load_host_keys(str(args.handoff/'attachments/known_hosts.smartcar'))
    c.set_missing_host_key_policy(paramiko.RejectPolicy())
    changed, unit_changed, restarted, backup = [], False, False, {}
    def status():
        return json.loads(command(c, ['curl', '--fail', '--silent', '--max-time', '3', 'http://127.0.0.1:8080/api/drive/status']))
    try:
        c.connect(cfg['host'], port=cfg.get('port', 22), username=cfg['username'], password=cfg['password'],
                  look_for_keys=False, allow_agent=False, timeout=6)
        sftp = c.open_sftp()
        if not idle(status()):
            raise RuntimeError('Control is active or pending. No files replaced and no service restart.')
        protected = {}
        for rel in PROTECTED:
            try: b = read(sftp, REMOTE+'/'+rel)
            except FileNotFoundError: b = None
            protected[rel] = b
            if b is not None:
                dst = record/'protected'/rel; dst.parent.mkdir(parents=True, exist_ok=True); dst.write_bytes(b)
        tree = ast.parse(protected['vision/src/carvision/manual_controls.py'].decode('utf-8'))
        node = next(n.value for n in tree.body if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'SCRIPT' for t in n.targets))
        while isinstance(node, ast.Call): node = node.func.value
        controls = SimpleNamespace(SCRIPT=ast.literal_eval(node), HTML='')
        extend_brake(controls); extend_traffic(controls)
        (record/'extended-controls.js').write_text(controls.SCRIPT.removeprefix('<script>').removesuffix('</script>'), encoding='utf-8')
        unit_before = read(sftp, UNIT)
        (record/'service.before.conf').write_bytes(unit_before)
        remote_backup = REMOTE+'/run/'+record.name
        command(c, ['mkdir', '-p', remote_backup])
        put_atomic(sftp, remote_backup+'/service.before.conf', unit_before)
        for rel in FILES:
            try: before = read(sftp, REMOTE+'/'+rel)
            except FileNotFoundError: before = None
            backup[rel] = before
            if before is not None:
                dst = record/'before'/rel; dst.parent.mkdir(parents=True, exist_ok=True); dst.write_bytes(before)
                command(c, ['mkdir', '-p', remote_backup+'/before/'+str(Path(rel).parent).replace('\\', '/')])
                put_atomic(sftp, remote_backup+'/before/'+rel, before)
        for rel in FILES:
            put_atomic(sftp, REMOTE+'/'+rel, (ROOT/rel).read_bytes()); changed.append(rel)
        tests = ['test_traffic_driving.py', 'test_traffic_trial.py', 'test_traffic_signal.py', 'test_traffic_voting.py',
                 'test_brake_trial.py', 'test_brake_parking_program.py', 'test_parking_speech.py']
        audit['pi_software_tests'] = command(c, ['env', 'PYTHONPATH='+REMOTE+'/vision/src', PYTHON, '-m', 'pytest',
                                        *[REMOTE+'/vision/tests/'+x for x in tests], '-q'], timeout=60)
        if not idle(status()):
            raise RuntimeError('Control became active during validation; rollback files without restart.')
        unit = ('[Service]\nExecStart=\nEnvironment=PYTHONPATH='+REMOTE+'/vision/src\nExecStart='+PYTHON+' '+REMOTE+
                '/vision/tools/serve_crosswalk_trial.py --root '+REMOTE+' --brake-config '+REMOTE+
                '/vision/configs/brake-parking.json --traffic-config '+REMOTE+'/vision/configs/traffic-driving.json\n')
        staging = remote_backup+'/service.new.conf'
        put_atomic(sftp, staging, unit.encode())
        # Re-check at the last possible moment; no updater sends a drive command.
        if not idle(status()):
            raise RuntimeError('Control is no longer idle; no restart performed.')
        unit_changed = True
        command(c, ['install', '-m', '644', staging, UNIT], password=cfg['password'])
        command(c, ['systemctl', 'daemon-reload'], password=cfg['password'])
        restarted = True
        command(c, ['systemctl', 'restart', 'smartcar-preview.service'], password=cfg['password'])
        deadline = time.monotonic()+20
        while True:
            try:
                after = status()
                if (after.get('traffic_trial') or {}).get('program') != 'traffic-fb-v1':
                    raise RuntimeError('Traffic program has not started')
                break
            except Exception:
                if time.monotonic() > deadline: raise
                time.sleep(.5)
        if not idle(after) or after['traffic_trial'].get('green_action') != 'speech_only':
            raise RuntimeError('New console did not remain inert')
        for rel, b in protected.items():
            try: actual = read(sftp, REMOTE+'/'+rel)
            except FileNotFoundError: actual = None
            if actual != b:
                raise RuntimeError('Unrelated car file changed: '+rel)
        audit.update(deployed=True, backup=remote_backup, protected_files_unchanged=True,
                     after={k: after.get(k) for k in ('mode', 'hardware_output', 'worker_started', 'boot_test_phase')},
                     traffic_phase=after['traffic_trial']['phase'],
                     sha256={rel: hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() for rel in FILES})
    except Exception as exc:
        audit['error'] = str(exc)
        if changed:
            try:
                for rel in reversed(changed):
                    if backup[rel] is None: sftp.remove(REMOTE+'/'+rel)
                    else: put_atomic(sftp, REMOTE+'/'+rel, backup[rel])
                if unit_changed:
                    command(c, ['install', '-m', '644', remote_backup+'/service.before.conf', UNIT], password=cfg['password'])
                    command(c, ['systemctl', 'daemon-reload'], password=cfg['password'])
                if restarted:
                    command(c, ['systemctl', 'restart', 'smartcar-preview.service'], password=cfg['password'])
                audit['rolled_back'] = True
            except Exception as exc2:
                audit['rollback_error'] = str(exc2)
    finally:
        c.close()
        (record/'deployment.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(audit, ensure_ascii=False, indent=2))
    return 0 if audit['deployed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
