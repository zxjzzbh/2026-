"""Desktop entry for the mutually exclusive race/debug vehicle services.

Connection secrets remain in the existing private connection file, never in
the desktop scripts or this source. Launching only selects an idle service and
opens its page; it never prepares, enables, or commands vehicle motion.
"""
import argparse
import json
from pathlib import Path
import shlex
import sys
import time
import webbrowser

REMOTE = '/home/pi/smartcar-race-20261002'
UNIT = 'smartcar-preview.service'
DROP_IN = '/etc/systemd/system/smartcar-preview.service.d/90-vehicle-mode.conf'


def command(client, args, *, password=None):
    line = shlex.join(args)
    if password is not None:
        line = "sudo -S -p '' " + line
    stdin, stdout, stderr = client.exec_command(line, timeout=40)
    if password is not None:
        stdin.write(password+'\n')
        stdin.flush()
    stdin.channel.shutdown_write()
    out, error = stdout.read().decode('utf-8', 'replace'), stderr.read().decode('utf-8', 'replace')
    if stdout.channel.recv_exit_status():
        raise RuntimeError(error.strip() or out.strip() or '车辆命令未完成')
    return out


def read_state(client):
    return json.loads(command(client, ['curl', '--fail', '--silent', '--max-time', '5',
                                      'http://127.0.0.1:8080/api/drive/status']))


def can_switch(state):
    if state.get('mode') != 'disabled' or state.get('boot_test_phase') != 'esc_off':
        return False
    if any(state.get(k) is not False for k in ('hardware_output', 'worker_started', 'pending_hardware_start')):
        return False
    if any((state.get(k) or {}).get('active') for k in
           ('race_flow', 'traffic_trial', 'crosswalk_trial', 'departure_trial', 'parking_trial')):
        return False
    cone = state.get('cone_trial') or {}
    return (not any((cone.get(k) or {}).get('active') for k in ('avoidance', 'short_test'))
            and not (state.get('parking_payment') or {}).get('motion_locked'))


def unit_content(mode):
    if mode not in ('race', 'debug'):
        raise ValueError('Unknown mode')
    return ('[Service]\nExecStart=\nExecStart='+REMOTE+'/.venv/bin/python '+REMOTE+
            '/vision/tools/run_'+mode+'.py --root '+REMOTE+' --live\n').encode('utf-8')


def select_mode(client, credentials, mode):
    state = read_state(client)
    if state.get('application_mode') == mode:
        return state  # Opening a second observer page must not restart driving.
    if not can_switch(state):
        raise RuntimeError('车辆正在准备、驾驶或执行任务。请先在原页面结束控制，再切换入口。')
    with client.open_sftp() as sftp:
        try:
            with sftp.open(DROP_IN, 'rb') as stream:
                previous = stream.read()
        except FileNotFoundError:
            previous = None
        temporary = REMOTE+'/run/desktop-mode-'+str(time.time_ns())+'.conf'
        with sftp.open(temporary, 'wb') as stream:
            stream.write(unit_content(mode))
        if not can_switch(read_state(client)):
            sftp.remove(temporary)
            raise RuntimeError('车辆状态已变化，本次没有切换。')
        stopped = False
        attempted_install = False
        try:
            command(client, ['systemctl', 'stop', UNIT], password=credentials['password'])
            stopped = True
            attempted_install = True
            command(client, ['install', '-m', '644', temporary, DROP_IN], password=credentials['password'])
            command(client, ['systemctl', 'daemon-reload'], password=credentials['password'])
            command(client, ['systemctl', 'start', UNIT], password=credentials['password'])
            deadline = time.monotonic()+30
            while time.monotonic() < deadline:
                try:
                    state = read_state(client)
                    if state.get('application_mode') == mode and can_switch(state):
                        return state
                except (OSError, ValueError, RuntimeError):
                    pass
                time.sleep(.4)
            raise RuntimeError('新入口未能正常启动，正在恢复原入口。')
        except Exception as launch_error:
            if stopped:
                try:
                    command(client, ['systemctl', 'stop', UNIT], password=credentials['password'])
                    # install can create/truncate its target before failing.
                    # Restore even when that first install never returned OK.
                    if attempted_install:
                        if previous is None:
                            command(client, ['rm', '-f', '--', DROP_IN], password=credentials['password'])
                        else:
                            with sftp.open(temporary, 'wb') as stream:
                                stream.write(previous)
                            command(client, ['install', '-m', '644', temporary, DROP_IN], password=credentials['password'])
                    command(client, ['systemctl', 'daemon-reload'], password=credentials['password'])
                    command(client, ['systemctl', 'start', UNIT], password=credentials['password'])
                except Exception as rollback_error:
                    raise RuntimeError('切换失败且原入口恢复失败：'+str(rollback_error)) from launch_error
            raise
        finally:
            try:sftp.remove(temporary)
            except FileNotFoundError:pass


def connect(settings):
    sys.path.insert(0, settings['ssh_dependency_directory'])
    import paramiko
    credentials = json.loads(Path(settings['connection_file']).read_text(encoding='utf-8-sig'))['ssh']
    client = paramiko.SSHClient()
    client.load_host_keys(settings['known_hosts'])
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect(credentials['host'], port=credentials.get('port', 22), username=credentials['username'],
                   password=credentials['password'], look_for_keys=False, allow_agent=False,
                   timeout=8, banner_timeout=8, auth_timeout=8)
    return client, credentials


def main(argv=None):
    parser = argparse.ArgumentParser(description='智能车比赛 / 调试桌面入口')
    parser.add_argument('--mode', choices=('race', 'debug'), required=True)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('launch-config.json'))
    parser.add_argument('--check', action='store_true', help='只读连接检查，不切换服务、不打开浏览器')
    args = parser.parse_args(argv)
    settings = json.loads(args.config.read_text(encoding='utf-8-sig'))
    print('正在连接树莓派……', flush=True)
    client, credentials = connect(settings)
    try:
        state = read_state(client) if args.check else select_mode(client, credentials, args.mode)
        if args.check:
            print(json.dumps({'connected': True, 'application_mode': state.get('application_mode'),
                              'idle': can_switch(state), 'motion_commands_sent': False}, ensure_ascii=False))
            return 0
        url = 'http://'+credentials['host']+':8080/'
        print(('比赛' if args.mode == 'race' else '调试')+'入口已就绪：'+url, flush=True)
        webbrowser.open(url, new=2)
        return 0
    finally:
        client.close()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print('启动未完成：'+str(exc), file=sys.stderr)
        raise SystemExit(1)
