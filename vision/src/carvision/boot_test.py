"""Boot into a waiting dashboard; fresh physical checks gate every first trial."""
import hashlib
import json
import os
from pathlib import Path
import secrets
import subprocess
import threading
import time

from .manual_drive import DriveError, ManualDrive
from .network_guard import interface_is_inactive


class BootTestDrive:
    def __init__(self, prepare, confirm, factory, initial_status, *, profiles=None):
        self.prepare, self.confirm, self.factory = prepare, confirm, factory
        self.initial_status = dict(initial_status)
        self.profiles = profiles or {'driving': dict(initial_status)}
        self.selected_mode = 'driving'
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.thread = self.delegate = self.context = None
        self.phase, self.mode = 'esc_off', 'disabled'
        self.reason = '请先确认电调关闭，再准备测试'
        self.setup_token = secrets.token_hex(16)
        self.owner = None
        self.closed = False

    def status(self):
        with self.lock:
            if self.delegate is not None:
                state = self.delegate.status()
            else:
                state = {**self.initial_status, 'mode': self.mode, 'reason': self.reason,
                         'hardware_output': False, 'worker_started': False,
                         'pending_hardware_start': False, 'motor_direction': 'stop',
                         'steering_direction': 'center', 'motor_pulse_us': None,
                         'session_remaining_s': None, 'ready_in_s': 0}
            return {**state, 'boot_test': True, 'boot_test_phase': self.phase,
                    'test_mode': self.selected_mode,
                    'setup_token': self.setup_token}

    def _advance(self, payload, first):
        candidate = None
        try:
            if first:
                context = self.prepare(self.cancelled)
                with self.lock:
                    if self.closed or self.cancelled.is_set():
                        return
                    self.context = context
                    steering = self.selected_mode == 'steering_pwm'
                    self.phase, self.mode = 'steering_ready' if steering else 'esc_on', 'disabled'
                    self.reason = ('联网和供电检查完成，电调保持关闭，确认架空后启用 S3' if steering
                                   else '零油门和联网检查完成，请打开电调并确认车轮静止')
                    self.setup_token = secrets.token_hex(16)
                    self.owner = None
            else:
                self.confirm(self.context)
                if self.cancelled.is_set():
                    return
                candidate = self.factory(self.context)
                with self.lock:
                    if self.closed or self.cancelled.is_set():
                        return
                    # Fresh setup cannot forward a held direction or old round token.
                    candidate.request('enable', {'client': payload['client'], 'bench_ready': True,
                                                 'trial_token': candidate.status().get('trial_token')})
                    self.delegate, candidate = candidate, None
                    self.phase = 'ready'
                    self.setup_token = secrets.token_hex(16)
        except Exception as error:
            with self.lock:
                if not self.closed:
                    self.phase, self.mode, self.reason = 'fault', 'fault', str(error)
        finally:
            if candidate is not None:
                candidate.close()

    def request(self, action, payload):
        if not isinstance(payload, dict):
            raise DriveError('JSON object required')
        with self.lock:
            if self.closed:
                raise DriveError('控制服务已停止')
            if action == 'status':
                return self.status()
            if action in ('stop', 'emergency'):
                if self.delegate is not None:
                    return {**self.delegate.request(action, payload), 'boot_test': True,
                            'boot_test_phase': self.phase, 'setup_token': self.setup_token,
                            'test_mode':self.selected_mode}
                if (action == 'stop' and self.phase == 'baseline'
                        and payload.get('stop_source') in ('window_blur', 'page_hidden')):
                    # Baseline writes only neutral and still needs a separate
                    # physical/static confirmation. No motion can be queued.
                    return self.status()
                if self.mode == 'starting' or action == 'emergency':
                    self.cancelled.set()
                    self.phase, self.mode, self.reason = 'fault', 'fault', '准备已取消，请关闭电调后检查'
                return self.status()
            if payload.get('setup_token') != self.setup_token:
                raise DriveError('准备步骤已更新，请重新确认当前页面')
            if action == 'select_mode':
                if self.delegate is not None or self.phase != 'esc_off' or self.mode != 'disabled':
                    raise DriveError('请先结束当前测试，再选择模式')
                if self.thread is not None and self.thread.is_alive():
                    raise DriveError('上一操作正在清理')
                mode = payload.get('test_mode')
                if not isinstance(mode, str) or mode not in self.profiles:
                    raise DriveError('不支持的测试模式')
                self.selected_mode = mode
                self.initial_status = dict(self.profiles[mode])
                self.owner = None
                self.setup_token = secrets.token_hex(16)
                return self.status()
            if action == 'finish_test':
                if self.phase != 'ready' or self.delegate is None:
                    raise DriveError('当前没有可结束的测试')
                self.delegate.request('emergency', {'stop_source': 'stop_button'})
                self.phase, self.mode, self.reason = 'closing', 'starting', '正在停车并结束当前测试'
                self.setup_token = secrets.token_hex(16)
                self.thread = threading.Thread(target=self._finish, name='boot-test-finish', daemon=True)
                self.thread.start()
                return self.status()
            if action == 'reset' and self.delegate is None and self.phase == 'fault':
                if self.thread is not None and self.thread.is_alive():
                    raise DriveError('准备进程正在清理，请保持电调关闭并稍后重试')
                if payload.get('bench_ready') is not True:
                    raise DriveError('请先关闭电调并勾选当前现场确认')
                self.cancelled = threading.Event()
                self.context = self.owner = None
                self.phase, self.mode, self.reason = 'esc_off', 'disabled', '请重新确认电调关闭，再准备测试'
                self.setup_token = secrets.token_hex(16)
                return self.status()
            if self.delegate is not None:
                if self.phase != 'ready':
                    raise DriveError('当前测试正在清理，方向请求不会排队')
                return {**self.delegate.request(action, payload), 'boot_test': True,
                        'boot_test_phase': self.phase, 'setup_token': self.setup_token,
                        'test_mode':self.selected_mode}
            if action != 'enable' or self.mode != 'disabled' or self.phase not in ('esc_off', 'esc_on', 'steering_ready'):
                raise DriveError('请先完成开机准备，方向请求不会排队')
            client = ManualDrive._client(self, payload.get('client'))
            if self.owner is not None and client != self.owner:
                raise DriveError('当前准备由另一页面使用')
            if payload.get('bench_ready') is not True:
                raise DriveError('请勾选当前现场确认')
            self.owner = client
            first = self.phase == 'esc_off'
            self.phase, self.mode = ('baseline' if first else 'confirming'), 'starting'
            self.reason = '正在保持零油门并检查 5G，请保持电调关闭' if first else '正在确认零油门，请保持车轮静止'
            self.thread = threading.Thread(target=self._advance, args=(dict(payload), first),
                                           name='boot-test-preparation', daemon=True)
            self.thread.start()
            return self.status()

    def _finish(self):
        try:
            self.delegate.close()
            with self.lock:
                self.delegate = self.context = self.owner = None
                self.cancelled = threading.Event()
                if not self.closed:
                    self.phase, self.mode = 'esc_off', 'disabled'
                    self.reason = '当前测试已结束，请关闭电调并选择下一种测试'
                    self.setup_token = secrets.token_hex(16)
        except Exception as error:
            with self.lock:
                self.phase, self.mode, self.reason = 'fault', 'fault', '清理失败，请关闭电调：'+str(error)

    def close(self):
        with self.lock:
            self.closed = True
            self.cancelled.set()
            delegate, thread = self.delegate, self.thread
        if thread is not None:
            thread.join(timeout=7)
        if delegate is not None:
            delegate.close()


def reviewed_manifest(root, template):
    path = (root/template['software_checks_record']).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('software checks must belong to this workspace')
    manifest = json.loads(path.read_text())
    if manifest['pi_checks']['rc'] != 0:
        raise ValueError('software checks failed')
    for item in manifest['files']:
        source = (root/item['path']).resolve()
        if not source.is_relative_to(root.resolve()) or hashlib.sha256(source.read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('已检查的软件发生变化，请先重新核验')
    return manifest


class PiBootPreparation:
    """Only the explicit ESC-off webpage action can write neutral; never at import/boot."""
    def __init__(self, root, template_path):
        self.root = root.resolve()
        self.template_path = template_path
        self.boot = self.current_boot()

    @staticmethod
    def current_boot():
        return Path('/proc/sys/kernel/random/boot_id').read_text().strip()

    def health(self):
        if self.current_boot() != self.boot:
            raise RuntimeError('树莓派开机状态已改变，请刷新后重新准备')
        flags = subprocess.check_output(['vcgencmd', 'get_throttled'], text=True, timeout=2).strip()
        temperature = subprocess.check_output(['vcgencmd', 'measure_temp'], text=True, timeout=2).strip()
        if flags != 'throttled=0x0' or float(temperature.split('=')[1].split("'")[0]) >= 70:
            raise RuntimeError('当前供电或温度检查未通过，请保持电调关闭')
        route = json.loads(subprocess.check_output(['ip', '-j', 'route', 'get', '1.1.1.1'], text=True, timeout=2))[0]
        if route.get('dev') != 'usb0' or Path('/sys/class/net/usb0/carrier').read_text().strip() != '1':
            raise RuntimeError('车辆当前没有通过 5G 模块上网，请检查模块连接')
        for name in ('eth0', 'wlan0'):
            if not interface_is_inactive(name):
                raise RuntimeError('本轮要核验纯 5G，请先拔掉车辆网线并断开车辆 Wi-Fi')
        return {'boot_id': self.boot, 'power_flags': 0, 'temperature': temperature, 'route': route}

    def prepare(self, cancelled, *, mode='driving'):
        import fcntl
        from .rasadapter5 import RasAdapter
        if mode not in ('driving', 'steering_pwm'):
            raise ValueError('invalid preparation mode')
        steering = mode == 'steering_pwm'
        template = json.loads(self.template_path.read_text())
        reviewed_manifest(self.root, template)
        model = Path('/proc/device-tree/model').read_bytes().rstrip(b'\0').decode()
        if model != 'Raspberry Pi 5 Model B Rev 1.0':
            raise RuntimeError('本入口仅用于已核验的树莓派 5 车辆')
        self.health()
        modem = json.loads(subprocess.check_output(['python3', str(self.root/'vision/tools/probe_5g_module.py')],
                                                  text=True, timeout=20))
        if modem['boot_id'] != self.boot or modem['modem']['nr5g_registration'] not in (1, 5):
            raise RuntimeError('5G 网络尚未注册，请保持电调关闭，稍后重新准备')
        if cancelled.is_set():
            raise RuntimeError('准备已取消')
        scope = 'run/boot-tests/'+self.boot[:8]+'-'+secrets.token_hex(6)
        folder = self.root/scope
        folder.mkdir(parents=True, mode=0o700, exist_ok=False)
        board = RasAdapter()
        with open('/tmp/carvision-pi-pwm.lock', 'a+') as ownership:
            fcntl.flock(ownership.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                if not steering:
                    board.open()
                if cancelled.is_set():
                    raise RuntimeError('准备已取消')
                if not steering:
                    board.set_esc(4, 1500)
                    time.sleep(.2)
                    if board.read_position(4) != 1500:
                        raise RuntimeError('零油门回读失败，请保持电调关闭')
                start = time.monotonic()
                samples = []
                for _ in range(30):
                    if cancelled.is_set():
                        raise RuntimeError('准备已取消')
                    samples.append(self.health())
                    time.sleep(1)
                if not steering and board.read_position(4) != 1500:
                    raise RuntimeError('零油门状态发生变化，请保持电调关闭')
            finally:
                if not steering:
                    board.close()
        baseline = {'boot_id': self.boot, 'duration_s': time.monotonic()-start, 'samples': samples,
                    'all_power_flags_zero': True, 'boot_unchanged': True,
                    'all_car_network_samples_cellular_only': True,
                    'esc_off_confirmed_by_user': True, 'confirmation_source': 'explicit webpage ESC-off checkbox and prepare request',
                    'forward_reverse_commands_sent': False, 'steering_gimbal_commands_sent': False}
        review = dict(template)
        for name in ('neutral_evidence_record', 'neutral_handoff_verified_monotonic_s', 'neutral_handoff_boot_id'):
            review.pop(name, None)
        review.update(boot_id=self.boot, baseline_record=scope+'/baseline.json', neutral_physically_verified=False,
                      neutral_handoff_operator_idle_confirmed=False, ground_short_requested_by_user=True,
                      wheels_on_ground_confirmed_by_user=True, clear_area_confirmed_by_user=True,
                      power_switch_in_reach_confirmed_by_user=True, esc_off_confirmed_by_user=True,
                      continuous_ground_driving_enabled=False, reverse_physically_verified=False,
                      steering_single_target_verified=False, current_boot_steering_stop_reverified=False)
        if template.get('ground_continuous_requested_by_user') is True:
            review.update(ground_short_requested_by_user=False, ground_held_requested_by_user=False,
                          ground_continuous_requested_by_user=True, continuous_ground_driving_enabled=True,
                          spotter_can_cut_power_confirmed_by_user=True, manual_session_limit_s=None,
                          hold_limit_s=None, reverse_revalidation_prepared=True,
                          current_boot_reverse_reverified=False, current_boot_keyboard_stop_reverified=False)
        if steering:
            review.update(steering_pwm_mode=True, ground_continuous_requested_by_user=False,
                          ground_short_requested_by_user=False, continuous_ground_driving_enabled=False,
                          wheels_raised_confirmed_by_user=True, wheels_on_ground_confirmed_by_user=False,
                          reverse_revalidation_prepared=False)
        for name, value in (('baseline.json', baseline), ('review.json', review), ('modem.json', modem)):
            (folder/name).write_text(json.dumps(value, indent=2)+'\n')
        return {'folder': str(folder), 'scope': scope, 'boot_id': self.boot, 'review': review, 'test_mode': mode}

    def confirm(self, context):
        import fcntl
        from .rasadapter5 import RasAdapter
        self.health()
        if context['boot_id'] != self.boot:
            raise RuntimeError('零油门准备记录来自旧开机状态')
        reviewed_manifest(self.root, context['review'])
        if context.get('test_mode') == 'steering_pwm':
            record = {'boot_id':self.boot, 'monotonic_s':time.monotonic(),
                      'esc_off_wheels_raised_hands_clear_confirmed_by_user':True,
                      'confirmation_source':'fresh webpage S3-only checkbox and enable request',
                      'actuator_commands_sent':False}
            (Path(context['folder'])/'steering-ready.json').write_text(json.dumps(record,indent=2)+'\n')
            return
        board = RasAdapter()
        with open('/tmp/carvision-pi-pwm.lock', 'a+') as ownership:
            fcntl.flock(ownership.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                board.open()
                if board.read_position(4) != 1500:
                    raise RuntimeError('零油门状态改变，请关闭电调后检查')
            finally:
                board.close()
        record = {'boot_id': self.boot, 'monotonic_s': time.monotonic(),
                  'esc_on_wheels_static_confirmed_by_user': True,
                  'confirmation_source': 'explicit current webpage ESC-on/static/hands-clear checkbox and enable request',
                  'actuator_commands_sent': False, 'neutral_physically_verified': True}
        folder = Path(context['folder'])
        (folder/'neutral-observation.json').write_text(json.dumps(record, indent=2)+'\n')
        context['review'].update(neutral_physically_verified=True,
                                 neutral_evidence_record=context['scope']+'/neutral-observation.json')
        (folder/'review.json').write_text(json.dumps(context['review'], indent=2)+'\n')
