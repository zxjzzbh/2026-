"""One operator, one UART owner, one autonomous C++ process."""
import json
import hashlib
from pathlib import Path
import subprocess
import tempfile
import threading
import time

from .transport import Hardware, packet
from .preview import Preview


def public_session(secret):
    """Public ownership label must never be reusable as the authentication cookie."""
    return hashlib.sha256(secret.encode('utf-8')).hexdigest() if secret else None


class Supervisor:
    TASKS = frozenset(('race', 'zebra', 'traffic', 'obstacle', 'parking', 'observe'))
    MANUAL_LEASE_S = 1.5
    TEST_NEUTRAL_S = 3.0

    def __init__(self, root, *, hardware_factory=Hardware, preview=None,
                 popen=subprocess.Popen, clock=time.monotonic, background=True):
        self.root = Path(root).resolve()
        self.config = self.root / 'cpp/config/race.json'
        self.hardware_factory, self.popen, self.clock = hardware_factory, popen, clock
        self.preview = preview or Preview(self.config)
        self.lock = threading.RLock()
        self.mode, self.reason, self.owner = 'idle', '待准备；执行器未打开', None
        self.hardware = self.process = self.process_log = None
        self.process_drive = False
        self.run_dir = None
        self.task = None
        self.sequence = -1
        self.generation = 0
        self.requested = (1610, 1500)
        self.output = (1610, 1500)
        self.output_confirmed = False
        self.last_command = self.clock()
        self.brake_until = 0
        self.test_ready_at = None
        self.closed = False
        self.stop_event = threading.Event()
        self.worker = None
        if background:
            self.preview.start()
            self.worker = threading.Thread(target=self._loop, name='5g-supervisor', daemon=True)
            self.worker.start()

    def _write(self, steering, throttle):
        self.output_confirmed = False
        self.hardware.write(steering, throttle)
        self.output = (steering, throttle)
        self.output_confirmed = True

    def _open(self):
        self.output_confirmed = False
        options = json.loads(self.config.read_text(encoding='utf-8-sig'))
        self.hardware = self.hardware_factory(options.get('uart', '/dev/ttyAMA0'))
        try:
            self.hardware.open()
            self._write(1610, 1500)
        except BaseException:
            self.hardware.close()
            self.hardware = None
            raise

    def _release(self):
        self.requested = (1610, 1500)
        self.brake_until = 0
        if self.hardware is not None:
            try:
                self._write(1610, 1500)
            finally:
                self.hardware.close()
                self.hardware = None

    def _neutral_after_process(self):
        if self.process_drive:
            self._open()
            self._release()

    def _finish_process(self, code):
        self.process = None
        if self.process_log:
            self.process_log.close()
            self.process_log = None
        try:
            self._neutral_after_process()
            self.mode = 'idle' if code == 0 else 'fault'
            self.reason = '任务已结束，执行器已释放' if code == 0 else '任务进程退出，请检查运行记录（退出码 %s）' % code
        finally:
            self.process_drive = False
            self.owner = None
            self.preview.start()

    def _stop(self):
        self.generation += 1
        self.test_ready_at = None
        self.mode = 'stopping'
        self.requested = (1610, 1500)
        if self.process is not None:
            process = self.process
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            self._finish_process(process.returncode)
        self._release()
        self.mode, self.reason, self.owner = 'idle', '已停止，执行器已释放；重新准备后才能遥控', None
        self.preview.start()

    def _start(self, session, data):
        task = data.get('task')
        if task not in self.TASKS:
            raise ValueError('未知任务')
        drive = task != 'observe'
        if self.process is not None:
            raise ValueError('已有任务运行，请先停止')
        if drive:
            if self.mode not in ('prepared', 'manual') or self.hardware is None:
                raise ValueError('先在电调关闭时准备，再确认电调打开且车辆停稳')
            if data.get('supervised') is not True or data.get('stationary') is not True:
                raise ValueError('需要现场监管及停稳确认')
            if task == 'race' and data.get('handoff_complete') is not True:
                raise ValueError('需要确认遥控切换区交接完成')
            if self.output[1] != 1500 or self.requested[1] != 1500 or self.brake_until > self.clock():
                raise ValueError('先松开遥控并停稳，再启动自主任务')
        self._launch(session, task)

    def _start_test(self, session, data):
        task = data.get('task')
        if not isinstance(task, str) or task not in self.TASKS:
            raise ValueError('未知测试项目')
        if self.mode not in ('idle', 'prepared', 'fault') or self.process is not None:
            raise ValueError('请先结束当前操作，再开始测试')
        if task == 'race' and data.get('handoff_complete') is not True:
            raise ValueError('全流程测试须在切换区完成交接后开始')
        if task == 'observe':
            self._launch(session, task)
            return
        if not (self.root / 'cpp/build/smartcar').is_file():
            raise RuntimeError('C++ 主程序尚未编译完成')
        self.generation += 1
        self.owner, self.task, self.run_dir = session, task, None
        self.mode = 'test_preparing'
        self.reason = f'正在输出中位，约 {self.TEST_NEUTRAL_S:g} 秒后自动开始测试'
        self.requested = (1610, 1500)
        if self.hardware is None:
            self._open()
        else:
            self._write(*self.requested)
        self.preview.stop()
        self.last_command = self.clock()
        self.test_ready_at = self.last_command + self.TEST_NEUTRAL_S

    def _launch(self, session, task, *, preview_stopped=False):
        drive = task != 'observe'
        executable = self.root / 'cpp/build/smartcar'
        if not executable.is_file():
            raise RuntimeError('C++ 主程序尚未编译完成')
        # Neutral and invalidate manual inputs before any slow camera operation.
        self.mode = 'starting'
        self.generation += 1
        self._release()
        if not preview_stopped:
            self.preview.stop()
        self.test_ready_at = None
        runs = self.root / 'run'
        runs.mkdir(parents=True, exist_ok=True)
        self.run_dir = Path(tempfile.mkdtemp(prefix=f'cpp-{time.time_ns()}-', dir=runs))
        command = [str(executable), '--task', task, '--config', str(self.config),
                   '--output', str(self.run_dir)]
        if drive:
            command += ['--drive', '--supervised']
            if task == 'race':
                command += ['--handoff-complete']
        self.process_log = (self.run_dir / 'process.log').open('wb')
        try:
            self.process = self.popen(command, cwd=str(self.root / 'cpp'),
                                      stdin=subprocess.DEVNULL, stdout=self.process_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            self.process_log.close()
            self.process_log = None
            self.preview.start()
            raise
        self.process_drive = drive
        if drive:
            self.output_confirmed = False
        self.owner, self.task = session, task
        self.mode = 'autonomous' if drive else 'observing'
        self.reason = 'C++ 车端自主运行；网页心跳不控制此任务' if drive else '仅观察识别；未打开执行器'

    def action(self, session, data):
        if not isinstance(data, dict):
            raise ValueError('请求必须为对象')
        action = data.get('action')
        with self.lock:
            if self.closed:
                raise RuntimeError('控制服务已关闭')
            # Any authenticated same-origin operator may explicitly stop.
            # This grants no driving rights and recovers a lost browser session.
            if action != 'stop' and self.owner is not None and self.owner != session:
                raise PermissionError('其他页面持有控制权，请在原页面停止后再操作')
            if action in ('enable', 'command', 'brake', 'start', 'start_test'):
                generation = data.get('generation')
                if type(generation) is not int or generation != self.generation:
                    raise ValueError('控制轮次已改变，拒绝旧指令；请等待页面读取当前状态')
            if self.mode == 'test_preparing' and action != 'stop':
                raise ValueError('测试正在准备中，可以点击结束测试取消')
            try:
                if action == 'prepare':
                    if self.mode not in ('idle', 'fault') or self.process is not None:
                        raise ValueError('当前已准备或已有任务运行')
                    if data.get('esc_off') is not True or data.get('supervised') is not True:
                        raise ValueError('准备需要确认电调关闭、有人现场监管')
                    self._open()
                    self.mode, self.owner, self.reason = 'prepared', session, '已输出中位；打开电调并确认静止后启用遥控'
                    self.sequence = -1
                    self.generation += 1
                    self.last_command = self.clock()
                elif action == 'enable':
                    if self.mode != 'prepared' or data.get('esc_on') is not True or data.get('stationary') is not True:
                        raise ValueError('先准备，并确认电调已打开、车辆保持静止')
                    self.mode, self.reason = 'manual', '5G 遥控已启用；松开按钮回中位'
                    self.generation += 1
                    self.last_command = self.clock()
                elif action == 'command':
                    if self.mode != 'manual':
                        raise ValueError('当前不是遥控状态；自主运行拒绝网页行驶指令')
                    if self.clock() - self.last_command > self.MANUAL_LEASE_S:
                        self.mode = 'prepared'
                        self.requested = (1610, 1500)
                        self._write(1610, 1500)
                        self.reason = '遥控指令已过期，已回中位；请确认静止后重新启用遥控'
                        raise ValueError(self.reason)
                    steering, throttle = data.get('steering_us'), data.get('throttle_us')
                    packet(steering, throttle)
                    if throttle == 1400:
                        raise ValueError('制动请使用独立制动按钮')
                    sequence = data.get('sequence')
                    if type(sequence) is not int or sequence <= self.sequence:
                        raise ValueError('过期或重复遥控指令')
                    self.sequence = sequence
                    self.last_command = self.clock()
                    # In-flight neutral/forward cannot shorten an accepted brake.
                    self.requested = (steering, throttle) if self.brake_until == 0 else (1610, 1500)
                    if throttle == 1500 and self.brake_until == 0:
                        # Acknowledge neutral only after UART write, so the next
                        # start request cannot race the periodic output loop.
                        self._write(steering, 1500)
                elif action == 'brake':
                    if self.mode != 'manual':
                        raise ValueError('制动按钮仅用于遥控；自主时请使用停止任务')
                    if self.brake_until == 0:
                        self.requested = (1610, 1500)
                        self._write(1610, 1400)
                        self.brake_until = self.clock() + .35
                elif action == 'start':
                    self._start(session, data)
                elif action == 'start_test':
                    self._start_test(session, data)
                elif action == 'stop':
                    self._stop()
                else:
                    raise ValueError('未知操作')
            except (ValueError, PermissionError) as exc:
                if action == 'start_test' and self.mode in ('test_preparing', 'starting'):
                    self._fault(exc)
                raise
            except Exception as exc:
                self._fault(exc)
                raise
        return self.status()

    def _fault(self, exc):
        failed_start = self.mode in ('test_preparing', 'starting')
        self.mode, self.reason = 'fault', str(exc)
        try:
            self._release()
        except Exception as release_error:
            self.reason += '；中位输出未确认：' + str(release_error)
        if failed_start:
            self.owner = self.test_ready_at = None
            self.preview.start()

    def tick(self):
        with self.lock:
            if self.mode == 'test_preparing':
                try:
                    now = self.clock()
                    if now - self.last_command > self.MANUAL_LEASE_S:
                        self.test_ready_at = now + self.TEST_NEUTRAL_S
                    self._write(1610, 1500)
                    self.last_command = now
                    if now >= self.test_ready_at:
                        self._launch(self.owner, self.task, preview_stopped=True)
                except Exception as exc:
                    self._fault(exc)
                return
            if self.process is not None:
                code = self.process.poll()
                if code is not None:
                    self._finish_process(code)
            if self.hardware is not None and self.mode in ('prepared', 'manual'):
                now = self.clock()
                if self.brake_until:
                    if now < self.brake_until:
                        self._write(1610, 1400)
                        return
                    self.brake_until = 0
                    self.requested = (1610, 1500)
                if self.mode == 'manual' and now - self.last_command > self.MANUAL_LEASE_S:
                    self.requested = (1610, 1500)
                    self.mode = 'prepared'
                    self.reason = '遥控指令中断超过 1.5 秒，已回中位；确认静止后重新启用遥控，自主任务不受此限制'
                self._write(*self.requested)

    def telemetry(self, run_dir):
        if not run_dir:
            return None
        paths = sorted(run_dir.glob('*/telemetry.jsonl'))
        if not paths:
            return None
        try:
            with paths[-1].open('rb') as stream:
                stream.seek(0, 2)
                stream.seek(max(0, stream.tell() - 65536))
                lines = stream.read().splitlines()
            for line in reversed(lines):
                try:
                    row = json.loads(line)
                    row['telemetry_age_s'] = max(0., time.time() - paths[-1].stat().st_mtime)
                    return row
                except (ValueError, UnicodeError):
                    continue
        except OSError:
            pass
        return None

    def status(self):
        with self.lock:
            run_dir, generation = self.run_dir, self.generation
            read_telemetry = self.mode not in ('manual', 'prepared')
        # SD-card/video I/O must not hold the manual output/stop lock.
        telemetry = self.telemetry(run_dir) if read_telemetry else None
        with self.lock:
            if run_dir != self.run_dir or generation != self.generation:
                telemetry = None
            process_running = self.process is not None and self.process.poll() is None
            throttle = self.output[1] if self.output_confirmed else None
            steering = self.output[0] if self.output_confirmed else None
            output_fresh = self.output_confirmed
            output_source = ('manual_uart' if self.hardware is not None else 'last_release_command') if output_fresh else 'unknown_output_not_confirmed'
            if self.process_drive and process_running:
                output_fresh = bool(telemetry and telemetry.get('telemetry_age_s', 1e9) < 2)
                throttle = telemetry.get('last_output_us') if output_fresh else None
                steering = telemetry.get('steering_us') if output_fresh else None
                output_source = 'cpp_telemetry' if output_fresh else 'unknown_no_fresh_cpp_telemetry'
            return {'mode': self.mode, 'reason': self.reason, 'owner': public_session(self.owner),
                    'generation': self.generation,
                    'task': self.task, 'throttle_us': throttle, 'steering_us': steering,
                    'output_fresh': output_fresh, 'output_source': output_source,
                    'process_running': process_running,
                    'hardware_output': self.hardware is not None or self.process_drive,
                    'run_dir': str(self.run_dir) if self.run_dir else None,
                    'manual_lease_s': self.MANUAL_LEASE_S, 'telemetry': telemetry,
                    'preview_error': self.preview.error}

    def frame(self, camera='secondary'):
        with self.lock:
            process, run_dir = self.process, self.run_dir
        if process is not None and run_dir:
            name = 'primary.jpg' if camera == 'primary' else 'secondary.jpg'
            paths = list(run_dir.glob('*/' + name))
            if paths:
                try:
                    path = max(paths, key=lambda p: p.stat().st_mtime)
                    if time.time() - path.stat().st_mtime < 2:
                        frame = path.read_bytes()
                        with self.lock:
                            return frame if process is self.process and run_dir == self.run_dir else None
                except OSError:
                    pass
            return None
        return self.preview.frame(camera)

    def _loop(self):
        while not self.stop_event.wait(.02):
            try:
                self.tick()
            except Exception as exc:
                with self.lock:
                    self._fault(exc)

    def close(self):
        self.stop_event.set()
        if self.worker:
            self.worker.join(2)
        with self.lock:
            try:
                self._stop()
            finally:
                self.closed = True
                self.preview.stop()
