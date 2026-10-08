"""Short raised-wheel manual trials, separate from race driving calibration.

The lease is checked by the hardware worker, independently of HTTP/video work.
Every motor stage is finite and followed by queued neutral by the backend.
"""

import hashlib
import json
import os
from pathlib import Path
import secrets
import socket
import threading
import time


class DriveError(ValueError):
    pass


def worker_control_endpoint(output, *, socket_root=Path('/tmp')):
    """Keep Unix IPC short while preserving the full, unique round log folder."""
    output = Path(output).resolve()
    endpoint = output/'control.sock'
    if len(os.fsencode(endpoint)) <= 103:
        return endpoint
    identity = hashlib.sha256(os.fsencode(output)).hexdigest()[:24]
    endpoint = Path(socket_root)/('cv-drive-'+identity)/'control.sock'
    if len(os.fsencode(endpoint)) > 103:
        raise ValueError('private worker socket root is too long')
    return endpoint


class PausedDrive:
    """Camera dashboard with simulation and physical control both inactive."""

    def __init__(self, reason):
        self.reason = reason

    def status(self):
        return {'mode': 'paused', 'reason': self.reason, 'controls_paused': True,
                'hardware_output': False, 'continuous_simulation': False,
                'speed_adjustable': False, 'speed_percent': 0, 'active_speed_percent': 0,
                'motor_available': False, 'steering_available': False,
                'reverse_available': False, 'motor_direction': 'stop',
                'steering_direction': 'center', 'real_output_paused_reason': self.reason,
                'session_remaining_s': None}

    def request(self, action, payload):
        if action not in ('stop', 'emergency'):
            raise DriveError(self.reason)
        return self.status()

    def close(self):
        pass


class SimulationBackend:
    hardware_output = False
    steering_available = True
    motor_available = True

    def motion(self, pulse, seconds):
        pass

    def neutral(self):
        pass

    def steering(self, pulse):
        pass

    def check(self):
        pass

    def close(self):
        pass


class ManualDrive:
    lease_s = .2

    def __init__(self, backend=None, *, clock=time.monotonic, settling_s=12,
                 session_s=120, preparation_s=None, event_sink=None, continuous_simulation=False,
                 raised_held=False, ground_held=False):
        self.backend = backend or SimulationBackend()
        if continuous_simulation and self.backend.hardware_output:
            raise ValueError('continuous simulation must never drive real hardware')
        if raised_held and (continuous_simulation or not self.backend.hardware_output
                            or getattr(self.backend, 'raised_held_reviewed', False) is not True
                            or session_s != 60):
            raise ValueError('held hardware needs the reviewed raised-wheel backend and a 60-second session')
        if ground_held and (continuous_simulation or raised_held or not self.backend.hardware_output
                            or getattr(self.backend, 'ground_held_reviewed', False) is not True
                            or type(session_s) is not int or session_s not in (180, 600)
                            or session_s != getattr(self.backend, 'manual_session_limit_s', 180)):
            raise ValueError('ground held control needs its reviewed backend and matching bounded session')
        ground_hold_s = getattr(self.backend, 'manual_motion_hold_max_s', 60) if ground_held else None
        if ground_held and (type(ground_hold_s) is not int or ground_hold_s not in (60, 600) or ground_hold_s > session_s):
            raise ValueError('ground motion hold must stay inside the reviewed session')
        self.raised_held = raised_held
        self.ground_held = ground_held
        self.motion_hold_max_s = 3.0 if raised_held else float(ground_hold_s) if ground_held else None
        self.continuous_simulation = continuous_simulation
        self.speed_percent = 20 if continuous_simulation else 100
        self.clock, self.settling_s = clock, settling_s
        self.session_s, self.preparation_s = session_s, preparation_s
        self.session_started = preparation_s is None
        initial_limit = preparation_s if preparation_s is not None else session_s
        self.deadline = clock() + initial_limit if initial_limit is not None else None
        self.event_sink = event_sink or (lambda event: None)
        self.lock = threading.RLock()
        self.mode, self.reason = 'disabled', '等待启用'
        self.owner, self.sequence, self.expires = None, -1, None
        self.last_input_source = None
        self.last_command_at = None
        self.motor, self.turn, self.started = 'stop', 'center', None
        self.stage, self.blocked, self.pulse = None, False, 1500
        self.stage_end = None
        self.reverse_cancelled = False
        self.steering_center_us = getattr(self.backend, 'steering_center_us', 1650)
        self.steering_us = self.steering_center_us
        self.last_steering_change = clock() if getattr(self.backend, 'steering_active', False) else float('-inf')
        self.ready_at = None
        self.stop_event = threading.Event()
        self.thread = None

    def event(self, name, **values):
        self.event_sink({'event': name, 'monotonic_s': self.clock(), **values})

    def _neutral(self, reason, *, latch=True):
        self.reverse_cancelled = (self.motor == 'reverse' and self.stage in ('brake', 'neutral_gap')
                                  and reason in ('松键或停车请求', '按键已松开'))
        self.backend.neutral()
        self.pulse, self.motor, self.turn, self.stage = 1500, 'stop', 'center', None
        self.started = None
        self.stage_end = None
        self.blocked = latch
        self.reason = reason
        self.event('neutral', reason=reason)

    def _client(self, value):
        if not isinstance(value, str) or not 8 <= len(value) <= 100 or not value.isascii():
            raise DriveError('invalid client identifier')
        return value

    def request(self, action, payload):
        with self.lock:
            if action in ('stop', 'emergency') and isinstance(payload, dict):
                source = payload.get('stop_source')
                if source not in ('keyboard_space', 'key_release', 'pointer_release',
                                  'window_blur', 'page_hidden', 'page_exit',
                                  'stop_button', 'emergency_button', 'conflicting_keys',
                                  'speed_zero', 'speed_failure', 'connection_recovery',
                                  'gamepad_disarmed', 'gamepad_mode_change', 'gamepad_poll_gap',
                                  'gamepad_read_failure', 'gamepad_disconnected', 'gamepad_device_changed',
                                  'gamepad_invalid', 'gamepad_focus_lost', 'gamepad_emergency',
                                  'gamepad_mixed_input', 'gamepad_deadman_release', 'gamepad_conflict',
                                  'gamepad_unavailable_direction', 'gamepad_release'):
                    source = 'unknown'
                self.event('safety_request', action=action, input_source=source,
                           motor_pulse_us=self.pulse, motor_stage=self.stage,
                           mode_before=self.mode)
            self.tick()
            if not isinstance(payload, dict):
                self.emergency('控制消息格式无效')
                raise DriveError('JSON object required')
            if action == 'emergency':
                self.emergency('网页急停')
            elif action == 'stop':
                # A stop is never blocked by ownership or a stale sequence.
                self._neutral('松键或停车请求')
                self.expires = None
            elif action == 'reset':
                if self.mode != 'emergency':
                    raise DriveError('only an emergency latch can be reset')
                self._neutral('已解除急停，需重新启用', latch=False)
                self.mode, self.owner, self.expires = 'disabled', None, None
            elif action == 'enable':
                client = self._client(payload.get('client'))
                if self.mode != 'disabled' or payload.get('bench_ready') is not True:
                    raise DriveError('confirm raised wheels and ready ESC before enabling')
                self._neutral('保持零油门，等待就绪' if getattr(self.backend, 'motor_available', True)
                              else '电调保持关闭，准备转向短测', latch=False)
                self.owner, self.sequence, self.expires = client, -1, None
                self.last_command_at = None
                self.mode, self.ready_at = 'settling', self.clock() + self.settling_s
                if not self.session_started:
                    # Reserve the active test interval only once. Resetting an
                    # emergency latch must not keep extending the test window.
                    self.deadline = self.ready_at + self.session_s
                    self.session_started = True
                self.event('enable', hardware_output=self.backend.hardware_output)
            elif action == 'command':
                self._command(payload)
            elif action == 'speed':
                if not self.continuous_simulation:
                    raise DriveError('speed adjustment is unavailable in fixed bench trials')
                client = self._client(payload.get('client'))
                if self.owner is not None and client != self.owner:
                    raise DriveError('speed control owned by another page')
                speed = payload.get('speed_percent')
                if type(speed) is not int or not 0 <= speed <= 100:
                    raise DriveError('speed must be an integer from 0 to 100')
                self.speed_percent = speed
                if speed == 0:
                    self._neutral('速度已设为零，请松开方向键', latch=True)
                    self.expires = None
                self.event('speed_changed', speed_percent=speed)
            else:
                raise DriveError('unknown control action')
            return self.status()

    def _command(self, payload):
        if self.mode != 'enabled':
            raise DriveError('控制尚未启用或已停车锁定，请重新启用')
        if self._client(payload.get('client')) != self.owner:
            raise DriveError('当前控制由另一页面使用，请回到已启用的页面')
        seq, motor, turn = (payload.get(k) for k in ('sequence', 'motor', 'steering'))
        if type(seq) is not int or seq <= self.sequence:
            raise DriveError('old or invalid command sequence')
        if motor not in ('stop', 'forward', 'reverse') or turn not in ('center', 'left', 'right'):
            self.emergency('控制值无效')
            raise DriveError('invalid direction')
        if turn != 'center' and not getattr(self.backend, 'steering_available', True):
            self.emergency('本次只测试驱动轮，转向信号已关闭')
            raise DriveError('steering disabled for motor-only trial')
        if motor != 'stop' and not getattr(self.backend, 'motor_available', True):
            self.emergency('本次只测试转向，驱动轮信号已关闭')
            raise DriveError('motor disabled for steering-only trial')
        if motor == 'reverse' and not getattr(self.backend, 'reverse_available', True):
            self.emergency('本轮只测试前进停车保护，倒车信号关闭')
            raise DriveError('reverse disabled for parking protection trial')
        self.sequence = seq
        # A release confirmation can arrive after the urgent stop request.
        # It must not start a new lease that expires while the keyboard is idle.
        self.expires = None if motor == 'stop' and turn == 'center' else self.clock()+self.lease_s
        source = payload.get('input_source')
        self.last_input_source = source if source in ('keyboard', 'pointer', 'gamepad') else 'unknown'
        received_at = self.clock()
        self.event('control_command', sequence=seq, motor=motor, steering=turn,
                   input_source=self.last_input_source,
                   interval_ms=None if self.last_command_at is None else
                   round((received_at-self.last_command_at)*1000, 3))
        self.last_command_at = received_at
        if motor == 'stop' and turn == 'center':
            if self.started is not None or self.blocked:
                self._neutral('按键已松开', latch=False)
            self.blocked = False
            return
        if self.blocked:
            return
        if self.started is not None and motor != self.motor:
            # A keyboard chord can arrive as two messages. Joining a motor key
            # immediately after a turn key is not a forward/reverse change.
            if (self.continuous_simulation or self.raised_held or self.ground_held) and motor == 'stop':
                self._neutral('行驶方向键已松开', latch=False)
            join = ((getattr(self.backend, 'combined_trial', False) or self.continuous_simulation or self.raised_held or self.ground_held)
                    and self.motor == 'stop' and motor in ('forward', 'reverse')
                    and (self.continuous_simulation or self.raised_held or self.ground_held or self.clock()-self.started <= .2))
            if self.started is not None and not join:
                self._neutral('方向已改变，请松开全部方向键后再按')
                return
            if join:
                self.event('chord_joined', motor=motor, steering=turn)
            self.started, self.stage, self.stage_end = None, None, None
        self.motor, self.turn = motor, turn
        if self.started is None:
            self.started = self.clock()
            self.stage = None
            self.reverse_cancelled = False
            self.event('trial_started', motor=motor, steering=turn,
                       input_source=self.last_input_source)

    def _reverse_tick(self, now):
        if not (self.continuous_simulation or self.ground_held) and now-self.started >= 3.0:
            self._neutral('倒车短测期限已到，请松开全部方向键后再按')
            return
        if self.stage is None:
            self.backend.motion(1300, .3)
            self.pulse, self.stage, self.stage_end = 1300, 'brake', now+.3
            self.event('motor_stage', pulse_us=1300, duration_s=.3, stage='brake')
        elif now >= self.stage_end:
            if self.stage == 'brake':
                self.backend.neutral()
                # The hardware call cancels PWM before restoring neutral.
                # Start the entire neutral gap after that call completes.
                self.pulse, self.stage = 1500, 'neutral_gap'
                self.stage_end = self.clock()+1.2
                self.event('motor_stage', pulse_us=1500, duration_s=0, stage='neutral_gap')
            elif self.stage == 'neutral_gap':
                if self.continuous_simulation or self.raised_held or self.ground_held:
                    self._renewed_motion(now, 'reverse')
                    return
                duration = min(.8, 3.0-(now-self.started))
                self.backend.motion(1300, duration)
                self.pulse, self.stage, self.stage_end = 1300, 'reverse', now+duration
                self.event('motor_stage', pulse_us=1300, duration_s=duration, stage='reverse')
            else:
                if self.continuous_simulation or self.raised_held or self.ground_held:
                    self._renewed_motion(now, 'reverse')
                else:
                    self._neutral('短测结束，请松开全部方向键后再按')

    def _renewed_motion(self, now, direction):
        if self.raised_held or self.ground_held:
            remaining = self.motion_hold_max_s - (now-self.started)
            if remaining < .02:
                self._neutral('本次按住行驶时间已到，请松开方向键')
                return
            pulse = 1575 if direction == 'forward' else 1300
            if self.stage != direction or self.stage_end is None or now >= self.stage_end:
                # Only fresh direction messages renew motion. Guardian checks
                # alone cannot renew this short motor segment.
                duration = min(.2, remaining)
                self.backend.motion(pulse, duration)
                self.pulse, self.stage, self.stage_end = pulse, direction, now+.06
                self.event('motor_stage', pulse_us=pulse, duration_s=duration,
                           stage=direction, raised_held=self.raised_held, ground_held=self.ground_held)
            return
        # This path is deliberately unavailable to every hardware backend.
        if self.speed_percent == 0:
            self._neutral('速度已设为零，请松开方向键')
            return
        offset = round((75 if direction == 'forward' else -200)*self.speed_percent/100)
        pulse = 1500 + offset
        if self.stage != direction or pulse != self.pulse or self.stage_end is None or now >= self.stage_end:
            self.backend.motion(pulse, .3)
            self.pulse, self.stage, self.stage_end = pulse, direction, now+.1
            self.event('motor_stage', pulse_us=pulse, duration_s=.3, stage=direction,
                       speed_percent=self.speed_percent, simulation=True)

    def emergency(self, reason):
        with self.lock:
            self._neutral(reason)
            if self.mode not in ('expired', 'fault', 'closed'):
                self.mode = 'emergency'
            self.expires = None

    def tick(self):
        with self.lock:
            if self.mode in ('expired', 'fault', 'closed'):
                return
            now = self.clock()
            try:
                self.backend.check()
                if self.deadline is not None and now >= self.deadline:
                    self._neutral('本次短测时间已到')
                    self.mode = 'expired'
                    self.backend.close()
                    return
                if self.mode == 'settling' and now >= self.ready_at:
                    self.mode = 'enabled'
                    self.reason = ('低速遥控已就绪，松键停车' if self.ground_held else
                                   '架空按住测试，每次最多 3 秒' if self.raised_held else '可以按住方向键做短测')
                if self.expires is not None and now >= self.expires:
                    # Ordinary heartbeat messages cannot clear this latch.
                    self.emergency('控制消息超时，已停车，需重新启用')
                if self.mode == 'enabled' and self.started is not None:
                    elapsed = now - self.started
                    if self.motion_hold_max_s is not None and elapsed >= self.motion_hold_max_s:
                        self._neutral('本次按住行驶时间已到，请松开方向键')
                    elif self.continuous_simulation and self.speed_percent == 0:
                        self._neutral('速度已设为零，请松开方向键')
                    elif self.motor == 'reverse':
                        self._reverse_tick(now)
                    elif self.continuous_simulation or self.raised_held or self.ground_held:
                        if self.motor == 'forward':
                            self._renewed_motion(now, 'forward')
                    else:
                        if self.motor == 'forward':
                            stage, pulse, duration, limit = 'forward', getattr(self.backend, 'forward_pulse_us', 1575), .8, .8
                        else:
                            stage, pulse, duration, limit = 'steering_only', 1500, 0, 2.0
                        if elapsed >= limit:
                            self._neutral('短测结束，请松开全部方向键后再按')
                        elif stage != self.stage:
                            if pulse == 1500:
                                self.backend.neutral()
                            else:
                                # If scheduling is late, do not extend the stage.
                                duration = min(duration, limit - elapsed)
                                self.backend.motion(pulse, duration)
                            self.pulse, self.stage = pulse, stage
                            self.event('motor_stage', pulse_us=pulse, duration_s=duration, stage=stage)
                target = {'center': self.steering_center_us, 'left': 1750, 'right': 1550}[
                    self.turn if self.mode == 'enabled' and not self.blocked else 'center']
                if target != self.steering_us and now - self.last_steering_change >= .02:
                    single_target = getattr(self.backend, 'steering_single_target', False)
                    pulse = target if single_target else self.steering_us + max(-10, min(10, target - self.steering_us))
                    self.backend.steering(pulse)
                    self.steering_us = pulse
                    self.last_steering_change = now
                    self.event('steering_step', pulse_us=self.steering_us, target_us=target,
                               single_target=single_target)
                # Give the final center pulse time to reach the servo before
                # releasing its signal; an input pin is no longer a servo.
                if (target == self.steering_us == self.steering_center_us and self.started is None
                        and now - self.last_steering_change >= getattr(self.backend, 'steering_settle_s', .2)):
                    idle = getattr(self.backend, 'steering_idle', None)
                    if idle:
                        idle()
            except Exception as exc:
                self.mode, self.reason = 'fault', str(exc)
                try:
                    self.backend.neutral()
                finally:
                    self.backend.close()
                self.event('fault', reason=self.reason)

    def status(self):
        with self.lock:
            return {'mode': self.mode, 'reason': self.reason,
                    'hardware_output': self.backend.hardware_output,
                    'continuous_simulation': self.continuous_simulation,
                    'raised_held_trial': self.raised_held,
                    'ground_held_trial': self.ground_held,
                    'motion_hold_max_s': self.motion_hold_max_s,
                    'speed_adjustable': self.continuous_simulation,
                    'speed_percent': self.speed_percent,
                    'active_speed_percent': self.speed_percent if self.pulse != 1500 else 0,
                    'motor_direction': self.motor,
                    'steering_direction': self.turn,
                    'real_output_paused_reason': getattr(self.backend, 'real_output_paused_reason', ''),
                    'steering_available': getattr(self.backend, 'steering_available', True),
                    'motor_available': getattr(self.backend, 'motor_available', True),
                    'steering_signal_active': getattr(self.backend, 'steering_active', False),
                    'combined_trial': getattr(self.backend, 'combined_trial', False),
                    'ground_short_trial': getattr(self.backend, 'ground_short_trial', False),
                    'forward_pulse_us': getattr(self.backend, 'forward_pulse_us', 1575),
                    'load_probe': getattr(self.backend, 'load_probe', False),
                    'raised_load_probe': getattr(self.backend, 'raised_load_probe', False),
                    'test_session_s': self.session_s,
                    'parking_protection_trial': getattr(self.backend, 'parking_protection_trial', False),
                    'reverse_available': getattr(self.backend, 'reverse_available', True),
                    'motor_pulse_us': self.pulse if getattr(self.backend, 'motor_available', True) else None,
                    'motor_stage': self.stage,
                    'reverse_wait_remaining_s': max(0, self.stage_end-self.clock())
                        + (1.2 if self.stage == 'brake' else 0)
                        if self.motor == 'reverse' and self.started is not None
                        and self.stage in ('brake', 'neutral_gap') else 0,
                    'reverse_cancelled': self.reverse_cancelled,
                    'steering_pulse_us': self.steering_us,
                    'steering_center_us': self.steering_center_us,
                    'steering_single_target': getattr(self.backend, 'steering_single_target', False),
                    'last_command_sequence': self.sequence, 'last_input_source': self.last_input_source,
                    'lease_ms': round(self.lease_s * 1000), 'release_required': self.blocked,
                    'ready_in_s': max(0, self.ready_at - self.clock()) if self.mode == 'settling' else 0,
                    'session_phase': 'test' if self.session_started else 'preparing',
                    'session_remaining_s': max(0, self.deadline-self.clock()) if self.deadline is not None else None}

    def start(self):
        def watch():
            while not self.stop_event.wait(.02):
                self.tick()
        self.thread = threading.Thread(target=watch, name='manual-drive-lease', daemon=True)
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1)
        with self.lock:
            if self.mode != 'closed':
                try:
                    self._neutral('服务已停止')
                finally:
                    self.backend.close()
                    self.mode = 'closed'


class WorkerDrive:
    """RPC into the system-Python GPIO worker; the camera venv has no lgpio."""
    def __init__(self, path, *, steering_available=True, motor_available=True, parking_protection=False, ground_short_trial=False, raised_load_probe=False, ground_held=False):
        self.path = str(path)
        self.last_status = {'hardware_output': True, 'steering_available': steering_available,
                            'motor_available': motor_available, 'reverse_available': not parking_protection,
                            'parking_protection_trial': parking_protection, 'ground_short_trial': ground_short_trial,
                            'ground_held_trial': ground_held,
                            'raised_load_probe': raised_load_probe, 'reverse_available': not (parking_protection or raised_load_probe)}
        self.last_status_at = time.monotonic()

    def request(self, action, payload):
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                connection.settimeout(.7)
                connection.connect(self.path)
                connection.sendall((json.dumps({'action': action, 'payload': payload})+'\n').encode())
                with connection.makefile('rb') as stream:
                    reply = json.loads(stream.readline(8193))
        except OSError as error:
            raise DriveError('控制进程已停止，请关闭电调；重新准备后才能继续测试。') from error
        if 'error' in reply:
            raise DriveError(reply['error'])
        self.last_status, self.last_status_at = dict(reply['status']), time.monotonic()
        return reply['status']

    def status(self):
        try:
            return self.request('status', {})
        except (OSError, ValueError, DriveError):
            remaining = self.last_status.get('session_remaining_s')
            if remaining is not None:
                remaining = max(0, remaining - (time.monotonic() - self.last_status_at))
            expired = remaining == 0
            return {**self.last_status, 'mode': 'expired' if expired else 'fault',
                    'reason': '本轮测试窗口已结束，请关闭电调' if expired else '控制进程已停止，请关闭电调',
                    'motor_pulse_us': None, 'steering_signal_active': None,
                    'session_remaining_s': remaining, 'worker_available': False}

    def close(self):
        try:
            self.request('shutdown', {})
        except (OSError, ValueError):
            pass


class DeferredDrive:
    """Start one ground worker only on an explicit ready/enable request.

    No hardware owner or motion requests are created by status polling. Startup
    is asynchronous so an emergency can cancel it while the worker opens.
    An active/expired worker is never replaced to extend the test deadline.
    """
    def __init__(self, factory, initial_status):
        self.factory, self.initial_status = factory, dict(initial_status)
        self.lock = threading.RLock()
        self.delegate = None
        self.thread = None
        self.attempted = False
        self.cancelled = False
        self.closed = False
        self.mode, self.reason = 'disabled', '准备就绪，等待点击启用'
        self.owner = None

    def status(self):
        with self.lock:
            delegate = self.delegate
            if delegate is None:
                return {**self.initial_status, 'mode': self.mode, 'reason': self.reason,
                        'hardware_output': False, 'pending_hardware_start': not self.attempted,
                        'worker_started': False, 'session_remaining_s': None,
                        'session_phase': 'preparing', 'ready_in_s': 0,
                        'motor_pulse_us': 1500, 'motor_direction': 'stop',
                        'steering_direction': 'center', 'active_speed_percent': 0}
        return {**delegate.status(), 'worker_started': True, 'pending_hardware_start': False}

    def _start(self, payload):
        candidate = None
        try:
            candidate = self.factory()
            with self.lock:
                cancel = self.cancelled or self.closed
            if not cancel:
                candidate.request('enable', payload)
            with self.lock:
                if self.cancelled or self.closed:
                    cancel = True
                elif not cancel:
                    self.delegate = candidate
                    candidate = None
            if cancel and not self.closed:
                with self.lock:
                    self.mode, self.reason = 'fault', '启用已取消，请关闭电调后重新准备'
        except Exception as error:
            with self.lock:
                if not self.closed:
                    self.mode, self.reason = 'fault', str(error)
        finally:
            if candidate is not None:
                candidate.close()

    def request(self, action, payload):
        if not isinstance(payload, dict):
            raise DriveError('JSON object required')
        with self.lock:
            if self.closed:
                raise DriveError('控制服务已停止')
            delegate = self.delegate
            if delegate is None:
                if action == 'status':
                    return self.status()
                if action == 'enable':
                    client = ManualDrive._client(self, payload.get('client'))
                    if payload.get('bench_ready') is not True:
                        raise DriveError('请先确认车辆和现场就绪')
                    if self.mode == 'starting':
                        if client != self.owner:
                            raise DriveError('当前控制由另一页面使用')
                        return self.status()
                    if self.mode != 'disabled' or self.attempted:
                        raise DriveError('本轮尚不能启用，请重新准备')
                    self.owner, self.attempted = client, True
                    self.mode, self.reason = 'starting', '正在准备遥控信号，请保持车辆静止'
                    self.thread = threading.Thread(target=self._start, args=(dict(payload),),
                                                   name='ground-worker-start', daemon=True)
                    self.thread.start()
                    return self.status()
                if action in ('stop', 'emergency'):
                    if self.mode == 'starting':
                        self.cancelled = True
                        self.mode, self.reason = 'emergency', '启用已取消，保持零油门'
                    elif action == 'emergency':
                        self.mode, self.reason = 'emergency', '已急停，尚未启动行驶输出'
                    return self.status()
                if action == 'reset' and not self.attempted:
                    self.mode, self.reason = 'disabled', '准备就绪，等待点击启用'
                    return self.status()
                raise DriveError('控制尚未启用，方向请求不会排队')
        return delegate.request(action, payload)

    def close(self):
        with self.lock:
            self.closed = self.cancelled = True
            self.mode, self.reason = 'closed', '服务已停止'
            delegate, thread = self.delegate, self.thread
        if delegate is not None:
            delegate.close()
        if thread is not None:
            thread.join(timeout=7)


class RepeatableDrive(DeferredDrive):
    """Explicit new rounds after verified normal cleanup, never automatic retry."""
    def __init__(self, factory, initial_status):
        super().__init__(factory, initial_status)
        self.trial_number = 1
        self.trial_token = secrets.token_hex(16)

    def _describe(self, state):
        return {**state, 'repeatable_trials': True, 'trial_number': self.trial_number,
                'trial_token': self.trial_token,
                'can_start_next_round': state.get('mode') == 'expired'
                and state.get('trial_completed_normally') is True}

    def status(self):
        with self.lock:
            return self._describe(super().status())

    def request(self, action, payload):
        if not isinstance(payload, dict):
            raise DriveError('JSON object required')
        with self.lock:
            # Old rounds can always stop output, but cannot enable or replay it.
            if action not in ('status', 'stop', 'emergency') and payload.get('trial_token') != self.trial_token:
                raise DriveError('本轮已更新，请松开按键并核对页面状态')
            if action == 'next_trial':
                if self.closed or not self.status()['can_start_next_round']:
                    raise DriveError('仅正常结束并完成停车清理后才能开始下一轮')
                ManualDrive._client(self, payload.get('client'))
                if payload.get('bench_ready') is not True:
                    raise DriveError('请确认车已停稳且现场就绪')
                try:
                    self.delegate.close()
                except Exception as error:
                    self.delegate = None
                    self.mode, self.reason = 'fault', '上一轮清理失败：'+str(error)
                    raise DriveError(self.reason) from error
                self.delegate = None
                self.attempted = self.cancelled = False
                self.mode, self.reason, self.owner = 'disabled', '下一轮已准备，等待启用', None
                self.trial_number += 1
                self.trial_token = secrets.token_hex(16)
                # This explicit ready request starts a new bounded worker; no
                # direction values from the previous worker are forwarded.
                payload = {'client': payload['client'], 'bench_ready': True}
                return self._describe(super().request('enable', payload))
            return self._describe(super().request(action, payload))
