"""Raspberry Pi 4/5 hardware PWM and a bounded speed-feedback executor.

BCM12/PWM0 is steering; BCM13/PWM1 is the ESC. Opening PWM requires explicit
run mode and measured pulse calibration. Importing/checking this module never
touches output pins. The software watchdog cannot replace a physical cutoff.
"""

import json
import math
import os
import subprocess
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from .race import finite_number


@dataclass
class PiControlConfig:
    steering_gpio: int = 12
    esc_gpio: int = 13
    pwm_chip: str | None = None
    frequency_hz: int = 50
    esc_model: str = ""
    calibration_confirmed: bool = False
    esc_neutral_us: int | None = None
    esc_forward_us: int | None = None
    steering_left_us: int | None = None
    steering_center_us: int | None = None
    steering_right_us: int | None = None
    speed_kp: float | None = None
    speed_ki: float = 0.0
    maximum_throttle: float = 0.15
    command_lease_s: float = 0.25
    announcement_wav: str | None = None
    announcement_text: str = ""

    def missing(self):
        names = [n for n in ("esc_neutral_us", "esc_forward_us", "steering_left_us",
                             "steering_center_us", "steering_right_us", "speed_kp") if getattr(self, n) is None]
        if not self.esc_model.strip():
            names.append("confirmed_esc_model")
        if not self.calibration_confirmed:
            names.append("bench_calibration_confirmation")
        return names

    def validate(self, require_ready=False):
        if type(self.steering_gpio) is not int or type(self.esc_gpio) is not int or (self.steering_gpio, self.esc_gpio) != (12, 13):
            raise ValueError("this configuration reserves BCM12/channel0 for steering and BCM13/channel1 for ESC")
        if type(self.frequency_hz) is not int or self.frequency_hz != 50:
            raise ValueError("this actuator requires confirmed 50-Hz RC equipment")
        if type(self.calibration_confirmed) is not bool or not isinstance(self.esc_model, str):
            raise ValueError("invalid calibration confirmation or ESC model")
        for name in ("esc_neutral_us", "esc_forward_us", "steering_left_us", "steering_center_us", "steering_right_us"):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or not 500 <= value <= 2500):
                raise ValueError(f"{name} must be a measured 500..2500 microsecond pulse")
        if self.esc_neutral_us is not None and self.esc_forward_us is not None and self.esc_neutral_us == self.esc_forward_us:
            raise ValueError("ESC neutral and forward endpoint cannot be equal")
        pulse = (self.steering_left_us, self.steering_center_us, self.steering_right_us)
        if all(v is not None for v in pulse) and not (pulse[0] < pulse[1] < pulse[2] or pulse[2] < pulse[1] < pulse[0]):
            raise ValueError("steering center must lie between calibrated left/right endpoints")
        for name in ("speed_kp", "speed_ki", "maximum_throttle", "command_lease_s"):
            value = getattr(self, name)
            if value is not None and (not finite_number(value) or value < 0):
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.speed_kp is not None and self.speed_kp <= 0:
            raise ValueError("speed_kp must be positive after calibration")
        if not 0 < self.maximum_throttle <= 1 or not 0 < self.command_lease_s <= .25:
            raise ValueError("invalid throttle ceiling or command lease; maximum lease is 250 ms")
        if require_ready and self.missing():
            raise ValueError("PWM calibration incomplete: " + ", ".join(self.missing()))
        return self


def load_pi_config(path):
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("Pi control config must be an object")
    return PiControlConfig(**data).validate()


def inspect_pwm_chips():
    chips = []
    for p in Path('/sys/class/pwm').glob('pwmchip*'):
        compatible = p / 'device/of_node/compatible'
        text = compatible.read_bytes().replace(b'\0', b',').decode('ascii', 'replace') if compatible.exists() else ''
        chips.append({"path": str(p), "compatible": text,
                      "channels": int((p / 'npwm').read_text()), "export_writable": os.access(p / 'export', os.W_OK)})
    return chips


def select_pwm_chip(model, pin_state, chips, configured=None):
    """Validate board, exact pin mux and controller before any PWM mutation."""
    if b'Raspberry Pi 5 Model B' in model:
        compatible, count = 'raspberrypi,rp1-pwm', 4
        labels = ('GPIO12 = PWM0_CHAN0', 'GPIO13 = PWM0_CHAN1')
    elif b'Raspberry Pi 4 Model B' in model:
        compatible, count = 'brcm,bcm2835-pwm', 2
        labels = ('GPIO12 = PWM0', 'GPIO13 = PWM1')
    else:
        raise RuntimeError('BCM12/13 mapping supports only Raspberry Pi 4B and Pi 5B')
    if not all(label in pin_state for label in labels):
        raise RuntimeError('BCM12/13 hardware PWM pin mux is not active; no output was enabled')
    candidates = [c for c in chips if compatible in c['compatible'] and c['channels'] >= count
                  and (configured is None or c['path'] == configured)]
    if len(candidates) != 1:
        raise RuntimeError('a unique board-compatible PWM controller must be configured')
    return candidates[0]


class HardwarePWM:
    """Linux sysfs PWM. Refuses to take over enabled channels or wrong pinmux."""
    def __init__(self, config):
        self.config = config
        self.chip = None
        self.channels = {}
        self.exported = []
        self.lock_file = None

    def write(self, channel, name, value):
        (self.channels[channel] / name).write_text(str(value))

    def open(self):
        import fcntl

        self.config.validate(require_ready=True)
        model = Path('/proc/device-tree/model').read_bytes()
        result = subprocess.run(['pinctrl', 'get', '12-13'], capture_output=True, text=True, check=True)
        chip = select_pwm_chip(model, result.stdout, inspect_pwm_chips(), self.config.pwm_chip)
        self.chip = Path(chip['path'])
        lock_path = Path('/tmp/carvision-pi-pwm.lock')
        lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | getattr(os, 'O_NOFOLLOW', 0), 0o600)
        self.lock_file = os.fdopen(lock_fd, 'w')
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            for channel in (0, 1):
                folder = self.chip / f'pwm{channel}'
                if folder.exists() and (folder / 'enable').read_text().strip() != '0':
                    raise RuntimeError(f"PWM{channel} is already enabled; existing output was preserved")
            for channel in (0, 1):
                folder = self.chip / f'pwm{channel}'
                if not folder.exists():
                    (self.chip / 'export').write_text(str(channel))
                    self.exported.append(channel)
                    deadline = time.monotonic() + 1
                    while not folder.exists() and time.monotonic() < deadline:
                        time.sleep(.01)
                self.channels[channel] = folder
                self.write(channel, 'enable', 0)
                self.write(channel, 'duty_cycle', 0)
                self.write(channel, 'period', 1_000_000_000 // self.config.frequency_hz)
                self.write(channel, 'polarity', 'normal')
            self.set_pulses(self.config.esc_neutral_us, self.config.steering_center_us)
            self.write(0, 'enable', 1)
            self.write(1, 'enable', 1)
        except BaseException:
            self.close()
            raise

    def set_pulses(self, esc_us, steering_us):
        self.write(1, 'duty_cycle', int(esc_us) * 1000)
        self.write(0, 'duty_cycle', int(steering_us) * 1000)

    def neutral(self):
        if 1 in self.channels:
            self.write(1, 'duty_cycle', self.config.esc_neutral_us * 1000)

    def close(self):
        errors = []
        if 1 in self.channels:
            try:
                self.neutral()
                time.sleep(.4)
            except Exception as exc:
                errors.append(str(exc))
        for channel in self.channels:
            for name in ('enable', 'duty_cycle'):
                try:
                    self.write(channel, name, 0)
                except Exception as exc:
                    errors.append(str(exc))
        for channel in self.exported:
            try:
                (self.chip / 'unexport').write_text(str(channel))
            except Exception as exc:
                errors.append(str(exc))
        self.channels.clear()
        self.exported.clear()
        if self.lock_file:
            self.lock_file.close()
            self.lock_file = None
        return errors


class PiActuator:
    def __init__(self, config, run=False, backend=None):
        self.config = config.validate(require_ready=run)
        self.run = run
        self.backend = backend
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.expires = None
        self.error = None
        self.watchdog = None
        self.opened = False

    def open(self):
        if not self.run:
            return
        self.backend = self.backend or HardwarePWM(self.config)
        self.backend.open()
        self.opened = True
        self.watchdog = threading.Thread(target=self._watchdog, daemon=True, name='pi-pwm-watchdog')
        self.watchdog.start()

    def planned_pulses(self, throttle, steering):
        if not finite_number(throttle) or not finite_number(steering) or not 0 <= throttle <= 1 or not -1 <= steering <= 1:
            raise ValueError("throttle must be 0..1 and steering -1..1, both finite")
        if self.config.missing():
            return {"esc_us": None, "steering_us": None, "missing": self.config.missing()}
        c = self.config
        throttle = min(throttle, c.maximum_throttle)
        esc = round(c.esc_neutral_us + throttle * (c.esc_forward_us - c.esc_neutral_us))
        endpoint = c.steering_right_us if steering >= 0 else c.steering_left_us
        servo = round(c.steering_center_us + abs(steering) * (endpoint - c.steering_center_us))
        return {"esc_us": esc, "steering_us": servo, "missing": []}

    def apply(self, throttle, steering):
        try:
            pulses = self.planned_pulses(throttle, steering)
        except Exception:
            self.neutral()
            raise
        if self.run:
            with self.lock:
                if not self.opened or self.error:
                    raise RuntimeError(self.error or "PWM output has not been opened")
                try:
                    self.backend.set_pulses(pulses['esc_us'], pulses['steering_us'])
                    self.expires = time.monotonic() + self.config.command_lease_s
                except Exception as exc:
                    self.error = str(exc)
                    self.backend.neutral()
                    raise
        return {**pulses, "hardware_output": self.run}

    def neutral(self):
        if self.run and self.opened:
            with self.lock:
                self.backend.neutral()
                self.expires = None

    def _watchdog(self):
        while not self.stop_event.wait(.01):
            with self.lock:
                if self.expires is not None and time.monotonic() >= self.expires:
                    try:
                        self.backend.neutral()
                    except Exception as exc:
                        self.error = str(exc)
                    self.expires = None

    def close(self):
        self.stop_event.set()
        if self.watchdog:
            self.watchdog.join(timeout=1)
        if self.run and self.opened:
            self.opened = False
            return self.backend.close()
        return []


class SpeedFeedbackController:
    def __init__(self, config):
        self.config = config
        self.integral = 0.0
        self.previous_t = None

    def command(self, intent, observation):
        c = self.config
        if intent['action'] == 'stop' or intent['speed_mps'] <= 0 or not observation.telemetry_valid or not finite_number(observation.speed_mps):
            self.integral = 0
            self.previous_t = observation.t_s
            return 0.0, 0.0
        if c.speed_kp is None:
            return 0.0, 0.0
        dt = 0 if self.previous_t is None else max(0, min(c.command_lease_s, observation.t_s - self.previous_t))
        self.previous_t = observation.t_s
        error = intent['speed_mps'] - observation.speed_mps
        self.integral = max(-c.maximum_throttle, min(c.maximum_throttle, self.integral + error * dt * c.speed_ki))
        throttle = max(0, min(c.maximum_throttle, c.speed_kp * error + self.integral))
        return throttle, intent['steering_normalized']


def pi_check(args):
    config = load_pi_config(args.pi_config)
    return {"architecture": "Raspberry Pi 4B only", "hardware_output": False,
            "steering": {"bcm": 12, "physical_pin": 32, "pwm_channel": 0},
            "esc": {"bcm": 13, "physical_pin": 33, "pwm_channel": 1},
            "calibration_ready": not config.missing(), "missing": config.missing(),
            "pwm_controllers": inspect_pwm_chips(), "config": asdict(config)}
