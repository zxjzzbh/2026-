"""Two direct-wired pan/tilt servos. Defaults never open GPIO or a daemon.

Uses only pigpio's PCM/DMA-timed servo command on BCM17/27. Waveforms and
hardware-PWM commands are deliberately absent: they would share PWM resources
with the car's BCM12/13 Linux PWM controller. Not a public network controller.
"""

import json
import socket
import struct
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from .race import finite_number


@dataclass
class ServoCalibration:
    model: str = ''
    minimum_us: int | None = None
    center_us: int | None = None
    maximum_us: int | None = None

    def validate(self):
        if not isinstance(self.model, str):
            raise ValueError('servo model must be a string')
        values = (self.minimum_us, self.center_us, self.maximum_us)
        if any(v is not None and (type(v) is not int or not 500 <= v <= 2500) for v in values):
            raise ValueError('servo pulses must be measured integer microseconds in 500..2500')
        if all(v is not None for v in values) and not (values[0] < values[1] < values[2] or values[2] < values[1] < values[0]):
            raise ValueError('servo center must be between the two calibrated directional limits')
        return self

    def pulse(self, value):
        if not finite_number(value) or not -1 <= value <= 1:
            raise ValueError('gimbal position must be a finite value in -1..1')
        if any(v is None for v in (self.minimum_us, self.center_us, self.maximum_us)):
            return None
        target = self.maximum_us if value >= 0 else self.minimum_us
        return round(self.center_us + abs(value) * (target - self.center_us))


@dataclass
class GimbalConfig:
    pan_gpio: int = 17
    tilt_gpio: int = 27
    supply_voltage_v: float | None = None
    signal_3v3_confirmed: bool = False
    calibration_confirmed: bool = False
    daemon_compatibility_confirmed: bool = False
    pan: ServoCalibration = field(default_factory=ServoCalibration)
    tilt: ServoCalibration = field(default_factory=ServoCalibration)
    maximum_step_us: int = 10
    update_interval_s: float = .02
    local_port: int = 8890

    def missing(self):
        result = []
        for name in ('pan', 'tilt'):
            axis = getattr(self, name)
            if not axis.model.strip():
                result.append(name + '_model')
            result += [name + '_' + k for k in ('minimum_us','center_us','maximum_us') if getattr(axis, k) is None]
        for name in ('signal_3v3_confirmed','calibration_confirmed','daemon_compatibility_confirmed'):
            if not getattr(self, name):
                result.append(name)
        if self.supply_voltage_v is None:
            result.append('confirmed_servo_supply_voltage')
        return result

    def validate(self, require_ready=False):
        if type(self.pan_gpio) is not int or type(self.tilt_gpio) is not int or (self.pan_gpio,self.tilt_gpio) != (17,27):
            raise ValueError('gimbal signals are reserved on BCM17/27, separate from BCM12/13 car control')
        for name in ('signal_3v3_confirmed','calibration_confirmed','daemon_compatibility_confirmed'):
            if type(getattr(self,name)) is not bool:
                raise ValueError(name + ' must be boolean')
        self.pan.validate()
        self.tilt.validate()
        if self.supply_voltage_v is not None and (not finite_number(self.supply_voltage_v) or not 3 <= self.supply_voltage_v <= 12):
            raise ValueError('servo supply voltage must be confirmed for the actual model')
        if type(self.maximum_step_us) is not int or not 1 <= self.maximum_step_us <= 20:
            raise ValueError('maximum pulse step is 20 microseconds')
        if not finite_number(self.update_interval_s) or not .02 <= self.update_interval_s <= .1:
            raise ValueError('invalid gimbal update interval')
        if type(self.local_port) is not int or not 1024 <= self.local_port <= 65535:
            raise ValueError('invalid local daemon port')
        if require_ready and self.missing():
            raise ValueError('gimbal calibration incomplete: ' + ', '.join(self.missing()))
        return self


def load_gimbal_config(path):
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if not isinstance(data, dict):
        raise ValueError('gimbal config must be an object')
    for axis in ('pan','tilt'):
        data[axis] = ServoCalibration(**data.get(axis, {}))
    return GimbalConfig(**data).validate()


class LocalPigpio:
    """Minimal fixed-command loopback client. No remote host or wave API."""
    def __init__(self, port):
        self.port = port
        self.connection = None

    def open(self):
        self.connection = socket.create_connection(('127.0.0.1', self.port), timeout=.25)

    def command(self, command, p1=0, p2=0):
        self.connection.sendall(struct.pack('<4I', command,p1,p2,0))
        result = b''
        while len(result) < 16:
            part = self.connection.recv(16-len(result))
            if not part:
                raise ConnectionError('gimbal daemon disconnected')
            result += part
        value = struct.unpack('<3Ii',result)[3]
        if value < 0:
            raise RuntimeError(f'pigpio command {command} failed: {value}')
        return value

    def version(self):
        return self.command(26)

    def mode(self, gpio):
        return self.command(1,gpio)

    def servo_pulse(self, gpio):
        if gpio not in (17,27):
            raise ValueError('only BCM17/27 are allowed by the gimbal client')
        return self.command(84,gpio)

    def set_servo(self,gpio,pulse):
        if gpio not in (17,27):
            raise ValueError('only BCM17/27 are allowed by the gimbal client')
        if type(pulse) is not int or (pulse != 0 and not 500 <= pulse <= 2500):
            raise ValueError('invalid servo pulse')
        self.command(8,gpio,pulse)

    def close(self):
        if self.connection:
            self.connection.close()
            self.connection = None

    def release(self, gpio):
        if gpio not in (17,27):
            raise ValueError('only BCM17/27 are allowed by the gimbal client')
        self.command(0,gpio,0)


def check_daemon_configuration(config):
    """Read process arguments before using a daemon that could share hardware."""
    candidates = []
    for cmdline in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = [v.decode() for v in cmdline.read_bytes().split(b'\0') if v]
        except (OSError,UnicodeError):
            continue
        if argv and Path(argv[0]).name == 'pigpiod':
            candidates.append(argv)
    valid = []
    for argv in candidates:
        def option(name):
            for i,arg in enumerate(argv):
                if arg == name and i+1 < len(argv):
                    return argv[i+1]
                if arg.startswith(name) and len(arg)>len(name):
                    return arg[len(name):]
            return None
        mask = option('-x')
        if '-l' in argv and '-f' in argv and option('-n') == '127.0.0.1' and option('-t') == '1' and option('-p') == str(config.local_port) and mask is not None:
            try:
                if int(mask,0) == ((1<<17)|(1<<27)):
                    valid.append(argv)
            except ValueError:
                pass
    if len(valid) != 1 or len(candidates) != 1:
        raise RuntimeError('require one local-only PCM pigpiod with -f -l -n 127.0.0.1 -t 1 -p 8890 -x 0x08020000; never use wave commands')


def gimbal_check(args):
    config = load_gimbal_config(args.gimbal_config)
    return {'hardware_output':False,'pan':{'bcm':17,'physical_pin':11},
            'tilt':{'bcm':27,'physical_pin':13},'ready':not config.missing(),
            'missing':config.missing(),'config':asdict(config)}


def gimbal_test(args, backend=None):
    config = load_gimbal_config(args.gimbal_config)
    pulses = {'pan_us':config.pan.pulse(args.pan),'tilt_us':config.tilt.pulse(args.tilt)}
    if not args.run:
        return {'hardware_output':False,'requested_normalized':{'pan':args.pan,'tilt':args.tilt},
                **pulses,'missing':config.missing()}
    if not args.acknowledge_motion:
        raise ValueError('gimbal motion requires prepared wiring/calibration and --acknowledge-motion')
    config.validate(require_ready=True)
    if not finite_number(args.seconds) or not .1 <= args.seconds <= 3:
        raise ValueError('gimbal bench test duration must be .1..3 seconds')
    if backend is None:
        if sys.platform != 'linux':
            raise RuntimeError('actual gimbal output is for the prepared Raspberry Pi only')
        check_daemon_configuration(config)
        backend = LocalPigpio(config.local_port)
    opened = False
    enabled = False
    cleanup_errors = []
    try:
        backend.open()
        opened = True
        if any(backend.mode(gpio) != 0 for gpio in (17,27)):
            raise RuntimeError('gimbal pins are already configured as outputs; existing owner was preserved')
        current = [config.pan.center_us,config.tilt.center_us]
        enabled = True
        # The first center pulse is only permitted after bench-confirmed calibration.
        for gpio,pulse in zip((17,27),current):
            backend.set_servo(gpio,pulse)
        deadline = time.monotonic()+args.seconds
        while time.monotonic()<deadline:
            for i,(gpio,target) in enumerate(zip((17,27),(pulses['pan_us'],pulses['tilt_us']))):
                current[i] += max(-config.maximum_step_us,min(config.maximum_step_us,target-current[i]))
                backend.set_servo(gpio,current[i])
            time.sleep(config.update_interval_s)
    finally:
        if enabled:
            for gpio in (17,27):
                try:
                    backend.set_servo(gpio,0)
                except Exception as exc:
                    cleanup_errors.append(str(exc))
                try:
                    backend.release(gpio)
                except Exception as exc:
                    cleanup_errors.append(str(exc))
        if opened:
            backend.close()
    if cleanup_errors:
        raise RuntimeError('failed to disable gimbal pulses: '+ '; '.join(cleanup_errors))
    return {'hardware_output':True,**pulses,'pulse_output_stopped':True,
            'note':'pulse off releases holding torque; support the camera during bench testing'}
