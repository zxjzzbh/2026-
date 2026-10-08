"""Narrow RasAdapter5A UART transport, based on Hiwonder's published SDK.

Source: the board_demo SDK linked from the manufacturer's expansion-board
control lesson. Importing/opening this transport sends no actuator commands.
PWM position readback is the controller's stored command, not servo feedback.
"""
import math
import os
import select
import struct
import time

SOURCE = 'https://wiki.hiwonder.com/projects/Raspberry-Pi-5-Expansion-Board/en/latest/docs/2_Expansion_Board_Control_Lesson.html'


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8c if value & 1 else 0)
    return value


def frame(function, payload):
    data = bytes((function, len(payload))) + bytes(payload)
    return b'\xaa\x55' + data + bytes((crc8(data),))


class Decoder:
    def __init__(self):
        self.buffer = b''

    def feed(self, data):
        self.buffer += data
        packets = []
        while True:
            index = self.buffer.find(b'\xaa\x55')
            if index < 0:
                self.buffer = self.buffer[-1:] if self.buffer.endswith(b'\xaa') else b''
                break
            self.buffer = self.buffer[index:]
            if len(self.buffer) < 5:
                break
            length = self.buffer[3] + 5
            if len(self.buffer) < length:
                break
            if crc8(self.buffer[2:length-1]) == self.buffer[length-1]:
                packets.append((self.buffer[2], self.buffer[4:length-1]))
                self.buffer = self.buffer[length:]
            else:
                self.buffer = self.buffer[1:]
        return packets


class RasAdapter:
    def __init__(self, port='/dev/ttyAMA0'):
        self.port = port
        self.fd = None
        self.original = None
        self.decoder = Decoder()

    def open(self):
        import fcntl
        import termios
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.original = termios.tcgetattr(self.fd)
            state = termios.tcgetattr(self.fd)
            state[0] = state[1] = state[3] = 0
            state[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            state[4] = state[5] = termios.B1000000
            state[6][termios.VMIN] = state[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, state)
        except BaseException:
            self.close()
            raise

    def receive(self, timeout=.5):
        if select.select([self.fd], [], [], timeout)[0]:
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                # Readiness can expire before a nonblocking UART read. The
                # caller's deadline still applies; absence is not a bad frame.
                return []
            return self.decoder.feed(data)
        return []

    def send(self, function, payload):
        packet = frame(function, payload)
        if not select.select([], [self.fd], [], .2)[1] or os.write(self.fd, packet) != len(packet):
            raise RuntimeError('RasAdapter UART write incomplete')

    def read_position(self, channel, timeout=.5):
        if type(channel) is not int or not 1 <= channel <= 6:
            raise ValueError('RasAdapter5A PWM channels are numbered 1..6')
        self.send(4, bytes((5, channel)))  # manufacturer PWM read-position command
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for function, payload in self.receive(max(0, deadline - time.monotonic())):
                if function == 4 and len(payload) == 4 and payload[:2] == bytes((channel, 5)):
                    return struct.unpack('<H', payload[2:])[0]
        raise RuntimeError('RasAdapter PWM position readback timed out')

    def set_position(self, channel, pulse, seconds=.02):
        if (type(channel) is not int or not 1 <= channel <= 6
                or type(pulse) is not int or not 1550 <= pulse <= 1750
                or type(seconds) not in (int, float) or not math.isfinite(seconds)
                or not .02 <= seconds <= .5):
            raise ValueError('only conservative, observed steering points are allowed')
        duration = int(seconds * 1000)
        self.send(4, struct.pack('<BHBBH', 1, duration, 1, channel, pulse))

    def set_esc(self, channel, pulse):
        if type(channel) is not int or channel != 4 or type(pulse) is not int or pulse not in (1500, 1575, 1300):
            raise ValueError('only the identified S4 ESC and previously observed short-trial points are allowed')
        self.send(4, struct.pack('<BHBBH', 1, 20, 1, channel, pulse))

    def set_gimbal_reference(self, channel, pulse, seconds=.1):
        if (type(channel) is not int or channel not in (1, 2)
                or type(pulse) is not int or not 1500 <= pulse <= 1550
                or type(seconds) not in (int, float) or not math.isfinite(seconds)
                or not .02 <= seconds <= .5):
            raise ValueError('only S1/S2 and the previously observed 1500..1550 us reference range are allowed')
        self.send(4, struct.pack('<BHBBH', 1, int(seconds * 1000), 1, channel, pulse))

    def align_tilt_step(self, previous, pulse):
        """One small S2 alignment step, separate from a calibrated gimbal API.

        The finite alignment tool additionally limits each reviewed run to
        50 us, requires stored-command continuity and saves real-camera frames.
        Neither an angle nor a safe mechanical endpoint is inferred here.
        """
        if (type(previous) is not int or type(pulse) is not int
                or not 1000 <= previous <= 1550 or not 1000 <= pulse <= 1550
                or not 0 < previous - pulse <= 10):
            raise ValueError('only a downward S2 alignment step of at most 10 us is allowed')
        if self.read_position(2) != previous:
            raise RuntimeError('tilt stored command changed during alignment')
        self.send(4, struct.pack('<BHBBH', 1, 100, 1, 2, pulse))

    def align_pan_step(self, previous, pulse):
        """One operator-guided S1 step; this window is not endpoint calibration."""
        if (type(previous) is not int or type(pulse) is not int
                or not 1450 <= previous <= 1750 or not 1450 <= pulse <= 1750
                or not 0 < abs(previous-pulse) <= 10):
            raise ValueError('only a small S1 alignment step is allowed')
        if self.read_position(1) != previous:
            raise RuntimeError('pan stored command changed during alignment')
        self.send(4, struct.pack('<BHBBH', 1, 100, 1, 1, pulse))

    def close(self):
        if self.fd is not None:
            import termios
            try:
                if self.original is not None:
                    # Restore baud only after the final neutral/center frame
                    # has left the UART, otherwise its last bytes can corrupt.
                    termios.tcdrain(self.fd)
                    termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)
            finally:
                os.close(self.fd)
                self.fd = None
