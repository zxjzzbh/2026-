"""Exclusive RasAdapter5A S3/S4 transport. Construction sends nothing."""
import os
import select
import struct
import time


def packet(steering, throttle):
    if type(steering) is not int or not 1350 <= steering <= 1870:
        raise ValueError('转向必须在 1350–1870 μs 内')
    if type(throttle) is not int or not (throttle in (1400, 1500) or 1570 <= throttle <= 1600):
        raise ValueError('前进油门必须在 1570–1600 μs 内；1500 为中位，1400 为制动')
    data = bytes((4, 10)) + struct.pack('<BHBBHBH', 1, 20, 2, 3, steering, 4, throttle)
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8c if crc & 1 else 0)
    return b'\xaa\x55' + data + bytes((crc,))


class Hardware:
    def __init__(self, port='/dev/ttyAMA0'):
        self.port = port
        self.fd = self.owner_fd = None

    def open(self):
        import fcntl
        import termios
        if self.fd is not None:
            raise RuntimeError('执行器已打开')
        try:
            self.owner_fd = os.open('/tmp/carvision-pi-pwm.lock',
                                    os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
            fcntl.flock(self.owner_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            state = termios.tcgetattr(self.fd)
            state[0] = state[1] = state[3] = 0
            state[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            state[4] = state[5] = termios.B1000000
            state[6][termios.VMIN] = state[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, state)
        except BaseException:
            self.close()
            raise

    def write(self, steering, throttle):
        data = packet(steering, throttle)
        if self.fd is None:
            raise RuntimeError('执行器未打开')
        deadline = time.monotonic() + .15
        while data:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise RuntimeError('串口输出未完成')
            try:
                count = os.write(self.fd, data)
            except BlockingIOError:
                continue
            if count <= 0:
                raise RuntimeError('串口输出中断')
            data = data[count:]

    def close(self):
        for name in ('fd', 'owner_fd'):
            value = getattr(self, name)
            setattr(self, name, None)
            if value is not None:
                os.close(value)
