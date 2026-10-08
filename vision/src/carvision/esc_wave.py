"""BCM13-only DMA pulses for the observed, finite raised-wheel ESC trials.

Every motion chain includes its own finite neutral tail. Python scheduling
does not determine either the pulse width or the end of the motion stage.
The caller must own the loopback pigpio daemon and its independent timeout.
"""

import math
import struct
import threading
import time

from .gimbal import LocalPigpio


class DmaEsc(LocalPigpio):
    signal_backend = 'dma_finite_wave'
    neutral_cycles = 3000

    def __init__(self, port, *, clock=time.monotonic, sleep=time.sleep):
        super().__init__(port)
        self.clock, self.sleep = clock, sleep
        self.lock = threading.RLock()
        self.waves = {}
        self.claimed = False
        self.refresh_at = None

    def command(self, command, p1=0, p2=0, data=b''):
        with self.lock:
            self.connection.sendall(struct.pack('<4I', command, p1, p2, len(data)) + data)
            reply = b''
            while len(reply) < 16:
                part = self.connection.recv(16-len(reply))
                if not part:
                    raise ConnectionError('DMA ESC driver disconnected')
                reply += part
            result = struct.unpack('<3Ii', reply)[3]
            if result < 0:
                raise RuntimeError(f'DMA ESC command {command} failed: {result}')
            return result

    def prepare(self):
        if self.mode(13) != 0:
            raise RuntimeError('ESC signal already owned')
        # The daemon is private to this worker; never clear another owner's waves.
        self.command(27)  # WVCLR
        for width in (1500, 1575, 1300):
            self.command(53)  # WVNEW
            self.command(28, data=struct.pack('<6I', 1 << 13, 0, width,
                                             0, 1 << 13, 20000-width))
            wave_id = self.command(49)
            if not 0 <= wave_id <= 250:
                raise RuntimeError('wave ID cannot be encoded in a finite chain')
            self.waves[width] = wave_id
        self.command(0, 13, 1)
        self.claimed = True
        self.neutral()

    def _block(self, width, cycles):
        if type(cycles) is not int or not 1 <= cycles <= self.neutral_cycles:
            raise ValueError('finite wave cycle count required')
        return bytes((255, 0, self.waves[width], 255, 1, cycles & 255, cycles >> 8))

    def _cancel(self):
        # Finish a high pulse before cancelling its chain. Truncating a neutral
        # pulse could otherwise look like a throttle command to the ESC.
        deadline = self.clock()+.008
        while self.command(3, 13) == 1 and self.clock() < deadline:
            self.sleep(.0001)
        self.command(33)
        self.command(4, 13, 0)
        # A cancelled high pulse must not merge with the next neutral pulse.
        self.sleep(.04)

    def neutral(self):
        if not self.claimed:
            return
        with self.lock:
            self._cancel()
            self.command(93, data=self._block(1500, self.neutral_cycles))
            self.refresh_at = self.clock()+45

    def start_brief_command(self, pulse, seconds):
        if (type(pulse) is not int or pulse not in (1575, 1300)
                or type(seconds) not in (int, float) or not math.isfinite(seconds)
                or not .02 <= seconds <= .8):
            raise ValueError('only observed finite ESC motion is allowed')
        if not self.claimed:
            raise RuntimeError('ESC output is not prepared')
        with self.lock:
            # Round down so the DMA stage cannot exceed the requested limit.
            cycles = max(1, int(seconds*50))
            self._cancel()
            chain = self._block(pulse, cycles) + self._block(1500, self.neutral_cycles)
            self.command(93, data=chain)
            self.refresh_at = self.clock()+cycles/50+45

    def check(self):
        if self.claimed:
            with self.lock:
                if self.command(32) != 1:
                    raise RuntimeError('finite ESC signal unexpectedly ended')
                if self.clock() >= self.refresh_at:
                    self.neutral()

    def shutdown(self):
        errors = []
        if self.connection is None:
            return errors
        try:
            if self.claimed:
                self._cancel()
                self.command(93, data=self._block(1500, 20))
                self.sleep(.42)
        except Exception as exc:
            errors.append(str(exc))
        # Try each cleanup operation even if restoring neutral failed.
        for command, p1, p2 in ((33, 0, 0), (4, 13, 0), (0, 13, 0), (27, 0, 0)):
            try:
                self.command(command, p1, p2)
            except Exception as exc:
                errors.append(str(exc))
        self.claimed = False
        self.close()
        return errors
