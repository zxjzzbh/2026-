"""One additional forward point for a user-requested, finite load trial.

Neutral, cancellation, refresh and cleanup are inherited from the unchanged
DMA backend with physical stop evidence. This is not a full throttle range.
"""

import math
import struct

from .esc_wave import DmaEsc


class LoadProbeEsc(DmaEsc):
    forward_pulse_us = 1600

    def prepare(self):
        super().prepare()
        with self.lock:
            self.command(53)
            self.command(28, data=struct.pack('<6I', 1 << 13, 0, 1600,
                                             0, 1 << 13, 18400))
            wave_id = self.command(49)
            if not 0 <= wave_id <= 250:
                raise RuntimeError('load probe wave ID is not encodable')
            self.waves[1600] = wave_id

    def start_brief_command(self, pulse, seconds):
        if pulse != 1600:
            return super().start_brief_command(pulse, seconds)
        if (type(pulse) is not int or type(seconds) not in (int, float)
                or not math.isfinite(seconds) or not .02 <= seconds <= .8):
            raise ValueError('load probe requires a finite 1600-us short command')
        if not self.claimed:
            raise RuntimeError('ESC output is not prepared')
        with self.lock:
            cycles = max(1, int(seconds*50))
            self._cancel()
            chain = self._block(1600, cycles) + self._block(1500, self.neutral_cycles)
            self.command(93, data=chain)
            self.refresh_at = self.clock()+cycles/50+45
