"""Nonblocking real WAV/TTS playback with explicit completion and deadlines.

TTS uses the existing verified Hiwonder write format. Busy/idle bytes must be
measured on this particular module, never guessed from another chip's manual.
No completion comes from a timer alone or an observation's success flag.
"""
import hashlib
import os
import subprocess
import time
from pathlib import Path

from .announcement_backend import AnnouncementBackend
from .autonomy_profile import positive


MAX_TTS_MESSAGE_BYTES = 64  # Application bound for the short race announcement.


def tts_frames(text, volume=3):
    if not isinstance(text, str) or not text.strip() or type(volume) is not int or not 0 <= volume <= 10:
        raise ValueError("TTS requires nonempty GB2312 text and volume 0..10")
    prefix = f"[h0][v{volume}]".encode("gb2312")
    content = prefix + text.encode("gb2312")
    length = len(content) + 2
    packet = bytes([0, 0xFD, length >> 8, length & 255, 1, 0]) + content
    if len(packet) > MAX_TTS_MESSAGE_BYTES:
        raise ValueError("TTS sentence exceeds the 64-byte single-message application limit")
    # This backend uses Linux raw I2C, whose complete messages are not SMBus
    # blocks. Preserve the whole sentence in one synthesis command; splitting
    # at the 32-byte SMBus block limit introduces audible pauses.
    return [packet]


class I2CTTS:
    def __init__(self, bus=1, address=0x40):
        if type(bus) is not int or bus < 0 or address != 0x40:
            raise ValueError("reviewed TTS is I2C 0x40 on a nonnegative bus")
        self.bus, self.address, self.fd = bus, address, None

    def open(self):
        import fcntl
        self.fd = os.open(f"/dev/i2c-{self.bus}", os.O_RDWR)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, 0x0703, self.address)
        except BaseException:
            self.close()
            raise

    def status(self):
        value = os.read(self.fd, 1)
        if len(value) != 1:
            raise OSError("incomplete TTS status")
        return value[0]

    def send(self, packet):
        if len(packet) > MAX_TTS_MESSAGE_BYTES:
            raise ValueError("TTS packet exceeds the single-message application limit")
        if os.write(self.fd, packet) != len(packet):
            raise OSError("incomplete TTS packet")

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class RealAnnouncement:
    simulated = False

    def __init__(self, config, expected_text, *, transport=None, clock=time.monotonic, popen=subprocess.Popen):
        if config.get("verified") is not True or not expected_text:
            raise ValueError("audio hardware and exact sentence must be verified before real playback")
        timeout = config.get("timeout_s")
        if not positive(timeout) or timeout > 60:
            raise ValueError("audio timeout must be in (0,60]")
        self.config, self.text, self.clock, self.popen = config, expected_text, clock, popen
        self.transport, self.process = transport, None
        self.started = self.done = self.failed = False
        self.error = None
        self.deadline = self.sent_at = None
        self.seen_busy = False
        self.frames = []
        self.frame_index = 0
        self.next_poll = 0
        if config.get("backend") == "tts":
            busy, idle = config.get("busy_status"), config.get("idle_status")
            if type(busy) is not int or type(idle) is not int or not 0 <= busy <= 255 or not 0 <= idle <= 255 or busy == idle:
                raise ValueError("TTS requires distinct measured busy/idle bytes")
            self.frames = tts_frames(expected_text, config.get("volume", 3))
        elif config.get("backend") == "wav":
            path = Path(config.get("wav_path") or "")
            if not path.is_file() or not config.get("wav_sha256") or hashlib.sha256(path.read_bytes()).hexdigest() != config["wav_sha256"]:
                raise ValueError("exact verified WAV file/hash required")
        else:
            raise ValueError("real audio backend must be wav or tts")

    def start(self, text):
        if text != self.text:
            raise ValueError("announcement sentence mismatch")
        if self.started:
            return False
        self.started = True
        self.deadline = self.clock() + self.config["timeout_s"]
        try:
            if self.config["backend"] == "wav":
                self.process = self.popen(["aplay", "--quiet", self.config["wav_path"]], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                if self.transport is None:
                    self.transport = I2CTTS(self.config.get("bus", 1), self.config.get("address", 64))
                self.transport.open()
                if self.transport.status() != self.config["idle_status"]:
                    raise RuntimeError("TTS was not idle before announcement")
                self._send_next()
        except BaseException as exc:
            self._fail(exc)
            raise
        return True

    def _send_next(self):
        self.transport.send(self.frames[self.frame_index])
        self.sent_at = self.clock()
        self.seen_busy = False

    def _fail(self, error):
        self.failed, self.error = True, str(error)
        self.close()

    def poll(self):
        if not self.started or self.failed or self.done:
            return self.status()
        try:
            now = self.clock()
            if now >= self.deadline:
                raise TimeoutError("announcement completion timeout")
            if self.process is not None:
                code = self.process.poll()
                if code is not None:
                    if code != 0:
                        raise RuntimeError(f"audio player exited {code}")
                    self.done = True
            elif now >= self.next_poll:
                self.next_poll = now + .05  # do not hammer the I2C bus
                status = self.transport.status()
                if status == self.config["busy_status"]:
                    self.seen_busy = True
                elif status == self.config["idle_status"]:
                    if self.seen_busy:
                        self.frame_index += 1
                        if self.frame_index == len(self.frames):
                            self.done = True
                            self.transport.close()
                        else:
                            self._send_next()
                    elif now - self.sent_at > .75:
                        raise RuntimeError("TTS never acknowledged busy; completion unverified")
                else:
                    raise RuntimeError(f"unrecognized TTS status byte {status}")
        except Exception as exc:
            self._fail(exc)
        return self.status()

    def status(self):
        return {"started": self.started, "done": self.done, "failed": self.failed,
                "simulated": False, "hardware_verified": self.config["verified"], "error": self.error}

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1)
        if self.transport is not None:
            self.transport.close()


def make_announcement(profile, text, run=False):
    if run:
        return RealAnnouncement(profile.data["audio"], text)
    backend = AnnouncementBackend(simulated=True)
    backend.text = text
    return backend
