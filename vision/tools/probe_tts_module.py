"""Check and speak through the Hiwonder 0x40 text-to-speech module.

Protocol source: Hiwonder/TonyPi, HiwonderSDK/hiwonder/TTS.py.
Use I2C bus 1 at 40000 Hz (or a temporary slower diagnostic bus).
This module synthesizes supplied text; it does not recognize spoken commands.
The default operation reads one status byte. --text sends one short phrase.
"""

import argparse
import json
import os
import time
from pathlib import Path


class TTSModule:
    def __init__(self, bus=1):
        import fcntl

        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, 0x0703, 0x40)
        except BaseException:
            os.close(self.fd)
            raise

    def __enter__(self):
        return self

    def __exit__(self, *_):
        os.close(self.fd)

    def status(self):
        data = os.read(self.fd, 1)
        if len(data) != 1:
            raise OSError("Incomplete TTS status response")
        return data[0]

    def speak(self, text, volume=3):
        if not isinstance(volume, int) or not 0 <= volume <= 10:
            raise ValueError("volume must be an integer from 0 to 10")
        if not text.strip():
            raise ValueError("text must not be empty")
        content = f"[h0][v{volume}]".encode("gb2312") + text.encode("gb2312")
        length = len(content) + 2
        frame = bytes([0xFD, length >> 8, length & 0xFF, 0x01, 0x00]) + content
        if len(frame) > 32:
            raise ValueError("Phrase exceeds the manufacturer's 32-byte I2C block limit")
        # write_i2c_block_data(0x40, 0, frame) sends command byte 0, then frame.
        # I2C block mode has no extra SMBus length byte on the wire.
        payload = bytes([0]) + frame
        if os.write(self.fd, payload) != len(payload):
            raise OSError("Incomplete TTS playback command")
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--text", help="One short GB2312 phrase; no repeated playback")
    parser.add_argument("--volume", type=int, choices=range(0, 11), default=3)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.bus < 0:
        parser.error("bus must be non-negative")
    report = {
        "protocol": "Hiwonder text-to-speech (TTS)",
        "bus": args.bus,
        "address": "0x40",
        "speech_recognition_supported": False,
        "communication_ok": False,
        "playback_command_sent": False,
        "playback_heard": None,
        "motion_commands_sent": False,
    }
    try:
        with TTSModule(args.bus) as module:
            report["status_raw_hex"] = hex(module.status())
            report["communication_ok"] = True
            if args.text:
                module.speak(args.text, args.volume)
                report.update(playback_command_sent=True, text=args.text, volume=args.volume)
                report["status_after_hex"] = hex(module.status())
    except (OSError, ValueError, UnicodeError, ImportError) as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["errno"] = getattr(exc, "errno", None)
    content = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content, encoding="utf-8")
    print(content, end="")
    return 0 if report["communication_ok"] and "error" not in report else 1


if __name__ == "__main__":
    raise SystemExit(main())
