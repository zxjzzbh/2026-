"""Check a Hiwonder CI1302 voice module using the manufacturer's I2C protocol.

Reads one recognition result by default. Optional playback sends one documented
announcement, never a loop. No GPIO, motor, servo, firmware or word-table writes.
Uses Linux i2c-dev directly so the Pi does not need an extra smbus package.
"""

import argparse
import json
import os
from pathlib import Path


class VoiceModule:
    def __init__(self, bus, address):
        if os.name != "posix":
            raise RuntimeError("Run this check on the Raspberry Pi Linux system")
        if address not in (0x33, 0x34):
            raise ValueError("Only the CI1302 module addresses 0x33/0x34 are supported")
        import fcntl

        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, 0x0703, address)  # I2C_SLAVE, seven-bit address
        except BaseException:
            os.close(self.fd)
            raise

    def __enter__(self):
        return self

    def __exit__(self, *_):
        os.close(self.fd)

    def recognition_result(self):
        # Same register selection and read as the manufacturer's C++ example.
        if os.write(self.fd, bytes([0x64])) != 1:
            raise OSError("Recognition register selection was incomplete")
        result = os.read(self.fd, 1)
        if len(result) != 1:
            raise OSError("No complete recognition result was received")
        return result[0]

    def announce(self, phrase):
        phrase_ids = {"recyclable": 1, "hazardous": 3}
        payload = bytes([0x6E, 0xFF, phrase_ids[phrase]])
        if os.write(self.fd, payload) != len(payload):
            raise OSError("Announcement command was incomplete")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x34,
                        choices=(0x33, 0x34))
    parser.add_argument("--speak", choices=("recyclable", "hazardous"),
                        help="Send one fixed announcement after a successful read")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.bus < 0:
        parser.error("bus must be non-negative")
    report = {
        "model": "Hiwonder CI1302 voice interaction module",
        "bus": args.bus,
        "address": hex(args.address),
        "communication_ok": False,
        "playback_command_sent": False,
        "playback_heard": None,
        "motion_commands_sent": False,
        "read_only": args.speak is None,
    }
    try:
        with VoiceModule(args.bus, args.address) as module:
            report["recognition_result"] = module.recognition_result()
            report["communication_ok"] = True
            if args.speak:
                module.announce(args.speak)
                report["playback_command_sent"] = True
                report["phrase"] = args.speak
    except (OSError, RuntimeError, ValueError) as exc:
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
