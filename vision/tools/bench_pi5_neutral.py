"""Read-only Pi 5 inspection by default; explicit finite neutral-only trial."""
import argparse
import json
from pathlib import Path
import signal
import time

from carvision.pi5_pwm import Pi5BenchBackend, inspect_pi5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--review', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--seconds', type=int, default=90)
    args = parser.parse_args()
    audit = inspect_pi5()
    if not args.run:
        print(json.dumps(audit, indent=2))
        return
    if not args.review or not args.output or args.output.exists() or not 10 <= args.seconds <= 120:
        parser.error('run requires fresh review, new output folder and a 10..120-second duration')
    args.output.mkdir(parents=True)
    review = json.loads(args.review.read_text())
    backend = Pi5BenchBackend(args.root.resolve(), args.output, audit['boot_id'], review, neutral_only=True)
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    try:
        backend.open()
        route = 'RasAdapter S4' if review.get('esc_backend') == 'rasadapter5a_uart' else 'BCM13'
        print(f'PI5_NEUTRAL_ACTIVE: {route}=1500us; all servos disabled; no motion commands', flush=True)
        deadline = time.monotonic() + args.seconds
        while not stopping and time.monotonic() < deadline:
            backend.check()
            time.sleep(.02)
    finally:
        backend.close()
        print('PI5_NEUTRAL_FINISHED: switch ESC off', flush=True)


if __name__ == '__main__':
    main()
