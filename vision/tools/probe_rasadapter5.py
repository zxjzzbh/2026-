"""Read board telemetry and stored PWM commands; never command movement."""
import argparse
import json
from pathlib import Path
import struct
import time

from carvision.rasadapter5 import RasAdapter, SOURCE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyAMA0')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    board = RasAdapter(args.port)
    report = {'model': 'Hiwonder RasAdapter5A V1.0, identified from user photograph',
              'port': args.port, 'baudrate': 1000000, 'source': SOURCE,
              'movement_commands_sent': False, 'pwm_position_is_physical_feedback': False,
              'channels': {}, 'telemetry': []}
    try:
        board.open()
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            for function, payload in board.receive(.1):
                if function == 0 and len(payload) == 3 and payload[0] == 4:
                    report['telemetry'].append({'board_voltage_mv': struct.unpack('<H', payload[1:])[0]})
        for channel in range(1, 7):
            try:
                report['channels'][str(channel)] = {'stored_pulse_us': board.read_position(channel)}
            except RuntimeError as exc:
                report['channels'][str(channel)] = {'error': str(exc)}
    finally:
        board.close()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
