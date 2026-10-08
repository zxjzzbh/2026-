"""Finite S1 or S2 reference test; never sends steering or ESC commands."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import time
from urllib.request import urlopen

from carvision.rasadapter5 import RasAdapter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--channel', type=int, choices=(1, 2), required=True)
    parser.add_argument('--review', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    review = json.loads(args.review.read_text())
    assert review['boot_id'] == boot and review['user_esc_off_confirmed']
    assert review['user_gimbal_free_confirmed'] and review['user_cable_slack_confirmed']
    assert review['previous_observed_reference_range_us'] == [1500, 1550]
    args.output.mkdir(parents=True, exist_ok=False)
    deadline = time.monotonic()+10

    def get(path):
        with urlopen('http://127.0.0.1:8080'+path, timeout=1) as response:
            return response.read()

    def paused():
        status = json.loads(get('/api/drive/status'))
        assert status['controls_paused'] and status['hardware_output'] is False
        assert time.monotonic() < deadline

    def capture(stage):
        paused()
        for camera in ('primary', 'secondary'):
            (args.output/f'{stage}-{camera}.jpg').write_bytes(get('/frame.jpg?camera='+camera+'&view=raw'))

    def terminate(*_):
        raise SystemExit('finite test interrupted')

    signal.signal(signal.SIGTERM, terminate)
    board = RasAdapter()
    rows = []
    restored = False
    reference_started = False
    report = {'boot_id': boot, 'channel': args.channel, 'gimbal_hardware_output': True,
              'chassis_commands_sent': False, 'reference_is_calibrated_center': False,
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'commands': rows}
    try:
        paused()
        board.open()
        before = {str(channel): board.read_position(channel) for channel in (1, 2, 3, 4)}
        report['stored_commands_before'] = before
        assert before[str(args.channel)] == 0 or 1500 <= before[str(args.channel)] <= 1550
        capture('before')

        def command(pulse, duration=.1):
            paused()
            board.set_gimbal_reference(args.channel, pulse, duration)
            rows.append({'pulse_us': pulse, 't_monotonic': time.monotonic()})

        command(1500, .3)
        reference_started = True
        time.sleep(.8)
        capture('reference')
        for pulse in (1510, 1520, 1530, 1540, 1550):
            command(pulse)
            time.sleep(.12)
        time.sleep(.6)
        capture('offset')
        for pulse in (1540, 1530, 1520, 1510, 1500):
            command(pulse)
            time.sleep(.12)
        time.sleep(.6)
        capture('returned')
        report['stored_commands_after'] = {str(channel): board.read_position(channel) for channel in (1, 2, 3, 4)}
        assert all(report['stored_commands_after'][str(channel)] == before[str(channel)] for channel in (3, 4))
        report['sequence_completed'] = True
    finally:
        if board.fd is not None:
            try:
                if reference_started:
                    board.set_gimbal_reference(args.channel, 1500, .1)
                    restored = True
            finally:
                board.close()
        report['final_reference_command_sent'] = restored
        report['gimbal_hardware_output'] = reference_started
        report['pwm_output_disabled'] = False
        report['physical_result_pending'] = True
        report['boot_unchanged'] = Path('/proc/sys/kernel/random/boot_id').read_text().strip() == boot
        (args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
