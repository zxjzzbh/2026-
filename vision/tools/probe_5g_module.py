"""Read-only RM500U-CNV / camera snapshot; never sends dialing or GPIO commands.

Use the USB interface number rather than ttyUSB numbering: a CH340 or another
serial device can take ttyUSB0 before the modem appears. No SIM identifiers are
requested. Internet and remote-control reachability require separate checks.
"""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time
from urllib.request import urlopen


QUERIES = (
    'AT+CGMM', 'AT+CGMR', 'AT+CPIN?', 'AT+QUIMSLOT?',
    'AT+QSIMSTAT?', 'AT+CFUN?', 'AT+COPS?', 'AT+CEREG?',
    'AT+C5GREG?', 'AT+CGATT?', 'AT+CSQ', 'AT+QCFG="usbnet"',
    'AT+QCFG="nat"', 'AT+QNETDEVCTL?',
)


def read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def command(argv):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=5)
        return result.stdout.strip() if result.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def modem_ports():
    ports = []
    for tty in sorted(Path('/sys/class/tty').glob('ttyUSB*')):
        device = (tty / 'device').resolve()
        interface = next((p for p in (device, *device.parents)
                          if (p / 'bInterfaceNumber').exists()), None)
        usb = next((p for p in (device, *device.parents)
                    if (p / 'idVendor').exists()), None)
        if usb and interface and read(usb / 'idVendor') == '2c7c' and read(usb / 'idProduct') == '0900':
            ports.append({'device': '/dev/' + tty.name,
                          'interface': read(interface / 'bInterfaceNumber'),
                          'product': read(usb / 'product')})
    return ports


def at_snapshot(port):
    import fcntl
    import select
    import termios

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        original = termios.tcgetattr(fd)
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        responses = {}
        for query in QUERIES:
            termios.tcflush(fd, termios.TCIFLUSH)
            os.write(fd, (query + '\r').encode('ascii'))
            deadline = time.monotonic() + 2
            buffer = b''
            while time.monotonic() < deadline:
                if select.select([fd], [], [], .1)[0]:
                    chunk = os.read(fd, 8192)
                    if not chunk:
                        raise OSError('modem disconnected while reading')
                    buffer += chunk
                    if re.search(rb'\r\n(?:OK|ERROR|\+CME ERROR:[^\r]*)\r\n', buffer):
                        break
                    if len(buffer) > 32768:
                        raise OSError('modem response exceeded snapshot limit')
            lines = [line.strip() for line in buffer.decode(errors='replace').splitlines()
                     if line.strip() and line.strip() != query]
            responses[query] = [re.sub(r'\d{14,}', '[REDACTED]', line) for line in lines]
        return responses
    finally:
        try:
            if original is not None:
                termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            os.close(fd)


def registration(responses, query):
    text = '\n'.join(responses.get(query, []))
    match = re.search(r'\+(?:CEREG|C5GREG):\s*\d+,(\d+)', text)
    return int(match.group(1)) if match else None


def camera_status(camera):
    try:
        with urlopen('http://127.0.0.1:8080/api/status?camera=' + camera, timeout=3) as response:
            status = json.load(response)
        identity = status.get('camera_identity') or {}
        return {'state': status.get('state'), 'error': status.get('error'),
                'name': identity.get('name'), 'bus': identity.get('bus'),
                'frame_age_ms': status.get('host_frame_age_ms'),
                'preview_fps': status.get('preview_fps')}
    except (OSError, ValueError) as exc:
        return {'state': 'unreachable', 'error': str(exc)}


def snapshot():
    ports = modem_ports()
    matches = [p for p in ports if p['interface'] == '04' and p['product'] == 'RM500U-CNV']
    responses, error = {}, None
    if len(matches) == 1:
        try:
            responses = at_snapshot(matches[0]['device'])
        except (OSError, ValueError) as exc:
            error = str(exc)
    else:
        error = 'expected exactly one RM500U-CNV AT interface 04'
    pin = '\n'.join(responses.get('AT+CPIN?', []))
    sim = ('ready' if '+CPIN: READY' in pin else
           'not_detected' if '+CME ERROR: 10' in pin else
           'pin_required' if '+CPIN: SIM PIN' in pin else 'unknown')
    lte, nr = registration(responses, 'AT+CEREG?'), registration(responses, 'AT+C5GREG?')
    address_output = command(['ip', '-j', '-4', 'address', 'show', 'dev', 'usb0'])
    addresses = [a['local'] for link in json.loads(address_output or '[]')
                 for a in link.get('addr_info', []) if a.get('scope') == 'global']
    return {
        'schema_version': 1,
        'boot_id': read('/proc/sys/kernel/random/boot_id'),
        'uptime_s': float((read('/proc/uptime') or '0').split()[0]),
        'power_flags': command(['vcgencmd', 'get_throttled']),
        'modem': {'model': 'RM500U-CNV' if matches else None,
                  'at_port': matches[0]['device'] if len(matches) == 1 else None,
                  'usb_ports': ports, 'sim_state': sim, 'lte_registration': lte,
                  'nr5g_registration': nr, 'registered': lte in (1, 5) or nr in (1, 5),
                  'interface': 'usb0', 'carrier': read('/sys/class/net/usb0/carrier'),
                  'ipv4_addresses': addresses, 'responses': responses, 'error': error},
        'cameras': {c: camera_status(c) for c in ('primary', 'secondary')},
        'read_only': True, 'motion_commands_sent': False,
        'cellular_internet_verified': False, 'remote_5g_control_verified': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if os.name != 'posix':
        parser.error('run this snapshot on the Raspberry Pi Linux target')
    result = snapshot()
    content = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content, encoding='utf-8')
    print(content, end='')


if __name__ == '__main__':
    main()
