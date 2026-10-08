"""Probe a temporary Pi4 daemon with ALL GPIO update permissions disabled.

Only version and input-mode queries are sent; no servo or wave command. The
root-owned timeout ends the daemon after five seconds. Requires cached sudo.
"""

import argparse
import json
import socket
import subprocess
import time
from pathlib import Path

from carvision.gimbal import LocalPigpio


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if b'Raspberry Pi 4 Model B' not in Path('/proc/device-tree/model').read_bytes():
        raise RuntimeError('this driver probe is only for the current Raspberry Pi 4B')
    args.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[2]
    driver=root/'drivers/pigpio'
    before=subprocess.check_output(['pinctrl','get','12,13,17,27'],text=True)
    client=LocalPigpio(8890)
    try:
        with socket.create_connection(('127.0.0.1',8890),timeout=.2):
            raise RuntimeError('port 8890 is already occupied; no existing service was replaced')
    except (ConnectionRefusedError,TimeoutError):
        pass
    command=['sudo','-n','timeout','--signal=TERM','5s','env',f'LD_LIBRARY_PATH={driver}',
             str(driver/'pigpiod'),'-g','-l','-f','-n','127.0.0.1','-t','1','-x','0','-p','8890']
    version=None
    modes={}
    with (args.output/'driver.log').open('w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+4
        try:
            while time.monotonic()<deadline and process.poll() is None:
                try:
                    client.open()
                    version=client.version()
                    modes={str(gpio):client.mode(gpio) for gpio in (12,13,17,27)}
                    break
                except (OSError,ConnectionError):
                    client.close()
                    time.sleep(.05)
        finally:
            client.close()
            exit_code=process.wait(timeout=8)
    after=subprocess.check_output(['pinctrl','get','12,13,17,27'],text=True)
    result={'version':version,'gpio_update_permission_mask':0,'servo_commands_sent':0,
            'wave_commands_sent':0,'input_modes':modes,'pins_preserved':before==after,
            'before':before,'after':after,'daemon_timeout_exit':exit_code,
            'compatible_without_output':version is not None and before==after and all(v==0 for v in modes.values())}
    (args.output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
    if not result['compatible_without_output']:
        raise RuntimeError('driver startup was not verified; inspect driver.log')


if __name__=='__main__':
    main()
