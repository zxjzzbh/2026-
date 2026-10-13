"""Local simulated PWM dashboard; no camera, UART, GPIO or vehicle connection."""
import argparse
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'vision/src'))

import cv2
import numpy as np
from carvision.config import Config
from carvision.manual_drive import ManualDrive, SimulationBackend
from carvision.pipeline import Pipeline
from carvision.sources import Frame
from carvision.web_preview import PreviewState, make_server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8931)
    parser.add_argument('--seconds', type=int, default=600, help='bounded demo server lifetime')
    args = parser.parse_args()
    if not 1 <= args.seconds <= 3600 or not 1024 <= args.port <= 65535:
        parser.error('demo lifetime 1..3600 seconds; port 1024..65535')
    backend = SimulationBackend()
    backend.motor_available = False
    backend.steering_pwm_available = True
    backend.steering_center_us = 1715  # UI example, never applied as real calibration
    backend.real_output_paused_reason = '电脑模拟演示：没有连接车辆或摄像头。'
    drive = ManualDrive(backend, settling_s=1, session_s=120, preparation_s=600)
    state = PreviewState()
    state.identity = {'name': 'SIMULATION - no camera'}
    server = make_server(('127.0.0.1', args.port), state, drive=drive)
    server.timeout = .1
    stopped = threading.Event()

    def frames():
        image = np.full((480, 640, 3), (32, 25, 18), dtype=np.uint8)
        cv2.putText(image, 'SIMULATION / NO HARDWARE', (30, 210), cv2.FONT_HERSHEY_SIMPLEX, .8, (220, 190, 120), 2)
        cv2.putText(image, 'S3 PWM UI CHECK', (30, 260), cv2.FONT_HERSHEY_SIMPLEX, .7, (180, 180, 180), 2)
        jpeg = cv2.imencode('.jpg', image)[1].tobytes()
        pipeline, count = Pipeline(Config()), 0
        while not stopped.wait(.1):
            frame = Frame(image, count, None, time.monotonic() * 1000)
            result, _ = pipeline.process(frame, live=True)
            state.publish({'raw': jpeg, 'processed': jpeg, 'mask': jpeg}, result.to_dict())
            state.publish_raw(jpeg, frame)
            count += 1

    drive.start()
    thread = threading.Thread(target=frames, daemon=True)
    thread.start()
    print(f'SIMULATION ONLY: http://127.0.0.1:{args.port} (Ctrl+C to close)', flush=True)
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        state.finish()
        drive.close()
        server.server_close()
        thread.join(timeout=2)


if __name__ == '__main__':
    main()
