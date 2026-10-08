"""Pi-only race executor. Default is replay without opening hardware PWM."""

import json
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from .pi_control import PiActuator, SpeedFeedbackController, load_pi_config
from .race import Observation, Phase, RaceController, load_race_config
from .race_workflows import json_line, read_observations


class Announcement:
    def __init__(self, config, expected_text, run):
        self.config, self.run = config, run
        self.process = None
        self.started = False
        self.done = False
        if run:
            if not expected_text or config.announcement_text != expected_text:
                raise ValueError("recorded announcement text must match the configured school/team sentence")
            if not config.announcement_wav or not Path(config.announcement_wav).is_file():
                raise ValueError("a verified school/team announcement WAV is required")
            import shutil
            if not shutil.which('aplay'):
                raise RuntimeError("aplay is required to play the local announcement")

    def update(self, events):
        if self.run and not self.started and any(e['event'] == 'play_announcement' for e in events):
            self.process = subprocess.Popen(['aplay', '--quiet', self.config.announcement_wav], stdin=subprocess.DEVNULL,
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.started = True
        if self.process is not None and self.process.poll() is not None:
            if self.process.returncode != 0:
                raise RuntimeError("announcement playback failed; crosswalk task cannot be acknowledged")
            self.done = True

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1)


def live_observations(stop_event):
    latest = queue.Queue(maxsize=1)

    def publish(item):
        try:
            latest.get_nowait()
        except queue.Empty:
            pass
        latest.put_nowait(item)

    def read():
        try:
            for line in sys.stdin:
                if stop_event.is_set():
                    break
                if line.strip():
                    publish(('observation', Observation.from_dict(json.loads(line))))
            publish(('eof', None))
        except Exception as exc:
            publish(('error', exc))

    threading.Thread(target=read, daemon=True, name='live-race-observations').start()
    while not stop_event.is_set():
        try:
            kind, value = latest.get(timeout=.02)
        except queue.Empty:
            yield None
            continue
        if kind == 'error':
            raise value
        if kind == 'eof':
            break
        yield value


def pi_run(args):
    backend = getattr(args, 'backend', 'hardware-pwm')
    if backend == 'rasadapter5':
        from .pi_runtime_bridge import pi_bridge_run
        return pi_bridge_run(args)
    if backend != 'hardware-pwm':
        raise ValueError('unknown race execution backend')
    if not getattr(args, 'pi_config', None):
        raise ValueError('--pi-config is required for the hardware-pwm backend')
    race_config = load_race_config(args.race_config)
    pi_config = load_pi_config(args.pi_config)
    if args.run and (args.input != '-' or not args.acknowledge_motion):
        raise ValueError("real PWM requires live stdin, --run and --acknowledge-motion; file replay never drives hardware")
    if args.run:
        pi_config.validate(require_ready=True)
        if race_config.vehicle_width_m is None:
            raise ValueError("measured vehicle width is required for real cone/parking control")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    controller = RaceController(race_config)
    actuator = PiActuator(pi_config, run=args.run)
    speed = SpeedFeedbackController(pi_config)
    audio = None
    stop_event = threading.Event()
    saved_handlers = {}
    count = 0
    failure = None
    cleanup_errors = []
    try:
        audio = Announcement(pi_config, race_config.announcement, args.run)
        for name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
            signum = getattr(signal, name, None)
            if signum is not None:
                saved_handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, lambda _signum, _frame: stop_event.set())
        actuator.open()
        rows = live_observations(stop_event) if args.run else read_observations(args.input)
        last_received = time.monotonic()
        timeout_logged = False
        with (output / 'pi-decisions.jsonl').open('w', encoding='utf-8') as log:
            for observation in rows:
                if stop_event.is_set():
                    break
                if observation is None:
                    if time.monotonic() - last_received >= pi_config.command_lease_s:
                        actuator.neutral()
                        if not timeout_logged:
                            json_line(log, {"hardware_output": args.run, "action": "stop", "reason": "live_input_timeout"})
                            timeout_logged = True
                    if time.monotonic() - last_received > race_config.max_stationary_s:
                        raise TimeoutError("live observations absent for more than 20 seconds")
                    continue
                last_received = time.monotonic()
                timeout_logged = False
                if args.run and abs(last_received - observation.t_s) > pi_config.command_lease_s:
                    raise ValueError("live observation timestamp must use this Pi's monotonic clock and be fresh")
                if args.run:
                    # Only successful local audio playback acknowledges speech.
                    audio.update([])
                    observation.announcement_done = audio.done
                intent = controller.update(observation)
                audio.update(intent['events'])
                throttle, steering = speed.command(intent, observation)
                if intent['action'] == 'stop':
                    actuator.neutral()
                pulses = actuator.apply(throttle, steering)
                json_line(log, {"t_s": observation.t_s, "intent": intent, "actuator": pulses})
                count += 1
                if args.run and controller.phase in (Phase.COMPLETE, Phase.FAILED, Phase.ESTOP):
                    break
            actuator.neutral()
            json_line(log, {"hardware_output": args.run, "action": "stop", "reason": "input_ended_or_stop_requested"})
    except BaseException as exc:
        failure = exc
        with (output / 'pi-decisions.jsonl').open('a', encoding='utf-8') as log:
            json_line(log, {"hardware_output": args.run, "action": "stop",
                            "reason": "input_error_or_interruption", "error": f"{type(exc).__name__}: {exc}"})
    finally:
        requested_stop = stop_event.is_set()
        try:
            cleanup_errors.extend(actuator.close())
        except Exception as exc:
            cleanup_errors.append(f"PWM close: {exc}")
        if audio:
            try:
                audio.close()
            except Exception as exc:
                cleanup_errors.append(f"audio close: {exc}")
        stop_event.set()
        for signum, handler in saved_handlers.items():
            signal.signal(signum, handler)
        summary = {**controller.summary(), "hardware_output": args.run, "observations": count,
                   "cleanup_errors": cleanup_errors, "stop_requested": requested_stop,
                   "error": None if failure is None else f"{type(failure).__name__}: {failure}"}
        (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    if failure:
        raise failure
    if cleanup_errors:
        raise RuntimeError('PWM cleanup failed: ' + '; '.join(cleanup_errors))
    return summary
