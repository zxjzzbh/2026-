"""One race runtime for offline replay and explicitly enabled calibrated Pi output.

Live JSONL must originate on the Pi monotonic clock, with increasing sequence
and capture timestamps. Latest-input buffering bounds lag; any parse failure,
EOF, timeout, signal or output error latches this run off and closes resources.
"""
import json
import queue
import signal
import sys
import threading
import time
from dataclasses import asdict, replace
from pathlib import Path

from .autonomy_audio import make_announcement
from .autonomy_motion import AutonomousS3S4Driver, MetricMotionController
from .autonomy_observation import DualCameraObservationBuilder
from .autonomy_profile import AutonomyProfile, fit_ground_calibration, number
from .race import Observation, Phase, RaceController, load_race_config
from .race_workflows import demonstration, json_line


def validate_envelope(envelope, *, live, now, previous_sequence=None, previous_capture=None):
    if not number(now) or now < 0:
        raise ValueError("invalid local monotonic clock")
    if not isinstance(envelope, dict) or envelope.get("schema_version") != 1 or envelope.get("mode") != ("live" if live else "replay"):
        raise ValueError("input must be a matching versioned live/replay envelope")
    sequence, captured = envelope.get("sequence"), envelope.get("captured_monotonic_s")
    if type(sequence) is not int or sequence < 0 or not number(captured) or captured < 0:
        raise ValueError("invalid input sequence/capture timestamp")
    if previous_sequence is not None and (sequence <= previous_sequence or captured <= previous_capture):
        raise ValueError("duplicate or reversed input timestamp/sequence")
    if live and not 0 <= now - captured <= .25:
        raise ValueError("live observation is stale or from the future")
    obs = Observation.from_dict(envelope.get("observation"))
    if abs(obs.t_s - captured) > 1e-6:
        raise ValueError("observation and envelope clock mismatch")
    oldest = envelope.get("source_oldest_monotonic_s")
    if live and obs.source_ok and (not number(oldest) or not 0 <= now - oldest <= .25):
        raise ValueError("camera measurement expired while queued")
    if live and obs.telemetry_valid and (obs.telemetry_age_s is None or not 0 <= obs.telemetry_age_s + now - captured <= .25):
        raise ValueError("measured speed feedback expired")
    return obs, sequence, captured


class AutonomyRuntime:
    def __init__(self, race_config, profile, *, run=False, live=False, clock=time.monotonic, driver=None, audio=None):
        if run and not live:
            raise ValueError("real autonomous output only accepts a live local stream")
        if run and profile.missing():
            raise ValueError("autonomous output blocked: " + ", ".join(profile.missing()))
        if run and race_config.vehicle_width_m != profile.data["vehicle"]["width_m"]:
            raise ValueError("race and measured vehicle widths disagree")
        self.controller = RaceController(race_config)
        self.profile, self.run, self.live, self.clock = profile, run, live, clock
        self.motion = MetricMotionController(profile)
        if driver is not None and getattr(driver, "run", None) is not run:
            raise ValueError("injected driver must match runtime dry/real mode")
        if audio is not None and getattr(audio, "simulated", None) is not (not run):
            raise ValueError("injected announcement must match runtime dry/real mode")
        self.driver = driver if driver is not None else AutonomousS3S4Driver(profile, run=run, clock=clock)
        self.audio = audio if audio is not None else make_announcement(profile, race_config.announcement, run)
        self.sequence = self.last_capture = None
        self.fault = None
        self.closed = False
        self.count = self.planned_motions = self.hardware_motions = 0
        self.cleanup_errors = []
        if not run:
            self.driver.open()

    def step(self, envelope):
        if self.fault or self.closed:
            raise RuntimeError("autonomous runtime is closed or fault-latched")
        try:
            obs, sequence, captured = validate_envelope(envelope, live=self.live, now=self.clock(),
                                                       previous_sequence=self.sequence, previous_capture=self.last_capture)
            self.sequence, self.last_capture = sequence, captured
            if hasattr(self.audio, "poll"):
                self.audio.poll()
            audio_status = self.audio.status()
            if audio_status["failed"]:
                raise RuntimeError("announcement failed: " + str(audio_status.get("error")))
            # The runtime owns audio acknowledgment, including in replay.
            obs.announcement_done = audio_status["done"]
            decision = self.controller.update(obs)
            for event in decision.get("events", []):
                if event.get("event") == "play_announcement":
                    self.audio.start(event["text"])
            plan = self.motion.plan(decision, obs)
            if decision["phase"] in (Phase.ESTOP.value, Phase.FAILED.value):
                self.trip(decision.get("reason", "terminal_fault"))
                execution = {"hardware_output": self.run and self.driver.opened, "motion_output": False, "fault": self.fault}
            else:
                # The manual 5G stage retains its separate driver. Acquire the
                # exclusive UART only after verified stationary zone handover.
                if self.run and not self.driver.opened and decision["phase"] != Phase.REMOTE.value:
                    if not obs.in_switch_zone or not obs.enable_autonomy or not obs.telemetry_valid or obs.speed_mps is None or abs(obs.speed_mps) > self.controller.config.stopped_speed_mps:
                        raise RuntimeError("stationary handover required before UART ownership")
                    self.driver.open()
                execution = self.driver.apply(plan) if self.driver.opened else {"hardware_output": False, "motion_output": False, "reason": "manual_stage_separate_driver"}
            self.count += 1
            self.planned_motions += bool(plan["motion_planned"])
            self.hardware_motions += bool(execution.get("motion_output"))
            return {"sequence": sequence, "t_s": obs.t_s, "simulated": not self.live,
                    "decision": decision, "execution": execution, "motion_plan": plan,
                    "audio": self.audio.status(), "measurement_sources": envelope.get("measurement_sources", {}),
                    "issues": envelope.get("issues", []), "runtime_fault": self.fault}
        except BaseException as exc:
            self.trip(f"{type(exc).__name__}: {exc}")
            raise

    def trip(self, reason):
        self.fault = self.fault or reason
        self.driver.trip(reason)

    def close(self, reason="input_ended"):
        if self.closed:
            return list(self.cleanup_errors)
        self.trip(reason)
        self.closed = True
        try:
            if hasattr(self.audio, "close"):
                self.audio.close()
        except Exception as exc:
            self.cleanup_errors.append("audio: " + str(exc))
        finally:
            self.cleanup_errors.extend(self.driver.close())
        return list(self.cleanup_errors)

    def summary(self):
        return {**self.controller.summary(), "simulated": not self.live,
                "hardware_output": bool(self.run), "observations": self.count,
                "planned_motion_updates": self.planned_motions, "hardware_motion_updates": self.hardware_motions,
                "decision_flow_completed": self.controller.phase == Phase.COMPLETE,
                "actuation_mapping_ready": not self.profile.missing(),
                "physical_race_completed": False, "runtime_fault_or_end_reason": self.fault,
                "cleanup_errors": list(self.cleanup_errors), "readiness": self.profile.report()}


class LatestInput:
    """Bounded latest row plus a separate fatal channel that cannot be overwritten."""
    def __init__(self, stream, *, clock=time.monotonic):
        self.latest = queue.Queue(maxsize=1)
        self.fatal = queue.Queue(maxsize=1)
        self.ended = threading.Event()
        self.clock = clock
        self.thread = threading.Thread(target=self._read, args=(stream,), daemon=True, name="autonomy-local-jsonl")
        self.thread.start()

    def _read(self, stream):
        sequence = captured = None
        try:
            for line in stream:
                if len(line) > 262144 or not line.strip():
                    raise ValueError("empty or oversized live JSONL row")
                value = json.loads(line, parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
                # Validate every row before dropping an older valid row from
                # the latest slot. A future/duplicate/invalid row must not be
                # hidden by a later good row.
                _obs, sequence, captured = validate_envelope(value, live=True, now=self.clock(),
                                                            previous_sequence=sequence, previous_capture=captured)
                try:
                    self.latest.get_nowait()
                except queue.Empty:
                    pass
                self.latest.put_nowait(value)
        except BaseException as exc:
            self.fatal.put(exc)
        finally:
            self.ended.set()

    def get(self, timeout=.02):
        try:
            exc = self.fatal.get_nowait()
        except queue.Empty:
            exc = None
        if exc:
            raise exc
        if self.ended.is_set():
            raise EOFError("local sensor stream ended")
        try:
            return self.latest.get(timeout=timeout)
        except queue.Empty:
            if self.ended.is_set():
                raise EOFError("local sensor stream ended")
            return None


def replay_envelope(obs, sequence):
    return {"schema_version": 1, "mode": "replay", "sequence": sequence,
            "captured_monotonic_s": obs.t_s, "observation": asdict(obs)}


def run_to_directory(runtime, rows, output):
    output = Path(output)
    try:
        output.mkdir(parents=True, exist_ok=False)
        (output / "configuration.json").write_text(json.dumps({"race": asdict(runtime.controller.config), "profile": runtime.profile.data, "live": runtime.live, "run": runtime.run}, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    except BaseException:
        runtime.close("log_directory_unavailable")
        raise
    error = None
    original_signals = {}
    def interrupted(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")
    if threading.current_thread() is threading.main_thread():
        for name in ("SIGTERM", "SIGHUP", "SIGINT"):
            signum = getattr(signal, name, None)
            if signum is not None:
                original_signals[signum] = signal.getsignal(signum)
                signal.signal(signum, interrupted)
    try:
        with (output / "decisions.jsonl").open("w", encoding="utf-8") as log:
            last_input = runtime.clock()
            for row in rows:
                if row is None:
                    if runtime.clock() - last_input >= .25:
                        raise TimeoutError("local observation gap reached 250 ms")
                    continue
                last_input = runtime.clock()
                json_line(log, runtime.step(row))
                if runtime.fault:
                    break
    except BaseException as exc:
        error = exc
    finally:
        runtime.close("input_ended" if error is None else f"{type(error).__name__}: {error}")
        summary = runtime.summary()
        summary["output"] = str(output)
        if error:
            summary["error"] = f"{type(error).__name__}: {error}"
        (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        for signum, handler in original_signals.items():
            signal.signal(signum, handler)
    if error and not isinstance(error, EOFError):
        raise error
    if runtime.cleanup_errors:
        raise RuntimeError("cleanup errors: " + "; ".join(runtime.cleanup_errors))
    return summary


def _live_rows(stream):
    reader = LatestInput(stream)
    while True:
        yield reader.get()


def autonomy_demo(args):
    config = load_race_config(args.race_config)
    config = replace(config, school=config.school or "山东大学", team=config.team or "启航队", vehicle_width_m=config.vehicle_width_m or .19)
    profile = AutonomyProfile.load(args.profile)
    runtime = AutonomyRuntime(config, profile)
    try:
        return run_to_directory(runtime, (replay_envelope(obs, i) for i, obs in enumerate(demonstration())), args.output)
    finally:
        runtime.close("demo_exit")


def autonomy_run(args):
    live = args.input == "-"
    if args.run and (not live or not args.acknowledge_motion):
        raise ValueError("real mode requires live stdin and explicit prepared-motion acknowledgment")
    profile = AutonomyProfile.load(args.profile)
    runtime = AutonomyRuntime(load_race_config(args.race_config), profile, run=args.run, live=live)
    try:
        if live:
            return run_to_directory(runtime, _live_rows(sys.stdin), args.output)
        with Path(args.input).open(encoding="utf-8-sig") as stream:
            return run_to_directory(runtime, (json.loads(line) for line in stream if line.strip()), args.output)
    finally:
        runtime.close("command_exit")


def autonomy_observe(args):
    builder = DualCameraObservationBuilder(AutonomyProfile.load(args.profile))
    for line in sys.stdin:
        json_line(sys.stdout, builder.build(json.loads(line), time.monotonic()))
    return 0


def ground_calibrate(args):
    result = fit_ground_calibration(json.loads(Path(args.input).read_text(encoding="utf-8-sig")))
    with Path(args.output).open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return result
