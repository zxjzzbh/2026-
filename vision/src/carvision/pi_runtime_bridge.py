"""RasAdapter runtime: fresh newest live input and ordered, hardware-free replay.

The current bridge has no continuous motion mapping. Real mode is unavailable.
"""
import json
import signal
import sys
import threading
import time
from dataclasses import asdict, replace
from pathlib import Path

from .announcement_backend import AnnouncementBackend
from .race import Observation, Phase, RaceController, finite_number, load_race_config
from .race_executor_bridge import RaceExecutorBridge
from .race_workflows import json_line


def _now(clock):
    value = clock()
    if not finite_number(value) or value < 0:
        raise ValueError("clock must return finite local monotonic seconds")
    return value


def _fresh_timestamp(timestamp, now, limit):
    if timestamp > now:
        raise ValueError("observation timestamp is in the future")
    if now - timestamp > limit:
        raise ValueError("observation timestamp is stale")


class LiveObservationPump:
    """One-slot inbox; errors and EOF cannot be overwritten by newer data."""
    def __init__(self, stream, *, clock=time.monotonic, max_input_age_s=.25):
        if not finite_number(max_input_age_s) or not 0 < max_input_age_s <= .25:
            raise ValueError("max_input_age_s must be in (0, 0.25]")
        self.stream, self.clock, self.max_input_age_s = stream, clock, max_input_age_s
        self.latest = None
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.done_event = threading.Event()
        self.error = None
        self.thread = None
        self.eof = False
        self.last_t = None

    def start(self):
        if self.thread is not None:
            raise RuntimeError("observation pump is single-use")

        def read():
            try:
                for line in self.stream:
                    if self.stop_event.is_set():
                        break
                    if not line.strip():
                        continue
                    observation = Observation.from_dict(json.loads(line))
                    arrived = _now(self.clock)
                    _fresh_timestamp(observation.t_s, arrived, self.max_input_age_s)
                    if self.last_t is not None and observation.t_s <= self.last_t:
                        raise ValueError("observation timestamps must strictly increase")
                    self.last_t = observation.t_s
                    with self.condition:
                        self.latest = (arrived, observation)
                        self.condition.notify_all()
                with self.condition:
                    self.eof = True
            except BaseException as exc:
                with self.condition:
                    self.error = exc
            finally:
                with self.condition:
                    self.done_event.set()
                    self.condition.notify_all()

        self.thread = threading.Thread(target=read, daemon=True, name="race-observation-reader")
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        # Caller-owned stdin is not closed. A blocked reader remains daemonized
        # until its stream unblocks; stop prevents it from publishing new data.
        if self.thread and self.thread.is_alive() and self.thread is not threading.current_thread():
            self.thread.join(timeout=.02)

    def is_finished(self):
        return self.done_event.is_set()

    def next(self, timeout):
        with self.condition:
            self.condition.wait_for(
                lambda: self.error is not None or self.latest is not None
                or self.eof or self.stop_event.is_set(), timeout=timeout)
            if self.error is not None:
                raise self.error
            item, self.latest = self.latest, None
            return item


class RaceRuntime:
    """Bounded logs, local audio acknowledgment and fail-closed cleanup."""
    def __init__(self, controller: RaceController, *, bridge=None, announcement=None,
                 run=False, clock=time.monotonic, max_input_age_s=.25, on_row=None):
        if type(run) is not bool:
            raise ValueError("run must be boolean")
        if not finite_number(max_input_age_s) or not 0 < max_input_age_s <= .25:
            raise ValueError("max_input_age_s must be in (0, 0.25]")
        if on_row is not None and not callable(on_row):
            raise ValueError("on_row must be callable")
        self.controller = controller
        self.bridge = bridge if bridge is not None else RaceExecutorBridge()
        self.announcement = announcement if announcement is not None else AnnouncementBackend(simulated=not run)
        self.run, self.clock, self.max_input_age_s = run, clock, max_input_age_s
        if run:
            if self.announcement.simulated or not self.announcement.status()["hardware_verified"]:
                raise ValueError("real runtime requires a verified non-simulated announcement backend")
            raise ValueError("real RasAdapter race runtime unavailable: continuous speed/reverse mapping is not calibrated")
        if self.bridge.executor.run:
            raise ValueError("dry runtime cannot use a hardware-enabled execution bridge")
        self.on_row = on_row
        self.last_observation_t = None
        self.failure = None
        self.cleanup_errors = []
        self.rows = []
        self.observations = 0
        self._old_handlers = {}
        self.used = False
        self.stop_requested = False
        self.reader_stopped = True

    def _install_signals(self, stop_event):
        for name in ("SIGINT", "SIGTERM", "SIGHUP"):
            signum = getattr(signal, name, None)
            if signum is not None:
                previous = signal.getsignal(signum)
                signal.signal(signum, lambda *_: stop_event.set())
                self._old_handlers[signum] = previous

    def _restore_signals(self):
        for signum, handler in self._old_handlers.items():
            try:
                signal.signal(signum, handler)
            except Exception as exc:
                self.cleanup_errors.append(f"restore signal: {exc}")
        self._old_handlers.clear()

    def _record(self, row):
        row = {"hardware_output": False, "announcement_simulated": self.announcement.simulated, **row}
        self.rows.append(row)
        if len(self.rows) > 256:
            del self.rows[0]
        if self.on_row:
            self.on_row(row)

    def _audio_status(self):
        status = self.announcement.status()
        if status["failed"]:
            raise RuntimeError("announcement failed; crosswalk cannot be released")
        if status["done"] and not status["started"]:
            raise RuntimeError("announcement completion before start is invalid")
        return status

    def _process(self, observation):
        observation = Observation.from_dict(asdict(observation))
        if self.last_observation_t is not None and observation.t_s <= self.last_observation_t:
            raise ValueError("observation timestamps must strictly increase")
        # Ignore input claims of speaker completion; refresh local ack before
        # every new controller decision, including when the backend is pending.
        observation = replace(observation, announcement_done=self._audio_status()["done"])
        self.last_observation_t = observation.t_s
        intent = self.controller.update(observation)
        for event in intent["events"]:
            if event["event"] == "play_announcement":
                self.announcement.start(event["text"])
        self._audio_status()
        result = self.bridge.apply(intent)
        self.observations += 1
        self._record({"observation_t_s": observation.t_s, "intent": intent, "execution": result})
        return result

    def _consume(self, arrived, observation):
        now = _now(self.clock)
        if not finite_number(arrived) or arrived > now:
            raise ValueError("arrival must use the local monotonic clock")
        if now - arrived > self.max_input_age_s:
            result = self.bridge.input_timeout()
            self._record({"execution": result})
            return result
        _fresh_timestamp(observation.t_s, now, self.max_input_age_s)
        return self._process(observation)

    def summary(self, mode):
        return {**self.controller.summary(), "backend": "rasadapter5", "input_mode": mode,
                "hardware_output": False, "actuation_mapping_ready": False,
                "decision_flow_completed": self.controller.phase == Phase.COMPLETE,
                "announcement_simulated": self.announcement.simulated,
                "observations": self.observations, "rows": list(self.rows),
                "stop_requested": self.stop_requested, "reader_stopped": self.reader_stopped,
                "cleanup_errors": list(self.cleanup_errors),
                "error": None if self.failure is None else f"{type(self.failure).__name__}: {self.failure}"}

    def _execute(self, mode, work, *, pump=None, stop_event=None, install_signals=True):
        if self.used:
            raise RuntimeError("race runtime is single-use")
        self.used = True
        event = stop_event if stop_event is not None else threading.Event()
        try:
            if install_signals:
                self._install_signals(event)
            self.bridge.open()
            work(event)
        except BaseException as exc:
            self.failure = exc
            try:
                result = self.bridge.input_error()
                self._record({"execution": result, "error": f"{type(exc).__name__}: {exc}"})
            except BaseException as stop_error:
                self.cleanup_errors.append(f"error handling: {stop_error}")
            raise
        finally:
            if pump:
                try:
                    pump.stop()
                except Exception as exc:
                    self.cleanup_errors.append(f"reader stop: {exc}")
                finally:
                    self.reader_stopped = pump.thread is None or not pump.thread.is_alive()
            try:
                self.cleanup_errors.extend(self.bridge.close())
            except Exception as exc:
                self.cleanup_errors.append(f"bridge close: {exc}")
            try:
                close_audio = getattr(self.announcement, "close", None)
                if close_audio:
                    close_audio()
            except Exception as exc:
                self.cleanup_errors.append(f"audio close: {exc}")
            self._restore_signals()
            if self.failure is None and self.cleanup_errors:
                self.failure = RuntimeError("runtime cleanup failed: " + "; ".join(self.cleanup_errors))
                raise self.failure
        return self.summary(mode)

    def _notify_stop(self, event):
        self.stop_requested = event.is_set()
        self._record({"execution": self.bridge.stop_requested()})

    def run_stream(self, stream, *, idle_timeout_s=None, stop_event=None, install_signals=True):
        timeout = self.max_input_age_s if idle_timeout_s is None else idle_timeout_s
        if not finite_number(timeout) or not 0 < timeout <= self.max_input_age_s:
            raise ValueError("idle timeout must be positive and no greater than input protection")
        pump = LiveObservationPump(stream, clock=self.clock, max_input_age_s=self.max_input_age_s)

        def work(event):
            last_arrival = _now(self.clock)
            pump.start()
            while True:
                if event.is_set():
                    self._notify_stop(event)
                    break
                wait_s = min(.02, max(0, timeout - (_now(self.clock) - last_arrival)))
                item = pump.next(wait_s)
                if event.is_set():
                    self._notify_stop(event)
                    break
                if item is None:
                    if pump.is_finished() and pump.eof:
                        self._record({"execution": self.bridge.input_ended()})
                        break
                    if _now(self.clock) - last_arrival >= timeout:
                        self._record({"execution": self.bridge.input_timeout()})
                        break
                    continue
                last_arrival = item[0]
                result = self._consume(*item)
                if result["reason"] == "live_input_timeout" or self.controller.phase in (Phase.ESTOP, Phase.FAILED, Phase.COMPLETE):
                    break

        return self._execute("live", work, pump=pump, stop_event=stop_event, install_signals=install_signals)

    def run_replay(self, rows, *, stop_event=None, install_signals=True):
        """Ordered historical replay; it cannot open hardware."""
        if self.run:
            raise ValueError("file replay cannot drive hardware")

        def work(event):
            for row in rows:
                if event.is_set():
                    self._notify_stop(event)
                    return
                if isinstance(row, str):
                    if not row.strip():
                        continue
                    row = json.loads(row)
                observation = row if isinstance(row, Observation) else Observation.from_dict(row)
                self._process(observation)
            self._record({"execution": self.bridge.input_ended()})

        return self._execute("replay", work, stop_event=stop_event, install_signals=install_signals)


def pi_bridge_run(args):
    """Explicit RasAdapter backend for the existing pi-run CLI."""
    config = load_race_config(args.race_config)
    if args.run:
        raise ValueError("real RasAdapter autonomous motion is unavailable until speed mapping and hardware audio are verified")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    runtime = None
    mode = "live" if args.input == "-" else "replay"
    try:
        with (output / "pi-decisions.jsonl").open("w", encoding="utf-8") as log:
            runtime = RaceRuntime(RaceController(config), on_row=lambda row: json_line(log, row))
            try:
                if mode == "live":
                    result = runtime.run_stream(sys.stdin)
                else:
                    with Path(args.input).open(encoding="utf-8-sig") as stream:
                        result = runtime.run_replay(stream)
            except BaseException as exc:
                if runtime.failure is None:
                    runtime.failure = exc
                    runtime._record({"execution": runtime.bridge.input_error(), "error": f"{type(exc).__name__}: {exc}"})
                raise
            finally:
                if not runtime.used:
                    runtime.bridge.close()
        return {key: value for key, value in result.items() if key != "rows"}
    finally:
        if runtime is not None:
            result = runtime.summary(mode)
            result.pop("rows", None)
            (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
