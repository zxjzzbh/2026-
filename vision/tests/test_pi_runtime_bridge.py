import io
import json
import threading
import time
from dataclasses import replace

import pytest

from carvision.announcement_backend import AnnouncementBackend
from carvision.pi_runtime_bridge import LiveObservationPump, RaceRuntime
from carvision.race import Observation, Phase, RaceConfig, RaceController
from carvision.race_executor_bridge import RaceExecutorBridge
from carvision.race_workflows import demonstration
from carvision.rasadapter5_executor import RasAdapterExecutionConfig, RasAdapterS3S4Executor


def observation(t_s=1.0):
    return Observation(
        t_s=t_s,
        source_ok=True,
        telemetry_valid=True,
        telemetry_age_s=0,
        speed_mps=0,
        in_switch_zone=True,
    )


def line(obs):
    return json.dumps(obs.__dict__, ensure_ascii=False) + "\n"


def test_real_runtime_rejects_simulated_or_unverified_audio_before_open():
    with pytest.raises(ValueError, match="announcement"):
        RaceRuntime(
            RaceController(RaceConfig()),
            run=True,
            announcement=AnnouncementBackend(),
        )


def test_live_runtime_eof_notifies_bridge_and_cleans_up():
    runtime = RaceRuntime(RaceController(RaceConfig()), clock=lambda: 1.0)
    result = runtime.run_stream(io.StringIO(line(observation())))
    assert result["error"] is None
    assert any(row.get("execution", {}).get("reason") == "input_ended"
               for row in result["rows"])
    assert runtime.bridge.closed


def test_duplicate_timestamp_fails_and_still_cleans_up():
    class SlowStream(io.StringIO):
        def __iter__(self):
            yield line(observation(1.0))
            import time
            time.sleep(.05)
            yield line(observation(1.0))

    runtime = RaceRuntime(RaceController(RaceConfig()), clock=lambda: 1.0)
    with pytest.raises(ValueError, match="strictly increase"):
        runtime.run_stream(SlowStream(""))
    assert runtime.bridge.closed


def race_controller():
    return RaceController(RaceConfig(school="山东大学", team="启航队"))


def test_fresh_arrival_does_not_make_old_observation_fresh():
    runtime = RaceRuntime(race_controller(), clock=lambda: 2.0)
    with pytest.raises(ValueError, match="stale"):
        runtime._consume(2.0, observation(1.0))


def test_input_audio_success_cannot_acknowledge_local_announcement():
    controller = race_controller()
    controller.phase = Phase.CROSSWALK_HOLD
    runtime = RaceRuntime(controller, clock=lambda: 2.0,
                          announcement=AnnouncementBackend(result=None))
    row = replace(observation(2.0), task_monitor_valid=True,
                  crosswalk_distance_m=.2, announcement_done=True)
    runtime._consume(2.0, row)
    assert not controller.announcement_acknowledged
    assert controller.phase == Phase.CROSSWALK_HOLD


def test_local_audio_completion_is_applied_before_each_new_decision():
    controller = race_controller()
    controller.phase = Phase.CROSSWALK_HOLD
    now = [0.0]
    runtime = RaceRuntime(controller, clock=lambda: now[0])
    for tick in range(102):
        now[0] = round(tick/10, 1)
        row = replace(observation(now[0]), task_monitor_valid=True,
                      crosswalk_distance_m=.2, announcement_done=False)
        runtime._consume(now[0], row)
    assert controller.phase == Phase.LIGHT_APPROACH
    assert controller.announcement_acknowledged


def test_cleanup_failure_is_reported_instead_of_success():
    class FailedCleanup(RaceExecutorBridge):
        def close(self):
            super().close()
            return ["close failed"]

    runtime = RaceRuntime(race_controller(), bridge=FailedCleanup(), clock=lambda: 1.0)
    with pytest.raises(RuntimeError, match="close failed"):
        runtime.run_stream(io.StringIO(line(observation())))


def test_signal_setup_failure_still_cleans_up(monkeypatch):
    def fail_setup(*args):
        raise ValueError("signal setup failed")

    monkeypatch.setattr("carvision.pi_runtime_bridge.signal.signal", fail_setup)
    runtime = RaceRuntime(race_controller())
    with pytest.raises(ValueError, match="signal setup failed"):
        runtime.run_stream(io.StringIO(""))
    assert runtime.bridge.closed


def test_idle_timeout_cannot_exceed_input_protection():
    runtime = RaceRuntime(race_controller())
    with pytest.raises(ValueError, match="timeout"):
        runtime.run_stream(io.StringIO(""), idle_timeout_s=.26)


def test_latest_queue_does_not_hide_a_parse_error():
    pump = LiveObservationPump(io.StringIO(line(observation()) + "{broken}\n"), clock=lambda: 1.0)
    try:
        pump.start()
        assert pump.done_event.wait(.5)
        with pytest.raises(ValueError):
            pump.next(0)
    finally:
        pump.stop()


def test_latest_queue_does_not_hide_duplicate_timestamps():
    pump = LiveObservationPump(io.StringIO(line(observation()) * 2), clock=lambda: 1.0)
    try:
        pump.start()
        assert pump.done_event.wait(.5)
        with pytest.raises(ValueError, match="strictly increase"):
            pump.next(0)
    finally:
        pump.stop()


@pytest.mark.parametrize("timestamp", [.74, 1.01])
def test_pump_rejects_old_or_future_measurements_at_arrival(timestamp):
    pump = LiveObservationPump(io.StringIO(line(observation(timestamp))), clock=lambda: 1.0)
    try:
        pump.start()
        assert pump.done_event.wait(.5)
        with pytest.raises(ValueError):
            pump.next(0)
    finally:
        pump.stop()


def test_pump_consumes_newest_observation_once_and_preserves_eof():
    stream = io.StringIO(''.join(line(observation(t)) for t in (.8, .9, 1.0)))
    pump = LiveObservationPump(stream, clock=lambda: 1.0)
    try:
        pump.start()
        assert pump.done_event.wait(.5)
        assert pump.next(0)[1].t_s == 1.0
        assert pump.next(0) is None
        assert pump.eof
    finally:
        pump.stop()
    assert not pump.thread.is_alive()


def test_queue_delay_latches_timeout_even_if_input_was_fresh_at_arrival():
    runtime = RaceRuntime(race_controller(), clock=lambda: 1.3)
    result = runtime._consume(1.0, observation(1.0))
    assert result["reason"] == "live_input_timeout"
    assert runtime.bridge.locked


def test_live_idle_timeout_stops_with_no_new_lines():
    release = threading.Event()

    class BlockedStream:
        def __iter__(self):
            release.wait(.5)
            if False:
                yield ""

    runtime = RaceRuntime(race_controller())
    started = time.monotonic()
    try:
        result = runtime.run_stream(BlockedStream(), idle_timeout_s=.04)
        assert time.monotonic() - started < .4
        assert result["rows"][-1]["execution"]["reason"] == "live_input_timeout"
        assert runtime.bridge.closed and runtime.bridge.locked
        assert result["reader_stopped"] is False  # blocked stdin is not forcibly closed
    finally:
        release.set()


def test_signal_handler_sets_stop_and_restores_originals(monkeypatch):
    handlers, restored = {}, []
    initial = object()

    def fake_signal(number, handler):
        if handler is initial:
            restored.append(number)
        else:
            handlers[number] = handler

    monkeypatch.setattr("carvision.pi_runtime_bridge.signal.getsignal", lambda _: initial)
    monkeypatch.setattr("carvision.pi_runtime_bridge.signal.signal", fake_signal)

    class InterruptedStream:
        def __iter__(self):
            handlers[next(iter(handlers))](None, None)
            if False:
                yield ""

    runtime = RaceRuntime(race_controller())
    result = runtime.run_stream(InterruptedStream())
    assert result["stop_requested"]
    assert result["rows"][-1]["execution"]["reason"] == "stop_requested"
    assert set(restored) == set(handlers)
    assert runtime.bridge.closed and runtime.bridge.locked


def test_malformed_json_stops_closes_and_retains_primary_error():
    runtime = RaceRuntime(race_controller(), clock=lambda: 1.0)
    with pytest.raises(ValueError):
        runtime.run_stream(io.StringIO('{broken}\n'))
    assert runtime.bridge.closed and runtime.bridge.locked
    assert runtime.failure is not None
    assert runtime.rows[-1]["execution"]["reason"] == "input_error"


def test_reader_start_failure_still_closes_bridge(monkeypatch):
    def failed_start(self):
        raise OSError("reader start failed")

    monkeypatch.setattr("carvision.pi_runtime_bridge.threading.Thread.start", failed_start)
    runtime = RaceRuntime(race_controller())
    with pytest.raises(OSError, match="reader start failed"):
        runtime.run_stream(io.StringIO(""))
    assert runtime.bridge.closed and runtime.bridge.locked


def test_reader_stop_failure_does_not_skip_bridge_cleanup(monkeypatch):
    def failed_stop(self):
        raise OSError("reader stop failed")

    monkeypatch.setattr("carvision.pi_runtime_bridge.LiveObservationPump.stop", failed_stop)
    runtime = RaceRuntime(race_controller())
    with pytest.raises(RuntimeError, match="reader stop failed"):
        runtime.run_stream(io.StringIO(""))
    assert runtime.bridge.closed


def test_real_bridge_cannot_be_injected_into_dry_runtime():
    config = RasAdapterExecutionConfig(
        steering_left_us=1750, steering_center_us=1650, steering_right_us=1550,
        esc_neutral_us=1500, esc_forward_us=1575, esc_reverse_us=1300,
        calibration_confirmed=True)
    executor = RasAdapterS3S4Executor(config, run=True)
    with pytest.raises(ValueError, match="dry runtime"):
        RaceRuntime(race_controller(), bridge=RaceExecutorBridge(executor))
    assert executor.adapter.fd is None and not executor.opened


def test_real_runtime_cannot_bypass_missing_motion_mapping_with_claimed_audio():
    class ClaimedAudio:
        simulated = False
        def status(self):
            return {"hardware_verified": True}

    with pytest.raises(ValueError, match="mapping"):
        RaceRuntime(race_controller(), run=True, announcement=ClaimedAudio())


@pytest.mark.parametrize("limit", [True, float("nan"), float("inf"), .251, 0, "0.25"])
def test_invalid_input_age_limit_rejected(limit):
    with pytest.raises(ValueError):
        RaceRuntime(race_controller(), max_input_age_s=limit)


@pytest.mark.parametrize("complete", [None, False])
def test_runtime_does_not_accept_external_audio_success_when_local_pending_or_failed(complete):
    controller = race_controller()
    controller.phase = Phase.CROSSWALK_HOLD
    runtime = RaceRuntime(controller, announcement=AnnouncementBackend(result=complete))
    rows = [replace(observation(round(t/10,1)), task_monitor_valid=True,
                    crosswalk_distance_m=.2, announcement_done=True) for t in range(102)]
    if complete is False:
        with pytest.raises(RuntimeError, match="announcement failed"):
            runtime.run_replay(rows)
    else:
        runtime.run_replay(rows)
    assert controller.phase == Phase.CROSSWALK_HOLD
    assert not controller.announcement_acknowledged
    assert runtime.bridge.closed


def test_primary_exception_is_not_masked_by_cleanup_failure():
    class FailedCleanup(RaceExecutorBridge):
        def close(self):
            super().close()
            return ["release failed"]

    runtime = RaceRuntime(race_controller(), bridge=FailedCleanup())
    with pytest.raises(ValueError):
        runtime.run_replay(['{broken}'])
    assert "release failed" in runtime.cleanup_errors
    assert isinstance(runtime.failure, ValueError)


def test_failed_log_callback_stops_and_cleans_up():
    def failed_log(row):
        raise OSError("disk full")

    runtime = RaceRuntime(race_controller(), on_row=failed_log)
    with pytest.raises(OSError, match="disk full"):
        runtime.run_replay([observation()])
    assert runtime.bridge.closed and runtime.bridge.locked


def test_runtime_history_is_bounded_and_instance_cannot_restart():
    runtime = RaceRuntime(race_controller())
    result = runtime.run_replay([observation(float(t)) for t in range(300)])
    assert result["observations"] == 300
    assert len(result["rows"]) == 256
    with pytest.raises(RuntimeError, match="single-use"):
        runtime.run_replay([])


@pytest.mark.parametrize("clear_slot", ["left", "right"])
def test_ordered_runtime_replay_preserves_all_synthetic_decisions_without_hardware(clear_slot, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("replay touched hardware")

    monkeypatch.setattr("carvision.rasadapter5_executor.RasAdapter", forbidden)
    controller = RaceController(RaceConfig(school="山东大学", team="启航队", vehicle_width_m=.19))
    rows = list(demonstration())
    if clear_slot == "left":
        for row in rows:
            row.cones = [{**c, "lateral_m": -c["lateral_m"]} for c in row.cones]
            row.parking_slots = [{**s, "availability": "clear" if s["id"] == "left" else "blocked"}
                                 for s in row.parking_slots]
            if row.parked_slot_id:
                row.parked_slot_id = "left"
    runtime = RaceRuntime(controller)
    result = runtime.run_replay(rows)
    assert result["observations"] == 221
    assert result["decision_flow_completed"]
    assert result["parking_slot_id"] == clear_slot
    assert result["payment_bonus_points"] == 5
    assert not result["hardware_output"] and not result["actuation_mapping_ready"]
    assert result["announcement_simulated"]
    assert all(row["execution"]["action"] == "stop" for row in result["rows"])
    assert runtime.bridge.executor.commands == []
    releases = [row for row in result["rows"] if row.get("intent", {}).get("reason") ==
                "crosswalk_configured_hold_completed"]
    assert len(releases) == 1
    assert releases[0]["observation_t_s"] - controller.crosswalk_since >= 10
