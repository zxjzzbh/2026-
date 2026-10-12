from carvision.announcement_backend import AnnouncementBackend
from carvision.race_executor_bridge import RaceExecutorBridge
from carvision.rasadapter5_executor import RasAdapterExecutionConfig, RasAdapterS3S4Executor


def test_bridge_defaults_to_dry_run_and_rejects_continuous_motion():
    bridge = RaceExecutorBridge()
    bridge.open()
    result = bridge.apply({"action": "follow_lane", "speed_mps": .1})
    assert result["action"] == "stop"
    assert result["reason"] == "continuous_speed_mapping_not_calibrated"
    assert result["actuator"]["hardware_output"] is False


def test_bridge_stop_is_safe_and_estop_stays_locked():
    bridge = RaceExecutorBridge()
    bridge.open()
    result = bridge.apply({"action": "stop", "reason": "emergency_stop_latched"})
    assert result["action"] == "stop"
    bridge.emergency_stop()
    result = bridge.apply({"action": "stop"})
    assert result["reason"] == "executor_locked"


def test_bridge_with_calibrated_fake_executor_still_rejects_continuous_speed():
    config = RasAdapterExecutionConfig(
        steering_left_us=1750,
        steering_center_us=1650,
        steering_right_us=1550,
        esc_neutral_us=1500,
        esc_forward_us=1575,
        esc_reverse_us=1300,
        calibration_confirmed=True,
    )
    executor = RasAdapterS3S4Executor(config)
    bridge = RaceExecutorBridge(executor)
    result = bridge.apply({"action": "follow_corridor", "speed_mps": .1})
    assert result["reason"] == "continuous_speed_mapping_not_calibrated"


def test_announcement_backend_has_simulated_completion_and_explicit_failure():
    text = AnnouncementBackend.text
    simulated = AnnouncementBackend()
    assert simulated.start(text) is True
    assert simulated.status()["done"] is True
    failed = AnnouncementBackend(simulated=False, result=False)
    failed.start(text)
    assert failed.status()["failed"] is True
    assert failed.status()["done"] is False
import pytest
from dataclasses import replace

from carvision.race import Observation, Phase, RaceConfig, RaceController
from carvision.race_workflows import demonstration



def test_real_announcement_cannot_be_verified_by_injected_success_flag():
    with pytest.raises(ValueError, match="real"):
        AnnouncementBackend(simulated=False, result=True)


def test_simulated_failure_is_reported_as_failure():
    backend = AnnouncementBackend(simulated=True, result=False)
    backend.start(backend.text)
    assert backend.status()["failed"] is True
    assert backend.status()["done"] is False
    assert backend.status()["hardware_verified"] is False


@pytest.mark.parametrize("kwargs", [
    {"simulated": "false"}, {"simulated": 1}, {"result": "true"}, {"result": 1},
])
def test_announcement_flags_require_strict_types(kwargs):
    with pytest.raises(ValueError):
        AnnouncementBackend(**kwargs)


def test_invalid_intent_stops_existing_output_and_latches_bridge():
    executor = RasAdapterS3S4Executor(RasAdapterExecutionConfig())
    bridge = RaceExecutorBridge(executor)
    executor.last_command = ("motion", 1575, 1650)
    with pytest.raises(ValueError):
        bridge.apply(None)
    assert executor.last_command == ("neutral",)
    assert bridge.locked


def test_estop_intent_latches_without_separate_emergency_call():
    bridge = RaceExecutorBridge()
    bridge.apply({"action": "stop", "phase": "emergency_stop", "reason": "emergency_stop_latched"})
    assert bridge.locked
    assert bridge.executor.estop_latched
    assert bridge.apply({"action": "follow_lane", "speed_mps": .1})["reason"] == "executor_locked"


@pytest.mark.parametrize("speed", [True, float("nan"), float("inf"), -1, 0, None])
def test_invalid_positive_speed_is_distinguished_from_missing_mapping(speed):
    bridge = RaceExecutorBridge()
    result = bridge.apply({"action": "follow_lane", "speed_mps": speed})
    assert result["reason"] == "invalid_positive_speed"


def test_bridge_cleans_up_when_context_exits_on_exception():
    bridge = RaceExecutorBridge()
    with pytest.raises(LookupError):
        with bridge:
            raise LookupError("input failed")
    assert bridge.executor.closed


class FakeAdapter:
    def __init__(self):
        self.commands = []
        self.fail_neutral = False

    def open(self):
        self.commands.append(("open",))

    def set_position(self, channel, pulse, seconds=.02):
        self.commands.append(("steering", channel, pulse))

    def set_esc(self, channel, pulse):
        self.commands.append(("esc", channel, pulse))
        if self.fail_neutral and pulse == 1500:
            raise OSError("neutral failed")

    def close(self):
        self.commands.append(("close",))


def fake_bridge():
    config = RasAdapterExecutionConfig(
        steering_left_us=1750, steering_center_us=1650, steering_right_us=1550,
        esc_neutral_us=1500, esc_forward_us=1575, esc_reverse_us=1300,
        calibration_confirmed=True)
    adapter = FakeAdapter()
    bridge = RaceExecutorBridge(RasAdapterS3S4Executor(config, run=True, adapter=adapter))
    return bridge, adapter


def test_malformed_intent_stops_a_running_fake_vehicle():
    bridge, adapter = fake_bridge()
    with bridge:
        bridge.executor.apply(1, 0, duration_s=.8)
        marker = len(adapter.commands)
        with pytest.raises(ValueError):
            bridge.apply([])
        assert ("esc", 4, 1500) in adapter.commands[marker:]
        assert bridge.locked and bridge.executor.estop_latched
        assert bridge.apply({"action": "follow_lane", "speed_mps": .1})["reason"] == "executor_locked"
        assert ("esc", 4, 1575) not in adapter.commands[marker:]


@pytest.mark.parametrize("notification,reason", [
    ("input_timeout", "live_input_timeout"), ("input_ended", "input_ended"),
])
def test_runtime_failure_notifications_stop_and_cannot_reopen(notification, reason):
    bridge, adapter = fake_bridge()
    with bridge:
        bridge.executor.apply(1, 0, duration_s=.8)
        marker = len(adapter.commands)
        result = getattr(bridge, notification)()
        assert result["reason"] == reason
        assert result["actuator"] == {"hardware_output": True, "motion_output": False}
        assert ("esc", 4, 1500) in adapter.commands[marker:]
        with pytest.raises(RuntimeError, match="locked"):
            bridge.open()
        assert bridge.apply({"action": "follow_corridor", "speed_mps": .1})["reason"] == "executor_locked"
        assert ("esc", 4, 1575) not in adapter.commands[marker:]


@pytest.mark.parametrize("intent", [
    {"action": "follow_lane", "speed_mps": .1},
    {"action": "follow_corridor", "speed_mps": .25},
    {"action": "remote_intent", "speed_mps": .1},
    {"action": "reverse", "speed_mps": -.1},
    {"action": "stop"},
])
def test_bridge_never_issues_motion_and_reports_actual_neutral_mode(intent):
    bridge, adapter = fake_bridge()
    with bridge:
        result = bridge.apply(intent)
        assert result["action"] == "stop"
        assert result["actuator"]["hardware_output"] is True
        assert result["actuator"]["motion_output"] is False
    assert {c[2] for c in adapter.commands if c[0] == "esc"} == {1500}
    assert adapter.commands[-1] == ("close",)


def test_failed_neutral_is_not_returned_as_success_and_latches_bridge():
    bridge, adapter = fake_bridge()
    bridge.open()
    adapter.fail_neutral = True
    with pytest.raises(RuntimeError, match="neutral failed"):
        bridge.apply({"action": "stop"})
    assert bridge.locked and bridge.error
    assert bridge.close()
    assert adapter.commands[-1] == ("close",)


def test_bridge_cannot_apply_after_close():
    bridge, adapter = fake_bridge()
    bridge.open()
    bridge.close()
    marker = len(adapter.commands)
    with pytest.raises(RuntimeError, match="closed"):
        bridge.apply({"action": "stop"})
    assert adapter.commands[marker:] == []


def test_pending_simulation_can_complete_once_and_never_verifies_hardware():
    backend = AnnouncementBackend(result=None)
    with pytest.raises(RuntimeError, match="not started"):
        backend.complete(True)
    assert backend.start(backend.text)
    assert not backend.status()["done"] and not backend.status()["failed"]
    assert backend.complete(True)
    assert backend.status()["done"]
    assert not backend.complete(False)
    assert not backend.start(backend.text)
    assert backend.status()["done"] and not backend.status()["hardware_verified"]


def test_pending_simulated_failure_is_terminal():
    backend = AnnouncementBackend(result=None)
    backend.start(backend.text)
    assert backend.complete(False)
    assert not backend.complete(True)
    assert backend.status()["failed"] and not backend.status()["done"]


def test_real_placeholder_stays_unknown_and_rejects_completion_injection():
    backend = AnnouncementBackend(simulated=False)
    backend.start(backend.text)
    with pytest.raises(RuntimeError, match="real"):
        backend.complete(True)
    assert backend.status() == {
        "started": True, "done": False, "failed": False,
        "simulated": False, "hardware_verified": False,
    }


def test_wrong_announcement_sentence_does_not_start():
    backend = AnnouncementBackend()
    with pytest.raises(ValueError, match="sentence"):
        backend.start("错误队名")
    assert not backend.status()["started"]
    assert not backend.status()["done"]


@pytest.mark.parametrize("completion", [None, False])
def test_crosswalk_cannot_release_without_simulator_success(completion):
    controller = RaceController(RaceConfig(school="山东大学", team="启航队"))
    controller.phase = Phase.CROSSWALK_HOLD
    backend = AnnouncementBackend(result=completion)
    with RaceExecutorBridge() as bridge:
        for tick in range(102):
            observation = Observation(
                round(tick/10, 1), source_ok=True, telemetry_valid=True,
                telemetry_age_s=0, speed_mps=0, task_monitor_valid=True,
                crosswalk_distance_m=.2, announcement_done=backend.status()["done"])
            intent = controller.update(observation)
            for event in intent["events"]:
                if event["event"] == "play_announcement":
                    backend.start(event["text"])
            assert bridge.apply(intent)["action"] == "stop"
    assert controller.phase == Phase.CROSSWALK_HOLD
    assert not controller.announcement_acknowledged


@pytest.mark.parametrize("clear_slot", ["left", "right"])
def test_synthetic_decision_flow_while_bridge_refuses_all_motion(clear_slot):
    # All vehicle/geometry/payment feedback here is synthetic, independent of
    # the stopped executor. Completion proves decision flow, not vehicle motion.
    controller = RaceController(RaceConfig(
        school="山东大学", team="启航队", vehicle_width_m=.19))
    backend = AnnouncementBackend(result=None)
    phases, play_events, refused_moves = set(), [], 0
    with RaceExecutorBridge() as bridge:
        for original in demonstration():
            row = replace(original, announcement_done=backend.status()["done"])
            if clear_slot == "left":
                row.cones = [{**c, "lateral_m": -c["lateral_m"]} for c in row.cones]
                row.parking_slots = [{**s, "availability": "clear" if s["id"] == "left" else "blocked"}
                                     for s in row.parking_slots]
                if row.parked_slot_id is not None:
                    row.parked_slot_id = "left"
            intent = controller.update(row)
            phases.add(intent["phase"])
            for event in intent["events"]:
                if event["event"] == "play_announcement":
                    play_events.append(event)
                    backend.start(event["text"])
            if row.t_s >= 2.5 and backend.started and not backend.done:
                backend.complete(True)
            output = bridge.apply(intent)
            assert output["action"] == "stop"
            assert not output["actuator"]["hardware_output"]
            if intent["action"] in ("follow_lane", "follow_corridor"):
                refused_moves += 1
                assert output["reason"] == "continuous_speed_mapping_not_calibrated"
            if intent["reason"] == "crosswalk_configured_hold_completed":
                assert row.t_s - controller.crosswalk_since >= 10
                assert backend.done
        assert bridge.executor.commands == []
        assert bridge.input_ended()["action"] == "stop"
    assert refused_moves > 0
    assert len(play_events) == 1
    assert phases == set(p.value for p in Phase) - {Phase.FAILED.value, Phase.ESTOP.value, Phase.CROSSWALK_EXIT.value}
    assert controller.summary()["payment_bonus_points"] == 5
    assert controller.summary()["parking_slot_id"] == clear_slot
    assert not backend.status()["hardware_verified"]
