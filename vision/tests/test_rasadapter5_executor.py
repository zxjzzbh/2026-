import pytest
from dataclasses import replace
import threading
import struct

from carvision.rasadapter5 import RasAdapter

from carvision.rasadapter5_executor import (
    RasAdapterExecutionConfig,
    RasAdapterS3S4Executor,
)


def calibrated():
    return RasAdapterExecutionConfig(
        steering_left_us=1750,
        steering_center_us=1650,
        steering_right_us=1550,
        esc_neutral_us=1500,
        esc_forward_us=1575,
        esc_reverse_us=1300,
        calibration_confirmed=True,
    )


def test_default_executor_is_dry_and_rejects_unconfirmed_run():
    executor = RasAdapterS3S4Executor(RasAdapterExecutionConfig())
    executor.open()
    result = executor.apply(0, 0)
    assert result["hardware_output"] is False
    assert result["missing"]
    with pytest.raises(ValueError, match="calibration incomplete"):
        RasAdapterS3S4Executor(RasAdapterExecutionConfig(), run=True)


def test_dry_executor_plans_known_s3_s4_points_without_serial():
    executor = RasAdapterS3S4Executor(calibrated())
    result = executor.apply(1, -1)
    assert result == {
        "esc_us": 1575,
        "steering_us": 1750,
        "missing": [],
        "hardware_output": False,
    }
    assert executor.commands == []


def test_fake_adapter_receives_s3_s4_commands_only():
    adapter = type("Fake", (), {
        "commands": [],
        "open": lambda self: self.commands.append(("open",)),
        "set_position": lambda self, ch, pulse, seconds=.02: self.commands.append(("steering", ch, pulse, seconds)),
        "set_esc": lambda self, ch, pulse: self.commands.append(("esc", ch, pulse)),
        "close": lambda self: self.commands.append(("close",)),
    })()
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    executor.open()
    executor.apply(1, -1)
    executor.neutral()
    executor.close()
    assert ("steering", 3, 1750, .02) in adapter.commands
    assert ("esc", 4, 1575) in adapter.commands
    assert ("esc", 4, 1500) in adapter.commands


def test_emergency_stop_is_latched_and_rejects_following_motion():
    executor = RasAdapterS3S4Executor(calibrated())
    executor.emergency_stop()
    with pytest.raises(RuntimeError, match="latched"):
        executor.apply(.1, 0)


def test_motion_window_and_command_lease_are_bounded():
    executor = RasAdapterS3S4Executor(calibrated())
    with pytest.raises(ValueError, match="bounded"):
        executor.apply(.1, 0, duration_s=.81)


class FakeAdapter:
    def __init__(self):
        self.commands = []
        self.neutral_seen = threading.Event()
        self.fail_forward = False
        self.fail_neutral = False
        self.fail_steering = False

    def open(self):
        self.commands.append(("open",))

    def set_position(self, channel, pulse, seconds=.02):
        self.commands.append(("steering", channel, pulse, seconds))
        if self.fail_steering:
            raise OSError("steering write failed")

    def set_esc(self, channel, pulse):
        self.commands.append(("esc", channel, pulse))
        if pulse == 1500:
            if self.fail_neutral:
                raise OSError("neutral write failed")
            self.neutral_seen.set()
        elif self.fail_forward:
            raise OSError("forward write failed")

    def close(self):
        self.commands.append(("close",))


def test_watchdog_stops_without_caller_tick():
    adapter = FakeAdapter()
    executor = RasAdapterS3S4Executor(
        replace(calibrated(), command_lease_s=.04), run=True, adapter=adapter)
    try:
        executor.open()
        adapter.neutral_seen.clear()
        executor.apply(1, 0, duration_s=.8)
        assert adapter.neutral_seen.wait(.6), "expiry must not require a caller tick"
        with pytest.raises(RuntimeError):
            executor.apply(1, 0, duration_s=.8)
    finally:
        executor.close()


def test_refresh_does_not_extend_continuous_motion_window():
    now = [0.0]
    executor = RasAdapterS3S4Executor(calibrated(), clock=lambda: now[0])
    executor.apply(1, 0, duration_s=.8)
    for moment in (.2, .4, .6):
        now[0] = moment
        executor.apply(1, 0, duration_s=.8)
    now[0] = .81
    executor.tick()
    assert executor.last_command == ("neutral",)
    with pytest.raises(RuntimeError):
        executor.apply(1, 0, duration_s=.8)


def test_partial_throttle_is_rejected_before_unreviewed_s4_pulse():
    executor = RasAdapterS3S4Executor(calibrated())
    with pytest.raises(ValueError):
        executor.planned_pulses(.1, 0)


def test_bad_input_stops_existing_output():
    adapter = FakeAdapter()
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    try:
        executor.open()
        executor.apply(1, 0, duration_s=.8)
        adapter.neutral_seen.clear()
        with pytest.raises(ValueError):
            executor.apply(float("nan"), 0)
        assert adapter.neutral_seen.is_set()
        with pytest.raises(RuntimeError):
            executor.apply(1, 0)
    finally:
        executor.close()


def test_write_error_stops_and_latches_failure():
    adapter = FakeAdapter()
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    try:
        executor.open()
        adapter.fail_forward = True
        adapter.neutral_seen.clear()
        with pytest.raises(OSError):
            executor.apply(1, 0, duration_s=.8)
        assert adapter.neutral_seen.is_set()
        adapter.fail_forward = False
        with pytest.raises(RuntimeError):
            executor.apply(1, 0)
    finally:
        executor.close()


def test_close_releases_adapter_even_when_neutral_fails():
    adapter = FakeAdapter()
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    executor.open()
    adapter.fail_neutral = True
    assert executor.close()
    assert adapter.commands[-1] == ("close",)


def test_open_sends_neutral_before_any_motion():
    adapter = FakeAdapter()
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    try:
        executor.open()
        assert adapter.commands[:3] == [
            ("open",), ("esc", 4, 1500), ("steering", 3, 1650, .02)]
    finally:
        executor.close()


@pytest.mark.parametrize("field,value", [
    ("calibration_confirmed", "false"),
    ("steering_center_us", 2000),
    ("esc_forward_us", 1510),
    ("esc_reverse_us", 1400),
])
def test_config_rejects_unreviewed_or_non_boolean_confirmation(field, value):
    with pytest.raises(ValueError):
        replace(calibrated(), **{field: value}).validate()


def test_incomplete_confirmed_config_reports_missing_instead_of_type_error():
    with pytest.raises(ValueError, match="incomplete"):
        RasAdapterExecutionConfig(calibration_confirmed=True).validate(require_ready=True)


def test_run_requires_boolean():
    with pytest.raises(ValueError):
        RasAdapterS3S4Executor(calibrated(), run="false", adapter=FakeAdapter())


def test_dry_run_never_constructs_transport_or_calls_injected_adapter(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("dry mode touched UART")

    monkeypatch.setattr("carvision.rasadapter5_executor.RasAdapter", forbidden)
    adapter = FakeAdapter()
    for injected in (None, adapter):
        with RasAdapterS3S4Executor(calibrated(), adapter=injected) as executor:
            executor.apply(1, -1, duration_s=.8)
            executor.neutral()
            executor.emergency_stop()
        assert executor.watchdog is None
    assert adapter.commands == []


def test_watchdog_exits_and_closed_executor_cannot_reopen_or_drive():
    adapter = FakeAdapter()
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    executor.open()
    executor.open()  # no second UART open/thread
    assert adapter.commands.count(("open",)) == 1
    assert executor.close() == []
    assert not executor.watchdog.is_alive()
    commands = list(adapter.commands)
    assert executor.close() == []
    executor.neutral()  # must not write to a released UART
    with pytest.raises(RuntimeError, match="closed"):
        executor.open()
    with pytest.raises(RuntimeError, match="closed"):
        executor.apply(1, 0)
    assert adapter.commands == commands


def test_unopened_real_executor_does_not_write():
    adapter = FakeAdapter()
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    with pytest.raises(RuntimeError, match="not open"):
        executor.apply(1, 0)
    executor.neutral()
    executor.close()
    assert adapter.commands == []


def test_real_estop_stops_and_neutral_does_not_clear_latch():
    adapter = FakeAdapter()
    with RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter) as executor:
        executor.apply(1, -1, duration_s=.8)
        adapter.neutral_seen.clear()
        executor.emergency_stop()
        assert adapter.neutral_seen.is_set()
        commands = list(adapter.commands)
        executor.neutral()
        with pytest.raises(RuntimeError, match="latched"):
            executor.apply(1, 0)
        with pytest.raises(RuntimeError, match="latched"):
            executor.open()
        assert ("esc", 4, 1575) not in adapter.commands[len(commands):]


def test_exception_inside_context_stops_and_releases_uart():
    adapter = FakeAdapter()
    with pytest.raises(LookupError, match="input failed"):
        with RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter) as executor:
            executor.apply(1, 1, duration_s=.8)
            adapter.neutral_seen.clear()
            raise LookupError("input failed")
    assert adapter.neutral_seen.is_set()
    assert adapter.commands[-1] == ("close",)
    assert executor.closed and not executor.watchdog.is_alive()


def test_startup_neutral_failure_closes_without_starting_watchdog():
    adapter = FakeAdapter()
    adapter.fail_neutral = True
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    with pytest.raises(RuntimeError, match="neutral write failed"):
        executor.open()
    assert adapter.commands[-1] == ("close",)
    assert executor.closed and not executor.opened and executor.watchdog is None
    assert not any(c[:1] == ("esc",) and c[2] != 1500 for c in adapter.commands)


def test_partial_open_failure_closes_adapter():
    class FailedOpen(FakeAdapter):
        def open(self):
            super().open()
            raise OSError("UART occupied")

    adapter = FailedOpen()
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    with pytest.raises(OSError, match="occupied"):
        executor.open()
    assert adapter.commands == [("open",), ("close",)]
    assert executor.closed


def test_center_failure_does_not_skip_neutral_or_release():
    adapter = FakeAdapter()
    executor = RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter)
    executor.open()
    adapter.fail_steering = True
    adapter.neutral_seen.clear()
    errors = executor.close()
    assert adapter.neutral_seen.is_set()
    assert adapter.commands[-1] == ("close",)
    assert any("steering write failed" in e for e in errors)


def test_cleanup_failure_is_reported_and_does_not_mask_primary_exception():
    adapter = FakeAdapter()
    with pytest.raises(RuntimeError, match="cleanup failed"):
        with RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter):
            adapter.fail_neutral = True
    assert adapter.commands[-1] == ("close",)
    adapter = FakeAdapter()
    with pytest.raises(LookupError, match="primary"):
        with RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter) as executor:
            adapter.fail_neutral = True
            raise LookupError("primary")
    assert executor.cleanup_errors
    assert adapter.commands[-1] == ("close",)


def test_expired_command_is_not_revived_by_late_refresh():
    now = [0.0]
    adapter = FakeAdapter()
    with RasAdapterS3S4Executor(
            calibrated(), run=True, adapter=adapter, clock=lambda: now[0]) as executor:
        executor.apply(1, 0, duration_s=.8)
        now[0] = .25
        with pytest.raises(RuntimeError, match="lease expired"):
            executor.apply(1, 0, duration_s=.8)
        executor.neutral()
        with pytest.raises(RuntimeError, match="latched"):
            executor.apply(1, 0, duration_s=.8)
        assert adapter.commands.count(("esc", 4, 1575)) == 1


def test_uart_write_time_counts_against_lease_and_no_late_throttle():
    now = [0.0]

    class SlowSteering(FakeAdapter):
        def set_position(self, channel, pulse, seconds=.02):
            super().set_position(channel, pulse, seconds)
            if pulse != 1650:
                now[0] += .3

    adapter = SlowSteering()
    with RasAdapterS3S4Executor(
            calibrated(), run=True, adapter=adapter, clock=lambda: now[0]) as executor:
        with pytest.raises(RuntimeError, match="lease expired"):
            executor.apply(1, -1, duration_s=.8)
        assert ("esc", 4, 1575) not in adapter.commands
        assert ("esc", 4, 1500) in adapter.commands


@pytest.mark.parametrize("throttle,steering,duration", [
    (float("nan"), 0, .8), (float("inf"), 0, .8), (True, 0, .8),
    (.1, 0, .8), (-1, 0, .8), (1, float("nan"), .8), (1, True, .8),
    (1, 2, .8), (1, 0, float("nan")), (1, 0, float("inf")),
    (1, 0, True), (1, 0, 0), (1, 0, .81),
])
def test_invalid_motion_input_stops_and_remains_latched(throttle, steering, duration):
    adapter = FakeAdapter()
    with RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter) as executor:
        executor.apply(1, 0, duration_s=.8)
        adapter.neutral_seen.clear()
        with pytest.raises(ValueError):
            executor.apply(throttle, steering, duration_s=duration)
        assert adapter.neutral_seen.is_set()
        with pytest.raises(RuntimeError, match="latched"):
            executor.apply(1, 0, duration_s=.8)


@pytest.mark.parametrize("field,value", [
    ("command_lease_s", float("nan")), ("command_lease_s", .251),
    ("command_lease_s", False), ("maximum_motion_s", float("inf")),
    ("maximum_motion_s", .801), ("maximum_motion_s", "0.8"),
    ("steering_channel", 3.0), ("esc_channel", 4.0),
    ("steering_left_us", 1751), ("steering_right_us", 1549),
    ("steering_center_us", 1550), ("esc_neutral_us", 1501),
    ("port", ""), ("calibration_confirmed", 1),
])
def test_invalid_configuration_rejected_before_transport_creation(monkeypatch, field, value):
    def forbidden(*args, **kwargs):
        pytest.fail("invalid config constructed transport")

    monkeypatch.setattr("carvision.rasadapter5_executor.RasAdapter", forbidden)
    with pytest.raises(ValueError):
        RasAdapterS3S4Executor(replace(calibrated(), **{field: value}), run=True)


def test_real_transport_validation_and_s3_s4_packet_encoding_without_uart():
    class PacketAdapter(RasAdapter):
        def __init__(self):
            super().__init__("/not-opened")
            self.packets = []
            self.released = False

        def open(self):
            pass  # The only fake boundary; no file descriptor is opened.

        def send(self, function, payload):
            self.packets.append((function, bytes(payload)))

        def close(self):
            self.released = True

    adapter = PacketAdapter()
    with RasAdapterS3S4Executor(calibrated(), run=True, adapter=adapter) as executor:
        for steering in (-1, 0, 1):
            executor.apply(1, steering, duration_s=.8)
        executor.neutral()
    decoded = [struct.unpack("<BHBBH", payload) for function, payload in adapter.packets
               if function == 4]
    assert len(decoded) == len(adapter.packets)
    assert all(command == 1 and duration == 20 and count == 1
               and channel in (3, 4) for command, duration, count, channel, pulse in decoded)
    assert {pulse for _, _, _, channel, pulse in decoded if channel == 3} == {1750, 1650, 1550}
    assert {pulse for _, _, _, channel, pulse in decoded if channel == 4} == {1500, 1575}
    assert adapter.released and adapter.fd is None
