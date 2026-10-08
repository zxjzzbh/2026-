import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from carvision.gamepad_controls import SCRIPT
from carvision.manual_controls import SCRIPT as DASHBOARD_SCRIPT
from carvision.manual_drive import DriveError, ManualDrive, SimulationBackend


def test_gamepad_browser_guard_and_dashboard_integration():
    node = os.environ.get("SMARTCAR_NODE") or shutil.which("node")
    if not node:
        pytest.skip("Node.js is required to execute the browser input checks")
    result = subprocess.run(
        [node, str(Path(__file__).parent / "js/gamepad_controls.cjs")],
        input=json.dumps({"component": SCRIPT, "dashboard": DASHBOARD_SCRIPT}),
        capture_output=True, text=True, encoding="utf-8", timeout=15,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    record = json.loads(result.stdout)
    assert record["checks_passed"] == 30
    assert record["hardware_output"] is False


def test_gamepad_source_is_logged_and_same_lease_and_stop_rules_apply():
    now = [0.0]
    drive = ManualDrive(SimulationBackend(), clock=lambda: now[0], settling_s=0)
    drive.request("enable", {"client": "gamepad-test", "bench_ready": True})
    drive.tick()
    result = drive.request("command", {"client": "gamepad-test", "sequence": 1,
        "motor": "forward", "steering": "right", "input_source": "gamepad"})
    assert result["last_input_source"] == "gamepad"
    assert result["lease_ms"] == 200
    now[0] = .201
    drive.tick()
    assert drive.status()["release_required"] is True
    # Refreshing a held controller after the lease expiry cannot start it again.
    with pytest.raises(DriveError):
        drive.request("command", {"client": "gamepad-test", "sequence": 2,
            "motor": "forward", "steering": "right", "input_source": "gamepad"})
    assert drive.status()["motor_direction"] == "stop"


def test_gamepad_stop_reason_is_retained_in_safety_log():
    now = [0.0]
    events = []
    drive = ManualDrive(SimulationBackend(), clock=lambda: now[0], settling_s=0, event_sink=events.append)
    drive.request("enable", {"client": "gamepad-test", "bench_ready": True})
    drive.tick()
    drive.request("stop", {"stop_source": "gamepad_deadman_release"})
    assert any(e["event"] == "safety_request" and
               e["input_source"] == "gamepad_deadman_release" for e in events)
