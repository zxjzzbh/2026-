"""CLI integration checks on the project environment; no device access."""
import json
from dataclasses import asdict

import pytest

from carvision.cli import main
from carvision.race import RaceConfig
from carvision.race_workflows import demonstration


def inputs(tmp_path):
    config = tmp_path / "race.json"
    config.write_text(json.dumps(asdict(RaceConfig(
        school="山东大学", team="启航队", vehicle_width_m=.19))), encoding="utf-8")
    source = tmp_path / "synthetic.jsonl"
    source.write_text(''.join(json.dumps(asdict(o)) + '\n' for o in demonstration()), encoding="utf-8")
    output = tmp_path / "output"
    args = ["pi-run", "--backend", "rasadapter5", "--race-config", str(config),
            "--input", str(source), "--output", str(output)]
    return args, source, output


def test_cli_rasadapter_backend_replays_every_row_and_preserves_existing_output(tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("hardware touched")

    monkeypatch.setattr("carvision.rasadapter5_executor.RasAdapter", forbidden)
    args, source, output = inputs(tmp_path)
    assert main(args) == 0
    summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
    assert summary["observations"] == 221 and summary["decision_flow_completed"]
    assert summary["backend"] == "rasadapter5" and not summary["hardware_output"]
    assert not summary["actuation_mapping_ready"]
    original = (output/"pi-decisions.jsonl").read_bytes()
    assert main(args) == 1
    assert (output/"pi-decisions.jsonl").read_bytes() == original


def test_cli_real_mode_refuses_before_creating_output_or_transport(tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("hardware touched")

    monkeypatch.setattr("carvision.rasadapter5_executor.RasAdapter", forbidden)
    args, source, output = inputs(tmp_path)
    assert main(args + ["--run", "--acknowledge-motion"]) == 1
    assert not output.exists()


@pytest.mark.parametrize("failure", ["missing_file", "malformed_json"])
def test_cli_input_error_persists_failure_and_stop(tmp_path, failure):
    args, source, output = inputs(tmp_path)
    if failure == "missing_file":
        source.unlink()
    else:
        source.write_text('{broken}', encoding="utf-8")
    assert main(args) == 1
    summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
    assert summary["error"] and not summary["hardware_output"]
    rows = [json.loads(s) for s in (output/"pi-decisions.jsonl").read_text(encoding="utf-8").splitlines()]
    assert rows[-1]["execution"]["reason"] == "input_error"


def test_legacy_cli_requires_its_config_without_opening_output(tmp_path):
    args, source, output = inputs(tmp_path)
    args[2] = "hardware-pwm"
    assert main(args) == 1
    assert not output.exists()
