import json
from dataclasses import asdict, replace
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.cli import main
from carvision.crosswalk import CrosswalkDetector, CrosswalkVisionConfig
from carvision.crosswalk_lab import CrosswalkPerception, default_config, crosswalk_plan
from carvision.config import Config
from carvision.race import Observation, Phase, RaceController, load_race_config
from carvision.sources import Frame, read_image, write_image


def config(**changes):
    return replace(load_race_config(default_config("crosswalk-3s.json")), **changes)


def obs(t, **changes):
    return replace(Observation(t, source_ok=True, telemetry_valid=True, telemetry_age_s=0,
                               speed_mps=0, task_monitor_valid=True, lane_valid=True, lane_offset=.1,
                               crosswalk_state="present", crosswalk_distance_m=.24), **changes)


def controller(**changes):
    return RaceController(config(**changes), scope="crosswalk")


def test_three_seconds_starts_from_stopped_feedback_not_detection():
    c = controller()
    for tick in range(10):
        d = c.update(obs(tick / 10, speed_mps=.08))
        assert d["action"] == "stop" and d["crosswalk_elapsed_s"] == 0
    for tick in range(10, 40):
        d = c.update(obs(tick / 10))
        assert not d["crosswalk_served"]
    d = c.update(obs(4.0))
    assert d["crosswalk_served"] and d["crosswalk_elapsed_s"] == 3
    assert d["action"] == "stop"  # transition frame still stops
    assert c.update(obs(4.1))["action"] == "follow_lane"


@pytest.mark.parametrize("change", [{"speed_mps": .02}, {"telemetry_valid": False},
                                    {"telemetry_age_s": 1}, {"source_ok": False},
                                    {"task_monitor_valid": False}, {"crosswalk_distance_m": None}])
def test_stop_timer_restarts_after_unproven_stationary_interval(change):
    c = controller()
    for tick in range(20):
        c.update(obs(tick / 10))
    assert c.update(obs(2.0, **change))["crosswalk_elapsed_s"] == 0
    for tick in range(21, 51):
        assert not c.update(obs(tick / 10))["crosswalk_served"]
    assert c.update(obs(5.1))["crosswalk_served"]


def test_timestamp_gap_does_not_satisfy_three_second_wait():
    c = controller()
    c.update(obs(0))
    d = c.update(obs(3))
    assert d["reason"] == "observation_gap" and not d["crosswalk_served"]


@pytest.mark.parametrize("scope", ["crosswalk", "full_race"])
def test_seen_crosswalk_leaving_view_never_restores_cruise(scope):
    c = RaceController(config(), scope=scope)
    c.phase = Phase.CROSSWALK_APPROACH
    assert c.update(obs(0, crosswalk_distance_m=.51, speed_mps=.08))["speed_mps"] == .08
    for tick in range(1, 6):
        d = c.update(obs(tick / 10, crosswalk_distance_m=None, crosswalk_state="absent", speed_mps=.08))
        assert d["reason"] == "crosswalk_distance_lost_after_detection" and d["speed_mps"] == 0
    assert c.update(obs(.6, crosswalk_distance_m=.4))["speed_mps"] == .08


def test_seen_evidence_is_kept_while_speed_feedback_is_missing():
    c = controller()
    c.update(obs(0, telemetry_valid=False, crosswalk_distance_m=.51))
    assert c.update(obs(.1, crosswalk_distance_m=None, crosswalk_state="absent"))["speed_mps"] == 0


def test_confirmed_candidate_without_metric_distance_stops():
    c = controller()
    assert c.update(obs(0, crosswalk_distance_m=None))["speed_mps"] == 0


def test_served_crosswalk_does_not_retrigger_but_lane_loss_and_estop_still_stop():
    c = controller()
    events = []
    for tick in range(61):
        d = c.update(obs(tick / 10))
        events.extend(d["events"])
    assert len([e for e in events if e["event"] == "crosswalk_completed"]) == 1
    assert d["action"] == "follow_lane"
    assert c.update(obs(6.1, lane_valid=False))["speed_mps"] == 0
    assert c.update(obs(6.2, emergency_stop=True))["phase"] == Phase.ESTOP.value
    assert c.update(obs(6.3))["speed_mps"] == 0


def test_negative_distance_and_duplicate_time_cannot_release_vehicle():
    c = controller()
    assert c.update(obs(0, crosswalk_distance_m=-.01))["phase"] == Phase.FAILED.value
    c = controller()
    c.update(obs(0))
    assert c.update(obs(0))["phase"] == Phase.FAILED.value


def test_audio_is_optional_for_stage_but_required_when_configured():
    c = controller(announcement_required=True)
    for tick in range(31):
        assert not c.update(obs(tick / 10))["crosswalk_served"]
    assert c.update(obs(3.1, announcement_done=True))["crosswalk_served"]


def test_full_race_uses_selected_three_seconds_and_continues_to_traffic():
    c = RaceController(config())
    c.phase = Phase.CROSSWALK_APPROACH
    for tick in range(31):
        d = c.update(obs(tick / 10))
    assert d["phase"] == Phase.LIGHT_APPROACH.value


def papers():
    image = np.zeros((480, 640, 3), np.uint8)
    for x in (150, 250, 350, 450):
        cv2.rectangle(image, (x, 270), (x+35, 345), (255, 255, 255), -1)
    return image


def test_temporal_confirmation_requires_continuous_media_time():
    d = CrosswalkDetector()
    assert d.detect(papers(), 0)[0]["presence"] == "unknown"
    assert d.detect(papers(), 100)[0]["presence"] == "unknown"
    assert d.detect(papers(), 200)[0]["presence"] == "present"
    assert d.detect(papers(), 2000)[0]["presence"] == "unknown"
    assert d.detect(papers(), 2000)[0]["presence"] == "unknown"


@pytest.mark.parametrize("name,expected", [("crosswalk-papers-secondary.jpg", True),
                                          ("crosswalk-background-primary.jpg", False)])
def test_recorded_positive_and_negative_fixtures(name, expected):
    data, _ = CrosswalkDetector().detect(read_image(Path(__file__).parent / "fixtures" / name))
    assert data["candidate"] == expected
    assert not data["metric_valid"] and data["distance_m"] is None


def test_old_calibration_never_extrapolates_closer_than_validated_region():
    p = CrosswalkPerception(Config(), CrosswalkVisionConfig(), profile=default_config("autonomy-pi5.json"))
    image = np.zeros((480, 640, 3), np.uint8)
    for x in (150, 250, 350, 450):
        cv2.rectangle(image, (x, 400), (x+35, 465), (255, 255, 255), -1)
    _, result, _ = p.process(Frame(image, 0, 0, 0))
    assert result["candidate"]
    assert result["distance_estimate_m"] is None
    assert result["projection_status"] == "pixel outside measured ground region"
    assert not result["metric_valid"]


def test_secondary_requires_explicit_matching_pose():
    p = CrosswalkPerception(Config(), CrosswalkVisionConfig(), profile=default_config("autonomy-pi5.json"),
                           camera_name="secondary")
    _, result, _ = p.process(Frame(papers(), 0, 0, 0))
    assert result["projection_status"] == "camera servo pose changed; calibration invalid"


@pytest.mark.parametrize("camera", ["primary", "secondary"])
def test_real_clipped_papers_remain_presence_without_a_fake_near_edge(camera):
    image = read_image(Path(__file__).parent / "fixtures" / f"crosswalk-clipped-{camera}-20261009.jpg")
    result, _ = CrosswalkDetector().detect(image, 0)
    assert result["candidate"]
    assert result["bottom_clipped"]
    assert result["near_edge_y_normalized"] is None
    assert result["metric_valid"] is False


def test_real_distant_papers_fit_row_width_and_confirm_in_time():
    image=read_image(Path(__file__).parent/'fixtures/crosswalk-distant-secondary-20261009.jpg')
    detector=CrosswalkDetector()
    for t in (0,100,200,300):
        result,_=detector.detect(image,t)
    assert result['candidate'] and len(result['stripes_xywh'])>=3
    assert result['presence']=='present'


def test_observation_replay_complete_stop_resume_without_hardware_or_webpage(tmp_path, monkeypatch):
    from carvision.rasadapter5 import RasAdapter
    def forbidden(*args, **kwargs):
        raise AssertionError("software lab touched UART")
    monkeypatch.setattr(RasAdapter, "open", forbidden)
    source = tmp_path / "observations.jsonl"
    source.write_text("\n".join(json.dumps(asdict(obs(i / 10))) for i in range(41)), encoding="utf-8")
    out = tmp_path / "output"
    args = SimpleNamespace(output=out, input=source, race_config=None)
    summary = crosswalk_plan(args)
    assert summary["crosswalk_served"] and summary["resume_frames"] > 0
    assert not summary["hardware_output"] and not summary["physical_task_verified"]
    records = [json.loads(s) for s in (out / "decisions.jsonl").read_text(encoding="utf-8").splitlines()]
    assert records[-1]["action"] == "stop"
    starts = [r["t_s"] for r in records if any(e["event"] == "crosswalk_stationary_hold_started" for e in r["events"])]
    ends = [r["t_s"] for r in records if any(e["event"] == "crosswalk_completed" for e in r["events"])]
    assert len(starts) == len(ends) == 1 and ends[0] - starts[0] == pytest.approx(3)
    assert not list(out.glob("*.html")) and not list(out.glob("*.avi"))
    with pytest.raises(FileExistsError):
        crosswalk_plan(args)


def test_image_cli_does_not_invent_speed_or_start_a_three_second_timer(tmp_path):
    source = tmp_path / "白纸.jpg"
    write_image(source, papers())
    out = tmp_path / "结果"
    assert main(["crosswalk-replay", "--source", str(source), "--output", str(out)]) == 0
    observation = json.loads((out / "observations.jsonl").read_text(encoding="utf-8"))
    assert observation["speed_mps"] is None and not observation["telemetry_valid"]
    assert observation["crosswalk_distance_m"] is None
    assert not json.loads((out / "summary.json").read_text(encoding="utf-8"))["crosswalk_served"]


def test_observation_plan_writes_stop_and_error_for_corrupt_input(tmp_path):
    path = tmp_path / "input.jsonl"
    path.write_text(json.dumps(asdict(obs(0))) + '\n{"broken":\n', encoding="utf-8")
    out = tmp_path / "bad"
    with pytest.raises(json.JSONDecodeError):
        crosswalk_plan(SimpleNamespace(input=path, output=out, race_config=None))
    log = (out / "decisions.jsonl").read_text(encoding="utf-8").splitlines()
    assert json.loads(log[-1])["action"] == "stop"
    assert json.loads((out / "summary.json").read_text(encoding="utf-8"))["error"]


def test_empty_plan_is_not_success(tmp_path):
    path = tmp_path / "empty.jsonl"
    path.write_text("", encoding="utf-8")
    with pytest.raises(ValueError, match="no observations"):
        crosswalk_plan(SimpleNamespace(input=path, output=tmp_path / "empty", race_config=None))


@pytest.mark.parametrize("kwargs", [{"min_stripes": 2}, {"roi_top": 1}, {"confirm_ms": float("nan")},
                                  {"white_v_min": True}, {"max_gap_ms": 0}])
def test_bad_detector_config_is_rejected(kwargs):
    with pytest.raises(ValueError):
        CrosswalkVisionConfig(**kwargs).validate()
