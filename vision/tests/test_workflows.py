import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from carvision.cli import main
from carvision.dataset import check_dataset
from carvision.demo import generate_demo
from carvision.sources import FileSource, read_image, write_image
from carvision.results import RACE_CLASSES


def test_image_reading_supports_chinese_path_and_missing_file(tmp_path):
    p = tmp_path/"赛道.png"
    write_image(p, np.full((60, 80, 3), 45, np.uint8))
    assert read_image(p).shape == (60, 80, 3)
    with pytest.raises(FileNotFoundError):
        FileSource(tmp_path/"missing.mp4")


def test_demo_end_to_end_and_existing_output_is_not_overwritten(tmp_path):
    video = generate_demo(tmp_path/"demo")
    output = tmp_path/"results"
    args = ["replay", "--source", str(video), "--output", str(output), "--save-video"]
    assert main(args) == 0
    summary = json.loads((output/"summary.json").read_text())
    assert summary["frames"] == 160
    assert summary["lane_valid_frames"] == 140
    rows = [json.loads(s) for s in (output/"results.jsonl").read_text().splitlines()]
    assert rows[0]["source_time_ms"] == 0
    assert rows[1]["source_time_ms"] == 50
    assert not any(row["lane"]["valid"] for row in rows[100:120])
    assert (output/"overlay.avi").stat().st_size > 1000
    assert main(args) == 1
    assert len((output/"results.jsonl").read_text().splitlines()) == 160


def test_invalid_video_writes_error_summary(tmp_path):
    video = tmp_path/"broken.avi"
    video.write_bytes(b"not a video")
    output = tmp_path/"run"
    assert main(["replay", "--source", str(video), "--output", str(output)]) == 1
    assert json.loads((output/"summary.json").read_text())["status"] == "error"


def make_dataset(root):
    for split, color in [("train", 40), ("val", 90)]:
        image = root/"images"/split/f"{split}-session"/"001.png"
        label = root/"labels"/split/f"{split}-session"/"001.txt"
        write_image(image, np.full((80, 100, 3), color, np.uint8))
        label.parent.mkdir(parents=True)
        label.write_text("\n".join(f"{i} .5 .5 .2 .2" for i in range(4)))
    p = root/"dataset.yaml"
    p.write_text(yaml.safe_dump({"path": ".", "train": "images/train", "val": "images/val",
                                 "names": list(RACE_CLASSES)}))
    return p


def test_dataset_requires_annotations_and_rejects_pixel_leakage(tmp_path):
    p = make_dataset(tmp_path)
    report, resolved = check_dataset(p)
    assert report["valid"]
    assert Path(resolved["path"]).is_absolute()
    write_image(tmp_path/"images/val/val-session/001.png", read_image(tmp_path/"images/train/train-session/001.png"))
    report, _ = check_dataset(p)
    assert not report["valid"]
    assert any("duplicate" in e for e in report["errors"])
    (tmp_path/"labels/train/train-session/001.txt").unlink()
    report, _ = check_dataset(p)
    assert any("missing label" in e for e in report["errors"])


def test_dataset_rejects_out_of_bounds_box(tmp_path):
    p = make_dataset(tmp_path)
    (tmp_path/"labels/train/train-session/001.txt").write_text("0 .99 .5 .2 .2")
    report, _ = check_dataset(p)
    assert any("outside image" in e for e in report["errors"])


def test_extraction_preserves_unlabeled_status(tmp_path):
    video = generate_demo(tmp_path/"demo")
    assert main(["extract", "--source", str(video), "--output", str(tmp_path/"frames"),
                 "--session", "session1", "--interval-s", "1"]) == 0
    folder = tmp_path/"frames/session1"
    assert len(list(folder.glob("*.jpg"))) == 8
    assert "unreviewed" in (folder/"manifest.csv").read_text()


def test_capture_duration_starts_after_first_frame(tmp_path, monkeypatch):
    from carvision import workflows
    from carvision.sources import Frame

    clock = [0.0]
    cameras = []

    class SlowOpeningCamera:
        def __init__(self, *args):
            clock[0] += 5.0  # Hardware initialization exceeds requested recording time.
            self.reported = {"width": 64, "height": 48, "fps": 2}
            self.request_results = {}
            self.closed = False
            cameras.append(self)

        def __iter__(self):
            for i in range(12):
                clock[0] = 5+i*.5
                yield Frame(np.full((48, 64, 3), i*10, np.uint8), i, None, clock[0]*1000)

        def close(self):
            self.closed = True

    monkeypatch.setattr(workflows, "CameraSource", SlowOpeningCamera)
    monkeypatch.setattr(workflows.time, "monotonic", lambda: clock[0])
    output = tmp_path/"capture"
    assert main(["capture", "--seconds", "2", "--output", str(output)]) == 0
    info = json.loads((output/"capture-info.json").read_text())
    assert info["frames"] == 5
    assert info["recorded_span_s"] == 2
    assert info["startup_s"] == 5
    assert info["stop_reason"] == "duration_reached"
    assert cameras[0].closed
