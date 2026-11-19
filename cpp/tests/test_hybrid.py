"""Combined entry-point tests. All frames come from files or generated scenes."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import cv2
import numpy as np

CPP = Path(__file__).resolve().parents[1]
FIXTURES = CPP / "tests" / "fixtures"
CONFIG = CPP / "config" / "race.json"
EXECUTABLE = Path(sys.argv.pop(1)).resolve()
sys.path.insert(0, str(CPP / "algorithms"))
from hybrid import Hybrid, read_json
from cone_sequence import estimate_one, lane


def fixture(name):
    return cv2.imdecode(np.fromfile(FIXTURES / name, np.uint8), cv2.IMREAD_COLOR)


def lamp(active):
    image = np.full((360, 480, 3), 25, np.uint8)
    for i, (x, color) in enumerate(((100, (0, 0, 70)), (210, (0, 70, 70)), (320, (0, 70, 0)))):
        if i == active:
            color = tuple(255 if channel else 0 for channel in color)
        cv2.circle(image, (x, 120), 27, color, -1)
    return image


def track(cone=False):
    image = np.full((360, 480, 3), (45, 50, 150), np.uint8)
    ys = np.arange(150, 324)
    width = 180 + .65 * (ys - 226)
    for sign in (-1, 1):
        pts = np.column_stack((240 + sign * width / 2, ys)).astype(np.int32)
        cv2.polylines(image, [pts], False, (255, 255, 255), 6)
    if cone:
        cv2.fillPoly(image, [np.int32([[248, 205], [228, 260], [266, 260]])], (210, 55, 25))
    return image


def replay(task, road, primary=None):
    with tempfile.TemporaryDirectory() as tmp:
        folder = Path(tmp)
        def video(name, frames):
            path = folder / (name + ".avi")
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 10, (480, 360))
            if not writer.isOpened():
                raise RuntimeError("OpenCV cannot write the test video")
            for image in frames:
                writer.write(cv2.resize(image, (480, 360)))
            writer.release()
            return str(path)

        args = [str(EXECUTABLE), "--task", task, "--config", "config/race.json",
                "--output", str(folder / "out"), "--video", video("secondary", road)]
        if primary is not None:
            args += ["--traffic-video", video("primary", primary)]
        subprocess.run(args, cwd=CPP, capture_output=True, check=True, timeout=150)
        log, = (folder / "out").glob("*/telemetry.jsonl")
        rows = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
        if any(row["drive_enabled"] for row in rows[:-1]):
            raise AssertionError("replay enabled vehicle outputs")
        return rows[:-1], rows[-1]


class CombinedProgram(unittest.TestCase):
    def test_camera_reopen_after_outage_keeps_same_run(self):
        # A one-frame file exercises the live capture worker's reopen path without a vehicle.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            camera = root / "camera.avi"
            backup = root / "disconnected.avi"
            writer = cv2.VideoWriter(str(camera), cv2.VideoWriter_fourcc(*"MJPG"), 10, (480, 360))
            self.assertTrue(writer.isOpened())
            writer.write(track())
            writer.release()
            args = [str(EXECUTABLE), "--task", "parking", "--config", "config/race.json",
                    "--secondary", str(camera), "--max-frames", "7", "--output", str(root/"out")]
            process = subprocess.Popen(args, cwd=CPP, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    logs = list((root/"out").glob("*/telemetry.jsonl"))
                    if logs and '"frame":' in logs[0].read_text(encoding="utf-8"):
                        try:
                            camera.rename(backup)
                            break
                        except PermissionError:
                            pass
                    self.assertIsNone(process.poll())
                    time.sleep(.03)
                self.assertTrue(backup.exists(), "capture never produced its first frame")
                time.sleep(6)
                self.assertIsNone(process.poll(), "temporary loss terminated the live session")
                self.assertIn("waiting_for_road_camera", logs[0].read_text(encoding="utf-8"))
                backup.rename(camera)
                self.assertEqual(process.wait(timeout=15), 0)
                log, = (root/"out").glob("*/telemetry.jsonl")
                rows = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
                frames = [row for row in rows if "frame" in row]
                self.assertEqual(len(frames), 7)
                self.assertGreater(frames[-1]["time"] - frames[0]["time"], 6)
                self.assertFalse(any(row["drive_enabled"] for row in frames))
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()

    def test_outdoor_paper_and_negative_frames(self):
        cfg = read_json(CONFIG)
        engine = Hybrid(CONFIG, cfg["primary_camera"], cfg["secondary_camera"])
        for quality in (None, 95, 85, 70):
            for i, suffix in enumerate(("0401", "0410", "0420", "0430", "0460", "0470", "0780")):
                image = fixture("zebra_outdoor_" + suffix + ".jpg")
                if quality:
                    image = cv2.imdecode(cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])[1], 1)
                row = engine.crosswalk.update({"secondary": (image, i + (quality or 100) * 10, i / 10)})
                self.assertEqual(row["detected"], i < 4, (suffix, quality, row))
                if i == 3:
                    self.assertEqual((row["status"], row["front_m"]), ("clipped", -1.))

    def test_two_calibrations_keep_own_frame_and_distance(self):
        cfg = read_json(CONFIG)
        engine = Hybrid(CONFIG, cfg["primary_camera"], cfg["secondary_camera"])
        def paper(bottom):
            image = np.zeros((360, 480, 3), np.uint8)
            for x in (95, 175, 255, 335):
                cv2.rectangle(image, (x, bottom-73), (x+27, bottom), (245, 245, 245), -1)
            return image
        rows = {"secondary": (paper(300), 27, 1.), "primary": (paper(340), 19, .99)}
        engine.crosswalk.update(rows)
        self.assertIsNotNone(engine.crosswalk.distance_disagreement)
        self.assertEqual(engine.crosswalk.distance_disagreement["raw_frame_ids"], {"primary": 19, "secondary": 27})
        self.assertEqual(engine.crosswalk.trigger, .4)
        swapped = Hybrid(CONFIG, cfg["secondary_camera"], cfg["primary_camera"])
        self.assertFalse(any(swapped.calibrated.values()))

    def test_signal_conflict_then_same_camera_release(self):
        black = np.zeros((360, 480, 3), np.uint8)
        red, green = lamp(0), lamp(2)
        road = [black]*15 + [green]*28 + [black]*35
        primary = [red]*15 + [black]*28 + [green]*35
        rows, summary = replay("traffic", road, primary)
        self.assertEqual(rows[0]["requested_throttle_us"], 1400)
        self.assertTrue(any(row["red"] for row in rows))
        self.assertTrue(all(not row["green"] for row in rows[:43]))
        self.assertTrue(summary["completed"], rows[-1])
        self.assertEqual(len(rows), len(road))
        for row in rows[-5:]:
            self.assertEqual((row["state"], row["requested_throttle_us"]), ("finished", 1500))

    def test_short_yellow_can_release_only_on_same_lamp_green(self):
        black = np.zeros((360, 480, 3), np.uint8)
        green = lamp(2)
        other_fixture = np.roll(green, 90, axis=0)
        primary = [lamp(1)] + [other_fixture] * 32 + [green] * 44
        rows, summary = replay("traffic", [black] * len(primary), primary)
        self.assertTrue(rows[0]["signal_stop"])
        self.assertFalse(any(row["red"] for row in rows))
        self.assertTrue(any(row["state"] == "traffic_wait_green" for row in rows))
        self.assertTrue(summary["completed"])
        self.assertFalse(any(row["green"] for row in rows[:50]))

    def test_cone_repeated_frame_and_recoverable_loss(self):
        cfg = read_json(CONFIG)
        engine = Hybrid(CONFIG, cfg["primary_camera"], cfg["secondary_camera"])
        image = track().tobytes()
        def sample(fid, captured, now):
            return json.loads(engine.step("two_cones", image, fid, captured, None, 0, -1, now))["cone"]
        for i in range(6):
            fresh = sample(i+1, i/10, i/10)
        self.assertGreater(fresh["pwm"], 1500)
        repeated = sample(6, .5, .55)
        self.assertEqual(repeated["repeat_forward"], 1)
        self.assertEqual(repeated["captured_s"], .5)
        paused = sample(6, .5, 1.2)
        self.assertEqual((paused["pwm"], paused["phase"]), (1500, "waiting_for_feedback"))
        for i in range(13, 23):
            resumed = sample(i, i/10, i/10)
        self.assertGreater(resumed["pwm"], 1500)
        self.assertEqual(resumed["passed"], 0)

    def test_two_distinct_passes_require_lane_return(self):
        cfg = read_json(CONFIG)
        engine = Hybrid(CONFIG, cfg["primary_camera"], cfg["secondary_camera"])
        image = track().tobytes()
        for i in range(3):
            engine.step("two_cones", image, i+1, i/10, None, 0, -1, i/10)
        controller = engine.cones.controller
        estimate_one(controller, .4, "secondary-first")
        self.assertEqual(controller.count, 1)
        _, second = estimate_one(controller, 1.4, "secondary-second")
        self.assertEqual(controller.count, 2)
        self.assertNotEqual(second["phase"], "complete")
        for i in range(23, 30):
            result = controller.step(lane(i/10), i/10)
        self.assertEqual(result["phase"], "complete")
        self.assertFalse(result["tail_clearance_verified"])

    def test_race_board_to_cones(self):
        board = fixture("start_outdoor_board_hand.jpg")
        clear = fixture("start_outdoor_clear.jpg")
        far = fixture("zebra_outdoor_0401.jpg")
        near = fixture("zebra_outdoor_0430.jpg")
        red, green = lamp(0), lamp(2)
        road = [board]*10 + [clear]*10 + [far]*5 + [near]*135 + [track()]*25 + [track(True)]*28
        primary = [red]*155 + [green]*(len(road)-155)
        rows, summary = replay("race", road, primary)
        states = [r["state"] for r in rows]
        expected = ["wait_board", "zebra_approach", "zebra_brake", "zebra_hold",
                    "traffic_approach", "traffic_brake", "traffic_wait_green", "two_cones"]
        self.assertEqual(list(dict.fromkeys(states)), expected, (set(states), rows[-1]["reason"]))
        self.assertTrue(any(r["zebra_detected"] and r["zebra_range_status"] == "beyond_far" for r in rows[20:25]))
        self.assertFalse(summary["completed"])
        cone = [r for r in rows if r["legacy_cone_phase"]]
        self.assertTrue(any(r["requested_throttle_us"] >= 1570 for r in cone), cone[-1])
        self.assertTrue(any(r["cones_visible"] for r in cone), cone[-1])
        self.assertTrue(all(r["cones_passed"] == 0 for r in cone))


if __name__ == "__main__":
    unittest.main()
