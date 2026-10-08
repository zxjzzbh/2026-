"""Hardware-free checks of the integrated runtime and its failure boundaries."""
import copy
import io
import json
import math
import struct
import time
from dataclasses import asdict, replace
from pathlib import Path

import pytest
import cv2
import numpy as np

from carvision.autonomy_audio import I2CTTS, RealAnnouncement, tts_frames
from carvision.autonomy_feed import LocalFeedbackFile
from carvision.autonomy_geometry import GroundFeatureExtractor
from carvision.autonomy_motion import AutonomousS3S4Driver, MetricMotionController
from carvision.autonomy_observation import ConeTracker, DualCameraObservationBuilder, EncoderFeedback, inside_wheels
from carvision.autonomy_profile import AutonomyProfile, GroundProjection, fit_ground_calibration
from carvision.autonomy_runtime import AutonomyRuntime, LatestInput, replay_envelope, run_to_directory
from carvision.race import Observation, RaceConfig, Phase
from carvision.race_workflows import demonstration
from carvision.ground_odometry import GroundVisualOdometry, OpticalFeedback


ROOT = Path(__file__).resolve().parents[1]


def measured_profile():
    # Synthetic calibration, solely for exercising software. Never deployed.
    data = json.loads((ROOT / "configs/autonomy-pi5.json").read_text(encoding="utf-8"))
    # Keep synthetic geometry independent of newly surveyed real dimensions.
    data["vehicle"].pop("front_to_rear_axle_m", None)
    data["vehicle"].pop("length_m", None)
    data["vehicle"].update(measured=True, width_m=.19, wheelbase_m=.25, track_width_m=.15,
                           front_overhang_m=.05, rear_overhang_m=.07, tyre_width_m=.035, tyre_length_m=.06)
    data["motion"].update(calibrated=True, continuous_motion_verified=True,
                          max_speed_mps=.25, max_steering_rad=.35, speed_kp_us_per_mps=40,
                          speed_ki_us_per_m=20, braking_deceleration_mps2=.5,
                          speed_table=[{"speed_mps": .1, "pulse_us": 1540}, {"speed_mps": .25, "pulse_us": 1575}])
    data["telemetry"].update(verified=True, pose_verified=True, distance_per_count_m=.001, maximum_speed_mps=.5)
    data["course"].update(verified=True, traffic_stop_line_world_m=[[-.6, 10], [.6, 10]])
    data["audio"].update(verified=True, busy_status=1, idle_status=0)
    for camera in data["cameras"].values():
        camera.update(verified=True, detector_verified=True,
                      detector_model_sha256="0" * 64, monitored_region_m=[[-.6, .4], [.6, .4], [.6, 2], [-.6, 2]],
                      homography=[[.003, 0, -.96], [0, -.005, 2.4], [0, 0, 1]],
                      valid_polygon_px=[[0, 0], [639, 0], [639, 479], [0, 479]])
    return AutonomyProfile(data)


class Clock:
    def __init__(self, now=10):
        self.now = now
    def __call__(self):
        return self.now


class Adapter:
    def __init__(self, clock=None):
        self.calls = []
        self.clock = clock
        self.fail_channel = None
        self.delay = 0
    def open(self):
        self.calls.append("open")
    def send(self, function, payload):
        _, _, _, channel, pulse = struct.unpack("<BHBBH", payload)
        self.calls.append((channel, pulse))
        if self.clock:
            self.clock.now += self.delay
        if channel == self.fail_channel:
            raise OSError("injected UART failure")
    def close(self):
        self.calls.append("close")


def observation(t=10, **kwargs):
    data = dict(source_ok=True, telemetry_valid=True, telemetry_age_s=0,
                speed_mps=.1, lane_valid=True, lane_offset=0, lane_width_m=1.22,
                task_monitor_valid=True, cone_monitor_valid=True)
    data.update(kwargs)
    return Observation(t, **data)


def move(**kwargs):
    return dict(action="follow_lane", speed_mps=.1, steering_normalized=0, **kwargs)


def test_real_profile_remains_incomplete_and_never_opens_adapter():
    profile = AutonomyProfile.load(ROOT / "configs/autonomy-pi5.json")
    assert not profile.report()["actuation_mapping_ready"]
    adapter = Adapter()
    driver = AutonomousS3S4Driver(profile, adapter=adapter)
    driver.open()
    driver.apply({"motion_planned": False})
    driver.close()
    assert adapter.calls == []
    with pytest.raises(ValueError, match="blocked"):
        AutonomousS3S4Driver(profile, run=True, adapter=adapter)
    assert adapter.calls == []


@pytest.mark.parametrize("field,value", [("calibrated", 1), ("max_speed_mps", .26), ("steering_center_us", 1800), ("max_steering_rad", math.nan)])
def test_profile_rejects_invalid_or_outside_reviewed_range(field, value):
    data = copy.deepcopy(measured_profile().data)
    data["motion"][field] = value
    with pytest.raises(ValueError):
        AutonomyProfile(data)


def test_metric_speed_mapping_uses_feedback_not_full_discrete_throttle():
    profile = measured_profile()
    controller = MetricMotionController(profile)
    low = controller.plan(move(), observation())
    assert low["esc_us"] == 1540 and low["motion_planned"]
    correction = controller.plan(move(), observation(10.1, speed_mps=.08))
    assert 1540 <= correction["esc_us"] < 1575
    assert controller.plan({"action": "stop"}, observation(10.2))["esc_us"] == 1500


@pytest.mark.parametrize("changes,reason", [({"telemetry_valid": False}, "fresh_measured"), ({"telemetry_age_s": .251}, "fresh_measured"), ({"cone_monitor_valid": False}, "visibility"), ({"cones": [{"lateral_m": 0, "forward_m": .45, "radius_m": .04}]}, "hits_obstacle"), ({"lane_width_m": .3}, "leaves_measured_lane")])
def test_motion_refuses_missing_feedback_or_swept_collision(changes, reason):
    plan = MetricMotionController(measured_profile()).plan(move(), observation(**changes))
    assert not plan["motion_planned"] and reason in plan["reason"]


def test_fractional_velocity_is_not_converted_to_discrete_throttle():
    controller = MetricMotionController(measured_profile())
    assert controller.plan(dict(action="follow_lane", speed_mps=.04, steering_normalized=0), observation())["reason"] == "target_outside_measured_speed_table"
    with pytest.raises(ValueError):
        controller.plan(dict(action="follow_lane", speed_mps=.3, steering_normalized=0), observation())


def test_driver_watchdog_expires_without_another_apply_and_latches():
    clock, adapter = Clock(), Adapter()
    driver = AutonomousS3S4Driver(measured_profile(), run=True, adapter=adapter, clock=clock)
    driver.open()
    driver.apply({"motion_planned": True, "esc_us": 1540, "steering_us": 1650})
    clock.now += .26
    deadline = time.monotonic() + 1
    while driver.fault is None and time.monotonic() < deadline:
        time.sleep(.01)
    assert driver.fault == "command_lease_expired"
    assert (4, 1500) in adapter.calls[-2:]
    with pytest.raises(RuntimeError):
        driver.apply({"motion_planned": False})
    driver.close()
    assert adapter.calls[-1] == "close"
    with pytest.raises(RuntimeError):
        driver.open()


def test_slow_steering_write_cannot_be_followed_by_stale_throttle():
    clock = Clock()
    adapter = Adapter(clock)
    driver = AutonomousS3S4Driver(measured_profile(), run=True, adapter=adapter, clock=clock)
    driver.open()
    adapter.delay = .26
    with pytest.raises(RuntimeError, match="lease"):
        driver.apply({"motion_planned": True, "esc_us": 1540, "steering_us": 1660})
    assert (4, 1540) not in adapter.calls
    driver.close()


def test_failed_steering_does_not_prevent_neutral_or_close():
    adapter = Adapter()
    driver = AutonomousS3S4Driver(measured_profile(), run=True, adapter=adapter)
    driver.open()
    adapter.fail_channel = 3
    with pytest.raises(OSError):
        driver.apply({"motion_planned": True, "esc_us": 1540, "steering_us": 1660})
    assert adapter.calls[-2:] == [(4, 1500), (3, 1650)]
    errors = driver.close()
    assert errors and adapter.calls[-1] == "close"


def test_driver_total_run_limit_is_not_extended_by_updates():
    profile, clock, adapter = measured_profile(), Clock(), Adapter()
    profile.data["motion"]["max_run_s"] = .2
    driver = AutonomousS3S4Driver(profile, run=True, adapter=adapter, clock=clock)
    driver.open()
    clock.now += .21
    with pytest.raises(RuntimeError, match="total_autonomous"):
        driver.apply({"motion_planned": False})
    driver.close()


def test_projection_rejects_horizon_region_resolution_or_pose_change():
    camera = measured_profile().data["cameras"]["secondary"]
    projection = GroundProjection(camera)
    assert projection.point([320, 380]) == pytest.approx([0, .5])
    with pytest.raises(ValueError, match="outside"):
        projection.point([320, -1])
    with pytest.raises(ValueError, match="resolution"):
        projection.check_frame(dict(image_size=[1280, 720], pose_us=[1650, 1150]))
    with pytest.raises(ValueError, match="pose"):
        projection.check_frame(dict(image_size=[640, 480], pose_us=[1640, 1150]))


def test_calibration_fit_reports_independent_error_without_authorizing_hardware():
    data = dict(image_size=[640, 480], pose_us=None, valid_polygon_px=[[0, 0], [639, 0], [639, 479], [0, 479]],
                points=[dict(pixel=[x, y], ground_m=[x / 100, y / 100]) for x, y in [(10, 10), (500, 10), (500, 400), (10, 400)]],
                check_points=[dict(pixel=[100, 100], ground_m=[1, 1]), dict(pixel=[200, 200], ground_m=[2, 2])])
    result = fit_ground_calibration(data)
    assert not result["verified"] and result["independent_check_max_error_m"] < 1e-6
    data["check_points"][0] = data["points"][0]
    with pytest.raises(ValueError, match="independent"):
        fit_ground_calibration(data)


def encoder(seq, t, count):
    return dict(source="encoder", sequence=seq, captured_monotonic_s=t, count=count)


def test_encoder_needs_two_fresh_real_samples_and_rejects_duplicates_jumps():
    feedback = EncoderFeedback(measured_profile())
    assert not feedback.update(encoder(0, 10, 0), 10)["telemetry_valid"]
    result = feedback.update(encoder(1, 10.1, 20), 10.12)
    assert result["speed_mps"] == pytest.approx(.2)
    assert result["telemetry_age_s"] == pytest.approx(.02)
    assert not feedback.update(encoder(1, 10.1, 20), 10.12)["telemetry_valid"]
    assert not feedback.update(encoder(2, 10.2, 2000), 10.2)["telemetry_valid"]
    assert not feedback.update(encoder(3, 10.3, 2000), 10.6)["telemetry_valid"]
    assert not feedback.update(encoder(4, 11, 2000), 10.8)["telemetry_valid"]


def camera_packet(profile, name, seq, now, lane=True, light="unknown"):
    return dict(frame_id=seq, captured_monotonic_s=now, image_size=[640, 480], pose_us=profile.data["cameras"][name]["pose_us"],
                perception=dict(detector_status="ok", lane=dict(valid=lane, left=[[100, 300]], right=[[540, 300]], target=[320, 300]), presence={"blue_board": "present"}, traffic_light_state=light),
                ground_features=dict(board_roi_visible=True, task_region_visible=True, obstacle_region_visible=True,
                                     crosswalk_edge_px=[[160, 380], [480, 380]], traffic_stop_line_px=[[160, 300], [480, 300]], cones=[]))


def test_stop_distance_uses_direct_front_measurement_without_moving_axle_stop_line():
    profile = measured_profile()
    profile.data["vehicle"]["front_to_rear_axle_m"] = .34
    packet = {"primary": camera_packet(profile, "primary", 0, 10)}
    obs = DualCameraObservationBuilder(profile).build(packet, 10)["observation"]
    assert obs["crosswalk_distance_m"] == pytest.approx(.16)
    assert obs["traffic_stop_distance_m"] == pytest.approx(.65)


def test_sweep_checks_measured_front_and_larger_rear_envelope():
    profile = measured_profile()
    ahead = observation(cones=[{"lateral_m": 0, "forward_m": .81, "radius_m": .01}])
    assert MetricMotionController(profile).plan(move(), ahead)["motion_planned"]
    profile.data["vehicle"]["front_to_rear_axle_m"] = .34
    assert "hits_obstacle" in MetricMotionController(profile).plan(move(), ahead)["reason"]
    behind = observation(cones=[{"lateral_m": 0, "forward_m": -.167, "radius_m": .01}])
    assert MetricMotionController(profile).plan(move(), behind)["motion_planned"]
    profile.data["vehicle"]["length_m"] = .42
    assert "hits_obstacle" in MetricMotionController(profile).plan(move(), behind)["reason"]


@pytest.mark.parametrize("field,value", [("front_to_rear_axle_m", .20), ("front_to_rear_axle_m", True), ("length_m", .2)])
def test_profile_refuses_impossible_direct_body_measurements(field, value):
    data = copy.deepcopy(measured_profile().data)
    data["vehicle"][field] = value
    with pytest.raises(ValueError):
        AutonomyProfile(data)


def test_dual_camera_main_priority_aux_fallback_and_conflicting_light_unknown():
    profile = measured_profile()
    builder = DualCameraObservationBuilder(profile)
    packet = dict(primary=camera_packet(profile, "primary", 1, 10, lane=False, light="green"), secondary=camera_packet(profile, "secondary", 1, 10, light="red"))
    row = builder.build(packet, 10)
    assert row["measurement_sources"]["lane"] == "secondary"
    assert row["observation"]["traffic_light_state"] == "unknown"
    assert row["observation"]["crosswalk_distance_m"] == pytest.approx(.2)
    packet["primary"] = camera_packet(profile, "primary", 2, 10.1)
    packet["secondary"] = camera_packet(profile, "secondary", 2, 10.1)
    assert builder.build(packet, 10.1)["measurement_sources"]["lane"] == "primary"


def test_duplicate_frame_cannot_refresh_source_or_board_absence():
    profile = measured_profile()
    builder = DualCameraObservationBuilder(profile)
    frame = camera_packet(profile, "primary", 1, 10)
    builder.build(dict(primary=frame), 10)
    frame["perception"]["presence"]["blue_board"] = "absent"
    row = builder.build(dict(primary=frame), 10.05)
    assert not row["observation"]["source_ok"]
    assert row["observation"]["start_board_state"] == "unknown"


def test_unverified_detector_cannot_become_live_calibrated_geometry():
    profile = measured_profile()
    profile.data["cameras"]["primary"]["detector_verified"] = False
    row = DualCameraObservationBuilder(profile).build(dict(primary=camera_packet(profile, "primary", 1, 10)), 10)
    assert not row["observation"]["source_ok"]


def test_wheel_centres_inside_but_tyre_edges_outside_is_not_four_wheels():
    vehicle = measured_profile().data["vehicle"]
    too_narrow = [[-.08, -.05], [.08, -.05], [.08, .3], [-.08, .3]]
    assert inside_wheels(too_narrow, vehicle) == 0
    full = [[-.12, -.05], [.12, -.05], [.12, .3], [-.12, .3]]
    assert inside_wheels(full, vehicle) == 4
    assert inside_wheels(full, {**vehicle, "measured": False}) == 0


def test_cone_ids_require_verified_pose_and_actual_passing_not_disappearance():
    tracker = ConeTracker()
    cone = dict(lateral_m=.3, forward_m=.8, radius_m=.04)
    assert tracker.update([cone], None, 10, .07) == []
    pose = dict(verified=True, x_m=0, y_m=0, yaw_rad=0, captured_monotonic_s=10, error_bound_m=.02)
    assert tracker.update([cone], pose, 10, .07) == []
    pose.update(captured_monotonic_s=10.1)
    assert tracker.update([], pose, 10.1, .07) == []
    pose.update(y_m=1.1, captured_monotonic_s=10.2)
    assert tracker.update([], pose, 10.2, .07) == ["cone-1"]


def test_integrated_full_flow_completes_decisions_but_missing_calibration_never_moves(tmp_path):
    profile = AutonomyProfile.load(ROOT / "configs/autonomy-pi5.json")
    runtime = AutonomyRuntime(RaceConfig(school="山东大学", team="启航队", vehicle_width_m=.19), profile)
    summary = run_to_directory(runtime, (replay_envelope(obs, i) for i, obs in enumerate(demonstration())), tmp_path / "run")
    assert summary["decision_flow_completed"]
    assert not summary["actuation_mapping_ready"]
    assert not summary["hardware_output"] and summary["hardware_motion_updates"] == 0
    assert not summary["physical_race_completed"]
    logs = [json.loads(line) for line in (tmp_path / "run/decisions.jsonl").read_text(encoding="utf-8").splitlines()]
    assert any(row["audio"]["done"] for row in logs)
    assert runtime.closed and runtime.driver.closed


@pytest.mark.parametrize("change", ["future", "old", "duplicate", "mode", "clock", "bad_boolean", "old_speed"])
def test_live_runtime_rejects_bad_input_and_cannot_resume(change):
    clock = Clock()
    runtime = AutonomyRuntime(RaceConfig(school="山东大学", team="启航队", vehicle_width_m=.19), measured_profile(), live=True, clock=clock)
    row = replay_envelope(observation(), 1)
    row["mode"] = "live"
    row["source_oldest_monotonic_s"] = 10
    runtime.step(copy.deepcopy(row))
    row["sequence"] = 2
    row["captured_monotonic_s"] = row["observation"]["t_s"] = 10.1
    row["source_oldest_monotonic_s"] = 10.1
    clock.now = 10.1
    if change == "future":
        row["captured_monotonic_s"] = row["observation"]["t_s"] = 11
    elif change == "old":
        clock.now = 10.4
    elif change == "duplicate":
        row["sequence"] = 1
    elif change == "mode":
        row["mode"] = "replay"
    elif change == "clock":
        row["observation"]["t_s"] = 10.11
    elif change == "bad_boolean":
        row["observation"]["source_ok"] = 1
    elif change == "old_speed":
        row["observation"]["telemetry_age_s"] = .26
    with pytest.raises((ValueError, RuntimeError)):
        runtime.step(row)
    assert runtime.fault and runtime.driver.fault
    with pytest.raises(RuntimeError, match="fault-latched"):
        runtime.step(copy.deepcopy(row))
    runtime.close()


def test_runtime_ignores_fake_audio_acknowledgment():
    runtime = AutonomyRuntime(RaceConfig(school="山东大学", team="启航队", vehicle_width_m=.19), measured_profile())
    runtime.controller.phase = Phase.CROSSWALK_HOLD
    runtime.controller.crosswalk_since = 0
    runtime.controller.announcement_requested = True
    row = replay_envelope(observation(10, speed_mps=0, crosswalk_distance_m=.2, announcement_done=True), 1)
    result = runtime.step(row)
    assert result["decision"]["phase"] == Phase.CROSSWALK_HOLD.value
    assert not runtime.controller.announcement_acknowledged
    runtime.close()


def test_live_input_timeout_always_closes_and_records_failure(tmp_path):
    clock = Clock()
    runtime = AutonomyRuntime(RaceConfig(school="山东大学", team="启航队", vehicle_width_m=.19), measured_profile(), live=True, clock=clock)
    def rows():
        yield None
        clock.now += .26
        yield None
    with pytest.raises(TimeoutError):
        run_to_directory(runtime, rows(), tmp_path / "timeout")
    assert runtime.closed and runtime.driver.closed
    assert "250 ms" in json.loads((tmp_path / "timeout/summary.json").read_text(encoding="utf-8"))["error"]


def test_live_parse_failure_not_hidden_by_latest_buffer_and_eof_cannot_start_motion():
    row = replay_envelope(observation(), 1)
    row.update(mode="live", source_oldest_monotonic_s=10)
    payload = json.dumps(row) + "\n"
    reader = LatestInput(io.StringIO(payload + 'not json\n'), clock=Clock())
    reader.thread.join(timeout=1)
    with pytest.raises(json.JSONDecodeError):
        reader.get()
    reader = LatestInput(io.StringIO(payload), clock=Clock())
    reader.thread.join(timeout=1)
    with pytest.raises(EOFError):
        reader.get()


def test_tts_whole_sentence_has_one_synthesis_command_and_no_chunk_pauses():
    sentence = "我是山东大学启航队的智能车，请为我加油！"
    frames = tts_frames(sentence)
    assert len(frames) == 1 and 33 < len(frames[0]) <= 64
    prefix = b"[h0][v3]"
    recovered = "".join(frame[6 + len(prefix):].decode("gb2312") for frame in frames)
    assert recovered == sentence
    assert all(frame[:2] == b"\x00\xfd" for frame in frames)
    assert all(int.from_bytes(frame[2:4], "big") == len(frame) - 4 for frame in frames)


def test_raw_i2c_sends_whole_race_sentence_in_one_write(monkeypatch):
    packet = tts_frames("我是山东大学启航队的智能车，请为我加油！", 8)[0]
    writes = []
    def write(fd, data):
        writes.append((fd, data))
        return len(data)
    monkeypatch.setattr("carvision.autonomy_audio.os.write", write)
    transport = I2CTTS()
    transport.fd = 123
    transport.send(packet)
    assert writes == [(123, packet)]


def test_tts_oversized_sentence_is_rejected_instead_of_split():
    with pytest.raises(ValueError, match="single-message"):
        tts_frames("加油" * 20)


def test_tts_incomplete_write_is_never_accepted(monkeypatch):
    packet = tts_frames("我是山东大学启航队的智能车，请为我加油！", 8)[0]
    monkeypatch.setattr("carvision.autonomy_audio.os.write", lambda fd, data: len(data)-1)
    transport = I2CTTS()
    transport.fd = 123
    with pytest.raises(OSError, match="incomplete"):
        transport.send(packet)


class TTSFake:
    def __init__(self, statuses):
        self.statuses = iter(statuses)
        self.sent = []
        self.closed = False
    def open(self):
        pass
    def status(self):
        return next(self.statuses)
    def send(self, frame):
        self.sent.append(frame)
    def close(self):
        self.closed = True


def test_real_tts_cannot_ack_without_busy_then_idle_for_whole_sentence():
    config = measured_profile().data["audio"]
    sentence = "我是山东大学启航队的智能车，请为我加油！"
    frames = tts_frames(sentence)
    transport, clock = TTSFake([0] + [s for _ in frames for s in (1, 0)]), Clock()
    audio = RealAnnouncement(config, sentence, transport=transport, clock=clock)
    audio.start(sentence)
    assert not audio.status()["done"]
    for _ in range(len(frames) * 2):
        clock.now += .1
        audio.poll()
    assert audio.status()["done"] and len(transport.sent) == len(frames) and transport.closed


@pytest.mark.parametrize("statuses,advance", [([0, 0], .8), ([0, 9], .1), ([0, 1], 21)])
def test_tts_idle_only_unknown_status_and_timeout_fail(statuses, advance):
    clock, transport = Clock(), TTSFake(statuses)
    audio = RealAnnouncement(measured_profile().data["audio"], "你好", transport=transport, clock=clock)
    audio.start("你好")
    clock.now += advance
    status = audio.poll()
    assert status["failed"] and not status["done"] and transport.closed


def test_unverified_audio_cannot_be_enabled_by_observation_flag():
    config = copy.deepcopy(measured_profile().data["audio"])
    config["verified"] = False
    with pytest.raises(ValueError, match="verified"):
        RealAnnouncement(config, "你好")


def parking_image():
    image = np.full((480, 640, 3), 80, np.uint8)
    cv2.rectangle(image, (198, 80), (318, 280), (240, 240, 240), 4)
    cv2.rectangle(image, (322, 80), (442, 280), (240, 240, 240), 4)
    return image


def geometry_profile():
    profile = measured_profile()
    profile.data["cameras"]["primary"]["homography"] = [[.005, 0, -1.6], [0, -.005, 2.4], [0, 0, 1]]
    return profile


def rule_parking_image(mixed_lane_lines=False, ground=(80, 80, 80)):
    image = np.full((480, 640, 3), ground, np.uint8)
    # One metre deep, divided into two bays, with 10 cm yellow tape.
    yellow, white = (0, 220, 220), (240, 240, 240)
    for x in (198, 320, 442):
        cv2.line(image, (x, 80), (x, 280), yellow, 20)
    for y in (80, 280):
        cv2.line(image, (198, y), (442, y), white if mixed_lane_lines else yellow, 20)
    return image


@pytest.mark.parametrize("mixed_lane_lines", [False, True])
@pytest.mark.parametrize("blocked", ["left", "right"])
def test_rule_yellow_tape_and_white_lane_edges_find_both_parking_layouts(mixed_lane_lines, blocked):
    image = rule_parking_image(mixed_lane_lines)
    x = 230 if blocked == "left" else 354
    cv2.rectangle(image, (x, 140), (x + 40, 220), (200, 30, 30), -1)
    result = dict(pose_us=None, detections=[dict(label="blue_board", score=.9,
                                               bbox_xyxy=[x, 140, x + 40, 220])])
    slots = GroundFeatureExtractor(geometry_profile(), "primary").extract(image, result)["parking_slots"]
    assert len(slots) == 2
    assert {slot["id"]: slot["availability"] for slot in slots} == {
        "left": "blocked" if blocked == "left" else "clear",
        "right": "blocked" if blocked == "right" else "clear",
    }


def test_yellow_tape_does_not_authorize_unverified_red_track_or_incomplete_bays():
    image = rule_parking_image(ground=(60, 65, 145))
    extractor = GroundFeatureExtractor(geometry_profile(), "primary")
    slots = extractor.extract(image, dict(pose_us=None, detections=[]))["parking_slots"]
    assert len(slots) == 2
    assert all(slot["availability"] == "unknown" for slot in slots)
    cv2.rectangle(image, (185, 65), (455, 96), (60, 65, 145), -1)
    assert extractor.extract(image, dict(pose_us=None, detections=[]))["parking_slots"] == []


def test_real_geometry_baseline_finds_two_full_bays_and_blue_obstruction():
    extractor = GroundFeatureExtractor(geometry_profile(), "primary")
    image = parking_image()
    cv2.rectangle(image, (220, 140), (260, 220), (200, 30, 30), -1)
    result = dict(pose_us=None, detections=[dict(label="blue_board", score=.9, bbox_xyxy=[220, 140, 260, 220])])
    features = extractor.extract(image, result)
    assert len(features["parking_slots"]) == 2
    assert [slot["availability"] for slot in features["parking_slots"]] == ["blocked", "clear"]


def test_parking_occlusion_is_unknown_not_clear_and_incomplete_boundaries_refuse():
    extractor = GroundFeatureExtractor(geometry_profile(), "primary")
    image = parking_image()
    cv2.rectangle(image, (330, 100), (435, 260), (0, 0, 0), -1)
    features = extractor.extract(image, dict(pose_us=None, detections=[]))
    assert len(features["parking_slots"]) == 2
    assert features["parking_slots"][1]["availability"] == "unknown"
    cv2.rectangle(image, (190, 75), (450, 90), (80, 80, 80), -1)
    assert extractor.extract(image, dict(pose_us=None, detections=[]))["parking_slots"] == []


def test_traffic_lamp_box_is_never_a_ground_stop_line():
    image = parking_image()
    result = dict(pose_us=None, detections=[dict(label="traffic_light", score=.9, bbox_xyxy=[100, 20, 200, 100])])
    features = GroundFeatureExtractor(geometry_profile(), "primary").extract(image, result)
    assert "traffic_stop_line_px" not in features


def test_surveyed_traffic_line_needs_fresh_verified_pose():
    profile = geometry_profile()
    profile.data["course"]["traffic_stop_line_world_m"] = [[-.5, 1], [.5, 1]]
    extractor = GroundFeatureExtractor(profile, "primary")
    pose = dict(verified=True, x_m=0, y_m=0, yaw_rad=0, captured_monotonic_s=10, error_bound_m=.02)
    result = dict(pose_us=None, detections=[])
    assert "traffic_stop_line_px" in extractor.extract(parking_image(), result, pose=pose, now=10)
    assert "traffic_stop_line_px" not in extractor.extract(parking_image(), result, pose=pose, now=10.3)


def test_missing_camera_calibration_or_dark_frame_yields_unknown():
    profile = AutonomyProfile.load(ROOT / "configs/autonomy-pi5.json")
    out = GroundFeatureExtractor(profile, "primary").extract(parking_image(), {})
    assert not out["task_region_visible"] and out["parking_slots"] == []
    out = GroundFeatureExtractor(geometry_profile(), "primary").extract(np.zeros((480, 640, 3), np.uint8), dict(pose_us=None))
    assert not out["board_roi_visible"]


def test_cached_primary_remains_priority_when_aux_delivers_new_frame():
    profile = measured_profile()
    builder = DualCameraObservationBuilder(profile)
    primary = camera_packet(profile, "primary", 1, 10)
    builder.build(dict(primary=primary), 10)
    row = builder.build(dict(primary=primary, secondary=camera_packet(profile, "secondary", 1, 10.1)), 10.1)
    assert row["measurement_sources"]["lane"] == "primary"
    assert row["source_oldest_monotonic_s"] == 10


def test_live_camera_age_includes_queue_delay():
    clock = Clock(10.2)
    runtime = AutonomyRuntime(RaceConfig(school="山东大学", team="启航队", vehicle_width_m=.19), measured_profile(), live=True, clock=clock)
    row = replay_envelope(observation(10.2), 1)
    row.update(mode="live", source_oldest_monotonic_s=9.94)
    with pytest.raises(ValueError, match="camera measurement expired"):
        runtime.step(row)
    runtime.close()


def test_local_feedback_missing_is_unknown_and_malformed_is_not_ignored(tmp_path):
    path = tmp_path / "feedback.json"
    assert LocalFeedbackFile(path).read() == {}
    path.write_text("broken", encoding="utf-8")
    with pytest.raises(json.JSONDecodeError):
        LocalFeedbackFile(path).read()
    path.write_text("[]", encoding="utf-8")
    with pytest.raises(ValueError):
        LocalFeedbackFile(path).read()


def visual_profile():
    profile = measured_profile()
    profile.data["telemetry"].update(source="optical_ground", visual_error_bound_m=.02, origin_pose=dict(x_m=0, y_m=0, yaw_rad=0))
    return profile


def textured_image():
    rng = np.random.default_rng(731)
    image = np.full((480, 640, 3), 80, np.uint8)
    for x, y in rng.integers([30, 30], [610, 440], (180, 2)):
        cv2.circle(image, (int(x), int(y)), 3, (230, 230, 230), -1)
    return image


def test_ground_optical_flow_measures_speed_and_pose_from_pixels():
    odometry = GroundVisualOdometry(visual_profile())
    image = textured_image()
    info = dict(image_size=[640, 480], pose_us=None, detections=[])
    assert not odometry.update(image, 10, info)["valid"]
    moved = cv2.warpAffine(image, np.array([[1., 0, 0], [0, 1, 4]]), (640, 480))
    result = odometry.update(moved, 10.1, info)
    assert result["valid"] and result["speed_mps"] == pytest.approx(.2, abs=.015)
    assert result["pose"]["y_m"] == pytest.approx(.02, abs=.002)


def test_visual_stationary_feedback_needs_texture_and_missing_texture_never_reports_zero():
    odometry = GroundVisualOdometry(visual_profile())
    image = textured_image()
    info = dict(image_size=[640, 480], pose_us=None, detections=[])
    odometry.update(image, 10, info)
    stationary = odometry.update(image, 10.1, info)
    assert stationary["valid"] and abs(stationary["speed_mps"]) < .001
    lost = odometry.update(np.zeros_like(image), 10.2, info)
    assert not lost["valid"] and lost["speed_mps"] is None


def test_visual_gaps_invalidate_world_pose_and_large_motion_refuses_speed():
    odometry = GroundVisualOdometry(visual_profile())
    image = textured_image()
    info = dict(image_size=[640, 480], pose_us=None, detections=[])
    odometry.update(image, 10, info)
    assert not odometry.update(image, 10.3, info)["valid"]
    assert odometry.update(image, 10.4, info)["pose"] is None
    moved = cv2.warpAffine(image, np.array([[1., 0, 0], [0, 1, 20]]), (640, 480))
    assert not odometry.update(moved, 10.5, info)["valid"]


def test_optical_feedback_rejects_repeated_or_stale_measurements():
    feedback = OpticalFeedback(visual_profile())
    sample = dict(source="optical_ground", valid=True, sequence=1, captured_monotonic_s=10, speed_mps=.2)
    assert feedback.update(sample, 10.1)["telemetry_valid"]
    assert not feedback.update(sample, 10.1)["telemetry_valid"]
    sample.update(sequence=2, captured_monotonic_s=10.2)
    assert not feedback.update(sample, 10.46)["telemetry_valid"]


def test_invalid_plan_cannot_send_a_steering_command_before_validation():
    adapter = Adapter()
    driver = AutonomousS3S4Driver(measured_profile(), run=True, adapter=adapter)
    driver.open()
    with pytest.raises(ValueError):
        driver.apply(dict(motion_planned=True, steering_us=1700, esc_us=1800))
    assert (3, 1700) not in adapter.calls
    driver.close()


def test_dry_runtime_rejects_injected_real_driver_without_touching_it():
    profile, adapter = measured_profile(), Adapter()
    driver = AutonomousS3S4Driver(profile, run=True, adapter=adapter)
    with pytest.raises(ValueError, match="match runtime"):
        AutonomyRuntime(RaceConfig(school="山东大学", team="启航队", vehicle_width_m=.19), profile, driver=driver)
    assert adapter.calls == []


def test_existing_log_folder_does_not_leave_executor_open(tmp_path):
    runtime = AutonomyRuntime(RaceConfig(school="山东大学", team="启航队", vehicle_width_m=.19), measured_profile())
    with pytest.raises(FileExistsError):
        run_to_directory(runtime, [], tmp_path)
    assert runtime.closed and runtime.driver.closed


def test_live_real_mode_handover_waits_for_measured_stop_and_exclusive_uart():
    profile, clock, adapter = measured_profile(), Clock(), Adapter()
    driver = AutonomousS3S4Driver(profile, run=True, adapter=adapter, clock=clock)
    audio = RealAnnouncement(profile.data["audio"], "我是山东大学启航队的智能车，请为我加油！", transport=TTSFake([0]), clock=clock)
    runtime = AutonomyRuntime(RaceConfig(school="山东大学", team="启航队", vehicle_width_m=.19), profile, run=True, live=True, clock=clock, driver=driver, audio=audio)
    first = replay_envelope(observation(start_line_crossed=True, remote={"authenticated": True, "verified_5g": True, "deadman": True, "command_age_s": 0, "speed_mps": .1, "steering_normalized": 0}), 1)
    first.update(mode="live", source_oldest_monotonic_s=10)
    assert runtime.step(first)["execution"]["hardware_output"] is False
    assert adapter.calls == []
    clock.now = 10.1
    second = replay_envelope(observation(10.1, speed_mps=0, in_switch_zone=True, enable_autonomy=True, board_monitor_valid=True, start_board_state="present"), 2)
    second.update(mode="live", source_oldest_monotonic_s=10.1)
    assert runtime.step(second)["decision"]["phase"] == Phase.BOARD.value
    assert adapter.calls[:3] == ["open", (4, 1500), (3, 1650)]
    assert (4, 1540) not in adapter.calls and (4, 1575) not in adapter.calls
    runtime.close()


def test_wav_requires_hash_and_acknowledges_only_successful_player_exit(tmp_path):
    path = tmp_path / "verified.wav"
    path.write_bytes(b"test fixture audio, not a real recording")
    import hashlib
    config = dict(verified=True, backend="wav", timeout_s=20, wav_path=str(path), wav_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    class Process:
        def __init__(self):
            self.code = None
        def poll(self):
            return self.code
        def terminate(self):
            self.code = -15
        def wait(self, timeout):
            return self.code
    process = Process()
    audio = RealAnnouncement(config, "你好", popen=lambda *a, **k: process)
    audio.start("你好")
    assert not audio.poll()["done"]
    process.code = 0
    assert audio.poll()["done"]
    config["wav_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="WAV"):
        RealAnnouncement(config, "你好")


@pytest.mark.parametrize("defect", ["future", "duplicate", "bad_boolean"])
def test_bad_live_row_cannot_be_hidden_by_a_later_good_row(defect):
    first = replay_envelope(observation(), 1)
    first.update(mode="live", source_oldest_monotonic_s=10)
    bad = copy.deepcopy(first)
    bad["sequence"] = 2
    bad["captured_monotonic_s"] = bad["observation"]["t_s"] = 10.005
    if defect == "future":
        bad["captured_monotonic_s"] = bad["observation"]["t_s"] = 11
    elif defect == "duplicate":
        bad["sequence"] = 1
    else:
        bad["observation"]["source_ok"] = 1
    good = copy.deepcopy(first)
    good["sequence"] = 3
    good["captured_monotonic_s"] = good["observation"]["t_s"] = 10.01
    reader = LatestInput(io.StringIO("".join(json.dumps(row) + "\n" for row in (first, bad, good))), clock=Clock(10.02))
    reader.thread.join(timeout=1)
    with pytest.raises(ValueError):
        reader.get()
