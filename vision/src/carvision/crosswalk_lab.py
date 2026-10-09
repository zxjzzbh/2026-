"""Crosswalk-only software entry points. Never import or create an actuator.

replay: actual pixels, no invented distance or stationary feedback;
plan: timestamped Observation JSONL, for sensor-adapter integration offline.
All entries end with an explicit zero-speed intent, including input errors.
"""
import json
import time
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

from .config import load_config
from .crosswalk import CrosswalkDetector, load_crosswalk_config
from .display import overlay
from .lane import LaneDetector
from .race import Observation, Phase, RaceController, load_race_config
from .race_workflows import json_line
from .results import Detection, PerceptionResult
from .sources import open_source, write_image

VISION_ROOT = Path(__file__).resolve().parents[2]


def default_config(name):
    return str(VISION_ROOT / "configs" / name)


def json_write(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


class CrosswalkPerception:
    def __init__(self, lane_config, crosswalk_config, *, profile=None, camera_name="primary", pose_us=None):
        self.config = lane_config
        self.detector = CrosswalkDetector(crosswalk_config)
        self.lane = LaneDetector(lane_config.lane)
        self.projection = None
        self.pose_us = pose_us
        if profile:
            from .autonomy_profile import AutonomyProfile, GroundProjection, front_extent_m
            p = AutonomyProfile.load(profile)
            self.projection = GroundProjection(p.data["cameras"][camera_name])
            self.front = front_extent_m(p.data["vehicle"])
            if self.front is None:
                raise ValueError("profile is missing measured front extent")

    def process(self, frame, *, live=False):
        started = time.perf_counter()
        timestamp = frame.receive_monotonic_ms if live else frame.source_time_ms
        zebra, mask = self.detector.detect(frame.image, timestamp)
        # Exclude identified paper interiors from white-line sampling. Otherwise
        # the narrow gaps between stripes can masquerade as the driving corridor.
        lane_image = frame.image.copy()
        for x, y, w, h in zebra["stripes_xywh"]:
            lane_image[y:y+h, x:x+w] = 0
        lane, _ = self.lane.detect(lane_image)
        zebra["distance_estimate_m"] = None
        zebra["projection_status"] = "no_profile"
        if self.projection is not None:
            try:
                self.projection.check_frame({"image_size": [frame.image.shape[1], frame.image.shape[0]],
                                             "pose_us": self.pose_us})
                if zebra["candidate"] and zebra["bottom_clipped"]:
                    zebra["projection_status"] = "near_edge_clipped_no_metric_distance"
                elif zebra["candidate"]:
                    points = [self.projection.point([x + w / 2, y + h - 1])
                              for x, y, w, h in zebra["stripes_xywh"]]
                    zebra["distance_estimate_m"] = min(p[1] for p in points) - self.front
                    zebra["projection_status"] = "inside_calibration_detector_unverified"
                else:
                    zebra["projection_status"] = "no_candidate"
            except ValueError as exc:
                zebra["projection_status"] = str(exc)
        detections = ([Detection("crosswalk", zebra["score"], zebra["bbox_xyxy"])]
                      if zebra["candidate"] else [])
        result = PerceptionResult(frame.frame_id, frame.source_time_ms, frame.receive_monotonic_ms,
                                  [frame.image.shape[1], frame.image.shape[0]], lane,
                                  self.detector.status, detections, {"crosswalk": zebra["presence"]})
        result.processing_ms = (time.perf_counter() - started) * 1000
        return result, zebra, mask


def diagnostic_image(image, result, zebra, decision, roi_top):
    out = overlay(image, result, roi_top)
    for x, y, w, h in zebra["stripes_xywh"]:
        cv2.rectangle(out, (x, y), (x+w-1, y+h-1), (0, 220, 255), 1)
    labels = [f"CROSSWALK: {zebra['presence']} | strips {len(zebra['stripes_xywh'])}",
              f"{decision['phase']} | HOLD {decision['crosswalk_elapsed_s']:.1f}/{decision['crosswalk_hold_s']:.1f}s",
              f"INTENT {decision['speed_mps']:.2f} m/s | {decision['reason']}",
              "SOFTWARE ONLY - NO MOTOR / UART OUTPUT"]
    for i, label in enumerate(labels):
        y = 118 + i * 21
        cv2.putText(out, label, (9, y), cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 0, 0), 3)
        cv2.putText(out, label, (9, y), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
    return out


def _run(rows, config, output, *, input_kind, vision_config=None, lane_config=None, profile=None,
         camera_name="primary", pose_us=None, save_video=False, video_fps=10, show=False, origin=None):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    controller = RaceController(config, scope="crosswalk")
    perception = CrosswalkPerception(lane_config, vision_config, profile=profile,
                                    camera_name=camera_name, pose_us=pose_us) if vision_config else None
    writer = None
    counts = {"frames": 0, "resume_frames": 0, "candidate_frames": 0}
    transitions, timings = [], []
    sample_count = 0
    last_phase = None
    failure = None
    json_write(output / "config.json", {"race": asdict(config), "vision": asdict(vision_config) if vision_config else None,
                                        "input_kind": input_kind, "origin": origin, "hardware_output": False})
    try:
        with ((output / "decisions.jsonl").open("w", encoding="utf-8") as decisions,
              (output / "observations.jsonl").open("w", encoding="utf-8") as observations,
              (output / "perception.jsonl").open("w", encoding="utf-8") as perceptions):
            for item in rows:
                frame = item.get("frame")
                result = zebra = mask = None
                if frame is not None:
                    result, zebra, mask = perception.process(frame, live=input_kind == "camera")
                    counts["candidate_frames"] += zebra["candidate"]
                    timings.append(result.processing_ms)
                    json_line(perceptions, {"frame_id": frame.frame_id, "perception": result.to_dict(), "crosswalk": zebra})
                if "observation" in item:
                    values = dict(item["observation"])
                    obs = Observation.from_dict(values)
                else:
                    timestamp_s = (frame.receive_monotonic_ms / 1000 if input_kind == "camera"
                                   else (frame.source_time_ms or 0) / 1000)
                    obs = Observation(timestamp_s, source_ok=True, lane_valid=result.lane.valid,
                                      lane_offset=result.lane.offset_normalized, crosswalk_state=zebra["presence"])
                    # Real video contains neither measured speed nor an accepted
                    # near-field distance. Keep those fields unknown, not zero.
                decision = controller.update(obs)
                json_line(observations, asdict(obs))
                json_line(decisions, {"t_s": obs.t_s, "input_kind": input_kind, **decision})
                counts["frames"] += 1
                counts["resume_frames"] += decision["phase"] == Phase.CROSSWALK_EXIT.value and decision["action"] == "follow_lane"
                changed = decision["phase"] != last_phase
                if changed:
                    transitions.append((obs.t_s, decision["phase"], decision["reason"]))
                    last_phase = decision["phase"]
                if result is not None:
                    rendered = diagnostic_image(frame.image, result, zebra, decision, lane_config.lane.roi_top)
                    if counts["frames"] == 1 or (changed and sample_count < 8):
                        name = f"frame-{counts['frames']:05d}.jpg"
                        write_image(output / name, rendered)
                        sample_count += 1
                    write_image(output / "latest.jpg", rendered)
                    write_image(output / "crosswalk-mask.png", mask)
                    if save_video:
                        if writer is None:
                            writer = cv2.VideoWriter(str(output / "annotated.avi"), cv2.VideoWriter_fourcc(*"MJPG"),
                                                     video_fps, (rendered.shape[1], rendered.shape[0]))
                            if not writer.isOpened():
                                raise RuntimeError("cannot create MJPG video")
                        writer.write(rendered)
                    if show:
                        cv2.imshow("Crosswalk - software only", rendered)
                        cv2.imshow("Crosswalk white mask", mask)
                        wait_ms = (0 if input_kind != "camera" and frame.source_time_ms is None
                                   else 1 if input_kind == "camera" else max(1, round(1000 / video_fps)))
                        if cv2.waitKey(wait_ms) & 0xff in (27, ord('q')):
                            break
            if counts["frames"] == 0:
                raise ValueError("input contains no observations or frames")
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if writer is not None:
            writer.release()
        if show:
            cv2.destroyAllWindows()
        if hasattr(rows, "close"):
            rows.close()
        with (output / "decisions.jsonl").open("a", encoding="utf-8") as log:
            json_line(log, {"t_s": controller.last_t,
                            **controller.decision("stop", "input_error" if failure else "input_ended")})
        summary = {**counts, "input_kind": input_kind, "hardware_output": False,
                   "physical_task_verified": False, "hold_s": config.crosswalk_hold_s,
                   "announcement_required": config.announcement_required,
                   "crosswalk_served": controller.crosswalk_served, "phase": controller.phase.value,
                   "error": failure, "output": str(output),
                   "processing_p95_ms": float(np.percentile(timings, 95)) if timings else None}
        json_write(output / "summary.json", summary)
        json_write(output / "state-transitions.json", transitions)
    return summary


def _settings(args):
    return (load_race_config(args.race_config or default_config("crosswalk-3s.json")),
            load_config(getattr(args, "config", None)),
            load_crosswalk_config(getattr(args, "vision_config", None) or default_config("crosswalk-vision.json")))


def crosswalk_plan(args):
    config, _, _ = _settings(args)
    def rows():
        with Path(args.input).open(encoding="utf-8-sig") as stream:
            for line in stream:
                if line.strip():
                    yield {"observation": json.loads(line)}
    return _run(rows(), config, args.output, input_kind="observation_replay", origin=str(args.input))


def crosswalk_replay(args):
    config, lane, vision = _settings(args)
    if args.max_frames <= 0 or not 0 < args.seconds <= 180:
        raise ValueError("max-frames must be positive and seconds in (0,180]")
    live = args.source.startswith("camera:")
    source = open_source(args.source, fallback_fps=args.fallback_fps,
                         width=640, height=480, fps=10, fourcc="MJPG", decode_fps=10)
    def rows():
        start = time.monotonic()
        try:
            for count, frame in enumerate(source):
                if count >= args.max_frames or (live and time.monotonic() - start > args.seconds):
                    break
                yield {"frame": frame}
        finally:
            source.close()
    try:
        result = _run(rows(), config, args.output, input_kind="camera" if live else "image_or_video",
                      vision_config=vision, lane_config=lane, profile=args.profile, camera_name=args.camera_name,
                      pose_us=args.camera_pose_us, save_video=args.save_video, video_fps=10 if live else (source.fps or 10),
                      show=args.show, origin=args.source)
    finally:
        source.close()
    return result
