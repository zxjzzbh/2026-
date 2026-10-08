"""Bounded local camera producer, with fresh real feedback from an atomic file.

This owns explicitly selected cameras; do not run it alongside the preview's
camera owners. Default run is ten seconds, at five perception updates/second.
It never opens the UART, moves the gimbal, or starts the vehicle.
"""
import hashlib
import json
import sys
import threading
import time
from pathlib import Path

from .autonomy_geometry import GroundFeatureExtractor
from .autonomy_observation import DualCameraObservationBuilder
from .autonomy_profile import AutonomyProfile
from .config import load_config
from .pipeline import Pipeline
from .race_workflows import json_line
from .sources import CameraSource


class LocalFeedbackFile:
    def __init__(self, path):
        self.path = Path(path)

    def read(self):
        # Writers must publish by atomic rename. Empty/missing samples remain
        # unknown; timestamps in each sensor record determine freshness.
        try:
            with self.path.open("rb") as stream:
                payload = stream.read(262145)
        except FileNotFoundError:
            return {}
        if len(payload) > 262144:
            raise ValueError("local feedback snapshot too large")
        value = json.loads(payload)
        if not isinstance(value, dict):
            raise ValueError("local feedback snapshot must be an object")
        return value


class PerceptionWorker:
    def __init__(self, name, selector, profile, config, detector, feedback, *, fps=5, source_factory=CameraSource):
        self.name, self.profile, self.feedback, self.fps = name, profile, feedback, fps
        self.pipeline = Pipeline(config, detector)
        self.geometry = GroundFeatureExtractor(profile, name)
        self.odometry = None
        if name == "primary" and profile.data["telemetry"]["source"] == "optical_ground":
            from .ground_odometry import GroundVisualOdometry
            self.odometry = GroundVisualOdometry(profile)
        width, height = profile.data["cameras"][name]["image_size"]
        self.source = source_factory(selector, width=width, height=height, fps=10, fourcc="MJPG", decode_fps=10, timeout_s=.25)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = self.error = None
        self.thread = threading.Thread(target=self._work, daemon=True, name="autonomy-camera-" + name)
        self.thread.start()

    def _work(self):
        next_processing = 0
        try:
            for frame in self.source:
                if self.stop.is_set():
                    break
                now = time.monotonic()
                if now < next_processing:
                    continue
                next_processing = now + 1 / self.fps
                sample = self.feedback.read()
                pose_us = sample.get("camera_pose_us", {}).get(self.name)
                result, _ = self.pipeline.process(frame, live=True)
                data = result.to_dict()
                data["pose_us"] = pose_us
                odometry = self.odometry.update(frame.image, frame.receive_monotonic_ms / 1000, data) if self.odometry else None
                if odometry and odometry.get("pose") is not None:
                    sample["pose"] = odometry["pose"]
                features = self.geometry.extract(frame.image, data, pose=sample.get("pose"), now=now)
                camera = {"frame_id": frame.frame_id, "captured_monotonic_s": frame.receive_monotonic_ms / 1000,
                          "image_size": data["image_size"], "pose_us": pose_us,
                          "perception": data, "ground_features": features, "odometry": odometry}
                with self.lock:
                    self.latest = camera
        except BaseException as exc:
            with self.lock:
                self.error = exc
        finally:
            self.source.close()

    def snapshot(self):
        with self.lock:
            return self.latest, self.error

    def close(self):
        self.stop.set()
        self.source.close()
        self.thread.join(timeout=1)


def autonomy_feed(args):
    if not 1 <= args.seconds <= 180 or not 5 <= args.fps <= 10:
        raise ValueError("camera feed must be bounded to 1..180 s and 5..10 Hz")
    profile = AutonomyProfile.load(args.profile)
    config = load_config(args.config)
    feedback = LocalFeedbackFile(args.feedback)
    workers = []
    try:
        for name, selector in (("primary", args.camera), ("secondary", args.aux_camera)):
            if selector is None:
                continue
            detector = None
            if args.weights:
                from .detection import YoloDetector
                digest = hashlib.sha256(Path(args.weights).read_bytes()).hexdigest()
                if profile.data["cameras"][name].get("detector_model_sha256") != digest:
                    raise ValueError(f"{name} weights are not the field-verified model")
                detector = YoloDetector(args.weights, config.yolo)
            workers.append(PerceptionWorker(name, selector, profile, config, detector, feedback, fps=args.fps))
        builder = DualCameraObservationBuilder(profile)
        end = time.monotonic() + args.seconds
        emitted = {}
        while time.monotonic() < end:
            packet = feedback.read()
            healthy = 0
            new_frame = False
            for worker in workers:
                camera, error = worker.snapshot()
                if error is None:
                    healthy += 1
                if camera is not None and error is None:
                    packet[worker.name] = camera
                    if worker.name == "primary" and camera.get("odometry") is not None:
                        packet["telemetry"] = camera["odometry"]
                        if camera["odometry"].get("pose") is not None:
                            packet["pose"] = camera["odometry"]["pose"]
                    if emitted.get(worker.name) != camera["frame_id"]:
                        new_frame = True
                        emitted[worker.name] = camera["frame_id"]
            if healthy == 0:
                raise RuntimeError("both explicitly selected camera workers failed")
            if new_frame:
                json_line(sys.stdout, builder.build(packet, time.monotonic()))
            time.sleep(.01)
    finally:
        for worker in workers:
            worker.close()
    return 0
