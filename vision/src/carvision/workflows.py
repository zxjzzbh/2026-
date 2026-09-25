import csv
import importlib.metadata
import json
import platform
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from . import __version__
from .dataset import check_dataset
from .detection import YoloDetector, import_yolo
from .display import overlay
from .pipeline import Pipeline
from .sources import CameraSource, FileSource, open_source, read_image, write_image


def json_write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def new_directory(path):
    path = Path(path).resolve()
    path.mkdir(parents=True, exist_ok=False)
    return path


def percentile_summary(values):
    return {"count": len(values), "p50_ms": float(np.percentile(values, 50)) if values else None,
            "p95_ms": float(np.percentile(values, 95)) if values else None,
            "mean_ms": statistics.mean(values) if values else None}


def doctor():
    info = {"python": sys.version, "executable": sys.executable, "platform": platform.platform(),
            "carvision": __version__}
    for package in ("numpy", "opencv-python", "ultralytics", "torch", "torchvision"):
        try:
            info[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            info[package] = "not installed"
    try:
        import torch
        info["cuda_available"] = torch.cuda.is_available()
        info["torch_cuda"] = torch.version.cuda
        if info["cuda_available"]:
            info["gpu"] = torch.cuda.get_device_name(0)
            info["vram_bytes"] = torch.cuda.get_device_properties(0).total_memory
    except ImportError:
        info["cuda_available"] = False
    return info


def replay(args, config):
    started = time.perf_counter()
    if args.max_frames is not None and args.max_frames <= 0:
        raise ValueError("max-frames must be positive")
    detector = YoloDetector(args.weights, config.yolo) if args.weights else None
    output = new_directory(args.output)
    json_write(output/"config.json", config.to_dict())
    source = None
    writer = None
    durations, result_ages, valid = [], [], 0
    summary = {"software_version": __version__, "input": args.source, "mode": "perception_only",
               "weights": args.weights, "status": "running", "frames": 0,
               "limitations": ["No calibrated metric distance or actuator commands",
                               "Lane/light algorithms require real-scene validation"]}
    try:
        source = open_source(args.source, fallback_fps=args.fallback_fps)
        summary["timestamp_basis"] = source.timestamp_basis
        pipeline = Pipeline(config, detector)
        live = isinstance(source, CameraSource)
        with (output/"results.jsonl").open("w", encoding="utf-8") as log:
            for frame in source:
                result, mask = pipeline.process(frame, live=live)
                image = overlay(frame.image, result, config.lane.roi_top)
                log.write(json.dumps(result.to_dict(), ensure_ascii=False, allow_nan=False)+"\n")
                log.flush()
                if summary["frames"] == 0:
                    write_image(output/"preview.jpg", image)
                    write_image(output/"lane-mask.png", mask)
                if args.save_video and not getattr(source, "is_image", False):
                    if writer is None:
                        fps = source.fps if source.fps and source.fps > 0 else 20
                        writer = cv2.VideoWriter(str(output/"overlay.avi"), cv2.VideoWriter_fourcc(*"MJPG"),
                                                 fps, (image.shape[1], image.shape[0]))
                        if not writer.isOpened():
                            raise RuntimeError("cannot create output video")
                        summary["output_video_fps"] = fps
                        summary["video_timing_note"] = "Preview is CFR; live dropped-frame timing is only in JSONL"
                    writer.write(image)
                durations.append(result.processing_ms)
                result_ages.append(result.receive_to_result_ms)
                valid += int(result.lane.valid)
                summary["frames"] += 1
                if args.show:
                    cv2.imshow("SmartCar perception", image)
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        summary["status"] = "user_stopped"
                        break
                if args.max_frames and summary["frames"] >= args.max_frames:
                    summary["status"] = "frame_limit"
                    break
        if summary["status"] == "running":
            summary["status"] = source.end_status
    except BaseException as exc:
        summary["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "error"
        summary["error"] = str(exc)
        raise
    finally:
        if source is not None:
            source.close()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()
        summary["lane_valid_frames"] = valid
        summary["processing"] = percentile_summary(durations)
        summary["host_receive_to_result"] = percentile_summary(result_ages)
        summary["elapsed_s"] = time.perf_counter()-started
        json_write(output/"summary.json", summary)
    return summary


def capture(args):
    """Explicit user-invoked capture; records unannotated frames and receipt times."""
    output = new_directory(args.output)
    camera = None
    writer = None
    info = {"status": "starting", "frames": 0, "camera_index": args.camera,
            "note": "timestamps are host receipt, not exposure; video is constant-rate preview"}
    if args.seconds <= 0 or args.file_fps <= 0:
        raise ValueError("seconds and file-fps must be positive")
    opening_started = time.monotonic()
    recording_started = None
    times = []
    try:
        camera = CameraSource(args.camera, args.width, args.height, args.fps)
        info["reported"] = camera.reported
        info["request_results"] = camera.request_results
        with (output/"timestamps.csv").open("w", encoding="utf-8", newline="") as f:
            log = csv.writer(f)
            log.writerow(["saved_frame", "capture_sequence", "host_receive_ms"])
            for frame in camera:
                if recording_started is None:
                    recording_started = time.monotonic()
                    info["startup_s"] = recording_started-opening_started
                if writer is None:
                    h, w = frame.image.shape[:2]
                    info["decoded_size"] = [w, h]
                    info["file_fps"] = args.file_fps
                    writer = cv2.VideoWriter(str(output/"raw.avi"), cv2.VideoWriter_fourcc(*"MJPG"),
                                             args.file_fps, (w, h))
                    if not writer.isOpened():
                        raise RuntimeError("cannot create capture video")
                    write_image(output/"first-frame.jpg", frame.image)
                writer.write(frame.image)
                log.writerow([info["frames"], frame.frame_id, frame.receive_monotonic_ms])
                times.append(frame.receive_monotonic_ms)
                info["frames"] += 1
                if args.show:
                    cv2.imshow("Recording raw camera - Q to finish", frame.image)
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        info["stop_reason"] = "user_stopped"
                        break
                if time.monotonic()-recording_started >= args.seconds:
                    info["stop_reason"] = "duration_reached"
                    break
        info["status"] = "completed"
    except BaseException as exc:
        info["status"] = "error"
        info["error"] = str(exc)
        raise
    finally:
        if camera:
            camera.close()
        if writer:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()
        info["measured_saved_fps"] = ((len(times)-1)*1000/(times[-1]-times[0])) if len(times)>1 else None
        info["recorded_span_s"] = (times[-1]-times[0])/1000 if len(times)>1 else 0.0
        json_write(output/"capture-info.json", info)
    return info


def extract_frames(args):
    if args.interval_s <= 0:
        raise ValueError("interval-s must be positive")
    if not args.session or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in args.session):
        raise ValueError("session must contain only letters, digits, underscore or hyphen")
    output = new_directory(Path(args.output)/args.session)
    source = FileSource(args.source)
    next_time, count = 0.0, 0
    try:
        with (output/"manifest.csv").open("w", newline="", encoding="utf-8") as f:
            manifest = csv.writer(f)
            manifest.writerow(["image", "session", "source", "frame_id", "media_ms", "annotation_status"])
            for frame in source:
                if frame.source_time_ms is None or frame.source_time_ms >= next_time:
                    filename = f"frame-{frame.frame_id:06d}.jpg"
                    write_image(output/filename, frame.image)
                    manifest.writerow([filename, args.session, str(Path(args.source).resolve()),
                                       frame.frame_id, frame.source_time_ms, "unreviewed"])
                    count += 1
                    if frame.source_time_ms is not None:
                        next_time = frame.source_time_ms+args.interval_s*1000
    finally:
        source.close()
    return {"images": count, "output": str(output), "status": "unlabeled"}


def train(args):
    if args.epochs <= 0 or args.batch <= 0 or args.imgsz < 32 or args.imgsz % 32:
        raise ValueError("epochs/batch must be positive; imgsz must be a multiple of 32")
    report, resolved = check_dataset(args.data)
    if not report["valid"]:
        raise ValueError("Dataset invalid:\n"+"\n".join(report["errors"][:30]))
    output = new_directory(args.output)
    dataset_path = output/"dataset-resolved.yaml"
    dataset_path.write_text(yaml.safe_dump(resolved, allow_unicode=True, sort_keys=False), encoding="utf-8")
    json_write(output/"data-check.json", report)
    model = import_yolo()(args.base)
    model.train(data=str(dataset_path), epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                device=args.device, workers=0, project=str(output), name="training", exist_ok=False,
                seed=42, deterministic=True, patience=15, cache=False, plots=True)
    return {"output": str(output/"training"), "note": "validate on held-out real sessions before deployment"}


def benchmark(args):
    if args.runs <= 0 or args.warmup < 0:
        raise ValueError("runs must be positive; warmup must be nonnegative")
    output = new_directory(args.output)
    model = import_yolo()(args.weights)
    image = read_image(args.image)
    import torch
    use_cuda = args.device != "cpu" and torch.cuda.is_available()
    durations = []
    for i in range(args.warmup+args.runs):
        if use_cuda:
            torch.cuda.synchronize()
        start = time.perf_counter()
        model.predict(image, imgsz=args.imgsz, device=args.device, verbose=False)
        if use_cuda:
            torch.cuda.synchronize()
        elapsed = (time.perf_counter()-start)*1000
        if i >= args.warmup:
            durations.append(elapsed)
    result = {"weights": args.weights, "device": args.device, "imgsz": args.imgsz,
              "timing": percentile_summary(durations), "environment": doctor(),
              "note": "single-image predict latency including pre/postprocess; not car FPS or accuracy"}
    json_write(output/"benchmark.json", result)
    return result


def export_model(args):
    path = Path(args.weights)
    if not path.is_file():
        raise FileNotFoundError(path)
    model = import_yolo()(str(path))
    # Keep the verified torch 2.6 / sympy 1.13 environment consistent. Newer
    # onnxslim versions require a conflicting sympy; graph slimming is optional.
    options = {"simplify": False} if args.format == "onnx" else {}
    result = model.export(format=args.format, imgsz=args.imgsz, device="cpu", **options)
    return {"export": str(result), "note": "verify exported model on target Pi; export may require extra packages"}
