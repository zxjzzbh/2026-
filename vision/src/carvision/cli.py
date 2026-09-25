import argparse
import json
import sys
from pathlib import Path

from .config import load_config
from .dataset import check_dataset
from .demo import generate_demo
from . import workflows


def parser():
    p = argparse.ArgumentParser(description="SmartCar perception and dataset tools (no motor control)")
    p.add_argument("--debug", action="store_true", help="show Python traceback on error")
    sub = p.add_subparsers(dest="command", required=True)
    d = sub.add_parser("doctor", help="report Python, OpenCV, YOLO and CUDA environment")
    d.add_argument("--output", help="optional JSON report file")
    d = sub.add_parser("demo", help="generate clearly marked synthetic lane test video")
    d.add_argument("--output", required=True)
    r = sub.add_parser("replay", help="run perception on an image, video or camera:N")
    r.add_argument("--source", required=True)
    r.add_argument("--output", required=True, help="new output folder; existing folders are never overwritten")
    r.add_argument("--config")
    r.add_argument("--weights", help="local race-trained YOLO model; omitted means detector disabled")
    r.add_argument("--device", help="override config, e.g. cpu or 0")
    r.add_argument("--show", action="store_true")
    r.add_argument("--save-video", action="store_true")
    r.add_argument("--max-frames", type=int)
    r.add_argument("--fallback-fps", type=float)
    s = sub.add_parser("serve", help="real-camera browser preview; no graphical desktop required")
    s.add_argument("--camera", type=int, required=True, help="verified camera index on this device")
    s.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 for trusted LAN viewing")
    s.add_argument("--port", type=int, default=8080)
    s.add_argument("--width", type=int)
    s.add_argument("--height", type=int)
    s.add_argument("--fps", type=float)
    s.add_argument("--preview-fps", type=float, default=10)
    s.add_argument("--preview-width", type=int, default=960)
    s.add_argument("--jpeg-quality", type=int, default=75)
    s.add_argument("--stale-ms", type=float, default=1500)
    s.add_argument("--config")
    s.add_argument("--weights")
    s.add_argument("--device", default="cpu")
    c = sub.add_parser("capture", help="explicitly open camera and record raw video")
    c.add_argument("--camera", type=int, default=0)
    c.add_argument("--output", required=True)
    c.add_argument("--seconds", type=float, default=30)
    c.add_argument("--width", type=int)
    c.add_argument("--height", type=int)
    c.add_argument("--fps", type=float, help="request camera FPS; support must be verified")
    c.add_argument("--file-fps", type=float, default=20, help="CFR AVI rate; host timestamps are saved separately")
    c.add_argument("--show", action="store_true")
    e = sub.add_parser("extract", help="sample original video frames for annotation")
    e.add_argument("--source", required=True)
    e.add_argument("--output", required=True)
    e.add_argument("--session", required=True, help="recording session id, e.g. day1-run1")
    e.add_argument("--interval-s", type=float, default=1)
    d = sub.add_parser("check-data", help="validate labels and train/validation recording separation")
    d.add_argument("--data", required=True)
    t = sub.add_parser("train", help="fine-tune on an explicitly labeled race dataset")
    t.add_argument("--data", required=True)
    t.add_argument("--base", required=True, help="trusted local weights or official yolov8n.pt")
    t.add_argument("--output", required=True)
    t.add_argument("--epochs", type=int, default=60)
    t.add_argument("--batch", type=int, default=8)
    t.add_argument("--imgsz", type=int, default=416)
    t.add_argument("--device", default="0")
    b = sub.add_parser("benchmark", help="test generic/race model latency; not detection accuracy")
    b.add_argument("--weights", required=True)
    b.add_argument("--image", required=True)
    b.add_argument("--output", required=True)
    b.add_argument("--device", default="cpu")
    b.add_argument("--imgsz", type=int, default=416)
    b.add_argument("--warmup", type=int, default=3)
    b.add_argument("--runs", type=int, default=20)
    e = sub.add_parser("export", help="export trained weights, then validate on target hardware")
    e.add_argument("--weights", required=True)
    e.add_argument("--format", choices=["onnx", "ncnn"], default="onnx")
    e.add_argument("--imgsz", type=int, default=416)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "doctor":
            result = workflows.doctor()
            if args.output:
                path = Path(args.output)
                path.parent.mkdir(parents=True, exist_ok=True)
                workflows.json_write(path, result)
        elif args.command == "demo":
            result = {"video": str(generate_demo(Path(args.output))), "synthetic": True}
        elif args.command == "replay":
            config = load_config(args.config)
            if args.device:
                config.yolo.device = args.device
            result = workflows.replay(args, config)
        elif args.command == "check-data":
            result, _ = check_dataset(args.data)
        elif args.command == "serve":
            from .web_preview import serve
            config = load_config(args.config)
            config.yolo.device = args.device
            result = serve(args, config)
        else:
            functions = {"capture": workflows.capture, "extract": workflows.extract_frames,
                         "train": workflows.train, "benchmark": workflows.benchmark,
                         "export": workflows.export_model}
            result = functions[args.command](args)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 2 if args.command == "check-data" and not result["valid"] else 0
    except KeyboardInterrupt:
        print("Stopped by user", file=sys.stderr)
        return 130
    except Exception as exc:
        if args.debug:
            raise
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
