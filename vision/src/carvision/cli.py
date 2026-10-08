import argparse
import json
import sys
from pathlib import Path

from .config import load_config
from .dataset import check_dataset
from .demo import generate_demo
from . import workflows
from .cameras import camera_selector


def parser():
    p = argparse.ArgumentParser(description="SmartCar perception, 2026 race decisions and dataset tools")
    p.add_argument("--debug", action="store_true", help="show Python traceback on error")
    sub = p.add_subparsers(dest="command", required=True)
    d = sub.add_parser("doctor", help="report Python, OpenCV, YOLO and CUDA environment")
    d.add_argument("--output", help="optional JSON report file")
    sub.add_parser("cameras", help="read-only Linux camera inventory including metadata nodes and stable device paths")
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
    s.add_argument("--camera", type=camera_selector, required=True, help="verified index or stable /dev/v4l/by-id/by-path selector")
    s.add_argument("--aux-camera", type=camera_selector, help="second explicit USB video source, independent preview only")
    s.add_argument('--camera-format', choices=['MJPG','YUYV'],default='MJPG',help='verified USB format; MJPG reduces shared USB bandwidth')
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
    s.add_argument("--race-config", help="show race rules and QR cues; no motion output")
    s.add_argument("--classic-candidates", action="store_true", help="experimental color/shape cues without YOLO weights")
    d = sub.add_parser("race-demo", help="simulate the complete 2026 race; no hardware output")
    d.add_argument("--output", required=True, help="new output directory")
    d.add_argument("--race-config")
    d = sub.add_parser("race-plan", help="replay calibrated observation JSONL into decisions; no hardware output")
    d.add_argument("--input", required=True, help="observation JSONL file, or - for stdin")
    d.add_argument("--output", required=True, help="new output directory")
    d.add_argument("--race-config", required=True)
    d = sub.add_parser("race-check", help="show rule settings and actual-vehicle integration requirements")
    d.add_argument("--race-config", required=True)
    d = sub.add_parser("pi-check", help="check Pi-only PWM configuration without touching GPIO")
    d.add_argument("--pi-config", required=True)
    d = sub.add_parser('gimbal-check', help='check two direct-wired camera servos without GPIO output')
    d.add_argument('--gimbal-config', required=True)
    d = sub.add_parser('gimbal-test', help='brief calibrated camera-servo bench test; default has no output')
    d.add_argument('--gimbal-config', required=True)
    d.add_argument('--pan', type=float, default=0)
    d.add_argument('--tilt', type=float, default=0)
    d.add_argument('--seconds', type=float, default=1)
    d.add_argument('--run', action='store_true')
    d.add_argument('--acknowledge-motion', action='store_true')
    d = sub.add_parser("pi-run", help="Pi-only race executor; default is no-output replay")
    d.add_argument("--race-config", required=True)
    d.add_argument("--backend", choices=("hardware-pwm", "rasadapter5"), default="hardware-pwm",
                   help="select GPIO hardware PWM or RasAdapter S3/S4 runtime (currently no-output only)")
    d.add_argument("--pi-config", help="required for hardware-pwm; RasAdapter motion is not yet enabled")
    d.add_argument("--input", required=True, help="observation file for replay; real PWM only accepts live stdin (-)")
    d.add_argument("--output", required=True, help="new output directory")
    d.add_argument("--run", action="store_true", help="explicitly enable calibrated hardware PWM")
    d.add_argument("--acknowledge-motion", action="store_true", help="confirm prepared physical cutoff and raised drive wheels")
    d = sub.add_parser("autonomy-check", help="show measured Pi5 autonomous readiness without hardware access")
    d.add_argument("--profile", required=True)
    d = sub.add_parser("autonomy-demo", help="integrated rule/audio/motion replay; simulation only")
    d.add_argument("--profile", required=True)
    d.add_argument("--race-config", required=True)
    d.add_argument("--output", required=True)
    d = sub.add_parser("autonomy-run", help="versioned observation runtime; dry by default, calibrated S3/S4 live output explicit")
    d.add_argument("--profile", required=True)
    d.add_argument("--race-config", required=True)
    d.add_argument("--input", required=True, help="versioned envelope JSONL or local live stdin (-)")
    d.add_argument("--output", required=True)
    d.add_argument("--run", action="store_true")
    d.add_argument("--acknowledge-motion", action="store_true")
    d = sub.add_parser("autonomy-observe", help="local timestamped sensor JSONL to observations; no hardware commands")
    d.add_argument("--profile", required=True)
    d = sub.add_parser("ground-calibrate", help="fit measured ground points with independent check points; never auto-verify")
    d.add_argument("--input", required=True)
    d.add_argument("--output", required=True)
    d = sub.add_parser("autonomy-feed", help="bounded local dual-camera/feedback producer; no actuator commands")
    d.add_argument("--profile", required=True)
    d.add_argument("--camera", type=camera_selector, required=True)
    d.add_argument("--aux-camera", type=camera_selector)
    d.add_argument("--feedback", required=True, help="atomic local sensor/gateway JSON snapshot")
    d.add_argument("--config")
    d.add_argument("--weights", help="local field-verified race model")
    d.add_argument("--seconds", type=float, default=10)
    d.add_argument("--fps", type=float, default=5)
    c = sub.add_parser("capture", help="explicitly open camera and record raw video")
    c.add_argument("--camera", type=camera_selector, default=0)
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
        elif args.command == "cameras":
            from .cameras import camera_inventory
            result = camera_inventory()
        elif args.command in ('gimbal-check','gimbal-test'):
            from .gimbal import gimbal_check, gimbal_test
            result = {'gimbal-check':gimbal_check,'gimbal-test':gimbal_test}[args.command](args)
        elif args.command == "pi-check":
            from .pi_control import pi_check
            result = pi_check(args)
        elif args.command == "pi-run":
            from .pi_runtime import pi_run
            result = pi_run(args)
        elif args.command == "autonomy-check":
            from .autonomy_profile import AutonomyProfile
            result = AutonomyProfile.load(args.profile).report()
        elif args.command == "autonomy-observe":
            from .autonomy_runtime import autonomy_observe
            return autonomy_observe(args)
        elif args.command == "autonomy-feed":
            from .autonomy_feed import autonomy_feed
            return autonomy_feed(args)
        elif args.command in ("autonomy-run", "autonomy-demo", "ground-calibrate"):
            from .autonomy_runtime import autonomy_run, autonomy_demo, ground_calibrate
            result = {"autonomy-run": autonomy_run, "autonomy-demo": autonomy_demo, "ground-calibrate": ground_calibrate}[args.command](args)
        elif args.command in ("race-demo", "race-plan", "race-check"):
            from .race_workflows import race_demo, race_plan, race_check
            result = {"race-demo": race_demo, "race-plan": race_plan, "race-check": race_check}[args.command](args)
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
