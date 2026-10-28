"""Offline-only two-stream departure replay. No camera devices, network or motor imports.

Use two synchronized video files or two sorted image directories. Output times are
video/sample timestamps, NOT latency measured on the car.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import cv2

from carvision.departure import (CAMERAS, DEFAULT_CLEAR_CONFIRM_S,
                                 DEFAULT_MIN_BLUE_PERCENT, DepartureSequence,
                                 detect_board, validate_blue_percent)

IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


class FrameSource:
    """Read a file or image directory, never a live camera or device index."""

    def __init__(self, location):
        self.path = Path(location)
        self._cap = None
        self._paths = None
        self._index = 0
        if self.path.is_dir():
            self._paths = sorted(p for p in self.path.iterdir()
                                 if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS)
            if not self._paths:
                raise ValueError(f'图片目录为空: {self.path}')
        elif self.path.is_file():
            if self.path.suffix.lower() in IMAGE_EXTENSIONS:
                self._paths = [self.path]
            else:
                self._cap = cv2.VideoCapture(str(self.path))
                if not self._cap.isOpened():
                    raise ValueError(f'不能打开视频文件: {self.path}')
        else:
            raise ValueError(f'文件不存在: {self.path}')

    @property
    def fps(self):
        return float(self._cap.get(cv2.CAP_PROP_FPS)) if self._cap else None

    def read(self):
        if self._paths is not None:
            if self._index >= len(self._paths):
                return None
            # imdecode supports non-ASCII paths better than imread on Windows.
            import numpy as np
            path = self._paths[self._index]
            image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
            self._index += 1
            if image is None:
                raise ValueError(f'不能读取图片: {path}')
            return image
        success, image = self._cap.read()
        return image if success else None

    def close(self):
        if self._cap is not None:
            self._cap.release()


def _demo_frames(root, fps):
    """Fixture-based demonstration, not two independent physical cameras."""
    import numpy as np
    folder = root / 'vision' / 'tests' / 'fixtures'

    def photo(name):
        image = cv2.imdecode(np.fromfile(str(folder / name), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'缺少演示图片: {name}')
        return image

    present = photo('blue-board-close.jpg')
    absent = photo('crosswalk-background-primary.jpg')
    parking_blue = photo('parking-border-clipped-left-board-20261013.jpg')
    # 0.8s present, 2.0s absent, 0.6s parking-like blue after departure.
    images = ([present] * math.ceil(fps * .8) + [absent] * math.ceil(fps * 2.)
              + [parking_blue] * math.ceil(fps * .6))
    for img in images:
        yield {'primary': img, 'secondary': img}


def run_replay(pairs, fps, blue_percent=DEFAULT_MIN_BLUE_PERCENT, clear_s=DEFAULT_CLEAR_CONFIRM_S,
               max_frames=10000, *, on_frame=None):
    """Accepts iterable of {'primary':BGR_image,'secondary':BGR_image}."""
    if not math.isfinite(fps) or fps <= 0 or fps > 240:
        raise ValueError('FPS必须在0–240之间')
    if type(max_frames) is not int or max_frames < 1:
        raise ValueError('max_frames必须是正整数')
    validate_blue_percent(blue_percent)
    seq = DepartureSequence(clear_s)
    board_confirm_at = None
    first_clear_at = None
    trigger_at = None
    triggers = 0
    states_count = {cam: {'present': 0, 'absent': 0, 'unknown': 0} for cam in CAMERAS}
    total_frames = 0
    for i, pair in enumerate(pairs):
        if i >= max_frames:
            break
        t = round(i / fps, 8)
        observations = {}
        for cam in CAMERAS:
            item = detect_board(pair[cam], blue_percent)
            states_count[cam][item['state']] += 1
            observations[cam] = {**item, 'frame_id': i + 1, 'captured_s': t, 'camera_id': cam}
        old_phase = seq.phase
        fired = seq.update(observations, t)
        if seq.phase == 'wait_removed' and old_phase != 'wait_removed':
            board_confirm_at = t
        if seq.phase == 'wait_removed' and all(observations[c]['state'] == 'absent' for c in CAMERAS):
            if old_phase != 'wait_removed' or not first_clear_at or seq.clear_elapsed == 0:
                first_clear_at = t
        if fired:
            triggers += 1
            trigger_at = t
        if on_frame:
            on_frame({'index': i + 1, 'time_s': t, 'phase': seq.phase, 'event': 'departure' if fired else None,
                      'clear_elapsed_s': seq.clear_elapsed,
                      'observations': {c: {'state': observations[c]['state'],
                                           'largest_blue_fraction': observations[c]['largest_blue_fraction'],
                                           'boxes': observations[c]['boxes']} for c in CAMERAS}})
        total_frames = i + 1
    return {'mode': 'offline_simulated_timestamps', 'frames': total_frames, 'fps': fps,
            'min_blue_percent': blue_percent, 'clear_confirm_s': clear_s,
            'board_confirmed_at_s': board_confirm_at,
            'first_clear_at_s': first_clear_at, 'departure_trigger_at_s': trigger_at,
            'observed_clear_to_trigger_s': round(trigger_at - first_clear_at, 5)
            if trigger_at is not None and first_clear_at is not None else None,
            'departure_event_count': triggers, 'final_phase': seq.phase,
            'camera_state_counts': states_count,
            'note': '模拟视频帧时间；不代表实车推理、网络、播报、电机的实际延迟'}


def run_sources(primary_path, secondary_path, fps_override, blue_percent, clear_s,
                max_frames, on_frame):
    a = FrameSource(primary_path)
    try:
        b = FrameSource(secondary_path)
        try:
            afps, bfps = a.fps, b.fps
            if not fps_override:
                if afps is None or bfps is None or afps <= 0 or bfps <= 0:
                    raise ValueError('图片目录或无FPS的视频须显式指定 --fps')
                if abs(afps - bfps) > .01:
                    raise ValueError('两路视频FPS不一致；请先同步并转换，再回放')
            fps = fps_override or afps
            def pairs():
                while True:
                    ai = a.read()
                    bi = b.read()
                    if ai is None or bi is None:
                        break
                    yield {'primary': ai, 'secondary': bi}
            return run_replay(pairs(), fps, blue_percent, clear_s, max_frames, on_frame=on_frame)
        finally:
            b.close()
    finally:
        a.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description='蓝色挡板离线回放；只分析文件，不操作车或摄像头')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--demo', action='store_true', help='使用仓库历史图片模拟双路，非双摄实测')
    source.add_argument('--primary', type=Path, help='第一路视频文件/连续图片目录')
    parser.add_argument('--secondary', type=Path, help='第二路视频文件/连续图片目录')
    parser.add_argument('--fps', type=float, default=None, help='帧率；图片序列必须指定，demo默认10')
    parser.add_argument('--blue-percent', type=float, default=DEFAULT_MIN_BLUE_PERCENT)
    parser.add_argument('--clear-s', type=float, default=DEFAULT_CLEAR_CONFIRM_S)
    parser.add_argument('--max-frames', type=int, default=10000)
    parser.add_argument('--report', type=Path, help='保存JSON结果')
    parser.add_argument('--frames-jsonl', type=Path, help='可选：保存每帧检测日志')
    args = parser.parse_args(argv)
    if bool(args.primary) != bool(args.secondary):
        parser.error('使用真实视频必须同时提供 --primary 和 --secondary，不得将单路伪称双摄')
    log_file = None
    try:
        if args.frames_jsonl:
            args.frames_jsonl.parent.mkdir(parents=True, exist_ok=True)
            log_file = args.frames_jsonl.open('w', encoding='utf-8')
        def record(frame):
            if log_file:
                log_file.write(json.dumps(frame, ensure_ascii=False, allow_nan=False) + '\n')
        if args.demo:
            root = Path(__file__).resolve().parents[2]
            fps = args.fps if args.fps is not None else 10.
            result = run_replay(_demo_frames(root, fps), fps, args.blue_percent,
                                args.clear_s, args.max_frames, on_frame=record)
            result['input'] = 'historical fixtures duplicated on two simulated streams'
        else:
            result = run_sources(args.primary, args.secondary, args.fps,
                                 args.blue_percent, args.clear_s, args.max_frames, record)
            result['input'] = 'two provided files/directories (time-aligned assumed)'
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (ValueError, cv2.error, OSError, KeyError) as exc:
        print(f'回放失败: {exc}', file=sys.stderr)
        return 2
    finally:
        if log_file is not None:
            log_file.close()


if __name__ == '__main__':
    raise SystemExit(main())
