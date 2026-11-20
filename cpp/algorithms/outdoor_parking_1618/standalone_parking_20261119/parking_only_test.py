#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停车单项测试（2026-10-30），只新增此文件，不改全流程程序。

放在 race_master_20261011 目录，复用同目录的识别、策略、参数和驱动。
本测试自动确认空侧，沿外白线引导；入口线到位或锁定后在近底部丢失，右库继续巡线0.6秒、左库0.6秒再刹车。
泊车转向只取白线位置与方向，不使用中间黄色纵线。
入口触发位置由 --entry-stop-y 设置，默认0.90，即288像素画面约259像素。
这是入口识别验证，不代表四轮已入库；实际滑行距离需现场校准。起步仍须在停车入口前。
从 PARK_SEARCH 开始，不等起跑挡板、不跑其他比赛模块，泊车结束即退出，
不调用扫码或语音。蓝色停车入口板仍是原停车识别的一部分。

树莓派上由使用者执行（此文件不会自动切换服务）：
  /home/pi/src26/car-mode.sh auto
  cd /home/pi/src26/race_master_20261011
  python3 -u parking_only_test.py --check
  python3 -u parking_only_test.py --drive --seconds 60

只观察、不驱动：python3 -u parking_only_test.py --observe --seconds 30
实车模式按 Enter 开始、Ctrl+C 停止；初次先架空确认驱动和停止。
启动位置：停车区入口前的白线车道，车头朝停车区。保留三帧可靠车道、
丢线、图像过期、指令看门狗与停车通过超时保护。不能无条件盲目前进。
PWM 回读是 pigpio 的配置值，不是示波器测量或车轮转动反馈。
发现库口时主动制动；入库后按已选库位计时，再反向刹车；白线与画面新鲜度保护保留。
其他停车请求仍转换为中位；实际制动距离取决于电调和车速。
本测试统一使用300毫秒图像上限，仅覆盖本测试进程的运行配置。
"""

import argparse
from collections import deque
from copy import deepcopy
from datetime import datetime
from dataclasses import replace
import importlib.util
import inspect
import json
import math
import os
import queue
from pathlib import Path
import signal
import sys
import threading
import textwrap
import time

# Standalone CLI uses the preserved indoor camera and parking modules.
# Keep ROOT in this directory so existing commands and run logs stay in place.
if __name__ == '__main__':
    indoor_root = Path(__file__).resolve().parents[2] / '室内版'
    if not (indoor_root / 'parking_modules' / 'parking_entry_geometry.py').is_file():
        raise RuntimeError('Indoor parking baseline is missing: ' + str(indoor_root))
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(indoor_root))

import cv2
import numpy as np
import config as C
from camera_stream import LatestFrameCamera
from cone_solo import restore_gimbal
from fusion_run import FusionIO, runtime_config, draw_frame
from race_master_run import build_controller, load_config, check_conflicts, close_motion
from parking_corridor_tracker import ParkingCorridorTracker
from standalone_parking_20261119.parking_side_exit_perception import SideExitPerception

ROOT = Path(__file__).resolve().parents[1]
HINTS = {
    'WAIT_RELIABLE_LANE': '等待连续三帧可靠白线，此时不发送前进动力',
    'LANE_REACQUIRE_TIMEOUT': '三秒内未获得可靠车道，已停止；检查车位、白线和画面',
    'LANE_LOST': '车道丢失，已停止',
    'PARK_ENTRY_SETTLE_BRAKE': '已识别停车入口，行进中以10600制动0.25秒后回中位',
    'PARK_STATIONARY_BASELINE': '停车入口稳定等待',
    'STALE_FRAME': '相机画面缺失或过期，已停止',
    'PROCESSING_TOO_SLOW': '本帧处理超时，已停止',
    'WATCHDOG': '驱动超过0.4秒没有收到新命令，已锁定停止',
    'HARD_FRAME_AGE_LIMIT': '相机最新采集帧超时，已锁定停止',
    'CONTROL_LOOP_TIMEOUT': '驱动使能后控制循环未及时处理新帧，已锁定停止',
    'HARD_PARK_PASS_LIMIT': '停车通过动作超过保护时限，已停止',
    'PARKED_QR_READY': '原停车策略已结束；本测试不扫码，是否入库需现场核对',
    'PARK_TEST_TIME_LIMIT': '本次测试达到时限，已停止',
    'INTERRUPTED': '使用者中断，已结束测试',
    'ENTRY_YELLOW_DELAY_DRIVE': '入口黄线已触发停车计时，沿外侧白线继续行驶',
    'ENTRY_YELLOW_DELAY_COMPLETE': '所选库位的延时已结束，反向刹车后回中位',
    'ENTRY_YELLOW_LINE_LOST': '入口线未锁定或检测数据无效，立即刹车后回中位',
    'WAIT_OUTER_WHITE_LINE': '等待连续三帧外侧白线确认',
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--drive', action='store_true', help='实车：按回车后开始')
    mode.add_argument('--observe', action='store_true', help='观察：不打开电机/云台GPIO')
    mode.add_argument('--check', action='store_true', help='检查依赖，不打开硬件')
    parser.add_argument('--seconds', type=float, default=60, help='运动/观察时限，0至180秒（不含0）')
    parser.add_argument('--parking-side', choices=('auto',), default='auto', help='沿用原全流程自动确认空车位左右侧')
    parser.add_argument('--white-offset', type=float, default=.22, help='与外侧白线的图像横向偏移，占画面宽度比例')
    parser.add_argument('--entry-stop-y', type=float, default=.90, help='入口线停车位置：画面高度比例，默认0.90（288像素画面约259像素）')
    parser.add_argument('--config', type=Path, default=ROOT / 'race_master_config.json')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 180:
        parser.error('--seconds 必须大于0且不超过180')
    if not math.isfinite(args.entry_stop_y) or not .55 <= args.entry_stop_y <= .98:
        parser.error('--entry-stop-y 必须在0.55至0.98之间')
    if not math.isfinite(args.white_offset) or not .10 <= args.white_offset <= .40:
        parser.error('--white-offset 必须在0.10至0.40之间')
    return args


class OuterWhiteTracker:
    """Single outer-white tracking; offset follows lane_follow.py's single-side idea."""

    def __init__(self, side, offset, *, left_pink=False):
        self.side, self.offset = side, offset
        self.left_pink = left_pink
        self.white = self.reference_slope = None
        self.models = None  # Virtual search corridor for transverse lines ONLY.
        self.last_id = self.last_ts = None
        self.hits = 0
        self.cached = {}

    def update(self, frame, fid, ts):
        if fid == self.last_id:
            return dict(self.cached)
        result = dict(valid=False, side=self.side, captured_at=ts,
                      center_x=None, heading_error=None, reason='OUTER_WHITE_MISSING')
        if not math.isfinite(ts) or (self.last_ts is not None and ts <= self.last_ts):
            result['reason'] = 'WHITE_FRAME_NOT_FRESH'
            return result
        self.last_id, self.last_ts = fid, ts
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 65], np.uint8),
                           np.array([179, 65, 255], np.uint8))
        if self.side == 'RIGHT' or (self.side == 'LEFT' and self.left_pink):
            # Recover a narrow pink cast on white paint over the red track.
            # Both flanks must be darker red pavement, including at a shadow edge.
            lower = hsv[h // 2:]
            red = (lower[:, :, 0] < 12) | (lower[:, :, 0] >= 170)
            sat = lower[:, :, 1].astype(np.int16)
            val = lower[:, :, 2].astype(np.int16)
            flank = max(2, round(.025 * w))
            mid = slice(flank, -flank)
            left, right = slice(None, -2 * flank), slice(2 * flank, None)
            tinted = (red[:, mid] & red[:, left] & red[:, right]
                      & (sat[:, mid] > 65) & (sat[:, mid] <= 120)
                      & (val[:, mid] >= 65)
                      & (sat[:, mid] + 22 <= np.minimum(sat[:, left], sat[:, right]))
                      & (val[:, mid] >= np.maximum(val[:, left], val[:, right]) + 12))
            mask[h // 2:, mid] |= tinted.astype(np.uint8) * 255
        # Pale yellow can also satisfy the old low-saturation white threshold.
        # Exclude that hue/saturation range so the divider cannot steer us.
        mask[(hsv[:, :, 0] >= 12) & (hsv[:, :, 0] <= 43)
             & (hsv[:, :, 1] >= 28)] = 0
        mask[:int(.50*h)] = 0
        seeds = [self.white] if self.white is not None else []
        if not seeds:
            lines = cv2.HoughLinesP(mask, 1, np.pi/180, 20,
                                   minLineLength=int(.12*h), maxLineGap=8)
            for line in ([] if lines is None else lines[:, 0]):
                x1,y1,x2,y2 = map(float, line)
                if abs(y2-y1) < .08*h:
                    continue
                a=(x2-x1)/(y2-y1);b=x1-a*y1;x=a*.80*h+b
                if ((self.side=='RIGHT' and .55*w < x < 1.03*w and a > .15)
                        or (self.side=='LEFT' and -.20*w < x < .45*w and a < -.15)):
                    seeds.append([a,b])
        # Bound acquisition cost on the Pi; merge duplicate Hough segments.
        expected=(.47+(1 if self.side=='RIGHT' else -1)*self.offset)*w
        selected=[]
        for seed in sorted(seeds, key=lambda m:abs(m[0]*.80*h+m[1]-expected)):
            if all(abs((seed[0]-old[0])*.80*h+seed[1]-old[1]) > .025*w for old in selected):
                selected.append(seed)
            if len(selected) >= 3:
                break
        fits=[]
        for seed in selected:
            fit=ParkingCorridorTracker._fit(mask,seed,int(.50*h),int(.94*h))
            if fit is not None and fit['y1']-fit['y0'] >= .12*h:
                fits.append(fit)
        if fits:
            sign=1 if self.side=='RIGHT' else -1
            expected=(.47+sign*self.offset)*w
            fit=min(fits,key=lambda f:abs(f['model'][0]*.80*h+f['model'][1]-expected))
            a,b=fit['model'];x=a*.80*h+b
            previous=None if self.white is None else self.white[0]*.80*h+self.white[1]
            if previous is not None and abs(x-previous) > .12*w:
                result['reason']='OUTER_WHITE_JUMP'
            elif not (-.20*w if self.side == 'LEFT' else -.03*w) <= x <= 1.03*w:
                result['reason']='OUTER_WHITE_OUTSIDE_IMAGE'
            else:
                if self.reference_slope is None:
                    self.reference_slope=a
                heading=(a-self.reference_slope)*(.20*h)/w
                center=x/w-sign*self.offset
                if not 0 <= center <= 1 or abs(heading) > .25:
                    result['reason']='OUTER_WHITE_HEADING_INVALID'
                else:
                    self.white=[a,b];self.hits+=1
                    # Perspective search band for the end line, not a detected
                    # yellow boundary and never used to compute steering.
                    inner_a=a-sign*2*self.offset*w/(.35*h)
                    inner=[inner_a,b+(a-inner_a)*.45*h]
                    self.models=[inner,self.white]
                    result.update(valid=self.hits>=3,center_x=center,heading_error=heading,
                                  reason='OUTER_WHITE_TRACKED' if self.hits>=3 else 'OUTER_WHITE_CONFIRMING',
                                  white=fit, reference_y=.80*h, white_x=x,
                                  offset_px=self.offset*w, stable_frames=self.hits)
        if not result['valid'] and result['reason']!='OUTER_WHITE_CONFIRMING':
            self.hits=0
        self.cached=result
        return dict(result)


def outdoor_entry_mask(hsv):
    """Add warm tape only where it is a narrow bright stripe across red pavement."""
    yellow = cv2.inRange(hsv, (12, 28, 45), (43, 255, 255))
    h = hsv.shape[0]
    # The geometry search begins at 50% height, final stripe tracking at 40%.
    start = int(.40 * h)
    lower = hsv[start:]
    hue, sat, value = cv2.split(lower)
    radius = max(4, round(.027 * h))
    # Require red pavement on at least one flank and a brightness peak
    # relative to both. Uniform pavement and one-sided shadow edges fail.
    red = ((hue < 12) | (hue >= 170)) & (sat >= 85)
    value_i = value.astype(np.int16)
    sat_i = sat.astype(np.int16)
    mid = slice(radius, -radius)
    above, below = slice(None, -2*radius), slice(2*radius, None)
    if lower.shape[0] <= 2 * radius:
        return yellow
    near_red = red[above] | red[below]
    rise = value_i[mid] - np.maximum(value_i[above], value_i[below])
    bright = (rise >= 4) & (value_i[mid] >= np.minimum(value_i[above], value_i[below]) + 32)
    pale = sat_i[mid] + 18 <= np.maximum(sat_i[above], sat_i[below])
    warm = ((hue[mid] < 12) | (hue[mid] >= 175)) & (sat[mid] >= 32) & (sat[mid] <= 145)
    support = near_red & bright & pale & warm
    # Nearby orange pixels can bridge small colour shifts along the same tape.
    seeds = (support & (hue[mid] >= 4) & (hue[mid] < 12) & (rise >= 12)).astype(np.uint8)
    nearby = cv2.dilate(seeds, np.ones((3, 33), np.uint8))
    yellow[start + radius:h - radius] |= (support & (nearby != 0)).astype(np.uint8) * 255
    return yellow


class EntryLineTracker:
    """Track only the nearest entrance stripe; stop at the image bottom."""

    def __init__(self, stop_y=.90):
        self.stop_y = stop_y
        self.y = self.last_ts = self.last_id = None
        self.last_input_ts = None
        self.hits = 0
        self.locked = False
        self.previous = None
        self.cached = {}

    def start_from_confirmed_bay(self, observation):
        """Use the entrance measured during bay selection as the first reference."""
        ratio = observation.get('entrance_y')
        if not isinstance(ratio, (int, float)) or not 0 < ratio <= 1:
            return {}
        h = observation.get('frame_height', C.CAMERA_HEIGHT)
        self.y = ratio*h
        self.last_ts = self.last_input_ts = observation['captured_at']
        self.last_id = observation['frame_id']
        self.locked = True
        self.cached = dict(frame_id=self.last_id, captured_at=self.last_ts,
                           y=self.y, valid=True, locked=True, stop=ratio >= self.stop_y,
                           near_bottom=ratio >= self.stop_y-.05, threshold_y=self.stop_y*h,
                           last_seen_at=self.last_ts, reason='ENTRY_FROM_CONFIRMED_BAY')
        return dict(self.cached)

    @staticmethod
    def stripe_centers(mask, transverse):
        """Locate paint-band centers at a fixed x, rather than Hough midpoints."""
        h, w = mask.shape
        reference_x = .47*w
        bands = {}  # Reuse column scans across Hough fragments in this frame.
        normalized = []
        for candidate in transverse:
            x1,y1,x2,y2 = candidate['endpoints']
            if x2 <= x1:
                continue
            slope = (y2-y1)/(x2-x1)
            left, right = max(1., x1), min(float(w-2), x2)
            if right <= left:
                continue
            # Sample inside this visible segment, avoiding its noisy endpoints.
            columns = sorted(set(int(round(left+fraction*(right-left)))
                                 for fraction in (.1, .3, .5, .7, .9)))
            centers = []
            sample_columns = []
            for x in columns:
                if x not in bands:
                    active = np.count_nonzero(mask[:, x-1:x+2], axis=1) >= 2
                    edges = np.diff(np.pad(active.astype(np.int8), (1,1), mode='constant'))
                    bands[x] = [(float(a), float(b-1)) for a,b in zip(
                        np.flatnonzero(edges==1), np.flatnonzero(edges==-1))
                        if 2 <= b-a <= .12*h]
                predicted = y1+slope*(x-x1)
                matching = [(a,b) for a,b in bands[x] if a-3 <= predicted <= b+3]
                if matching:
                    a,b = min(matching,key=lambda ab:abs((ab[0]+ab[1])/2-predicted))
                    centers.append((a+b)/2+slope*(reference_x-x))
                    sample_columns.append(x)
            if len(centers) >= 2:
                normalized.append(dict(candidate, y=float(np.median(centers)),
                                       reference_x=reference_x, position_method='YELLOW_BAND_CENTER',
                                       sample_columns=sample_columns, sample_count=len(centers)))
        # Same paint band produces the same central position even when Hough
        # returns its upper edge, lower edge, or differently sized fragments.
        stripes = []
        for candidate in sorted(normalized,key=lambda c:c['overlap_px'],reverse=True):
            if all(abs(candidate['y']-other['y']) > .025*h for other in stripes):
                stripes.append(candidate)
        return stripes

    def update(self, frame, fid, ts, obs, tracker):
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = outdoor_entry_mask(hsv)
        mask[:int(.40*h)] = 0
        lines = cv2.HoughLinesP(mask, 1, np.pi/180, 20,
                                minLineLength=int(.08*w), maxLineGap=8)
        transverse = []
        models = tracker.models
        for line in ([] if lines is None else lines.reshape(-1, 4)):
            x1, y1, x2, y2 = map(float, line)
            if x2 < x1:
                x1, y1, x2, y2 = x2, y2, x1, y1
            if x2-x1 < .08*w or abs(y2-y1) > .25*(x2-x1) or models is None:
                continue
            y = (y1+y2)/2
            edges = sorted(a*y+b for a, b in models)
            # The entry stripe may be cropped near the bumper. Check its
            # visible intersection, not 65% of an extrapolated offscreen bay.
            edges = [max(0., edges[0]), min(float(w-1), edges[1])]
            width = edges[1]-edges[0]
            overlap = max(0., min(x2, edges[1])-max(x1, edges[0]))
            if width < .06*w or overlap < max(.08*w, .30*width):
                continue
            transverse.append(dict(y=y, endpoints=[x1, y1, x2, y2], overlap_px=overlap))
        # Merge Hough fragments of the same painted stripe before assigning
        # entry identity. No far-line identity is acquired.
        stripes = self.stripe_centers(mask, transverse)
        return self.update_candidates(stripes, fid, ts, h)

    def update_candidates(self, stripes, fid, ts, h):
        if not math.isfinite(ts):
            return dict(valid=False, locked=self.locked, stop=False, reason='ENTRY_LINE_TIME_INVALID')
        if fid == self.last_id and ts == self.last_input_ts:
            return dict(self.cached)
        if fid == self.last_id or (self.last_input_ts is not None and ts <= self.last_input_ts):
            return dict(valid=False, locked=self.locked, stop=False, reason='ENTRY_LINE_TIME_INVALID')
        self.last_id, self.last_input_ts = fid, ts
        result = dict(valid=False, locked=self.locked, stop=False,
                      reason='ENTRY_LINE_NOT_FOUND', captured_at=ts, frame_id=fid,
                      threshold_y=self.stop_y*h, transverse_candidates=stripes)
        matches = stripes
        if self.y is not None:
            # Scale the allowed downward motion with elapsed capture time,
            # bounded to 18% of image height. Never relabel a distant stripe.
            dt = ts-self.last_ts
            limit = min(.18*h, max(.10*h, 1.2*h*dt))
            matches = []
            for c in stripes:
                if not -.035*h <= c['y']-self.y <= limit:
                    continue
                if self.previous is None:
                    # Bay selection supplies a height. The first matching stripe
                    # in the selected corridor supplies the measured endpoints.
                    matches.append(c)
                    continue
                x1,y1,x2,y2 = c['endpoints']
                px1,py1,px2,py2 = self.previous['endpoints']
                if (x2 <= x1 or px2 <= px1
                        or abs((y2-y1)/(x2-x1)-(py2-py1)/(px2-px1)) > .15
                        or min(x2,px2)-max(x1,px1) < .08*h):
                    continue
                matches.append(c)
        if matches:
            if self.y is not None:
                # Prefer well-supported candidates after continuity checks.
                strongest = max(c['overlap_px'] for c in matches)
                matches = [c for c in matches if c['overlap_px'] >= .5*strongest]
            c = (max(matches,key=lambda c:c['y']) if self.y is None
                 else min(matches,key=lambda c:abs(c['y']-self.y)))
            self.y, self.last_ts, self.previous = c['y'], ts, c
            self.hits += 1
            self.locked = self.locked or self.hits >= 3
            result.update(valid=self.hits >= 3, locked=self.locked, y=self.y,
                          endpoints=c['endpoints'], stop=self.hits >= 3 and self.y >= self.stop_y*h,
                          reason='ENTRY_LINE_TRACKED' if self.locked else 'ENTRY_LINE_CONFIRMING')
        else:
            self.hits = 0
            if not self.locked:
                self.y = self.previous = None
        result['near_bottom'] = self.locked and self.y is not None and self.y >= (self.stop_y-.05)*h
        result['last_seen_at'] = self.last_ts
        self.cached = result
        return dict(result)


BAY_START_FRAMES = 2


def install_entry_line_stop(ctl, stop_y, left_delay):
    detector = EntryLineTracker(stop_y)
    original_extra = ctl.update_extra
    original_policy_update = ctl.policy.parking.update
    parking = ctl.policy.parking
    parking.entry_delay_started_at = None
    parking.entry_delay_deadline = None
    right_white_offset = ctl.white_tracker.offset

    def update_extra(frame, fid, ts, now):
        nonlocal detector
        original_extra(frame, fid, ts, now)
        if ctl.policy.stage == 'PARKING':
            # Do not gate this detector on the old board/entrance initializer.
            ctl.parking_observation = dict(ctl.parking_observation or {},
                                          frame_id=fid, captured_at=ts)
            # Acquire the currently observed outer white line while the original
            # policy confirms the empty side. This is perception only: the
            # original policy alone decides whether/when parking may start.
            candidate_side = ctl.parking_observation.get('open_side')
            if (parking.state == 'WAIT' and candidate_side in ('LEFT', 'RIGHT')
                    and candidate_side != ctl.white_tracker.side):
                ctl.white_tracker = OuterWhiteTracker(
                    candidate_side, right_white_offset, left_pink=ctl.white_tracker.left_pink)
                detector = EntryLineTracker(stop_y)
            ctl.parking_observation['lane_guide'] = (
                measure_step(ctl, 'outer_white_ms', ctl.white_tracker.update, frame, fid, ts) if ctl.white_tracker.side is not None
                else dict(valid=False, side=None, captured_at=ts, reason='WAIT_AUTO_PARKING_SIDE'))
            if parking.state == 'WAIT':
                ctl.parking_observation['frame_height'] = frame.shape[0]
            elif parking.state == 'ALIGN':
                ctl.parking_observation['entry_line'] = measure_step(ctl, 'entry_yellow_ms', detector.update,
                    frame, fid, ts, ctl.parking_observation, ctl.white_tracker)
            ctl.policy.parking_observation = ctl.parking_observation

    def update_policy(now, observation):
        observation = observation if isinstance(observation, dict) else {}
        entry = observation.get('entry_line') or {}
        if parking.state == 'WAIT':
            # Only WAIT counts the combined entrance, board and white-line samples.
            previous_confirm_frames = parking.cfg['confirm_frames']
            try:
                parking.cfg['confirm_frames'] = BAY_START_FRAMES
                request = original_policy_update(now, observation)
            finally:
                parking.cfg['confirm_frames'] = previous_confirm_frames
            if parking.state != 'ALIGN':
                return request
            ctl.white_tracker.side = parking.selected_side
            ctl.white_tracker.offset = right_white_offset
            entry = detector.start_from_confirmed_bay(observation)
            observation['entry_line'] = entry
        if parking.state == 'ALIGN':
            problem = parking._pass_observation_problem(now, observation)
            if problem:
                return parking._start_brake(now, problem)
            if parking.last_now is not None and now < parking.last_now:
                return parking._start_brake(now, 'CLOCK_MOVED_BACKWARD')
            if parking.entry_delay_deadline is None:
                fresh = (entry.get('frame_id') == observation.get('frame_id')
                         and entry.get('captured_at') == observation.get('captured_at'))
                if not (fresh and entry.get('locked')):
                    return parking._start_brake(now, 'ENTRY_YELLOW_LINE_LOST')
                if entry.get('stop') or (not entry.get('valid') and entry.get('near_bottom')):
                    parking.entry_delay_started_at = now
                    parking.entry_delay_deadline = now + (.6 if parking.selected_side == 'RIGHT' else left_delay)
            # Once the entrance reaches the bottom it may leave the image.
            # Continue using fresh white guidance without restarting the timer.
            if parking.entry_delay_deadline is not None and now >= parking.entry_delay_deadline:
                return parking._start_brake(now, 'ENTRY_YELLOW_DELAY_COMPLETE', fault=False)
            dt = 0. if parking.last_now is None else now-parking.last_now
            if dt < 0:
                return parking._start_brake(now, 'CLOCK_MOVED_BACKWARD')
            parking.last_now = now
            parking._record_pass_frame(now, observation)
            guide = observation.get('lane_guide') or {}
            if parking.last_lane_at is not None and not guide.get('valid'):
                # Keep the last steering command while the outer white line is reacquired.
                parking.reason = 'OUTER_WHITE_HOLD_STEERING'
                request = parking._result(now)
            else:
                request = parking._lane_steering(now, dt, observation)
            if parking.entry_delay_deadline is not None and request['state'] == 'ALIGN':
                parking.reason = 'ENTRY_YELLOW_DELAY_DRIVE'
                request = dict(request, reason=parking.reason)
            return request
        return original_policy_update(now, observation)

    def update_with_left_speed(now, observation):
        # Left-only earlier countersteering; restore shared instance settings
        # even on exceptions so right-side control retains its original gain.
        original_heading_gain = parking.cfg['lane_heading_kp_us']
        original_steer_max = parking.cfg['steer_max_us']
        if parking.selected_side == 'LEFT':
            parking.cfg['lane_heading_kp_us'] = 900.0
            parking.cfg['steer_max_us'] = min(original_steer_max, 1810)
        try:
            request = update_policy(now, observation)
        finally:
            parking.cfg['lane_heading_kp_us'] = original_heading_gain
            parking.cfg['steer_max_us'] = original_steer_max
        return limit_left_parking_speed(request, parking.selected_side, parking.cfg['low_duty'])

    ctl.update_extra = update_extra
    parking.update = update_with_left_speed


def limit_left_parking_speed(request, side, limit=12600):
    """Cap forward parking requests on the selected left side only."""
    if side == 'LEFT' and request['duty'] > limit:
        request = dict(request, duty=limit)
    return request


def install_test_entry_margin():
    """Apply the standalone test's three small entry-geometry relaxations."""
    import parking_entry_geometry as geometry
    import parking_entry_preview as preview
    source = inspect.getsource(geometry.detect_entry_geometry)
    original = 'ex[0]+w*.04<cross[0]<ex[1]+w*.02'
    replacement = 'ex[0]+w*.03<cross[0]<ex[1]+w*.02'
    if source.count(original) != 1:
        raise RuntimeError('停车入口识别源码已变化，无法安全应用测试专用3%余量')
    namespace = dict(geometry.__dict__)
    depth_check = 'y_near-y_far<h*.025'
    if source.count(depth_check) != 1:
        raise RuntimeError('停车通道识别源码已变化，无法安全应用测试专用深度门槛')
    source = source.replace(original, replacement).replace(depth_check, 'y_near-y_far<h*.020')
    original_mask = 'yellow=cv2.inRange(hsv,np.array([12,28,45],np.uint8),np.array([43,255,255],np.uint8))'
    if original_mask not in source:
        # Some import-based callers load the outdoor geometry module directly.
        original_mask = 'yellow=cv2.inRange(hsv,np.array([12,28,125],np.uint8),np.array([55,255,255],np.uint8))'
    if source.count(original_mask) != 1:
        raise RuntimeError('入口颜色掩膜源码已变化，无法应用室外停车测试配置')
    namespace['outdoor_entry_mask'] = outdoor_entry_mask
    source = source.replace(original_mask, 'yellow=outdoor_entry_mask(hsv)')
    exec(compile(source,
                 '<parking_only_entry_margin_03>', 'exec'), namespace)
    preview.detect_entry_geometry = namespace['detect_entry_geometry']
    import parking_bypass_perception as perception
    from standalone_parking_20261119.parking_side_exit_perception import SideExitPerception
    source = textwrap.dedent(inspect.getsource(perception.BypassPerception._vanishing_point))
    length_check = "divider.get('observed_length_px', 0) < .045*height"
    if source.count(length_check) != 1:
        raise RuntimeError('停车方向识别源码已变化，无法安全应用测试专用黄线长度门槛')
    source = source.replace('@staticmethod\n', '').replace(
        length_check, "divider.get('observed_length_px', 0) < .040*height")
    namespace = dict(perception.__dict__)
    exec(compile(source, '<parking_only_divider_length_04>', 'exec'), namespace)
    SideExitPerception._vanishing_point = staticmethod(namespace['_vanishing_point'])


def measure_step(ctl, name, function, *args, **kwargs):
    started = time.monotonic()
    try:
        return function(*args, **kwargs)
    finally:
        ctl.step_timing[name] = ctl.step_timing.get(name, 0.) + (time.monotonic()-started)*1000


def prepare_step_timing(ctl):
    ctl.step_timing = {}
    # Perception is recreated when entering PARKING, so inspect each iteration.
    for obj, method, name in ((ctl.parking_perception, 'update', 'parking_perception_ms'),
                              (ctl.lane, 'update', 'ordinary_lane_ms')):
        original = getattr(obj, method)
        if getattr(original, '_parking_timed', False):
            continue
        def wrapped(*args, _original=original, _name=name, **kwargs):
            return measure_step(ctl, _name, _original, *args, **kwargs)
        wrapped._parking_timed = True
        setattr(obj, method, wrapped)


class BoardSidePerception(SideExitPerception):
    """Standalone trial: locate the upright board relative to the car's front axis."""

    selecting_bay = False

    def approach_board(self, frame, min_stripe_width=.055):
        """Find a distant upright blue board with transverse entrance tape at its base."""
        h, w = frame.shape[:2]
        top, bottom = int(.32*h), int(.78*h)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        roi = hsv[top:bottom]
        low = self.preview_detector.blue_low.copy()
        low[1] = max(95, low[1])
        mask = cv2.inRange(roi, low, self.preview_detector.blue_high)
        candidates = []
        for x, y, bw, bh, area in cv2.connectedComponentsWithStats(mask, 8)[2][1:]:
            foot = top + y + bh
            if not (.075*w <= bw <= .26*w and .085*h <= bh <= .31*h
                    and .52*h <= foot <= .76*h and .55 <= bw/bh <= 1.6
                    and area >= .68*bw*bh and y > 0):
                continue
            if not self._is_qr_placard(roi[y:y+bh, x:x+bw]):
                candidates.append([int(x), int(top+y), int(bw), int(bh)])
        if len(candidates) != 1:
            return None
        x, y, bw, bh = candidates[0]
        foot = y + bh
        start, end = int(foot-.04*h), int(foot+.05*h)
        yellow = outdoor_entry_mask(hsv)[start:end]
        lines = cv2.HoughLinesP(yellow, 1, np.pi/180, 8, minLineLength=int(min_stripe_width*w), maxLineGap=5)
        if lines is None:
            return None
        for x1, y1, x2, y2 in lines.reshape(-1, 4):
            if x2-x1 < min_stripe_width*w or abs(y2-y1) > .20*(x2-x1):
                continue
            # Require tape beside the board: bright marks entirely on its face do not count.
            if x1 <= x+bw+.06*w and x2 >= x-.06*w and max(x-x1, x2-x-bw) >= .045*w:
                return dict(box=candidates[0], stripe_y=float(start+(y1+y2)/2),
                            stripe_x=float((x1+x2)/2))
        return None

    def _initial_update(self, frame, fid, ts):
        from parking_entry_preview import _for_side
        h, w = frame.shape[:2]
        geometry = _for_side(frame, 'LEFT')
        obs = self._empty(fid, ts)
        obs.update(geometry=geometry, board_conflict=False, geometry_valid=False,
                   tracking_valid=False, front_clear=False, side_exit_confirmed=False)
        entrance = geometry.get('entrance')
        if entrance is None and self.selecting_bay:
            # At rest, a shorter visible piece of tape can sit beside the board.
            # Use its measured height for the usual full-width board/conflict check.
            short = self.approach_board(frame, min_stripe_width=.09)
            if short is not None and short['stripe_y'] >= .58*h:
                entrance = {'center': [short['stripe_x'], short['stripe_y']]}
                obs['short_entry_center'] = entrance['center']
        boxes = []
        if entrance and 0 < entrance['center'][1] <= .96*h:
            top = int(.30*h)
            bottom = int(entrance['center'][1] - .08*h)
            if bottom > top:
                hsv = cv2.cvtColor(frame[top:bottom], cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, self.preview_detector.blue_low, self.preview_detector.blue_high)
                _, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
                for x, y, bw, bh, area in stats[1:]:
                    if area >= .003*w*h and bh >= .08*h and area >= .65*bw*bh:
                        if self._is_qr_placard(hsv[y:y+bh, x:x+bw]):
                            continue
                        boxes.append([int(x), int(y)+top, int(bw), int(bh)])
        split = self.front_x*w
        left = [box for box in boxes if box[0]+.70*box[2] < split-.01*w]
        right = [box for box in boxes if box[0]+.30*box[2] > split+.01*w]
        obs.update(board_boxes=boxes, side_reference_x=split)
        if not boxes or (left and right) or len(left)+len(right) != len(boxes):
            self._initial_hits = 0
            obs.update(board_conflict=bool(boxes), blocked='BOTH' if left and right else 'UNKNOWN')
            obs['reasons'] = ['Entrance or unambiguous upper-board side unavailable']
            return self._remember(obs, fid, ts)

        side = 'RIGHT' if left else 'LEFT'
        self._initial_hits = self._initial_hits+1 if self._initial_side == side else 1
        self._initial_side = side
        ready = self.selecting_bay or self._initial_hits >= self.confirm_frames
        ey = float(entrance['center'][1])
        if ready:
            self._track_box = max(boxes, key=lambda b: b[2]*b[3])[:]
            self._track_height = max(float(self._track_box[3]), ey-self._track_box[1])
            self._trusted_geometry = {'entrance': deepcopy(entrance)}
            self.last_entrance_y = ey/h
        obs.update(valid=ready, observation_ready=ready, tracking_valid=ready,
                   open_side=side, blocked='LEFT' if left else 'RIGHT', board_visible=True,
                   entrance_y=ey/h, board_side_hits=self._initial_hits)
        obs['reasons'] = ['Board side observed' if self.selecting_bay else
                          'Board side confirmed' if ready else 'Confirming board side']
        return self._remember(obs, fid, ts)

    def update(self, frame, fid, ts, selected_side=None):
        if selected_side is not None and self.selected_side is None:
            if (selected_side != self._initial_side or self._initial_hits < BAY_START_FRAMES
                    or self._track_box is None):
                obs = self._empty(fid, ts)
                obs.update(board_conflict=True, reasons=['Selected side lacks board confirmation'])
                return obs
            self.selected_side = selected_side
        if selected_side is not None and selected_side != self.selected_side:
            obs = self._empty(fid, ts)
            obs.update(board_conflict=True, reasons=['Selected side changed'])
            return obs
        # The base class normally accepts a new side via a divider-derived yaw
        # baseline. The selection above is already committed from the board.
        return super().update(frame, fid, ts)

def install_parking_guidance(ctl, white_offset=.22, entry_stop_y=.90, *, board_min_saturation=None,
                             left_pink=False, left_delay=.4):
    """Install the outdoor parking perception and steering on either controller."""
    install_test_entry_margin()
    def fresh_perception():
        perception = BoardSidePerception(front_x=ctl.policy.parking.cfg['front_x'])
        if board_min_saturation is not None:
            perception.preview_detector.blue_low[1] = board_min_saturation
        return perception
    ctl.parking_perception = fresh_perception()
    original_step = ctl.step
    def step(*args):
        command = original_step(*args)
        # The race controller creates a fresh observer on entering PARKING.
        if not isinstance(ctl.parking_perception, BoardSidePerception):
            ctl.parking_perception = fresh_perception()
        return command
    ctl.step = step
    ctl.step_timing = {}
    ctl.policy.parking.cfg['max_observation_age'] = .300
    ctl.white_tracker = OuterWhiteTracker(None, white_offset, left_pink=left_pink)
    install_entry_line_stop(ctl, entry_stop_y, left_delay)
    original_extra = ctl.update_extra
    previous_approach = None
    def update_extra(frame, fid, ts, now):
        nonlocal previous_approach
        ctl.parking_perception.selecting_bay = (
            ctl.policy.stage == 'PARKING' and ctl.policy.parking.state == 'WAIT')
        original_extra(frame, fid, ts, now)
        if ctl.policy.stage == 'PARK_SEARCH':
            obs = ctl.parking_observation
            if not obs.get('board_visible') and not obs.get('board_conflict'):
                approach = ctl.parking_perception.approach_board(frame)
                if approach is not None:
                    x, y, w, h = approach['box']
                    center = (x+w/2, y+h/2)
                    if previous_approach is not None:
                        prev_ts, px, py = previous_approach
                        if (0 < ts-prev_ts <= .30 and abs(center[0]-px) <= w/2
                                and abs(center[1]-py) <= h/2):
                            obs.update(board_visible=True, board_boxes=[approach['box']],
                                       approach_stripe_y=approach['stripe_y'],
                                       reasons=['Distant board and entrance tape confirmed'])
                    previous_approach = (ts, *center)
                else:
                    previous_approach = None
            else:
                previous_approach = None
        if ctl.policy.stage == 'PARKING' and ctl.policy.parking.state == 'WAIT':
            obs = ctl.parking_observation
            guide = obs.get('lane_guide') or {}
            if guide.get('reason') == 'OUTER_WHITE_CONFIRMING':
                # This fit already passed the pixel and geometry checks. During
                # bay selection the policy counts simultaneous fits across frames.
                guide = dict(guide, valid=True, reason='OUTER_WHITE_MEASURED')
                obs['lane_guide'] = guide
            if (guide.get('valid') and guide.get('side') == obs.get('open_side')
                    and guide.get('captured_at') == ts and obs.get('frame_id') == fid):
                obs['target_x'] = guide['center_x']
            else:
                obs['observation_ready'] = False
    ctl.update_extra = update_extra
    return ctl


def parking_controller(cfg):
    C.MAX_FRAME_AGE_SECONDS = .300
    ctl = build_controller(cfg)
    ctl.s.max_age = .300
    ctl.policy.stage = 'PARK_SEARCH'
    ctl.policy.cross_done = True
    ctl.policy.transition_reason = 'START_PARKING_ONLY'
    ctl.policy.clock = time.monotonic
    return install_parking_guidance(ctl, cfg.get('white_offset', .22), cfg.get('entry_stop_y', .90),
                                   board_min_saturation=75, left_pink=cfg.get('left_pink', False), left_delay=.6)


def check_other_programs():
    check_conflicts()
    # Cover other standalone entries and wrappers as well as the original list.
    for proc in Path('/proc').glob('[0-9]*'):
        if int(proc.name) == os.getpid():
            continue
        try:
            parts = (proc / 'cmdline').read_bytes().decode(errors='replace').split('\0')
        except OSError:
            continue
        if parts and 'python' in Path(parts[0]).name:
            scripts = [Path(p).name for p in parts[1:] if p.endswith('.py')]
            if any(s.startswith(('parking_', 'race_master_', 'cone_stage_', 'capture_qr_'))
                   for s in scripts):
                raise RuntimeError('其他车控/相机程序正在运行，先退出：PID ' + proc.name)


def drive_command(car, cmd, captured_at):
    """Use a bounded brake pulse at the bay entrance and at the end of the maneuver."""
    active_brake = cmd.state == 'BRAKE' and cmd.reason in (
        'PARK_ENTRY_SETTLE_BRAKE', 'ENTRY_YELLOW_DELAY_COMPLETE', 'ENTRY_YELLOW_LINE_LOST')
    if cmd.state in ('BRAKE', 'FAULT_BRAKE', 'LIGHT_BRAKE', 'PARK_BRAKE') and not active_brake:
        # Use the driver's normal neutral path, not its reverse-brake path.
        # Do not mutate the policy command used by mark_applied below.
        cmd = replace(cmd, state='PARK_NEUTRAL', duty=C.STOP_DUTY)
    if car is None:
        return cmd
    if not car.enabled and cmd.duty > C.STOP_DUTY:
        car.arm()
        print('[enable] 驱动已使能；前进PWM按原斜坡逐步增加', flush=True)
    if car.enabled or cmd.state in ('HALT', 'DONE'):
        car.write_command(cmd, captured_at)
    return cmd


def steering_snapshot(ctl, cmd, driver_cmd, car, captured_at):
    """Observe commands/output settings only; no physical steering feedback."""
    parking = ctl.policy.parking
    guide = (ctl.parking_observation or {}).get('lane_guide') or {}
    fresh_target = (guide.get('valid') is True
                    and parking.last_lane_at == captured_at
                    and cmd.reason in ('FRONT_CLEAR_LANE_GUIDANCE',
                                       'SIDE_EXIT_LANE_GUIDED_PASS'))
    result = dict(
        guide_valid=guide.get('valid', False),
        white_x=guide.get('white_x'),
        center_error=(guide['center_x']-parking.cfg['front_x']
                      if guide.get('center_x') is not None else None),
        heading_error=guide.get('heading_error'),
        lane_target_fresh=fresh_target,
        lane_target_us=parking.last_lane_target if fresh_target else None,
        policy_command_us=cmd.steer_us,
        driver_command_us=driver_cmd.steer_us,
        gpio_pulse_us=None,
        physical_angle_feedback=False,
        captured_at=captured_at)
    if car is not None:
        try:
            with car.lock:
                pulse = car.pi.get_servo_pulsewidth(C.STEER_GPIO)
                result['gpio_read_at'] = time.monotonic()
                result['driver_stop_reason'] = car.reason
                if pulse < 0:
                    result['gpio_read_error'] = int(pulse)
                else:
                    result['gpio_pulse_us'] = pulse
        except Exception as exc:
            result['gpio_read_error'] = str(exc)
    return result


def motion_snapshot(car):
    if car is None:
        return dict(enabled=False, sent_duty=None, gpio=None)
    # Same lock as the driver so the stop watcher cannot interleave the reads.
    with car.lock:
        return dict(enabled=car.enabled, sent_duty=car.current, gpio={
            'enable_pin': car.pi.read(C.ENABLE_GPIO),
            'pwm_duty': car.pi.get_PWM_dutycycle(C.MOTOR_GPIO),
            'pwm_frequency': car.pi.get_PWM_frequency(C.MOTOR_GPIO),
            'pwm_range': car.pi.get_PWM_range(C.MOTOR_GPIO),
        })


def guard_reason(ctl, begin, last_capture, now, seconds, *,
                 processed_capture=None, driving=False):
    if now - begin >= seconds:
        return 'PARK_TEST_TIME_LIMIT'
    # Camera freshness and control-loop progress are separate clocks. A slow
    # startup log must not look like a dead camera while the motor is disabled.
    if (last_capture is None or not math.isfinite(last_capture)
            or not 0 <= now - last_capture <= C.MAX_FRAME_AGE_SECONDS):
        return 'HARD_FRAME_AGE_LIMIT'
    if driving and (processed_capture is None
                    or now - processed_capture > C.MAX_FRAME_AGE_SECONDS):
        return 'CONTROL_LOOP_TIMEOUT'
    parking = ctl.policy.parking
    if (ctl.policy.stage == 'PARKING' and parking.state == 'PASS_TIMED'
            and parking.pass_started_at is not None
            and now >= parking.pass_started_at + parking.cfg['pass_seconds'] + .05):
        return 'HARD_PARK_PASS_LIMIT'
    return None


class YellowLineCapture:
    """Record failed entrance-geometry observations off the control thread."""

    def __init__(self, folder):
        import parking_entry_preview as preview
        self.folder = folder / 'yellow_line_loss'
        # install_test_entry_margin compiles this function in a private namespace.
        self.namespace = preview.detect_entry_geometry.__globals__
        self.original = self.namespace['_hough']
        self.current = None
        self.jobs = queue.Queue(maxsize=48)
        self.saved = 0
        self.dropped = 0
        self.errors = []
        self.worker = threading.Thread(target=self._write_loop, daemon=True)
        self.worker.start()
        self.namespace['_hough'] = self._capture_hough

    def begin_frame(self):
        self.current = None

    def _capture_hough(self, mask, votes, length, gap):
        lines = self.original(mask, votes, length, gap)
        self.current = (mask, lines, votes, length, gap)
        return lines

    def collect(self, raw, fid, elapsed, stage, observation):
        geometry = observation.get('geometry') or {}
        if (self.current is None or observation.get('frame_id') != fid
                or geometry.get('entrance') is not None):
            return
        reasons = geometry.get('reasons') or []
        if not any(reason in ('No sufficiently long yellow transverse candidate',
                              'Insufficient yellow transverse support') for reason in reasons):
            return
        try:
            self.jobs.put_nowait((raw, fid, elapsed, stage, geometry['open_side'],
                                  list(reasons), self.current))
        except queue.Full:
            self.dropped += 1

    @staticmethod
    def _describe(lines, width, height, side, min_length):
        candidates = []
        for line in lines:
            x1, y1, x2, y2 = map(int, line)
            if side == 'RIGHT':
                x1, x2 = width - 1 - x1, width - 1 - x2
            dx, dy = abs(x2 - x1), abs(y2 - y1)
            angle = math.degrees(math.atan2(dy, dx))
            mid_y = (y1 + y2) / 2.
            failed = []
            # OpenCV HoughLinesP uses the larger axis span for minLineLength.
            if max(dx, dy) < min_length:
                failed.append('SHORT_SPAN')
            if angle > 18:
                failed.append('ANGLE_TOO_STEEP')
            if mid_y < .58 * height:
                failed.append('MIDPOINT_TOO_HIGH')
            candidates.append(dict(endpoints=[x1, y1, x2, y2], length_px=math.hypot(dx, dy),
                                   axis_span_px=max(dx, dy), angle_deg=angle,
                                   midpoint_y_px=mid_y, failed=failed))
        return candidates

    def _write(self, raw, fid, elapsed, stage, side, reasons, captured):
        mask, lines, votes, length, gap = captured
        h, w = mask.shape
        def describe(items):
            return self._describe(items, w, h, side, length)
        pixels = cv2.countNonZero(mask)
        data = dict(frame=fid, t=elapsed, stage=stage, geometry_side=side,
                    detector_reasons=reasons, yellow_pixels_in_roi=pixels,
                    thresholds=dict(color_model='yellow_or_local_orange_tape',
                                    base_hsv_low=[12,28,45], base_hsv_high=[43,255,255],
                                    roi_top_px=int(.50*h), min_line_length_px=length,
                                    hough_votes=votes, max_gap_px=gap, max_angle_deg=18,
                                    min_midpoint_y_px=.58*h),
                    candidates=describe(lines))
        if not pixels:
            data['outcome'] = 'EMPTY_YELLOW_ROI'
        elif not len(lines):
            data['outcome'] = 'NO_HOUGH_SEGMENTS'
            # Separate diagnostic reruns. Each changes exactly one Hough setting.
            for name, params in [('short_line_probe', (votes, max(2, length//4), gap)),
                                 ('lower_votes_probe', (max(2, votes//2), length, gap)),
                                 ('larger_gap_probe', (votes, length, 2*gap))]:
                data[name] = describe(self.original(mask, *params))
                data.setdefault('probe_settings', {})[name] = dict(
                    votes=params[0], min_length_px=params[1], gap_px=params[2])
        elif 'Insufficient yellow transverse support' in reasons:
            data['outcome'] = 'INSUFFICIENT_SUPPORT_PIXELS'
        else:
            data['outcome'] = 'CANDIDATES_REJECTED'
        directory = self.folder / ('frame_%06d' % fid)
        directory.mkdir(parents=True, exist_ok=True)
        # Both PNGs and the endpoints in the report use the original camera axes.
        visible_mask = cv2.flip(mask, 1) if side == 'RIGHT' else mask
        for name, image in [('raw.png', raw), ('yellow.png', visible_mask)]:
            if not cv2.imwrite(str(directory/name), image, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                raise OSError('Cannot write ' + str(directory/name))
        (directory/'candidates.json').write_text(json.dumps(data, indent=2), encoding='utf8')

    def _write_loop(self):
        while True:
            job = self.jobs.get()
            if job is None:
                return
            try:
                self._write(*job)
                self.saved += 1
            except Exception as exc:
                self.errors.append(dict(frame=job[1], error=str(exc)))

    def finish(self):
        self.namespace['_hough'] = self.original
        self.jobs.put(None)
        self.worker.join()
        return dict(saved=self.saved, dropped=self.dropped, errors=self.errors)


class LeftLineCapture:
    """Keep a short window of actual detector inputs and outputs for each loss."""
    window = 5
    max_clips = 12
    missing_modes = {'RIGHT_ONLY', 'RIGHT_SEEN', 'RIGHT_JUMP', 'LOST'}

    def __init__(self):
        self.before = deque(maxlen=self.window)
        self.clips = []
        self.pending = []
        self.was_missing = False
        self.skipped = 0

    def add(self, frame, lane, fid, elapsed, stage):
        # Finish any open window even when PARK_SEARCH has just ended.
        if stage != 'PARK_SEARCH' and not self.pending:
            return
        sample = (fid, elapsed, stage, lane.mode, frame, lane.binary)
        for clip, remaining in self.pending:
            clip.append(sample)
        self.pending = [(clip, n - 1) for clip, n in self.pending if n > 1]
        missing = lane.mode in self.missing_modes
        if stage == 'PARK_SEARCH' and missing and not self.was_missing:
            if len(self.clips) < self.max_clips:
                clip = list(self.before) + [sample]
                self.clips.append((fid, clip))
                self.pending.append((clip, self.window))
            else:
                self.skipped += 1
        self.was_missing = missing
        self.before.append(sample)

    def save(self, folder):
        for trigger, clip in self.clips:
            directory = folder / 'left_line_loss' / ('trigger_%06d' % trigger)
            directory.mkdir(parents=True, exist_ok=True)
            rows = []
            for fid, elapsed, stage, mode, raw, binary in clip:
                stem = 'frame_%06d' % fid
                for kind, image in [('raw', raw), ('white', binary)]:
                    path = directory / (stem + '_' + kind + '.png')
                    if not cv2.imwrite(str(path), image):
                        raise OSError('Failed to write ' + str(path))
                rows.append(dict(frame=fid, t=elapsed, stage=stage, lane_mode=mode))
            meta = dict(trigger_frame=trigger, frames=rows,
                        white_roi_top_px=int(clip[0][4].shape[0] * C.LANE_ROI_Y0_RATIO))
            (directory / 'frames.json').write_text(json.dumps(meta, indent=2), encoding='utf8')


def run(args, cfg, ctl):
    import fcntl  # Linux only; --help / --check do not reach this path.

    check_other_programs()
    # Share the original race lock; never drive beside the full race program.
    with open('/tmp/wisecar_race_master.lock', 'a') as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise RuntimeError('另一个全程/停车程序持有控制锁，请先退出') from exc
        folder = ROOT / 'parking_test_runs' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        folder.mkdir(parents=True, exist_ok=False)
        (folder / 'run.json').write_text(json.dumps(dict(
            drive=args.drive, config=cfg, source='parking_only_test.py',
            entry_stage='PARK_SEARCH', qr_enabled=False), ensure_ascii=False, indent=2), encoding='utf8')
        print('[mode]', '实车：自动停车' if args.drive else '观察：电机/云台不会使能', flush=True)
        print('[flow] PARK_SEARCH -> PARK_SETTLE -> PARKING -> STOP（不扫码）', flush=True)
        print('[stop mode] 入口到位或近底部锁定后丢线：沿白线继续，右库0.6秒、左库0.6秒再刹车；保护保留。PWM %s，最长%.2f秒后回12000中位' % (
            ctl.policy.parking.cfg['brake_duty'], ctl.policy.parking.cfg['brake_seconds']), flush=True)
        print('[logs]', folder, flush=True)
        print('[entry line] entry-stop-y=%.2f；需实车校准，此比例不是实际距离' % cfg['entry_stop_y'], flush=True)
        print('[white line] side=%s offset=%.2f' % (cfg['parking_side'], cfg['white_offset']), flush=True)
        car = camera = watcher = None
        done = threading.Event()
        last_capture = [None]
        guard_detail = {}
        phase = ['STARTING']
        previous_signals = {}
        status, frames = 'STARTING', 0
        failure_frames = {}
        left_loss = LeftLineCapture()
        yellow_loss = YellowLineCapture(folder)

        def interrupted(signum, frame):
            raise KeyboardInterrupt

        try:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                previous_signals[sig] = signal.signal(sig, interrupted)
            if args.drive:
                car = FusionIO(C)
                restore_gimbal(C)
            from parking_record_camera import RecordingCamera
            camera = RecordingCamera(folder)
            camera.start()
            begin = time.monotonic()

            def deadlines():
                while not done.wait(.01):
                    # Read the capture thread, not the last frame consumed by
                    # the main loop. No image copy or GPIO operations here.
                    _, latest_id, latest_ts = camera.read(copy=False)
                    now = time.monotonic()
                    reason = guard_reason(ctl, begin, latest_ts, now, args.seconds,
                                          processed_capture=last_capture[0],
                                          driving=car.enabled)
                    if reason:
                        guard_detail.update(reason=reason, phase=phase[0],
                                            latest_frame=latest_id,
                                            camera_age_ms=(now-latest_ts)*1000,
                                            processed_age_ms=(None if last_capture[0] is None
                                                              else (now-last_capture[0])*1000))
                        car.stop(reason)
                        return

            if car is not None:
                watcher = threading.Thread(target=deadlines, daemon=True)
                watcher.start()
            last_id, previous_key = None, None
            last_print = last_image = -math.inf
            with (folder / 'events.jsonl').open('w', encoding='utf8', buffering=1) as log:
                while time.monotonic() - begin < args.seconds:
                    phase[0] = 'READ_FRAME'
                    prepare_step_timing(ctl)
                    frame, fid, ts = measure_step(ctl, 'camera_read_ms', camera.read)
                    now = time.monotonic()
                    ctl.step_timing['frame_age_at_read_ms'] = (now-ts)*1000 if ts is not None else None
                    if car is not None and car.reason:
                        status = car.reason
                        break
                    if frame is None or not math.isfinite(ts) or not 0 <= now - ts <= C.MAX_FRAME_AGE_SECONDS:
                        status = 'STALE_FRAME'
                        break
                    if fid == last_id:
                        time.sleep(.002)
                        continue
                    last_id, last_capture[0] = fid, ts
                    phase[0] = 'PERCEPTION'
                    yellow_loss.begin_frame()
                    observation_stage = ctl.policy.stage
                    cmd = measure_step(ctl, 'controller_total_ms', ctl.step, frame, fid, ts, now)
                    if cmd is None:
                        continue
                    ctl.step_timing['frame_age_after_control_ms'] = (time.monotonic()-ts)*1000
                    if ctl.step_timing['frame_age_after_control_ms'] > C.MAX_FRAME_AGE_SECONDS*1000:
                        cmd = ctl.policy.halt('PROCESSING_TOO_SLOW')
                    phase[0] = 'DRIVE_COMMAND'
                    driver_cmd = measure_step(ctl, 'drive_command_ms', drive_command, car, cmd, ts)
                    key = (ctl.policy.stage, cmd.state)
                    if key != previous_key:
                        ctl.policy.mark_applied(cmd, time.monotonic())
                    frames += 1
                    lane = ctl.lane_result
                    yellow_loss.collect(frame, fid, now-begin, observation_stage,
                                        ctl.parking_observation or {})
                    if 'ordinary_lane_ms' in ctl.step_timing:
                        left_loss.add(frame, lane, fid, now-begin, ctl.policy.stage)
                    event = dict(t=now-begin, frame=fid, stage=ctl.policy.stage,
                                 state=cmd.state, reason=cmd.reason, target_duty=driver_cmd.duty,
                                 policy_target_duty=cmd.duty, driver_state=driver_cmd.state,
                                 sent_duty=car.current if car is not None else None,
                                 enabled=car.enabled if car is not None else False,
                                 lane_valid=getattr(lane, 'valid', False),
                                 lane_mode=getattr(lane, 'mode', None),
                                 lane_hits=ctl.policy.lane_hits,
                                 parking=ctl.parking_observation,
                                 timing_ms=dict(ctl.step_timing))
                    event['steering'] = measure_step(ctl, 'steering_readback_ms',
                        steering_snapshot, ctl, cmd, driver_cmd, car, ts)
                    event['entry_delay_started_at'] = ctl.policy.parking.entry_delay_started_at
                    event['entry_delay_deadline'] = ctl.policy.parking.entry_delay_deadline
                    event['timing_ms'] = dict(ctl.step_timing)
                    guide = (ctl.parking_observation or {}).get('lane_guide') or {}
                    failure = guide.get('reason')
                    entry_line = (ctl.parking_observation or {}).get('entry_line') or {}
                    if (cmd.reason in ('ENTRY_YELLOW_DELAY_DRIVE', 'ENTRY_YELLOW_DELAY_COMPLETE', 'ENTRY_YELLOW_LINE_LOST',
                                       'PROCESSING_TOO_SLOW')
                            and cmd.reason not in failure_frames):
                        failure_frames[cmd.reason] = (frame.copy(), deepcopy(event))
                    if (ctl.policy.stage == 'PARKING' and not guide.get('valid')
                            and failure and failure != 'OUTER_WHITE_CONFIRMING'
                            and failure not in failure_frames and len(failure_frames) < 12):
                        # Bounded RAM copies only while driving; disk writes
                        # happen after shutdown, avoiding extra image I/O stalls.
                        failure_frames[failure] = (frame.copy(), deepcopy(event))
                    terminal = cmd.state in ('HALT', 'DONE')
                    phase[0] = 'DIAGNOSTICS'
                    diagnostics_started = time.monotonic()
                    if key != previous_key or now - last_print >= .5 or terminal:
                        event.update(motion_snapshot(car))
                        print('[state] ' + json.dumps({k: v for k, v in event.items() if k != 'parking'},
                                                      ensure_ascii=False), flush=True)
                        if cmd.reason in HINTS:
                            print('[说明]', HINTS[cmd.reason], flush=True)
                        last_print = now
                    output_done = time.monotonic()
                    diagnostics_ms = (output_done-diagnostics_started)*1000
                    log.write(json.dumps(event, ensure_ascii=False) + '\n')
                    log_ms = (time.monotonic()-output_done)*1000
                    image_ms = 0.
                    if now - last_image >= 1 or terminal:
                        phase[0] = 'SAVE_IMAGES'
                        image_started = time.monotonic()
                        cv2.imwrite(str(folder / 'latest_raw.jpg'), frame)
                        cv2.imwrite(str(folder / 'latest_debug.jpg'), draw_frame(frame, ctl, cmd))
                        last_image = now
                        image_ms = (time.monotonic()-image_started)*1000
                    log.write(json.dumps(dict(type='loop_io_timing', frame=fid,
                        timing_ms=dict(log_write_ms=log_ms, save_images_ms=image_ms,
                                       terminal_output_ms=diagnostics_ms)))+'\n')
                    previous_key = key
                    if terminal:
                        status = car.reason if car is not None and car.reason else cmd.reason
                        break
                else:
                    status = 'PARK_TEST_TIME_LIMIT'
        except KeyboardInterrupt:
            status = 'INTERRUPTED'
        except Exception as exc:
            status = 'ERROR: ' + str(exc)
        finally:
            # Stop the motor before waiting on threads or releasing the camera.
            if car is not None:
                car.stop(status)
                status = car.reason or status
            done.set()
            if watcher is not None:
                watcher.join(timeout=.3)
            try:
                close_motion(car, camera)
            finally:
                for sig, handler in previous_signals.items():
                    signal.signal(sig, handler)
        summary = dict(status=status, frames=frames, stage=ctl.policy.stage,
                       ever_armed=bool(car and car.ever_armed),
                       peak_sent_duty=car.peak if car else None,
                       parking_verified=False, qr_enabled=False,
                       guard_detail=guard_detail)
        if args.drive and car is not None and car.ever_armed:
            from parked_photos import capture_pair
            summary['parked_photos'] = capture_pair(folder)
        summary['yellow_line_loss'] = yellow_loss.finish()
        summary['left_loss_clips'] = len(left_loss.clips)
        summary['left_loss_skipped'] = left_loss.skipped
        try:
            left_loss.save(folder)
        except (OSError, cv2.error) as exc:
            summary['left_loss_save_error'] = str(exc)
            print('[log warning]', exc, flush=True)
        for index, (reason, (raw, event)) in enumerate(failure_frames.items()):
            stem = folder / ('failure_%02d_frame_%s_%s' % (index, event['frame'], reason))
            if not cv2.imwrite(str(stem)+'.jpg', raw):
                print('[log warning] Failed to save ' + str(stem), flush=True)
            Path(str(stem)+'.json').write_text(
                json.dumps(event, ensure_ascii=False, indent=2), encoding='utf8')
        (folder / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf8')
        print('[result]', json.dumps(summary, ensure_ascii=False), flush=True)
        print('[说明]', HINTS.get(status, status), flush=True)
        return 0 if status == 'PARKED_QR_READY' else 130 if status == 'INTERRUPTED' else 1


def main(argv=None):
    args = parse_args(argv)
    cfg = load_config(args.config)
    cfg['cruise_duty'] = cfg['parking_duty'] = 12700
    cfg['max_seconds'] = args.seconds
    cfg['lane_mode'] = 'original'
    cfg['entry_stop_y'] = args.entry_stop_y
    cfg['parking_side'] = args.parking_side.upper()
    cfg['white_offset'] = args.white_offset
    cfg['left_pink'] = True
    cv2.setNumThreads(1)
    with runtime_config(True, cfg['lane_roi']):
        ctl = parking_controller(cfg)
        if (args.drive or args.check) and importlib.util.find_spec('pigpio') is None:
            raise RuntimeError('缺少pigpio Python模块；实车测试需要树莓派原运行环境')
        if args.check:
            print('[check] 停车控制器及依赖可加载；未打开硬件，未验证电调或车轮运动')
            print('[PWM] freq=%s range=%s stop=%s search=%s parking=%s' % (
                C.MOTOR_PWM_FREQ, C.MOTOR_PWM_RANGE, C.STOP_DUTY,
                cfg['cruise_duty'], cfg['parking_duty']))
            return 0
        if args.drive:
            check_other_programs()
            print('实车自动停车；首次先架空检查驱动/停止，确认其他车控已退出。', flush=True)
            try:
                input('车放在停车入口前的白线车道，车头朝停车区。按 Enter 开始；Ctrl+C 取消：')
            except (KeyboardInterrupt, EOFError):
                print('\n[cancelled] 未启动硬件')
                return 130
        return run(args, cfg, ctl)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('[error]', exc, file=sys.stderr, flush=True)
        sys.exit(1)
