"""Near-edge crosswalk measurements over the console's existing camera caches.

No camera, network, file or actuator is opened. Every view retains its real
frame number/time and its own detector, lane fit and optical-motion history.
Only the selected fresh view can produce a new controller batch. Switching
normally prefers primary at close range; either view can request braking.
"""
import copy
import math

import cv2
import numpy as np

from .brake_parking import GroundMotionEstimate
from .config import load_config
from .crosswalk import CrosswalkDetector, CrosswalkVisionConfig
from .crosswalk_calibration import IMAGE_FEATURE_NEAR, estimate_distance, validate_profile
from .lane import LaneDetector


CAMERAS = ('primary', 'secondary')
DEFAULT_MASKS = {'primary': {'s_max': 80, 'v_min': 200},
                 'secondary': {'s_max': 100, 'v_min': 170}}


def _number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


class CalibratedTrialCamera:
    """Read both calibrated views and return one source with unmodified timing."""

    def __init__(self, states, settings, *, clock=None):
        self.settings = copy.deepcopy(settings)
        self.clock=clock
        self.profile = validate_profile(self.settings.get('crosswalk_calibration'), require_ready=True)
        if self.profile['image_feature'] != IMAGE_FEATURE_NEAR:
            raise ValueError('双摄运行读取器只接受独立近边标定，不能把旧远边表当近边')
        self.max_age = self.settings.get('crosswalk_max_frame_age_s',self.settings.get('max_frame_age_s'))
        if not _number(self.max_age) or not 0 < self.max_age <= .5:
            raise ValueError('双摄标定读取器图像期限必须在0–500毫秒内')
        if not isinstance(states, dict) or any(camera not in states for camera in CAMERAS):
            raise ValueError('双摄标定需要现有primary和secondary画面缓存')
        self.states = dict(states)
        self.trigger = self.settings.get('crosswalk_distance_trigger_m', self.profile['overlap_range_m'][0])
        if not _number(self.trigger) or self.trigger <= 0:
            raise ValueError('双摄制动触发距离无效')
        self.detectors, self.lanes, self.motions = {}, {}, {}
        for camera in CAMERAS:
            override = self.profile['cameras'][camera].get('white_mask', {})
            if not isinstance(override, dict) or set(override) - {'s_max', 'v_min'}:
                raise ValueError(camera + ' 白色阈值格式无效')
            mask = {**DEFAULT_MASKS[camera], **override}
            if any(type(mask[key]) is not int or not 0 <= mask[key] <= 255 for key in ('s_max', 'v_min')):
                raise ValueError(camera + ' 白色阈值必须为0–255整数')
            self.detectors[camera] = CrosswalkDetector(CrosswalkVisionConfig(
                white_s_max=mask['s_max'], white_v_min=mask['v_min'], min_stripes=3))
            lane_config = copy.deepcopy(load_config(None).lane)
            lane_config.white_s_max, lane_config.white_v_min = mask['s_max'], mask['v_min']
            self.lanes[camera] = LaneDetector(lane_config)
            self.motions[camera] = GroundMotionEstimate()
        self.rows, self.ids, self.times, self.jpegs = {}, {}, {}, {}
        self.selection_revision = 'crosswalk_single_source_v1'
        self.distance_disagreement = None
        self.active_source = 'secondary'
        self.primary_locked = False
        self.confirmed_primary_frames = 0
        self.confirmation_id = None
        self.sequence = 0
        self.last_returned_capture = None
        self.last_returned_source = None
        self.last_read = None
        self.jpeg = None

    def _update(self, camera, now):
        state = self.states[camera]
        try:
            with state.condition:
                jpeg, frame_id, captured_ms = state.raw_jpeg, state.raw_frame_id, state.raw_received_ms
                failed = state.state in ('error', 'stopped')
                identity = copy.deepcopy(getattr(state, 'identity', None))
        except (AttributeError, TypeError) as exc:
            raise ValueError(camera + ' 摄像头缓存结构无效') from exc
        if (not isinstance(identity, dict) or not isinstance(identity.get('name'), str)
                or not isinstance(identity.get('bus'), str)):
            raise ValueError(camera + ' 缺少可核验的物理摄像头身份')
        name = identity['name'] + ' | ' + identity['bus']
        if name != self.profile['cameras'][camera]['identity']:
            raise ValueError(camera + ' 摄像头身份与本次独立标定不一致')
        # A second camera can publish a new frame while the first is processed.
        # Compare it with the clock AFTER its own cache snapshot, not read-entry.
        checked=self.clock() if self.clock is not None else now
        if not _number(checked) or checked<now:
            raise ValueError(camera+' 摄像头校验时钟无效或倒退')
        if (failed or not isinstance(jpeg, (bytes, bytearray)) or not jpeg
                or type(frame_id) is not int or frame_id < 0 or not _number(captured_ms)):
            raise ValueError(camera+' 摄像头缓存不可用：状态、图像或采样信息无效')
        age=checked-captured_ms/1000
        if not 0<=age<=self.max_age:
            raise ValueError(camera+f' 摄像头画面失效（采样时间异常）：帧龄{age*1000:.1f}毫秒，允许0–{self.max_age*1000:g}毫秒')
        stamp = captured_ms / 1000
        if camera in self.ids:
            if frame_id < self.ids[camera] or stamp < self.times[camera]:
                raise ValueError(camera + ' 图像编号或采样时钟倒退，需重新准备')
            if frame_id == self.ids[camera]:
                if stamp != self.times[camera]:
                    raise ValueError(camera + ' 重复图像编号不能伪造新时间')
                return False
            if stamp <= self.times[camera]:
                raise ValueError(camera + ' 新图像没有递增的真实采样时间')
        try:
            image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        except (cv2.error, ValueError, TypeError) as exc:
            raise ValueError(camera + ' 原始JPEG不能解码') from exc
        if image is None or [image.shape[1], image.shape[0]] != [480, 360]:
            raise ValueError(camera + ' 画面必须保持本次480×360独立标定尺寸')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if float(gray.mean()) < 15 or float(gray.std()) < 1 or float((gray >= 250).mean()) > .98:
            raise ValueError(camera + ' 画面全黑、被遮挡或过曝，不能作为搜索依据')
        zebra, _ = self.detectors[camera].detect(image, captured_ms)
        candidate = zebra['candidate'] is True
        clipped = candidate and zebra['bottom_clipped'] is True
        # The detector still demands >=3 horizontally arranged strips. Never
        # treat the three depth-wise calibration papers as a crosswalk row.
        strips = zebra['stripes_xywh']
        near_y = (float(np.median([y + h - 1 for _, y, _, h in strips])) / (image.shape[0] - 1)
                  if candidate and not clipped else None)
        distance = None
        metric_valid = False
        near_limit = bool(clipped)
        status = 'too_near' if clipped else 'not_detected'
        if near_y is not None:
            refs = self.profile['cameras'][camera]['references']
            ys = [row['near_edge_y_normalized'] for row in refs]
            if near_y < min(ys):
                status = 'beyond_far'
            elif near_y > max(ys):
                status, near_limit = 'too_near', True
            else:
                estimate = estimate_distance(self.profile, camera, near_y)
                distance, metric_valid, status = estimate['distance_m'], True, 'inside'
        clean = image.copy()
        for x, y, w, h in strips:
            clean[y:y + h, x:x + w] = 0
        lane, _ = self.lanes[camera].detect(clean)
        motion = self.motions[camera].update(image, frame_id, stamp)
        self.rows[camera] = {
            'camera_id': camera, 'camera_identity': name, 'frame_id': frame_id,
            'raw_frame_id': frame_id, 'raw_received_ms': captured_ms, 'captured_s': stamp,
            'fresh': True, 'image_size': [480, 360], 'candidate': candidate,
            'presence': zebra['presence'], 'stripes_xywh': copy.deepcopy(strips),
            'bottom_clipped': bool(clipped), 'near_edge_y_normalized': near_y,
            'far_edge_y_normalized': zebra['far_edge_y_normalized'],
            'near_limit_reached': near_limit, 'metric_valid': metric_valid,
            'distance_estimate_m': distance, 'distance_status': status,
            'lane_valid': bool(lane.valid), 'lane_offset': lane.offset_normalized,
            'motion': motion, 'calibration_id': self.profile['profile_id'],
            'physical_stop_verified': False,
        }
        self.ids[camera], self.times[camera] = frame_id, stamp
        self.jpegs[camera] = bytes(jpeg)
        return True

    def _danger(self, row):
        return row['candidate'] and (row['near_limit_reached']
            or row['metric_valid'] and row['distance_estimate_m'] <= self.trigger)

    def _select(self, primary_changed):
        primary, secondary = self.rows['primary'], self.rows['secondary']
        self.distance_disagreement = None
        if primary['metric_valid'] and secondary['metric_valid']:
            delta = abs(primary['distance_estimate_m'] - secondary['distance_estimate_m'])
            if delta > .05 + 1e-9:
                # The views are not target-associated or simultaneous. Record
                # their native measurements, never average them or abort for
                # this difference alone. Each selected view keeps its own table.
                self.distance_disagreement = {
                    'threshold_m': .05, 'difference_m': delta,
                    'primary_distance_m': primary['distance_estimate_m'],
                    'secondary_distance_m': secondary['distance_estimate_m'],
                    'capture_skew_ms': abs(primary['captured_s'] - secondary['captured_s']) * 1000,
                    'captured_s': {camera: self.rows[camera]['captured_s'] for camera in CAMERAS},
                    'raw_frame_ids': {camera: self.rows[camera]['raw_frame_id'] for camera in CAMERAS},
                    'target_association_verified': False,
                    'policy': 'one_source_without_distance_averaging',
                }
        if primary['near_limit_reached']:
            self.active_source, self.primary_locked = 'primary', True
            return
        if secondary['near_limit_reached']:
            self.active_source = 'secondary'
            self.confirmed_primary_frames, self.confirmation_id = 0, None
            return
        if self.primary_locked:
            # Primary remains preferred, but cannot hide a secondary stop
            # request. This override is a real observation, not a fused value.
            if self._danger(secondary) and (not self._danger(primary)
                    or secondary['distance_estimate_m'] < primary['distance_estimate_m']):
                self.active_source = 'secondary'
            else:
                self.active_source = ('secondary' if not primary['candidate']
                                      and secondary['candidate'] else 'primary')
            return
        if self._danger(primary) and (not self._danger(secondary)
                or primary['distance_estimate_m'] <= secondary['distance_estimate_m']):
            self.active_source, self.primary_locked = 'primary', True
            return
        if self._danger(secondary):
            self.active_source = 'secondary'
            self.confirmed_primary_frames, self.confirmation_id = 0, None
            return
        self.active_source = 'secondary'
        if primary['candidate'] != secondary['candidate']:
            # One recognized view suffices; the other need not recognize a row.
            self.active_source = 'primary' if primary['candidate'] else 'secondary'
            self.confirmed_primary_frames, self.confirmation_id = 0, None
            return
        lo, hi = self.profile['overlap_range_m']
        measured_overlap = (primary['metric_valid'] and secondary['metric_valid']
            and lo <= primary['distance_estimate_m'] <= hi
            and lo <= secondary['distance_estimate_m'] <= hi
            and primary['distance_estimate_m'] <= .5)
        if not measured_overlap:
            self.confirmed_primary_frames, self.confirmation_id = 0, None
        elif primary_changed and primary['frame_id'] != self.confirmation_id:
            self.confirmation_id = primary['frame_id']
            self.confirmed_primary_frames += 1
            if self.confirmed_primary_frames >= 2:
                self.active_source, self.primary_locked = 'primary', True

    def read(self, now):
        if not _number(now) or self.last_read is not None and now < self.last_read:
            raise ValueError('双摄读取控制时钟无效或倒退')
        self.last_read = now
        changed = {camera: self._update(camera, now) for camera in CAMERAS}
        checked=self.clock() if self.clock is not None else now
        if not _number(checked) or checked<now:raise ValueError('双摄处理完成时钟无效或倒退')
        ages={camera:checked-self.rows[camera]['captured_s'] for camera in CAMERAS}
        for camera,age in ages.items():
            if not 0<=age<=self.max_age:
                raise ValueError(camera+f' 摄像头处理后帧龄{age*1000:.1f}毫秒，超出{self.max_age*1000:g}毫秒期限')
        self._select(changed['primary'])
        row = self.rows[self.active_source]
        stamp = row['captured_s']
        if (not changed[self.active_source]
                or self.last_returned_capture is not None and stamp <= self.last_returned_capture):
            return None  # No cached frame or older cross-camera sample renews a lease.
        self.sequence += 1
        changed_source = (self.last_returned_source is not None
                          and self.active_source != self.last_returned_source)
        result = copy.deepcopy(row)
        result.update(frame_id=self.sequence, source_camera=self.active_source,
                      source_changed=changed_source,
                      selection_revision=self.selection_revision,
                      distance_disagreement=copy.deepcopy(self.distance_disagreement),
                      frame_age_limit_ms=self.max_age*1000,
                      camera_ages_ms={camera:round(age*1000,2) for camera,age in ages.items()},
                      camera_observations=copy.deepcopy(self.rows))
        self.last_returned_capture, self.last_returned_source = stamp, self.active_source
        self.jpeg = self.jpegs[self.active_source]
        return result
