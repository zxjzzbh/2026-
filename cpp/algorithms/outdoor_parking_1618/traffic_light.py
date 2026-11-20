"""先找正在发光的灯，再连续确认颜色；灯壳只辅助识别和测位置，不访问硬件。"""

from dataclasses import dataclass, field

import cv2
import numpy as np

DETECTOR_VERSION = 'DIRECT_LIGHT_LED_GREEN_LOCAL_V3'


@dataclass
class LightResult:
    """保存一帧的灯色、灯壳位置和连续确认次数，供比赛控制与调试画面使用。"""

    visible: bool = False
    color: str = 'UNKNOWN'
    confirmed: bool = False
    width_ratio: float = 0.0
    bbox: object = None
    scores: dict = field(default_factory=dict)
    streak: int = 0
    lamp_bbox: object = None
    position_valid: bool = False
    source: str = 'none'


class TrafficLightDetector:
    """按给定参数寻找灯壳和灯色，并记录同一盏灯连续出现的次数。"""

    def __init__(self, cfg):
        """接收红绿灯配置，建立检测器并清空上一帧的确认记录。"""
        self.c = cfg
        self.last_box = None
        self.last_color = 'UNKNOWN'
        self.streak = 0

    def reset_confirmation(self):
        """清空灯壳和灯色记录，使之后的画面重新累计确认次数。"""
        self.last_box = None
        self.last_color = 'UNKNOWN'
        self.streak = 0

    def _color_scores(self, hsv, box):
        """接收颜色图和一个灯壳框，返回红、黄、绿各自灯位中亮色所占的比例。"""
        cfg = self.c
        x, y, width, height = box
        scores = {}
        for index, name in enumerate(('red', 'yellow', 'green')):
            # 每个灯位只看中间部分，避开灯壳边缘。
            left = x + int((index + 0.10) * width / 3)
            right = x + int((index + 0.90) * width / 3)
            top = y + int(height * 0.10)
            bottom = y + int(height * 0.90)
            cell = hsv[top:bottom, left:right]
            mask = np.zeros(cell.shape[:2], np.uint8)
            for low, high in cfg[name + '_h']:
                mask |= cv2.inRange(
                    cell, (low, cfg['s_min'], cfg['v_min']), (high, 255, 255)
                )
            count = cv2.countNonZero(mask)
            scores[name] = (
                count / float(max(1, mask.size))
                if count >= cfg['min_lit_pixels']
                else 0.0
            )
        return scores

    @staticmethod
    def _candidate_priority(candidate):
        """接收一个灯壳候选项，返回排序依据：先选灯色明确的，再选更宽的。"""
        return candidate[2] != 'UNKNOWN', candidate[0]

    def _housing_candidates(self, frame):
        """寻找当前画面里完整的黑色灯壳；找不到也不影响直接识别亮灯。"""
        cfg = self.c
        height, width = frame.shape[:2]
        x0, y0, x1, y1 = [
            int(value * size)
            for value, size in zip(cfg['roi'], (width, height, width, height))
        ]
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dark = cv2.inRange(hsv, (0, 0, 0), (179, 255, cfg['housing_v_max']))

        candidates = []
        contours = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[
            -2
        ]
        for contour in contours:
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if (
                box_width < cfg['min_width_px'] * width / 352
                or box_height < cfg['min_height_px'] * height / 288
                or not cfg['aspect_min']
                <= box_width / float(box_height)
                <= cfg['aspect_max']
                or x <= 0
                or y <= 0
                or x + box_width >= roi.shape[1]
                or y + box_height >= roi.shape[0]
            ):
                continue
            dark_pixels = cv2.countNonZero(dark[y : y + box_height, x : x + box_width])
            if dark_pixels / float(box_width * box_height) < cfg['min_dark_fraction']:
                continue

            scores = self._color_scores(hsv, (x, y, box_width, box_height))
            lit = [
                name
                for name, score in scores.items()
                if score >= cfg['min_lit_fraction']
            ]
            color = lit[0].upper() if len(lit) == 1 else 'UNKNOWN'
            box = (x + x0, y + y0, box_width, box_height)
            candidates.append((box_width, box, color, scores))

        return candidates

    def _direct_candidates(self, frame):
        """找小而集中的亮灯芯，再看灯芯周围的彩色光，避免白色过曝把灯色盖住。"""
        height, width = frame.shape[:2]
        x0, y0, x1, y1 = [int(v*s) for v, s in
                          zip(self.c['roi'], (width, height, width, height))]
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # 亮灯中央经常接近白色；颜色要到外围找，不能要求中央也保持高饱和度。
        # 同时检查偏白、略带颜色的两种亮灯芯；避免曝光变化把灯芯切成碎块。
        cores = [np.uint8((np.min(roi, axis=2) >= level) &
                          (hsv[:, :, 2] >= 230)) * 255 for level in (190, 130)]
        masks = {}
        for name in ('red', 'yellow', 'green'):
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in self.c[name + '_h']:
                mask |= cv2.inRange(hsv, (lo, self.c['s_min'], self.c['v_min']),
                                    (hi, 255, 255))
            masks[name] = mask
        candidates = []
        contours = []
        for core in cores:
            contours.extend(cv2.findContours(core, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)[-2])
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if (min(w, h) < max(3, 3*width/352) or max(w, h) > .18*width
                    or not .5 <= w/float(h) <= 2.0 or area < .45*w*h):
                continue
            cx, cy = x + (w-1)/2., y + (h-1)/2.
            radius = max(2., np.sqrt(area/np.pi))
            reach = int(np.ceil(2.2*radius))
            left, top = max(0, int(cx)-reach), max(0, int(cy)-reach)
            right, bottom = min(roi.shape[1], int(cx)+reach+1), min(roi.shape[0], int(cy)+reach+1)
            yy, xx = np.ogrid[top:bottom, left:right]
            distance = (xx-cx)**2 + (yy-cy)**2
            ring = (distance >= (.85*radius)**2) & (distance <= (2.2*radius)**2)
            if np.count_nonzero(ring) < .8*np.pi*((2.2*radius)**2-(.85*radius)**2):
                continue  # 灯芯贴着观察范围边缘，只剩半圈时不猜颜色。
            scores = {name: float(np.count_nonzero((mask[top:bottom, left:right] > 0) & ring)) /
                      max(1, np.count_nonzero(ring)) for name, mask in masks.items()}
            order = sorted(scores, key=scores.get, reverse=True)
            name = order[0]
            if scores[name] < .35 or scores[order[1]] > .15:
                continue
            colored = (masks[name][top:bottom, left:right] > 0) & ring
            # 彩色光至少分布在三个方向，过滤白色物体旁边恰好有一块彩色背景。
            quadrants = [(xx < cx) & (yy < cy), (xx >= cx) & (yy < cy),
                         (xx < cx) & (yy >= cy), (xx >= cx) & (yy >= cy)]
            if sum(np.count_nonzero(colored & q) >= .15*np.count_nonzero(ring & q)
                   for q in quadrants) < 3:
                continue
            box = (left+x0, top+y0, right-left, bottom-top)
            candidates.append((scores[name], box, name.upper(), scores))
        return candidates

    def _led_candidates(self, frame):
        """Group small lit LEDs; reject flat unlit lenses and clipped reflections."""
        height, width = frame.shape[:2]
        x0, y0, x1, y1 = [int(v*s) for v, s in
                          zip(self.c['roi'], (width, height, width, height))]
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        candidates = []
        scale = width / 352.
        kernel = max(3, int(round(7*scale)) | 1)
        # 红灯沿用现有高亮识别；绿灯由局部亮点分支处理。
        for name in ('red',):
            support = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in self.c[name+'_h']:
                support |= cv2.inRange(hsv, (lo, 60, 150), (hi, 255, 255))
            near = cv2.dilate(support, np.ones((3, 3), np.uint8))
            bright = np.uint8((hsv[:, :, 2] >= 230) & (near > 0))*255
            joined = cv2.morphologyEx(bright, cv2.MORPH_CLOSE,
                                     np.ones((kernel, kernel), np.uint8))
            for contour in cv2.findContours(joined, cv2.RETR_EXTERNAL,
                                             cv2.CHAIN_APPROX_SIMPLE)[-2]:
                x, y, w, h = cv2.boundingRect(contour)
                if (min(w, h) < max(5, 8*scale) or max(w, h) > .18*width
                        or not .7 <= w/float(h) <= 1.4
                        or x <= 0 or y <= 0 or x+w >= roi.shape[1]
                        or y+h >= roi.shape[0]):
                    continue
                lit = bright[y:y+h, x:x+w] > 0
                count = int(np.count_nonzero(lit))
                if count < max(6, 20*scale*scale) or np.mean(lit) < .10:
                    continue
                if np.count_nonzero((support[y:y+h, x:x+w] > 0) & lit) < 4:
                    continue
                pad = max(3, int(.3*max(w, h)))
                left, top = max(0, x-pad), max(0, y-pad)
                right, bottom = min(roi.shape[1], x+w+pad), min(roi.shape[0], y+h+pad)
                ring = np.ones((bottom-top, right-left), bool)
                ring[y-top:y-top+h, x-left:x-left+w] = False
                surround = hsv[top:bottom, left:right, 2][ring]
                contrast = (float(np.percentile(hsv[y:y+h, x:x+w, 2][lit], 90))
                            - float(np.median(surround)))
                if contrast < 35:
                    continue
                score = count / float(w*h)
                scores = dict(red=0., yellow=0., green=0.)
                scores[name] = score
                candidates.append((score, (x+x0, y+y0, w, h), name.upper(), scores))
        return candidates + self._green_led_candidates(frame, hsv=hsv)

    def _green_led_candidates(self, frame, hsv=None):
        """识别绿色散点 LED：局部亮点、空间分布及周边对比度共同确认。"""
        height, width = frame.shape[:2]
        x0, y0, x1, y1 = [int(v*s) for v, s in
                          zip(self.c['roi'], (width, height, width, height))]
        roi = frame[y0:y1, x0:x1]
        # 红、绿分支共用本帧颜色图，避免在树莓派上重复转换。
        if hsv is None:
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        value = hsv[:, :, 2]
        scale = width / 352.
        kernel = max(3, int(round(7*scale)) | 1)
        support = np.zeros(value.shape, np.uint8)
        for low, high in self.c['green_h']:
            support |= cv2.inRange(hsv, (low, 50, 100), (high, 255, 255))
        near = cv2.dilate(support, np.ones((3, 3), np.uint8))
        # 去掉平滑灯罩亮度，保留相对附近更亮的 LED 小点。
        local_brightness = cv2.morphologyEx(
            value, cv2.MORPH_TOPHAT,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel, kernel)))
        peaks = np.uint8((value >= 155) & (local_brightness >= 18)
                         & (near > 0)) * 255
        grouped = cv2.morphologyEx(peaks, cv2.MORPH_CLOSE,
                                  np.ones((kernel, kernel), np.uint8))
        candidates = []
        contours = cv2.findContours(grouped, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)[-2]
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (min(w, h) < max(5, 8*scale) or max(w, h) > .18*width
                    or not .7 <= w/float(h) <= 1.4
                    or x <= 0 or y <= 0 or x+w >= roi.shape[1]
                    or y+h >= roi.shape[0]):
                continue
            lit = peaks[y:y+h, x:x+w]
            count = cv2.countNonZero(lit)
            fraction = count / float(w*h)
            _, _, stats, centroids = cv2.connectedComponentsWithStats(lit)
            dots = [p for s, p in zip(stats[1:], centroids[1:])
                    if s[4] >= max(1, scale*scale)]
            pad = max(3, int(.3*max(w, h)))
            left, top = max(0, x-pad), max(0, y-pad)
            right = min(roi.shape[1], x+w+pad)
            bottom = min(roi.shape[0], y+h+pad)
            ring = np.ones((bottom-top, right-left), bool)
            ring[y-top:y-top+h, x-left:x-left+w] = False
            surround = value[top:bottom, left:right][ring]
            contrast = (float(np.percentile(value[y:y+h, x:x+w][lit > 0], 90))
                        - float(np.median(surround))) if count and surround.size else 0.
            green_fraction = cv2.countNonZero(support[y:y+h, x:x+w]) / float(w*h)
            span = np.ptp(np.asarray(dots), axis=0) if len(dots) > 1 else np.array([0., 0.])
            # 单个反光点、平坦绿色物体或成线噪声不作为通行绿灯。
            if (count < max(8, 12*scale*scale) or not .035 <= fraction <= .65
                    or len(dots) < 3 or span[0] < .35*w or span[1] < .35*h
                    or contrast < 35 or green_fraction < .15):
                continue
            candidates.append((fraction, (x+x0, y+y0, w, h), 'GREEN',
                               dict(red=0., yellow=0., green=fraction)))
        return candidates

    @staticmethod
    def _matching_housing(lamp, housings):
        """只有亮灯落在完整灯壳的正确灯位，才允许用这个灯壳的宽度量位置。"""
        _, box, color, _ = lamp
        lx, ly, lw, lh = box
        cx, cy = lx+lw/2., ly+lh/2.
        index = ('RED', 'YELLOW', 'GREEN').index(color)
        matches = []
        for item in housings:
            _, (x, y, w, h), housing_color, _ = item
            if (housing_color == color and y < cy < y+h
                    and x+(index+.15)*w/3 < cx < x+(index+.85)*w/3
                    and .2*h <= lh <= 1.8*h):
                matches.append(item)
        return max(matches, key=lambda item: item[0]) if matches else None

    def update(self, frame):
        """每张新画面独立找灯；颜色或位置变了就重新计数，不沿用消失的绿灯。"""
        cfg = self.c
        height, width = frame.shape[:2]
        led_mode = cfg.get('led_cluster_enabled', False)
        housings = [] if led_mode else self._housing_candidates(frame)
        direct = self._led_candidates(frame) if led_mode else self._direct_candidates(frame)
        housing = None
        if direct:
            # 同时看见不同亮灯时不放行，不能挑最大的一盏绿灯忽略红灯。
            if len({item[2] for item in direct}) != 1:
                self.reset_confirmation()
                return LightResult(visible=True, source='conflict')
            selected = max(direct, key=lambda item: item[0])
            _, box, color, scores = selected
            housing = self._matching_housing(selected, housings)
            source = ('green_led_local_contrast' if color == 'GREEN' else 'led_cluster') if led_mode else 'direct'
            lamp_box = box
        else:
            known = [item for item in housings if item[2] != 'UNKNOWN']
            if not known or len({item[2] for item in known}) != 1:
                self.reset_confirmation()
                return LightResult()
            housing = max(known, key=self._candidate_priority)
            _, box, color, scores = housing
            source, lamp_box = 'housing', None

        same = False
        if self.last_box:
            old_x, old_y, old_width, old_height = self.last_box
            x, y, box_width, box_height = box
            same = (
                abs((x + box_width / 2) - (old_x + old_width / 2)) < 0.10 * width
                and abs((y + box_height / 2) - (old_y + old_height / 2)) < 0.10 * height
                and max(box_width / old_width, old_width / box_width) < 1.7
            )
        if same and color == self.last_color and color != 'UNKNOWN':
            self.streak += 1
        else:
            self.streak = 0 if color == 'UNKNOWN' else 1
        self.last_box, self.last_color = box, color
        count = cfg['green_frames'] if color == 'GREEN' else cfg['confirm_frames']
        return LightResult(visible=True, color=color, confirmed=self.streak >= count,
                           width_ratio=housing[1][2]/float(width) if housing else 0.,
                           bbox=housing[1] if housing else None, scores=scores,
                           streak=self.streak, lamp_bbox=lamp_box,
                           position_valid=housing is not None, source=source)

    @staticmethod
    def draw(frame, result, cfg):
        """根据检测结果和配置，在图像副本上画出观察区域、灯壳及灯色文字。"""
        out = frame.copy()
        height, width = out.shape[:2]
        x0, y0, x1, y1 = [
            int(value * size)
            for value, size in zip(cfg['roi'], (width, height, width, height))
        ]
        cv2.rectangle(out, (x0, y0), (x1, y1), (220, 180, 0), 1)
        if result.lamp_bbox:
            x, y, w, h = result.lamp_bbox
            cv2.rectangle(out, (x, y), (x+w, y+h), (255, 0, 255), 2)
        if result.bbox:
            x, y, box_width, box_height = result.bbox
            cv2.rectangle(
                out, (x, y), (x + box_width, y + box_height), (0, 255, 255), 2
            )
        label = 'LIGHT %s n=%d pos=%s' % (
            result.color,
            result.streak,
            '%.3f' % result.width_ratio if result.position_valid else 'unavailable',
        )
        cv2.putText(
            out, label, (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1
        )
        return out
