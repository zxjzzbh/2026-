"""Fast, experimental race candidates from color and shape, without weights.

Scores here are heuristics, not calibrated probabilities. Track colors,
shadows and viewing distance can confuse these cues. They support field data
collection and must not alone become board-removal or free-space evidence.
"""

import cv2
import numpy as np

from .results import Detection
from .cone_shapes import cone_candidates
from .traffic_lamps import horizontal_lamp_groups
from .crosswalk import CrosswalkDetector


class ClassicRaceDetector:
    status = 'experimental_color_shape'

    def detect(self, image):
        h, w = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masks = {
            'blue': cv2.inRange(hsv, (90, 90, 50), (135, 255, 255)),
            'red': cv2.bitwise_or(cv2.inRange(hsv, (0, 90, 50), (10, 255, 255)),
                                 cv2.inRange(hsv, (170, 90, 50), (179, 255, 255))),
            'yellow': cv2.inRange(hsv, (15, 90, 50), (38, 255, 255)),
            'green': cv2.inRange(hsv, (40, 90, 50), (90, 255, 255)),
        }
        detections = []
        for color in ('blue', 'red'):
            mask = cv2.morphologyEx(masks[color], cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                x, y, bw, bh = cv2.boundingRect(contour)
                if bw < 6 or bh < 6:
                    continue
                fraction = area / (w * h)
                fill = area / (bw * bh)
                aspect = bh / bw
                vertices = len(cv2.approxPolyDP(contour, .04 * cv2.arcLength(contour, True), True))
                clipped = x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1
                normal_board = (not clipped and fraction >= .004 and bh / h >= .18
                                and .5 <= aspect <= 2.5 and fill >= .75 and 4 <= vertices <= 6)
                # A nearby panel can extend beyond the field of view. A large,
                # connected, solid blue region is still an occluding-board cue;
                # unseen edges must not turn it into evidence of board removal.
                # Color alone remains ambiguous (e.g. blue floor/wall), so this
                # is experimental observation evidence, never launch authority.
                near_board = (clipped and fraction >= .25 and bw / w >= .55
                              and bh / h >= .35 and fill >= .85 and 4 <= vertices <= 6)
                if color == 'blue' and (normal_board or near_board):
                    detections.append(Detection('blue_board', .55 if clipped else .65, [x, y, x + bw, y + bh]))

        detections.extend(cone_candidates(hsv))

        # Preserve three-lens structure even when the lit lens has a white core.
        for group in horizontal_lamp_groups(hsv):
            detections.append(Detection('traffic_light', .65, group['bbox']))

        crosswalk, _ = CrosswalkDetector().detect(image)
        if crosswalk['candidate']:
            detections.append(Detection('crosswalk', crosswalk['score'], crosswalk['bbox_xyxy']))
        # Remove duplicate lamps while keeping ambiguity between real candidates.
        unique = {(d.label, tuple(d.bbox_xyxy)): d for d in detections}
        return list(unique.values())
