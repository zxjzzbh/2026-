"""Wisecar lane image detector, translated to Python/OpenCV for our controller.

Source: https://github.com/ytz7-xzg/wisecar
Commit: b85cb7ba3aad14d23da719adb515cf215d45ca6f
Files: get_binary_frame.cpp, get_lines.cpp (get_lines only), tasks.cpp (ROI).
No camera calibration from the other vehicle, ground mapping, GPIO or motor I/O.
On blue track, detect white paint beside the track and fit short connected
segments independently. The source detector remains for other track surfaces.
"""
import math
from bisect import bisect_left
import cv2
import numpy as np

VERSION = 'WISECAR_PLAYGROUND_V4_OBSERVED'
SOURCE_COMMIT = 'b85cb7ba3aad14d23da719adb515cf215d45ca6f'
WIDTH, HEIGHT = 320, 240


def _paint_runs(white):
    edges = np.diff(np.pad((white != 0).astype(np.int8), ((0, 0), (1, 1)), mode='constant'), axis=1)
    # Python integer lists avoid costly numpy-scalar comparisons inside bisect
    # on the Pi's 32-bit numpy build; the paint intervals are unchanged.
    return [(np.flatnonzero(row == 1).tolist(),
             (np.flatnonzero(row == -1)-1).tolist()) for row in edges]


def _nearest_run(starts, ends, predicted, scale):
    """Equivalent to min(distance,start,end), using sorted disjoint intervals."""
    index = bisect_left(ends, predicted)
    choices = []
    for i in (index-1, index):
        if not 0 <= i < len(starts):
            continue
        start, end = int(starts[i]), int(ends[i])
        distance = max(start-predicted, predicted-end, 0.) / scale
        if distance <= 3.5:
            choices.append((distance, start, end))
    return min(choices) if choices else None


def _trace_tape(white, a, b, y_min, y_max, runs=None, contiguous=False):
    """Measure thin paint runs, not an infinite line or the edge of a wide stripe."""
    points, widths = [], []
    scale = math.hypot(1., a)
    if runs is None:
        runs = _paint_runs(white)
    for y in range(max(0, int(y_min)), min(white.shape[0], int(y_max)+1)):
        starts, ends = runs[y]
        predicted = a*y+b
        choice = _nearest_run(starts, ends, predicted, scale)
        if choice is None:
            continue
        _, start, end = choice
        thickness = (end-start+1) / scale
        if not 1.0 <= thickness <= 12.0:
            continue
        if contiguous and thickness > 7.0:
            continue
        center = (start+end) / 2.
        if abs(center-predicted) / scale > 6:
            continue
        points.append((center, float(y)))
        if start > 0 and end < white.shape[1]-1:
            widths.append(thickness)
    if len(points) < 8 or len(widths) < 5:
        return None
    points = np.asarray(points)
    if contiguous:
        blocks = np.split(points, np.flatnonzero(np.diff(points[:, 1]) > 3) + 1)
        points = max(blocks, key=len)
        if len(points) < 12:
            return None
    aa, bb = map(float, np.linalg.lstsq(
        np.column_stack((points[:, 1], np.ones(len(points)))), points[:, 0], rcond=None)[0])
    if abs(aa) < 1e-9 or (aa < 0) != (a < 0):
        return None
    residual = np.abs(points[:, 0]-aa*points[:, 1]-bb) / math.hypot(1., aa)
    keep = residual <= 2.5
    if np.count_nonzero(keep) < .75 * len(points):
        return None
    points = points[keep]
    span = float(np.ptp(points[:, 1])) * math.hypot(1., aa)
    tape_width = float(np.median(widths))
    if span < max(22., 4.5 * tape_width):
        return None
    if len(points) < .55 * (np.ptp(points[:, 1])+1):
        return None
    return dict(m=1/aa, b=-bb/aa, length=span, paint_points=points.tolist(),
                tape_width=tape_width)


def _white_track_lines(roi, floor, white=None):
    """Fit actual white tape edges; never average unrelated same-side lines."""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    contiguous = white is not None
    if white is None:
        white = cv2.inRange(hsv, (0, 0, 100), (179, 55, 255))
    near_floor = cv2.dilate(floor, np.ones((11, 11), np.uint8))
    binary = cv2.bitwise_and(cv2.Canny(white, 50, 140), near_floor)
    # The faded edge of yellow paint can be low-saturation after resizing.
    # Remove a narrow band around yellow paint as well as its colored interior.
    yellow = cv2.inRange(hsv, (12, 35, 80), (50, 255, 255))
    yellow = cv2.dilate(yellow, np.ones((7, 7), np.uint8))
    binary = cv2.bitwise_and(binary, cv2.bitwise_not(yellow))
    paint = cv2.bitwise_and(white, cv2.bitwise_not(yellow))
    paint_runs = _paint_runs(paint)
    hough = cv2.HoughLinesP(binary, 1, np.pi / 180, 10,
                            minLineLength=18, maxLineGap=8)
    segments = [[], []]
    if hough is not None:
        for raw in hough[:, 0]:
            x1, y1, x2, y2 = map(float, raw)
            dx, dy = x2-x1, y2-y1
            if abs(dx) < 4 or abs(dy) < 4 or not .30 < abs(dy/dx) < 3:
                continue
            a = dx/dy
            segments[0 if a < 0 else 1].append(
                (a, x1-a*y1, math.hypot(dx, dy), (x1, y1, x2, y2)))
    groups = [[], []]
    for side in (0, 1):
        seen_inliers = set()
        for a, b, _, _ in segments[side]:
            inliers = []
            for item in segments[side]:
                aa, bb, length, (x1, y1, x2, y2) = item
                # Both endpoints must belong to this tape, including its two edges.
                distance = max(abs(x1-a*y1-b), abs(x2-a*y2-b)) / math.hypot(1, a)
                angle = abs(math.atan(aa)-math.atan(a))
                if distance <= 6 and angle <= math.radians(15):
                    inliers.append(item)
            # Identical ordered inliers yield exactly the same fit, score and tape.
            key = tuple(inliers)
            if key in seen_inliers:
                continue
            seen_inliers.add(key)
            points, weights = [], []
            for aa, bb, length, coords in inliers:
                points.extend([(coords[0], coords[1]), (coords[2], coords[3])])
                weights.extend([length, length])
            points = np.asarray(points)
            weights = np.asarray(weights)
            if len(points) < 2:
                continue
            design = np.column_stack((points[:, 1], np.ones(len(points))))
            fitted = np.linalg.lstsq(design * np.sqrt(weights[:, None]),
                                    points[:, 0] * np.sqrt(weights), rcond=None)[0]
            aa, bb = map(float, fitted)
            if abs(aa) < 1e-9 or (aa < 0) != (side == 0):
                continue
            span = float(np.ptp(points[:, 1])) * math.hypot(1, aa)
            if span < 22:
                continue
            # One long tape is stronger than many duplicate fragments on a short mark.
            # Continue only where actual thin paint is present; a short Hough seed
            # need not hide the adjacent visible tape. No inferred pixels are drawn.
            line = _trace_tape(paint, aa, bb, points[:, 1].min()-8,
                               points[:, 1].max()+8, paint_runs, contiguous)
            if line is None:
                continue
            line['length'] += .05 * sum(i[2] for i in inliers)
            line['segments'] = len(inliers)
            groups[side].append(line)
        groups[side].sort(key=lambda line: line['length'], reverse=True)
        # Limit candidates; suppress repeated Hough hypotheses for the same tape.
        unique = []
        for line in groups[side]:
            x0, x1 = -line['b']/line['m'], (roi.shape[0]-1-line['b'])/line['m']
            if any(max(abs(x0+other['b']/other['m']),
                       abs(x1-(roi.shape[0]-1-other['b'])/other['m'])) < 8
                   for other in unique):
                continue
            unique.append(line)
            if len(unique) == 4:
                break
        groups[side] = unique
    lines = [side[0] if side else None for side in groups]
    mode = sum(1 << side for side in (0, 1) if lines[side] is not None)
    return dict(lines=lines, mode=mode, binary=binary, line_groups=groups)


def detect(frame, roi_top=.35, roi_bottom=.65):
    """Return guarded lines y=m*x+b in normalized ROI coordinates."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError('Expected a BGR camera frame')
    if not (math.isfinite(roi_top) and math.isfinite(roi_bottom)
            and 0 <= roi_top < roi_bottom <= 1):
        raise ValueError('Invalid lane ROI')
    normalized = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
    y0 = int(HEIGHT * roi_top)
    roi_height = int(HEIGHT * (roi_bottom - roi_top))
    if roi_height < 2:
        raise ValueError('Lane ROI must span at least two normalized pixels')
    roi = normalized[y0:y0 + roi_height]
    # Fit near-field evidence below the steering reference region too.
    # A line reaching the image edge must not disappear because ROI ends early.
    fit_roi = normalized[y0:min(HEIGHT, y0 + roi_height + int(.08 * HEIGHT))]
    fit_hsv = cv2.cvtColor(fit_roi, cv2.COLOR_BGR2HSV)
    fit_floor = cv2.inRange(fit_hsv, (80, 85, 35), (140, 255, 255))
    if cv2.countNonZero(fit_floor) >= .08 * fit_floor.size:
        result = _white_track_lines(fit_roi, fit_floor)
        # Keep binary/debug coordinates identical to the requested controller ROI.
        result['binary'] = result['binary'][:roi_height]
        result['roi_y0'] = y0
        return result
    # Athletics-track paint is tinted by the red surface and can be dark in
    # shadow. Keep the senior's edge/Hough approach, but fit each separate tape
    # hypothesis to observed paint instead of averaging all same-side lines.
    red_floor = cv2.bitwise_or(
        cv2.inRange(fit_hsv, (0, 75, 25), (15, 255, 255)),
        cv2.inRange(fit_hsv, (170, 75, 25), (179, 255, 255)))
    if cv2.countNonZero(red_floor) >= .20 * red_floor.size:
        gray = cv2.cvtColor(fit_roi, cv2.COLOR_BGR2GRAY)
        background = cv2.morphologyEx(gray, cv2.MORPH_OPEN,
                                     np.ones((3, 21), np.uint8))
        contrast = gray.astype(np.float32) - background
        saturation = fit_hsv[:, :, 1]
        red_neighbours = cv2.dilate(saturation, np.ones((1, 21), np.uint8))
        paint = np.uint8((contrast >= np.maximum(6., gray * .075))
                         & (red_neighbours.astype(np.int16) - saturation >= 22)
                         & (saturation < 200)
                         & (fit_hsv[:, :, 2] >= 25)) * 255
        result = _white_track_lines(fit_roi, red_floor, paint)
        result['surface'] = 'red_track'
        result['binary'] = result['binary'][:roi_height]
        result['roi_y0'] = y0
        return result
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), .52)
    binary = cv2.Canny(gray, 50, 140)
    binary = cv2.dilate(binary, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    hough = cv2.HoughLinesP(binary, 1, np.pi / 180, 15,
                            minLineLength=55, maxLineGap=0)
    vectors = [np.zeros(2), np.zeros(2)]
    intercepts, lengths, counts = [0., 0.], [0., 0.], [0, 0]
    if hough is not None:
        for raw in hough[:, 0]:
            x1, y1, x2, y2 = map(float, raw)
            dx, dy = x1-x2, y1-y2
            if abs(dx) < 3 or abs(dy) < 3:
                continue
            k = dy / dx
            if abs(k) <= .25:
                continue
            vec = np.array([dx, dy])
            squared = float(np.dot(vec, vec))
            length = math.sqrt(squared)
            side = 0 if k < 0 else 1
            vectors[side] += vec / squared if side == 0 else vec
            intercepts[side] += (y1-k*x1) * length
            lengths[side] += length
            counts[side] += 1
    lines, mode = [None, None], 0
    for side in (0, 1):
        if lengths[side] <= 1 or abs(vectors[side][0]) < 1e-12:
            continue
        m = float(vectors[side][1] / vectors[side][0])
        b = intercepts[side] / lengths[side]
        # Source lacks numerical protection; invalid geometry never enters control.
        if not (math.isfinite(m) and math.isfinite(b)) or abs(m) < 1e-9:
            continue
        lines[side] = dict(m=m, b=b, length=lengths[side], segments=counts[side])
        mode |= 1 << side
    # Even the non-blue fallback must reject broad stripes and report real support.
    fallback_white = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV),
                                (0, 0, 140), (179, 110, 255))
    for side, line in enumerate(lines):
        if line is None:
            continue
        traced = _trace_tape(fallback_white, 1/line['m'], -line['b']/line['m'],
                             0, roi.shape[0]-1)
        if traced is None:
            lines[side] = None
            mode &= ~(1 << side)
        else:
            line['paint_points'] = traced['paint_points']
            line['tape_width'] = traced['tape_width']
    return dict(lines=lines, mode=mode, binary=binary, roi_y0=y0)


def candidates(frame, roi_top, roi_bottom, ref_ratio):
    """Map source detections back to our frame/ROI, preserving controller units."""
    height, width = frame.shape[:2]
    y0 = max(0, min(height-2, int(height*roi_top)))
    y1 = max(y0+2, min(height, int(height*roi_bottom)))
    roi_height = y1-y0
    reference_y, bottom_y = ref_ratio*(roi_height-1), float(roi_height-1)
    detected = detect(frame, roi_top, roi_bottom)
    sides = [[], []]
    scale_x, scale_y = width / float(WIDTH), HEIGHT / float(height)
    groups = detected.get('line_groups', [[line] if line else [] for line in detected['lines']])
    for side, lines in enumerate(groups):
        for line in lines:
            a = scale_x * scale_y / line['m']
            b = scale_x * (y0*scale_y-detected['roi_y0']-line['b']) / line['m']
            observed = [(point[0]*scale_x, (point[1]+detected['roi_y0'])/scale_y)
                        for point in line.get('paint_points', [])]
            if detected.get('surface') == 'red_track':
                # A short distant marking must not become a near lane boundary
                # by extending its fit to the steering measurement row.
                measurement_y = y0 + reference_y
                tolerance = .03 * height
                if (not observed or min(p[1] for p in observed) > measurement_y+tolerance
                        or max(p[1] for p in observed) < measurement_y-tolerance):
                    continue
            sides[side].append(dict(a=a, b=b, x_ref=a*reference_y+b,
                                    x_bottom=a*bottom_y+b,
                                    length=line['length']*scale_x,
                                    angle=math.degrees(math.atan2(1., abs(a))),
                                    segments=line['segments'], observed_points=observed,
                                    tape_width=line.get('tape_width'),
                                    surface=detected.get('surface')))
    binary = cv2.resize(detected['binary'], (width, roi_height), interpolation=cv2.INTER_NEAREST)
    return binary, sides[0], sides[1], reference_y, bottom_y
