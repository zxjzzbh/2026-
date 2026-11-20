"""Follow the selected bay's yellow divider and white outer tape; no hardware.

Only seed from the existing confirmed entrance geometry. Later observations
need fresh pixels on BOTH boundaries, but do not require an entrance crossbar.
Image geometry is not a measurement of vehicle clearance or distance.
"""
import math
import cv2
import numpy as np


def _model(line):
    dx, dy = line['direction']
    if abs(dy) < .15:
        return None
    x, y = line['center']
    return [float(dx / dy), float(x - dx / dy * y)]


class ParkingCorridorTracker:
    def __init__(self):
        self.side = None
        self.models = None
        self.baseline = None
        self.last_good_at = None
        self.hits = 0
        self.last_identity = None
        self.cached = None

    @staticmethod
    def _fit(mask, previous, first, last):
        """Fit thin row runs associated with the previously confirmed tape."""
        h, w = mask.shape
        active = mask[first:last] > 0
        edges = np.diff(np.pad(active.astype(np.int8), ((0, 0), (1, 1)), mode='constant'), axis=1)
        rows, starts = np.nonzero(edges == 1)
        _, ends = np.nonzero(edges == -1)
        centers = (starts + ends - 1) * .5
        ys = rows + first
        widths = ends - starts
        predicted = previous[0] * ys + previous[1]
        keep = ((widths >= 2) & (widths <= .10*w)
                & (np.abs(centers-predicted) <= .12*w))
        # One candidate per row, chosen by continuity rather than brightness.
        points = []
        for row in np.unique(ys[keep]):
            choices = np.flatnonzero(keep & (ys == row))
            i = choices[np.argmin(np.abs(centers[choices]-predicted[choices]))]
            points.append((centers[i], float(row)))
        if len(points) < 10:
            return None
        pts = np.asarray(points, np.float32)
        for _ in range(2):
            if len(pts) < 10 or np.ptp(pts[:, 1]) < .045*h:
                return None
            dx, dy, x, y = cv2.fitLine(pts, cv2.DIST_HUBER, 0, .01, .01).reshape(-1)
            if abs(dy) < .20:
                return None
            a, b = float(dx/dy), float(x-dx/dy*y)
            residual = np.abs(pts[:, 0] - (a*pts[:, 1]+b))
            pts = pts[residual <= max(2.5, .012*w)]
        if len(pts) < 10 or np.ptp(pts[:, 1]) < .045*h:
            return None
        return dict(model=[a, b], y0=float(pts[:, 1].min()),
                    y1=float(pts[:, 1].max()), rows=len(pts))

    def update(self, frame, fid, ts, side, geometry=None, baseline=None):
        identity = (fid, ts)
        if identity == self.last_identity:
            return dict(self.cached)
        result = dict(valid=False, side=side, reason='NO_CONFIRMED_CORRIDOR',
                      captured_at=ts, center_x=None, heading_error=None)
        if (not isinstance(ts, (int, float)) or not math.isfinite(ts)
                or (self.last_identity is not None and ts <= self.last_identity[1])):
            result['reason'] = 'CORRIDOR_FRAME_NOT_FRESH'
            return result
        self.last_identity = identity
        self.cached = result
        if frame is None or side not in ('LEFT', 'RIGHT'):
            return result
        if self.side is not None and side != self.side:
            result['reason'] = 'CORRIDOR_SIDE_CHANGED'
            return result
        geometry = geometry or {}
        expired = self.last_good_at is not None and ts-self.last_good_at > .20
        if self.models is None or expired:
            if not geometry.get('valid') or baseline is None:
                self.hits = 0
                return result
            yellow = _model(geometry['divider'])
            white = _model(geometry['outer_boundary'])
            if yellow is None or white is None:
                return result
            self.models = [yellow, white]
            self.side = side
            if self.baseline is None:
                self.baseline = float(baseline)
            self.hits = 0
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = [cv2.inRange(hsv, np.array([12, 28, 125], np.uint8),
                            np.array([55, 255, 255], np.uint8)),
                 cv2.inRange(hsv, np.array([0, 0, 65], np.uint8),
                            np.array([179, 65, 255], np.uint8))]
        fits = [self._fit(mask, model, int(.50*h), int(.94*h))
                for mask, model in zip(masks, self.models)]
        if any(fit is None for fit in fits):
            self.hits = 0
            result['reason'] = 'YELLOW_OR_WHITE_TAPE_MISSING'
            return result
        y0, y1 = max(f['y0'] for f in fits), min(f['y1'] for f in fits)
        if y1-y0 < .035*h:
            self.hits = 0
            result['reason'] = 'CORRIDOR_SHARED_SUPPORT_TOO_SHORT'
            return result
        yellow, white = [f['model'] for f in fits]
        left, right = (white, yellow) if side == 'LEFT' else (yellow, white)
        xs = [[m[0]*y+m[1] for m in (left, right)] for y in (y0, y1)]
        widths = [r-l for l, r in xs]
        denom = left[0]-right[0]
        if (min(widths) < .06*w or max(widths) > 1.05*w
                or widths[1] < .92*widths[0] or abs(denom) < .08
                or any(x < -.03*w or x > 1.03*w for pair in xs for x in pair)):
            self.hits = 0
            result['reason'] = 'CORRIDOR_PAIR_IMPLAUSIBLE'
            return result
        vp_y = (right[1]-left[1])/denom
        vp_x = left[0]*vp_y+left[1]
        heading = vp_x/w-self.baseline
        if not (.05*h < vp_y < y0-.02*h and -.10*w <= vp_x <= 1.10*w
                and abs(heading) <= .25):
            self.hits = 0
            result['reason'] = 'CORRIDOR_HEADING_IMPLAUSIBLE'
            return result
        self.models = [yellow, white]
        self.last_good_at = ts
        self.hits += 1
        result.update(valid=self.hits >= 3, center_x=sum(xs[1])/(2*w),
                      heading_error=heading, support_y=[y0, y1],
                      yellow=fits[0], white=fits[1], stable_frames=self.hits,
                      reason='CORRIDOR_TRACKED' if self.hits >= 3 else 'CORRIDOR_CONFIRMING')
        return result
