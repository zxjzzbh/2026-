"""Acquire a bay from observed floor and tape, without board-height cropping.

Background is filtered by masks and geometry. Every accepted corner belongs
to a current closed floor component; neither an ROI nor an image border can
supply a missing edge. This module contains no motion or temporal state.
"""
import cv2
import numpy as np
from .parking_inner_edges import inner_quad


def _quad(points):
    points = np.asarray(points, np.float32)
    ordered = points[np.argsort(points[:, 1])]
    far = ordered[:2][np.argsort(ordered[:2, 0])]
    near = ordered[2:][np.argsort(ordered[2:, 0])]
    return np.array([near[0], near[1], far[1], far[0]], np.float32)


def _support(expanded, a, b):
    points = np.rint(np.linspace(a, b, max(12, int(np.linalg.norm(a-b))))).astype(int)
    h, w = expanded.shape
    if ((points < 0).any() or (points[:, 0] >= w).any() or (points[:, 1] >= h).any()):
        return 0.
    return float(np.mean(expanded[points[:, 1], points[:, 0]] > 0))


def _beside_board(q, board, side, image_shape):
    """Associate after detection; board position never clips visible tape."""
    h, w = image_shape
    x1, y1, x2, y2 = board
    bw, bh = x2-x1, y2-y1
    span = float(q[1, 0]-q[0, 0])
    far_span = float(q[2, 0]-q[3, 0])
    depths = q[:2, 1]-q[[3, 2], 1]
    if (span < w*.08 or far_span < w*.08 or depths.min() < 8
            or not .15 <= far_span/span <= 1.3 or depths.max() > 2.5*depths.min()
            or not .35*span <= bw <= 1.7*span):
        return False
    board_x = (x1+x2)/2
    if (side == 'left' and q[:, 0].mean() >= board_x
            or side == 'right' and q[:, 0].mean() <= board_x):
        return False
    entrance_y = q[0, 1]+(board_x-q[0, 0])*(q[1, 1]-q[0, 1])/span
    tolerance = max(10, h*.07, min(h*.12, bh*.25))
    if abs(entrance_y-y2) > tolerance:
        return False
    divider = q[[1, 2]] if side == 'left' else q[[0, 3]]
    divider_x = divider[0, 0]+(y2-divider[0, 1])*(divider[1, 0]-divider[0, 0])/(divider[1, 1]-divider[0, 1])
    gap = x1-divider_x if side == 'left' else divider_x-x2
    return -.04*w <= gap <= .12*w


def board_adjacent_entry(edges, ground, blue, hint, *, yellow=None, diagnostics=None):
    """Keep contrast-supported fits; retry color barriers if no enclosure exists."""
    stats={}
    result=_entry_pass(edges,ground,blue,hint,yellow=yellow,partition=edges,diagnostics=stats)
    retry=(not result and not stats.get('ambiguous') and yellow is not None
           and stats.get('reason')!='no_supported_board_direction' and np.any((yellow>0)&(edges==0)))
    if retry:
        previous_reason=stats.get('reason')
        result=_entry_pass(edges,ground,blue,hint,yellow=yellow,
                           partition=cv2.bitwise_or(edges,yellow),diagnostics=stats)
        stats['contrast_partition_reason']=previous_reason
    stats['yellow_partition_retry']=bool(retry)
    if diagnostics is not None:diagnostics.update(stats)
    return result


def _entry_pass(edges, ground, blue, hint, *, yellow, partition, diagnostics):
    """Find a small free enclosure adjacent to a board, then validate its edges."""
    stats = diagnostics if diagnostics is not None else {}
    stats.update(method='ground_components_then_board_association', cropped_to_board=False,
                 floor_components=0, quadrilaterals=0, associated=0,
                 rejected_border=0, rejected_edges=0,
                 rejected_interior=0, refitted_boundaries=0, accepted=0)
    side, blocked = hint.get('suggested_slot'), hint.get('blocked_side')
    boards = [b for b in hint.get('boards', []) if b.get('side') == blocked]
    if side not in ('left', 'right') or not boards:
        stats['reason']='no_supported_board_direction'
        return []
    board = max(boards, key=lambda b: b['area_fraction'])['bbox_px']
    h, w = edges.shape
    # Keep background/obstruction pixels out of free space. Do not fill holes:
    # unknown dark patches and blue objects must survive the interior check.
    markings = edges if yellow is None else cv2.bitwise_or(edges,yellow)
    barriers = cv2.dilate(cv2.bitwise_or(partition, blue), np.ones((3, 3), np.uint8))
    free = ground.copy()
    free[barriers > 0] = 0
    # A tape enclosure creates a white island inside a hole in the larger
    # floor component. EXTERNAL drops that island; CCOMP exposes each white
    # component while allowing us to exclude the black-hole contours.
    contours,hierarchy = cv2.findContours(free, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    expanded = cv2.dilate(markings, np.ones((9, 9), np.uint8))
    yellow_support = None if yellow is None else cv2.dilate(yellow, np.ones((9, 9), np.uint8))
    result = []
    for index,contour in enumerate(contours):
        if hierarchy[0,index,3]>=0:continue
        if not .003*w*h < cv2.contourArea(contour) < .25*w*h:
            continue
        stats['floor_components'] += 1
        q = cv2.approxPolyDP(contour, .03*cv2.arcLength(contour, True), True)
        if len(q) != 4:
            q = cv2.approxPolyDP(contour, .04*cv2.arcLength(contour, True), True)
        if len(q) != 4 or not cv2.isContourConvex(q):
            continue
        q = _quad(q[:, 0, :])
        stats['quadrilaterals'] += 1
        if (q.min() <= 2 or q[:, 0].max() >= w-3 or q[:, 1].max() >= h-3):
            stats['rejected_border'] += 1
            continue
        if not _beside_board(q, board, side, edges.shape):
            continue
        stats['associated'] += 1
        # Contour chords only propose the enclosure. Refit the four floor-side
        # tape edges before evaluating support, especially the short far edge.
        fitted=inner_quad(markings,ground,q,acquiring=True)
        if fitted is not None:
            if (not cv2.isContourConvex(fitted) or fitted.min()<=2
                    or fitted[:,0].max()>=w-3 or fitted[:,1].max()>=h-3
                    or not _beside_board(fitted,board,side,edges.shape)):
                continue
            q=fitted;stats['refitted_boundaries']+=1
        supports = [_support(expanded, q[i], q[(i+1) % 4]) for i in range(4)]
        if min(supports) < .7:
            stats['rejected_edges'] += 1
            continue
        color = None if yellow_support is None else [
            _support(yellow_support, q[0], q[1]), _support(yellow_support, q[2], q[3])]
        # Washed tape can lose its yellow hue entirely. Color is supporting
        # evidence, not a new mandatory HSV threshold. Four real boundaries,
        # visible ground interior and board association remain mandatory.
        mask = np.zeros_like(edges)
        cv2.fillConvexPoly(mask, q.astype(np.int32), 255)
        inside = cv2.erode(mask, np.ones((3, 3), np.uint8)) > 0
        if inside.sum() < 180:
            continue
        floor = float(np.mean(ground[inside] > 0))
        occupied = float(np.mean(blue[inside] > 0))
        if floor < .9 or occupied > .005:
            stats['rejected_interior'] += 1
            continue
        result.append({'id': side, 'corners_px': q.tolist(), 'availability': 'clear',
                       'candidate_only': True, 'occupancy_verified': False, 'polygon_m': None,
                       'ground_fraction': floor, 'blue_inside_fraction': occupied,
                       'edge_support': supports, 'yellow_end_support': color,
                       'tape_evidence': 'yellow_and_contrast' if color and max(color) >= .6 else 'pale_contrast',
                       'geometry_source': 'board_adjacent_contrast',
                       'search_method': stats['method']})
    stats['accepted'] = len(result) if len(result) == 1 else 0
    stats['ambiguous'] = len(result) > 1
    stats['reason']=('accepted_current_enclosure' if len(result)==1 else 'ambiguous_enclosures' if len(result)>1
                     else 'interior_not_confirmed' if stats['rejected_interior'] else 'missing_tape_edges' if stats['rejected_edges']
                     else 'no_board_associated_enclosure' if stats['quadrilaterals'] else 'no_closed_ground_enclosure')
    return result if len(result) == 1 else []
