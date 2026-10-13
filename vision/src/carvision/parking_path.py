"""Rear-axle parking paths and arc-length Pure Pursuit (no hardware I/O).

Axes are x right, y forward; positive yaw/curvature turn right. All distances
are metres. Sampling is accompanied by an explicit body-clearance allowance
in the caller; it is not a proof about unobserved obstacles.
"""
import math

import numpy as np


def entry_curve(pose, align_y, goal_x, goal_y, step=.015):
    """Quintic lateral transition with the *actual* initial heading.

    The final straight segment begins before the front enters the bay. An
    already aligned vehicle can enter directly without first moving sideways.
    """
    x0, y0 = pose['x_m'], pose['y_m']
    yaw = pose['yaw_rad']
    if abs(x0-goal_x) < 1e-6 and abs(yaw) < 1e-6:
        align_y = y0
        rows = []
    else:
        span = align_y-y0
        if span <= .1:
            raise ValueError('接近距离不足以在入口前完成横移，请更早识别停车区')
        delta = goal_x-x0
        a1 = math.tan(yaw)*span
        a3, a4, a5 = 10*delta-6*a1, -15*delta+8*a1, 6*delta-3*a1
        rows = []
        for u in np.linspace(0, 1, max(3, math.ceil(span/step)+1))[:-1]:
            x = x0+a1*u+a3*u**3+a4*u**4+a5*u**5
            slope = (a1+3*a3*u*u+4*a4*u**3+5*a5*u**4)/span
            second = (6*a3*u+12*a4*u*u+20*a5*u**3)/(span*span)
            rows.append({'x_m': float(x), 'y_m': float(y0+span*u),
                         'yaw_rad': math.atan(slope),
                         'curvature': float(second/(1+slope*slope)**1.5)})
    if goal_y <= align_y:
        raise ValueError('停车目标已在车后或无有效前进距离')
    for y in np.linspace(align_y, goal_y, max(2, math.ceil((goal_y-align_y)/step)+1)):
        rows.append({'x_m': goal_x, 'y_m': float(y), 'yaw_rad': 0., 'curvature': 0.})
    return rows


def pursuit(path, pose, progress_m, lookahead_m):
    """Project onto a polyline, then look ahead by travelled path length.

    Progress cannot move backwards. Interpolation avoids vertex quantisation;
    unlike a world-y lookup, the distance is correct along lateral approaches.
    """
    lengths = [0.]
    for a, b in zip(path, path[1:]):
        lengths.append(lengths[-1]+math.hypot(b['x_m']-a['x_m'], b['y_m']-a['y_m']))
    best = None
    for i, (a, b) in enumerate(zip(path, path[1:])):
        length = lengths[i+1]-lengths[i]
        if length <= 0 or lengths[i+1] < progress_m:
            continue
        dx, dy = b['x_m']-a['x_m'], b['y_m']-a['y_m']
        fraction = ((pose['x_m']-a['x_m'])*dx+(pose['y_m']-a['y_m'])*dy)/length**2
        fraction = max(0., (progress_m-lengths[i])/length, min(1., fraction))
        fraction = min(1., fraction)
        px, py = a['x_m']+fraction*dx, a['y_m']+fraction*dy
        candidate = (math.hypot(pose['x_m']-px, pose['y_m']-py), lengths[i]+fraction*length)
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        best = (math.hypot(pose['x_m']-path[-1]['x_m'], pose['y_m']-path[-1]['y_m']), lengths[-1])
    error, progress = best
    target_s = min(lengths[-1], progress+lookahead_m)
    target = (path[-1]['x_m'], path[-1]['y_m'])
    for i in range(len(path)-1):
        if lengths[i] <= target_s <= lengths[i+1] and lengths[i+1] > lengths[i]:
            f = (target_s-lengths[i])/(lengths[i+1]-lengths[i])
            target = tuple(path[i][k]+f*(path[i+1][k]-path[i][k]) for k in ('x_m', 'y_m'))
            break
    dx, dy = target[0]-pose['x_m'], target[1]-pose['y_m']
    co, si = math.cos(pose['yaw_rad']), math.sin(pose['yaw_rad'])
    lateral, forward = co*dx-si*dy, si*dx+co*dy
    curvature = 2*lateral/max(1e-9, dx*dx+dy*dy)
    return {'progress_m': progress, 'remaining_m': lengths[-1]-progress,
            'tracking_error_m': error, 'target_x_m': target[0], 'target_y_m': target[1],
            'target_forward_m': forward, 'curvature_1pm': curvature}


def arc_poses(pose, curvature, distance):
    """Actual constant-curvature command through the stopping horizon."""
    yaw = pose['yaw_rad']
    for s in np.linspace(0., distance, max(2, math.ceil(distance/.01)+1)):
        end = yaw+curvature*s
        if abs(curvature) < 1e-9:
            dx, dy = s*math.sin(yaw), s*math.cos(yaw)
        else:
            dx = (math.cos(yaw)-math.cos(end))/curvature
            dy = (math.sin(end)-math.sin(yaw))/curvature
        yield {'x_m': pose['x_m']+dx, 'y_m': pose['y_m']+dy, 'yaw_rad': end}
