"""Image-only parking geometry probe. No camera, GPIO, motor, or network imports.

No metric coordinates: all lines are observed pixel candidates. `valid` means
all three pixel features passed local checks, NOT a safe motion authorization.
"""
from pathlib import Path
import argparse
import json
import math
import cv2
import numpy as np


def _hough(mask, threshold, length, gap):
    lines = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold,
                            minLineLength=length, maxLineGap=gap)
    return [] if lines is None else [a.astype(float) for a in lines[:, 0]]


def _line_dict(points, support, kind):
    points = np.asarray(points, np.float32)
    vx, vy, cx, cy = map(float, cv2.fitLine(points, cv2.DIST_HUBER, 0, .01, .01).ravel())
    direction = np.array([vx, vy]); center = np.array([cx, cy])
    project = (points - center) @ direction
    ends = np.array([center + direction * project.min(), center + direction * project.max()])
    return dict(kind=kind, endpoints=ends.round(2).tolist(), center=center.round(2).tolist(),
                direction=direction.tolist(), support_pixels=int(support),
                observed_length_px=float(project.max()-project.min()))


def _intersection(a, b):
    p=np.asarray(a['center']); v=np.asarray(a['direction'])
    q=np.asarray(b['center']); u=np.asarray(b['direction'])
    matrix=np.column_stack((v,-u))
    if abs(np.linalg.det(matrix))<1e-5: return None
    return (p+np.linalg.solve(matrix,q-p)[0]*v).tolist()


def _x_at_y(line, y):
    vx, vy=line['direction']; x0,y0=line['center']
    return None if abs(vy)<1e-5 else x0+(y-y0)*vx/vy


def _group_median_values(groups, values):
    """Return sorted unique groups and exact medians without a Python loop."""
    order = np.lexsort((values, groups))
    g, v = groups[order], values[order]
    unique, starts, counts = np.unique(g, return_index=True, return_counts=True)
    lower = starts + (counts - 1) // 2
    upper = starts + counts // 2
    return unique, (v[lower].astype(float) + v[upper]) * .5


def _extend_yellow_support(points, seed_points, seed_line, height, width):
    """Extend a fitted Hough seed using observed, connected collinear pixels.

    A Hough segment may cover only the right part of one yellow tape strip.
    Its endpoints are not tape endpoints. Search the full measured yellow mask
    near the fitted line, retaining the x-connected run that overlaps the seed.
    Empty image space is never used to create or extrapolate an endpoint.
    """
    supported = seed_points
    model = seed_line
    seed_left = float(seed_points[:, 0].min())
    seed_right = float(seed_points[:, 0].max())
    for _ in range(2):
        center = np.asarray(model['center'])
        direction = np.asarray(model['direction'])
        delta = points-center
        distance = np.abs(delta[:, 0]*direction[1]-delta[:, 1]*direction[0])
        nearby = points[distance <= max(2.5, height*.009)]
        if len(nearby) < 30:
            break
        columns = np.unique(nearby[:, 0])
        breaks = np.flatnonzero(np.diff(columns) > max(3, int(width*.025)))+1
        groups = np.split(columns, breaks)
        overlaps = [max(0., min(float(g[-1]), seed_right)
                        - max(float(g[0]), seed_left)) for g in groups]
        selected = groups[int(np.argmax(overlaps))]
        if max(overlaps) <= 0:
            break
        supported = nearby[(nearby[:, 0] >= selected[0])
                           & (nearby[:, 0] <= selected[-1])]
        group_x, median_y = _group_median_values(supported[:, 0], supported[:, 1])
        model = _line_dict(np.column_stack((group_x, median_y)), len(supported),
                           'yellow_transverse_candidate')
    return supported, model


def _best_white_fit(points, h, w, entry_y, split_x, entrance_left):
    """Exact original deterministic RANSAC scoring, vectorized in bounded chunks.

    Pair enumeration remains in original order; strict winner updates preserve
    the original first-winner tie behavior. No random or pose-specific samples.
    """
    i, j = np.triu_indices(len(points), 1)
    dy = points[j, 1] - points[i, 1]
    keep = np.abs(dy) >= h * .025
    i, j, dy = i[keep], j[keep], dy[keep]
    if len(i) == 0: return None
    slope = (points[j, 0] - points[i, 0]) / dy
    intercept = points[i, 0] - slope * points[i, 1]
    crossx = slope * entry_y + intercept
    keep = ((slope > -4.0) & (slope < -.2) & (crossx > -w*.02)
            & (crossx < split_x-w*.10) & (np.abs(crossx-entrance_left) <= w*.12))
    slope, intercept = slope[keep], intercept[keep]
    if len(slope) == 0: return None
    # candidate rows are generated in row order, so each row forms one group.
    row_starts = np.r_[0, np.flatnonzero(np.diff(points[:, 1]) != 0) + 1]
    best_score = -np.inf; best_points = None
    x, y = points[:, 0], points[:, 1]
    for start in range(0, len(slope), 512):
        residual = np.abs(x[None, :] - slope[start:start+512, None] * y[None, :]
                          - intercept[start:start+512, None])
        inliers = residual < max(2.5, w*.008)
        counts = np.count_nonzero(inliers, axis=1)
        row_counts = np.count_nonzero(np.logical_or.reduceat(inliers, row_starts, axis=1), axis=1)
        ymin = np.min(np.where(inliers, y[None, :], np.inf), axis=1)
        ymax = np.max(np.where(inliers, y[None, :], -np.inf), axis=1)
        mean_residual = np.sum(np.where(inliers, residual, 0.), axis=1) / np.maximum(1, counts)
        scores = row_counts + .12*(ymax-ymin) - .1*mean_residual
        scores[counts < 8] = -np.inf
        winner = int(np.argmax(scores))
        if scores[winner] > best_score:
            best_score = float(scores[winner]); best_points = points[inliers[winner]]
    return None if best_points is None else (best_score, best_points)


def detect_entry_geometry(frame, open_side='LEFT'):
    h,w=frame.shape[:2]
    result=dict(valid=False, purpose='pixel_observation_only', open_side=open_side,
                entrance=None, divider=None, left_boundary=None, centerline=None,
                corridor=None, reasons=[])
    if open_side!='LEFT':
        result['reasons'].append('Probe currently supports left-open geometry only')
        return result
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    yellow=cv2.inRange(hsv,np.array([12,28,125],np.uint8),np.array([55,255,255],np.uint8))
    yellow[:int(.50*h)]=0
    # Keep the entire lower image; do not end ROI before the real transverse tape.
    segments=_hough(yellow,max(10,int(w*.035)),max(16,int(w*.12)),max(3,int(w*.015)))
    transverse=[]
    for x1,y1,x2,y2 in segments:
        angle=abs(math.degrees(math.atan2(y2-y1,x2-x1)))
        if min(angle,180-angle)<=18 and (y1+y2)/2>=.58*h:
            transverse.append((math.hypot(x2-x1,y2-y1),np.array([x1,y1,x2,y2])))
    if not transverse:
        result['reasons'].append('No sufficiently long yellow transverse candidate')
        return result
    _,line=max(transverse,key=lambda x:x[0])
    a,b=line[:2],line[2:]; v=(b-a)/np.linalg.norm(b-a)
    ys,xs=np.where(yellow>0); points=np.column_stack((xs,ys))
    delta=points-a
    distance=np.abs(delta[:,0]*v[1]-delta[:,1]*v[0]); project=delta@v
    use=(distance<=max(2.5,h*.009))&(project>=-w*.025)&(project<=np.linalg.norm(b-a)+w*.025)
    ep=points[use]
    if len(ep)<30:
        result['reasons'].append('Insufficient yellow transverse support'); return result
    # Row-center fit limits thickness bias from tape edges.
    group_x, median_y = _group_median_values(ep[:,0], ep[:,1])
    rowpoints=np.column_stack((group_x,median_y))
    entrance=_line_dict(rowpoints,len(ep),'yellow_transverse_candidate')
    ep, entrance = _extend_yellow_support(points, ep, entrance, h, w)
    result['entrance']=entrance
    entry_y=entrance['center'][1]; ex=[int(ep[:,0].min()), int(ep[:,0].max())]
    # Separate the branch at the crossing. This accepts oblique longitudinal
    # tape, rather than requiring a near-vertical image orientation.
    # Only these rows can survive the existing vertical branch bounds.
    # ogrid avoids two full-frame index planes and all full-frame floats.
    branch_first=max(0,int(math.ceil(entry_y-h*.18)))
    branch_last=min(h,int(math.floor(entry_y))+1)
    if branch_last<=branch_first:
        result['reasons'].append('No yellow branch connected toward the entrance'); return result
    branch=yellow[branch_first:branch_last].copy()
    gy,gx=np.ogrid[branch_first:branch_last,:w]
    c=np.array(entrance['center']); d=np.array(entrance['direction'])
    signed=(gx-c[0])*d[1]-(gy-c[1])*d[0]
    branch[np.abs(signed)<=max(3,h*.012)]=0
    # LEFT-open view permits a measured divider at the right tape endpoint.
    # Mirror adaptation provides the corresponding RIGHT-open case.
    outside=((gx<ex[0]+.025*w)|(gx>ex[1]+.025*w)).ravel()
    branch[:,outside]=0
    # Height-one morphology cannot depend on rows outside this ROI.
    horizontal_residual = cv2.morphologyEx(
        branch,cv2.MORPH_OPEN,np.ones((1,max(21,int(w*.12))),np.uint8))
    branch=cv2.bitwise_and(branch,cv2.bitwise_not(horizontal_residual))
    n,labels,stats,_=cv2.connectedComponentsWithStats(branch,8)
    stats[:,1]+=branch_first
    branches=[]
    for i in range(1,n):
        x,y,bw,bh,area=stats[i]
        if area<max(12,h*w*.0002) or bh<max(8,h*.025) or bw>w*.20: continue
        if entry_y-(y+bh)>h*.055: continue
        yy,xx=np.where(labels[y-branch_first:y-branch_first+bh,x:x+bw]==i)
        yy,xx=yy+y,xx+x
        group_y, median_x = _group_median_values(yy,xx)
        rowpoints=np.column_stack((median_x,group_y))
        model=_line_dict(rowpoints,area,'yellow_divider_candidate')
        dx,dy=model['direction']
        if abs(math.degrees(math.atan2(dy,dx)))<20: continue
        cross=_intersection(entrance,model)
        if cross is None or not(ex[0]+w*.04<cross[0]<ex[1]+w*.02): continue
        # For the newly allowed T junction, require actual branch pixels
        # close to the intersection, not only an extrapolated fitted axis.
        if cross[0]>=ex[1]-w*.04:
            near=(np.abs(xx-cross[0])<=w*.025)&(np.abs(yy-cross[1])<=h*.055)
            if np.count_nonzero(near)<max(6,int(h*w*.00008)): continue
        branches.append((bh*math.sqrt(area),model))
    if not branches:
        result['reasons'].append('No yellow branch connected toward the entrance'); return result
    divider=max(branches,key=lambda item:item[0])[1]; result['divider']=divider
    far_y=min(p[1] for p in divider['endpoints'])
    # Neutral, bright paint/tape support; background is excluded using the
    # measured ground-tape extent, not fixed image corner coordinates.
    split_cross=_intersection(entrance,divider)
    first_row=max(0,int(far_y)); last_row=min(h,int(entry_y+h*.11))
    # Thresholding and this height-one close operate independently per row.
    # The previous full-frame top clearing never reached first_row.
    white=cv2.inRange(hsv[first_row:last_row],np.array([0,0,65],np.uint8),
                      np.array([179,65,255],np.uint8))
    white=cv2.morphologyEx(white,cv2.MORPH_CLOSE,np.ones((1,5),np.uint8))
    active=white>0
    transitions=np.diff(np.pad(active.astype(np.int8),((0,0),(1,1)), mode='constant'),axis=1)
    row_ids, run_starts = np.nonzero(transitions==1)
    end_rows, run_ends = np.nonzero(transitions==-1)
    # For each row, every run start has exactly one end in the same order.
    widths=run_ends-run_starts
    centers=(run_starts+run_ends-1)*.5
    rows=row_ids+first_row
    keep=((widths>=3)&(widths<=w*.11)&(run_starts!=0)
          &(np.abs(rows-entry_y)>=h*.014)&(centers<split_cross[0]-w*.13))
    points=np.column_stack((centers[keep],rows[keep]))
    if len(points)<8:
        result['reasons'].append('Insufficient separated left white tape rows'); return result
    if len(points)>128:
        result['reasons'].append('Excessive white tape candidates; conservative rejection'); return result
    best=_best_white_fit(points,h,w,entry_y,split_cross[0],ex[0])
    if best is None:
        result['reasons'].append('No consistent left white tape centerline'); return result
    boundary=_line_dict(best[1],len(best[1]),'white_left_boundary_candidate')
    result['left_boundary']=boundary
    left_cross=_intersection(entrance,boundary); middle_cross=_intersection(entrance,divider)
    if left_cross is None or middle_cross is None:
        result['reasons'].append('Parallel or singular candidate lines'); return result
    # Corridor limited to rows where divider was actually observed. Never
    # extrapolate the short divider to the bottom of the image or metric path.
    y_near=min(left_cross[1],middle_cross[1]); y_far=max(far_y,min(p[1] for p in boundary['endpoints']))
    if y_near-y_far<h*.025:
        result['reasons'].append('Visible shared corridor too short');return result
    lx0,lx1=_x_at_y(boundary,y_far),_x_at_y(boundary,y_near)
    rx0,rx1=_x_at_y(divider,y_far),_x_at_y(divider,y_near)
    coordinates = [lx0, lx1, rx0, rx1, y_far, y_near]
    if any(value is None for value in coordinates) or not np.isfinite(coordinates).all():
        result['reasons'].append('Non-finite or degenerate candidate corridor'); return result
    if not (0 <= y_far < y_near < h) or any(
            value < -.02*w or value > 1.02*w for value in (lx0,lx1,rx0,rx1)):
        result['reasons'].append('Candidate corridor lies outside the visible image'); return result
    if min(rx0-lx0,rx1-lx1)<w*.06:
        result['reasons'].append('Crossed or narrow candidate corridor');return result
    result['corridor']=[[lx0,y_far],[rx0,y_far],[rx1,y_near],[lx1,y_near]]
    result['centerline']=[[(lx0+rx0)/2,y_far],[(lx1+rx1)/2,y_near]]
    result['entrance_left_intersection']=left_cross
    result['entrance_divider_intersection']=middle_cross
    result['visible_depth_px']=y_near-y_far
    result['valid']=True
    result['reasons']=['Three color-supported pixel features found; no meter scale or vehicle clearance known',
                       'Short visible divider limits heading accuracy; far/end boundary not inferred']
    return result


def draw_geometry(frame,result,scale=3):
    out=frame.copy()
    if result.get('corridor'):
        overlay=out.copy(); cv2.fillPoly(overlay,[np.round(result['corridor']).astype(np.int32)],(50,170,50))
        out=cv2.addWeighted(overlay,.25,out,.75,0)
    for key,color in [('entrance',(0,255,255)),('divider',(0,140,255)),('left_boundary',(255,255,0))]:
        candidate=result.get(key)
        if candidate:
            p,q=[tuple(np.round(pt).astype(int)) for pt in candidate['endpoints']]
            cv2.line(out,p,q,color,1,cv2.LINE_AA)
            cv2.circle(out,p,2,color,-1);cv2.circle(out,q,2,color,-1)
    if result.get('centerline'):
        a,b=[tuple(np.round(pt).astype(int)) for pt in result['centerline']]
        cv2.line(out,a,b,(0,255,0),1,cv2.LINE_AA)
    out=cv2.resize(out,None,fx=scale,fy=scale,interpolation=cv2.INTER_NEAREST)
    labels=['OBSERVE ONLY - NO MOTION', 'Yellow: entrance | Orange: divider', 'Cyan: left outer | Green: image midpoint']
    for i,text in enumerate(labels):cv2.putText(out,text,(12,23+i*22),cv2.FONT_HERSHEY_SIMPLEX,.53,(0,255,255),1,cv2.LINE_AA)
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('images',nargs='+');p.add_argument('--out',default='tmp/geometry_probe/results');a=p.parse_args()
    out=Path(a.out);out.mkdir(parents=True,exist_ok=True);records=[]
    for path in a.images:
        frame=cv2.imread(path)
        if frame is None: continue
        r=detect_entry_geometry(frame);r['source']=path;records.append(r)
        cv2.imwrite(str(out/(Path(path).stem+'_geometry.png')),draw_geometry(frame,r))
    (out/'observations.json').write_text(json.dumps(records,indent=2),encoding='utf8')
    print(json.dumps(records,indent=2))
