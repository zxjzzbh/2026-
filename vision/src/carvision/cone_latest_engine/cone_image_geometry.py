"""Separate image lane position from road direction. Author: 13246.

The vanishing point of two straight, parallel course edges depends on their
direction, whereas their near-row position also changes with lateral offset.
These are pixel/fraction observations, not a measured vehicle pose or yaw.
This helper keeps no camera history and cannot supply travel/pass evidence.
"""
import json
import math
from pathlib import Path

import numpy as np

from .brake_parking import numeric
from .cone_timing import frame_fresh


def _reference(settings,camera,primary_ground):
    if camera=='secondary':return np.asarray(settings['ground']['homography'],float)
    if primary_ground is None:
        path=Path(__file__).parents[3]/'configs/cone-primary-ground-reference.json'
        primary_ground=json.loads(path.read_text(encoding='utf-8-sig'))
    if primary_ground.get('image_size')!=[640,480] or primary_ground.get('historical_reference_only') is not True:
        raise ValueError('Unexpected primary reference')
    resize=np.array([[4/3,0,1/6],[0,4/3,1/6],[0,0,1.]])
    shift=np.eye(3)
    shift[1,2]=settings['geometry']['rear_axle_to_front_m']-primary_ground['frontmost_to_rear_axle_m']
    return shift@np.asarray(primary_ground['homography'],float)@resize


def _body_fraction(bounds,rows,H):
    """Dimensionless origin estimate only when projected edges agree.

    The nearest visible row is ahead of the bumper and mixes position/yaw.
    Parallel projected edges can separate that lookahead from lateral position;
    disagreeing ground geometry must keep the direct image cue instead.
    """
    slopes=[];intercepts=[]
    for col in (0,1):
        p=np.column_stack((bounds[:,col],rows,np.ones(2)))@H.T
        if np.any(np.abs(p[:,2])<1e-8) or p[0,2]*p[1,2]<=0:return None
        p=p[:,:2]/p[:,2,None]
        if not np.isfinite(p).all() or np.any(p[:,1]<=0) or np.any(p[:,1]>20):return None
        dy=p[1,1]-p[0,1]
        if abs(dy)<.1:return None
        a=float((p[1,0]-p[0,0])/dy);slopes.append(a);intercepts.append(float(p[0,0]-a*p[0,1]))
    width=intercepts[1]-intercepts[0]
    if abs(slopes[1]-slopes[0])>.12 or not .5<width<2.5:return None
    q=-intercepts[0]/width
    return (float(q),float(sum(slopes)/2),float(width)) if -.5<q<1.5 else None


def _single_view_geometry(nav,settings,*,now=None,primary_ground=None):
    """Return coherent single-camera image cues, or None for unusable lines.

    `heading_correction_fraction > 0` means the observed road direction is to
    the RIGHT of the historical forward image line, so correcting road heading
    requires a rightward steering contribution. It is not a heading in radians.

    A held lane keeps its original observation timestamp and fresh_lane=False.
    The current image timestamp must not turn cached lines into new feedback.
    """
    camera=nav.get('source_camera');captured=nav.get('captured_s')
    if camera not in ('primary','secondary') or not numeric(captured):return None
    if now is not None and not frame_fresh(captured,now):return None
    if nav.get('line_missing') or nav.get('guard')=='image_only_boundary_unknown':return None
    held=bool(nav.get('line_held',False));observed=nav.get('line_observed_s')
    if observed is None and not held:observed=captured
    if not numeric(observed) or observed>captured+1e-6:return None
    reported_age=nav.get('line_age_s',0.)
    if not numeric(reported_age) or reported_age<0:return None
    age=max(reported_age,(now if now is not None else captured)-observed)
    single=bool(nav.get('single_edge_observed'))
    if not 0<=age<=(1. if single else .8)+1e-6:return None
    if not held and abs(captured-observed)>1e-6:return None
    try:
        bounds=np.asarray(nav.get('lane_bounds_px'),float)
        rows=np.asarray(nav.get('lane_rows'),float)
        width,height=map(float,settings['image_size'])
        H=_reference(settings,camera,primary_ground)
    except (KeyError,TypeError,ValueError,OSError):return None
    if (bounds.shape!=(2,2) or rows.shape!=(2,) or H.shape!=(3,3)
            or not all(np.isfinite(v).all() for v in (bounds,rows,H))
            or width<=0 or height<=0 or rows[1]-rows[0]<12 or abs(H[0,0])<1e-8):return None
    widths=bounds[:,1]-bounds[:,0]
    if not np.all((widths>.15*width)&(widths<1.8*width)):return None
    expansion=widths[1]-widths[0]
    # Almost-parallel image edges have an ill-conditioned intersection. Do not
    # turn a few noisy pixels of width change into an enormous heading error.
    if expansion<max(6.,.03*max(widths)):return None
    slope=(bounds[1]-bounds[0])/(rows[1]-rows[0]);intercept=bounds[0]-slope*rows[0]
    vanishing_y=(intercept[1]-intercept[0])/(slope[0]-slope[1])
    vanishing_x=slope[0]*vanishing_y+intercept[0]
    if not (-2*height<=vanishing_y<=rows[0]-3 and -2*width<=vanishing_x<=3*width):return None
    def forward_x(y):return float(-(H[0,1]*y+H[0,2])/H[0,0])
    reference=[forward_x(y) for y in rows]
    fractions=[float((x-left)/w) for x,left,w in zip(reference,bounds[:,0],widths)]
    correction=float((vanishing_x-forward_x(vanishing_y))/width)
    if not all(math.isfinite(x) for x in (*fractions,correction)):return None
    origin=_body_fraction(bounds,rows,H)
    return {'camera':camera,'captured_s':captured,'lane_observed_s':observed,
            'line_held':held,'line_age_s':age,'fresh_lane':not held,
            'single_edge_observed':single,
            'near_ego_fraction':fractions[1],'far_ego_fraction':fractions[0],
            'body_ego_fraction':origin[0] if origin else None,
            'ground_lane_slope':origin[1] if origin else None,
            'ground_lane_width_estimate_m':origin[2] if origin else None,
            'near_far_ego_difference':fractions[1]-fractions[0],
            'heading_correction_fraction':correction,
            'vanishing_point_px':[float(vanishing_x),float(vanishing_y)],
            'heading_reference_x_px':forward_x(vanishing_y),
            'lane_rows_px':rows.tolist(),'lane_widths_px':widths.tolist(),
            'reference_center_x_px':reference,'metric_verified':False,
            'heading_source':'straight_edge_vanishing_point_vs_historical_forward_line'}


def _raw_lane_candidate(row,settings,reference_time):
    """Fit each camera's two edges independently; never join camera pixels."""
    camera=row.get('camera');captured=row.get('captured_s')
    if camera not in ('primary','secondary') or not numeric(captured):return None
    if not frame_fresh(captured,reference_time):return None
    try:
        edges=[np.asarray(row.get(side),float) for side in ('left','right')]
        height=float(settings['image_size'][1])
    except (KeyError,TypeError,ValueError):return None
    if any(v.ndim!=2 or v.shape[1]!=2 or len(v)<6 or not np.isfinite(v).all()
           or len(np.unique(v[:,1]))<6 for v in edges):return None
    start=max(v[:,1].min() for v in edges);end=min(v[:,1].max() for v in edges)
    if end-start<height*.09:return None
    fits=[np.polyfit(v[:,1],v[:,0],1) for v in edges]
    residual=max(float(np.max(np.abs(np.polyval(f,v[:,1])-v[:,0]))) for f,v in zip(fits,edges))
    if residual>8:return None
    return {'source_camera':camera,'captured_s':captured,'line_observed_s':captured,
            'line_age_s':reference_time-captured,'line_held':False,
            'lane_rows':[float(start),float(end)],
            'lane_bounds_px':[[float(np.polyval(f,y)) for f in fits] for y in (start,end)],
            'raw_fit_residual_px':residual,'raw_fit_points':min(map(len,edges))}


def image_lane_geometry(nav,settings,*,now=None,primary_ground=None):
    """Choose a coherent image-lane cue and mark its relationship to cone goals.

    A valid current selected view is preferred. Otherwise use a fresh point-set
    from local_course.image_lane: selected camera first, then secondary, then
    primary. Held selected-view lines are only a final fallback. Raw-view lane
    fractions are NOT compatible with a cone's selected-view/image-only goal.
    The caller may use their independent heading/boundary direction but must
    not subtract them from that goal.
    """
    captured=nav.get('captured_s')
    if not numeric(captured):return None
    if now is not None and not frame_fresh(captured,now):return None
    course=nav.get('local_course') or {}
    course_time=course.get('captured_s')
    reference_time=now if now is not None else max(captured,course_time if numeric(course_time) else captured)
    direct=_single_view_geometry(nav,settings,now=reference_time,primary_ground=primary_ground)
    if direct is not None:
        direct.update(coordinate_domain='lane_fraction',geometry_source='selected_navigation_lane',
                      nav_goal_compatible=True,same_camera_as_navigation=True)
        if not direct['line_held']:return direct
    source=nav.get('source_camera')
    cameras=list(dict.fromkeys([source,'secondary','primary']))
    raw=course.get('image_lane') or []
    for camera in cameras:
        rows=sorted((r for r in raw if isinstance(r,dict) and r.get('camera')==camera
                     and numeric(r.get('captured_s'))),key=lambda r:r['captured_s'],reverse=True)
        for row in rows:
            candidate=_raw_lane_candidate(row,settings,reference_time)
            if candidate is None:continue
            geometry=_single_view_geometry(candidate,settings,now=reference_time,primary_ground=primary_ground)
            if geometry is None:continue
            geometry.update(coordinate_domain='lane_fraction',geometry_source='raw_image_lane',
                            nav_goal_compatible=False,same_camera_as_navigation=camera==source,
                            fit_residual_px=candidate['raw_fit_residual_px'],fit_points=candidate['raw_fit_points'])
            return geometry
    for camera in cameras:
        for row in course.get('single_heading') or []:
            if row.get('camera')!=camera:continue
            geometry=_single_edge_heading(row,settings,reference_time,primary_ground)
            if geometry is not None:
                geometry['same_camera_as_navigation']=camera==source
                return geometry
    return direct


def _single_edge_heading(row,settings,now,primary_ground=None):
    """Direction of a continuously tracked real stripe, without lane position."""
    captured=row.get('captured_s');anchor=row.get('paired_observed_s')
    if not frame_fresh(captured,now) or not numeric(anchor) or not 0<=captured-anchor<=5.:return None
    try:
        points=np.asarray(row.get('points'),float)
        H=_reference(settings,row['camera'],primary_ground)
        if points.ndim!=2 or points.shape[1]!=2 or len(points)<6 or not np.isfinite(points).all():return None
        if np.ptp(points[:,1])<settings['image_size'][1]*.10:return None
        projected=np.column_stack((points,np.ones(len(points))))@H.T
        if np.any(np.abs(projected[:,2])<1e-8):return None
        ground=projected[:,:2]/projected[:,2,None]
        ground=ground[(ground[:,1]>.2)&(ground[:,1]<6.)&(np.abs(ground[:,0])<3.)]
        if len(ground)<6 or np.ptp(ground[:,1])<.15:return None
        slope,offset=np.polyfit(ground[:,1],ground[:,0],1)
        if abs(slope)>2. or np.max(np.abs(ground[:,0]-slope*ground[:,1]-offset))>.06:return None
        vanish=np.linalg.solve(H,np.array([slope,1.,0.]))
        if abs(vanish[2])<1e-8 or abs(H[0,0])<1e-8:return None
        x,y=vanish[:2]/vanish[2]
        reference=-(H[0,1]*y+H[0,2])/H[0,0]
        correction=float((x-reference)/settings['image_size'][0])
        if not math.isfinite(correction) or abs(correction)>.35:return None
    except (KeyError,ValueError,TypeError,OSError,np.linalg.LinAlgError):return None
    return {'camera':row['camera'],'captured_s':captured,'lane_observed_s':captured,
        'heading_only':True,'single_edge_observed':True,'line_held':False,
        'line_age_s':now-captured,'fresh_lane':False,'nav_goal_compatible':False,
        'near_ego_fraction':None,'body_ego_fraction':None,
        'heading_correction_fraction':correction,'geometry_source':'current_single_stripe_direction',
        'heading_source':'tracked_stripe_vs_historical_ground_direction',
        'paired_reference_age_s':captured-anchor,'metric_verified':False,
        **_visible_edge(row, points, H, settings)}


def _visible_edge(row, points, H, settings):
    """Current stripe relative to the forward image line, not a fabricated pair."""
    side=row.get('side')
    if side not in ('left','right'):return {}
    y=float(np.max(points[:,1]))
    x=float(np.polyval(np.polyfit(points[:,1],points[:,0],1),y))
    reference=float(-(H[0,1]*y+H[0,2])/H[0,0])
    gap=(reference-x) if side=='left' else (x-reference)
    return {'observed_edge_side':side,'edge_clearance_fraction':gap/settings['image_size'][0],
            'edge_row_fraction':y/settings['image_size'][1]}
