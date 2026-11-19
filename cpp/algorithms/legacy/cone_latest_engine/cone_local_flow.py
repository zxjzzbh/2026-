"""Historical projection diagnostics; these estimates never switch the driver."""
import math
import numpy as np

from .brake_parking import numeric
from .cone_timing import frame_fresh


def ground(h,points):
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=2 or not np.isfinite(p).all():return None
    q=np.column_stack((p,np.ones(len(p))))@h.T
    if (np.abs(q[:,2])<1e-6).any():return None
    q=q[:,:2]/q[:,2,None]
    return q if np.isfinite(q).all() else None


def local_observations(projections,rows,now,lane_width):
    lanes=[];cones=[];times=[];diagnostics=[];image_lanes=[];single_headings=[]
    for camera,row in rows.items():
        t=row.get('captured_s')
        if not frame_fresh(t,now) or camera not in projections:continue
        times.append(t);h=projections[camera]
        context=row.get('road_context',{})
        if context.get('single_heading'):
            single_headings.append({**context['single_heading'],'camera':camera})
        if context.get('valid') and not context.get('single_edge_observed'):
            edges=[];counts=[];reason='insufficient_ground_support'
            left=context.get('left_inner',[]);right=context.get('right_inner',[])
            if len(left)>=6 and len(left)==len(right):
                image_lanes.append({'camera':camera,'captured_s':t,'left':left,'right':right})
            for side in ('left_inner','right_inner'):
                p=ground(h,context.get(side,[]))
                # A few pixels around the old horizon can project hundreds of
                # metres away. Never fit them or relax residual gates to accept.
                if p is not None:p=p[(p[:,1]>.3)&(p[:,1]<3.)&(np.abs(p[:,0])<2.)]
                counts.append(0 if p is None else len(p))
                if p is None or len(p)<6 or np.ptp(p[:,1])<.25:break
                a,b=np.polyfit(p[:,1],p[:,0],1)
                if np.max(np.abs(p[:,0]-a*p[:,1]-b))>.035:reason='ground_fit_residual';break
                edges.append((float(a),float(b)))
            if len(edges)==2:
                a=(edges[0][0]+edges[1][0])/2;heading=-math.atan(a)
                width=(edges[1][1]-edges[0][1])*math.cos(heading)
                if abs(width-lane_width)<.15 and abs(edges[0][0]-edges[1][0])<.12:
                    x=-(edges[0][1]+edges[1][1])/2*math.cos(heading)
                    lanes.append({'x':x,'heading':heading,'captured_s':t,'camera':camera})
                    reason='accepted_historical_estimate'
                else:reason='ground_width_or_parallelism_mismatch'
            diagnostics.append({'camera':camera,'reason':reason,'kept_points':counts})
        for track in row.get('tracks',[]):
            if not track.get('visible') or not track.get('confirmed') or track.get('clipped'):
                continue
            if track.get('kind')!='cone_candidate' or not track.get('ground_contact_usable'):continue
            if not track.get('new_target_supported'):continue
            p=ground(h,[track['foot_pixel']])
            if p is None:continue
            x,y=map(float,p[0])
            if not (-1.5<x<1.5 and .2<y<5.):continue
            cones.append({'alias':track['id'],'camera':camera,'x':x,'y':y,'captured_s':t})
    return {'captured_s':max(times,default=now),'lane':lanes,'cones':cones,'image_lane':image_lanes,
            'single_heading':single_headings,
            'ready':bool(lanes),'image_ready':bool(image_lanes),'diagnostics':diagnostics,
            'field_verified':False,'projection_source':'historical_planar_estimate'}

