"""Yellow-tape parking layout and blue entrance obstruction on real images.

Left/right are relative to the forward-facing camera, not the paper diagram.
Image-only candidates never claim metric pose or clearance verification.
"""
import copy
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .autonomy_profile import GroundProjection
from .parking_guidance import visible_far_guides


def number(x):return type(x) in (int,float) and math.isfinite(x)


def validate(s):
    if not isinstance(s,dict) or s.get('schema_version')!=1 or s.get('camera')!='secondary' or s.get('image_size')!=[480,360]:
        raise ValueError('停车使用第二路 H65 的480×360原图')
    s=copy.deepcopy(s);s.setdefault('max_run_s',30.);s.setdefault('surface','auto')
    if s['surface'] not in ('auto','red_running_track','indoor_green'):raise ValueError('停车地面类型无效')
    for name,lo,hi in [('zone_length_m',1,1),('lane_width_m',.6,2),('vehicle_width_m',.1,.6),
                       ('front_extent_m',.1,.8),('rear_extent_m',.01,.4),('clearance_m',.01,.1),
                       ('blocked_blue_fraction',.005,.3),('clear_ground_fraction',.5,.95),
                       ('selection_confirm_s',.2,2),('stop_hold_s',.4,2),('max_frame_age_s',.1,.25),('max_speed_mps',.02,.25),('max_run_s',1,60)]:
        if not number(s.get(name)) or not lo<=s[name]<=hi:raise ValueError('停车参数越界：'+name)
    for name in ('minimum_turn_radius_m','braking_distance_m','maximum_pose_error_m'):
        if s.get(name) is not None and (not number(s[name]) or s[name]<=0):raise ValueError(name+' must be measured positive metres or null')
    if type(s.get('perception_verified')) is not bool:raise ValueError('perception_verified must be boolean')
    for color in ('yellow','blue'):
        a,b=s.get(color+'_lower'),s.get(color+'_upper')
        if not all(isinstance(v,list) and len(v)==3 and all(type(x) is int for x in v) for v in (a,b)):
            raise ValueError('HSV must contain three integers')
        if not all(0<=a[i]<=b[i]<=(179 if i==0 else 255) for i in range(3)):raise ValueError('HSV range invalid')
    if s.get('ground') is not None:
        if not isinstance(s['ground'],dict) or type(s['ground'].get('verified')) is not bool:raise ValueError('ground.verified must be boolean')
        GroundProjection(s['ground'],require_verified=False)
        if s['ground'].get('image_size')!=s['image_size']:raise ValueError('地面标定分辨率不匹配')
    return copy.deepcopy(s)


def load_settings(path):return validate(json.loads(Path(path).read_text(encoding='utf-8-sig')))


def readiness(s):
    missing=[]
    if (s.get('ground') or {}).get('verified') is not True:missing.append('第二路固定姿态的近场地面标定')
    if not s.get('perception_verified'):missing.append('停车区/入口遮挡识别的现场验证')
    for key,label in [('minimum_turn_radius_m','实测转弯半径'),('braking_distance_m','当前低速的实测制动距离'),('maximum_pose_error_m','持续位姿反馈的误差上限')]:
        if s.get(key) is None:missing.append(label)
    if s['lane_width_m']/2<=s['vehicle_width_m']+2*s['clearance_m']:missing.append('半区宽度不足以容纳整车及余量')
    if s['front_extent_m']+s['rear_extent_m']+2*s['clearance_m']>=s['zone_length_m']:missing.append('车位长度不足以容纳整车及余量')
    return {'ready':not missing,'missing':missing,'hardware_output_enabled':False}


def _order(points):
    p=np.asarray(points,dtype=float)
    near=p[np.argsort(p[:,1])[-2:]];far=p[np.argsort(p[:,1])[:2]]
    near=near[np.argsort(near[:,0])];far=far[np.argsort(far[:,0])]
    return np.array([near[0],near[1],far[1],far[0]],np.float32)


def _edge_support(mask,a,b,radius=3):
    pts=np.linspace(a,b,max(12,int(np.linalg.norm(np.asarray(a)-b))))
    h,w=mask.shape;valid=0
    for x,y in np.rint(pts).astype(int):
        if 0<=x<w and 0<=y<h and np.any(mask[max(0,y-radius):min(h,y+radius+1),max(0,x-radius):min(w,x+radius+1)]):valid+=1
    return valid/len(pts)


def _pixel_quads(yellow,white,kernel=5):
    mask=cv2.morphologyEx(cv2.bitwise_or(yellow,white),cv2.MORPH_CLOSE,np.ones((kernel,kernel),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    h,w=mask.shape;quads=[]
    for contour in contours:
        area=cv2.contourArea(contour)
        if not .02*w*h<area<.7*w*h:continue
        q=cv2.approxPolyDP(contour,.025*cv2.arcLength(contour,True),True)
        if len(q)!=4 or not cv2.isContourConvex(q):continue
        q=_order(q[:,0,:]);center=q.mean(axis=0)
        if np.any(q<=2) or np.any(q[:,0]>=w-3) or np.any(q[:,1]>=h-3):continue
        if any(np.linalg.norm(center-old.mean(axis=0))<.035*w for old in quads):continue
        if min(_edge_support(yellow,q[0],q[1]),_edge_support(yellow,q[2],q[3]))<.45:continue
        quads.append(q)
    return quads


def pixel_layout(yellow,white):
    """Find two side-by-side enclosures sharing a yellow longitudinal divider."""
    quads=_pixel_quads(yellow,white);w=yellow.shape[1]
    pairs=[]
    for i,a in enumerate(quads):
        for b in quads[i+1:]:
            left,right=sorted((a,b),key=lambda q:q[:,0].mean())
            gap=max(np.linalg.norm(left[1]-right[0]),np.linalg.norm(left[2]-right[3]))
            divider=(left[[1,2]]+right[[0,3]])/2
            if gap<=.06*w and _edge_support(yellow,*divider)>=.6:
                pairs.append((left,right))
    return list(pairs[0]) if len(pairs)==1 else []


def occluded_layout(yellow,white,blue):
    """One observed bay plus adjacent entrance board: visual selection only.

    Return None for the hidden polygon. Never mirror a polygon across an
    uncalibrated perspective image or invent the missing outer boundary.
    """
    h,w=yellow.shape;quads=_pixel_quads(yellow,white,9);boards=[]
    contours,_=cv2.findContours(blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x,y,bw,bh=cv2.boundingRect(contour);area=cv2.contourArea(contour)
        if area>=.008*w*h and bw>=.08*w and bh>=.08*h and area/(bw*bh)>=.5 and .5<=bw/bh<=4:
            boards.append((x,y,bw,bh,float(area/(bw*bh))))
    candidates=[];edges=cv2.bitwise_or(yellow,white)
    for q in quads:
        span=float(np.linalg.norm(q[1]-q[0]))
        if span<w*.15 or q[:2,1].mean()-q[2:,1].mean()<h*.06:continue
        for missing,edge in [('left',q[[0,3]]),('right',q[[1,2]])]:
            # Nine-pixel gap closing can shift the contour chord several
            # pixels inward; inspect its finite neighbourhood, not a single
            # ideal straight chord through the dark interior.
            support=_edge_support(yellow,*edge,radius=6)
            if support<.6 or _edge_support(edges,*edge,radius=6)<.8:continue
            for x,y,bw,bh,fill in boards:
                cx=x+bw/2;bottom=y+bh-1
                if abs(q[1,0]-q[0,0])<1 or abs(edge[1,1]-edge[0,1])<1:continue
                near_y=q[0,1]+(cx-q[0,0])*(q[1,1]-q[0,1])/(q[1,0]-q[0,0])
                divider_x=edge[0,0]+(bottom-edge[0,1])*(edge[1,0]-edge[0,0])/(edge[1,1]-edge[0,1])
                if abs(bottom-near_y)>max(10,h*.065) or not .35*span<=bw<=1.7*span:continue
                if missing=='left':
                    if not cx<divider_x or not -.04*w<=divider_x-(x+bw)<=.12*w:continue
                else:
                    if not cx>divider_x or not -.04*w<=x-divider_x<=.12*w:continue
                candidates.append({'missing':missing,'visible':q,'board_bbox':[x,y,x+bw,y+bh],
                                   'board_fraction':fill,'divider_support':support})
    return candidates[0] if len(candidates)==1 else None,{'visible_quad_candidates':len(quads),
        'blue_board_candidates':len(boards),'occluded_layout_candidates':len(candidates)}


def board_direction_hint(hsv,s):
    """Coarse direction cue independent of complete parking geometry.

    A large blue rectangle away from the bonnet suggests an obstructed side;
    the opposite side is only a recommendation, never a verified empty bay.
    """
    h,w=hsv.shape[:2];lower=list(s['blue_lower']);lower[0]=min(s['blue_upper'][0],max(100,lower[0]))
    mask=cv2.inRange(hsv,tuple(lower),tuple(s['blue_upper']))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    areas={'left':0.,'right':0.};boxes=[]
    for contour in contours:
        x,y,bw,bh=cv2.boundingRect(contour);area=float(cv2.contourArea(contour));cx=x+bw/2
        if (area<.005*w*h or bw<.06*w or bh<.08*h or not .5<=bw/bh<=4
                or area/(bw*bh)<.5 or not .30*h<=y+bh<=.90*h):continue
        # A central board does not establish left/right; let the other cues
        # or explicit fallback select a side without pretending it is empty.
        side='left' if cx<w*.47 else 'right' if cx>w*.53 else None
        boxes.append({'bbox_px':[x,y,x+bw,y+bh],'side':side,'area_fraction':area/(w*h)})
        if side:areas[side]+=area/(w*h)
    blocked=max(areas,key=areas.get);total=sum(areas.values())
    if total<=0 or areas[blocked]/total<.68:return {'suggested_slot':None,'source':'ambiguous_blue_position','areas':areas,'boards':boxes}
    chosen='right' if blocked=='left' else 'left'
    return {'suggested_slot':chosen,'blocked_side':blocked,'source':'blue_board_position',
            'basis':'蓝色挡板主要在'+('左' if blocked=='left' else '右')+'侧，暂选另一侧；边界未必完整',
            'areas':areas,'boards':boxes}


def visible_entry_candidates(yellow,white,ground,blue,visible,hint):
    """Observed clear quadrilateral plus a coarse board-side hint, not a map."""
    side=hint.get('suggested_slot');blocked=hint.get('blocked_side')
    boards=[b for b in hint.get('boards',[]) if b.get('side')==blocked]
    if side not in ('left','right') or not boards:return []
    board=max(boards,key=lambda b:b['area_fraction'])['bbox_px'];board_x=(board[0]+board[2])/2
    result=[];h,w=yellow.shape
    for q in _pixel_quads(yellow,white,9):
        center_x=float(q[:,0].mean())
        if (side=='right' and center_x<=board_x) or (side=='left' and center_x>=board_x):continue
        if abs(float(q[:2,1].mean())-board[3])>h*.12:continue
        mask=np.zeros(yellow.shape,np.uint8);cv2.fillConvexPoly(mask,q.astype(np.int32),255)
        inside=cv2.erode(mask,np.ones((7,7),np.uint8))>0
        if inside.sum()<150:continue
        floor=float(np.mean(ground[inside]>0));br=float(np.mean(blue[inside]>0))
        if floor<.65 or br>=.01 or float(visible[inside].mean())<.65:continue
        result.append({'id':side,'corners_px':q.tolist(),'availability':'clear','candidate_only':True,
            'occupancy_verified':False,'polygon_m':None,'ground_fraction':floor,'blue_inside_fraction':br,
            'geometry_source':'visible_candidate'})
    return result if len(result)==1 else []


def _clusters(values):
    groups=[]
    for value in values:
        if not groups or value-groups[-1][-1]>3:groups.append([int(value)])
        else:groups[-1].append(int(value))
    return [int(round(np.mean(g))) for g in groups]


def metric_layout(yellow,white,s):
    """Infer the occluded half only when measured width and visible lines agree.

    Axis-aligned straight approach only. No unsupported corner extrapolation.
    """
    p=GroundProjection(s['ground']);scale=160.;grid=np.array([[scale,0,240],[0,-scale,480],[0,0,1.]])
    warp=grid@p.h;size=(480,512)
    y=cv2.warpPerspective(yellow,warp,size,flags=cv2.INTER_NEAREST)>0
    edges=cv2.warpPerspective(cv2.bitwise_or(yellow,white),warp,size,flags=cv2.INTER_NEAREST)>0
    rows=_clusters(np.flatnonzero(y.sum(axis=1)>s['lane_width_m']*scale*.2))
    found=[];half=s['lane_width_m']*scale/2
    for far in rows:
        for near in rows:
            if not .88*scale<=near-far<=1.12*scale:continue
            centers=_clusters(np.flatnonzero(y[far:near+1].sum(axis=0)>(near-far)*.6))
            for mid in centers:
                bounds=[]
                for expected in (mid-half,mid+half):
                    lo=max(0,int(expected-6));hi=min(480,int(expected+7))
                    if hi<=lo:break
                    support=edges[far:near+1,lo:hi].sum(axis=0)
                    x=lo+int(support.argmax())
                    if support.max()<(near-far)*.45:break
                    bounds.append(x)
                if len(bounds)!=2:continue
                left,right=bounds
                if not abs((right-left)/scale-s['lane_width_m'])<=.08 or abs(mid-(left+right)/2)>6:continue
                metric=[];pixels=[]
                try:
                    for a,b in ((left,mid),(mid,right)):
                        ground=np.array([[(a-240)/scale,(480-near)/scale],[(b-240)/scale,(480-near)/scale],
                                         [(b-240)/scale,(480-far)/scale],[(a-240)/scale,(480-far)/scale]])
                        inv=np.linalg.inv(p.h);q=np.c_[ground,np.ones(4)]@inv.T;q=q[:,:2]/q[:,2:]
                        for point in q:p.point(point)
                        metric.append(ground.tolist());pixels.append(q.astype(np.float32))
                except (ValueError,np.linalg.LinAlgError):continue
                found.append((pixels,metric))
    return found[0] if len(found)==1 else ([],[])


def detect_parking(image,s):
    if [image.shape[1],image.shape[0]]!=s['image_size']:raise ValueError('停车图像尺寸与配置不一致')
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    yellow=cv2.inRange(hsv,tuple(s['yellow_lower']),tuple(s['yellow_upper']))
    original_yellow_pixels=int(cv2.countNonZero(yellow))
    # Actual H65 frames render cream tape at S~30..60. This pale-color band
    # excludes saturated green flooring and is still subject to line geometry.
    pale=cv2.inRange(hsv,(15,25,110),(65,110,255))
    yellow=cv2.bitwise_or(yellow,pale)
    white=cv2.inRange(hsv,(0,0,150),(179,80,255))
    blue=cv2.inRange(hsv,tuple(s['blue_lower']),tuple(s['blue_upper']))
    red=cv2.bitwise_or(cv2.inRange(hsv,(0,55,35),(15,255,255)),cv2.inRange(hsv,(165,55,35),(179,255,255)))
    green=cv2.inRange(hsv,(40,45,35),(99,255,255))
    grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);visible=(grey>18)&(grey<250)
    pixels=[];metric=[]
    if (s.get('ground') or {}).get('verified') is True:
        pixels,metric=metric_layout(yellow,white,s)
    if not pixels:pixels=pixel_layout(yellow,white)
    source='metric' if metric else 'two_visible_halves' if pixels else None
    partial=None;diagnostics={'original_yellow_pixels':original_yellow_pixels,'yellow_pixels':int(cv2.countNonZero(yellow))}
    if not pixels:
        partial,detail=occluded_layout(yellow,white,blue);diagnostics.update(detail)
        if partial:
            pixels=[None,partial['visible']] if partial['missing']=='left' else [partial['visible'],None]
            source='one_visible_half_with_adjacent_blue_board'
    slots=[];display=image.copy()
    for i,poly in enumerate(pixels):
        name='left' if i==0 else 'right'
        if poly is None:
            box=partial['board_bbox'];x1,y1,x2,y2=box
            slots.append({'id':name,'availability':'blocked','occupancy_verified':False,'corners_px':None,'polygon_m':None,
                'blue_entrance_fraction':None,'blue_board_fill_fraction':partial['board_fraction'],
                'red_ground_fraction':0.,'visible_fraction':0.,
                'ground_fraction':0.,'ground_surface':'unknown','geometry_visible':False,'board_bbox_px':box})
            cv2.rectangle(display,(x1,y1),(x2,y2),(0,0,255),2)
            cv2.putText(display,name+' blocked / hidden edges',(max(0,x1),max(15,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,255),1)
            continue
        interior=np.zeros(grey.shape,np.uint8);cv2.fillConvexPoly(interior,poly.astype(np.int32),255)
        interior=cv2.erode(interior,np.ones((7,7),np.uint8))>0
        # In image coordinates this near-edge band is for candidate visibility;
        # no blue top corner is projected as a ground contact point.
        band=np.array([poly[0],poly[1],poly[1]*.7+poly[2]*.3,poly[0]*.7+poly[3]*.3],np.int32)
        entrance=np.zeros(grey.shape,np.uint8);cv2.fillConvexPoly(entrance,band,255);entrance=entrance>0
        enough=int(interior.sum())>150 and int(entrance.sum())>50
        red_floor=float(np.mean(red[interior]>0)) if enough else 0.
        green_floor=float(np.mean(green[interior]>0)) if enough else 0.
        surface=s.get('surface','auto')
        if surface=='auto':surface='indoor_green' if green_floor>red_floor else 'red_running_track'
        floor=green_floor if surface=='indoor_green' else red_floor
        selected_blue=blue
        if surface=='indoor_green':
            # Cyan cast on this green floor overlaps the old blue H=90 edge.
            # Match the cone pipeline's blue band instead of counting floor noise.
            lower=list(s['blue_lower']);lower[0]=min(s['blue_upper'][0],max(100,lower[0]))
            selected_blue=cv2.inRange(hsv,tuple(lower),tuple(s['blue_upper']))
        br=float(np.mean(selected_blue[entrance]>0)) if enough else 0.
        vis=float(visible[interior].mean()) if enough else 0.
        blue_inside=float(np.mean(selected_blue[interior]>0)) if enough else 0.
        availability=('blocked' if enough and br>=s['blocked_blue_fraction'] else
                      'clear' if enough and br<s['blocked_blue_fraction']/2 and blue_inside<.01 and floor>=s['clear_ground_fraction'] and vis>=.65 else 'unknown')
        verified=bool(metric and s['perception_verified'] and availability!='unknown')
        slot={'id':name,'availability':availability,'occupancy_verified':verified,
              'corners_px':poly.tolist(),'polygon_m':metric[i] if metric else None,
              'blue_entrance_fraction':br,'red_ground_fraction':red_floor,'visible_fraction':vis,
              'ground_fraction':floor,'ground_surface':surface,'geometry_visible':True}
        slots.append(slot);color={'blocked':(0,0,255),'clear':(0,200,0),'unknown':(0,180,255)}[availability]
        cv2.polylines(display,[poly.astype(np.int32)],True,color,2)
        center=tuple(poly.mean(axis=0).astype(int));cv2.putText(display,slot['id']+' '+availability,center,cv2.FONT_HERSHEY_SIMPLEX,.45,color,2)
    # Always show observed yellow tape, even without a complete layout.
    contours,_=cv2.findContours(yellow,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(display,contours,-1,(0,255,255),1)
    hint=board_direction_hint(hsv,s)
    ground=red if s.get('surface')=='red_running_track' else green if s.get('surface')=='indoor_green' else cv2.bitwise_or(red,green)
    lower=list(s['blue_lower']);lower[0]=min(s['blue_upper'][0],max(100,lower[0]))
    strong_blue=cv2.inRange(hsv,tuple(lower),tuple(s['blue_upper']))
    candidates=visible_entry_candidates(yellow,white,ground,strong_blue,visible,hint) if not slots else []
    guides=visible_far_guides(yellow,white,ground,strong_blue)
    return {'layout_found':len(slots)==2,'layout_complete':len(slots)==2 and partial is None,
            'layout_source':source,'diagnostics':diagnostics,'metric_geometry':bool(metric),'slots':slots,
            'direction_hint':hint,'entry_candidates':candidates,'entry_guides':guides,
            'reason':('可见半区边界与相邻入口蓝挡板已识别；遮挡侧边界未完整确认' if partial else
                      '两个停车半区边界已识别' if slots else '黄线未形成可确认车位；需看到空闲半区近/远边与分隔线'),
            'perception_verified':s['perception_verified'],'hardware_output':False},display
