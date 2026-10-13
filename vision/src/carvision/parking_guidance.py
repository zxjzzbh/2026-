"""Image-only far/side parking guides after the entrance leaves the camera.

The near edge is a crop through visible ground, never an observed entrance.
A caller must already have locked a complete bay and match its far corners
across fresh frames before using one of these unlabelled guides.
"""
import cv2
import numpy as np


def _mask(value, shape=None):
    value=np.asarray(value)
    if value.ndim!=2 or (shape is not None and value.shape!=shape):
        raise ValueError('parking guide masks must share a two-dimensional shape')
    return np.where(value>0,255,0).astype(np.uint8)


def _order(points):
    points=np.asarray(points,dtype=np.float32)
    by_y=points[np.argsort(points[:,1])]
    far=by_y[:2][np.argsort(by_y[:2,0])]
    near=by_y[2:][np.argsort(by_y[2:,0])]
    return np.asarray([near[0],near[1],far[1],far[0]],dtype=np.float32)


def _support(expanded,a,b):
    pts=np.rint(np.linspace(a,b,max(16,int(np.linalg.norm(a-b))))).astype(int)
    h,w=expanded.shape
    valid=(pts[:,0]>=0)&(pts[:,0]<w)&(pts[:,1]>=0)&(pts[:,1]<h)
    if not np.all(valid):return 0.
    return float(np.mean(expanded[pts[:,1],pts[:,0]]>0))


def visible_far_guides(yellow,white,ground_mask,blue_mask):
    """Return actual U-shaped ground components; no bay labels or metric pose.

    Candidates must reach the bottom of the inspection ROI and have a real
    yellow far edge plus two real yellow/white side edges. Neither the image
    border nor the ROI's top edge can provide boundary evidence. Blue objects
    and unknown/dark interior pixels cannot be filled in as free ground.
    """
    yellow=_mask(yellow);h,w=yellow.shape
    white=_mask(white,yellow.shape);ground=_mask(ground_mask,yellow.shape)
    blue=_mask(blue_mask,yellow.shape)
    if h<40 or w<40:return []
    top=int(round(.30*h));bottom=min(h-2,int(round(.80*h)))
    edges=cv2.bitwise_or(yellow,white)
    barriers=cv2.dilate(cv2.bitwise_or(edges,blue),np.ones((3,3),np.uint8))
    # Only bridge tiny floor-mask speckles, then reapply every observed barrier.
    free=cv2.morphologyEx(ground,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    free[barriers>0]=0;free[:top]=0;free[bottom+1:]=0
    contours,_=cv2.findContours(free,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    expanded_yellow=cv2.dilate(yellow,np.ones((11,11),np.uint8))
    expanded_edges=cv2.dilate(edges,np.ones((11,11),np.uint8))
    guides=[]
    for contour in contours:
        x,y,cw,ch=cv2.boundingRect(contour)
        if y<=top+2 or y+ch-1<bottom-1 or x<=2 or x+cw>=w-2:continue
        if not .012*w*h<=cv2.contourArea(contour)<=.55*w*h:continue
        hull=cv2.convexHull(contour)
        approx=cv2.approxPolyDP(hull,.018*cv2.arcLength(hull,True),True)
        if len(approx)!=4 or not cv2.isContourConvex(approx):continue
        q=_order(approx[:,0,:]);near_width=q[1,0]-q[0,0];far_width=q[2,0]-q[3,0]
        depths=q[:2,1]-q[[3,2],1]
        if np.any(q[:2,1]<bottom-2) or np.any(q[2:,1]<=top+2):continue
        if min(near_width,far_width)<.06*w or near_width>.90*w:continue
        if not .15<=far_width/near_width<=1.25:continue
        if depths.min()<.06*h or depths.max()>2*depths.min():continue
        far_support=_support(expanded_yellow,q[2],q[3])
        side_support=[_support(expanded_edges,q[0],q[3]),_support(expanded_edges,q[1],q[2])]
        if far_support<.70 or min(side_support)<.75:continue
        interior=np.zeros_like(ground);cv2.fillConvexPoly(interior,np.rint(q).astype(np.int32),255)
        interior=cv2.erode(interior,np.ones((5,5),np.uint8))>0
        if int(interior.sum())<250:continue
        floor_fraction=float(np.mean(ground[interior]>0))
        # A hull may span a hole in the component. Do not hide an obstacle in it.
        if floor_fraction<.90 or np.count_nonzero(blue[interior])>max(5,.003*int(interior.sum())):continue
        guides.append({'corners_px':q.tolist(),'near_boundary_observed':False,
            'far_boundary_observed':True,'far_boundary_support':far_support,
            'side_boundary_support':side_support,'ground_fraction':floor_fraction,
            'guide_source':'visible_far_and_side_boundaries','geometry_source':'image_far_corridor',
            'roi_near_y_px':bottom})
    return sorted(guides,key=lambda row:np.asarray(row['corners_px'])[:,0].mean())
