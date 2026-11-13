"""Current tape-to-floor transitions shared by full and partial bay tracking."""
import cv2
import numpy as np


def inner_line(markings, ground, a, b, center, *, radius=16, minimum_alignment=.96):
    """Fit the floor side of the tape, never its brightness-weighted centre."""
    length=float(np.linalg.norm(b-a))
    if length<20:return None
    direction=(b-a)/length
    normal=np.array([-direction[1],direction[0]])
    if normal@(center-(a+b)/2)<0:normal=-normal
    # Track prediction places the band only. Each accepted point requires tape
    # outside and floor inside in this frame; no extrapolated edge is evidence.
    origins=a+np.linspace(.15,.85,max(20,int(length*.65)))[:,None]*(b-a)
    offsets=np.arange(-radius,radius+1,dtype=np.float32)
    coords=np.rint(origins[:,None,:]+offsets[None,:,None]*normal).astype(int)
    h,w=markings.shape
    valid=(coords[:,:,0]>=0)&(coords[:,:,0]<w)&(coords[:,:,1]>=0)&(coords[:,:,1]<h)
    x=np.clip(coords[:,:,0],0,w-1);y=np.clip(coords[:,:,1],0,h-1)
    tape=(markings[y,x]>0)&valid;floor=(ground[y,x]>0)&~tape&valid
    choices=(tape[:,:-3]&floor[:,1:-2]&floor[:,2:-1]&floor[:,3:])
    points=[];sections=[]
    for index,(origin,indices) in enumerate(zip(origins,choices)):
        positions=np.flatnonzero(indices)
        if len(positions):
            k=positions[np.argmin(np.abs(offsets[positions]))]
            points.append(origin+(offsets[k]+1.)*normal)
            sections.append(min(2,index*3//len(origins)))
    if len(points)<max(12,len(origins)*.55):return None
    points=np.asarray(points,np.float32)
    vx,vy,x,y=cv2.fitLine(points,cv2.DIST_HUBER,0,.01,.01).ravel()
    vector=np.array([vx,vy]);normal=np.array([-vy,vx])
    residual=np.abs((points-[x,y])@normal)
    if np.quantile(residual,.85)>2.5:
        # A tape corner or a small reflection can displace a few nearest
        # transitions. Retain a consensus only if current points still cover
        # all three sections of this edge, at the original pixel tolerance.
        keep=residual<=2.5;sections=np.asarray(sections)
        required=max(12,len(origins)*.55)
        if (keep.sum()<required or any(np.sum(keep&(sections==part))<max(2,len(origins)*.1)
                                      for part in range(3))):return None
        points=points[keep]
        vx,vy,x,y=cv2.fitLine(points,cv2.DIST_HUBER,0,.01,.01).ravel()
        vector=np.array([vx,vy]);normal=np.array([-vy,vx])
        residual=np.abs((points-[x,y])@normal)
    if abs(float(vector@direction))<minimum_alignment or np.quantile(residual,.85)>2.5:return None
    return normal,float(normal@np.array([x,y]))


def inner_quad(markings,ground,predicted,*,near_observed=True,radius=16,corner_limit=23,acquiring=False):
    lines=[];center=predicted.mean(axis=0)
    for i,(a,b) in enumerate(zip(predicted,np.roll(predicted,-1,axis=0))):
        if i==0 and not near_observed:
            # This is a cut through visible floor, explicitly NOT an entrance.
            lines.append((np.array([0.,1.]),float(min(a[1],b[1]))))
        else:
            # Initial contour simplification can displace the end of a short
            # side by several pixels. Permit that bounded seed uncertainty;
            # established tracking keeps its original angle restriction.
            alignment=.96
            if acquiring:
                angle=np.arctan2(radius,max(1.,np.linalg.norm(b-a)))
                alignment=max(.85,float(np.cos(angle)))
            line=inner_line(markings,ground,a,b,center,radius=radius,minimum_alignment=alignment)
            if line is None:return None
            lines.append(line)
    corners=[]
    for i in range(4):
        previous,current=lines[i-1],lines[i]
        matrix=np.asarray([previous[0],current[0]])
        if abs(np.linalg.det(matrix))<.15:return None
        corners.append(np.linalg.solve(matrix,[previous[1],current[1]]))
    q=np.asarray(corners,np.float32)
    if np.max(np.linalg.norm(q-predicted,axis=1))>corner_limit:return None
    return q


def associated_quad(markings,ground,predicted,dt,*,near_observed=True):
    """Expand only pixel search, retaining current tape/floor and shape checks."""
    limit=min(45.,23.+120.*max(0.,min(.2,dt)))
    for radius in (16,24,32):
        q=inner_quad(markings,ground,predicted,near_observed=near_observed,
                     radius=radius,corner_limit=limit)
        if q is None:continue
        old_width=np.array([np.linalg.norm(predicted[1]-predicted[0]),np.linalg.norm(predicted[2]-predicted[3])])
        width=np.array([np.linalg.norm(q[1]-q[0]),np.linalg.norm(q[2]-q[3])])
        if np.any(old_width<20) or np.any(width/old_width<.72) or np.any(width/old_width>1.4):continue
        return q,radius
    return None,None
