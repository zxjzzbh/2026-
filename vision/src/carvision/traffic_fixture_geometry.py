"""Current circle-edge evidence for a red fixture under color spill.

An incomplete circular edge is measured by angular coverage and fit residual,
not filled in. This extra evidence may support a red stop, never a green go.
"""
import cv2
import numpy as np


def arc_anchors(image, red_cores):
    if not red_cores:return []
    grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    edges=cv2.Canny(cv2.GaussianBlur(grey,(3,3),0),15,40)
    contours,_=cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    result=[]
    for contour in contours:
        if len(contour)<30:continue
        (x,y),(a,b),angle=cv2.fitEllipse(contour)
        if min(a,b)<8 or max(a,b)/min(a,b)>1.8:continue
        radius=(a+b)/4
        if not any(cx-r<=x<=cx+12*r and abs(y-cy)<=r*1.5 and .5*r<=radius<=1.8*r
                   for cx,cy,r in red_cores):continue
        rad=np.deg2rad(angle)
        q=(contour[:,0]-[x,y])@np.array([[np.cos(rad),-np.sin(rad)],[np.sin(rad),np.cos(rad)]])
        q/=np.array([a,b])/2
        sectors=np.unique(np.floor((np.arctan2(q[:,1],q[:,0])+np.pi)*16/(2*np.pi))).size
        if sectors<12 or np.percentile(abs(np.linalg.norm(q,axis=1)-1),90)>.15:continue
        if not any(np.hypot(x-u,y-v)<radius*.35 for u,v,_,_ in result):
            result.append((float(x),float(y),float(radius),1))
        if len(result)>=12:break
    return result


def radial_edges(image):
    grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    return cv2.Sobel(grey,cv2.CV_32F,1,0,ksize=3),cv2.Sobel(grey,cv2.CV_32F,0,1,ksize=3)


def rim_support(gradients,x,y,r):
    gx,gy=gradients
    angles=np.arange(24)*np.pi/12
    radii=np.linspace(r*.75,r*1.5,9)
    xx=np.rint(x+np.cos(angles)[:,None]*radii).astype(int)
    yy=np.rint(y+np.sin(angles)[:,None]*radii).astype(int)
    valid=(xx>=0)&(yy>=0)&(xx<gx.shape[1])&(yy<gx.shape[0])
    xx=xx.clip(0,gx.shape[1]-1);yy=yy.clip(0,gx.shape[0]-1)
    radial=np.abs(gx[yy,xx]*np.cos(angles)[:,None]+gy[yy,xx]*np.sin(angles)[:,None])
    tangent=np.abs(-gx[yy,xx]*np.sin(angles)[:,None]+gy[yy,xx]*np.cos(angles)[:,None])
    return float(np.mean(np.any(valid&(radial>30)&(radial>tangent*1.5),axis=1)))


def red_spill_supported(features,centers,radius,gradients):
    red,middle,last=features
    return bool(red['white_fraction']>=.6 and red['halo_fraction']>=.12 and red['distributed_halo']
        and red['median_v']-max(middle['median_v'],last['median_v'])>=15
        and red['shape_support'] and middle['shape_support']
        and rim_support(gradients,*centers[2],radius)>=.65)
