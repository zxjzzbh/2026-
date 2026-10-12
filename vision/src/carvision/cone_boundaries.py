"""Adaptive paired WHITE INNER edges for a straight cone-test corridor.

The fixed lower-image sampler may miss lines that have already left the frame.
Scan more rows, then require two jointly supported, continuous boundary tracks.
Wide horizontal markings and bright blobs are not treated as lane boundaries.
"""
import cv2
import numpy as np


def adaptive_inner_edges(image,blue_mask,config):
    h,w=image.shape[:2]
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(0,0,config.white_v_min),(179,config.white_s_max,255))
    mask[blue_mask>0]=0
    first_row=round(h*config.roi_top)
    mask[:first_row]=0
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    step=max(2,round(h/120));minimum=max(2,round(w*config.min_run_fraction))
    maximum=max(minimum,round(w*config.max_run_fraction));pairs=[]
    for y in range(first_row,round(h*.90),step):
        row=(mask[max(0,y-1):min(h,y+2)]>0).mean(axis=0)>=.5
        changes=np.diff(np.pad(row.astype(np.int8),(1,1)))
        starts,ends=np.where(changes==1)[0],np.where(changes==-1)[0]
        runs=[(int(a),int(b)) for a,b in zip(starts,ends) if minimum<=b-a<=maximum]
        left=[b for a,b in runs if (a+b)/2<w/2 and b<w-1]
        right=[a-1 for a,b in runs if (a+b)/2>w/2 and a>0]
        for l in left:
            for r in right:
                if config.min_width_fraction*w<r-l<config.max_width_fraction*w:pairs.append((y,l,r))
    diag={'detector':'adaptive_paired_rows','paired_candidates':len(pairs),'support_rows':0,
          'roi_top':config.roi_top,'white_v_min':config.white_v_min,'white_s_max':config.white_s_max}
    if len(pairs)<8:return [],[],mask,{**diag,'reason':'too_few_observed_pairs'}
    data=np.asarray(pairs,float);ys=data[:,0];groups=np.unique(ys,return_index=True)[1]
    tolerance=max(2.5,w*.007);required_span=h*.09;best=None
    rng=np.random.default_rng(20261012)
    # Deterministic bounded hypotheses keep performance and frame-to-frame
    # behavior repeatable. Both sides share the very same inlier rows.
    for i,j in rng.integers(0,len(data),size=(192,2)):
        dy=ys[j]-ys[i]
        if abs(dy)<required_span:continue
        slopes=(data[j,1:]-data[i,1:])/dy
        if not (-6<=slopes[0]<=.15 and -.15<=slopes[1]<=6 and slopes[1]-slopes[0]>=-.05):continue
        residual=np.max(np.abs(data[:,1:]-(data[i,1:]+(ys-data[i,0])[:,None]*slopes)),axis=1)
        good=residual<=tolerance
        row_good=np.maximum.reduceat(good,groups)
        count=int(row_good.sum())
        if count<8:continue
        span=float(np.ptp(ys[good]))
        if span<required_span or count/(span/step+1)<.65:continue
        # Nearby tape can leave the image sides after only 33–42 px of
        # shared support at 360p. Accept that shorter band only with dense,
        # outward-diverging pairs; short posts/glare are not a corridor.
        if span<h*.12 and (count<10 or count/(span/step+1)<.8
                           or slopes[0]>=-.25 or slopes[1]<=.25):continue
        score=(count,span,-float(np.median(residual[good])))
        if best is None or score>best[0]:best=(score,residual)
    if best is None:return [],[],mask,{**diag,'reason':'no_continuous_boundary_pair'}
    residual=best[1];chosen=[]
    for start,end in zip(groups,np.r_[groups[1:],len(data)]):
        index=int(start+np.argmin(residual[start:end]))
        if residual[index]<=tolerance:chosen.append(index)
    selected=data[chosen]
    # Refit and trim only outliers; returned points are actual observed inner
    # edges, never extrapolated pixels painted into a missing side.
    for _ in range(2):
        if len(selected)<8:break
        fit=np.polyfit(selected[:,0],selected[:,1:],1)
        err=np.max(np.abs(selected[:,1:]-(selected[:,0,None]*fit[0]+fit[1])),axis=1)
        selected=selected[err<=tolerance]
    if len(selected)<8 or np.ptp(selected[:,0])<required_span:
        return [],[],mask,{**diag,'reason':'insufficient_refined_support'}
    if np.max(np.diff(selected[:,0]))>h*.06:
        return [],[],mask,{**diag,'reason':'boundary_pair_has_large_gap'}
    short_span=bool(np.ptp(selected[:,0])<h*.12)
    if short_span:
        fit=np.polyfit(selected[:,0],selected[:,1:],1)
        err=np.max(np.abs(selected[:,1:]-(selected[:,0,None]*fit[0]+fit[1])))
        density=len(selected)/(np.ptp(selected[:,0])/step+1)
        if len(selected)<10 or density<.8 or fit[0,0]>=-.25 or fit[0,1]<=.25 or err>max(2.,w*.005):
            return [],[],mask,{**diag,'reason':'short_boundary_pair_not_strong_enough'}
    left=[[float(l),float(y)] for y,l,r in selected]
    right=[[float(r),float(y)] for y,l,r in selected]
    return left,right,mask,{**diag,'support_rows':len(left),'reason':'observed_adaptive_inner_edges',
                            'short_span_checked':short_span,
                            'observed_y_range':[float(selected[0,0]),float(selected[-1,0])]}
