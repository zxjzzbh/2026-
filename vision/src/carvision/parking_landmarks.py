"""Keep currently visible bay sides after an entrance or one corner leaves view.

Predictions place search bands only. Returned lines need current tape/floor
transitions; no absent entrance, hidden side or occupancy is fabricated.
"""
import cv2
import numpy as np
from .parking_inner_edges import inner_line


class ParkingLandmarks:
    def __init__(self):
        self.references={}

    @staticmethod
    def _segment(edge):
        try:p=np.asarray(edge,dtype=np.float32)
        except (ValueError,TypeError):return None
        if p.shape!=(2,2) or not np.isfinite(p).all():return None
        return p

    @staticmethod
    def _side_segment(edge,top,bottom=350):
        a,b=edge
        if abs(float(b[1]-a[1]))<15:return None
        slope=(b[0]-a[0])/(b[1]-a[1])
        def point(y):return [float(a[0]+slope*(y-a[1])),float(y)]
        return np.asarray([point(bottom),point(max(150,top+10))],np.float32)

    def update(self,observation,fid,t,markings,ground,blue):
        results=[];next_refs={}
        blocked={v.get('id') for v in observation.get('slots',[]) if v.get('availability')!='clear'}
        tracked=[v for v in observation.get('tracked_bay_boundaries',[])
                 if v.get('frame_id')==fid and v.get('captured_s')==t]
        candidates=tracked+list(observation.get('slots',[]))+list(observation.get('entry_candidates',[]))
        for side in ('left','right'):
            if side in blocked:continue
            far_rows=[v for v in observation.get('tracked_far_lines',[]) if v.get('id')==side
                      and v.get('frame_id')==fid and v.get('captured_s')==t and v.get('marking_support',0)>=.8]
            far=self._segment(far_rows[0]['edge_px']) if len(far_rows)==1 else None
            current=[v for v in candidates if v.get('id')==side and v.get('availability')=='clear']
            q=np.asarray(current[0].get('corners_px'),np.float32) if current else None
            if q is not None and (q.shape!=(4,2) or not np.isfinite(q).all()):q=None
            if q is not None:
                sides={'left':q[[0,3]],'right':q[[1,2]]}
                far=q[[3,2]]
                reference={'sides':sides,'far':far,'time':t,'frame':fid}
            else:
                reference=self.references.get(side)
                if reference is None or far is None or not 0<t-reference['time']<=.5 or fid<=reference['frame']:continue
                old=reference['far'];ratio=np.linalg.norm(far[1]-far[0])/max(1.,np.linalg.norm(old[1]-old[0]))
                delta=far.mean(axis=0)-old.mean(axis=0)
                if not .72<=ratio<=1.38 or np.linalg.norm(delta)>45:continue
                sides={name:(p-old.mean(axis=0))*ratio+far.mean(axis=0) for name,p in reference['sides'].items()}
            if far is None or np.linalg.norm(far[1]-far[0])<25:continue
            # Optical flow locates the far tape search band. Refit its inner
            # floor edge just like the sides so all metric lines agree.
            fitted_far=inner_line(markings,ground,*far,far.mean(axis=0)+[0,40])
            if fitted_far is None:continue
            normal,intercept=fitted_far
            if abs(float(normal[1]))<.7:continue
            far=far.copy();far[:,1]=(intercept-normal[0]*far[:,0])/normal[1]
            found={};support={}
            for name,edge in sides.items():
                bottom=(min(350,float(max(edge[:,1]))) if q is not None
                        and current[0].get('near_boundary_observed',True) else 350)
                segment=self._side_segment(edge,float(far[:,1].mean()),bottom)
                if segment is None:continue
                center=segment.mean(axis=0)+np.array([40 if name=='left' else -40,0])
                fitted=None
                for radius in (16,24,32):
                    fitted=inner_line(markings,ground,*segment,center,radius=radius)
                    if fitted is not None:break
                if fitted is None:continue
                normal,intercept=fitted
                if abs(float(normal[0]))<.5:continue
                ys=np.linspace(segment[1,1],segment[0,1],60)
                points=np.column_stack(((intercept-normal[1]*ys)/normal[0],ys))
                keep=(points[:,0]>=2)&(points[:,0]<478)&(points[:,1]>=2)&(points[:,1]<358)
                points=points[keep]
                if len(points)<15 or np.linalg.norm(points[-1]-points[0])<35:continue
                # Confirm tape on the outward side of the fitted floor edge.
                outward=normal if float(normal@(center-segment.mean(axis=0)))<0 else -normal
                tape_points=np.rint(points+outward*3).astype(int)
                good=(tape_points[:,0]>=0)&(tape_points[:,0]<480)&(tape_points[:,1]>=0)&(tape_points[:,1]<360)
                fraction=float(np.mean(markings[tape_points[good,1],tape_points[good,0]]>0)) if good.any() else 0.
                if fraction<.72:continue
                found[name]=points[[0,-1]].astype(np.float32);support[name]=fraction
            if not found:continue
            # Inspect the visible ground between current measured sides. A
            # missing side is not silently replaced to claim the whole bay free.
            if len(found)==2:
                left,right=found['left'],found['right']
                if min(right[:,0]-left[:,0])<25:continue
                polygon=np.asarray([left[0],right[0],right[1],left[1]],np.int32)
                mask=np.zeros_like(ground);cv2.fillConvexPoly(mask,polygon,255)
                inside=cv2.erode(mask,np.ones((9,9),np.uint8))>0
                if inside.sum()<200 or np.mean(blue[inside]>0)>.003 or np.mean(ground[inside]>0)<.9:continue
            next_refs[side]={'sides':found,'far':far,'time':t,'frame':fid}
            results.append({'id':side,'frame_id':fid,'captured_s':t,'source':'current_bay_landmarks',
                'far_edge_px':far.tolist(),'side_edges_px':{k:v.tolist() for k,v in found.items()},
                'side_support':support,'far_observed':True,'near_boundary_observed':False,
                'occupancy_verified':False,'physical_pose_verified':False})
        self.references=next_refs
        return results
