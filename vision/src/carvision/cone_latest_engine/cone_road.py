"""Shared paint detection with bounded, labelled single-edge recovery."""
import cv2
import numpy as np
from ..cone_road import ConeRoadContext as PaintContext
from .cone_edge_recovery import TurnEdgeRecovery

class ConeRoadContext(PaintContext):
    def __init__(self, settings, camera='secondary', *, thorough_boundary_search=False):
        super().__init__(settings, camera, thorough_boundary_search=thorough_boundary_search, turn_aware=True)
        self.recovery = TurnEdgeRecovery()

    def inspect(self, image, blue_mask, *, frame_id=None, captured_s=None):
        result = super().inspect(image, blue_mask, frame_id=frame_id, captured_s=captured_s)
        recovered=self.recovery.update(result,image.shape,captured_s)
        if self.recovery.current_heading is not None:
            result['single_heading']=self.recovery.current_heading
        if recovered is not None:
            ys = np.linspace(recovered.low, recovered.high, 13)
            result.update(valid=True, single_edge_observed=True,
                prediction_age_s=recovered.prediction_age_s, paired_observed_s=self.recovery.paired[0],
                left_inner=np.column_stack((recovered.left.at(ys),ys)).tolist(),
                right_inner=np.column_stack((recovered.right.at(ys),ys)).tolist(),
                reason='current_single_edge_with_bounded_width_reference')
            result['white_lines_detected'] = False
        return result

    @staticmethod
    def enrich(detections, image, context):
        PaintContext.enrich(detections, image, context)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        floor = (cv2.inRange(hsv,(40,45,35),(99,255,255)) if context.get('ground_color') is None else
                 cv2.bitwise_or(cv2.inRange(hsv,(0,55,35),(15,255,255)),cv2.inRange(hsv,(165,55,35),(179,255,255))))
        h,w = floor.shape
        for row in detections:
            x1,y1,x2,y2 = row['bbox_xyxy'];pad=max(4,round((x2-x1)*.4))
            patch=floor[max(0,y2):min(h,y2+pad),max(0,x1-pad):min(w,x2+pad)]
            support=float(np.mean(patch>0)) if patch.size else 0.
            row['floor_contact_fraction']=support
            edge=context.get('single_heading')
            if edge and edge.get('side') in ('left','right'):
                points=np.asarray(edge.get('points'),float)
                if points.ndim==2 and points.shape[1]==2 and len(points)>=6:
                    foot=row.get('foot_pixel',[(x1+x2)/2,y2]);y=foot[1]
                    if points[:,1].min()-h*.10<=y<=points[:,1].max()+h*.10:
                        boundary=float(np.polyval(np.polyfit(points[:,1],points[:,0],1),y))
                        if edge['side']=='right' and x1>boundary+5 or edge['side']=='left' and x2<boundary-5:
                            row['boundary_relation']='outside'
                            row['current_edge_exclusion']=True
            # Ground below a blue board is not sufficient new-cone evidence.
            # With no observed corridor, a far object at the image border is
            # not allowed to seed history. Already tracked near cones continue
            # separately in the navigator as they leave the picture.
            central=(x1+x2)/2/w
            image_entry_supported=(.15<=central<=.85 or y2/h>=.78)
            row['new_target_supported']=(row.get('boundary_relation')!='outside' and
                (row.get('scene_supported') is True and not context.get('single_edge_observed')
                 or support>=.55 and image_entry_supported))
        return detections
