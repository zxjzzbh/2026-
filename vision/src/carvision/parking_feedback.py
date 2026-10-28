"""Shared, verified ground-feedback adapter for metric parking; no output."""
import copy
import math

from .cone_navigation import ConeOdometry
from .parking_control import parking_blockers
from .cone_vision import BlueConeDetector, ImageConeTracker
from .cone_road import ConeRoadContext


class ParkingWorldFeed:
    """Convert verified camera-local bays into the same odometry world frame.

    No historical flag is promoted to current verification. A cached static
    layout is usable only briefly while calibrated motion and visibility remain
    valid; fresh contradictory layout/occupancy is never hidden by the cache.
    """
    def __init__(self,bundle):
        self.b=bundle;self.odo=None;self.map=None;self.map_at=None;self.last_id=None
        if not parking_blockers(bundle):
            settings=copy.deepcopy(bundle['cones']);settings['ground']=copy.deepcopy(bundle['parking']['ground'])
            self.odo=ConeOdometry(settings)

    def update(self,image,row,view,now):
        if self.odo is None or row['frame_id']==self.last_id:return None
        self.last_id=row['frame_id'];t=row['captured_s']
        result=self.odo.update(image,row['frame_id'],t,row['tracks'],row['road_context'])
        if not result['valid']:return None
        pose=result['pose'];co,si=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
        slots=None
        if view.get('metric_geometry') and len(view.get('slots',[]))==2:
            slots=copy.deepcopy(view['slots'])
            for slot in slots:
                slot['polygon_m']=[[pose['x_m']+co*x+si*y,pose['y_m']-si*x+co*y] for x,y in slot['polygon_m']]
            # Keep the fresh values: ParkingMission rejects jumps/changed bays.
            self.map=copy.deepcopy(slots);self.map_at=t
        elif view.get('layout_found'):
            # A partial image-only candidate is not a fresh measured layout.
            # Do not turn its hidden polygon into a cached metric clearance.
            self.map=None;self.map_at=None
            return None
        elif self.map is not None and 0<=t-self.map_at<=2.:
            # Only a short near-field occlusion with valid odometry is allowed.
            # New ambiguous blue in the near driving corridor prevents reuse.
            if any(r.get('visible') and (r.get('clipped') or r.get('kind') in ('unresolved_blue','unresolved_red'))
                   and r['bbox_xyxy'][3]>.8*image.shape[0] for r in row['tracks']):return None
            slots=copy.deepcopy(self.map)
        if slots is None:return None
        return {'frame_id':row['frame_id'],'captured_s':t,'coordinate_frame':'parking_road_world',
                'pose':{**pose,'verified':True,'captured_s':t},'speed_mps':result['speed_mps'],
                'slots':slots,'obstacle_region_verified':bool(row['road_context']['valid']),
                'source':'verified_ground_flow_and_parking_layout','layout_age_s':t-self.map_at}


class ParkingFrameFeedback:
    """Reuse cone masks/road visibility with the same world feed as race mode.

    Consumes the existing secondary frame; never opens a camera or connection.
    A new preparation creates a new odometry origin and discards old bay maps.
    """
    def __init__(self, parking, cones):
        self.detector=BlueConeDetector(cones)
        self.tracker=ImageConeTracker('secondary',cones)
        self.road=ConeRoadContext(cones,thorough_boundary_search=True)
        self.world=ParkingWorldFeed({'parking':parking,'cones':cones})

    def update(self,image,frame_id,captured_s,view,now):
        detections,mask=self.detector.detect(image)
        context=self.road.inspect(image,mask)
        self.road.enrich(detections,image,context)
        tracks=self.tracker.update(detections,frame_id,captured_s)
        row={'frame_id':frame_id,'captured_s':captured_s,'tracks':tracks,'road_context':context}
        return self.world.update(image,row,view,now)
