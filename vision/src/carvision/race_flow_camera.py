"""Phase-aware perception over the console's existing camera caches only."""
import copy
import math
import time

import cv2
import numpy as np

from .brake_trial import TrialCamera
from .brake_parking import GroundMotionEstimate
from .traffic_trial import TrafficCamera
from .traffic_signal import load_profile
from .cone_observer import ConeCamera
from .cone_low_speed import DualLowSpeedNavigator
from .cone_navigation import ConeOdometry
from .departure import detect_board
from .parking_bay import detect_parking
from .race_flow import parking_blockers
from .crosswalk import CrosswalkDetector


class _FrameOnlyNavigator:
    def update(self,*args):return None
    def draw(self,image):return image.copy()


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
            if any(r.get('visible') and (r.get('clipped') or r.get('kind')=='unresolved_blue')
                   and r['bbox_xyxy'][3]>.8*image.shape[0] for r in row['tracks']):return None
            slots=copy.deepcopy(self.map)
        if slots is None:return None
        return {'frame_id':row['frame_id'],'captured_s':t,'coordinate_frame':'parking_road_world',
                'pose':{**pose,'verified':True,'captured_s':t},'speed_mps':result['speed_mps'],
                'slots':slots,'obstacle_region_verified':bool(row['road_context']['valid']),
                'source':'verified_ground_flow_and_parking_layout','layout_age_s':t-self.map_at}


class RaceFlowCamera:
    def __init__(self,states,bundle,root,clock=time.monotonic):
        self.states,self.b,self.clock=states,bundle,clock
        self.cones=ConeCamera(states,bundle['cones'],clock)
        self.cones.navigator=_FrameOnlyNavigator()
        self.navigator=DualLowSpeedNavigator(bundle['cones'])
        self.zebra={camera:CrosswalkDetector() for camera in ('primary','secondary')}
        self.zebra_ids={};self.zebra_masks={}
        self.crosswalk=TrialCamera(states,bundle['brake'])
        self.traffic=TrafficCamera(states,bundle['traffic'],load_profile(root/'vision/configs/traffic-signal.json'),clock)
        self.parking=ParkingWorldFeed(bundle);self.motion=GroundMotionEstimate();self.motion_result={}
        self.board={};self.board_ids={};self.last_secondary=None;self.last_crosswalk=None;self.last_traffic=None
        self.preview=None;self.sequence=0;self.latest=None

    def read(self,stage,now):
        changed=self.cones.read()
        if changed is None:return None
        rows=copy.deepcopy(self.cones.rows);images=self.cones.images
        if stage in ('crosswalk','traffic'):
            # Like the original zebra trial, remove detected paper stripes
            # from lane fitting so they cannot masquerade as track boundaries.
            for camera,row in rows.items():
                if self.zebra_ids.get(camera)!=row['frame_id']:
                    result,_=self.zebra[camera].detect(images[camera],row['captured_s']*1000)
                    _,mask=self.cones.detectors[camera].detect(images[camera])
                    for x,y,w,h in result['stripes_xywh']:mask[y:y+h,x:x+w]=255
                    self.zebra_masks[camera]=mask;self.zebra_ids[camera]=row['frame_id']
                row['road_context']=self.cones.roads[camera].inspect(images[camera],self.zebra_masks[camera])
        navigation=self.navigator.update_views(images,rows,self.clock())
        secondary=rows.get('secondary')
        if secondary and secondary['frame_id']!=self.last_secondary:
            self.motion_result={**self.motion.update(images['secondary'],secondary['frame_id'],secondary['captured_s']),
                                'captured_s':secondary['captured_s']}
            self.last_secondary=secondary['frame_id']
        self.sequence+=1
        sample={'sequence':self.sequence,'captured_s':max(r['captured_s'] for r in rows.values()),
                'navigation':copy.deepcopy(navigation),'motion':dict(self.motion_result)}
        panes=[]
        for camera in ('primary','secondary'):
            if camera not in images:panes.append(np.zeros((360,480,3),np.uint8));continue
            display=images[camera].copy();context=rows[camera]['road_context']
            for edge in ('left_inner','right_inner'):
                if context.get(edge):cv2.polylines(display,[np.asarray(context[edge],np.int32)],False,(0,220,80),2)
            for track in rows[camera]['tracks']:
                if track.get('visible'):
                    x1,y1,x2,y2=track['bbox_xyxy'];cv2.rectangle(display,(x1,y1),(x2,y2),(255,160,40),2)
            cv2.putText(display,camera.upper()+' / '+stage,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(240,240,240),1)
            panes.append(self.navigator.draw_camera(camera,display))
        ok,jpg=cv2.imencode('.jpg',np.concatenate(panes,axis=1))
        self.preview=jpg.tobytes() if ok else None
        if stage in ('arming','wait_board'):
            for camera,row in rows.items():
                if self.board_ids.get(camera)!=row['frame_id']:
                    detected=detect_board(images[camera],self.b['departure'].get('min_blue_percent',3.))
                    self.board[camera]={**detected,'frame_id':row['frame_id'],'captured_s':row['captured_s']}
                    self.board_ids[camera]=row['frame_id']
            sample['board']=copy.deepcopy(self.board)
        elif stage=='crosswalk':
            value=self.crosswalk.read(self.clock())
            if value is not None:self.last_crosswalk=value
            sample['crosswalk']=copy.deepcopy(self.last_crosswalk)
        elif stage=='traffic':
            value=self.traffic.read(self.clock())
            if value is not None:self.last_traffic=value
            sample['traffic']=copy.deepcopy(self.last_traffic)
            self.preview=self.traffic.preview
        elif stage in ('parking_search','parking_entry') and secondary:
            view,display=detect_parking(images['secondary'],self.b['parking'])
            sample['parking_view']=view
            sample['parking']=self.parking.update(images['secondary'],secondary,view,self.clock())
            ok,jpg=cv2.imencode('.jpg',display)
            if ok:self.preview=jpg.tobytes()
        self.latest=sample
        return sample
