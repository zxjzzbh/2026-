"""Shared, verified ground-feedback adapter for metric parking; no output."""
import copy
import math

from .cone_navigation import ConeOdometry
from .parking_control import parking_blockers
from .cone_vision import BlueConeDetector, ImageConeTracker
from .cone_road import ConeRoadContext
from .runtime_timing import RACE_CONTROL_WINDOW_S, MOTION_MAX_AGE_S
from .parking_bay import number
from .parking_localization import (transform_slots, verified_slots, road_pose, register_slots, continuous_rest)


class ParkingWorldFeed:
    """Convert verified camera-local bays into the same odometry world frame.

    No historical flag is promoted to current verification. A cached static
    layout is usable only briefly while calibrated motion and visibility remain
    valid; fresh contradictory layout/occupancy is never hidden by the cache.
    """
    def __init__(self,bundle):
        self.b=bundle;self.odo=None;self.map=None;self.map_at=None;self.last_id=None
        self.anchor=None;self.last_pose=None;self.last_good_at=None;self.settings=None
        self.recovery_at=None;self.conflict_after=None;self.candidate=None;self.recovery_hits=0
        self.epoch=0;self.recovery_reason=None;self.last_capture=None;self.registration_error=0.
        if not parking_blockers(bundle):
            self.settings=copy.deepcopy(bundle['cones'])
            self.settings['ground']=copy.deepcopy(bundle['parking']['ground'])
            self.odo=ConeOdometry(self.settings,max_sample_gap_s=RACE_CONTROL_WINDOW_S)

    def _reset_hits(self,reason):
        self.candidate=None;self.recovery_hits=0;self.recovery_reason=reason

    def _recover(self,image,row,view,now,motion):
        t=row['captured_s'];s=self.b['parking'];moving=motion or {}
        stamp=moving.get('captured_s');since=moving.get('continuous_still_since_s')
        if (moving.get('valid') is not True or moving.get('moving') is not False
                or not number(stamp) or not 0<=now-stamp<=MOTION_MAX_AGE_S):
            self.conflict_after=now
            self._reset_hits('等待独立新鲜停稳反馈');return None
        if (not continuous_rest(moving,now,self.recovery_at+.35)
                or self.conflict_after is not None and (not number(since) or since<=self.conflict_after)
                or not row['road_context'].get('valid')):
            self._reset_hits('连续停稳或道路边界尚未确认');return None
        try:
            local=verified_slots(view)
            base_error=self.settings['driving']['visual_error_bound_m']
            if self.anchor is None:
                pose={**road_pose(local,s),'error_bound_m':base_error}
            else:
                pose=register_slots(local,self.anchor,s,self.last_pose,
                                    max(0.,t-self.last_good_at),base_error)
            if pose['error_bound_m']>s['maximum_pose_error_m']:
                raise ValueError('定位误差超过停车预算')
            if self.candidate is not None:
                prior=self.candidate
                if (math.hypot(pose['x_m']-prior['x_m'],pose['y_m']-prior['y_m'])>s['maximum_pose_error_m']
                        or abs(pose['yaw_rad']-prior['yaw_rad'])>.025):
                    raise ValueError('连续车位重定位不一致')
        except (ValueError,TypeError,KeyError) as exc:
            self._reset_hits(str(exc));return None
        self.candidate=pose;self.recovery_hits+=1
        if self.recovery_hits<3:return None
        # Start a new optical-flow baseline in the SAME road world. The first
        # image supplies feature points only; independent stillness supplies
        # the stopped state. No old failed estimate or old route is reused.
        replacement=ConeOdometry(self.settings,max_sample_gap_s=RACE_CONTROL_WINDOW_S)
        replacement.update(image,row['frame_id'],t,row['tracks'],row['road_context'])
        if replacement.failed or replacement.previous is None or replacement.previous[1] is None or len(replacement.previous[1])<20:
            self._reset_hits('重新定位后缺少有效地面特征');return None
        replacement.pose={k:pose[k] for k in ('x_m','y_m','yaw_rad')}
        replacement.travel=self.odo.travel
        self.registration_error=pose['error_bound_m']-base_error
        self.odo=replacement;self.epoch+=1
        self.recovery_at=None;self.conflict_after=None;self._reset_hits(None)
        self.map=transform_slots(local,pose);self.map_at=t
        if self.anchor is None:self.anchor=copy.deepcopy(self.map)
        self.last_pose=dict(pose);self.last_good_at=t
        return self._world(row,pose,0.,relocalized=True)

    def _world(self,row,pose,speed,*,relocalized=False):
        t=row['captured_s']
        return {'frame_id':row['frame_id'],'captured_s':t,'coordinate_frame':'parking_road_world',
                'pose':{**pose,'verified':True,'captured_s':t},'speed_mps':speed,
                'slots':copy.deepcopy(self.map),'obstacle_region_verified':bool(row['road_context']['valid']),
                'source':'verified_bay_relocalization_and_stillness' if relocalized else 'verified_ground_flow_and_parking_layout',
                'layout_age_s':t-self.map_at,'localization_epoch':self.epoch,'relocalized':relocalized}

    def update(self,image,row,view,now,*,motion=None):
        if self.odo is None or type(row.get('frame_id')) is not int or row['frame_id']==self.last_id:return None
        t=row['captured_s']
        maximum_age=getattr(self,'b',{}).get('parking',{}).get('max_frame_age_s',.25)
        if not number(t) or not number(now) or not 0<=now-t<=maximum_age:return None
        if self.last_id is not None and row['frame_id']<self.last_id:return None
        if getattr(self,'last_capture',None) is not None and t<=self.last_capture:return None
        self.last_id=row['frame_id'];self.last_capture=t
        if getattr(self,'recovery_at',None) is not None:
            return self._recover(image,row,view,now,motion)
        result=self.odo.update(image,row['frame_id'],t,row['tracks'],row['road_context'])
        if not result['valid']:
            if getattr(self.odo,'failed',False):
                self.recovery_at=now;self._reset_hits('地面定位失效，等待停稳后重定位原车位')
            return None
        pose=dict(result['pose']);slots=None
        if 'error_bound_m' in pose:pose['error_bound_m']+=getattr(self,'registration_error',0.)
        if view.get('metric_geometry') and len(view.get('slots',[]))==2:
            if self.anchor is None:
                try:local=verified_slots(view);pose={**road_pose(local,self.b['parking']),
                    'error_bound_m':pose['error_bound_m']}
                except (ValueError,KeyError,TypeError):return None
                self.odo.pose={k:pose[k] for k in ('x_m','y_m','yaw_rad')}
                self.anchor=transform_slots(local,pose)
            slots=transform_slots(view['slots'],pose)
            # Fresh contradiction remains visible to the mission; do not use
            # the anchor to hide changed occupancy or a displaced boundary.
            self.map=copy.deepcopy(slots);self.map_at=t
        elif view.get('layout_found'):
            self.map=None;self.map_at=None
            return None
        elif self.map is not None and 0<=t-self.map_at<=2.:
            if any(r.get('visible') and (r.get('clipped') or r.get('kind') in ('unresolved_blue','unresolved_red'))
                   and r['bbox_xyxy'][3]>.8*image.shape[0] for r in row['tracks']):return None
            slots=copy.deepcopy(self.map)
        if slots is None:return None
        self.last_pose=dict(pose);self.last_good_at=t
        return self._world(row,pose,result['speed_mps'])


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

    def update(self,image,frame_id,captured_s,view,now,*,motion=None):
        detections,mask=self.detector.detect(image)
        context=self.road.inspect(image,mask)
        self.road.enrich(detections,image,context)
        tracks=self.tracker.update(detections,frame_id,captured_s)
        row={'frame_id':frame_id,'captured_s':captured_s,'tracks':tracks,'road_context':context}
        return self.world.update(image,row,view,now,motion=motion)
