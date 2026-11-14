"""Phase-aware perception over the console's existing camera caches only."""
import copy
from .motion_cache import GroundMotionCache
from .preview_encoding import PreviewEncoder
from .runtime_timing import VISION_MAX_AGE_S
import time

import cv2
import numpy as np

from .brake_trial import TrialCamera
from .traffic_trial import TrafficCamera
from .traffic_signal import load_profile
from .cone_observer import ConeCamera
from .cone_low_speed import DualLowSpeedNavigator
from .departure import DEFAULT_MIN_BLUE_PERCENT,detect_board
from .parking_bay import detect_parking
from .parking_feedback import ParkingWorldFeed
from .crosswalk import CrosswalkDetector


class _FrameOnlyNavigator:
    def update(self,*args):return None
    def draw(self,image):return image.copy()




class RaceFlowCamera:
    def __init__(self,states,bundle,root,clock=time.monotonic):
        self.states,self.b,self.clock=states,bundle,clock
        self.cones=ConeCamera(states,bundle['cones'],clock,turn_aware=True)
        self.cones.navigator=_FrameOnlyNavigator()
        self.navigator=DualLowSpeedNavigator(bundle['cones'])
        self.zebra={camera:CrosswalkDetector() for camera in ('primary','secondary')}
        self.zebra_ids={};self.zebra_masks={};self.zebra_images={};self.zebra_results={}
        self.road_frames={};self.mask_zebra=False;self.calibrated_zebra=False
        self.crosswalk=TrialCamera(states,bundle['brake'],clock=self.clock)
        self.traffic=TrafficCamera(states,bundle['traffic'],load_profile(root/'vision/configs/traffic-signal.json'),clock)
        self.parking=ParkingWorldFeed(bundle);self.motion_result={}
        self.board={};self.board_ids={};self.last_crosswalk=None;self.last_traffic=None
        self.preview=None;self.sequence=0;self.latest=None
        self.motion_cache=GroundMotionCache(states,clock=clock)
        self.encoder=PreviewEncoder()

    def set_avoidance_target(self,camera,identifier):
        self.navigator.set_avoidance_target(camera if identifier else None,identifier or None)

    def _road_mask(self,camera,image,frame_id,captured_s,mask):
        # These references bind a mask to the exact decoded view, including a
        # restarted camera reusing a frame number. No cached view is re-fitted.
        self.road_frames[camera]={'frame_id':frame_id,'image':image,'zebra':self.mask_zebra,
                                  'calibrated_zebra':self.calibrated_zebra}
        if not self.mask_zebra:return mask
        if self.zebra_ids.get(camera)!=frame_id or self.zebra_images.get(camera) is not image:
            detector=(self.crosswalk.calibrated.detectors[camera] if self.calibrated_zebra
                      else self.zebra[camera])
            result,_=detector.detect(image,captured_s*1000)
            filtered=mask.copy()
            for x,y,w,h in result['stripes_xywh']:filtered[y:y+h,x:x+w]=255
            self.zebra_ids[camera]=frame_id;self.zebra_images[camera]=image
            self.zebra_masks[camera]=filtered
            self.zebra_results[camera]={'frame_id':frame_id,'captured_s':captured_s,
                'image':image,'detector':detector,'zebra':result}
        return self.zebra_masks[camera]

    def _road_ready(self,camera,row,image):
        source=self.road_frames.get(camera)
        return bool(source and source['frame_id']==row['frame_id']
            and source['image'] is image and source['zebra']==self.mask_zebra
            and source['calibrated_zebra']==self.calibrated_zebra)

    def _lane_preview(self,stage,rows,images):
        panes=[]
        for camera in ('primary','secondary'):
            if camera not in images:panes.append(np.zeros((360,480,3),np.uint8));continue
            display=images[camera].copy()
            ready=self._road_ready(camera,rows[camera],images[camera])
            context=rows[camera]['road_context'] if ready else {}
            for edge in ('left_inner','right_inner'):
                if context.get(edge):cv2.polylines(display,[np.asarray(context[edge],np.int32)],False,(0,220,80),2)
            for track in rows[camera]['tracks']:
                if track.get('visible'):
                    x1,y1,x2,y2=track['bbox_xyxy'];cv2.rectangle(display,(x1,y1),(x2,y2),(255,160,40),2)
            cv2.putText(display,camera.upper()+' / '+stage,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.45,(240,240,240),1)
            panes.append(self.navigator.draw_camera(camera,display) if ready else display)
        self.preview=self.encoder.submit(np.concatenate(panes,axis=1))

    def close(self):
        self.motion_cache.close()
        self.encoder.close()
        self.traffic.close()
        if callable(getattr(self.cones,'close',None)):self.cones.close()

    def _arming_sample(self):
        motion=self.motion_cache.snapshot()
        stamp=motion.get('captured_s')
        if stamp is None or stamp==getattr(self,'arming_stamp',None):return None
        self.arming_stamp=stamp;self.sequence+=1
        return {'sequence':self.sequence,'captured_s':stamp,'motion':motion}

    def ground_motion(self):
        return self.motion_cache.snapshot()

    def read(self,stage,now):
        started=time.perf_counter();timing={}
        self.motion_cache.snapshot()
        if stage=='arming':return self._arming_sample()
        # Keep one curve model. A stage change can reprocess the same real
        # frame with its proper mask, without inventing another capture.
        self.mask_zebra=stage in ('crosswalk','traffic')
        self.calibrated_zebra=stage=='crosswalk' and self.crosswalk.calibrated is not None
        self.navigator.set_single_recovery_limit(1. if stage=='cones' else .25)
        parking_entry=stage=='parking_entry'
        try:changed=self.cones.read(road_mask_filter=self._road_mask,render=False,
            track_cones=stage in ('cones','parking_search','parking_entry'),
            cameras=('secondary',) if parking_entry else ('primary','secondary'),
            road_mask_mode=(self.mask_zebra,self.calibrated_zebra))
        except ValueError as exc:
            if str(exc)=='两路摄像头均无新鲜画面':return None
            raise
        if changed is None:return None
        timing['camera_and_road_s']=time.perf_counter()-started
        section=time.perf_counter()
        rows=copy.deepcopy(self.cones.rows);images=self.cones.images
        lane_rows={camera:row for camera,row in rows.items()
            if self._road_ready(camera,row,images[camera])}
        lane_images={camera:images[camera] for camera in lane_rows}
        navigation=None if parking_entry else self.navigator.update_views(lane_images,lane_rows,self.clock())
        timing['navigation_s']=time.perf_counter()-section
        secondary=lane_rows.get('secondary')
        self.motion_result=self.motion_cache.snapshot()
        self.sequence+=1
        sample={'sequence':self.sequence,'captured_s':max(r['captured_s'] for r in rows.values()),
                'navigation':copy.deepcopy(navigation),'motion':dict(self.motion_result)}
        parking_preview=stage in ('parking_search','parking_entry') and secondary is not None
        # Traffic/parking provide the final preview themselves; avoid encoding
        # a temporary two-pane JPEG that would be overwritten below.
        section=time.perf_counter()
        if stage!='traffic' and not parking_preview:self._lane_preview(stage,rows,images)
        timing['lane_preview_s']=time.perf_counter()-section
        section=time.perf_counter()
        if stage in ('arming','wait_board'):
            for camera,row in rows.items():
                if self.board_ids.get(camera)!=row['frame_id']:
                    detected=detect_board(images[camera],self.b['departure'].get('min_blue_percent',DEFAULT_MIN_BLUE_PERCENT))
                    self.board[camera]={**detected,'frame_id':row['frame_id'],'captured_s':row['captured_s']}
                    self.board_ids[camera]=row['frame_id']
            sample['board']=copy.deepcopy(self.board)
        elif stage=='crosswalk':
            try:
                if self.calibrated_zebra:
                    views={camera:{'frame':self.cones.frames[camera],
                        'features':self.zebra_results[camera],'road':{**row,'image':images[camera]}}
                        for camera,row in lane_rows.items()}
                    value=self.crosswalk.read_processed(self.clock(),views,self.motion_cache.snapshot())
                else:value=self.crosswalk.read(self.clock())
            except ValueError as exc:
                if not any(text in str(exc) for text in ('画面失效','处理后帧龄','缓存不可用','画面全黑')):raise
                self.last_crosswalk=None;value=None
            if value is not None:self.last_crosswalk=value
            sample['crosswalk']=copy.deepcopy(self.last_crosswalk)
            if sample['crosswalk']:sample['crosswalk']['motion']=self.motion_cache.snapshot()
        elif stage=='traffic':
            try:value=self.traffic.read(self.clock())
            except ValueError as exc:
                if not str(exc).startswith('两路均无新鲜图像：'):raise
                self.last_traffic=None;value=None
            if value is not None:self.last_traffic=value
            sample['traffic']=copy.deepcopy(self.last_traffic)
            if sample['traffic']:sample['traffic']['motion']=self.motion_cache.snapshot()
            self.preview=self.traffic.preview
        elif stage in ('parking_search','parking_entry') and secondary:
            view,display=detect_parking(images['secondary'],self.b['parking'])
            sample['parking_view']=view
            sample['parking']=self.parking.update(images['secondary'],secondary,view,self.clock())
            self.preview=self.encoder.submit(display)
        timing['task_s']=time.perf_counter()-section
        sample['vision_processing_s']=time.perf_counter()-started
        sample['vision_stages_s']=timing
        self.latest=sample
        return sample
