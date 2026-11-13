"""Offline camera replay. Changed commands do not simulate a new trajectory."""
from pathlib import Path
import json
import cv2
import numpy as np
from carvision.parking_entry_run import ParkingEntryRun
from carvision.parking_bay import detect_parking, parking_road_context, contrast_markings, blue_board_mask, yellow_marking_mask
from carvision.parking_ground_view import ParkingGroundMotion
from carvision.parking_far_tracker import ParkingFarTracker
from carvision.brake_parking import GroundMotionEstimate
from carvision.cone_road import ConeRoadContext


def replay(data,*,staged=False):
    settings=json.loads((data/'settings.json').read_text(encoding='utf8'))
    settings['brake'].update(brake_pulse_s=.35,max_frame_age_s=.5,actuator_lease_s=.5)
    if staged:settings['parking'].setdefault('visual_entry',{})['ground_pose_confirmed']=True
    controller=ParkingEntryRun._controller(settings)
    ground_motion=ParkingGroundMotion(settings['cone']['ground']) if staged else None
    road=ConeRoadContext(settings['cone'],'secondary',thorough_boundary_search=True)
    tracker=ParkingFarTracker();motion=GroundMotionEstimate()
    original={r['sample']['frame_id']:r for r in map(json.loads,(data/'decisions.jsonl').read_text(encoding='utf8').splitlines())}
    frames=list(map(json.loads,(data/'frames.jsonl').read_text().splitlines()))
    outputs=[]
    with (data/'frames.mjpg').open('rb') as stream:
        for frame in frames:
            stream.seek(frame['offset']);image=cv2.imdecode(np.frombuffer(stream.read(frame['length']),np.uint8),1)
            fid,t=frame['frame_id'],frame['captured_s']
            observation,overlay=detect_parking(image,settings['parking'])
            observation['road_context']=parking_road_context(road,image,settings['parking'])
            observation['tracked_far_lines']=tracker.update(image,observation,fid,t,settings['parking'])
            movement=motion.update(image,fid,t)
            ground_row=None
            if ground_motion is not None:
                hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
                marks,floor=contrast_markings(hsv,cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),settings['parking'])
                mask=cv2.bitwise_or(floor,cv2.bitwise_or(marks,yellow_marking_mask(hsv,settings['parking'])))
                mask[blue_board_mask(hsv,settings['parking'])>0]=0
                ground_row=ground_motion.update(image,fid,t,mask)
            row=original.get(fid)
            if row is None or row['intent']['phase']=='ready':continue
            sample={'frame_id':fid,'captured_s':t,'camera_id':'secondary','image_size':[480,360],
                    'fresh':True,'parking':observation,'motion':movement}
            if ground_row is not None:sample['parking_ground_motion']=ground_row
            intent=controller.step(sample,row['now_s'])
            outputs.append({'frame_id':fid,'now_s':row['now_s'],'sample':sample,'intent':intent})
    return outputs


