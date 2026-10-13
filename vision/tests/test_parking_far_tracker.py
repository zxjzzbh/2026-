from pathlib import Path
import cv2
import numpy as np

from carvision.parking_bay import load_settings,detect_parking
from carvision.parking_entry import ParkingEntryController
from carvision.brake_parking import load_settings as load_brake
from carvision.parking_far_tracker import ParkingFarTracker

ROOT=Path(__file__).resolve().parents[2]


def scene():
    im=cv2.imdecode(np.frombuffer((ROOT/'vision/tests/fixtures/parking-washed-right-board-20261013.jpg').read_bytes(),np.uint8),1)
    settings=load_settings(ROOT/'vision/configs/parking-bay.json')
    return im,settings,detect_parking(im,settings)[0]


def test_actual_edge_pixels_keep_tracking_without_any_new_bay_boxes_and_trigger_brake():
    image,settings,obs=scene();tracker=ParkingFarTracker()
    c=ParkingEntryController(load_brake(ROOT/'vision/configs/brake-parking.json'),'left',steering_gain=1.25)
    braked=False;outputs=[]
    for i in range(78):
        shift=max(0,min(81,(i-30)*3));frame=cv2.warpAffine(image,np.float32([[1,0,0],[0,1,shift]]),(480,360))
        observed=dict(obs) if i<=30 else {'slots':[]}
        observed['tracked_far_lines']=tracker.update(frame,observed,i,round(i*.1,3),settings)
        out=c.step({'fresh':True,'frame_id':i,'captured_s':round(i*.1,3),'image_size':[480,360],
                    'parking':observed,'motion':{'valid':True,'moving':not braked}},round(i*.1,3))
        outputs.append(out)
        if out['phase']=='brake_pulse':braked=True
    assert all(x['esc_us']==1575 for x in outputs[30:56])
    assert outputs[55]['geometry_source']=='optical_flow_far_edge'
    assert outputs[55]['geometry']['near_edge_y_normalized'] is None
    assert any(x['esc_us']==1400 for x in outputs)
    assert outputs[-1]['done'] and not outputs[-1]['whole_vehicle_inside_verified']


def test_no_detector_seed_never_invents_an_optical_far_edge():
    image,settings,_=scene();tracker=ParkingFarTracker()
    for i in range(10):assert tracker.update(image,{'slots':[]},i,i*.1,settings)==[]


def test_blank_replacement_or_camera_gap_invalidates_tracking():
    image,settings,obs=scene()
    for blank,gap in ((True,.1),(False,.5)):
        tracker=ParkingFarTracker();tracker.update(image,obs,0,0.,settings)
        assert tracker.tracks
        assert tracker.update(np.zeros_like(image) if blank else image,{'slots':[]},1,gap,settings)==[]
        assert not tracker.tracks
