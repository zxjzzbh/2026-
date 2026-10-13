"""Real failed H65 scene: board choice was right, but steering/geometry were not."""
from pathlib import Path
import cv2
import numpy as np
import pytest
from carvision.parking_bay import detect_parking,load_settings,board_adjacent_entry
from carvision.parking_entry import ParkingEntryController
from carvision.brake_parking import load_settings as brake_settings

ROOT=Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('flip',[False,True])
def test_small_washed_actual_bay_drives_toward_open_side_after_old_timer(flip):
    image=cv2.imdecode(np.frombuffer((ROOT/'vision/tests/fixtures/parking-washed-right-board-20261013.jpg').read_bytes(),np.uint8),1)
    if flip:image=cv2.flip(image,1)
    obs,_=detect_parking(image,load_settings(ROOT/'vision/configs/parking-bay.json'))
    side='right' if flip else 'left'
    assert obs['direction_hint']['suggested_slot']==side
    assert len(obs['entry_candidates'])==1
    target=obs['entry_candidates'][0]
    assert target['id']==side and target['geometry_source']=='board_adjacent_contrast'
    assert target['ground_fraction']>.9 and min(target['edge_support'])>=.7
    assert not obs['metric_geometry'] and target['polygon_m'] is None
    c=ParkingEntryController(brake_settings(ROOT/'vision/configs/brake-parking.json'),side)
    for i in range(55):
        t=round(i*.1,3)
        result=c.step({'fresh':True,'frame_id':i,'captured_s':t,'image_size':[480,360],
                       'parking':obs,'motion':{'valid':True,'moving':True}},t)
        if i>=30:assert result['esc_us']==1575
    assert result['phase']=='approach' and result['far_boundary_fresh']
    assert result['steering_us']<1545 if flip else result['steering_us']>1675
    assert not result['done'] and not result['whole_vehicle_inside_verified']


def test_crop_limits_and_blue_board_alone_do_not_invent_parking_edges():
    edges=np.zeros((360,480),np.uint8);ground=np.full_like(edges,255);blue=np.zeros_like(edges)
    hint={'suggested_slot':'left','blocked_side':'right','boards':[{'side':'right','bbox_px':[259,78,364,176],'area_fraction':.05}]}
    assert board_adjacent_entry(edges,ground,blue,hint)==[]
