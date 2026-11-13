"""Illumination recovery must retain measured color, obstacles and boundaries."""
from pathlib import Path
import json
import cv2
import numpy as np
import pytest
from carvision.tape_pixels import local_tape_image,broad_glare
from carvision.parking_bay import board_direction_hint,blue_board_mask,detect_parking
from carvision.parking_entry_detection import board_adjacent_entry

CONFIG={'blue_lower':[90,70,40],'blue_upper':[135,255,255],
        'yellow_lower':[15,70,80],'yellow_upper':[40,255,255],'surface':'auto'}


def reflected_board(kind='supported'):
    hsv=np.full((360,480,3),(0,0,130),np.uint8)
    hsv[180:]=(65,180,120)
    hsv[120:220,280:380]=(115,220,160)
    hsv[222:249,280:380]=(115,220,90)
    hsv[215:220,200:275]=(25,180,200)
    if kind=='no_tape':hsv[215:220,200:275]=(65,180,120)
    if kind=='no_left_ground':hsv[220:250,190:280]=(0,0,130)
    if kind=='glare_ground':hsv[220:270,180:280]=(65,35,240)
    return hsv


@pytest.mark.parametrize('mirror',[False,True])
def test_reflected_blue_foot_uses_adjacent_tape_and_both_floor_sides(mirror):
    hsv=reflected_board()
    if mirror:hsv=cv2.flip(hsv,1)
    before=blue_board_mask(hsv,CONFIG).copy()
    hint=board_direction_hint(hsv,CONFIG)
    assert hint['suggested_slot']==('right' if mirror else 'left')
    assert hint['boards'][0]['support_source']=='lateral_ground_and_tape'
    assert np.array_equal(before,blue_board_mask(hsv,CONFIG))
    assert not any(row.get('occupancy_verified') for row in hint['boards'])


@pytest.mark.parametrize('kind',['no_tape','no_left_ground','glare_ground'])
def test_blue_patch_without_independent_surrounding_evidence_stays_ambiguous(kind):
    hint=board_direction_hint(reflected_board(kind),CONFIG)
    assert hint['suggested_slot'] is None
    assert hint['rejected_boards'][0]['reason']=='insufficient_ground_or_adjacent_tape'


def test_daylight_board_recovery_does_not_promote_unconfirmed_bay_to_clear():
    root=Path(__file__).parent/'fixtures/sunlight-parking-20261110'
    row=json.loads((root/'frames.jsonl').read_text().splitlines()[0])
    s=json.loads((root/'settings.json').read_text(encoding='utf8'))['parking']
    with (root/'frames.mjpg').open('rb') as f:
        f.seek(row['offset']);image=cv2.imdecode(np.frombuffer(f.read(row['length']),np.uint8),1)
    obs,_=detect_parking(image,s)
    assert obs['direction_hint']['suggested_slot']=='left'
    assert obs['direction_hint']['boards'][0]['support_source']=='lateral_ground_and_tape'
    assert not obs['entry_candidates'] and not obs['perception_verified']
    assert obs['diagnostics']['entry_search']['reason']=='no_board_associated_enclosure'


def test_local_normalization_keeps_original_glare_dark_pixels_and_colors():
    hsv=np.full((360,480,3),(65,180,100),np.uint8)
    hsv[180:260,190:300]=(60,35,240)
    hsv[280:330,30:130]=(70,40,60)
    cv2.line(hsv,(20,340),(150,160),(0,0,140),8)
    image=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR);before=image.copy()
    raw=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    after=cv2.cvtColor(local_tape_image(image),cv2.COLOR_BGR2HSV)
    assert np.array_equal(image,before)
    supported=(raw[:,:,1]<=70)&(raw[:,:,2]>=100)&(broad_glare(raw)==0)
    delta=after[:,:,2].astype(int)-raw[:,:,2].astype(int)
    assert delta.min()>=0 and delta.max()<=70
    assert np.all(delta[~supported]==0)
    assert np.max(np.abs(after[:,:,0].astype(int)-raw[:,:,0]))<=1
    assert np.max(np.abs(after[:,:,1].astype(int)-raw[:,:,1]))<=2


@pytest.mark.parametrize('kind',['closed','missing_edge','blue_inside','unknown_inside'])
def test_real_yellow_edges_partition_floor_without_grayscale_edges(kind):
    edges=np.zeros((360,480),np.uint8);yellow=np.zeros_like(edges)
    ground=np.full_like(edges,255);blue=np.zeros_like(edges)
    q=np.array([[100,245],[214,238],[222,211],[152,213]],np.int32)
    for i in range(4):
        if kind=='missing_edge' and i==1:continue
        cv2.line(yellow,tuple(q[i]),tuple(q[(i+1)%4]),255,3)
    cv2.rectangle(blue,(234,155),(358,236),255,-1)
    if kind=='blue_inside':cv2.rectangle(blue,(160,220),(190,231),255,-1)
    if kind=='unknown_inside':cv2.rectangle(ground,(160,220),(190,231),0,-1)
    hint={'suggested_slot':'left','blocked_side':'right',
          'boards':[{'side':'right','bbox_px':[234,155,358,236],'area_fraction':.05}]}
    stats={};result=board_adjacent_entry(edges,ground,blue,hint,yellow=yellow,diagnostics=stats)
    if kind=='closed':
        assert len(result)==1 and stats['reason']=='accepted_current_enclosure'
        assert not result[0]['occupancy_verified']
    else:assert not result
