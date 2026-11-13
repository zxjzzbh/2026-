"""Measured tape pixels under glare; never synthesize hidden geometry."""
from pathlib import Path
import json
import cv2
import numpy as np
import pytest
from carvision.tape_pixels import broad_glare,white_tape_mask
from carvision.parking_bay import contrast_markings,detect_parking,blue_board_mask
from carvision.cone_road import ConeRoadContext
from test_cone_white_fallback import settings
from test_cone_turn_corridor import track
from test_parking_image_search import raw_frame


@pytest.mark.parametrize('color',[(255,255,255),(248,248,241),(240,240,240)])
def test_broad_highlight_is_not_a_white_stripe(color):
    image=np.full((360,480,3),(35,100,35),np.uint8)
    cv2.ellipse(image,(240,250),(65,90),0,0,360,color,-1)
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    original=cv2.inRange(hsv,(0,0,200),(179,70,255))
    cleaned=white_tape_mask(hsv,200,70)
    assert np.count_nonzero(original)>15000
    assert np.count_nonzero(cleaned)<np.count_nonzero(original)*.05
    result=ConeRoadContext(settings(200),turn_aware=True).inspect(image,np.zeros((360,480),np.uint8))
    assert not result['white_lines_detected']


@pytest.mark.parametrize('width',[5,12,22])
def test_bleached_tape_keeps_observed_width_and_does_not_become_ground(width):
    image=np.full((360,480,3),(35,100,35),np.uint8)
    image[200:200+width,60:420]=255
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    edges,ground=contrast_markings(hsv,cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),{'surface':'indoor_green'})
    assert np.mean(edges[200:200+width,90:390]>0)>.98
    assert np.count_nonzero(ground[200:200+width,90:390])==0
    assert np.count_nonzero(edges[:198])==0 and np.count_nonzero(edges[202+width:])==0


@pytest.mark.parametrize('curve',[0.,10.,20.])
def test_real_white_curves_remain_when_central_ground_has_glare(curve):
    image=track(curve=curve)
    cv2.ellipse(image,(240,245),(28,65),0,0,360,(255,255,255),-1)
    result=ConeRoadContext(settings(200),turn_aware=True).inspect(image,np.zeros((360,480),np.uint8))
    assert result['valid'] and result['white_lines_detected']
    left=np.array(result['left_inner']);right=np.array(result['right_inner'])
    assert np.median(left[:,0])<190 and np.median(right[:,0])>300


@pytest.mark.parametrize('kind',['all_white','washed_bay','missing_edge','blue_block'])
def test_overexposure_or_obstacle_never_fabricates_a_clear_parking_bay(kind):
    image,config=raw_frame()
    if kind=='all_white':image[:]=255
    elif kind=='washed_bay':cv2.rectangle(image,(110,200),(240,253),(255,255,255),-1)
    elif kind=='missing_edge':cv2.line(image,(110,245),(160,213),(255,255,255),42)
    else:cv2.rectangle(image,(153,217),(200,237),(230,90,40),-1)
    obs,_=detect_parking(image,config)
    assert not obs['entry_candidates']
    assert not any(s['availability']=='clear' for s in obs['slots'])


def test_current_daylight_clip_is_explicitly_unknown_instead_of_free_ground():
    folder=Path(__file__).parent/'fixtures/sunlight-parking-20261110'
    frames=list(map(json.loads,(folder/'frames.jsonl').read_text().splitlines()))
    config=json.loads((folder/'settings.json').read_text(encoding='utf8'))['parking']
    with (folder/'frames.mjpg').open('rb') as stream:
        for row in frames[::15]:
            stream.seek(row['offset'])
            image=cv2.imdecode(np.frombuffer(stream.read(row['length']),np.uint8),1)
            obs,_=detect_parking(image,config)
            assert obs['diagnostics']['broad_glare_pixels']>0
            assert not obs['perception_verified']
            assert not any(s['occupancy_verified'] for s in obs['slots'])


def test_daylight_recovers_actual_outer_white_lines_but_not_complete_parking():
    folder=Path(__file__).parent/'fixtures/sunlight-parking-20261110'
    frames=list(map(json.loads,(folder/'frames.jsonl').read_text().splitlines()))
    config=json.loads((folder/'settings.json').read_text(encoding='utf8'))
    road=ConeRoadContext(config['cone'],'secondary',turn_aware=True);observed=0
    with (folder/'frames.mjpg').open('rb') as stream:
        for row in frames:
            stream.seek(row['offset']);image=cv2.imdecode(np.frombuffer(stream.read(row['length']),np.uint8),1)
            mask=blue_board_mask(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),config['parking'])
            result=road.inspect(image,mask,frame_id=row['frame_id'],captured_s=row['captured_s'])
            if result['valid']:
                observed+=1
                assert result['boundary_diagnostics']['illumination_retry']
                assert result['boundary_diagnostics']['contrast_original_ground_fraction']>=.30
                # Ground-truth outer stripes in this stationary recording,
                # not the bright central reflection or board sides.
                assert max(p[0] for p in result['left_inner'])<140
                assert min(p[0] for p in result['right_inner'])>370
            assert not result['metric_boundary_verified']
    assert observed>=40


@pytest.mark.parametrize('case',['plain_grey','dark_panel','weak_background_stripes'])
def test_contrast_retry_cannot_invent_track_without_current_ground(case):
    image=np.full((360,480,3),110,np.uint8)
    if case=='dark_panel':cv2.rectangle(image,(80,150),(400,330),(65,65,65),-1)
    elif case=='weak_background_stripes':
        for a,b in [((100,300),(190,150)),((380,300),(290,150))]:cv2.line(image,a,b,(155,155,155),8)
    result=ConeRoadContext(settings(200),turn_aware=True).inspect(image,np.zeros((360,480),np.uint8))
    assert not result['valid']


def test_color_and_input_image_are_not_rewritten_by_glare_filter():
    image,config=raw_frame();original=image.copy()
    detect_parking(image,config)
    assert np.array_equal(image,original)


def test_broad_green_tinted_highlight_is_unknown_not_ground():
    hsv=np.full((360,480,3),(60,120,130),np.uint8)
    # The reflected patch still has a green hue and fits the old floor bands.
    # Its low saturation/high value must override that color-only evidence.
    hsv[180:300,100:380]=(60,45,240)
    grey=cv2.cvtColor(cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR),cv2.COLOR_BGR2GRAY)
    edges,ground=contrast_markings(hsv,grey,{'surface':'indoor_green'})
    assert np.count_nonzero(ground[200:280,130:350])==0
    assert np.count_nonzero(edges[200:280,130:350])==0
    assert np.all(ground[310:340,130:350]>0)
