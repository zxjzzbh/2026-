"""Real uploaded mask regression and missing-boundary rejection; no motion."""
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.cone_planning import load_settings
from carvision.cone_road import ConeRoadContext
from carvision.cone_boundaries import adaptive_inner_edges

ROOT=Path(__file__).parents[1]


def photo():
    return cv2.imdecode(np.frombuffer((ROOT/'tests/fixtures/cone-white-mask-20261012.png').read_bytes(),np.uint8),cv2.IMREAD_COLOR)


@pytest.fixture
def settings():
    s=load_settings(ROOT/'configs/cone-trial.json')
    s['scene']['surface']='indoor_white_line_track'
    return s


def test_uploaded_clear_mask_fails_old_band_but_has_observed_adaptive_edges(settings):
    image=photo()
    assert image is not None
    context=ConeRoadContext(settings)
    before,_=context.lane.detect(image)
    assert not before.valid and before.reason=='insufficient_boundary_pairs'
    result=context.inspect(image,np.zeros(image.shape[:2],np.uint8))
    assert result['valid'] and result['white_lines_detected']
    d=result['boundary_diagnostics']
    assert d['detector']=='adaptive_paired_rows' and d['support_rows']>=20
    assert d['observed_y_range'][1]<image.shape[0]*.65
    # The central upright mark must not become the right track boundary.
    for l,r in zip(result['left_inner'],result['right_inner']):
        assert l[0]<image.shape[1]*.4 and r[0]>image.shape[1]*.6
        assert l[1]==r[1]


@pytest.mark.parametrize('case',['black','white','single','crossbar','wide_glare','flecks'])
def test_adaptive_fallback_does_not_invent_missing_boundaries(settings,case):
    image=np.full((360,480,3),(35,100,35),np.uint8)
    if case=='black':image[:]=0
    if case=='white':image[:]=255
    if case=='single':cv2.line(image,(30,320),(200,110),(255,255,255),9)
    if case=='crossbar':
        cv2.line(image,(20,210),(460,210),(255,255,255),12)
        cv2.line(image,(240,210),(240,120),(255,255,255),10)
    if case=='wide_glare':image[:,:170]=255;image[:,310:]=255
    if case=='flecks':
        for x,y in np.random.default_rng(42).integers([0,100],[480,360],(80,2)):
            cv2.circle(image,(int(x),int(y)),2,(255,255,255),-1)
    result=ConeRoadContext(settings).inspect(image,np.zeros(image.shape[:2],np.uint8))
    assert not result['valid'] and result['left_inner']==result['right_inner']==[]


def test_adaptive_geometry_does_not_bypass_red_surface_requirement(settings):
    settings['scene']['surface']='red_running_track'
    image=photo()
    result=ConeRoadContext(settings).inspect(image,np.zeros(image.shape[:2],np.uint8))
    assert result['white_lines_detected'] and not result['red_ground_supported']
    assert not result['valid'] and result['reason']=='red_ground_not_confirmed'


def test_dense_actual_inner_edges_remain_on_supported_pixels(settings):
    image=photo()
    config=ConeRoadContext(settings).lane.config
    left,right,mask,diagnostics=adaptive_inner_edges(image,np.zeros(image.shape[:2],np.uint8),config)
    assert len(left)==len(right)==diagnostics['support_rows']
    for l,r in zip(left,right):
        y=round(l[1]);row=(mask[y-1:y+2]>0).mean(0)>=.5
        assert row[round(l[0])-1] and not row[round(l[0])]
        assert row[round(r[0])+1] and not row[round(r[0])]


def test_live_green_track_glare_does_not_merge_into_left_white_boundary(settings):
    image=cv2.imdecode(np.frombuffer((ROOT/'tests/fixtures/cone-track-glare-secondary.jpg').read_bytes(),np.uint8),1)
    result=ConeRoadContext(settings).inspect(image,np.zeros(image.shape[:2],np.uint8))
    assert result['valid'],result['boundary_diagnostics']
    left=np.asarray(result['left_inner']);right=np.asarray(result['right_inner'])
    # Real tape flares outward below the crossbar. The green reflection in the
    # middle of the image must not supply an almost vertical "left boundary".
    assert np.polyfit(left[:,1],left[:,0],1)[0]<-1
    assert np.polyfit(right[:,1],right[:,0],1)[0]>1
    assert left[:,1].min()>145 and left[:,1].max()>225


def test_primary_cabinet_edges_above_track_do_not_become_white_line_danger(settings):
    image=cv2.imdecode(np.frombuffer((ROOT/'tests/fixtures/cone-track-glare-primary.jpg').read_bytes(),np.uint8),1)
    result=ConeRoadContext(settings,'primary').inspect(image,np.zeros(image.shape[:2],np.uint8))
    for side in ('left_inner','right_inner'):
        assert all(p[1]>=image.shape[0]*.60 for p in result[side])
