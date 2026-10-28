"""One final-mask road fit per real view; no hardware or camera is opened."""
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.cone_observer import ConeCamera
from carvision.race_flow import load_bundle
from carvision.race_flow_camera import RaceFlowCamera


ROOT=Path(__file__).resolve().parents[2]


@pytest.fixture
def bundle():return load_bundle(ROOT)


def caches():
    image=np.full((360,480,3),40,np.uint8)
    cv2.line(image,(60,350),(200,140),(230,230,230),7)
    cv2.line(image,(420,350),(280,140),(230,230,230),7)
    ok,jpeg=cv2.imencode('.jpg',image);assert ok
    return {camera:SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpeg.tobytes(),
        raw_frame_id=1,raw_received_ms=1000.,state='ok') for camera in ('primary','secondary')}


class FrameNavigator:
    def __init__(self):self.draws=0
    def update(self,*args):return None
    def draw(self,image):self.draws+=1;return image.copy()


class NavigationSpy:
    def __init__(self):self.calls=[]
    def update_views(self,images,rows,now):
        self.calls.append({camera:(row['frame_id'],row['captured_s']) for camera,row in rows.items()})
        source='secondary' if 'secondary' in rows else next(iter(rows))
        return {'source_camera':source,'captured_s':rows[source]['captured_s'],
            'frame_id':rows[source]['frame_id'],'line_state':'paired','cones':[]}
    def draw_camera(self,camera,image):return image.copy()


class ZebraSpy:
    def __init__(self,x):self.x=x;self.calls=[]
    def detect(self,image,ms):
        self.calls.append((image,ms))
        return {'stripes_xywh':[(self.x,200,70,8)]},image


def fit_spies(reader,monkeypatch):
    fits={camera:[] for camera in reader.roads};enriched={camera:[] for camera in reader.roads}
    for camera,road in reader.roads.items():
        original_fit=road.inspect;original_enrich=road.enrich
        def inspect(image,mask,*,camera=camera,original_fit=original_fit,**metadata):
            context=original_fit(image,mask,**metadata)
            fits[camera].append({'image':image,'mask':mask.copy(),'context':context,'metadata':metadata})
            return context
        def enrich(detections,image,context,*,camera=camera,original_enrich=original_enrich):
            assert context is fits[camera][-1]['context']
            enriched[camera].append(context)
            return original_enrich(detections,image,context)
        monkeypatch.setattr(road,'inspect',inspect);monkeypatch.setattr(road,'enrich',enrich)
    return fits,enriched


def encoder_spy(monkeypatch):
    calls=[];original=cv2.imencode
    def encode(*args,**kwargs):calls.append(args[0]);return original(*args,**kwargs)
    monkeypatch.setattr(cv2,'imencode',encode)
    return calls


def isolated_race(states,bundle,clock,monkeypatch):
    reader=RaceFlowCamera(states,bundle,ROOT,clock)
    reader.navigator=NavigationSpy()
    reader.zebra={'primary':ZebraSpy(15),'secondary':ZebraSpy(250)}
    monkeypatch.setattr(reader.crosswalk,'read',lambda now:None)
    monkeypatch.setattr(reader.traffic,'read',lambda now:None)
    reader.traffic.preview=b'traffic-final-preview'
    return reader


def test_cone_filter_runs_once_before_fit_and_enrich_for_the_same_view(bundle,monkeypatch):
    states=caches();reader=ConeCamera(states,bundle['cones'],lambda:1.,turn_aware=True)
    reader.navigator=FrameNavigator();fits,enriched=fit_spies(reader,monkeypatch);seen=[]
    def filtering(camera,image,fid,captured_s,mask):
        seen.append((camera,image,fid,captured_s));result=mask.copy();result[200:208,25:95]=255
        return result
    calls=encoder_spy(monkeypatch)
    first=reader.read(road_mask_filter=filtering,render=False)
    assert first['sequence']==1 and first['captured_s']==1.
    assert len(seen)==2 and all((fid,t)==(1,1.) for _,_,fid,t in seen)
    for camera,image,_,_ in seen:
        assert image is reader.images[camera]
        assert len(fits[camera])==len(enriched[camera])==1
        assert fits[camera][0]['metadata']=={'frame_id':1,'captured_s':1.}
        assert np.all(fits[camera][0]['mask'][200:208,25:95]==255)
    assert calls==[] and reader.navigator.draws==0 and not reader.views and reader.preview is None
    assert reader.read(road_mask_filter=filtering,render=False) is None
    assert len(seen)==2 and reader.sequence==1


def test_default_debug_read_still_renders_one_combined_preview(bundle,monkeypatch):
    reader=ConeCamera(caches(),bundle['cones'],lambda:1.)
    reader.navigator=FrameNavigator();fits,_=fit_spies(reader,monkeypatch);calls=encoder_spy(monkeypatch)
    result=reader.read()
    assert result['sequence']==1 and calls==['.jpg'] and reader.navigator.draws==1
    assert set(reader.views)=={'primary','secondary'}
    assert cv2.imdecode(np.frombuffer(reader.preview,np.uint8),1).shape==(360,960,3)
    assert all(len(items)==1 for items in fits.values())
    assert reader.read() is None and calls==['.jpg']


@pytest.mark.parametrize('stage,encodes',[('crosswalk',1),('traffic',0),('cones',1)])
def test_race_has_one_final_fit_per_view_and_no_intermediate_jpeg(bundle,monkeypatch,stage,encodes):
    states=caches();reader=isolated_race(states,bundle,lambda:1.,monkeypatch)
    fits,enriched=fit_spies(reader.cones,monkeypatch);calls=encoder_spy(monkeypatch)
    result=reader.read(stage,1.)
    assert result is not None and reader.cones.turn_aware
    assert all(road.turn_aware for road in reader.cones.roads.values())
    assert all(len(items)==1 for items in fits.values())
    assert all(len(items)==1 for items in enriched.values())
    assert len(calls)==encodes and reader.cones.preview is None
    if stage in ('crosswalk','traffic'):
        for camera,zebra in reader.zebra.items():
            assert len(zebra.calls)==1
            assert np.all(fits[camera][0]['mask'][200:208,zebra.x:zebra.x+70]==255)
    if stage=='traffic':assert reader.preview==b'traffic-final-preview'
    assert reader.read(stage,1.) is None and len(calls)==encodes


def test_mask_phase_change_waits_for_each_real_view_instead_of_refitting_cache(bundle,monkeypatch):
    states=caches();now=[1.];reader=isolated_race(states,bundle,lambda:now[0],monkeypatch)
    fits,_=fit_spies(reader.cones,monkeypatch)
    first=reader.read('cones',1.);assert first['sequence']==1
    assert set(reader.navigator.calls[-1])=={'primary','secondary'}
    roads=dict(reader.cones.roads)
    assert reader.read('traffic',1.) is None
    assert all(len(items)==1 for items in fits.values())
    now[0]=1.1;states['primary'].raw_frame_id=2;states['primary'].raw_received_ms=1100.
    reader.read('traffic',1.1)
    assert reader.navigator.calls[-1]=={'primary':(2,1.1)}
    assert len(fits['primary'])==2 and len(fits['secondary'])==1
    assert reader.cones.rows['secondary']['frame_id']==1
    assert reader.cones.rows['secondary']['captured_s']==1.
    now[0]=1.2;states['secondary'].raw_frame_id=2;states['secondary'].raw_received_ms=1200.
    reader.read('traffic',1.2)
    assert reader.navigator.calls[-1]=={'primary':(2,1.1),'secondary':(2,1.2)}
    assert all(len(items)==2 for items in fits.values())
    assert all(len(zebra.calls)==1 for zebra in reader.zebra.values())
    now[0]=1.3;states['secondary'].raw_frame_id=3;states['secondary'].raw_received_ms=1300.
    reader.read('cones',1.3)
    assert reader.navigator.calls[-1]=={'secondary':(3,1.3)}
    assert len(fits['primary'])==2 and len(fits['secondary'])==3
    assert not np.any(fits['secondary'][-1]['mask'][200:208,250:320])
    assert all(reader.cones.roads[camera] is road for camera,road in roads.items())
    assert all(reader.cones.trackers[camera].near_field_recovery for camera in roads)


def test_mask_cache_binds_camera_frame_and_exact_image_reference(bundle,monkeypatch):
    reader=isolated_race(caches(),bundle,lambda:1.,monkeypatch);reader.mask_zebra=True
    first=np.zeros((360,480,3),np.uint8);second=np.ones_like(first);base=np.zeros((360,480),np.uint8)
    reader._road_mask('primary',first,7,1.,base)
    reader._road_mask('primary',first,7,1.,base)
    assert len(reader.zebra['primary'].calls)==1
    reader._road_mask('primary',second,7,1.1,base)
    assert len(reader.zebra['primary'].calls)==2 and reader.zebra_images['primary'] is second
    mask=reader._road_mask('secondary',second,7,1.1,base)
    assert len(reader.zebra['secondary'].calls)==1
    assert np.all(mask[200:208,250:320]) and not np.any(mask[200:208,15:85])
