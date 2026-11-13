"""Regressions from the four supervised runs; no hardware or remote access."""
import copy
from pathlib import Path
import threading
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from test_cone_latest_recovery import settings,nav,geometry
from carvision.cone_latest_engine import cone_image_guidance as image_guidance
from carvision.cone_latest_engine.cone_continuous import ContinuousConeController
from carvision.cone_latest_engine.cone_vision import BlueConeDetector,iou
from carvision.cone_latest_engine.cone_low_speed import DualLowSpeedNavigator,PROFILE
from carvision.cone_latest_engine.cone_observer import ConeCamera
from carvision.cone_latest_engine.cone_execution import ConeRun
from carvision.cone_command import ConeCommandSender

FIXTURES=Path(__file__).parent/'fixtures/cone-regression-20261107'


@pytest.mark.parametrize('name,cone,board',[
    ('near-cone-and-board.jpg',[128,272,165,318],[265,155,313,197]),
    ('far-cone-and-board.jpg',[300,229,319,255],[257,153,304,194])])
def test_recorded_blue_board_is_not_a_cone(settings,name,cone,board):
    image=cv2.imdecode(np.frombuffer((FIXTURES/name).read_bytes(),np.uint8),1)
    rows,_=BlueConeDetector(settings).detect(image)
    accepted=[r for r in rows if r['kind']=='cone_candidate']
    assert any(iou(r['bbox_xyxy'],cone)>.5 for r in accepted)
    assert not any(iou(r['bbox_xyxy'],board)>.2 for r in accepted)


@pytest.mark.parametrize('age',[.263,.35,.49])
def test_near_cone_survives_the_old_ground_roi_and_camera_age(settings,age):
    now=[0.];jpg=(FIXTURES/'near-cone-and-board.jpg').read_bytes()
    state=SimpleNamespace(condition=threading.Condition(),state='running',raw_jpeg=jpg,
                          raw_frame_id=0,raw_received_ms=0.)
    reader=ConeCamera({'secondary':state},settings,clock=lambda:now[0])
    reader.navigator=DualLowSpeedNavigator(settings)
    for i in range(6):
        captured=i*.15;now[0]=captured+age
        state.raw_frame_id=i+1;state.raw_received_ms=captured*1000
        sample=reader.read()
    row=sample['navigation']
    cones=[c for c in row['cones'] if not c.get('provisional')]
    assert len(cones)==1 and cones[0]['y_fraction']>.8 and cones[0]['near']
    assert 'fraction' not in cones[0]  # No invented extrapolated ground coordinate.
    assert row['ready'] and sample['cameras']['secondary']['lane_valid']
    assert now[0]-row['captured_s']==pytest.approx(age)


def test_diagnostic_ground_pose_cannot_switch_the_driver(settings,monkeypatch):
    monkeypatch.setattr(image_guidance,'image_lane_geometry',lambda n,*a,**k:geometry(n['captured_s'],.01,.5))
    base={**nav(1.),'experimental':True,'ready':True,'local_course':{'captured_s':1.,'lane':[]}}
    other=copy.deepcopy(base)
    other['local_course'].update(ready=True,lane=[{'x':-1.2,'heading':.9,'captured_s':1.,'camera':'secondary'}])
    a=ContinuousConeController(settings,1.,initialized=True)
    b=ContinuousConeController(settings,1.,initialized=True)
    assert a.step(base,1.)==b.step(other,1.)
    assert a.progress()['steering_policy']==b.progress()['steering_policy']=='continuous_image_guidance'
    assert a.progress()['image_guidance']['lane_margin']
    assert b.progress()['estimated_pose'] is None


def test_aligned_tail_never_replays_old_lateral_goal(settings,monkeypatch):
    guide=image_guidance.ImageConeGuidance(settings)
    guide.corridor_memory={'camera':'secondary','goal':.2,'side':'left','target_id':'old',
        'captured_s':1.,'remaining_m':1.,'motion_samples':0,'estimated_limit_s':6.}
    monkeypatch.setattr(image_guidance,'image_lane_geometry',lambda n,*a,**k:geometry(n['captured_s'],.005,.6))
    for t in (1.,1.1,1.2,1.3,1.4):
        row={**nav(t),'line_missing':False,'lane_bounds_px':[[100,300],[0,400]],'ego_fraction':.6}
        result=guide.update(row,t)
        assert abs(result['command'])<.1
        assert result['target_fraction']==pytest.approx(.6)
    # A cached corridor alone is neither a near-side observation nor passage.
    assert result['phase']=='hold_side'
    assert not result['return_guard']['near_side_observed']


class Output:
    run=True
    def __init__(self):self.sent=[];self.closed=False
    def send(self,intent):self.sent.append(copy.deepcopy(intent));return {}
    def close(self):self.closed=True


def test_263ms_is_accepted_but_repeat_frame_cannot_extend_500ms(settings):
    clock=[1.263];output=Output();sender=ConeCommandSender(output,clock=lambda:clock[0],threaded=False)
    run=ConeRun();run.active=True;run.phase='driving';run.profile=PROFILE;run.s=settings
    run.lease_until=10.;run.actuator=sender
    run.controller=ContinuousConeController(settings,1.,initialized=True)
    row={**nav(1.),'experimental':True,'ready':True,'frame_id':7,'reason':'白线暂缺'}
    run.tick(row,clock[0])
    assert run.active and run.phase=='driving' and sender.deadline==pytest.approx(1.5)
    clock[0]=1.49;run.tick(row,clock[0])
    assert run.active and sender.deadline==pytest.approx(1.5)
    clock[0]=1.501;run.tick(row,clock[0])
    assert not run.active and output.closed and run.phase=='fault'
    assert '500 ms' in run.reason and '白线暂缺' not in run.reason


def test_expired_frame_never_opens_actuator(settings):
    run=ConeRun(actuator_factory=lambda _:pytest.fail('expired image must not acquire output'))
    run.active=True;run.profile=PROFILE;run.s=settings;run.prepared_s=0.;run.lease_until=20.
    run.tick({**nav(1.),'experimental':True,'ready':True},6.)
    assert not run.active and run.actuator is None
