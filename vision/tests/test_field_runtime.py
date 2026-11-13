"""Field regression: saved pixels, fake clocks and fake output only."""
import copy
import json
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.cone_command import ConeCommandSender
from carvision.motion_cache import GroundMotionCache
from carvision.race_flow import RaceFlowController, load_bundle
from carvision.traffic_tracking import TrafficApproachGuard
from carvision.traffic_signal import TrafficSignalDetector
from carvision.traffic_driving import approach_settings, load_settings
from carvision.cone_road import ConeRoadContext
from carvision.cone_vision import BlueConeDetector
from test_cone_command_cadence import FakeActuator, intent
from test_traffic_driving import Run

ROOT=Path(__file__).resolve().parents[2]
FIXTURES=Path(__file__).parent/'fixtures/field-20261112'


def test_prepared_neutral_renews_executor_without_old_pixels_but_page_still_expires():
    now=[0.];actuator=FakeActuator(lambda:now[0])
    sender=ConeCommandSender(actuator,clock=lambda:now[0],threaded=False)
    try:
        sender.send({'action':'neutral','esc_us':1500,'steering_us':1610,
                     'control_profile':'cone','observation_s':-10.,'page_deadline_s':3.})
        for i in range(1,60):
            now[0]=i*.05;sender.tick();actuator.guard.tick(now[0])
            assert sender.fault is None and actuator.guard.fault is None
        assert all(row['esc_us']==1500 for row in actuator.rows)
        now[0]=3.;sender.tick()
        assert '心跳' in sender.fault and actuator.rows[-1]['esc_us']==1500
    finally:sender.close()


def test_prepared_neutral_does_not_change_moving_frame_deadline():
    now=[0.];actuator=FakeActuator(lambda:now[0])
    sender=ConeCommandSender(actuator,clock=lambda:now[0],threaded=False)
    sender.send({**intent(),'action':'neutral','esc_us':1500,'steering_us':1610,'page_deadline_s':3.})
    now[0]=.1;sender.send(intent(.1))
    for i in range(1,10):now[0]=.1+i*.05;sender.tick()
    now[0]=.6;sender.tick()
    assert sender.fault and actuator.rows[-1]['esc_us']==1400
    sender.close()


def test_motion_cache_never_renews_a_duplicate_capture():
    now=[1.];pixels=np.random.default_rng(5).integers(0,255,(360,480,3),dtype=np.uint8)
    jpeg=cv2.imencode('.jpg',pixels)[1].tobytes()
    state=SimpleNamespace(condition=threading.Condition(),state='running',raw_jpeg=jpeg,raw_frame_id=1,raw_received_ms=1000.)
    cache=GroundMotionCache({'secondary':state},clock=lambda:now[0])
    # Avoid background scheduling in this exact timestamp regression.
    cache.thread=SimpleNamespace(join=lambda **kw:None)
    try:
        cache.sample_once();now[0]=1.1;state.raw_frame_id=2;state.raw_received_ms=1100.
        cache.sample_once();row=cache.snapshot()
        assert row['valid'] and row['moving'] is False
        now[0]=1.3;cache.sample_once();assert cache.snapshot()['captured_s']==1.1
        now[0]=1.36;assert not cache.snapshot()['valid'] and cache.snapshot()['moving'] is None
    finally:cache.close()


def race():
    controller=RaceFlowController(load_bundle(ROOT),0.,initialized=True)
    controller.last_drive=True;controller.last_seq=1
    return controller


def frame(seq,t,nav=True):
    result={'sequence':seq,'captured_s':t}
    if nav:result['navigation']={'captured_s':t}
    return result


def test_race_recovers_only_after_two_ordered_current_frames_and_brake_release():
    controller=race()
    assert controller.step(None,.1)['esc_us']==1400
    assert controller.step(frame(2,.2),.2)['esc_us']==1400
    assert controller.step(frame(2,.2),.5)['esc_us']==1500
    assert controller.vision_recovery_hits==1
    assert controller.step(frame(3,.6),.6)['esc_us']==1500
    assert controller.vision_recovery_started is None


@pytest.mark.parametrize('bad',[frame(1,.6),frame(4,.2),frame(4,.6,False),frame(4,9.)])
def test_race_repeated_old_missing_lane_or_future_frames_cannot_resume(bad):
    controller=race();controller.step(None,.1);controller.step(frame(2,.2),.2)
    assert controller.step(bad,.6)['esc_us']<=1500
    assert controller.vision_recovery_started is not None
    result=controller.step(None,1.61)
    assert result['phase']=='fault' and result['esc_us']==1500


def test_traffic_single_bad_motion_sample_brakes_then_recovers_without_subminimum_throttle():
    run=Run('off');run.until('approach',moving=True)
    assert run.step(moving=None)['phase']=='motion_recovery'
    assert run.rows[-1]['esc_us']==1400
    run.until('approach',moving=False,limit=12)
    assert run.step(moving=True)['esc_us']>=1570


def test_traffic_bad_motion_is_bounded_and_red_has_priority_during_recovery():
    run=Run('off');run.until('approach',moving=True);run.step(moving=None)
    run.until('fault',moving=None,limit=18)
    assert run.rows[-1]['esc_us']==1500
    run=Run('off');run.until('approach',moving=True);run.step(moving=None)
    assert run.step('red',moving=None)['phase']=='brake_pulse'
    assert run.rows[-1]['esc_us']==1400


def test_traffic_duplicate_motion_cannot_establish_stillness():
    run=Run('off');run.until('approach',moving=True);run.step(moving=None)
    for _ in range(4):run.step(moving=None)
    captured=run.t+.1
    for _ in range(4):
        row=run.step(motion={'valid':True,'moving':False,'captured_s':captured})
        assert row['phase']=='motion_recovery' and row['esc_us']<=1500


def read_image(name):return cv2.imdecode(np.frombuffer((FIXTURES/name).read_bytes(),np.uint8),1)


@pytest.mark.parametrize('name,camera',[
    ('early-primary.jpg','primary'),('late-primary.jpg','primary'),('late-secondary.jpg','secondary')])
def test_saved_field_white_edges_form_supported_pairs(name,camera):
    settings=json.loads((ROOT/'vision/tests/fixtures/configs/cone-latest-20261029-field-20261112.json').read_text(encoding='utf-8-sig'))
    settings['scene']['surface']='red_running_track'
    pixels=read_image(name);_,blue=BlueConeDetector(settings).detect(pixels)
    result=ConeRoadContext(settings,camera,turn_aware=True).inspect(pixels,blue,frame_id=1,captured_s=1.)
    assert result['valid'],result['reason']
    assert result['left_inner'] and result['right_inner']


def test_actual_bloomed_red_is_detected_and_scale_change_can_stop_but_cannot_release():
    detector=TrafficSignalDetector();pixels=read_image('red-secondary.jpg')
    assert detector.detect(pixels)['state']=='red'
    old=[261,135,356,157]
    result=detector.detect(pixels,reference_box=old)
    assert result['state']=='unknown' and result['stop_only_color']=='red'
    settings=approach_settings(load_settings(ROOT/'vision/configs/traffic-driving.json'))
    guard=TrafficApproachGuard(settings)
    guard.tracks['secondary'].box=old
    row={'camera_id':'secondary','image_size':[480,360],'frame_id':2,'captured_s':1.,'signal':result}
    filtered,evidence=guard.update({'secondary':row},1.01,approaching=True)
    assert evidence['stop_reason']=='first_trustworthy_red'
    assert filtered['secondary']['signal']['state']=='unknown'


@pytest.mark.parametrize('mode',['white_only','red_blob','wrong_neighbours'])
def test_isolated_bright_artifacts_do_not_become_red_or_green(mode):
    pixels=np.full((360,480,3),35,np.uint8)
    if mode!='white_only':cv2.circle(pixels,(120,100),20,(0,0,255),-1)
    cv2.circle(pixels,(120,100),14,(255,255,255),-1)
    if mode=='wrong_neighbours':
        cv2.circle(pixels,(210,160),18,(0,65,65),-1)
        cv2.circle(pixels,(300,220),18,(0,65,0),-1)
    assert TrafficSignalDetector().detect(pixels)['state']=='unknown'
