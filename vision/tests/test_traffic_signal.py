from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.traffic_signal import TrafficSignalDetector,SignalStability,TrafficLightStopLogic,traffic_light_intent,load_profile


def scene(active=None,white=False,missing_halo=False):
    im=np.full((150,360,3),25,np.uint8)
    for i,(x,color) in enumerate([(70,(0,0,70)),(180,(0,70,70)),(290,(0,70,0))]):
        on=i==active
        if on:color=tuple(255 if v else 0 for v in color)
        cv2.circle(im,(x,75),27,color,-1)
        if on and white:cv2.circle(im,(x,75),27 if missing_halo else 19,(255,255,255),-1)
    return im


@pytest.mark.parametrize('active,state',[(None,'off'),(0,'red'),(1,'yellow'),(2,'green')])
@pytest.mark.parametrize('white',[False,True])
def test_lamp_positions_and_emission_are_both_required(active,state,white):
    result=TrafficSignalDetector().detect(scene(active,white))
    assert result['state']==state and not result['hardware_output']


def test_white_patch_and_partial_green_rim_do_not_allow_green():
    detector=TrafficSignalDetector()
    im=scene(2,True,True)
    assert detector.detect(im)['state']!='green'
    cv2.ellipse(im,(290,75),(27,27),0,0,180,(0,255,0),4)
    assert detector.detect(im)['state']!='green'


def test_bright_colored_plastic_without_strong_emission_remains_unknown():
    im=np.full((150,360,3),25,np.uint8)
    for x,color in [(70,(0,0,190)),(180,(0,240,240)),(290,(0,175,0))]:
        cv2.circle(im,(x,75),27,color,-1)
    assert TrafficSignalDetector().detect(im)['state'] not in ('red','yellow','green')


@pytest.mark.parametrize('name',['traffic-background-primary.jpg','crosswalk-background-primary.jpg','blue-board-close.jpg'])
def test_existing_real_backgrounds_do_not_create_green(name):
    image=cv2.imdecode(np.frombuffer((Path(__file__).parent/'fixtures'/name).read_bytes(),np.uint8),1)
    assert TrafficSignalDetector().detect(image)['state']!='green'


def green(box=None):
    return {'state':'green','fixture_detected':True,'bbox_xyxy':box or [10,10,180,60]}


def test_green_needs_three_fresh_frames_and_elapsed_confirmation():
    s=SignalStability()
    assert not s.update(green(),1,0,0)['green_confirmed']
    assert not s.update(green(),2,.3,.3)['green_confirmed']
    d=s.update(green(),3,.61,.61)
    assert d['green_confirmed']
    assert not s.current(2)['green_confirmed']
    assert traffic_light_intent(d,camera='primary',in_stop_zone=True,stopped_verified=True,now_s=2)['intent']=='hold'


@pytest.mark.parametrize('state',['red','yellow','off','unknown'])
def test_any_non_green_cancels_green_immediately(state):
    s=SignalStability()
    for i,t in enumerate([0,.3,.61]):s.update(green(),i,t,t)
    assert not s.update({'state':state,'fixture_detected':True,'bbox_xyxy':[10,10,180,60]},4,.7,.7)['green_confirmed']


def test_duplicate_reordered_and_switched_fixture_frames_reset_green():
    s=SignalStability();s.update(green(),2,0,0)
    assert not s.update(green(),2,.5,.5)['green_confirmed']
    s.update(green(),3,.6,.6)
    assert s.update(green(),1,.7,.7)['reason']=='duplicate_or_reversed_frame'
    s=SignalStability();s.update(green(),1,0,0);s.update(green(),2,.3,.3)
    assert not s.update(green([200,100,320,140]),3,.7,.7)['green_confirmed']


def test_rule_adapter_never_infers_green_from_timeout_or_unverified_zone():
    s=SignalStability()
    for i,t in enumerate([0,.3,.61]):d=s.update(green(),i,t,t)
    assert traffic_light_intent(d,camera='primary',now_s=.7)['intent']=='hold'
    assert traffic_light_intent(d,camera='secondary',in_stop_zone=True,stopped_verified=True,now_s=.7)['intent']=='hold'
    assert traffic_light_intent(d,camera='primary',in_stop_zone=True,stopped_verified=True,red_seen_after_entry=True,now_s=.7)['intent']=='may_continue'
    assert traffic_light_intent({'state':'red','green_confirmed':False},camera='primary',in_stop_zone=True,stopped_verified=True,now_s=10)['intent']=='hold'


def test_invalid_green_geometry_and_nan_clock_are_unknown():
    s=SignalStability()
    assert not s.update(green([0,0,float('nan'),10]),1,0,0)['green_confirmed']
    assert not s.update(green(),2,float('nan'),1)['green_confirmed']


def test_zone_cycle_requires_observed_red_then_fresh_green_and_stationary_feedback():
    logic=TrafficLightStopLogic()
    for i,t in enumerate([0,.3,.61]):
        result=logic.update(green(),camera='primary',frame_id=i,captured_s=t,now_s=t,in_stop_zone=True,stopped_verified=True)
    assert result['intent']=='hold'  # Initial green before the sensor cycle is not release.
    red=green()|{'state':'red'}
    logic.update(red,camera='primary',frame_id=4,captured_s=.7,now_s=.7,in_stop_zone=True,stopped_verified=True)
    for i,t in enumerate([.8,1.1,1.41],5):
        result=logic.update(green(),camera='primary',frame_id=i,captured_s=t,now_s=t,in_stop_zone=True,stopped_verified=False)
    assert result['intent']=='hold'
    result=logic.update(green(),camera='primary',frame_id=8,captured_s=1.5,now_s=1.5,in_stop_zone=True,stopped_verified=True)
    assert result['intent']=='may_continue' and not result['hardware_output']
    assert logic.update(green(),camera='primary',frame_id=9,captured_s=1.6,now_s=3,in_stop_zone=True,stopped_verified=True)['intent']=='hold'
