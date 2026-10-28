"""Vehicle motion and identity regressions; all samples/output are simulated."""
from pathlib import Path

import pytest

from carvision.race_flow import FlowTrafficController
from carvision.traffic_driving import load_settings, validate_settings


CONFIG = Path(__file__).parents[1]/'configs/traffic-driving.json'
BOX = [100,60,250,130]


class Drive:
    def __init__(self):
        self.c = FlowTrafficController(load_settings(CONFIG), 0., 30.)
        self.t = -.1
        self.fid = -1
        self.history = []

    def step(self, color='off', *, box=BOX, camera='secondary', dt=.1,
             moving=True, rows=None, **signal):
        self.t = round(self.t+dt,8)
        self.fid += 1
        if rows is None:
            rows = {camera:{'camera_id':camera,'frame_id':self.fid,'captured_s':self.t,
                    'image_size':[480,360], 'signal':{
                        'fixture_detected':box is not None,'bbox_xyxy':box,'state':color,**signal}}}
        sample = {'fresh':True, 'camera_id':'dual', 'frame_id':self.fid,
                  'captured_s':self.t, 'image_size':[480,360], 'camera_observations':rows,
                  'motion':{'valid':True,'moving':moving,'captured_s':self.t}}
        result = self.c.step(sample,self.t)
        self.history.append(result)
        return result


@pytest.mark.parametrize('camera',['primary','secondary'])
def test_first_trustworthy_red_brakes_in_the_same_decision_even_with_cold_votes(camera):
    d=Drive()
    result=d.step('red',camera=camera)
    assert result['action']=='brake' and result['esc_us']==1400
    assert result['approach_guard']['first_red']=={
        'camera':camera,'frame_id':0,'captured_s':0.,'decision_s':0.}
    assert result['brake_pulse_s']==.35
    assert not result['red_seen_this_round']
    assert result['red_sources_this_round']==[] and result['events'][0]['event']=='brake_requested'


def test_first_red_then_green_never_fakes_a_complete_red_green_cycle():
    d=Drive();d.step('red')
    for _ in range(50):
        result=d.step('green',moving=False)
        assert result['esc_us']<=1500 and not result['red_seen_this_round']
        assert not any(e['event']=='speak' for e in result['events'])
    assert result['phase']=='fault'
    assert any(r['phase']=='verify_red' for r in d.history)
    assert not any(r['phase']=='wait_green' for r in d.history)


def test_initial_missing_fixture_allows_time_bounded_low_speed_approach_to_the_gate():
    d=Drive()
    for _ in range(5):result=d.step('unknown',box=None)
    assert result['phase']=='approach' and not result['red_seen_this_round']
    assert not any(r['esc_us']>1570 for r in d.history)
    assert all(r['action']=='search' for r in d.history)


def test_unlit_fixture_loss_does_not_imply_a_triggered_red_or_replace_the_target():
    d=Drive();d.step();d.step()
    assert d.c.approach_evidence['any_locked']
    d.step('off',box=[300,200,350,215])
    result=d.step('off',box=[300,200,350,215])
    assert result['action']=='search'
    assert result['approach_guard']['stop_reason'] is None
    assert result['signal_state']=='unknown'
    assert result['red_sources_this_round']==[]


def test_one_lamp_view_disappearing_can_use_the_other_fresh_camera_and_motion():
    d=Drive();d.step(camera='primary');d.step(camera='primary')
    d.step(camera='secondary')
    result=d.step(camera='secondary')
    assert result['action']=='search'
    assert result['approach_guard']['sources']==[]


@pytest.mark.parametrize('box',[[8,60,158,130],[100,7,250,77],[322,60,472,130],[100,282,250,353]])
def test_unlit_fixture_at_an_image_edge_is_not_a_photocell_or_stop_position(box):
    d=Drive();d.step(box=box)
    result=d.step(box=box)
    assert result['action']=='search'
    assert result['approach_guard']['stop_reason'] is None
    assert result['signal_state']=='off' and not result['red_seen_this_round']


def test_unlit_fixture_approaching_the_image_edge_does_not_latch_a_false_red_stop():
    d=Drive()
    for box in ([100,70,250,140],[100,66,250,136],[100,62,250,132],[100,58,250,128],[100,54,250,124],[100,50,250,120],[100,46,250,116]):
        result=d.step(box=box)
        assert result['action']=='search'
    assert result['approach_guard']['stop_reason'] is None


def test_small_nonmonotonic_box_noise_keeps_low_speed_tracking():
    d=Drive()
    for y in (65,64,65,64,65,64,65):
        result=d.step(box=[100,y,250,y+70])
        assert result['action']=='search' and result['esc_us']==1570


def test_unrelated_green_fixture_cannot_use_the_old_red_source_qualification():
    d=Drive()
    for _ in range(25):result=d.step('red',moving=False)
    assert result['phase']=='wait_green' and result['red_seen_this_round']
    for _ in range(30):
        result=d.step('green',box=[300,200,350,215],moving=False)
        assert result['phase']=='wait_green' and result['events']==[]
        assert not result['stable']['green_confirmed']


@pytest.mark.parametrize('key,value',[
    ('approach_speed_limit_us',1600),('approach_speed_limit_us',True),
    ('max_verify_red_s',.5),('max_verify_red_s',6),
    ('max_verify_red_s',True),('max_verify_red_s',float('nan'))])
def test_invalid_approach_parameters_are_rejected(key,value):
    s=load_settings(CONFIG);s[key]=value
    with pytest.raises(ValueError):validate_settings(s)
