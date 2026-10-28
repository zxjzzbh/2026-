"""Offline light-gate chronology; no physical gate, distance or stop is verified.

An initially unlit/unknown lamp may remain so while approaching the assumed
gate. We model its later red appearance, not a connected sensor. Neither the
assumed one-metre start nor a nominal 25-cm braking distance is measured here.
"""
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.race_flow import RaceFlowController, load_bundle
from carvision.race_flow_console import RaceFlowDrive
from carvision.shared_executor import BorrowedActuator
from carvision.traffic_driving import TrafficDrivingController, load_settings
from test_race_flow import Frames
from test_shared_executor import CLIENT, Worker, rig

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT/'vision/configs/traffic-driving.json'


class LightRun:
    def __init__(self, **settings):
        self.settings = copy.deepcopy(load_settings(CONFIG))
        self.settings.update(settings)
        self.controller = TrafficDrivingController(self.settings)
        self.t = 0.
        self.frame = 0
        self.rows = []

    def step(self, color='unknown', moving=True, dt=.1, camera=None):
        self.t = round(self.t+dt,8)
        self.frame += 1
        observations = {}
        for name in ('primary','secondary'):
            state = color if camera is None or camera==name else 'unknown'
            observations[name] = {'camera_id':name,'frame_id':self.frame,'captured_s':self.t,
                'image_size':[480,360], 'signal':{'state':state,'fixture_detected':state!='unknown',
                    'bbox_xyxy':[130,60,310,110] if state!='unknown' else None}}
        sample = {'fresh':True,'camera_id':'dual','frame_id':self.frame,'captured_s':self.t,
                  'image_size':[480,360], 'camera_observations':observations,
                  'motion':{'valid':moving is not None,'moving':moving,'captured_s':self.t}}
        output = self.controller.step(sample,self.t)
        self.rows.append((self.t,output))
        return output

    def approach(self, color='unknown'):
        for _ in range(40):
            output = self.step(color)
            assert output['phase']!='fault',output
            if output['phase']=='approach':return output
        pytest.fail(f'never reached approach: {self.rows[-1]}')

    def until(self, phase, color='red', moving=False, limit=45):
        for _ in range(limit):
            output = self.step(color,moving)
            if output['phase']==phase:return output
            assert output['phase']!='fault' or phase=='fault',output
        pytest.fail(f'never reached {phase}: {self.rows[-1]}')


@pytest.mark.parametrize('color',['off','unknown'])
def test_untriggered_lamp_can_be_approached_past_old_acquire_and_loss_deadlines(color):
    run = LightRun()
    assert run.approach(color)['action']=='search'
    # 1.2 seconds is deliberately longer than the old 300/150-ms guards.
    for _ in range(12):
        output = run.step(color)
        assert output['phase']=='approach' and output['action']=='search'
        assert output['esc_us']==1570 and not output['red_seen_this_round']
        assert output['external_sensor_connected'] is False
    assert not any(o['phase']=='wait_green' for _,o in run.rows)


@pytest.mark.parametrize('camera',['primary','secondary'])
def test_first_trustworthy_red_after_untriggered_approach_brakes_in_that_batch(camera):
    run = LightRun()
    run.approach('off')
    for _ in range(12):assert run.step('unknown')['action']=='search'
    gate_red_frame = run.frame+1  # Merely the first modelled red image.
    output = run.step('red',camera=camera)
    assert run.frame==gate_red_frame
    assert output['action']=='brake' and output['esc_us']==1400
    assert output['brake_pulse_s']==.35 and not output['red_seen_this_round']
    assert any(e['event']=='brake_requested' for e in output['events'])
    assert output['phase']!='wait_green' and output['physical_stop_verified'] is False


def test_wait_green_requires_both_formal_red_and_continuous_stationary_evidence():
    run = LightRun()
    run.approach('off')
    first = run.step('red',True)
    assert first['action']=='brake'
    for _ in range(10):
        output = run.step('red',True)
        assert output['phase']!='wait_green' and output['esc_us']<=1500
    assert output['red_seen_this_round']
    stopped = run.until('wait_green','red',False,limit=18)
    assert stopped['red_seen_this_round'] and stopped['stationary_estimated']
    assert stopped['esc_us']==1500


def test_a_single_red_then_only_green_cannot_fake_formal_red_or_resume():
    run = LightRun(max_verify_red_s=3.)
    run.approach('off')
    assert run.step('red',False)['action']=='brake'
    stop_at = run.t
    saw_verify = False
    while run.t-stop_at<3.2:
        output = run.step('green',False)
        saw_verify |= output['phase']=='verify_red'
        assert output['esc_us']<=1500 and output['phase'] not in ('wait_green','done')
        assert not output['red_seen_this_round']
        if output['phase']=='fault':break
    assert saw_verify and output['phase']=='fault'
    assert 3.<=run.t-stop_at<=3.1+1e-8
    for _ in range(5):
        output = run.step('red',False)
        assert output['phase']=='fault' and output['esc_us']==1500


@pytest.mark.parametrize('stop_case',['unknown_motion','approach_timeout'])
def test_no_red_failure_first_requests_a_finite_brake_then_latches_fault(stop_case):
    run = LightRun(max_approach_s=.5 if stop_case=='approach_timeout' else 5.)
    run.approach('unknown')
    if stop_case=='approach_timeout':
        # Arming already lasted >3s, but only moving-approach time counts.
        assert run.step('unknown',True)['action']=='search'
        for _ in range(6):
            output = run.step('unknown',True)
            if output['action']=='brake':break
    else:
        output = run.step('unknown',None)
    assert output['action']=='brake' and output['esc_us']==1400
    assert output['phase']!='fault' and not output['red_seen_this_round']
    assert output['brake_pulse_s']==.35
    first_stop = run.t
    for _ in range(40):
        output = run.step('unknown',None if stop_case=='unknown_motion' else False)
        assert output['esc_us']<=1500 and output['phase'] not in ('wait_green','done')
        if output['phase']=='fault':break
    assert output['phase']=='fault' and run.t-first_stop>=.35
    for _ in range(5):
        output = run.step('green',False)
        assert output['phase']=='fault' and output['esc_us']==1500


def test_red_verification_deadline_stays_anchored_to_first_stop_after_rebraking():
    run = LightRun(max_verify_red_s=3.)
    run.approach('off')
    assert run.step('red',False)['action']=='brake'
    first_stop = run.t
    run.until('verify_red','off',False,limit=16)
    # Camera-motion estimates can cause more pulses; those do not buy a new
    # three-second formal-red verification window.
    for index in range(40):
        output = run.step('off',index in (0,10))
        assert output['esc_us']<=1500 and not output['red_seen_this_round']
        if output['phase']=='fault':break
    assert output['phase']=='fault' and run.t-first_stop<=3.1+1e-8


def test_rebrake_at_2_9_seconds_finishes_its_finite_pulse_without_extending_red_deadline():
    run = LightRun(max_verify_red_s=3.)
    run.approach('off')
    assert run.step('red',False)['action']=='brake'
    first_stop = run.t
    deadline = run.rows[-1][1]['red_verification_deadline_s']
    run.until('verify_red','off',False,limit=16)
    while run.t-first_stop<2.8-1e-8:
        assert run.step('off',False)['phase']=='verify_red'
    output = run.step('off',True,dt=round(first_stop+2.9-run.t,8))
    assert output['action']=='brake' and output['red_verification_deadline_s']==deadline
    rebrake = run.t
    for offset in (.1,.2,.3,.349):
        output = run.step('red',False,dt=round(rebrake+offset-run.t,8))
        assert output['action']=='brake' and output['esc_us']==1400
        assert output['pending_fault_reason']
        assert output['red_verification_deadline_s']==deadline
        assert output['phase'] not in ('wait_green','done')
    output = run.step('red',False,dt=round(rebrake+.35-run.t,8))
    assert output['phase']=='fault' and output['esc_us']==1500
    for _ in range(25):
        output = run.step('green',False)
        assert output['phase']=='fault' and output['esc_us']==1500


def test_complete_traffic_gate_chronology_uses_shared_guard_then_original_cone_speed(rig):
    now,clock,backend,manual,owner,_ = rig
    bundle = load_bundle(ROOT)
    worker = Worker(owner)
    view = SimpleNamespace(borrow_actuator=lambda settings,client:BorrowedActuator(worker,settings,client,clock=clock))
    actuator = RaceFlowDrive._make_actuator(SimpleNamespace(drive=view,owner=CLIENT,live=False),bundle)
    controller = RaceFlowController(bundle,0.,initialized=True)
    controller._go('traffic',0.,'offline light-gate chronology')
    frames = Frames(controller)
    for _ in range(12):
        output = frames.step(color='off',moving=True)
        now[0] = frames.t
        actuator.send(output)
        assert output['action']=='traffic_approach' and owner.guard.esc==1570
        assert controller.stage=='traffic' and not controller.controller.red_seen
    output = frames.step(color='red',moving=True)
    now[0] = frames.t
    actuator.send(output)
    assert output['action']=='brake' and owner.guard.esc==1400
    assert owner.guard.brake_until==pytest.approx(frames.t+.35)
    for _ in range(25):
        output = frames.step(color='red',moving=False)
        now[0] = frames.t
        actuator.send(output)
        assert controller.stage=='traffic' and output['esc_us']<=1500
        if controller.controller.phase=='wait_green':break
    assert controller.controller.phase=='wait_green' and controller.controller.red_seen
    for _ in range(30):
        output = frames.step(color='green',moving=False)
        now[0] = frames.t
        actuator.send(output)
        if controller.stage=='cones':break
    assert controller.stage=='cones' and owner.guard.esc==output['esc_us']==1500
    for _ in range(2):
        output=frames.step(moving=True)
        now[0]=frames.t
        actuator.send(output)
    assert owner.guard.esc==output['esc_us']==1600
    assert not owner.guard.fault and bundle['cones']['driving']['forward_us']==1600
    assert bundle['brake']['pwm']['search_us']==1600
    actuator.close()
