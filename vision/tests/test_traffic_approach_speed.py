"""Traffic-only speed cap through the real flow and shared fake-backend guard."""
import copy
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest

from carvision.cone_trial_output import ConePulseGuard
from carvision.race_flow import RaceFlowController, load_bundle, prepare_bundle
from carvision.race_flow_console import RaceFlowDrive
from carvision.shared_executor import BorrowedActuator
from carvision.traffic_driving import approach_settings, load_settings
from carvision.traffic_trial import TrafficTrialDrive
from test_race_flow import Frames
from test_shared_executor import CLIENT, Worker, rig

ROOT = Path(__file__).resolve().parents[2]


def traffic_bundle(pulse=1570):
    bundle = load_bundle(ROOT)
    bundle['traffic']['pwm']['search_us'] = bundle['traffic']['pwm']['creep_us'] = pulse
    return prepare_bundle(bundle)


@pytest.mark.parametrize('stored', [1501, 1560, 1570, 1575, 1600, 1605, 1625])
def test_traffic_effective_speed_cap_preserves_lower_saved_values_and_input(stored):
    settings = load_settings(ROOT/'vision/configs/traffic-driving.json')
    settings['pwm']['search_us'] = settings['pwm']['creep_us'] = stored
    original = copy.deepcopy(settings)
    capped = approach_settings(settings)
    assert capped['pwm']['search_us'] == capped['pwm']['creep_us'] == min(stored, 1570)
    assert settings == original
    assert capped['pwm']['brake_us'] == original['pwm']['brake_us'] == 1400
    assert capped['brake_pulse_s'] == original['brake_pulse_s'] == .35


@pytest.mark.parametrize('stored', [1560, 1605])
def test_saved_traffic_value_is_shared_by_trial_and_flow_without_changing_saved_setting(tmp_path, stored):
    shutil.copytree(ROOT/'vision/configs', tmp_path/'vision/configs')
    saved = tmp_path/'vision/configs/traffic-driving.local.json'
    saved.write_text(json.dumps({'forward_us':stored, 'red_confirm_percent':50, 'max_approach_s':10}), encoding='utf-8')
    configs = tmp_path/'vision/configs'
    trial = TrafficTrialDrive(SimpleNamespace(), {}, configs/'traffic-driving.json', configs/'traffic-signal.json', tmp_path)
    assert not trial.settings_error
    assert trial.settings['pwm']['search_us'] == stored
    assert trial.active_settings['pwm']['search_us'] == min(stored,1570)
    bundle = load_bundle(tmp_path)
    assert bundle['traffic']['pwm']['search_us'] == min(stored,1570)
    assert bundle['flow']['forward_us'] == 1600
    assert json.loads(saved.read_text(encoding='utf-8'))['forward_us'] == stored


def test_traffic_cap_does_not_change_other_task_settings():
    low, high = traffic_bundle(1560), traffic_bundle(1600)
    assert low['traffic']['pwm']['search_us'] == 1560
    assert high['traffic']['pwm']['search_us'] == 1570
    for task in ('flow','brake','cones','parking','departure'):
        assert low[task] == high[task]
    assert high['cones']['driving']['forward_us'] == 1600
    assert high['brake']['pwm']['search_us'] == 1600
    assert high['brake']['pwm']['creep_us'] == 1575


@pytest.mark.parametrize('stored', [1560,1590])
def test_repeated_prepare_is_idempotent_without_rewriting_raw_traffic_value(stored):
    raw = load_bundle(ROOT)
    raw['traffic']['pwm']['search_us'] = raw['traffic']['pwm']['creep_us'] = stored
    original = copy.deepcopy(raw)
    once = prepare_bundle(raw)
    twice = prepare_bundle(once)
    assert once == twice and raw == original
    assert twice['traffic']['pwm']['search_us'] == min(stored,1570)
    assert raw['traffic']['pwm']['search_us'] == stored


@pytest.mark.parametrize('stored', [1501, 1560, 1570, 1600])
def test_flow_first_traffic_observation_and_lane_guidance_keep_traffic_speed(stored):
    bundle = traffic_bundle(stored)
    controller = RaceFlowController(bundle,0.,initialized=True)
    controller._go('traffic',0.,'test')
    frames = Frames(controller)
    result = frames.step(color='off',moving=True)
    assert result['action'] == 'traffic_approach'
    assert result['esc_us'] == min(stored,1570)
    assert result['phase'] == 'traffic'


def test_bridge_into_traffic_uses_traffic_speed_before_first_lamp_observation():
    bundle = traffic_bundle(1560)
    controller = RaceFlowController(bundle,0.,initialized=True)
    nav = {'experimental':True, 'ready':True, 'captured_s':.1, 'frame_id':1,
           'ego_fraction':.5, 'half_envelope':.20, 'lane_bounds_px':[[100,380],[50,430]],
           'cones':[], 'uncertain':[], 'source_camera':'secondary'}
    result = controller._advance('traffic',.1,'test',{'navigation':nav},{})
    assert result['action'] == 'traffic_approach' and result['esc_us'] == 1560


def test_shared_executor_accepts_capped_traffic_then_original_cone_speed(rig):
    now,clock,backend,manual,owner,_ = rig
    worker = Worker(owner)
    view = SimpleNamespace(borrow_actuator=lambda settings,client:BorrowedActuator(worker,settings,client,clock=clock))
    bundle = traffic_bundle(1600)
    handle = RaceFlowDrive._make_actuator(SimpleNamespace(drive=view,owner=CLIENT,live=False),bundle)
    acquired = worker.calls[0][1]['settings']
    assert acquired['traffic_forward_us'] == 1570
    assert acquired['pwm']['search_us'] == 1600 and acquired['pwm']['creep_us'] == 1575
    controller = RaceFlowController(bundle,0.,initialized=True)
    controller._go('traffic',0.,'test')
    frames = Frames(controller)
    traffic = frames.step(color='off',moving=True)
    now[0] = frames.t
    handle.send(traffic)
    assert owner.guard.esc == 1570 and backend.writes[-1][2] == 1570
    controller._go('cones',frames.t,'test')
    for index in range(3):
        cones = frames.step(moving=True)
        now[0] = frames.t
        handle.send(cones)
        if index<2:assert cones['esc_us']==1500
    assert cones['esc_us'] == owner.guard.esc == 1600
    assert not owner.guard.fault and backend.writes[-1][2] == 1600
    handle.close()


def test_dedicated_traffic_whitelist_does_not_change_search_creep_or_parking_actions():
    settings = {'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':1600,'creep_us':1575,'brake_us':1400},
                'brake_pulse_s':.35,'brake_release_s':.1,'actuator_lease_s':.2,
                'traffic_forward_us':1570,'parking_forward_us':[1560]}
    original = copy.deepcopy(settings)
    writes = []
    guard = ConePulseGuard(settings,lambda *row:writes.append(row),mode='F/B')
    for seq,(action,pulse) in enumerate((('traffic_approach',1570),('search',1600),('creep',1575),('parking_1560',1560))):
        guard.accept({'action':action,'esc_us':pulse,'steering_us':1610,'sequence':seq,'sent_s':seq*.05},seq*.05)
        assert guard.esc == pulse and not guard.fault
    assert settings == original and guard.parking_pulses == (1560,)


@pytest.mark.parametrize('wrong', [1565, 1575, 1600, 1625, True])
def test_shared_worker_rejects_other_traffic_pulses_without_broadening_forward(rig,wrong):
    now,clock,backend,manual,owner,settings = rig
    settings['traffic_forward_us'] = 1570
    handle = BorrowedActuator(Worker(owner),settings,CLIENT,clock=clock)
    with pytest.raises((ValueError,RuntimeError)):
        handle.send({'action':'traffic_approach','esc_us':wrong,'steering_us':1610})
    assert not any(row[0]=='motion' and row[2] not in (1400,1570) for row in backend.writes)


@pytest.mark.parametrize('invalid', [1500, 1576, 1625, True, 1570.0])
def test_traffic_whitelist_rejects_invalid_setpoint_before_shared_handoff(invalid,rig):
    now,clock,backend,manual,owner,settings = rig
    settings['traffic_forward_us'] = invalid
    before = list(backend.writes)
    with pytest.raises(ValueError):
        BorrowedActuator(Worker(owner),settings,CLIENT,clock=clock)
    assert backend.writes == before and not owner.active


def test_shared_worker_cannot_invent_a_traffic_action_without_whitelist(rig):
    now,clock,backend,manual,owner,settings = rig
    handle = BorrowedActuator(Worker(owner),settings,CLIENT,clock=clock)
    with pytest.raises((ValueError,RuntimeError)):
        handle.send({'action':'traffic_approach','esc_us':1570,'steering_us':1610})
    assert not any(row[0]=='motion' and row[2]==1570 for row in backend.writes)
