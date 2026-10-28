"""One-second red voting and actual 350-ms actuator edges, without hardware."""
import copy
import json
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.brake_output import PulseGuard
from carvision.cone_trial_output import ConePulseGuard
from carvision.race_flow import FlowTrafficController, load_bundle
from carvision.race_flow_console import RaceFlowDrive
from carvision.traffic_driving import load_settings, validate_settings
from carvision.traffic_trial import TrafficTrialDrive
from carvision.traffic_voting import DualSignalVotes, SignalFrameVote
from test_traffic_driving import Run, dual_step
from test_traffic_voting import frame

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT/'vision/configs/traffic-driving.json'
PROFILE = ROOT/'vision/configs/traffic-signal.json'


@pytest.mark.parametrize('camera', ['primary', 'secondary'])
def test_red_needs_one_full_second_but_not_two_and_unknown_other_camera_does_not_block(camera):
    votes = DualSignalVotes(1, 50, green_window_s=2)
    other = 'secondary' if camera == 'primary' else 'primary'
    for fid in range(11):
        t = fid/10
        result = votes.update({camera: frame(camera, 'red', fid, t),
                               other: frame(other, 'unknown', fid, t)}, t)
        assert result['red_confirmed'] == (fid == 10)
        assert not result['green_confirmed']
    assert result['red_sources'] == [camera]
    assert result['cameras'][camera]['total_frames'] == 10
    assert result['cameras'][camera]['green_total_frames'] == 11


def test_green_keeps_its_full_two_second_window_after_red_window_warms_up():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(21):
        t = fid/10
        result = votes.update({'secondary': frame('secondary', 'green', fid, t)}, t)
        assert result['green_confirmed'] == (fid == 20)
    row = result['cameras']['secondary']
    assert row['total_frames'] == 10 and row['green_total_frames'] == 20
    assert row['ready'] and row['green_ready']


def test_red_denominator_includes_unknown_off_yellow_and_green_at_half_threshold():
    votes = SignalFrameVote(1, 50)
    for fid in range(11):
        color = 'red' if fid % 2 else ['unknown', 'off', 'yellow', 'green'][fid//2 % 4]
        result = votes.update(color, fid, fid/10)
    assert result['total_frames'] == 10 and result['red_frames'] == 5
    assert result['red_percent'] == 50 and result['red_confirmed']
    result = votes.update('unknown', 11, 1.1)
    assert result['red_percent'] == 40 and not result['red_confirmed']


def test_one_second_is_time_based_with_unequal_camera_rates_and_cached_frames():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(41):
        t, secondary_id = fid*.05, fid//4
        result = votes.update({'primary': frame('primary', 'unknown', fid, t),
                               'secondary': frame('secondary', 'red', secondary_id, secondary_id*.2)}, t)
    assert result['red_sources'] == ['secondary']
    assert result['cameras']['primary']['total_frames'] == 20
    assert result['cameras']['secondary']['total_frames'] == 5
    assert result['cameras']['secondary']['green_total_frames'] == 10


def test_stale_red_and_green_votes_are_discarded_together():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(21):
        result = votes.update({'secondary': frame('secondary', 'green', fid, fid/10)}, fid/10)
    assert result['green_confirmed']
    result = votes.update({'secondary': frame('secondary', 'green', 20, 2.)}, 2.3)
    assert not result['green_confirmed']
    assert result['cameras']['secondary']['total_frames'] == 0
    assert result['cameras']['secondary']['green_total_frames'] == 0


def test_red_priority_and_yellow_hold_still_apply_with_separate_windows():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(21):
        t = fid/10
        result = votes.update({'primary': frame('primary', 'red', fid, t),
                               'secondary': frame('secondary', 'green', fid, t)}, t)
    assert result['conflict'] and result['red_confirmed'] and not result['green_confirmed']
    for fid in range(21, 42):
        t = fid/10
        result = votes.update({'primary': frame('primary', 'yellow', fid, t),
                               'secondary': frame('secondary', 'green', fid, t)}, t)
    assert not result['red_confirmed'] and result['yellow_present'] and not result['green_confirmed']


@pytest.mark.parametrize('camera', ['primary', 'secondary'])
def test_controller_fast_stops_before_formal_one_second_red_vote(camera):
    r = Run('off')
    for _ in range(40):
        dual_step(r, 'off', 'off', moving=True)
        if r.c.phase == 'approach':
            break
    assert r.c.phase == 'approach'
    for index in range(3):
        colors = {camera: 'red', 'secondary' if camera == 'primary' else 'primary': 'unknown'}
        result = dual_step(r, colors['primary'], colors['secondary'], moving=True)
        assert result['action'] == 'brake'
        assert result['signal_votes']['cameras'][camera]['red_streak_frames'] == index+1
    assert result['brake_pulse_s'] == .35
    assert result['signal_votes']['fast_red_sources'] == [camera] and not result['red_seen_this_round']
    for _ in range(2):
        result = dual_step(r, colors['primary'], colors['secondary'], moving=False)
    assert result['red_seen_this_round']


def observation(fid, t, color='red', moving=True):
    return {'fresh': True, 'camera_id': 'dual', 'frame_id': fid, 'captured_s': t,
            'image_size': [480, 360],
            'camera_observations': {'secondary': frame('secondary', color, fid, t)},
            'motion': {'valid': True, 'moving': moving, 'captured_s': t}}


def assert_350ms_edges(controller, guard, start, fid):
    for index, offset in enumerate((0., .1, .2, .3, .349, .35)):
        t = start+offset
        intent = controller.step(observation(fid+index, t), t)
        guard.accept({**intent, 'sequence': index, 'sent_s': t}, t)
        assert intent['brake_pulse_s'] == .35
        assert intent['action'] == ('brake' if offset < .35 else 'neutral')
        assert guard.esc == (1400 if offset < .35 else 1500)
    assert not guard.fault and guard.brake_until is None
    assert controller.brake_count == 1 and guard.last_brake_end == pytest.approx(start+.35)


def test_single_traffic_controller_and_guard_both_hold_350ms_not_120ms():
    r = Run('off')
    r.until('approach', moving=True)
    # The first trustworthy red starts the pulse, before formal confirmation.
    start, fid = r.t+.1, r.fid+1
    guard = PulseGuard(r.c.s, lambda *args: None, mode='F/B', forward_limit_us=1625)
    assert_350ms_edges(r.c, guard, start, fid)


def test_full_flow_guard_does_not_truncate_traffic_when_other_task_uses_120ms(monkeypatch):
    bundle = load_bundle(ROOT)
    bundle['brake']['brake_pulse_s'] = .12
    captured = {}
    def factory(settings, **kwargs):
        captured.update(copy.deepcopy(settings))
        return SimpleNamespace(run=False)
    monkeypatch.setattr('carvision.race_flow_console.ConeTrialActuator', factory)
    RaceFlowDrive._make_actuator(SimpleNamespace(live=False), bundle)
    assert captured['brake_pulse_s'] == .35
    controller = FlowTrafficController(bundle['traffic'], 0., 30.)
    guard = ConePulseGuard(captured, lambda *args: None, mode='F/B', right_us=1350, left_us=1870)
    assert_350ms_edges(controller, guard, 0., 0)


def test_repeated_brake_commands_do_not_extend_350ms_and_lease_stays_short():
    settings = load_settings(CONFIG)
    assert settings['actuator_lease_s'] == .25
    guard = PulseGuard(settings, lambda *args: None, mode='F/B', forward_limit_us=1625)
    for fid, t in enumerate((0., .1, .2, .3, .349, .35, .4)):
        guard.accept({'action': 'brake', 'esc_us': 1400, 'steering_us': 1610,
                      'sequence': fid, 'sent_s': t}, t)
        assert guard.esc == (1400 if t < .35 else 1500)
    assert not guard.fault


def test_shipped_saved_settings_use_half_threshold_and_preserve_field_throttle(tmp_path):
    saved = json.loads((ROOT/'vision/tests/fixtures/configs/traffic-driving-field-20261026.json').read_text(encoding='utf-8'))
    configs=tmp_path/'vision/configs'
    shutil.copytree(ROOT/'vision/configs',configs)
    (configs/'traffic-driving.local.json').write_text(json.dumps(saved),encoding='utf-8')
    trial = TrafficTrialDrive(SimpleNamespace(), {}, configs/'traffic-driving.json',
                              configs/'traffic-signal.json', tmp_path)
    assert not trial.settings_error
    assert saved['red_confirm_percent'] == trial.settings['red_confirm_percent'] == 50
    assert trial.settings['pwm']['search_us'] == saved['forward_us']
    assert 1501 <= saved['forward_us'] <= 1625
    assert trial.active_settings['pwm']['search_us'] <= saved['forward_us']
    assert trial.settings['max_approach_s'] == saved['max_approach_s'] == 10
    assert trial.settings['red_window_s'] == 1 and trial.settings['green_window_s'] == 2
    assert trial.settings['brake_pulse_s'] == .35
    assert 'parking_mode' not in trial.settings


@pytest.mark.parametrize('change', [
    {'brake_pulse_s': .12}, {'brake_pulse_s': .5}, {'red_window_s': 2},
    {'green_window_s': 1}, {'red_window_s': True},
])
def test_inconsistent_timing_config_is_rejected(change):
    with pytest.raises(ValueError):
        validate_settings({**load_settings(CONFIG), **change})


def test_console_uses_separate_red_green_denominators_and_displays_350ms():
    import shutil
    import subprocess
    from carvision.traffic_controls import SCRIPT
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node.js unavailable for offline console render')
    script = r'''
const vm=require('node:vm'), assert=require('node:assert/strict');
const nodes=new Map();
const env={trafficActive:false,trafficSaveBusy:false,trafficStartBusy:false,trafficStartError:'',
 trafficDirty:false,stale:false,client:'page',held:new Map(),gamepadControl:null,latency:0,
 feedback(){},document:{querySelectorAll(){return [];}},
 el(id){if(!nodes.has(id))nodes.set(id,{dataset:{},value:'',textContent:''});return nodes.get(id);}};
vm.createContext(env);vm.runInContext(require('node:fs').readFileSync(0,'utf8'),env);
const row={valid:true,red_frames:4,total_frames:10,red_percent:40,ready:true,
 green_frames:12,green_total_frames:20,green_percent:60,green_ready:false,green_collected_s:1.5};
const cfg={pwm:{search_us:1570},red_window_s:1,green_window_s:2,red_confirm_percent:50,
 brake_pulse_s:.35,max_approach_s:10};
env.renderTraffic({traffic_trial:{active:true,owner_client:'page',settings:cfg,phase:'wait_green',
 frame_age_s:0,observation:{signal:{state:'off'},camera_observations:{}},
 intent:{phase:'wait_green',esc_us:1500,signal_votes:{window_s:1,green_window_s:2,
 threshold_percent:50,cameras:{secondary:row}}}}});
assert.match(env.el('traffic-settings-state').textContent,/红灯 1 秒.*绿灯 2 秒.*350 ms/);
assert.match(env.el('traffic-red-vote').textContent,/红 4\/10.*绿 12\/20/);
assert.match(env.el('traffic-red-vote').textContent,/绿窗 1.5\/2 秒/);
'''
    result = subprocess.run([node, '-e', script], input=SCRIPT.split('async function trafficSend')[0],
                            encoding='utf-8', text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stderr
