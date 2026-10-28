"""Fast red braking must not weaken formal red/green release evidence."""
import copy
from pathlib import Path

import pytest

from carvision.race_flow import FlowTrafficController, RaceFlowController, load_bundle
from carvision.cone_trial_output import ConePulseGuard
from carvision.traffic_driving import load_settings, validate_settings
from carvision.traffic_voting import DualSignalVotes
from test_traffic_driving import Run, dual_step
from test_traffic_voting import frame

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT/'vision/configs/traffic-driving.json'


def sample(fid, t, rows, moving=True):
    return {'fresh': True, 'camera_id': 'dual', 'frame_id': fid, 'captured_s': t,
            'image_size': [480, 360], 'camera_observations': rows,
            'motion': {'valid': True, 'moving': moving, 'captured_s': t}}


def cold_controller(required=3):
    settings = load_settings(CONFIG)
    settings['fast_red_frames'] = required
    return FlowTrafficController(settings, 0., 30.)


def cold_step(controller, fid, color='red', camera='secondary', moving=True):
    t = fid/10
    rows = {camera: frame(camera, color, fid, t)}
    return controller.step(sample(fid, t, rows, moving), t)


@pytest.mark.parametrize('camera', ['primary', 'secondary'])
@pytest.mark.parametrize('required', [2, 3])
def test_first_red_stops_without_warm_window_and_short_vote_keeps_required_frames(camera, required):
    controller = cold_controller(required)
    votes = DualSignalVotes(1, 50, green_window_s=2, fast_red_frames=required)
    for fid in range(required):
        result = cold_step(controller, fid, camera=camera)
        assert result['action'] == 'brake'
        assert result['events'] == ([{'event':'brake_requested', 'reason':result['reason']}] if fid == 0 else [])
        counted = votes.update({camera: frame(camera, 'red', fid, fid/10)}, fid/10)
        assert bool(counted['fast_red_sources']) is (fid == required-1)
    assert not result['red_vote']['ready'] and not result['red_seen_this_round']
    assert counted['fast_red_sources'] == [camera]
    assert result['fast_red']['required_frames'] == required
    assert result['brake_pulse_s'] == .35


def test_unknown_on_other_camera_does_not_reset_good_camera_streak():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(3):
        t = fid/10
        result = votes.update({
            'primary': frame('primary', 'unknown', fid, t),
            'secondary': frame('secondary', 'red', fid, t)}, t)
    assert result['fast_red_sources'] == ['secondary']


def test_streaks_cannot_be_added_across_cameras():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid, colors in enumerate((('red','off'), ('red','red'), ('off','red'))):
        t = fid/10
        result = votes.update({
            c: frame(c, color, fid, t) for c, color in zip(('primary','secondary'), colors)}, t)
        assert result['fast_red_sources'] == [] and not result['red_confirmed']
    assert result['cameras']['primary']['red_streak_frames'] == 0
    assert result['cameras']['secondary']['red_streak_frames'] == 2


@pytest.mark.parametrize('color', ['unknown', 'off', 'green', 'yellow'])
def test_every_nonred_new_frame_resets_only_its_camera_counter(color):
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid, observed in enumerate(('red', 'red', color, 'red', 'red', 'red')):
        t = fid/10
        result = votes.update({'primary': frame('primary', observed, fid, t)}, t)
        assert result['cameras']['primary']['red_streak_frames'] == (1,2,0,1,2,3)[fid]


def test_cached_red_frames_are_not_counted_by_faster_other_camera():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    cached = frame('primary', 'red', 0, 0.)
    for fid in range(4):
        t = fid/10
        result = votes.update({'primary': cached, 'secondary': frame('secondary','off',fid,t)}, t)
        assert result['fast_red_sources'] == [] and not result['red_confirmed']
        assert result['cameras']['primary']['red_streak_frames'] == (1 if fid<3 else 0)
    assert result['fast_red_timeout_sources'] == ['primary']


def test_source_time_gap_resets_streak_even_if_other_camera_keeps_control_fresh():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    cached = None
    for fid in range(5):
        t = fid/10
        if fid in (0, 1, 4):
            cached = frame('primary', 'red', fid, t)
        result = votes.update({'primary': cached, 'secondary': frame('secondary','off',fid,t)}, t)
        assert result['fast_red_sources'] == [] and not result['red_confirmed']
    assert result['cameras']['primary']['red_streak_frames'] <= 1
    assert result['fast_red_timeout_sources'] == ['primary']


@pytest.mark.parametrize('change', [
    {'signal': {'state':'red', 'fixture_detected':False, 'bbox_xyxy':[100,60,250,130]}},
    {'signal': {'state':'red', 'fixture_detected':True, 'bbox_xyxy':[0,0,0,0]}},
    {'image_size': [640,480]}, {'camera_id':'secondary'}, {'frame_id':-1},
    {'captured_s': -.5}, {'captured_s': 1.},
])
def test_invalid_red_frame_does_not_supply_third_hit(change):
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(2):
        t = fid/10
        votes.update({
            'primary': frame('primary','red',fid,t),
            'secondary': frame('secondary','off',fid,t)}, t)
    bad = frame('primary', 'red', 2, .2)
    bad.update(copy.deepcopy(change))
    result = votes.update({'primary': bad, 'secondary': frame('secondary','off',2,.2)}, .2)
    assert result['fast_red_sources'] == [] and not result['red_confirmed']


def test_different_fixture_restarts_counter_instead_of_becoming_third_hit():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(2):
        votes.update({'primary':frame('primary','red',fid,fid/10)}, fid/10)
    changed = frame('primary', 'red', 2, .2, [300,200,470,280])
    result = votes.update({'primary': changed}, .2)
    assert result['fast_red_sources'] == []
    assert result['cameras']['primary']['red_streak_frames'] == 1


def test_reversed_source_frame_restarts_counter_without_borrowing_old_hits():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(2):
        votes.update({'secondary':frame('secondary','red',fid,fid/10)}, fid/10)
    result = votes.update({'secondary': frame('secondary','red',0,.2)}, .2)
    assert result['cameras']['secondary']['red_streak_frames'] == 0
    result = votes.update({'secondary': frame('secondary','red',1,.3)}, .3)
    assert result['fast_red_timeout_sources'] == ['secondary']
    assert result['fast_red_sources'] == [] and not result['red_confirmed']


def test_fast_braking_retains_red_votes_and_discards_pre_stop_green_votes():
    r = Run('green')
    r.until('approach', moving=True)
    result = r.step('red', True)
    assert result['action'] == 'brake' and not r.c.red_seen
    assert r.c.votes.green_votes['secondary'].summary()['total_frames'] == 0
    for _ in range(2):
        result = r.step('red', True)
    assert not r.c.red_seen
    assert r.c.votes.votes['secondary'].summary()['red_frames'] == 3
    r.step('red', False)
    result = r.step('red', False)
    assert result['red_seen_this_round']
    assert result['red_vote']['red_frames'] == 5
    assert result['red_vote']['total_frames'] == 10


def test_only_three_red_frames_then_green_cannot_fake_formal_red_confirmation():
    controller = cold_controller()
    for fid in range(3):
        result = cold_step(controller, fid)
    assert result['action'] == 'brake' and not result['red_seen_this_round']
    verified_stop_without_red = False
    for fid in range(3, 45):
        result = cold_step(controller, fid, 'green', moving=False)
        verified_stop_without_red |= result['phase'] == 'verify_red'
        assert result['esc_us'] <= 1500
        assert not result['red_seen_this_round']
        assert not any(event['event']=='speak' for event in result['events'])
        if result['phase'] == 'fault':
            break
    assert verified_stop_without_red and result['phase'] == 'fault'
    assert not result['stable']['green_confirmed']
    assert result['red_sources_this_round'] == []


@pytest.mark.parametrize('camera', ['primary', 'secondary'])
def test_formal_red_after_fast_stop_then_fresh_full_green_window_can_finish(camera):
    controller = cold_controller()
    for fid in range(3):
        result = cold_step(controller, fid, camera=camera)
    # The first red already stopped the car, but cannot vote before the
    # second observation has established the fixture's identity.
    assert result['signal_votes']['fast_red_sources'] == []
    assert result['signal_votes']['cameras'][camera]['red_streak_frames'] == 2
    for fid in range(3, 45):
        # Keep motion until the formal red window has filled, then stop. The
        # first following green observation starts a wholly new green window.
        result = cold_step(controller, fid, camera=camera, moving=fid <= 10)
        if result['phase'] == 'wait_green' and result['red_seen_this_round']:
            break
    assert result['phase'] == 'wait_green' and result['red_seen_this_round']
    for green_index in range(1, 21):
        result = cold_step(controller, fid+green_index, 'green', camera=camera, moving=False)
        assert result['phase'] == 'wait_green' and result['events'] == []
    result = cold_step(controller, fid+21, 'green', camera=camera, moving=False)
    assert result['phase'] == 'done' and result['esc_us'] == 1500
    assert result['events'] == [{'event':'speak','text':'红绿灯结束，开始前行'}]


def test_ongoing_motion_after_fast_stop_cannot_release_on_green():
    controller = cold_controller()
    for fid in range(3):
        cold_step(controller, fid)
    for fid in range(3, 45):
        result = cold_step(controller, fid, 'green', moving=True)
        assert result['esc_us'] <= 1500
        assert not any(event['event']=='speak' for event in result['events'])
        if result['phase']=='fault':
            break
    assert result['phase']=='fault' and result['brake_pulses'] <= 3


@pytest.mark.parametrize('value', [0,1,4,-1,True,'3',2.5,None])
def test_invalid_fast_red_frame_configuration_rejected(value):
    with pytest.raises(ValueError):
        validate_settings({**load_settings(CONFIG),'fast_red_frames':value})


def test_initialization_keeps_neutral_and_checks_fast_red_before_first_forward():
    r = Run('off')
    for _ in range(28):
        r.step('off', False)
    for _ in range(3):
        r.step('red', False)
    result = r.step('red', False)
    assert result['action']=='brake'
    assert not any(row['action']=='search' for row in r.rows)


def test_full_race_flow_first_red_brakes_with_existing_guard_then_advances_only_after_cycle():
    controller = RaceFlowController(load_bundle(ROOT), 0.)
    controller._go('traffic', 0., 'test')
    bundle = controller.b
    guard = ConePulseGuard({'pwm':{'neutral_us':1500, 'steering_center_us':1610,
        'search_us':bundle['flow']['forward_us'], 'creep_us':bundle['brake']['pwm']['creep_us'],
        'brake_us':1400}, 'traffic_forward_us':bundle['traffic']['pwm']['search_us'],
        'brake_pulse_s':.35, 'brake_release_s':.1, 'actuator_lease_s':.2},
        lambda channel,pulse:None, mode='F/B',
        right_us=bundle['flow']['right_us'], left_us=bundle['flow']['left_us'])
    nav = {'experimental':True,'ready':True,'ego_fraction':.5,'half_envelope':.2,
           'lane_bounds_px':[[100,380],[50,430]],'cones':[],'uncertain':[],'source_camera':'secondary'}
    for fid in range(1, 60):
        t = fid/10
        color = 'off' if fid<3 else 'red' if fid<25 else 'green'
        traffic = sample(fid, t, {'secondary':frame('secondary',color,fid,t)}, fid<=3)
        result = controller.step({'sequence':fid,'captured_s':t,'traffic':traffic,
            'navigation':{**nav,'captured_s':t,'frame_id':fid},'motion':traffic['motion']}, t)
        assert controller.stage!='fault', result
        guard.accept({**result,'sequence':fid,'sent_s':t}, t)
        if fid<3:
            assert result['action']=='traffic_approach' and 1500<result['esc_us']<=1575
        elif fid==3:
            assert result['esc_us']==1400
            assert not controller.controller.red_seen
        if controller.stage=='cones':
            break
    assert controller.stage=='cones' and result['esc_us']==1500
    for offset in (1,2):
        t2=t+offset*.1;fid2=fid+offset
        result=controller.step({'sequence':fid2,'captured_s':t2,
            'navigation':{**nav,'captured_s':t2,'frame_id':fid2},'motion':traffic['motion']},t2)
        guard.accept({**result,'sequence':fid2,'sent_s':t2},t2)
    assert result['esc_us']==bundle['flow']['forward_us']
