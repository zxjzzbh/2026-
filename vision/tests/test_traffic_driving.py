import copy
from pathlib import Path

import pytest

from carvision.brake_output import PulseGuard
from carvision.traffic_driving import RedFrameVote, TrafficDrivingController, load_settings, validate_settings

CONFIG = Path(__file__).parents[1]/'configs/traffic-driving.json'


class Run:
    def __init__(self, color='green'):
        self.c = TrafficDrivingController(load_settings(CONFIG))
        self.t, self.fid, self.color = 0., 0, color
        self.rows = []

    def step(self, color=None, moving=False, dt=.1, **changes):
        self.t = round(self.t+dt, 8)
        self.fid += 1
        sample = {'fresh': True, 'camera_id': 'dual', 'frame_id': self.fid, 'captured_s': self.t, 'image_size': [480, 360],
                  'signal': {'state': color or self.color, 'fixture_detected': True, 'bbox_xyxy': [100, 70, 250, 125]},
                  'motion': {'valid': moving is not None, 'moving': moving, 'captured_s': self.t}}
        sample.update(changes)
        if 'camera_observations' not in sample:
            sample['camera_observations'] = {'secondary':{k:sample[k] for k in ('frame_id','captured_s','image_size','signal')}}
            sample['camera_observations']['secondary']['camera_id']='secondary'
        row = self.c.step(sample, self.t)
        self.rows.append(row)
        return row

    def until(self, phase, color=None, moving=False, limit=100):
        for _ in range(limit):
            row = self.step(color, moving)
            if row['phase'] == phase:
                return row
        raise AssertionError(self.rows[-1])


def test_red_requires_half_the_last_two_seconds_and_brakes_at_equality():
    r = Run()
    r.until('approach', moving=True)
    for _ in range(9):
        row = r.step('red', True)
        assert row['action'] == 'search' and not row['red_seen_this_round']
    row = r.step('red', True)
    assert row['action'] == 'brake' and row['esc_us'] == 1400
    assert row['red_seen_this_round'] and not row['parking_zone_verified']
    assert row['red_vote']['red_frames'] == 10 and row['red_vote']['total_frames'] == 20
    assert row['red_vote']['red_percent'] == 50


def test_unlit_fixture_can_be_approached_to_trigger_sensor_then_red_stops():
    r = Run('off'); r.until('approach', moving=True)
    assert r.rows[-1]['action'] == 'search' and not r.c.red_seen
    r.until('brake_pulse', 'red', True)
    r.until('wait_green', 'off')
    for _ in range(10):
        assert r.step('off')['action'] == 'neutral'
    r.until('done', 'green')
    assert r.rows[-1]['events'] == [{'event': 'speak', 'text': '红绿灯结束，开始前行'}]
    assert r.rows[-1]['esc_us'] == 1500


def test_missing_fixture_allows_search_but_is_not_red_evidence():
    r = Run('off'); r.until('approach', moving=True)
    row = r.step(moving=True,signal={'state': 'red', 'fixture_detected': False})
    assert row['action'] == 'search' and row['signal_state']=='unknown'
    assert not row['red_seen_this_round'] and row['red_vote']['red_frames']==0


def test_start_without_lamp_searches_until_fixed_approach_timeout():
    r = Run('unknown');r.until('approach',moving=True)
    assert r.rows[-1]['action']=='search'
    r.until('fault','unknown',moving=True)
    assert '接近超时' in r.c.reason and not r.c.red_seen


def test_after_red_stop_missing_lamp_never_restarts_motion_or_speaks():
    r=Run('red');r.until('wait_green')
    for _ in range(25):
        row=r.step('unknown')
        assert row['action']=='neutral' and row['events']==[]
    assert r.c.phase=='wait_green'


def test_yellow_seen_during_arming_still_stops_if_lamp_then_turns_off():
    r = Run('off'); r.step('yellow'); r.until('wait_green')
    assert not any(x['action'] == 'search' for x in r.rows)


@pytest.mark.parametrize('color', ['yellow'])
def test_other_non_green_stops_approach_and_does_not_fake_a_red_cycle(color):
    r = Run()
    r.until('approach', moving=True)
    assert r.step(color, True)['action'] == 'brake'
    r.until('wait_green', color)
    for _ in range(15):
        assert r.step('green')['action'] == 'neutral'
    assert not r.c.red_seen


def test_red_during_arming_is_latched_even_when_it_turns_green_before_arm_end():
    r = Run()
    for _ in range(22): r.step('red')
    r.until('wait_green', 'green')
    assert not any(x['action'] == 'search' for x in r.rows)
    r.until('done', 'green')
    assert r.rows[-1]['events'] == [{'event': 'speak', 'text': '红绿灯结束，开始前行'}]
    assert r.rows[-1]['esc_us'] == 1500


def test_one_red_frame_during_arming_does_not_latch_red_or_brake():
    r = Run(); r.step('red')
    r.until('approach', 'green', True)
    assert not r.c.red_seen and not r.c.stop_signal_seen
    assert not any(x['action'] == 'brake' for x in r.rows)


def test_candidate_fixture_change_before_confirmation_resets_vote_not_brakes():
    r = Run('off'); r.until('approach', moving=True)
    row = r.step(moving=True, signal={'state': 'red', 'fixture_detected': True, 'bbox_xyxy': [280, 200, 460, 260]})
    assert row['action'] == 'search' and not row['red_seen_this_round']
    row = r.step('off', True)
    assert row['action'] == 'search' and row['red_vote']['total_frames'] == 1


def test_vote_needs_a_full_two_seconds_even_when_all_initial_frames_are_red():
    v = RedFrameVote(2, 50)
    for i in range(20):
        row = v.update('red', i, i/10)
        assert row['red_percent'] == 100 and not row['confirmed']
    row = v.update('red', 20, 2.)
    assert row['confirmed'] and row['ready'] and row['total_frames'] == 20


def test_vote_is_time_windowed_and_counts_every_state_in_denominator():
    v = RedFrameVote(2, 50)
    for i in range(21):
        # Last 20 frames contain exactly 10 red, 10 unknown/off/green/yellow.
        row = v.update('red' if i % 2 else ['unknown', 'off', 'green', 'yellow'][i//2 % 4], i, i/10)
    assert row['confirmed'] and row['red_frames'] == 10 and row['total_frames'] == 20
    row = v.update('off', 21, 2.1)
    assert row['red_frames'] == 9 and row['red_percent'] == 45 and not row['confirmed']


@pytest.mark.parametrize('threshold,red_frames', [(25, 5), (50, 10), (75, 15), (100, 20)])
def test_configured_threshold_changes_the_actual_brake_trigger(threshold, red_frames):
    r = Run('off'); r.c.s['red_confirm_percent'] = threshold
    r.c = TrafficDrivingController(r.c.s)
    r.until('approach', moving=True)
    for _ in range(red_frames-1):
        assert r.step('red', True)['action'] == 'search'
    assert r.step('red', True)['action'] == 'brake'
    assert r.rows[-1]['red_vote']['threshold_percent'] == threshold


def test_window_is_two_seconds_not_a_fixed_number_of_frames():
    v = RedFrameVote(2, 50)
    # Nonuniform frame rate: 10 non-red at 5 Hz followed by 10 red at 10 Hz.
    for i in range(10): v.update('off', i, i*.2)
    for i in range(10): row = v.update('red', 10+i, 2+i*.1)
    assert row['total_frames'] == 15 and row['red_frames'] == 10 and row['confirmed']


@pytest.mark.parametrize('fid,t', [(0,.1), (1,0), (1,.3), (1,float('nan'))])
def test_vote_rejects_duplicate_reversed_gapped_or_invalid_frames(fid,t):
    v = RedFrameVote(2,50); v.update('off',0,0)
    with pytest.raises(ValueError): v.update('red',fid,t)
    assert len(v.frames) == 1


def test_one_red_cannot_authorize_green_speech_after_independent_safety_stop():
    r = Run('off'); r.until('approach', moving=True); r.step('red',True)
    r.step('yellow',True); r.until('wait_green','green')
    for _ in range(15): assert r.step('green')['events'] == []
    assert not r.c.red_seen


@pytest.mark.parametrize('percent',[0,101,True,float('nan'),float('inf'),'50'])
def test_invalid_red_percentage_is_rejected(percent):
    with pytest.raises(ValueError): validate_settings({**load_settings(CONFIG),'red_confirm_percent':percent})


def test_no_automatic_release_after_three_or_ten_seconds_red():
    r = Run('red')
    r.until('wait_green')
    for _ in range(125):
        assert r.step()['action'] == 'neutral'
    assert r.c.phase == 'wait_green'
    for _ in range(6):
        assert r.step('green')['action'] == 'neutral'
    assert r.until('done', 'green', limit=20)['action'] == 'neutral'


def test_flicker_resets_green_confirmation_and_motion_restarts_braking():
    r = Run('red')
    r.until('wait_green')
    for _ in range(5): r.step('green')
    r.step('red')
    for _ in range(5): assert r.step('green')['action'] == 'neutral'
    assert r.step('green', moving=True)['action'] == 'brake'
    r.until('wait_green', 'green')
    for _ in range(6): assert r.step('green')['action'] == 'neutral'


def test_green_while_still_moving_never_releases():
    r = Run('red')
    r.until('brake_pulse', moving=True)
    for _ in range(20):
        assert r.step('green', moving=True)['action'] != 'search'
    assert not any(x['events'] == [{'event': 'green_release'}] for x in r.rows)
    r.until('fault', 'green', True)


def test_actual_red_green_cycle_only_speaks_and_never_moves_again():
    r = Run('red')
    r.until('wait_green')
    r.until('done', 'green')
    assert r.rows[-1]['esc_us'] == 1500
    assert not r.rows[-1]['physical_stop_verified']
    assert r.rows[-1]['events'] == [{'event': 'speak', 'text': '红绿灯结束，开始前行'}]
    assert not any(x['action'] == 'search' for x in r.rows)
    for _ in range(20):
        row = r.step('green')
        assert row['action'] == 'neutral' and row['events'] == []


def test_return_to_red_after_completion_does_not_restart_motion_or_speech():
    r = Run('red'); r.until('wait_green'); r.until('done', 'green')
    assert r.step('red')['action'] == 'neutral' and r.rows[-1]['events'] == []


@pytest.mark.parametrize('change', [{'captured_s': -5}, {'frame_id': 1}, {'fresh': False},
                                   {'image_size': [640, 480]}, {'camera_id':'primary'}])
def test_bad_frame_faults_and_cannot_refresh_forward(change):
    r = Run(); r.until('approach', moving=True)
    assert r.step('green', True, **change)['phase'] == 'fault'
    assert r.step('green', True)['action'] == 'neutral'


def test_stale_ground_feedback_brakes_and_never_grants_green():
    r = Run(); r.until('approach', moving=True)
    assert r.step(motion={'valid': True, 'moving': False, 'captured_s': 0})['action'] == 'brake'
    r.until('fault', 'green', moving=None)


def test_camera_time_gap_causes_fault():
    r = Run(); r.step()
    assert r.step(dt=.3)['phase'] == 'fault'


def test_another_fixture_cannot_use_previous_red_to_release():
    r = Run('red'); r.until('wait_green')
    row=r.step(signal={'state':'green','fixture_detected':True,'bbox_xyxy':[280,200,460,260]})
    assert row['phase']=='wait_green' and not row['stable']['green_confirmed']


def test_approach_without_red_is_bounded_and_no_movement_is_detected():
    r = Run(); r.until('approach', moving=True); r.until('fault', moving=True)
    assert '接近超时' in r.c.reason
    r = Run(); r.until('approach'); r.until('fault')
    assert '未观察到移动' in r.c.reason


def test_wait_timeout_stops_and_cannot_substitute_for_green():
    r = Run('red'); r.c.s['max_wait_green_s'] = 15
    r.until('wait_green'); r.until('fault', limit=155)
    assert not any(x['action'] == 'search' for x in r.rows)


def test_controller_actions_are_accepted_by_existing_finite_pulse_guard():
    r = Run('red')
    writes = []
    guard = PulseGuard(r.c.s, lambda channel, pulse: writes.append((channel, pulse)), mode='F/B', forward_limit_us=1625)
    for i in range(130):
        color = 'green' if i < 35 or i >= 60 else 'red'
        moving = r.c.phase == 'approach' or r.c.phase == 'brake_pulse' and i < 37
        row = r.step(color, moving)
        guard.accept({'sequence': i, 'sent_s': r.t, 'action': row['action'],
                      'esc_us': row['esc_us'], 'steering_us': row['steering_us']}, r.t)
    assert (4, 1400) in writes and (4, 1600) in writes and writes[-2:] == [(4, 1500), (3, 1610)]


@pytest.mark.parametrize('changes', [{'green_action': 'forward'}, {'max_frame_age_s': 1},
                                    {'required_esc_mode': 'F/B/R'}, {'arming_s': 0}, {'camera': 'primary'}])
def test_invalid_settings_are_rejected(changes):
    with pytest.raises(ValueError): validate_settings({**load_settings(CONFIG), **changes})


def test_forward_ceiling_rejects_fast_or_reverse_commands():
    for value in (1500, 1400, 1626, True, float('nan')):
        s = copy.deepcopy(load_settings(CONFIG))
        s['pwm']['search_us'] = s['pwm']['creep_us'] = value
        with pytest.raises(ValueError): validate_settings(s)


@pytest.mark.parametrize('pulse',[1501,1575,1600,1605,1625])
def test_traffic_approach_uses_the_configured_tuning_value(pulse):
    r = Run('off');r.c.s['pwm']['search_us']=r.c.s['pwm']['creep_us']=pulse
    r.c=TrafficDrivingController(r.c.s)
    row=r.until('approach',moving=True)
    assert row['esc_us']==pulse and row['steering_us']==1610
    r.until('brake_pulse','red',True)
    assert r.rows[-1]['esc_us']==1400


def dual_step(r, primary, secondary, moving=False):
    from test_traffic_voting import frame
    t=round(r.t+.1,8);fid=r.fid+1
    return r.step(moving=moving,camera_observations={
        'primary':frame('primary',primary,fid,t),
        'secondary':frame('secondary',secondary,fid,t)})


@pytest.mark.parametrize('red_camera',['primary','secondary'])
@pytest.mark.parametrize('green_camera',['primary','secondary'])
def test_either_camera_can_stop_and_either_camera_can_finish_with_same_threshold(red_camera,green_camera):
    r=Run('off');r.until('approach',moving=True)
    colors={red_camera:'red',('primary' if red_camera=='secondary' else 'secondary'):'off'}
    for _ in range(35):
        row=dual_step(r,colors['primary'],colors['secondary'],moving=r.c.phase=='approach')
        if row['phase']=='wait_green':break
    assert r.c.phase=='wait_green' and r.c.red_seen
    colors={green_camera:'green',('primary' if green_camera=='secondary' else 'secondary'):'off'}
    for _ in range(20):
        row=dual_step(r,colors['primary'],colors['secondary'])
        assert row['action']=='neutral' and row['events']==[]
    row=dual_step(r,colors['primary'],colors['secondary'])
    assert row['phase']=='done' and row['esc_us']==1500
    assert row['signal_votes']['green_sources']==[green_camera]
    assert row['events']==[{'event':'speak','text':'红绿灯结束，开始前行'}]


def test_green_before_stop_is_discarded_and_red_wins_conflicting_camera_after_stop():
    r=Run('off');r.until('approach',moving=True)
    for _ in range(35):
        row=dual_step(r,'red','green',moving=r.c.phase=='approach')
        if row['phase']=='wait_green':break
    assert r.c.phase=='wait_green'
    for _ in range(25):
        row=dual_step(r,'red','green')
        assert row['action']=='neutral' and row['events']==[]
    assert row['signal_votes']['conflict'] and not row['stable']['green_confirmed']
    for _ in range(25):
        row=dual_step(r,'off','green')
        if row['phase']=='done':break
    assert row['phase']=='done'
