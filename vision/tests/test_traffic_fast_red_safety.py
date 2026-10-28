"""Traffic safety behavior using fresh sensor fixtures; no camera or actuator.

Fast stopping is separate from formal red evidence and green release. The
fixtures use the service's default 15 Hz cadence unless a test varies timing.
"""
from pathlib import Path

import pytest

from carvision.race_flow import FlowTrafficController
from carvision.traffic_driving import TrafficDrivingController, load_settings
from carvision.traffic_voting import DualSignalVotes


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'vision/configs/traffic-driving.json'
DT = 1 / 15
BOX = [100, 60, 250, 130]


def camera_row(camera, color, fid, captured, box=None):
    return {'camera_id': camera, 'frame_id': fid, 'captured_s': captured,
            'image_size': [480, 360],
            'signal': {'state': color, 'fixture_detected': color != 'unknown',
                       'bbox_xyxy': list(BOX if box is None else box)}}


class Frames:
    def __init__(self, *, arming=False):
        settings = load_settings(CONFIG)
        self.c = (TrafficDrivingController(settings) if arming else
                  FlowTrafficController(settings, 0., 30.))
        self.t, self.fid = -DT, -1
        self.outputs = []

    def step(self, colors=None, *, dt=DT, moving=True, overrides=None, top=None):
        self.t = round(self.t + dt, 10)
        self.fid += 1
        colors = {'secondary': 'off'} if colors is None else colors
        rows = {camera: camera_row(camera, color, self.fid, self.t)
                for camera, color in colors.items()}
        for camera, change in (overrides or {}).items():
            if change is None:
                rows.pop(camera, None)
            else:
                rows[camera].update(change)
        sample = {'fresh': True, 'camera_id': 'dual', 'frame_id': self.fid,
                  'captured_s': self.t, 'image_size': [480, 360],
                  'camera_observations': rows,
                  'motion': {'valid': True, 'moving': moving, 'captured_s': self.t}}
        sample.update(top or {})
        result = self.c.step(sample, self.t)
        self.outputs.append(result)
        return result


def stopped_after_formal_red():
    frames = Frames()
    for _ in range(45):
        result = frames.step({'primary': 'red'}, moving=False)
        assert result['phase'] != 'fault', result
        if result['phase'] == 'wait_green' and result['red_seen_this_round']:
            return frames
    pytest.fail('formal red and stationary waiting were not reached')


def qualified_green_held_by_yellow():
    frames = stopped_after_formal_red()
    for _ in range(36):
        result = frames.step({'primary': 'green', 'secondary': 'yellow'}, moving=False)
        assert result['phase'] == 'wait_green', result
    assert result['signal_votes']['cameras']['primary']['green_confirmed']
    return frames


def stopped_with_red_confirmed_before_wait():
    frames = Frames()
    for _ in range(45):
        # First retain actual motion while the formal window fills, then
        # provide continuous stopped observations. The red incident must be
        # completed by this stop, rather than a new red first seen in waiting.
        result = frames.step({'primary': 'red'}, moving=not frames.c.red_seen)
        assert result['phase'] != 'fault', result
        if result['phase'] == 'wait_green':
            assert result['red_seen_this_round']
            return frames
    pytest.fail('formal red before stationary waiting was not reached')


@pytest.mark.parametrize('camera', ['primary', 'secondary'])
@pytest.mark.parametrize('warm', [False, True])
def test_default_15hz_first_red_stops_and_third_red_still_completes_short_vote(camera, warm):
    frames = Frames()
    votes = DualSignalVotes(1, 50, green_window_s=2)
    if warm:
        for _ in range(18):
            assert frames.step({camera: 'off'})['action'] == 'search'
            votes.update({camera:camera_row(camera,'off',frames.fid,frames.t)}, frames.t)
    first_red_at = frames.t + DT
    for index in range(3):
        result = frames.step({camera: 'red'})
        assert result['action'] == 'brake'
        # Raw short-vote timing is unchanged. A cold controller separately
        # withholds the first color vote until two frames lock its identity.
        counted = votes.update({camera:camera_row(camera,'red',frames.fid,frames.t)}, frames.t)
        assert counted['cameras'][camera]['red_streak_frames'] == index+1
        assert bool(counted['fast_red_sources']) is (index == 2)
        assert result['signal_votes']['cameras'][camera]['red_streak_frames'] == index+(1 if warm else 0)
    assert frames.t - first_red_at == pytest.approx(2 / 15)
    assert counted['fast_red_sources'] == [camera]
    assert not result['red_seen_this_round']
    assert result['brake_pulse_s'] == .35


def test_exact_200ms_three_frame_span_is_allowed():
    frames = Frames()
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for dt in (DT,.1,.1):
        result = frames.step({'secondary':'red'}, dt=dt)
        counted = votes.update({'secondary':camera_row('secondary','red',frames.fid,frames.t)}, frames.t)
        assert result['action'] == 'brake'
    assert counted['fast_red_sources'] == ['secondary']
    assert not result['red_seen_this_round']


def test_cached_source_frames_cannot_be_counted_by_faster_other_camera():
    frames = Frames()
    result = frames.step({'primary': 'off', 'secondary': 'red'})
    cached = camera_row('secondary', 'red', frames.fid, frames.t)
    votes = DualSignalVotes(1, 50, green_window_s=2)
    votes.update({'secondary':cached}, frames.t)
    for _ in range(3):
        result = frames.step({'primary': 'off', 'secondary': 'red'}, dt=.04,
                             overrides={'secondary': cached})
        assert result['esc_us'] <= 1500
        assert not result['fast_red']['triggered']
        counted = votes.update({'secondary':cached,
            'primary':camera_row('primary','off',frames.fid,frames.t)}, frames.t)
        assert counted['cameras']['secondary']['red_streak_frames'] == 1
        assert counted['fast_red_sources'] == []
    result = frames.step({'primary': 'off', 'secondary': 'red'}, dt=.0133333333)
    assert not result['fast_red']['triggered']


def test_two_camera_hits_cannot_be_pooled_into_fast_confirmation():
    frames = Frames()
    for colors in ({'primary': 'red', 'secondary': 'off'},
                   {'primary': 'red', 'secondary': 'red'},
                   {'primary': 'off', 'secondary': 'red'}):
        result = frames.step(colors)
        assert not result['fast_red']['triggered']
        assert not result['red_seen_this_round']


def test_fixture_jump_discards_hits_but_does_not_extend_pending_red_deadline():
    frames = Frames()
    frames.step({'primary': 'red'})
    frames.step({'primary': 'red'})
    result = frames.step({'primary': 'red'}, overrides={
        'primary': {'signal': {'state': 'red', 'fixture_detected': True,
                              'bbox_xyxy': [300, 200, 470, 280]}}})
    assert not result['fast_red']['triggered']
    result = frames.step({'primary': 'unknown'}, dt=.11)
    assert result['esc_us'] <= 1500
    assert result['action'] == 'brake' or result['phase'] == 'fault'
    assert not result['red_seen_this_round']


def test_gradually_moving_boxes_must_match_the_first_fixture_anchor():
    frames = Frames()
    for box in ([100, 60, 250, 130], [170, 60, 320, 130], [240, 60, 390, 130]):
        result = frames.step({'primary': 'red'}, overrides={
            'primary': {'signal': {'state': 'red', 'fixture_detected': True,
                                  'bbox_xyxy': box}}})
    # Each adjacent pair overlaps by .364, but first-to-last overlap is .034.
    assert not result['fast_red']['triggered']
    result = frames.step({'primary': 'unknown'}, dt=.11)
    assert result['action'] == 'brake' and not result['red_seen_this_round']


@pytest.mark.parametrize('change', [{'frame_id': 0}, {'captured_s': .01}])
def test_reversed_source_identity_cannot_complete_fast_streak_or_extend_deadline(change):
    frames = Frames()
    frames.step({'primary': 'red'})
    frames.step({'primary': 'red'})
    result = frames.step({'primary': 'red'}, overrides={'primary': change})
    assert not result['fast_red']['triggered']
    result = frames.step({'primary': 'unknown'}, dt=.11)
    assert result['action'] == 'brake' and not result['red_seen_this_round']


@pytest.mark.parametrize('timing', ['span', 'gap', 'age'])
def test_untrusted_third_red_stops_conservatively_without_fast_or_formal_confirmation(timing):
    frames = Frames()
    frames.step({'secondary': 'red'})
    if timing == 'span':
        frames.step({'secondary': 'red'}, dt=.11)
        result = frames.step({'secondary': 'red'}, dt=.11)
    elif timing == 'gap':
        frames.step({'secondary': 'red'})
        result = frames.step({'secondary': 'red'}, dt=.18)
    else:
        frames.step({'secondary': 'red'})
        result = frames.step({'primary': 'off', 'secondary': 'red'}, dt=.23,
                             overrides={'secondary': {'captured_s': .13}})
    assert result['esc_us'] <= 1500
    assert result['action'] == 'brake' or result['phase'] == 'fault'
    assert not result['fast_red']['triggered']
    assert not result['red_seen_this_round']


def test_unknown_and_missing_red_source_cannot_restart_candidate_deadline():
    frames = Frames()
    frames.step({'primary': 'red', 'secondary': 'off'})
    frames.step({'primary': 'unknown', 'secondary': 'off'}, dt=.08)
    frames.step({'secondary': 'off'}, dt=.08)
    result = frames.step({'primary': 'unknown', 'secondary': 'off'}, dt=.08)
    assert result['action'] == 'brake'
    assert not result['fast_red']['triggered']
    assert not result['red_seen_this_round']


@pytest.mark.parametrize('clearing_color', ['off', 'green'])
def test_same_fixture_current_nonred_can_clear_unexpired_red_candidate(clearing_color):
    frames = Frames()
    assert frames.step({'primary': 'red'})['action'] == 'brake'
    votes = DualSignalVotes(1, 50, green_window_s=2)
    votes.update({'primary':camera_row('primary','red',0,0.)}, 0.)
    for index in range(1,6):
        result = frames.step({'primary': clearing_color}, moving=False)
        # Clearing the short candidate never revokes the already issued stop.
        assert result['esc_us'] <= 1500
        assert not result['fast_red']['triggered']
        assert not result['red_seen_this_round']
        counted = votes.update({'primary':camera_row('primary',clearing_color,index,index*DT)}, index*DT)
        assert not counted['cameras']['primary']['fast_red_pending']
        assert counted['fast_red_timeout_sources'] == []


@pytest.mark.parametrize('clearing_color', ['off', 'green'])
def test_nonred_cannot_retroactively_clear_an_expired_red_deadline(clearing_color):
    frames = Frames()
    frames.step({'primary': 'red'})
    result = frames.step({'primary': clearing_color}, dt=.21)
    assert result['action'] == 'brake' and not result['fast_red']['triggered']
    assert not result['red_seen_this_round']


def test_another_fixture_off_cannot_clear_the_original_red_candidate():
    frames = Frames()
    frames.step({'primary': 'red'})
    for _ in range(4):
        result = frames.step({'primary': 'off'}, overrides={
            'primary': {'signal': {'state': 'off', 'fixture_detected': True,
                                  'bbox_xyxy': [300, 200, 470, 280]}}})
    assert result['esc_us'] <= 1500 and not result['red_seen_this_round']
    assert any(row['action'] == 'brake' for row in frames.outputs)


@pytest.mark.parametrize('stop_case', ['confirmed', 'unresolved_timeout'])
def test_stop_request_during_arming_is_latched_until_initialization_finishes(stop_case):
    frames = Frames(arming=True)
    for index in range(47):
        if stop_case == 'confirmed':
            color = 'red' if index < 3 else 'off'
        else:
            color = 'red' if index == 0 else 'unknown' if index < 6 else 'off'
        result = frames.step({'secondary': color}, moving=False)
        assert result['esc_us'] <= 1500
        if result['action'] == 'brake':
            break
    assert frames.t >= 3
    assert result['action'] == 'brake'
    assert not result['red_seen_this_round']
    assert not any(row['action'] == 'search' for row in frames.outputs)


def test_formal_one_second_red_evidence_survives_fast_brake_and_stationary_transition():
    frames = Frames()
    stationary_seen = False
    for _ in range(23):
        result = frames.step({'primary': 'red'}, moving=False)
        stationary_seen |= any(event['event'] == 'stationary_estimate' for event in result['events'])
        if frames.t >= 1 + DT:
            assert result['red_seen_this_round']
            assert result['red_vote']['ready']
    assert stationary_seen and result['phase'] == 'wait_green'
    assert result['signal_votes']['cameras']['primary']['collected_s'] == 1


def test_stopping_opens_a_new_full_two_second_green_window():
    frames = stopped_after_formal_red()
    wait_at = frames.c.wait_at
    for _ in range(45):
        result = frames.step({'primary': 'green'}, moving=False)
        if frames.t - wait_at < 2 - 1e-8:
            assert result['phase'] == 'wait_green'
            assert not any(event['event'] == 'speak' for event in result['events'])
        if result['phase'] == 'done':
            break
    assert result['phase'] == 'done' and frames.t - wait_at >= 2
    assert result['esc_us'] == 1500
    assert result['events'] == [{'event': 'speak', 'text': frames.c.s['speech_text']}]


def test_current_red_vetoes_historically_qualified_green_without_formal_red_votes():
    frames = qualified_green_held_by_yellow()
    result = frames.step({'primary': 'green', 'secondary': 'red'}, moving=False)
    assert not result['signal_votes']['cameras']['secondary']['red_confirmed']
    assert result['phase'] == 'wait_green' and result['events'] == []
    assert result['esc_us'] == 1500
    result = frames.step({'primary': 'green', 'secondary': 'off'}, moving=False)
    assert result['phase'] == 'done'


def test_release_requires_current_green_from_the_historically_qualified_camera():
    frames = qualified_green_held_by_yellow()
    result = frames.step({'primary': 'off', 'secondary': 'green'}, moving=False)
    assert result['signal_votes']['cameras']['primary']['green_confirmed']
    assert not result['signal_votes']['cameras']['secondary']['green_confirmed']
    assert result['phase'] == 'wait_green' and result['events'] == []
    result = frames.step({'primary': 'green', 'secondary': 'off'}, moving=False)
    assert result['phase'] == 'done'


@pytest.mark.parametrize('other_camera_observes_formal_red', [False, True])
def test_other_camera_must_confirm_its_own_formal_red_before_green_takeover(other_camera_observes_formal_red):
    frames = stopped_with_red_confirmed_before_wait()
    wait_at = frames.c.wait_at
    if other_camera_observes_formal_red:
        for _ in range(18):
            result = frames.step({'secondary':'red'}, moving=False)
            assert result['phase'] == 'wait_green' and result['events'] == []
        assert result['signal_votes']['cameras']['secondary']['red_confirmed']
    for _ in range(45):
        # Primary's old red cannot authorize a possibly distant secondary
        # green. Secondary can take over only after its own real red vote.
        result = frames.step({'secondary': 'green'}, moving=False)
        assert result['red_seen_this_round'] and result['esc_us'] <= 1500
        assert result['phase'] != 'fault', result
        if not other_camera_observes_formal_red:
            assert result['phase'] == 'wait_green' and result['events'] == []
        if frames.t - wait_at < 2 - 1e-8:
            assert result['phase'] == 'wait_green'
        if result['phase'] == 'done':
            break
    assert result['phase'] == ('done' if other_camera_observes_formal_red else 'wait_green')
    assert frames.t - wait_at >= 2
    assert result['signal_votes']['current_green_sources'] == ['secondary']


def test_formal_red_camera_cannot_transfer_its_identity_to_a_different_green_fixture():
    frames = stopped_after_formal_red()
    for _ in range(45):
        result = frames.step({'primary':'green'}, moving=False, overrides={
            'primary':{'signal':{'state':'green', 'fixture_detected':True,
                                'bbox_xyxy':[300,200,470,280]}}})
        assert result['phase'] == 'wait_green' and result['events'] == []
        assert result['signal_state'] == 'unknown' and result['esc_us'] == 1500
    # Recovering the original observed fixture can still complete the cycle;
    # rejecting a distant lamp must not turn into an irreversible software lock.
    for _ in range(45):
        result = frames.step({'primary':'green'}, moving=False)
        if result['phase'] == 'done':
            break
    assert result['phase'] == 'done' and result['esc_us'] == 1500
    assert result['signal_votes']['current_green_sources'] == ['primary']


def test_unlocked_alternating_fixture_boxes_cannot_supply_a_formal_red_green_cycle():
    frames = Frames()
    # Each adjacent pair passes the legacy vote's .25 IoU threshold but not
    # the identity tracker's .4 threshold. Neither physical lamp is locked.
    boxes = ([100,60,250,130], [170,60,320,130])
    for index in range(30):
        result = frames.step({'primary':'red'}, moving=False, overrides={
            'primary':{'signal':{'state':'red', 'fixture_detected':True,
                                'bbox_xyxy':boxes[index % 2]}}})
        assert result['esc_us'] <= 1500
    assert not result['approach_guard']['cameras']['primary']['locked']
    assert not result['red_seen_this_round'] and result['red_sources_this_round'] == []
    for index in range(60):
        result = frames.step({'primary':'green'}, moving=False, overrides={
            'primary':{'signal':{'state':'green', 'fixture_detected':True,
                                'bbox_xyxy':boxes[index % 2]}}})
        assert result['phase'] != 'done' and result['events'] == []
        assert result['esc_us'] <= 1500


@pytest.mark.parametrize('clearing_color', ['off', 'green'])
def test_new_red_while_waiting_remains_a_veto_through_unknown_until_same_fixture_clears(clearing_color):
    frames = qualified_green_held_by_yellow()
    result = frames.step({'primary': 'green', 'secondary': 'red'}, moving=False)
    assert result['phase'] == 'wait_green'
    assert not result['signal_votes']['cameras']['secondary']['red_confirmed']
    for _ in range(5):
        # Primary's old qualified green cannot clear a new secondary red
        # suspicion, even after the secondary classification becomes unknown
        # and its short confirmation deadline expires.
        result = frames.step({'primary': 'green', 'secondary': 'unknown'}, moving=False)
        assert result['phase'] == 'wait_green' and result['events'] == []
        assert result['esc_us'] == 1500
    result = frames.step({'primary': 'green', 'secondary': clearing_color}, moving=False)
    assert result['esc_us'] <= 1500
    if result['phase'] == 'wait_green':
        # A deadline diagnosis may still veto the batch that clears the
        # suspicion; the following fresh trusted observation can finish.
        result = frames.step({'primary': 'green', 'secondary': clearing_color}, moving=False)
    assert result['phase'] == 'done'
    assert result['events'] == [{'event': 'speak', 'text': frames.c.s['speech_text']}]


def test_brief_red_fast_stop_never_turns_initial_green_into_formal_red_evidence():
    frames = Frames()
    for _ in range(3):
        result = frames.step({'primary': 'red'})
    assert result['action'] == 'brake' and not result['red_seen_this_round']
    for _ in range(60):
        result = frames.step({'primary': 'green'}, moving=False)
        assert result['esc_us'] <= 1500
        assert not result['red_seen_this_round']
        assert result['phase'] != 'done'
        assert not any(event['event'] == 'speak' for event in result['events'])


def test_verify_red_is_neutral_and_a_second_brake_cannot_restart_its_deadline():
    frames = Frames()
    assert frames.step({'primary':'red'}, moving=True)['action'] == 'brake'
    first_stop_request = frames.t
    saw_verify = False
    while frames.t < 2:
        result = frames.step({'primary':'off'}, moving=False)
        saw_verify |= result['phase'] == 'verify_red'
        assert result['phase'] != 'wait_green'
        assert not any(event['event']=='speak' for event in result['events'])
        assert not result['red_seen_this_round'] and result['esc_us'] <= 1500
        if result['phase'] == 'verify_red':
            assert result['esc_us'] == 1500
    assert saw_verify
    # A renewed motion estimate can request a bounded brake, but may not
    # buy another three seconds of unconfirmed red/green waiting.
    result = frames.step({'primary':'off'}, moving=True)
    assert result['action'] == 'brake' and result['phase'] != 'wait_green'
    for _ in range(30):
        result = frames.step({'primary':'off'}, moving=False)
        assert result['phase'] != 'wait_green'
        assert not any(event['event']=='speak' for event in result['events'])
        assert result['esc_us'] <= 1500
        if result['phase'] == 'fault':
            break
    assert result['phase'] == 'fault'
    assert frames.t-first_stop_request <= 3+DT+1e-8


def test_formal_red_first_qualified_after_fixed_verification_deadline_cannot_revive_the_round():
    frames = Frames()
    for index in range(30):
        color = 'red' if index == 0 or index >= 26 else 'off'
        result = frames.step({'secondary':color}, dt=DT if index == 0 else .1,
                             moving=False)
    assert frames.t == pytest.approx(2.9)
    assert result['phase'] == 'verify_red' and not result['red_seen_this_round']
    # The last pre-deadline frame had only four reds in the window. A legal
    # but late batch can meet the vote ratio, never extend the stop deadline.
    assert result['red_vote']['red_percent'] == 40
    result = frames.step({'secondary':'red'}, dt=.2, moving=False)
    assert frames.t == pytest.approx(3.1)
    assert result['phase'] == 'fault' and result['esc_us'] == 1500
    assert result['events'] == []


@pytest.mark.parametrize('failure', ['no_sources', 'expired_sources', 'batch_not_fresh'])
def test_image_loss_never_returns_a_forward_command(failure):
    frames = Frames()
    assert frames.step()['action'] == 'search'
    if failure == 'no_sources':
        result = frames.step({})
    elif failure == 'expired_sources':
        result = frames.step({'primary': 'off'}, overrides={'primary': {'captured_s': -1}})
    else:
        result = frames.step(top={'fresh': False})
    assert result['phase'] == 'fault' and result['esc_us'] == 1500

