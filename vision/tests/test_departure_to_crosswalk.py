"""Departure-to-zebra regressions with fresh observations and fake PWM writes."""
import copy
import os
from pathlib import Path

import pytest

from carvision.brake_parking import load_settings as load_brake
from carvision.cone_trial_output import ConePulseGuard
from carvision.race_flow import RaceFlowController, load_bundle, prepare_bundle
from flow_calibration_fixture import current_crosswalk_sample


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def bundle():
    return load_bundle(Path(os.environ.get('CARVISION_FLOW_TEST_ROOT', ROOT)))


class Feed:
    """Keep secondary frames slower than primary, like the live cached reader."""
    def __init__(self, controller):
        self.controller = controller
        self.time = 0.
        self.sequence = 0
        self.secondary = None
        self.outputs = []
        self.ego = .6

    def step(self, *, dt=.1, board='absent', secondary_new=True,
             zebra=None, moving=False, audio=None):
        self.time = round(self.time + dt, 8)
        self.sequence += 1
        t, i = self.time, self.sequence
        if secondary_new or self.secondary is None:
            self.secondary = {'state': 'absent', 'scene_valid': True,
                              'frame_id': i, 'captured_s': t}
        motion = {'valid': True, 'moving': moving, 'captured_s': t}
        sample = {
            'sequence': i, 'captured_s': t, 'motion': motion,
            'board': {
                'primary': {'state': board, 'scene_valid': True,
                            'frame_id': i, 'captured_s': t},
                'secondary': dict(self.secondary),
            },
            'navigation': {
                'experimental': True, 'ready': True,
                'captured_s': t, 'frame_id': i,
                'ego_fraction': self.ego, 'half_envelope': .2,
                'lane_bounds_px': [[100, 380], [50, 430]],
                'cones': [], 'uncertain': [], 'source_camera': 'primary',
            },
        }
        # None means the active-task reader has not processed this task yet;
        # False is a fresh, genuinely negative zebra observation.
        if zebra is not None:
            sample['crosswalk'] = {
                'fresh': True, 'image_size': [480, 360],
                'frame_id': i, 'captured_s': t,
                'candidate': zebra is not False,
                'far_edge_y_normalized': zebra if zebra is not False else None,
                'motion': motion,
            }
        if 'crosswalk' in sample:
            sample['crosswalk']=current_crosswalk_sample(self.controller,sample['crosswalk'])
        output = self.controller.step(sample, t, audio or {})
        assert output is not None
        self.outputs.append((t, output))
        return output

    def confirm_board(self):
        for _ in range(40):
            assert self.step()['esc_us'] == 1500
            if self.controller.stage == 'wait_board':
                break
        assert self.controller.stage == 'wait_board'
        # The supplied reference algorithm requires .5s of presence and fresh
        # frame pairs. This deliberately replaces the old .02s/one-view setup.
        for i in range(8):
            assert self.step(dt=.1, board='present', secondary_new=True)['esc_us'] == 1500
        assert self.controller.departure.phase == 'wait_removed'
        assert self.controller.departure.seen_cameras == ['primary']

    def remove_board(self, *, zebra=None):
        for i in range(40):
            output = self.step(secondary_new=i % 3 == 0, zebra=zebra)
            if self.controller.stage == 'crosswalk':
                return output
            assert self.controller.stage == 'wait_board'
            assert output['esc_us'] == 1500
        pytest.fail('Fresh clear observations did not complete departure')


def accepted_by_real_guard(controller, outputs):
    writes = []
    settings = {
        'pwm': copy.deepcopy(controller.b['brake']['pwm']),
        'brake_pulse_s': .12, 'brake_release_s': .1,
        'actuator_lease_s': .2,
        'traffic_forward_us':controller.b['traffic']['pwm']['search_us'],
        'cone_forward_us':sorted({min(controller.b['flow']['forward_us'],1570),
                                 min(controller.b['flow']['forward_us'],1580)}),
    }
    guard = ConePulseGuard(settings, lambda channel, pulse: writes.append((channel, pulse)),
                           mode='F/B', right_us=1350, left_us=1870)
    for sequence, (t, output) in enumerate(outputs):
        guard.accept({**output, 'sequence': sequence, 'sent_s': t}, t)
    assert guard.fault is None
    assert writes and all(channel in (3, 4) for channel, _ in writes)
    return writes


def test_fast_asynchronous_board_removal_starts_and_keeps_searching(bundle):
    controller = RaceFlowController(bundle, 0.)
    feed = Feed(controller)
    feed.confirm_board()
    first = feed.remove_board()
    forward = bundle['flow']['forward_us']
    assert first['action'] in ('search','cone_1570','cone_1580') and 1500<first['esc_us']<=forward
    assert 1610<first['steering_us']<1870  # Shared PD corrects proportionally, not by an endpoint jump.
    for _ in range(8):
        output = feed.step(zebra=False, moving=True)
        assert output['phase'] == 'crosswalk'
        assert output['action'] in ('search','cone_1570','cone_1580') and 1500<output['esc_us']<=forward
        assert not output['events']
    assert not any(event.get('id') == 'departure'
                   for _, output in feed.outputs for event in output['events'])
    assert (4, first['esc_us']) in accepted_by_real_guard(controller, feed.outputs)


def test_centered_departure_does_not_add_a_speed_or_arming_penalty(bundle):
    controller=RaceFlowController(bundle,0.);feed=Feed(controller);feed.ego=.5
    feed.confirm_board();first=feed.remove_board()
    assert first['action']=='search' and first['esc_us']==bundle['flow']['forward_us']
    assert first['steering_us']==1610 and controller.stage=='crosswalk'
    accepted_by_real_guard(controller,feed.outputs)


@pytest.mark.parametrize('zebra,stop_stage', [(.5, 'first_detection'), (.75, 'original_operation')])
def test_fresh_zebra_at_handoff_brakes_before_any_forward_output(bundle, zebra, stop_stage):
    controller = RaceFlowController(bundle, 0.)
    feed = Feed(controller)
    feed.confirm_board()
    first = feed.remove_board(zebra=zebra)
    assert first['action'] == 'brake' and first['esc_us'] == 1400
    assert first['stop_stage'] == stop_stage
    assert all(output['esc_us'] <= 1500 for _, output in feed.outputs)
    assert not any(event.get('event') == 'speak'
                   for _, output in feed.outputs for event in output['events'])
    accepted_by_real_guard(controller, feed.outputs)


@pytest.mark.parametrize('stage', ['crosswalk', 'traffic'])
@pytest.mark.parametrize('action,pulse', [('neutral', 1500), ('brake', 1400)])
def test_handoff_preserves_guidance_stop_decisions(bundle, monkeypatch, stage, action, pulse):
    controller = RaceFlowController(bundle, 0.)
    monkeypatch.setattr(controller, '_guided', lambda *args: {
        'action': action, 'esc_us': pulse, 'steering_us': 1610,
        'reason': 'Guidance requires stopping',
    })
    output = controller._advance(stage, 1., 'handoff', {'navigation': {}}, {})
    assert output['action'] == action and output['esc_us'] == pulse
    accepted_by_real_guard(controller, [(1., output)])


def test_departure_is_silent_but_zebra_still_holds_ten_seconds_and_speaks_team_once(bundle):
    controller = RaceFlowController(bundle, 0.)
    feed = Feed(controller)
    feed.confirm_board()
    feed.remove_board()
    assert 1500<feed.step(zebra=False, moving=True)['esc_us']<=bundle['flow']['forward_us']
    braking = feed.step(zebra=.75, moving=True)
    assert braking['action'] == 'brake'
    hold_started = None
    speech = []
    for _ in range(160):
        output = feed.step(zebra=.75, audio={'crosswalk': 'command_sent'})
        for event in output['events']:
            if event.get('event') == 'speak':
                speech.append(event)
                hold_started = feed.time
        if controller.stage == 'traffic':
            break
        assert controller.stage == 'crosswalk' and output['esc_us'] <= 1500
    assert controller.stage == 'traffic' and hold_started is not None
    assert feed.time - hold_started >= 10.
    assert speech == [{'event': 'speak', 'id': 'crosswalk',
                       'text': bundle['flow']['school_team_speech']}]
    assert not any(event.get('id') == 'departure'
                   for _, output in feed.outputs for event in output['events'])
    accepted_by_real_guard(controller, feed.outputs)


def test_search_stops_on_stale_task_observation_after_departure(bundle):
    controller = RaceFlowController(bundle, 0.)
    feed = Feed(controller)
    feed.confirm_board()
    feed.remove_board()
    t = round(feed.time + .1, 8)
    output = controller.step({
        'sequence': feed.sequence + 1, 'captured_s': t,
        'crosswalk': {'captured_s': t - .3},
    }, t)
    assert output['phase'] == 'fault' and output['esc_us'] == 1500
    accepted_by_real_guard(controller, feed.outputs + [(t, output)])


def test_flow_search_change_keeps_independent_zebra_configuration_and_other_contracts(bundle):
    root = Path(os.environ.get('CARVISION_FLOW_TEST_ROOT', ROOT))
    path = root / 'vision/configs/brake-parking.json'
    independent = load_brake(path)
    original = copy.deepcopy(bundle)
    prepared = prepare_bundle(bundle)
    assert bundle == original
    assert independent['pwm']['search_us'] == independent['pwm']['creep_us'] == 1575
    assert load_brake(path) == independent
    assert prepared['brake']['pwm']['search_us'] == bundle['flow']['forward_us']
    assert prepared['brake']['pwm']['creep_us'] == min(1575, bundle['flow']['forward_us'])
    assert prepared['brake']['references'] == independent['references']
    assert prepared['brake']['hold_s'] == 10
    assert prepared['traffic']['green_action'] == 'speech_only'
    assert prepared['flow']['payment_window_s'] == 30
    assert prepared['flow']['max_frame_age_s'] == .25
    assert prepared['flow']['page_lease_s'] == .5
    assert prepared['parking'] == bundle['parking']
