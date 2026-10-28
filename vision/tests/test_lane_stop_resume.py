"""Task stops end old lane recovery without turning stale frames into new input.

The real flow and lane follower are used throughout.  ScriptedTask replaces only
the task detector/state machine so each test can choose a task brake explicitly.
No actuator or hardware service is started.
"""
import copy

import pytest

from carvision.race_flow import RaceFlowController, load_bundle
from test_cone_turn_controller import ROOT, lane


class ScriptedTask:
    def step(self, row, now):
        return copy.deepcopy(row['scripted_task_decision'])


class FlowInputs:
    def __init__(self, stage):
        self.flow = RaceFlowController(load_bundle(ROOT), 0., initialized=True)
        self.flow._go(stage, 0., 'synthetic task stop/resume scenario')
        self.flow.controller = ScriptedTask()
        self.stage = stage
        self.sequence = 0

    def decision(self, action='search'):
        pulse = (1400 if action == 'brake' else 1500 if action == 'neutral'
                 else self.flow.b['brake' if self.stage == 'crosswalk' else 'traffic']['pwm']['search_us'])
        return {'phase': 'braking' if pulse <= 1500 else 'approach',
                'action': action, 'esc_us': pulse, 'steering_us': 1610,
                'reason': 'scripted task decision'}

    def step(self, now, nav, action='search'):
        self.sequence += 1
        sample = {'captured_s': now, 'sequence': self.sequence, 'navigation': nav,
                  self.stage: {'captured_s': now, 'frame_id': self.sequence,
                               'scripted_task_decision': self.decision(action)}}
        return self.flow.step(sample, now)

    def approach_then_missing(self):
        assert self.step(.1, lane(.1, near=.6, far=.6))['esc_us'] > 1500
        missing = lane(.2, state='missing')
        result = self.step(.2, missing)
        assert result['esc_us'] > 1500
        assert self.flow.guide.missing_since == pytest.approx(.1)
        return missing


@pytest.mark.parametrize('stage', ['crosswalk', 'traffic'])
@pytest.mark.parametrize('stop_action', ['brake', 'neutral'])
def test_task_stop_ends_old_missing_budget_and_fresh_pair_can_resume(stage, stop_action):
    inputs = FlowInputs(stage)
    inputs.approach_then_missing()
    frames = dict(inputs.flow.guide.frames)

    stopped = inputs.step(.3, None, stop_action)
    assert stopped['action'] == stop_action and stopped['esc_us'] <= 1500
    assert inputs.flow.guide.frames == frames
    assert inputs.flow.guide.last_pair is None
    assert inputs.flow.guide.missing_since is None
    assert inputs.flow.guide.last_result is None
    assert inputs.step(1., None, 'neutral')['esc_us'] == 1500

    resumed = inputs.step(1.1, lane(1.1, near=.6, far=.6))
    assert resumed['phase'] == stage and inputs.flow.stage != 'fault'
    assert resumed['esc_us'] > 1500 and resumed['fresh_control']
    assert resumed['effective_steering_gain'] == 1.
    assert inputs.flow.guide.last_pair == pytest.approx(1.1)


@pytest.mark.parametrize('stage', ['crosswalk', 'traffic'])
def test_after_task_stop_missing_cannot_borrow_the_pre_stop_pair(stage):
    inputs = FlowInputs(stage)
    inputs.approach_then_missing()
    assert inputs.step(.3, None, 'brake')['esc_us'] == 1400
    assert inputs.step(1., None, 'neutral')['esc_us'] == 1500

    result = inputs.step(1.1, lane(1.1, state='missing'))
    assert result['phase'] == stage and result['esc_us'] == 1500
    assert result['lane_reference_source'] == 'no_observed_lane'
    assert inputs.flow.guide.last_pair is None
    assert inputs.flow.guide.missing_since == pytest.approx(1.1)


@pytest.mark.parametrize('stage', ['crosswalk', 'traffic'])
def test_task_stop_preserves_replay_memory_and_cached_frame_cannot_restart(stage):
    inputs = FlowInputs(stage)
    old = lane(.2, near=.6, far=.6)
    assert inputs.step(.2, old)['esc_us'] > 1500
    assert inputs.step(.3, None, 'brake')['esc_us'] == 1400

    # Still younger than the 250 ms frame-age ceiling, but already consumed.
    cached = inputs.step(.4, old)
    assert cached['esc_us'] == 1500 and cached['fresh_control'] is False
    assert inputs.flow.guide.last_pair is None
    fresh = inputs.step(.5, lane(.5, near=.6, far=.6))
    assert fresh['fresh_control'] and fresh['esc_us'] > 1500


@pytest.mark.parametrize('stage', ['crosswalk', 'traffic'])
def test_without_task_stop_late_paired_frame_cannot_wash_expired_moving_budget(stage):
    inputs = FlowInputs(stage)
    inputs.approach_then_missing()
    result = inputs.step(.6, lane(.6))
    assert result['phase'] == 'fault' and result['esc_us'] == 1500
    assert '0.4' in result['reason']


@pytest.mark.parametrize('stage', ['crosswalk', 'traffic'])
def test_task_red_brake_with_no_navigation_is_returned_unchanged_before_nav_validation(stage):
    inputs = FlowInputs(stage)
    inputs.approach_then_missing()
    guide = inputs.flow.guide
    actual, frames = guide.actual, dict(guide.frames)
    brake = {**inputs.decision('brake'), 'events': [{'event': 'task_brake'}]}

    result = inputs.flow._guided(None, .3, brake)
    assert result == brake
    assert brake['events'] == [{'event': 'task_brake'}]
    assert guide.actual == actual and guide.frames == frames
    assert guide.last_pair is None and guide.missing_since is None


def test_guidance_boundary_neutral_does_not_suspend_the_moving_lane_budget():
    inputs = FlowInputs('crosswalk')
    assert inputs.step(.1, lane(.1))['esc_us'] > 1500
    inputs.flow.guide.observe_output(1870, .1)
    result = inputs.step(.2, lane(.2, near=.1, far=.1, boundary_correction='right'))
    assert result['boundary_output_withdrawal'] and result['esc_us'] == 1500
    assert inputs.flow.guide.last_pair == pytest.approx(.2)
    assert inputs.flow.guide.last_result is not None

    assert inputs.step(.3, lane(.3, state='missing'))['esc_us'] > 1500
    result = inputs.step(.7, lane(.7))
    assert result['phase'] == 'fault' and '0.4' in result['reason']


def test_guidance_unknown_neutral_does_not_reset_the_recovery_deadline():
    inputs = FlowInputs('crosswalk')
    result = inputs.step(.1, lane(.1, state='missing'))
    assert result['esc_us'] == 1500 and result['lane_reference_source'] == 'no_observed_lane'
    assert inputs.flow.guide.missing_since == pytest.approx(.1)
    result = inputs.step(.6, lane(.6))
    assert result['phase'] == 'fault' and '0.4' in result['reason']
