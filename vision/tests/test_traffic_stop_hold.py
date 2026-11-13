"""Traffic stop timing and unknown optical flow; fake observations, no I/O."""
import copy

import pytest

from test_traffic_driving import Run
from test_race_flow import Frames, ROOT
from carvision.race_flow import RaceFlowController, load_bundle


def stop_with_unknown_motion():
    run = Run('off')
    run.until('approach', moving=True)
    first = run.step('red', moving=None)
    assert first['action'] == 'brake'
    while run.c.phase != 'wait_green':
        row = run.step('red', moving=None)
        assert row['phase'] != 'fault'
        assert run.t < 8
    return run


def test_unknown_ground_after_red_uses_full_brake_then_existing_neutral_hold():
    run = Run('off'); run.until('approach', moving=True)
    first = run.step('red', moving=None); start = run.t
    assert first['esc_us'] == 1400 and not first['stop_sequence_complete']
    first_neutral = None
    for _ in range(20):
        row = run.step('red', moving=None, dt=.05)
        if run.t-start < .35-1e-9:
            assert row['action'] == 'brake'
        if row['esc_us'] == 1500 and first_neutral is None:
            first_neutral = run.t
        if row['stop_sequence_complete']:
            assert run.t-first_neutral >= .5-1e-9
            assert not row['stationary_estimated'] and not row['physical_stop_verified']
            assert row['stop_basis'] == 'brake_neutral_timing'
            break
    else:
        pytest.fail('Stop sequence did not finish with unknown motion')


def test_missing_flow_during_red_wait_does_not_fault_or_erase_lamp_cycle():
    run = stop_with_unknown_motion()
    for _ in range(60):
        row = run.step('red', moving=None)
        assert row['phase'] == 'wait_green' and row['esc_us'] == 1500
        assert row['ground_feedback_warning'] and row['red_seen_this_round']
        assert not row['stationary_estimated'] and not row['events']
    row = run.until('done', 'green', moving=None)
    assert row['events'] == [{'event':'speak','text':'红绿灯结束，开始前行'}]
    assert row['esc_us'] == 1500 and not row['physical_stop_verified']


@pytest.mark.parametrize('motion', [
    {'valid':False, 'moving':None, 'reason':'insufficient_points'},
    {'valid':True, 'moving':False, 'captured_s':-100},
    {'valid':True, 'moving':True, 'captured_s':-100},
])
def test_unknown_or_stale_motion_is_diagnostic_not_fake_stationary(motion):
    run = stop_with_unknown_motion()
    for _ in range(40):
        row = run.step('red', motion=copy.deepcopy(motion))
        assert row['phase'] == 'wait_green' and row['esc_us'] == 1500
        assert row['ground_motion_state'] == 'unknown' and not row['stationary_estimated']


def test_real_sliding_rebrakes_and_green_before_new_hold_cannot_finish():
    run = stop_with_unknown_motion()
    for _ in range(10): run.step('green', moving=None)
    for _ in range(3): row = run.step('green', moving=True)
    assert row['action'] == 'brake' and row['brake_pulses'] == 2
    assert not row['stop_sequence_complete']
    for _ in range(4):
        row = run.step('green', moving=None)
        assert row['phase'] != 'done' and not any(e['event']=='speak' for e in row['events'])
    assert row['sliding_unresolved']
    run.until('wait_green', 'green', moving=False)
    assert run.until('done', 'green', moving=False)['esc_us'] == 1500


def test_unknown_cannot_erase_positive_sliding_observation():
    run = stop_with_unknown_motion()
    for _ in range(3): run.step('red', moving=True)
    for _ in range(40):
        row = run.step('green', moving=None)
        assert row['phase'] != 'done' and not row['stop_sequence_complete']
        if row['phase'] == 'fault': break
    assert row['phase'] == 'fault' and '滑动' in row['reason']


def test_brake_budget_is_not_reset_by_returning_to_green_wait():
    run = Run('red'); run.until('wait_green')
    for expected in range(2,7):
        for _ in range(3): row = run.step('red', moving=True)
        assert row['action'] == 'brake' and row['brake_pulses'] == expected
        run.until('wait_green', 'red', moving=False)
    for _ in range(3): row = run.step('red', moving=True)
    assert row['phase'] == 'fault' and row['esc_us'] == 1500 and row['brake_pulses'] == 6


def test_duplicate_ground_capture_does_not_establish_visual_still():
    run = stop_with_unknown_motion(); stamp = run.t+.1
    for _ in range(8):
        row = run.step('red', motion={'valid':True,'moving':False,'captured_s':stamp})
        assert not row['stationary_estimated']
    assert row['stop_sequence_complete']


def test_ground_feedback_can_recover_while_remaining_in_red_wait():
    run = stop_with_unknown_motion()
    for _ in range(7): row = run.step('red', moving=False)
    assert row['phase'] == 'wait_green' and row['stationary_estimated']
    assert row['stop_basis'] == 'visual_still' and not row['ground_feedback_warning']
    row = run.step('red', moving=None)
    assert row['phase'] == 'wait_green' and not row['stationary_estimated']


def test_unconfirmed_red_and_unknown_flow_cannot_complete_on_green():
    run = Run('off'); run.until('approach', moving=True)
    run.step('red', moving=None)
    for _ in range(45):
        row = run.step('green', moving=None)
        assert row['phase'] != 'done' and not any(e['event']=='speak' for e in row['events'])
        if row['phase'] == 'fault': break
    assert row['phase'] == 'fault' and '红灯复核超时' in row['reason']


def test_wait_timeout_and_camera_failure_still_stop():
    run = stop_with_unknown_motion(); run.c.s['max_wait_green_s'] = 15
    row = run.until('fault', 'red', moving=None, limit=160)
    assert '等待绿灯超时' in row['reason']
    run = stop_with_unknown_motion()
    assert run.step('green', moving=None, fresh=False)['phase'] == 'fault'


def test_competition_uses_same_unknown_flow_hold_and_only_advances_on_green():
    controller = RaceFlowController(load_bundle(ROOT), 0., initialized=True)
    controller._go('traffic', 0., 'test')
    frames = Frames(controller)
    original = controller.controller.step
    def unknown_motion(sample, now):
        sample = copy.deepcopy(sample)
        sample['motion'] = {'valid':False,'moving':None,'captured_s':now}
        return original(sample, now)
    controller.controller.step = unknown_motion
    for _ in range(70):
        row = frames.step(color='red')
        assert controller.stage == 'traffic' and row['esc_us'] <= 1500
    assert controller.controller.phase == 'wait_green'
    for _ in range(30):
        row = frames.step(color='green')
        if controller.stage == 'cones': break
    assert controller.stage == 'cones'
