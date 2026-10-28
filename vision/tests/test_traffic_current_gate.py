"""Current credible lamp state must agree with qualified historical green votes."""
from pathlib import Path

import pytest

from carvision.race_flow import RaceFlowController, load_bundle
from carvision.traffic_voting import DualSignalVotes
from test_traffic_voting import frame


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('current', ['red', 'yellow', 'off', 'unknown'])
def test_historical_green_cannot_release_after_current_signal_changes(current):
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(21):
        t = fid/10
        color = 'green' if fid < 20 else current
        result = votes.update({'secondary': frame('secondary', color, fid, t)}, t)
    assert result['green_sources'] == ['secondary']
    assert result['cameras']['secondary']['green_percent'] == 95
    assert result['current_green_sources'] == []
    assert result['red_present'] == (current == 'red')
    assert not result['green_confirmed']


@pytest.mark.parametrize('blocking_camera', ['primary', 'secondary'])
@pytest.mark.parametrize('current', ['red', 'yellow'])
def test_current_stop_color_in_either_view_vetoes_other_qualified_green(blocking_camera, current):
    green_camera = 'secondary' if blocking_camera == 'primary' else 'primary'
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(21):
        t = fid/10
        result = votes.update({
            green_camera: frame(green_camera, 'green', fid, t),
            blocking_camera: frame(blocking_camera, 'off' if fid < 20 else current, fid, t),
        }, t)
    assert result['green_sources'] == result['current_green_sources'] == [green_camera]
    assert not result['red_confirmed']  # One new stop-color frame is below 50%.
    assert not result['green_confirmed']


def test_current_green_must_belong_to_the_same_camera_as_qualified_history():
    votes = DualSignalVotes(1, 50, green_window_s=2)
    for fid in range(21):
        t = fid/10
        result = votes.update({
            'primary': frame('primary', 'green' if fid < 20 else 'off', fid, t),
            'secondary': frame('secondary', 'off' if fid < 20 else 'green', fid, t),
        }, t)
    assert result['green_sources'] == ['primary']
    assert result['cameras']['secondary']['state'] == 'green'
    assert result['current_green_sources'] == []
    assert not result['green_confirmed']
    result = votes.update({
        'primary': frame('primary', 'green', 21, 2.1),
        'secondary': frame('secondary', 'green', 21, 2.1),
    }, 2.1)
    assert result['green_sources'] == result['current_green_sources'] == ['primary']
    assert result['green_confirmed']


def flow_sample(fid, t, color):
    motion = {'valid': True, 'moving': False, 'captured_s': t}
    return {
        'sequence': fid, 'captured_s': t, 'motion': motion,
        'navigation': {
            'experimental': True, 'ready': True, 'captured_s': t, 'frame_id': fid,
            'ego_fraction': .5, 'ego_near_fraction': .5, 'half_envelope': .2,
            'lane_bounds_px': [[100, 380], [50, 430]], 'cones': [], 'uncertain': [],
            'source_camera': 'secondary',
        },
        'traffic': {
            'fresh': True, 'camera_id': 'dual', 'frame_id': fid, 'captured_s': t,
            'image_size': [480, 360], 'motion': motion,
            'camera_observations': {'secondary': frame('secondary', color, fid, t)},
        },
    }


def test_full_flow_stays_stopped_at_current_red_then_advances_on_real_green():
    controller = RaceFlowController(load_bundle(ROOT), 0.)
    controller._go('traffic', 0., 'offline waiting-stage setup')
    # Start at the state reached only after this round's red and stationary
    # confirmation. Exercise the real traffic controller and stage dispatch.
    controller.controller.phase = 'wait_green'
    controller.controller.red_seen = True
    controller.controller.red_source_cameras.add('secondary')
    controller.controller.wait_at = 0.
    for fid in range(21):
        t = fid/10
        result = controller.step(flow_sample(fid, t, 'green' if fid < 20 else 'red'), t)
    assert controller.stage == 'traffic'
    assert result['esc_us'] == 1500
    assert result['signal_state'] == 'red'
    assert result['signal_votes']['red_present']
    assert not any(item['to'] == 'cones' for item in controller.transitions)
    result = controller.step(flow_sample(21, 2.1, 'green'), 2.1)
    assert controller.stage == 'cones'
    assert result['esc_us'] == 1500
    for fid in (22,23):result=controller.step(flow_sample(fid,fid/10,'green'),fid/10)
    assert result['esc_us'] > 1500
    assert [item['to'] for item in controller.transitions] == ['traffic', 'cones']
