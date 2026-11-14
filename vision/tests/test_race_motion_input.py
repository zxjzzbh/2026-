"""The newest independent motion observation must reach the stage controller."""
from copy import deepcopy

from carvision.race_flow import RaceFlowController
from test_race_flow import bundle
from test_cone_turn_controller import lane


def test_fresh_independent_motion_replaces_old_task_snapshot_without_retiming(bundle):
    controller = RaceFlowController(bundle, 0., recover_gaps=True)
    controller._go('traffic', 0., 'fixture')
    seen = []
    def decide(module, now):
        seen.append(deepcopy(module))
        return {'phase': 'approach', 'action': 'neutral', 'esc_us': 1500,
                'steering_us': 1610, 'events': [], 'reason': 'fixture'}
    controller.controller.step = decide
    old = {'valid': False, 'moving': None, 'captured_s': .1}
    fresh = {'valid': True, 'moving': False, 'captured_s': .95,
             'continuous_still_since_s': .7}
    module = {'frame_id': 10, 'captured_s': .8, 'motion': old}
    sample = {'sequence': 10, 'captured_s': .8, 'navigation': lane(.8, fid=10),
              'traffic': module, 'motion': old}
    controller.step(sample, 1., motion=fresh)
    assert seen[0]['motion'] == fresh
    assert seen[0]['captured_s'] == .8
    assert module['motion'] == old and sample['motion'] == old
