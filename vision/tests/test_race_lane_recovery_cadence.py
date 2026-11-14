"""Frame spacing is distinct from stale pixels during race lane recovery."""
from carvision.cone_evidence import StablePairedLane
from carvision.race_flow import RaceFlowController
from carvision.runtime_timing import RACE_CONTROL_WINDOW_S
from test_cone_turn_controller import lane
from test_race_flow import Frames, bundle


def observed(t, fid):
    return lane(t, fid=fid, lane_rows=[180, 300])


def test_fresh_six_hundred_ms_frames_recover_lane_without_an_endless_reset(bundle):
    race = RaceFlowController(bundle, 0., recover_gaps=True)
    race._go('parking_search', 0., 'test')
    frames = Frames(race)
    assert frames.step(moving=True, nav_override=observed)['esc_us'] >= 1570
    for _ in range(4):
        paused = frames.step(moving=True, nav_override=lambda t, i: lane(t, fid=i, state='missing'))
    assert paused['control_recovering'] and race.control_pause['lane_lost']
    stopped_since = frames.t + .35
    now, fid = frames.t, frames.i
    for index in range(3):
        now += .6
        fid += 1
        sample = {'sequence': fid, 'captured_s': now, 'navigation': observed(now, fid),
                  'motion': {'valid': True, 'moving': False, 'captured_s': now,
                             'continuous_still_since_s': stopped_since}}
        result = race.step(sample, now)
        assert result['esc_us'] == 1500
        assert bool(result.get('control_recovered')) == (index == 2)
    now += .6
    fid += 1
    sample = {'sequence': fid, 'captured_s': now, 'navigation': observed(now, fid),
              'motion': {'valid': True, 'moving': True, 'captured_s': now}}
    assert race.step(sample, now)['esc_us'] >= 1570
    assert race.stage == 'parking_search' and race.control_pause is None


def test_race_frame_spacing_allowance_does_not_accept_stale_frames():
    gate = StablePairedLane(max_gap_s=RACE_CONTROL_WINDOW_S)
    for index, now in enumerate((1., 1.6, 2.2)):
        assert not gate.update(observed(now-.51, index+1), now)
    assert gate.count == 0
    for index, now in enumerate((3., 3.6, 4.2)):
        ready = gate.update(observed(now-.1, index+4), now)
    assert ready and gate.count == 3
    assert not gate.ready(4.61)


def test_other_entry_points_keep_the_original_half_second_spacing():
    gate = StablePairedLane()
    for index, now in enumerate((1., 1.6, 2.2)):
        assert not gate.update(observed(now, index+1), now)
    assert gate.count == 1
