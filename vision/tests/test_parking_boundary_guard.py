"""Replay the observed line-loss failure without any hardware output."""
import json
from pathlib import Path

from carvision.parking_entry import ParkingEntryController
from test_parking_entry import settings, sample, RIGHT
from test_parking_entry_run import prepared

ROOT = Path(__file__).resolve().parents[2]


def road():
    return {'white_lines_detected': True,
            'left_inner': [[40., float(y)] for y in range(180, 301, 10)],
            'right_inner': [[440., float(y)] for y in range(180, 301, 10)]}


def frame(i, boundary):
    s = sample(i, q=RIGHT)
    s['parking']['road_context'] = boundary
    return s


def test_live_session_cannot_disable_white_boundary_requirement():
    run, actuator = prepared(require_white_boundary=False)
    assert run.controller is None
    assert run._controller(run.s).require_white_boundary is True
    assert not any(c['esc_us'] > 1500 for c in actuator.commands)


def test_first_missing_boundary_never_starts_driving():
    c = ParkingEntryController(settings(), require_white_boundary=True)
    for i in range(31):
        out = c.step(frame(i, {}), i*.1)
        assert out['esc_us'] == 1500
    assert out['phase'] == 'fault'
    assert '白线' in out['reason']


def test_detected_white_pixels_with_inadequate_span_are_not_accepted():
    c = ParkingEntryController(settings(), require_white_boundary=True)
    boundary = road()
    for side in ('left_inner', 'right_inner'):
        boundary[side] = boundary[side][:3]
    for i in range(31):
        out = c.step(frame(i, boundary), i*.1)
    assert out['phase'] == 'fault' and out['esc_us'] == 1500
    assert out['white_boundary']['reason'] == '白线有效点或纵向覆盖不足'


def test_loss_after_motion_brakes_immediately_and_latches_when_lines_return():
    c = ParkingEntryController(settings(), require_white_boundary=True)
    for i in range(31):
        out = c.step(frame(i, road()), i*.1)
    assert out['esc_us'] > 1500
    out = c.step(frame(31, {}), 3.1)
    assert out['esc_us'] == 1400 and out['phase'] == 'safety_brake'
    for i in range(32, 45):
        out = c.step(frame(i, road()), i*.1)
        assert out['esc_us'] <= 1500
    assert out['phase'] == 'fault' and out['esc_us'] == 1500


def test_real_departure_replay_stops_before_continuing_with_unusable_boundaries():
    evidence = json.loads((ROOT/'vision/tests/fixtures/parking-boundary-loss-20261013.json').read_text(encoding='utf-8'))
    c = ParkingEntryController(settings(), steering_gain=1.25, require_white_boundary=True)
    moved = stopped = False
    for r in evidence['frames']:
        out = c.step(r['sample'], r['now_s'])
        if out['esc_us'] > 1500:
            moved = True
            assert out['white_boundary']['observed'] is True
            assert not stopped
        if moved and out['phase'] in ('safety_brake', 'fault'):
            stopped = True
            assert '白线' in out['reason']
    assert moved and stopped and out['phase'] == 'fault'
