"""Acquisition on Nov 9 raw images, with background and occupancy negatives."""
from pathlib import Path
import json
import cv2
import numpy as np
import pytest
from carvision.parking_bay import detect_parking
from carvision.parking_bay_console import BaySelection
from carvision.parking_entry_detection import board_adjacent_entry
from parking_replay_support import replay

DATA = Path(__file__).parent/'fixtures/parking-image-search-20261109'
RUNS = ['parking-entry-1794230003918909831', 'parking-entry-1794230062134026601']


def raw_frame(run=RUNS[1], fid=2557):
    folder = DATA/run
    frames = [json.loads(row) for row in (folder/'frames.jsonl').read_text().splitlines()]
    frame = next(row for row in frames if row['frame_id'] == fid)
    with (folder/'frames.mjpg').open('rb') as stream:
        stream.seek(frame['offset'])
        image = cv2.imdecode(np.frombuffer(stream.read(frame['length']), np.uint8), 1)
    settings = json.loads((folder/'settings.json').read_text(encoding='utf8'))['parking']
    return image, settings


@pytest.mark.parametrize('mirror', [False, True])
def test_actual_entrance_below_board_crop_remains_detectable(mirror):
    image, settings = raw_frame()
    if mirror:
        image = cv2.flip(image, 1)
    observation, _ = detect_parking(image, settings)
    assert len(observation['entry_candidates']) == 1
    candidate = observation['entry_candidates'][0]
    assert candidate['id'] == ('right' if mirror else 'left')
    assert min(candidate['edge_support']) >= .7 and candidate['ground_fraction'] >= .9
    assert not observation['diagnostics']['entry_search']['cropped_to_board']
    assert not candidate['occupancy_verified'] and candidate['polygon_m'] is None
    selector = BaySelection()
    assert not selector.update(observation, 1, 1.0, 1.0)['confirmed']
    assert not selector.update(observation, 2, 1.25, 1.25)['confirmed']
    selection = selector.update(observation, 3, 1.5, 1.5)
    assert selection['confirmed'] and selection['selected_slot'] == candidate['id']
    assert not selection['layout_complete'] and not selection['autonomous_entry_allowed']
    assert not selection['hardware_output']


@pytest.mark.parametrize('kind', ['cabinet', 'white_panel', 'blue_object'])
def test_background_objects_do_not_replace_the_current_ground_bay(kind):
    image, settings = raw_frame()
    color = {'cabinet': (170, 180, 180), 'white_panel': (245, 245, 245),
             'blue_object': (230, 90, 40)}[kind]
    cv2.rectangle(image, (15, 15), (100, 115), color, -1)
    observation, _ = detect_parking(image, settings)
    assert len(observation['entry_candidates']) == 1
    q = np.asarray(observation['entry_candidates'][0]['corners_px'])
    assert q[:, 1].min() > 200 and q[:, 1].max() < 260


@pytest.mark.parametrize('kind', ['dark_obstacle', 'white_panel', 'blue_obstacle'])
def test_occupied_bay_is_not_declared_clear(kind):
    image, settings = raw_frame()
    colors = {'dark_obstacle': (0, 0, 0), 'white_panel': (245, 245, 245),
              'blue_obstacle': (230, 90, 40)}
    cv2.rectangle(image, (153, 217), (200, 237), colors[kind], -1)
    observation, _ = detect_parking(image, settings)
    assert not observation['entry_candidates']
    assert not any(row['availability'] == 'clear' for row in observation['slots'])


@pytest.mark.parametrize('kind', ['plain_floor', 'missing_edge', 'border', 'wrong_board'])
def test_floor_mask_and_partial_lines_cannot_fabricate_a_parking_enclosure(kind):
    edges = np.zeros((360, 480), np.uint8)
    ground = np.full_like(edges, 255)
    blue = np.zeros_like(edges)
    q = np.array([[100, 245], [214, 238], [222, 211], [152, 213]], np.int32)
    hint = {'suggested_slot': 'left', 'blocked_side': 'right',
            'boards': [{'side': 'right', 'bbox_px': [234, 155, 358, 236], 'area_fraction': .05}]}
    if kind == 'missing_edge':
        cv2.polylines(edges, [q[1:]], False, 255, 3)
    elif kind == 'border':
        q[:2, 1] = 359
        cv2.polylines(edges, [q], True, 255, 3)
    elif kind == 'wrong_board':
        cv2.polylines(edges, [q], True, 255, 3)
        hint['boards'][0]['bbox_px'] = [390, 155, 479, 236]
    assert board_adjacent_entry(edges, ground, blue, hint) == []


def test_contrast_confirmation_restarts_after_board_conflict_or_enclosure_shift():
    image, settings = raw_frame()
    observation, _ = detect_parking(image, settings)
    hint = observation['direction_hint']
    conflicting_board = {**hint['boards'][0], 'side': hint['suggested_slot']}
    conflict = {**observation, 'direction_hint': {**hint, 'boards': [*hint['boards'], conflicting_board]}}
    candidate, = observation['entry_candidates']
    shifted_corners = (np.asarray(candidate['corners_px']) + [25, 0]).tolist()
    shifted = {**observation, 'entry_candidates': [{**candidate, 'corners_px': shifted_corners}]}
    selector = BaySelection()
    for frame_id, view in enumerate([observation, observation, conflict, observation, observation, shifted, observation]):
        captured = 1. + frame_id * .25
        assert not selector.update(view, frame_id, captured, captured)['confirmed']


def test_color_and_contrast_observations_can_confirm_the_same_side():
    image, settings = raw_frame()
    contrast, _ = detect_parking(image, settings)
    side = contrast['direction_hint']['suggested_slot']
    blocked = contrast['direction_hint']['blocked_side']
    color = {'layout_found': True, 'layout_complete': True, 'slots': [
        {'id': side, 'availability': 'clear'}, {'id': blocked, 'availability': 'blocked'}]}
    selector = BaySelection()
    assert not selector.update(contrast, 1, 1., 1.)['confirmed']
    assert not selector.update(color, 2, 1.25, 1.25)['confirmed']
    assert selector.update(contrast, 3, 1.5, 1.5)['confirmed']


@pytest.fixture(scope='module', params=RUNS)
def recorded(request):
    return request.param, replay(DATA/request.param, staged=True)


def test_two_failed_runs_acquire_the_visible_bay_and_keep_continuity(recorded):
    name, rows = recorded
    # Source recordings did not move. Outputs are counterfactual commands,
    # never evidence of a successful new driving trajectory or parking result.
    assert len(rows) == 45
    assert sum(bool(row['sample']['parking']['entry_candidates']) for row in rows) >= 43
    active = [row['intent'] for row in rows if row['intent']['phase'] != 'arming']
    assert len(active) == 14 and all(row['phase'] == 'approach' for row in active)
    assert all(row['esc_us'] >= 1570 for row in active)
    assert all(not row['done'] and not row['whole_vehicle_inside_verified'] for row in active)


def test_actual_desk_background_is_not_a_parking_entrance():
    image, settings = raw_frame(RUNS[0], 1932)
    observation, _ = detect_parking(image, settings)
    assert not observation['entry_candidates'] and not observation['layout_found']
