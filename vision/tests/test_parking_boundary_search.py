"""Observed short white corridor, with unchanged geometric acceptance limits."""
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.cone_boundaries import adaptive_inner_edges
from carvision.cone_road import ConeRoadContext


def detector(thorough=False):
    return ConeRoadContext({'scene': {'surface': 'indoor_white_line_track'}},
                           'secondary', thorough_boundary_search=thorough)


def image():
    # Binary white mask from the actual 2026-10-13 stationary camera frame.
    # Contains no colour photograph, account identity or fabricated edges.
    p = Path(__file__).parent/'fixtures/parking-white-mask-20261013.png'
    return cv2.imdecode(np.frombuffer(p.read_bytes(), np.uint8), 1)


def test_short_actual_corridor_no_longer_depends_on_sampled_endpoint_pair():
    im = image(); zero = np.zeros(im.shape[:2], np.uint8)
    config = detector().lane.config
    before = adaptive_inner_edges(im, zero, config)
    after = adaptive_inner_edges(im, zero, config, complete_small_sets=True)
    assert not before[0] and before[3]['reason'] == 'no_continuous_boundary_pair'
    assert len(after[0]) == len(after[1]) >= 10
    assert np.ptp(np.asarray(after[0])[:, 1]) >= 360*.09
    assert after[3]['hypothesis_policy'] == 'complete_small_set'
    assert after[3]['short_span_checked'] is True


def test_parking_opt_in_selects_actual_continuous_pairs_and_cone_default_unchanged():
    im = image(); zero = np.zeros(im.shape[:2], np.uint8)
    assert detector().thorough_boundary_search is False
    result = detector(True).inspect(im, zero)
    assert result['white_lines_detected']
    assert result['boundary_diagnostics']['hypothesis_policy'] == 'complete_small_set'
    assert result['metric_boundary_verified'] is False


@pytest.mark.parametrize('kind', ['blank', 'single_side', 'horizontal', 'short_posts'])
def test_enumeration_does_not_accept_missing_side_or_non_corridor(kind):
    im = np.zeros((360, 480, 3), np.uint8)
    if kind == 'single_side': cv2.line(im, (125, 180), (10, 250), (255, 255, 255), 5)
    if kind == 'horizontal': cv2.line(im, (20, 200), (460, 200), (255, 255, 255), 5)
    if kind == 'short_posts':
        for x in (120, 360): cv2.line(im, (x, 190), (x, 211), (255, 255, 255), 5)
    left, right, _, _ = adaptive_inner_edges(im, np.zeros(im.shape[:2], np.uint8),
                                            detector().lane.config, complete_small_sets=True)
    assert not left and not right
