"""Inner-edge continuity and alignment regression from real Nov 8 recordings."""
from pathlib import Path
import json
import cv2
import numpy as np
import pytest

from carvision.parking_inner_edges import inner_quad
from carvision.parking_entry import ParkingEntryController
from parking_replay_support import replay
from test_parking_entry import settings

DATA=Path(__file__).parent/'fixtures/parking-handoff-20261108'


def tape():
    q=np.array([[110,310],[370,310],[330,190],[150,190]],np.float32)
    markings=np.zeros((360,480),np.uint8)
    cv2.polylines(markings,[q.astype(np.int32)],True,255,12)
    ground=np.full_like(markings,255);ground[markings>0]=0
    return q,markings,ground


def test_boundary_is_floor_side_not_middle_of_thick_tape():
    q,markings,ground=tape();inner=inner_quad(markings,ground,q)
    assert inner is not None
    assert inner[0,0]>q[0,0]+6 and inner[1,0]<q[1,0]-6
    assert inner[2,1]>q[2,1]+6 and inner[0,1]<q[0,1]-6
    assert cv2.contourArea(inner)<cv2.contourArea(q)


def test_missing_entrance_can_use_three_current_edges_but_missing_side_cannot():
    q,markings,ground=tape()
    markings[296:]=0;ground[296:]=255
    assert inner_quad(markings,ground,q) is None
    assert inner_quad(markings,ground,q,near_observed=False) is not None
    cv2.line(markings,tuple(q[0].astype(int)),tuple(q[3].astype(int)),0,26)
    assert inner_quad(markings,ground,q,near_observed=False) is None


def test_roi_cut_does_not_change_heading_lateral_or_envelope():
    c=ParkingEntryController(settings(),'left')
    full=np.array([[110,310],[370,310],[330,190],[150,190]],np.float32)
    partial=full.copy();partial[0]=[120,280];partial[1]=[360,280]
    a,b=c._geometry(full),c._geometry(partial)
    for k in ('lateral_error','heading_error','steering_error'):
        assert a[k]==pytest.approx(b[k],abs=1e-6)
    assert a['envelope_checks']==b['envelope_checks']


@pytest.fixture(scope='module')
def second_run():
    return replay(DATA/'parking-entry-1794125721517040806')


def test_recorded_contour_gap_keeps_measured_sides_and_far_edge(second_run):
    by_frame={r['frame_id']:r for r in second_run}
    for fid in (3949,3950,3951,3952,3953):
        row=by_frame[fid];intent=row['intent']
        assert intent['geometry_source']=='tracked_bay_edges'
        # Reacquiring the real entrance can replace the former shallow floor
        # cut. The legacy image-only controller must still respect that newer
        # corridor; a tracking-continuity test cannot require forward output
        # when the now-complete observed corridor rejects its cross-section.
        if intent['geometry']['lateral_envelope_estimate_inside']:
            assert intent['esc_us']>=1570 and intent['boundary_wait_cause'] is None
        else:
            assert intent['esc_us']<=1500 and intent['boundary_wait_cause']=='bay_envelope_outside'
        edge=row['sample']['parking']['tracked_bay_boundaries'][0]
        assert edge['boundary_reference']=='inner_edge'
        assert min(edge['edge_support'][1:])>=.72


def test_recorded_heading_gets_bounded_correction_before_last_visible_box(second_run):
    by_frame={r['frame_id']:r['intent'] for r in second_run}
    # Old output only reached 1530 us after the heading error exceeded .45.
    assert by_frame[3944]['steering_us']<1559
    assert by_frame[3945]['steering_us']<1530
    assert all(1450<=r['intent']['steering_us']<=1770 for r in second_run)
    assert all(r['intent']['esc_us']>=1570 for r in second_run if r['intent']['esc_us']>1500)
    assert all(not r['intent']['whole_vehicle_inside_verified'] for r in second_run)


def test_recorded_later_outside_geometry_is_not_hidden_to_force_replay_success(second_run):
    outside=[r for r in second_run if r['intent'].get('boundary_wait_cause')=='bay_envelope_outside']
    assert outside and all(r['intent']['esc_us']<=1500 for r in outside)
    assert all(not r['intent']['done'] for r in second_run)


def test_first_recording_retains_limits_and_can_log_current_boundary_output():
    rows=replay(DATA/'parking-entry-1794125619139223726')
    assert any(r['intent']['geometry_source']=='tracked_bay_edges' for r in rows)
    assert all(r['intent']['esc_us']>=1570 for r in rows if r['intent']['esc_us']>1500)
    assert all(not r['intent']['done'] for r in rows)
