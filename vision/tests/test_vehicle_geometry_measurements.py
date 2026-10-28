"""Human body dimensions feed conservative envelopes, never motion calibration."""
import copy
import json
from pathlib import Path

import pytest

from carvision.cone_planning import load_settings, path_clear, readiness
from carvision.parking_bay import load_settings as load_parking
from carvision.parking_mission import body


ROOT=Path(__file__).resolve().parents[2]


@pytest.fixture
def records():
    cones=load_settings(ROOT/'vision/tests/fixtures/configs/cone-trial-field-20261026.json')
    parking=load_parking(ROOT/'vision/configs/parking-bay.json')
    source=json.loads((ROOT/'vision/configs/vehicle-geometry-measurements-20261025.json').read_text(encoding='utf-8'))
    return cones,parking,source


def test_raw_rear_measurement_stays_separate_from_full_length_conservative_envelope(records):
    cones,parking,source=records
    raw=source['raw_measurements_m'];derived=source['derived_geometry_m'];g=cones['geometry']
    assert raw['rear_axle_to_rear']==.07
    assert derived['rear_axle_to_front']==pytest.approx(raw['wheelbase']+raw['front_axle_to_front'])
    assert derived['overall_length_discrepancy']==pytest.approx(.015)
    assert raw['overall_length']-(raw['wheelbase']+raw['front_axle_to_front']+raw['rear_axle_to_rear'])==pytest.approx(.015)
    effective=max(raw['rear_axle_to_rear'],raw['overall_length']-derived['rear_axle_to_front'])
    assert g['rear_overhang_m']==parking['rear_extent_m']==pytest.approx(effective)
    assert g['rear_axle_to_front_m']+g['rear_overhang_m']==pytest.approx(.42)
    assert not g['measured_geometry']['effective_rear_extent_is_measured']
    assert not parking['measured_geometry']['effective_rear_extent_is_measured']


def test_cone_and_parking_use_the_same_full_width_and_body_envelope(records):
    cones,parking,source=records;g=cones['geometry']
    assert g['vehicle_width_m']==parking['vehicle_width_m']==.30
    assert g['rear_axle_to_front_m']==parking['front_extent_m']==.335
    assert g['rear_overhang_m']==parking['rear_extent_m']==.085
    corners=body({'x_m':0.,'y_m':0.,'yaw_rad':0.},parking)
    assert corners[:,0].max()-corners[:,0].min()==pytest.approx(.30)
    assert corners[:,1].max()-corners[:,1].min()==pytest.approx(.42)
    for metadata in (g['measured_geometry'],parking['measured_geometry']):
        assert (ROOT/metadata['source_record']).is_file()
        assert metadata['wheelbase_m']==.245 and metadata['wheel_track_m']==.24


def test_four_measured_tyre_contact_rectangles_fit_within_conservative_body(records):
    cones,parking,source=records;raw=source['raw_measurements_m']
    for axle_y in (0.,raw['wheelbase']):
        for wheel_x in (-raw['wheel_track']/2,raw['wheel_track']/2):
            for x in (wheel_x-raw['tyre_contact_width']/2,wheel_x+raw['tyre_contact_width']/2):
                for y in (axle_y-raw['tyre_contact_length']/2,axle_y+raw['tyre_contact_length']/2):
                    assert -parking['vehicle_width_m']/2<=x<=parking['vehicle_width_m']/2
                    assert -parking['rear_extent_m']<=y<=parking['front_extent_m']


def synthetic_clearance_settings(cones):
    # These zero uncertainty budgets isolate the envelope calculation only;
    # they do not update any saved calibration or verification flag.
    settings=copy.deepcopy(cones)
    settings['geometry']['clearance_m']=0.
    settings['ground']['max_error_m']=0.
    settings['motion'].update(maximum_tracking_error_m=0.,minimum_turn_radius_m=1.)
    return settings


@pytest.mark.parametrize('part',['rear','side'])
def test_swept_envelope_does_not_shrink_to_raw_rear_or_old_narrow_width(records,part):
    settings=synthetic_clearance_settings(records[0])
    pose=[{'x_m':0.,'y_m':0.,'yaw_rad':0.,'curvature':0.}]
    obstacle={'id':'envelope-probe','x_m':0. if part=='rear' else .169,
              'y_m':-.10 if part=='rear' else .1,'radius_m':.0005}
    assert not path_clear(pose,[obstacle],settings,(-1.,1.))
    unsafe=copy.deepcopy(settings)
    unsafe['geometry']['rear_overhang_m' if part=='rear' else 'vehicle_width_m']=.07 if part=='rear' else .295
    assert path_clear(pose,[obstacle],unsafe,(-1.,1.))


def test_dimensions_do_not_promote_clearance_ground_motion_or_parking_verification(records):
    cones,parking,source=records
    assert cones['geometry']['measured'] is False and cones['geometry']['clearance_m'] is None
    assert cones['ground']['verified'] is False
    assert cones['motion']['verified'] is False and cones['motion']['pose_feedback_verified'] is False
    assert not readiness(cones)['metric_planning_ready']
    assert parking['perception_verified'] is False and parking['ground'] is None
    assert parking['minimum_turn_radius_m'] is None and parking['braking_distance_m'] is None
    assert parking['maximum_pose_error_m'] is None
