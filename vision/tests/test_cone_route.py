"""Unknown route capability must not become fabricated metric feasibility."""
import copy
import json
from pathlib import Path

import pytest

from carvision.cone_route import assess_route


ROOT=Path(__file__).parents[1]


def image_case():
    settings=json.loads((ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json').read_text(encoding='utf-8'))
    nav={'source_camera':'secondary','line_state':'paired','ego_near_fraction':.63849,
         'lane_bounds_px':[[136,269],[3,369]],'metric_boundary_verified':False}
    target={'id':'secondary-2','fraction':.53468,'reference_distance_m':1.25}
    return settings,nav,target


def test_current_snapshot_does_not_verify_a_cross_side_route_or_stop_the_car():
    settings,nav,target=image_case();before=copy.deepcopy((settings,nav,target))
    result=assess_route(settings,nav,target,'left',.26809)
    assert result['status']=='unknown' and result['physical_reachable'] is None
    assert result['cross_side_requested'] is True and result['goal_space']=='lane_fraction'
    for name in ('current_ground_calibration','current_fixed_camera','verified_turn_radius','verified_speed_feedback'):
        assert result['capabilities'][name] is False
    assert not any(key in result for key in ('action','stop','esc_us','steering_us','can_drive'))
    assert (settings,nav,target)==before


def test_same_side_endpoint_is_still_not_a_dynamic_clearance_proof():
    settings,nav,target=image_case()
    result=assess_route(settings,nav,target,'right',.7)
    assert result['cross_side_requested'] is False
    assert result['dynamic_verified'] is False and result['physical_reachable'] is None


@pytest.mark.parametrize('change',[{'near_field_continuation':True},
    {'position_source':'last_valid_ground_observation'},{'fraction':float('nan')}])
def test_historical_or_invalid_lane_target_cannot_supply_cross_side_evidence(change):
    settings,nav,target=image_case();target.update(change)
    result=assess_route(settings,nav,target,'left',.26809)
    assert result['cross_side_requested'] is None and result['status']=='unknown'


def test_current_pixels_are_not_compared_with_lane_fractions_or_converted_to_metres():
    settings,nav,target=image_case()
    result=assess_route(settings,nav,target,'left',{'space':'image_fraction','fraction':.25})
    assert result['goal_space']=='image_fraction' and result['cross_side_requested'] is None
    assert result['reported_distance_is_metric_route'] is False
    assert result['physical_reachable'] is None


def test_old_reference_distance_and_reported_radius_never_promote_motion_verification():
    settings,nav,target=image_case()
    settings['low_speed_test']={'reported_turn_radius_m':12.03,'reported_displacement_m':.63,
        'reported_drive_s':1.5,'estimate_verified':False}
    target['reference_distance_m']=100.
    result=assess_route(settings,nav,target,'left',.26809)
    assert not result['capabilities']['verified_turn_radius']
    assert not result['capabilities']['metric_target_coordinates']
    assert result['status']=='unknown'


def test_even_complete_metric_config_does_not_turn_an_image_goal_into_a_swept_path():
    settings,nav,target=image_case()
    settings['ground'].update(verified=True,current_pose_verified=True)
    settings['ground']['calibration_evidence'].update(candidate_only=False,camera='secondary')
    settings['motion'].update(verified=True,pose_feedback_verified=True,minimum_turn_radius_m=.6)
    settings['driving'].update(fixed_camera_confirmed=True,steering_verified=True,
        odometry_verified=True,steering_radius_right_cm=60.,steering_radius_left_cm=-60.)
    nav['motion']={'valid':True,'speed_mps':.2,'pose':{'x_m':0.,'y_m':0.,'yaw_rad':0.}}
    target.update(x_m=.1,y_m=1.)
    result=assess_route(settings,nav,target,'left',.26809)
    assert all(result['capabilities'].values())
    assert result['status']=='unknown' and result['dynamic_verified'] is False
    assert result['reason_codes']==['verified_metric_goal_path','dynamic_body_sweep']


def test_other_camera_verification_cannot_authorize_secondary_geometry():
    settings,nav,target=image_case()
    settings['ground']['calibration_evidence']['camera']='primary'
    assert assess_route(settings,nav,target,'left',.26809)['capabilities']['same_camera_reference'] is False


@pytest.mark.parametrize('goal',[None,True,-.1,1.1,float('inf'),'0.3'])
def test_unrecognized_goal_produces_serializable_unknown_without_motion_action(goal):
    result=assess_route({},None,None,None,goal)
    assert result['status']=='unknown' and result['goal_fraction'] is None
    json.dumps(result,allow_nan=False)
