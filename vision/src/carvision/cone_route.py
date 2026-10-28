"""Route capability diagnostics, not a dynamic planner or movement gate.

A numeric goal is a normalized lane fraction. A caller using image fractions
must label it explicitly as {'space': 'image_fraction', 'fraction': value}.
Neither kind is converted into metres. This function never declares a route
physically feasible: a verified metric path and its swept body envelope have
not been supplied/evaluated by this interface. Unknown is not a stop request.
"""
import math


def _mapping(value):
    return value if isinstance(value,dict) else {}


def _number(value):
    return type(value) in (int,float) and math.isfinite(value)


def _point(value,keys):
    value=_mapping(value)
    return all(_number(value.get(key)) for key in keys)


def assess_route(settings,nav,target,side,goal):
    """Return pure, conservative evidence about an image-guidance endpoint.

    Configuration flags are reported, never promoted or rewritten. Even with
    all flags true, an image endpoint is not a metre-based collision-free path.
    The caller retains its existing best-effort and independent safety policy.
    """
    settings=_mapping(settings);nav=_mapping(nav);target=_mapping(target)
    ground=_mapping(settings.get('ground'));driving=_mapping(settings.get('driving'))
    motion=_mapping(settings.get('motion'));feedback=_mapping(nav.get('motion'))
    sample=_mapping(nav.get('sample'));evidence=_mapping(ground.get('calibration_evidence'))
    goal_space='lane_fraction';fraction=goal
    if isinstance(goal,dict):
        goal_space=goal.get('space','unknown');fraction=goal.get('fraction')
    fraction=fraction if _number(fraction) and 0<=fraction<=1 else None
    if fraction is None:goal_space='unknown'
    current_lane=(goal_space=='lane_fraction' and bool(nav.get('lane_bounds_px'))
        and nav.get('line_state','paired')!='missing' and not target.get('near_field_continuation')
        and target.get('position_source')!='last_valid_ground_observation'
        and _number(target.get('fraction')) and 0<=target['fraction']<=1)
    near=nav.get('ego_near_fraction',nav.get('ego_fraction'))
    cross_side=None
    if current_lane and _number(near) and 0<=near<=1:
        cross_side=(near-target['fraction'])*(fraction-target['fraction'])<0

    radius=motion.get('minimum_turn_radius_m')
    right=driving.get('steering_radius_right_cm');left=driving.get('steering_radius_left_cm')
    camera=evidence.get('camera',ground.get('camera'))
    pose=feedback.get('pose',sample.get('pose'))
    capabilities={
        'current_ground_calibration':(ground.get('verified') is True
            and ground.get('current_pose_verified') is not False
            and evidence.get('candidate_only') is not True),
        'current_fixed_camera':driving.get('fixed_camera_confirmed') is True,
        'same_camera_reference':(camera is not None and camera==nav.get('source_camera')),
        'rear_axle_metric_frame':ground.get('coordinate_frame')=='rear_axle_x_right_y_forward',
        'verified_turn_radius':(motion.get('verified') is True and driving.get('steering_verified') is True
            and _number(radius) and radius>0 and _number(right) and right>0 and _number(left) and left<0),
        'verified_speed_feedback':(driving.get('odometry_verified') is True
            and motion.get('pose_feedback_verified') is True and feedback.get('valid') is True
            and _number(feedback.get('speed_mps'))),
        'metric_vehicle_pose':feedback.get('valid') is True and _point(pose,('x_m','y_m','yaw_rad')),
        'metric_target_coordinates':_point(target,('x_m','y_m')),
    }
    missing=[key for key,available in capabilities.items() if not available]
    # A forward reference distance alone does not specify target lateral
    # position, vehicle yaw, same-frame coordinates, or a swept route.
    missing.extend(['verified_metric_goal_path','dynamic_body_sweep'])
    return {'status':'unknown','assessment_kind':'capability_only',
            'physical_reachable':None,'dynamic_verified':False,
            'reason':'A normalized endpoint is not a verified metric trajectory.',
            'reason_codes':missing,'capabilities':capabilities,
            'source_camera':nav.get('source_camera'),'target_id':target.get('id'),
            'side':side if side in ('left','right') else None,
            'goal_space':goal_space,'goal_fraction':fraction,
            'cross_side_requested':cross_side,
            'cross_side_basis':'nominal_lane_fractions_at_different_image_rows; not measured vehicle pose',
            'capability_basis':'persisted_settings_and_current_nav_only',
            'reported_distance_is_metric_route':False}
