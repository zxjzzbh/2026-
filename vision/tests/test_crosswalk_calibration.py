"""Pure schema/interpolation tests; every coordinate below is synthetic.

Fixture dimensions and verification flags test software contracts only. No
fixture is saved as an enabled vehicle configuration or actual calibration.
"""
import copy
import json

import pytest

from carvision.crosswalk_calibration import (
    CalibrationError, DISTANCE_REFERENCE, IMAGE_FEATURE, IMAGE_FEATURE_NEAR,
    estimate_distance, load_profile, reference_y, validate_profile,
)


def point(distance, y):
    return {'distance_m': distance, 'far_edge_y_normalized': y,
            'measured': True, 'evidence': 'synthetic-fixture-only.png'}


def view(identity, references):
    return {'identity': identity, 'image_size': [480, 360],
            'pose': {'fixed': True, 'verified': True, 'description': 'synthetic fixed test pose'},
            'captured_on': '2026-10-24', 'verified': True, 'references': references}


@pytest.fixture
def profile():
    return {
        'schema_version': 1, 'profile_id': 'synthetic-tests-only',
        'enabled': True, 'verified': True,
        'distance_reference': DISTANCE_REFERENCE, 'image_feature': IMAGE_FEATURE,
        'paper_spec': {'id': 'synthetic-equal-strip-row', 'width_m': .1,
                       'height_m': .2, 'same_spec_confirmed': True},
        'cameras': {
            'primary': view('synthetic-near-camera', [point(.45, .54), point(.40, .60), point(.25, .82)]),
            'secondary': view('synthetic-far-camera', [point(.65, .48), point(.50, .56), point(.40, .64)]),
        },
        'overlap': {'verified': True, 'reference_distance_m': .4,
                    'evidence': 'synthetic-same-position-pair-only'},
    }


def test_complete_independent_tables_are_ready_without_mutating_input(profile):
    before = copy.deepcopy(profile)
    checked = validate_profile(profile)
    assert checked['ready'] is True and checked['calibration_ready'] is True
    assert checked['errors'] == []
    assert checked['overlap_range_m'] == [.4, .45]
    assert checked['common_measured_distances_m'] == [.4]
    assert profile == before
    checked['cameras']['primary']['references'][0]['distance_m'] = 9.
    assert profile == before
    json.dumps(validate_profile(profile), allow_nan=False)


def test_same_pixel_row_uses_its_own_camera_table(profile):
    near = estimate_distance(profile, 'primary', .60)
    far = estimate_distance(profile, 'secondary', .60)
    assert near['distance_m'] == .4 and near['measured'] is True
    assert far['distance_m'] == pytest.approx(.45) and far['measured'] is False
    assert near['camera_id'] == 'primary' and far['camera_id'] == 'secondary'
    assert near['camera_identity'] != far['camera_identity']
    assert near['source'] == 'dual_camera_measured_point'
    assert far['source'] == 'dual_camera_in_range_interpolation'
    assert near['physical_stop_verified'] is False and far['extrapolated'] is False


def test_interpolation_and_inverse_preserve_camera_provenance(profile):
    result = estimate_distance(profile, 'primary', .71)
    assert result['distance_m'] == pytest.approx(.325)
    assert result['valid_distance_range_m'] == [.25, .45]
    assert result['profile_id'] == profile['profile_id']
    assert not result['measured']
    inverse = reference_y(profile, 'primary', .325)
    assert inverse['far_edge_y_normalized'] == pytest.approx(.71)
    assert not inverse['measured'] and not inverse['extrapolated']
    exact = reference_y(profile, 'secondary', .5)
    assert exact['far_edge_y_normalized'] == .56 and exact['measured'] is True


@pytest.mark.parametrize('camera,index', [('primary', 0), ('primary', -1),
                                         ('secondary', 0), ('secondary', -1)])
def test_exact_endpoints_are_allowed_without_extrapolation(profile, camera, index):
    row = profile['cameras'][camera]['references'][index]
    assert estimate_distance(profile, camera, row['far_edge_y_normalized'])['distance_m'] == row['distance_m']
    assert reference_y(profile, camera, row['distance_m'])['far_edge_y_normalized'] == row['far_edge_y_normalized']


@pytest.mark.parametrize('camera,y', [('primary', .5), ('primary', .83),
                                     ('secondary', .47), ('secondary', .8)])
def test_no_extrapolation_even_when_another_camera_covers_the_pixel_value(profile, camera, y):
    with pytest.raises(CalibrationError, match='estimate.range'):
        estimate_distance(profile, camera, y)


@pytest.mark.parametrize('camera,distance', [('primary', .6), ('secondary', .25),
                                            ('primary', .249999), ('secondary', .650001)])
def test_trigger_cannot_borrow_other_camera_range(profile, camera, distance):
    with pytest.raises(CalibrationError, match='estimate.range'):
        reference_y(profile, camera, distance)


def test_increasing_recording_order_is_accepted_but_not_combined_across_cameras(profile):
    for camera in profile['cameras'].values():
        camera['references'].reverse()
    assert validate_profile(profile)['ready']
    assert estimate_distance(profile, 'primary', .71)['distance_m'] == pytest.approx(.325)


@pytest.mark.parametrize('rows', [
    [point(.45, .54), point(.45, .60), point(.25, .82)],  # duplicate distance
    [point(.45, .54), point(.40, .54), point(.25, .82)],  # duplicate pixel
    [point(.45, .54), point(.25, .82), point(.40, .60)],  # nonmonotonic order
    [point(.25, .54), point(.40, .60), point(.45, .82)],  # physically reversed relationship
])
def test_noninvertible_reference_tables_are_rejected(profile, rows):
    profile['cameras']['primary']['references'] = rows
    with pytest.raises(CalibrationError, match='严格反向单调'):
        validate_profile(profile)


def test_positive_mathematical_overlap_without_same_distance_measurements_is_insufficient(profile):
    profile['cameras']['primary']['references'] = [point(.45, .54), point(.25, .82)]
    with pytest.raises(CalibrationError) as error:
        validate_profile(profile)
    assert any('overlap.common_point' in s for s in error.value.errors)


def test_touching_measured_endpoints_do_not_prove_a_handoff_interval(profile):
    profile['cameras']['primary']['references'] = [point(.4, .60), point(.25, .82)]
    with pytest.raises(CalibrationError, match='overlap.range'):
        validate_profile(profile)


def test_disjoint_ranges_cannot_be_claimed_as_a_continuous_twenty_five_to_sixty_five_cm_model(profile):
    profile['cameras']['primary']['references'] = [point(.3, .70), point(.25, .82)]
    profile['cameras']['secondary']['references'] = [point(.65, .48), point(.5, .56), point(.4, .64)]
    with pytest.raises(CalibrationError, match='overlap.range'):
        validate_profile(profile)


@pytest.mark.parametrize('field,value', [('verified', False), ('evidence', ''),
                                        ('reference_distance_m', .425),
                                        ('reference_distance_m', .7)])
def test_overlap_claim_requires_explicit_verified_evidence_at_a_shared_measured_point(profile, field, value):
    profile['overlap'][field] = value
    with pytest.raises(CalibrationError, match='overlap'):
        validate_profile(profile)


def test_a_single_suspected_primary_point_stays_a_disabled_draft(profile):
    profile.update(enabled=False, verified=False)
    near = profile['cameras']['primary']
    near['references'] = [point(.25, .82)]
    near['references'][0]['measured'] = False
    near['verified'] = False
    profile['overlap']['verified'] = False
    checked = validate_profile(profile)
    assert checked['ready'] is False and checked['calibration_ready'] is False
    assert any('至少需要两个' in s for s in checked['errors'])
    assert any('疑似值' in s for s in checked['errors'])
    assert any('overlap' in s for s in checked['errors'])
    assert any('不可用于运动' in s for s in checked['errors'])
    with pytest.raises(CalibrationError):
        estimate_distance(checked, 'primary', .82)
    profile['enabled'] = True
    with pytest.raises(CalibrationError):
        validate_profile(profile)


def test_two_cameras_with_one_point_each_cannot_form_a_two_point_model(profile):
    profile['cameras']['primary']['references'] = [point(.25, .82)]
    profile['cameras']['secondary']['references'] = [point(.65, .48)]
    with pytest.raises(CalibrationError, match='同一相机至少需要两个'):
        validate_profile(profile)


def test_valid_disabled_profile_is_never_motion_ready(profile):
    profile['enabled'] = False
    checked = validate_profile(profile)
    assert checked['calibration_ready'] is True
    assert checked['ready'] is False
    with pytest.raises(CalibrationError, match='禁用草稿'):
        validate_profile(profile, require_ready=True)
    with pytest.raises(CalibrationError, match='禁用草稿'):
        reference_y(profile, 'primary', .4)


@pytest.mark.parametrize('field,value', [('distance_reference', 'camera_to_paper_m'),
                                        ('image_feature', 'paper_near_edge_y'),
                                        ('verified', False)])
def test_global_measurement_semantics_and_verification_are_explicit(profile, field, value):
    profile[field] = value
    with pytest.raises(CalibrationError, match=field):
        validate_profile(profile)


@pytest.mark.parametrize('field,value', [('width_m', 0), ('height_m', float('nan')),
                                        ('same_spec_confirmed', False), ('id', '')])
def test_same_paper_specification_is_required(profile, field, value):
    profile['paper_spec'][field] = value
    with pytest.raises(CalibrationError, match='paper_spec'):
        validate_profile(profile)


@pytest.mark.parametrize('field,value', [('image_size', [640, 480]),
                                        ('image_size', [480., 360.]), ('identity', ''),
                                        ('verified', False), ('captured_on', '2026-02-30'),
                                        ('captured_on', '20261024')])
def test_camera_metadata_cannot_silently_inherit_old_pose_or_resolution(profile, field, value):
    profile['cameras']['primary'][field] = value
    with pytest.raises(CalibrationError, match='cameras.primary'):
        validate_profile(profile)


def test_both_roles_cannot_name_the_same_camera(profile):
    profile['cameras']['secondary']['identity'] = profile['cameras']['primary']['identity']
    with pytest.raises(CalibrationError, match='同一物理摄像头'):
        validate_profile(profile)


@pytest.mark.parametrize('field,value', [('fixed', False), ('verified', False),
                                        ('description', ''), ('pan_us', True), ('tilt_us', 3000)])
def test_current_fixed_pose_needs_its_own_confirmation(profile, field, value):
    profile['cameras']['primary']['pose'][field] = value
    with pytest.raises(CalibrationError, match='pose'):
        validate_profile(profile)


@pytest.mark.parametrize('field,value', [('distance_m', 0), ('distance_m', True),
                                        ('distance_m', float('inf')),
                                        ('far_edge_y_normalized', float('nan')),
                                        ('far_edge_y_normalized', 1.),
                                        ('measured', False), ('evidence', '')])
def test_measurements_require_finite_real_values_and_traceable_evidence(profile, field, value):
    profile['cameras']['primary']['references'][0][field] = value
    with pytest.raises(CalibrationError, match='references'):
        validate_profile(profile)


@pytest.mark.parametrize('value', [True, None, float('nan'), float('inf'), '.6'])
def test_estimates_reject_non_numeric_or_nonfinite_queries(profile, value):
    with pytest.raises(CalibrationError):
        estimate_distance(profile, 'primary', value)
    with pytest.raises(CalibrationError):
        reference_y(profile, 'primary', value)


def test_unknown_camera_cannot_choose_an_arbitrary_fallback(profile):
    with pytest.raises(CalibrationError, match='camera'):
        estimate_distance(profile, 'third', .6)


@pytest.mark.parametrize('value', [None, [], {'schema_version': True, 'enabled': False},
                                 {'schema_version': 2, 'enabled': False},
                                 {'schema_version': 1, 'enabled': 'false'}])
def test_profile_envelope_is_unambiguous_even_for_disabled_drafts(value):
    with pytest.raises(CalibrationError):
        validate_profile(value)


def test_absent_file_alone_can_select_legacy_and_explicit_requirement_rejects_it(tmp_path):
    missing = tmp_path / 'absent.json'
    assert load_profile(missing) is None and load_profile(None) is None
    with pytest.raises(CalibrationError, match='不存在'):
        load_profile(missing, require_ready=True)
    with pytest.raises(CalibrationError, match='未提供'):
        load_profile(None, require_ready=True)


def test_existing_enabled_invalid_file_never_falls_back_to_old_calibration(tmp_path, profile):
    profile['cameras']['primary']['references'] = [point(.25, .82)]
    path = tmp_path / 'incomplete.json'
    path.write_text(json.dumps(profile), encoding='utf-8')
    with pytest.raises(CalibrationError, match='至少需要两个'):
        load_profile(path)


def test_loading_valid_profile_and_disabled_draft_preserves_source_bytes(tmp_path, profile):
    path = tmp_path / 'optional-profile.json'
    path.write_text(json.dumps(profile), encoding='utf-8-sig')
    original = path.read_bytes()
    assert load_profile(path)['ready']
    assert path.read_bytes() == original
    profile.update(enabled=False, verified=False)
    path.write_text(json.dumps(profile), encoding='utf-8')
    draft = load_profile(path)
    assert draft is not None and draft['ready'] is False and draft['errors']
    with pytest.raises(CalibrationError):
        load_profile(path, require_ready=True)


@pytest.mark.parametrize('content', ['{broken', 'null', '[1,2,3]'])
def test_existing_malformed_file_is_not_treated_as_legacy(tmp_path, content):
    path = tmp_path / 'bad.json'
    path.write_text(content, encoding='utf-8')
    with pytest.raises(CalibrationError):
        load_profile(path)


def test_existing_directory_is_not_treated_as_absent_config(tmp_path):
    with pytest.raises(CalibrationError, match='无法读取'):
        load_profile(tmp_path)


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('path', [
    ('cameras',), ('cameras', 'primary'), ('cameras', 'primary', 'image_size'),
    ('cameras', 'primary', 'pose'), ('cameras', 'primary', 'references'),
    ('paper_spec',), ('overlap',),
])
@pytest.mark.parametrize('value', [None, True, False, 1, .5, float('nan'), [], {}])
def test_malformed_nested_json_is_a_report_or_calibration_error_never_a_type_error(
        profile, enabled, path, value):
    profile['enabled'] = enabled
    node = profile
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value
    if enabled:
        with pytest.raises(CalibrationError):
            validate_profile(profile)
    else:
        report = validate_profile(profile)
        assert report['ready'] is False and report['errors']


@pytest.mark.parametrize('field,value', [('distance_m', 10 ** 400),
                                        ('far_edge_y_normalized', 10 ** 400)])
def test_extremely_large_integers_do_not_escape_as_overflow_errors(profile, field, value):
    profile['cameras']['primary']['references'][0][field] = value
    with pytest.raises(CalibrationError):
        validate_profile(profile)
    profile['enabled'] = False
    assert validate_profile(profile)['errors']


@pytest.mark.parametrize('rows', [
    [point(.45, .54), None, point(.4, .6)],
    [point(.45, .6), point(.4, .54), point(.25, .82)],
    [point(.4, .54), point(.4, .6)],
    [point(.45, .54), {**point(.4, .6), 'measured': False}, point(.25, .82)],
    [point(.45, .54), {**point(.4, .6), 'evidence': ''}, point(.25, .82)],
])
def test_bad_tables_do_not_leave_apparently_usable_overlap_metadata(profile, rows):
    profile['enabled'] = False
    profile['cameras']['primary']['references'] = rows
    report = validate_profile(profile)
    assert report['overlap_ready'] is False
    assert report['overlap_range_m'] is None
    assert report['common_measured_distances_m'] == []


@pytest.mark.parametrize('content', [
    '{"schema_version":1,"enabled":true,"enabled":false}',
    '{"schema_version":1,"enabled":false,"extra":NaN}',
    '{"schema_version":1,"enabled":false,"extra":Infinity}',
    '{"schema_version":1,"enabled":false,"extra":1e999}',
])
def test_ambiguous_or_nonfinite_json_never_becomes_a_silent_disabled_profile(tmp_path, content):
    path = tmp_path / 'ambiguous.json'
    path.write_text(content, encoding='utf-8')
    with pytest.raises(CalibrationError, match='JSON无效'):
        load_profile(path)


@pytest.mark.parametrize('path', [True, 7, .5, [], {}, '\x00'])
def test_bad_path_arguments_are_calibration_errors(path):
    with pytest.raises(CalibrationError):
        load_profile(path)


def test_unreadable_and_invalid_utf8_files_are_not_missing_profiles(tmp_path, monkeypatch):
    path = tmp_path / 'bad-encoding.json'
    path.write_bytes(b'\xff\xfe\x00')
    with pytest.raises(CalibrationError, match='无法读取'):
        load_profile(path)
    original = type(path).read_text
    def denied(p, *args, **kwargs):
        if p == path:
            raise PermissionError('fixture permission denied')
        return original(p, *args, **kwargs)
    monkeypatch.setattr(type(path), 'read_text', denied)
    with pytest.raises(CalibrationError, match='无法读取'):
        load_profile(path)


def test_dangling_profile_symlink_is_broken_config_not_legacy(tmp_path, monkeypatch):
    path = tmp_path / 'broken-link.json'
    original = type(path).is_symlink
    monkeypatch.setattr(type(path), 'is_symlink', lambda p: True if p == path else original(p))
    with pytest.raises(CalibrationError, match='符号链接'):
        load_profile(path)


def test_cached_readiness_flags_cannot_authorize_a_mutated_table(profile):
    cached = validate_profile(profile)
    assert cached['ready'] and cached['overlap_ready']
    cached['cameras']['primary']['references'][0]['distance_m'] = .1
    cached['errors'] = []  # Forging derived report fields never bypasses checks.
    with pytest.raises(CalibrationError):
        estimate_distance(cached, 'primary', .6)
    assert validate_profile(profile)['ready']  # Validation returned a deep copy.


@pytest.fixture
def near_profile(profile):
    profile['image_feature'] = IMAGE_FEATURE_NEAR
    profile.pop('paper_spec')
    for camera in profile['cameras'].values():
        for row in camera['references']:
            row['near_edge_y_normalized'] = row.pop('far_edge_y_normalized')
    return profile


@pytest.mark.parametrize('paper', [None, 'unknown', {},
                                 {'same_spec_confirmed': False},
                                 {'primary': [.105, .297], 'secondary': [.2, .3]}])
def test_same_physical_near_edge_calibration_needs_no_assumed_paper_length(near_profile, paper):
    near_profile['paper_spec'] = paper
    checked = validate_profile(near_profile)
    assert checked['ready'] and checked['reference_key'] == 'near_edge_y_normalized'
    result = estimate_distance(near_profile, 'primary', y=.71)
    assert result['distance_m'] == pytest.approx(.325)
    assert result['near_edge_y_normalized'] == .71
    assert result['image_feature'] == IMAGE_FEATURE_NEAR
    assert 'far_edge_y_normalized' not in result
    assert reference_y(near_profile, 'primary', .325)['near_edge_y_normalized'] == pytest.approx(.71)


def test_near_edge_still_requires_explicit_vehicle_front_distance_reference(near_profile):
    near_profile['distance_reference'] = 'camera_to_nearest_paper_edge_m'
    with pytest.raises(CalibrationError, match='distance_reference'):
        validate_profile(near_profile)


@pytest.mark.parametrize('include_both', [False, True])
def test_near_and_far_coordinates_cannot_be_mixed_in_one_table(near_profile, include_both):
    row = near_profile['cameras']['primary']['references'][0]
    row['far_edge_y_normalized'] = row['near_edge_y_normalized']
    if not include_both:
        row.pop('near_edge_y_normalized')
    with pytest.raises(CalibrationError):
        validate_profile(near_profile)
    near_profile['enabled'] = False
    report = validate_profile(near_profile)
    assert not report['overlap_ready'] and report['overlap_range_m'] is None


def test_far_mode_also_rejects_mixed_near_coordinates(profile):
    profile['cameras']['primary']['references'][0]['near_edge_y_normalized'] = .9
    with pytest.raises(CalibrationError, match='不能混表'):
        validate_profile(profile)


def test_clipped_near_edge_cannot_be_a_measured_reference(near_profile):
    near_profile['cameras']['primary']['references'][0]['bottom_clipped'] = True
    with pytest.raises(CalibrationError, match='裁断'):
        validate_profile(near_profile)


def test_query_coordinate_must_match_the_profile_feature(profile, near_profile):
    # near_profile uses the same fixture object here; independently regenerate
    # a far table to check the keyword contract in both directions.
    far = copy.deepcopy(near_profile)
    far['image_feature'] = IMAGE_FEATURE
    far['paper_spec'] = {'id': 'synthetic-row', 'width_m': .1, 'height_m': .2,
                         'same_spec_confirmed': True}
    for camera in far['cameras'].values():
        for row in camera['references']:
            row['far_edge_y_normalized'] = row.pop('near_edge_y_normalized')
    assert estimate_distance(far, 'primary', far_edge_y_normalized=.6)['distance_m'] == .4
    assert estimate_distance(near_profile, 'primary', near_edge_y_normalized=.6)['distance_m'] == .4
    with pytest.raises(CalibrationError):
        estimate_distance(near_profile, 'primary', far_edge_y_normalized=.6)
    with pytest.raises(CalibrationError):
        estimate_distance(far, 'primary', near_edge_y_normalized=.6)
    with pytest.raises(CalibrationError):
        estimate_distance(near_profile, 'primary', .6, near_edge_y_normalized=.6)


def test_forty_fifty_sixty_five_cm_tables_cannot_invent_an_unseen_twenty_five_cm_reference(near_profile):
    # Synthetic rows mirror only the currently visible distance range, not
    # real measured pixel values. The unseen 25 cm point is never fabricated.
    for index, camera in enumerate(near_profile['cameras'].values()):
        camera['references'] = [
            {'distance_m': d, 'near_edge_y_normalized': y + index * .02,
             'measured': True, 'evidence': 'synthetic-visible-range-only'}
            for d, y in ((.65, .48), (.5, .56), (.4, .64))
        ]
    near_profile['overlap']['reference_distance_m'] = .5
    checked = validate_profile(near_profile)
    assert checked['overlap_range_m'] == [.4, .65]
    with pytest.raises(CalibrationError, match='禁止外推'):
        reference_y(near_profile, 'primary', .25)
    with pytest.raises(CalibrationError, match='禁止外推'):
        estimate_distance(near_profile, 'primary', .85)
    near_profile['enabled'] = False
    assert validate_profile(near_profile)['ready'] is False
