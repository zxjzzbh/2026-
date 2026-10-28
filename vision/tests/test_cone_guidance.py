"""Synthetic current pixels and clocks; no measured motion or hardware output."""
import copy

import pytest

from carvision.cone_guidance import (CurrentTargetEvidence, GuidanceBudgets,
                                    TARGET_ONLY_MAX_S, strict_visual)


def observation(t, *, identifier='secondary-A', camera='secondary', fid=None,
                revision=0, state='missing'):
    fid = round(t*1000) if fid is None else fid
    entry = {'id': identifier, 'source_camera': camera, 'frame_id': fid,
             'captured_s': t, 'confirmed': True, 'current_pixel': True,
             'identity_revision': revision, 'image_fraction': .6,
             'half_image_width': .05, 'image_reference_fraction': .5,
             'y_fraction': .7, 'bbox_xyxy': [264., 216., 312., 270.]}
    nav = {'source_camera': camera, 'frame_id': fid, 'captured_s': t,
           'line_state': state, 'line_missing': state == 'missing',
           'line_held': state == 'single', 'boundary_observed': state == 'paired',
           'visual_cones': [entry], 'ready': True}
    if state in ('paired', 'single'):
        nav.update(lane_bounds_px=[[100, 380], [50, 430]], ego_fraction=.5)
    if state == 'single':
        nav.update(prediction_age_s=.1)
    return nav


def test_strict_visual_accepts_actual_same_frame_normalized_foot_and_bbox():
    nav = observation(.2)
    entry = strict_visual(nav, 'secondary-A', .3)
    assert entry is nav['visual_cones'][0]
    assert strict_visual(nav, 'secondary-A', .450001) is None
    assert strict_visual(nav, 'secondary-A', .19) is None


@pytest.mark.parametrize('field,value', [
    ('source_camera', 'primary'), ('frame_id', 201), ('captured_s', .19),
    ('id', 'secondary-B'), ('confirmed', False), ('confirmed', 1),
    ('current_pixel', False), ('identity_revision', None),
    ('identity_revision', True), ('identity_revision', -1),
    ('image_fraction', float('nan')), ('image_fraction', .9),
    ('y_fraction', float('inf')), ('y_fraction', 1.), ('y_fraction', -.01),
    ('y_fraction', .1), ('half_image_width', .08),
    ('bbox_xyxy', [264., 216., float('nan'), 270.]),
    ('bbox_xyxy', [312., 216., 264., 270.]),
    ('bbox_xyxy', [264., 216., 481., 270.]),
])
def test_strict_visual_rejects_forged_or_inconsistent_current_pixels(field, value):
    nav = observation(.2)
    nav['visual_cones'][0][field] = value
    assert strict_visual(nav, 'secondary-A', .2) is None


@pytest.mark.parametrize('changes', [
    {'source_camera': 'primary'}, {'frame_id': True}, {'frame_id': -1},
    {'captured_s': float('nan')}, {'visual_cones': [None]}, {'visual_cones': None},
])
def test_strict_visual_requires_a_real_source_frame(changes):
    nav = observation(.2)
    nav.update(changes)
    assert strict_visual(nav, 'secondary-A', .2) is None


def test_id_camera_prefix_and_image_size_are_not_inferred_from_pixels():
    nav = observation(.2, identifier='primary-A')
    assert strict_visual(nav, 'primary-A', .2) is None
    nav = observation(.2)
    assert strict_visual(nav, 'secondary-A', .2, image_size=(960, 720)) is None
    assert strict_visual(nav, 'secondary-A', .2, image_size=(480, True)) is None
    assert strict_visual(observation(.2, identifier='secondary-'), 'secondary-', .2) is None


@pytest.mark.parametrize('field,value', [('frame_id', True), ('captured_s', True)])
def test_boolean_entry_clock_fields_cannot_impersonate_numeric_source_provenance(field, value):
    nav = observation(1., fid=1)
    nav['visual_cones'][0][field] = value
    assert strict_visual(nav, 'secondary-A', 1.) is None


def test_three_independent_current_frames_can_accumulate_before_road_is_lost():
    evidence = CurrentTargetEvidence()
    for t in (0., .075):
        result = evidence.update(observation(t, state='paired'), 'secondary-A', 'left', t)
        assert not result['target_stable']
    result = evidence.update(observation(.15, state='paired'), 'secondary-A', 'left', .15)
    assert result['current_valid'] and result['fresh_evidence'] and result['target_stable']
    assert result['frames'] == 3 and result['span_s'] == pytest.approx(.15)
    assert not result['physical_path_verified']
    assert evidence.update(observation(.25), 'secondary-A', 'left', .25)['target_stable']


def test_repeated_capture_or_frame_cannot_build_stability_or_renew_current_evidence():
    evidence = CurrentTargetEvidence()
    first = observation(0., fid=1)
    assert evidence.update(first, 'secondary-A', 'left', 0.)['frames'] == 1
    for now in (.05, .10, .15):
        result = evidence.update(first, 'secondary-A', 'left', now)
        assert result['frames'] == 1 and not result['target_stable']
        assert not result['fresh_evidence']
    result = evidence.update(observation(.2, fid=1), 'secondary-A', 'left', .2)
    assert result['frames'] == 1 and not result['target_stable']
    second = observation(.2, fid=2)
    assert evidence.update(second, 'secondary-A', 'left', .2)['frames'] == 2
    second['frame_id'] = 3
    second['visual_cones'][0]['frame_id'] = 3
    result = evidence.update(second, 'secondary-A', 'left', .21)
    assert result['frames'] == 2 and not result['fresh_evidence']


@pytest.mark.parametrize('change', ['camera', 'id', 'side', 'revision', 'gap'])
def test_identity_side_view_revision_or_gap_rebuilds_stable_target(change):
    evidence = CurrentTargetEvidence()
    for t in (0., .1, .2):
        result = evidence.update(observation(t), 'secondary-A', 'left', t)
    assert result['target_stable']
    t, identifier, camera, side, revision = .3, 'secondary-A', 'secondary', 'left', 0
    if change == 'camera':
        identifier, camera = 'primary-A', 'primary'
    elif change == 'id':
        identifier = 'secondary-B'
    elif change == 'side':
        side = 'right'
    elif change == 'revision':
        revision = 1
    else:
        t = .451
    result = evidence.update(observation(t, identifier=identifier, camera=camera, revision=revision), identifier, side, t)
    assert result['frames'] == 1 and not result['target_stable']


def test_unbound_side_or_invalid_visual_breaks_target_evidence():
    evidence = CurrentTargetEvidence()
    for t in (0., .1, .2):
        result = evidence.update(observation(t), 'secondary-A', None, t)
        assert result['current_valid'] and not result['target_stable']
        assert result['frames'] == 0
    for t in (.3, .4):
        evidence.update(observation(t), 'secondary-A', 'left', t)
    bad = observation(.5)
    bad['visual_cones'][0].pop('identity_revision')
    assert evidence.update(bad, 'secondary-A', 'left', .5)['frames'] == 0
    assert evidence.update(observation(.6), 'secondary-A', 'left', .6)['frames'] == 1


def test_no_guidance_budget_includes_capture_age_and_latches_before_late_good_frame():
    budget = GuidanceBudgets()
    first = budget.update(observation(0.), .2)
    assert first['none_since_s'] == 0. and first['none_deadline_s'] == 1.
    result = budget.update(observation(1., state='paired'), 1., paired_stable=True)
    assert result['expired'] and result['reason'] == 'no_reliable_guidance_deadline'
    assert budget.update(observation(1.1, state='paired'), 1.1, paired_stable=True)['expired']


@pytest.mark.parametrize('kind', ['paired', 'single', 'target'])
def test_only_fresh_reliable_guidance_can_end_an_unexpired_no_guidance_run(kind):
    budget = GuidanceBudgets()
    budget.update(observation(0.), 0.)
    kwargs = {'paired_stable': kind == 'paired', 'single_valid': kind == 'single',
              'target_stable': kind == 'target'}
    nav = observation(.8, state=kind if kind in ('paired', 'single') else 'missing')
    result = budget.update(nav, .8, **kwargs)
    assert not result['expired'] and result['none_since_s'] is None
    assert result['mode'] == ('target_only' if kind == 'target' else kind)
    result = budget.update(observation(.9), .9)
    assert result['none_since_s'] == pytest.approx(.9)
    assert not budget.update(observation(1.1), 1.1)['expired']


def test_duplicate_paired_or_target_claim_cannot_wash_earlier_no_guidance_deadline():
    budget = GuidanceBudgets()
    original = observation(0., state='paired')
    budget.update(original, 0., paired_stable=False)
    assert budget.update(original, .1, paired_stable=True)['none_since_s'] == 0.
    assert budget.update(original, .2, target_stable=True)['none_since_s'] == 0.
    result = budget.update(observation(1., state='paired'), 1., paired_stable=True)
    assert result['expired']


def test_five_second_target_only_episode_begins_before_stability_and_never_tracks_id():
    budget = GuidanceBudgets()
    first = budget.update(observation(0.), .2, target_current_valid=True)
    assert first['none_since_s'] == 0. and first['target_only_since_s'] == 0.
    assert first['target_only_limit_s'] == TARGET_ONLY_MAX_S == 5.
    assert not first['target_only_limit_verified']
    for i in range(3, 50):
        t = round(i*.1, 8)
        # Caller has independently re-established the new ID; switching IDs
        # still supplies no evidence that the road has been recovered.
        nav = observation(t, identifier='secondary-A' if i % 2 else 'secondary-B')
        result = budget.update(nav, t, target_stable=True)
        assert not result['expired'] and result['target_only_deadline_s'] == 5.
    result = budget.update(observation(5., state='paired'), 5., paired_stable=True)
    assert result['expired'] and result['reason'] == 'target_only_episode_deadline'


def test_one_paired_frame_or_valid_single_cannot_restart_the_target_only_episode():
    budget = GuidanceBudgets()
    budget.update(observation(0.), 0., target_stable=True)
    single = budget.update(observation(.2, state='single'), .2, single_valid=True)
    assert single['none_since_s'] is None and single['target_only_deadline_s'] == 5.
    unconfirmed_pair = budget.update(observation(.3, state='paired'), .3,
                                    target_stable=True, paired_stable=False)
    assert unconfirmed_pair['target_only_deadline_s'] == 5.
    paired = budget.update(observation(.4, state='paired'), .4, paired_stable=True)
    assert paired['target_only_since_s'] is None
    new_episode = budget.update(observation(.5), .5, target_stable=True)
    assert new_episode['target_only_deadline_s'] == pytest.approx(5.5)


@pytest.mark.parametrize('bad', ['stale', 'future', 'duplicate', 'backward', 'wrong_state'])
def test_untrusted_road_claim_cannot_close_target_only_episode(bad):
    budget = GuidanceBudgets()
    budget.update(observation(0.), 0., target_stable=True)
    nav, now = observation(.1, state='paired'), .1
    if bad == 'stale':
        now = .4
    elif bad == 'future':
        nav = observation(.2, state='paired')
    elif bad == 'duplicate':
        nav = observation(0., state='paired')
    elif bad == 'backward':
        nav = observation(.1, state='paired', fid=-1)
    else:
        nav = observation(.1)
    result = budget.update(nav, now, paired_stable=True)
    assert result['target_only_deadline_s'] == 5.


def test_single_claim_requires_current_valid_prediction_and_does_not_hide_no_guidance():
    budget = GuidanceBudgets()
    nav = observation(0., state='single')
    nav['prediction_age_s'] = .251
    result = budget.update(nav, 0., single_valid=True)
    assert result['mode'] == 'none' and result['none_deadline_s'] == 1.
    nav = observation(.2, state='single')
    nav['line_missing'] = True
    assert budget.update(nav, .2, single_valid=True)['none_deadline_s'] == 1.


def test_no_guidance_can_expire_inside_an_active_target_only_episode():
    budget = GuidanceBudgets()
    budget.update(observation(0.), 0., target_stable=True)
    budget.update(observation(.1), .1)
    result = budget.update(observation(1.1), 1.1, target_stable=True)
    assert result['expired'] and result['reason'] == 'no_reliable_guidance_deadline'
    assert result['target_only_deadline_s'] == 5.


def test_budget_clock_cannot_go_backwards_or_accept_nonfinite_now():
    budget = GuidanceBudgets()
    budget.update(observation(.2), .2)
    for now in (.1, float('nan'), float('inf'), True):
        with pytest.raises(ValueError, match='clock'):
            budget.update(observation(.2), now)


def test_validation_and_budget_updates_do_not_mutate_navigation():
    nav = observation(.2)
    before = copy.deepcopy(nav)
    CurrentTargetEvidence().update(nav, 'secondary-A', 'left', .2)
    GuidanceBudgets().update(nav, .2, target_current_valid=True)
    assert nav == before
