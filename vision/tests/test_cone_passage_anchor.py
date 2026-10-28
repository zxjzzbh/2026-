"""Early same-row anchors allow pixel-only exits, never timed pass counts.

Inputs are synthetic observations through the real dual-view navigator. These
checks concern evidence bookkeeping, not a prediction of the vehicle's path.
"""
import copy

import numpy as np
import pytest

from test_cone_passage import Scene,recovery_event


def anchor(scene,side='left'):
    first,_,_=scene.observe(.4,.55,side=side)
    assert first['anchor_hits']==1 and not first['anchor_valid']
    result,_,_=scene.observe(.5,.63,side=side)
    assert result['anchor_valid'] and result['anchor_hits']==2
    assert result['gate_hits']==0 and not result['legacy_gate_candidate']
    assert not result['visual_estimate_ready']
    assert result['observation_deadline_s']==pytest.approx(5.5)
    return result


def pixel_exit(scene,start=.6,side='left',**fields):
    for index,(x,y) in enumerate(((.80,.86),(.84,.90),(.88,.94))):
        if side=='right':x=1-x
        result,target,nav=scene.observe(round(start+.1*index,3),y,x,side=side,**fields)
        assert 'pass_evidence' not in target
        if fields.get('single'):assert nav['line_state'] in ('single','missing')
        if index<2:assert not result['visual_estimate_ready']
    return result


def test_early_anchor_then_lost_paired_ground_can_finish_current_pixel_estimate():
    scene=Scene();anchor(scene)
    # The second white boundary disappears before the near gate. It progresses
    # from a single-edge prediction to missing, while current same-ID pixels
    # remain measured. No stale lane coordinates are used to finish the exit.
    result=pixel_exit(scene,single=True)
    assert result['visual_estimate_ready'] and result['exit_frames']==3
    assert result['anchor_valid'] and result['gate_hits']==0
    assert not result['legacy_gate_candidate']
    assert result['evidence_kind']=='image_sequence_estimate'
    assert not result['tail_verified'] and not result['physical_tail_verified']


def test_early_anchor_also_supports_right_side_exit_without_near_gate_hits():
    scene=Scene((.7,.7,.7),.3);anchor(scene,'right')
    result=pixel_exit(scene,side='right')
    assert result['visual_estimate_ready'] and result['gate_hits']==0
    assert not result['physical_tail_verified']


def test_one_ground_frame_then_current_pixels_cannot_create_an_anchor():
    scene=Scene();scene.observe(.4,.55)
    result=pixel_exit(scene,start=.5)
    assert not result['anchor_valid'] and not result['visual_estimate_ready']
    assert result['anchor_hits']==0 and result['observation_deadline_s'] is None


def test_anchor_needs_independent_frames_and_minimum_span():
    scene=Scene()
    first,target,nav=scene.observe(.4,.55)
    duplicate=scene.passage.observe(target,nav,'left',.43)
    assert duplicate['anchor_hits']==1 and not duplicate['fresh_evidence']
    assert not duplicate['anchor_valid']
    second,_,_=scene.observe(.43,.63)
    assert second['anchor_hits']==2 and not second['anchor_valid']
    third,_,_=scene.observe(.45,.64)
    assert third['anchor_hits']==3 and third['anchor_valid']
    assert third['observation_deadline_s']==pytest.approx(5.45)


def test_missing_same_slice_before_anchor_breaks_consecutive_evidence():
    scene=Scene();scene.observe(.4,.55);scene.observe(.5)
    first,_,_=scene.observe(.6,.60)
    assert first['anchor_hits']==1 and not first['anchor_valid']
    result,_,_=scene.observe(.7,.63)
    assert result['anchor_hits']==2 and result['anchor_valid']


def test_current_same_foot_conflict_vetoes_anchor_before_pixel_only_exit():
    scene=Scene();anchor(scene)
    # Keep a genuine current paired observation, but move its same-foot ego
    # reference beside the cone. A prior safe anchor must not hide this gap.
    scene.q=np.polyfit([170,222,274],[.65,.65,.65],2)
    result,target,nav=scene.observe(.6,.73)
    proof=target['pass_evidence']
    assert proof['reference_fraction_at_foot']==pytest.approx(.65)
    assert result['anchor_rejection']=='current_same_slice_clearance_insufficient'
    assert not result['anchor_valid'] and result['anchor_hits']==0
    result=pixel_exit(scene,start=.7)
    assert not result['visual_estimate_ready'] and not result['anchor_valid']


def test_rebuilding_vetoed_anchor_requires_new_same_slice_and_keeps_first_deadline():
    scene=Scene();anchor(scene)
    scene.q=np.polyfit([170,222,274],[.65,.65,.65],2)
    scene.observe(.6,.73)
    scene.q=np.polyfit([170,222,274],[.3,.3,.3],2)
    one,_,_=scene.observe(.7,.55)
    assert one['anchor_hits']==1 and not one['anchor_valid']
    two,_,_=scene.observe(.8,.63)
    assert two['anchor_valid'] and two['anchor_established_s']==pytest.approx(.8)
    assert two['observation_deadline_s']==pytest.approx(5.5)
    assert pixel_exit(scene,start=.9)['visual_estimate_ready']


def test_average_lane_counterexample_invalidates_early_anchor_at_late_foot_row():
    scene=Scene((.875,.875,.5),.4);anchor(scene,'right')
    for t,y in ((.6,.73),(.7,.75),(.8,.76)):
        result,target,nav=scene.observe(t,y,side='right')
    assert nav['ego_fraction']==pytest.approx(.75)
    proof=target['pass_evidence']
    assert proof['reference_fraction_at_foot']==pytest.approx(.5043158284)
    assert proof['reference_fraction_at_foot']-proof['cone_fraction_at_foot']<scene.passage.vehicle_half
    assert result['anchor_rejection']=='current_same_slice_clearance_insufficient'
    assert not result['anchor_valid'] and result['gate_hits']==0
    assert not pixel_exit(scene,start=.9,side='right')['visual_estimate_ready']


def test_five_second_window_never_counts_or_restarts_from_current_same_id_pixels():
    scene=Scene();deadline=anchor(scene)['observation_deadline_s']
    for t in np.arange(.6,5.5,.1):
        result,_,_=scene.observe(round(float(t),2),.8,.62)
        assert result['anchor_valid'] and not result['visual_estimate_ready']
        assert result['observation_deadline_s']==deadline
    expired,_,_=scene.observe(5.5,.86,.8)
    assert expired['phase']=='unverified' and not expired['anchor_valid']
    assert expired['anchor_rejection']=='observation_deadline_expired'
    assert not pixel_exit(scene,start=5.6)['visual_estimate_ready']
    assert not result['physical_tail_verified']


def test_unique_recovery_freezes_early_anchor_and_restarts_only_exit_observations():
    scene=Scene();deadline=anchor(scene)['observation_deadline_s']
    scene.observe(.6,.86,.8)
    frozen,_,_=scene.observe(.7,.90,.84,identity_revision=1,identity_recovered=True,
                            identity_recovery=recovery_event(.7,anchor=500))
    assert frozen['anchor_valid'] and frozen['recovery_candidate_frozen']
    assert frozen['anchor_hits']==2 and frozen['exit_frames']==0
    assert frozen['observation_deadline_s']==deadline
    result=pixel_exit(scene,start=.8,identity_revision=1)
    assert result['visual_estimate_ready'] and not result['physical_tail_verified']


def test_recovery_near_deadline_cannot_extend_five_second_episode():
    scene=Scene();deadline=anchor(scene)['observation_deadline_s']
    frozen,_,_=scene.observe(5.49,.86,.8,identity_revision=1,identity_recovered=True,
                            identity_recovery=recovery_event(5.49,anchor=500))
    assert frozen['anchor_valid'] and frozen['recovery_candidate_frozen']
    assert frozen['observation_deadline_s']==deadline
    result,_,_=scene.observe(5.5,.90,.84,identity_revision=1)
    assert result['phase']=='unverified' and not result['visual_estimate_ready']


def test_recovery_frame_cannot_hide_current_same_slice_clearance_conflict():
    scene=Scene();deadline=anchor(scene)['observation_deadline_s']
    scene.q=np.polyfit([170,222,274],[.65,.65,.65],2)
    result,target,_=scene.observe(.6,.73,identity_revision=1,identity_recovered=True,
                                identity_recovery=recovery_event(.6,anchor=500))
    assert 'pass_evidence' in target
    assert result['anchor_rejection']=='current_same_slice_clearance_insufficient'
    assert not result['anchor_valid'] and result['gate_hits']==0
    assert result['observation_deadline_s']==deadline
    assert not pixel_exit(scene,start=.7,identity_revision=1)['visual_estimate_ready']


def test_unproved_revision_cannot_buy_same_target_a_new_observation_deadline():
    scene=Scene();deadline=anchor(scene)['observation_deadline_s']
    lost,_,_=scene.observe(.6,.73,identity_revision=1)
    assert not lost['anchor_valid'] and lost['observation_deadline_s']==deadline
    for t,y in ((.7,.55),(.8,.63)):
        result,_,_=scene.observe(t,y,identity_revision=1)
    assert result['anchor_valid'] and result['observation_deadline_s']==deadline


@pytest.mark.parametrize('change',[{'id':'secondary-other'},{'camera':'primary'},{'side':'right'}])
def test_anchor_cannot_transfer_across_target_camera_or_side(change):
    scene=Scene();anchor(scene);target,nav=scene.row(.6,.73)
    target=copy.deepcopy(target);nav=copy.deepcopy(nav)
    target['id']=change.get('id',target['id'])
    nav['source_camera']=change.get('camera',nav['source_camera'])
    result=scene.passage.observe(target,nav,change.get('side','left'),.6)
    assert not result['anchor_valid'] and not result['visual_estimate_ready']


def test_current_contradiction_is_checked_even_after_a_ready_frame():
    scene=Scene();anchor(scene);assert pixel_exit(scene)['visual_estimate_ready']
    scene.q=np.polyfit([170,222,274],[.65,.65,.65],2)
    result,_,_=scene.observe(.9,.75)
    assert result['anchor_rejection']=='current_same_slice_clearance_insufficient'
    assert not result['anchor_valid'] and not result['visual_estimate_ready']
