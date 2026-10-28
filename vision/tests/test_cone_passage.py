"""Actual navigation pipeline produces same-foot evidence; no physical claims."""
import copy
from pathlib import Path

import numpy as np
import pytest

from carvision.cone_low_speed import DualLowSpeedNavigator,trial_settings
from carvision.cone_passage import PassageEvidence
from carvision.cone_planning import load_settings


ROOT=Path(__file__).parents[1]


class Scene:
    def __init__(self,reference=(.3,.3,.3),fraction=.7):
        self.s=trial_settings(load_settings(ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json'))
        self.nav=DualLowSpeedNavigator(self.s);self.passage=PassageEvidence(self.s)
        self.H=self.nav.views['secondary'].projection.h
        self.q=np.polyfit([170,222,274],reference,2);self.fraction=fraction

    def row(self,t,y=None,x_image=None,*,single=False,**track_fields):
        def reference(z):return float(-(self.H[0,1]*z+self.H[0,2])/self.H[0,0])
        def left(z):return reference(z)-float(np.polyval(self.q,z))*180
        tracks=[]
        if y is not None:
            py=y*360;x=left(py)+self.fraction*180 if x_image is None else x_image*480
            tracks=[{'id':'secondary-1','visible':True,'confirmed':True,'seen_s':t,
                'bbox_xyxy':[x-5.4,py-18,x+5.4,min(360,py+3)],'foot_pixel':[x,py],
                'kind':'cone_candidate','ground_contact_usable':True,'clipped':False,
                'identity_revision':0,'identity_recovered':False,**track_fields}]
        ys=np.linspace(170,274,16)
        row={'captured_s':t,'frame_id':round(t*1000),'tracks':tracks,
            'road_context':{'valid':True,'ground_supported':True,'turn_aware':True,
                'boundary_candidates':[],
                'left_inner':[[left(z),float(z)] for z in ys],
                'right_inner':[[left(z)+180,float(z)] for z in ys]}}
        if single:
            road=row['road_context'];road['valid']=False
            road['boundary_candidates']=[{'right_edge':road['left_inner']}]
            road['right_inner']=[]
        nav=self.nav.update_views({'secondary':np.zeros((360,480,3),np.uint8)},
                                  {'secondary':row},t)
        target=next((c for c in nav.get('cones',[]) if c['id']=='secondary-1'),{'id':'secondary-1'})
        return target,nav

    def observe(self,t,y=None,x_image=None,side='left',**fields):
        target,nav=self.row(t,y,x_image,**fields)
        return self.passage.observe(target,nav,side,t),target,nav

    def candidate(self,side='left'):
        result=None
        for t,y in ((.4,.55),(.5,.63),(.6,.73),(.7,.75),(.8,.76)):
            result,_,_=self.observe(t,y,side=side)
        assert result['visual_pass_candidate'] and result['gate_hits']==3
        assert not result['visual_estimate_ready']
        return result


def test_nav_evidence_is_same_frame_same_foot_slice_and_average_counterexample_is_rejected():
    scene=Scene((.875,.875,.5),.4);observations=[]
    for t,y in ((.4,.55),(.5,.63),(.6,.73),(.7,.75),(.8,.76)):
        result,target,nav=scene.observe(t,y,side='right');proof=target['pass_evidence']
        model=scene.nav.views['secondary'].paired_corridor[1]
        left,right=model.bounds(y*360);H=scene.H
        reference=-(H[0,1]*y*360+H[0,2])/H[0,0]
        assert proof['reference_fraction_at_foot']==pytest.approx((reference-left)/(right-left))
        assert proof['frame_id']==nav['frame_id'] and proof['captured_s']==t
        assert proof['camera']==nav['source_camera']=='secondary'
        observations.append((result['gate_hits'],proof['reference_fraction_at_foot']))
    assert nav['ego_fraction']==pytest.approx(.75) and nav['ego_near_fraction']==pytest.approx(.5)
    assert observations[-2][1]==pytest.approx(.5421597633)
    assert observations[-1][1]==pytest.approx(.5043158284)
    assert observations[-2][0]==observations[-1][0]==0
    assert not result['visual_pass_candidate'] and not result['visual_estimate_ready']


def test_three_ground_gates_are_only_a_visual_candidate_not_tail_verification():
    scene=Scene();result=scene.candidate()
    assert result['near_seen'] and result['observation_deadline_s']==pytest.approx(5.5)
    assert result['anchor_valid'] and result['legacy_gate_candidate']
    assert not result['tail_verified'] and not result['physical_tail_verified']
    assert result['evidence_kind']=='image_sequence_estimate'


def test_timer_alone_cannot_confirm_a_pass_and_does_not_extend_its_deadline():
    scene=Scene();deadline=scene.candidate()['observation_deadline_s']
    for t in np.arange(.9,5.5,.1):
        t=round(float(t),2)
        result,_,_=scene.observe(t)
        assert not result['visual_estimate_ready'] and result['observation_deadline_s']==deadline
    result,_,_=scene.observe(5.5)
    assert result['phase']=='unverified' and not result['visual_estimate_ready']
    assert not result['tail_verified']


def test_candidate_requires_three_additional_current_side_bottom_frames_with_progress():
    scene=Scene();scene.candidate()
    for t,x,y in ((.9,.80,.86),(1.,.84,.90)):
        result,target,_=scene.observe(t,y,x)
        assert 'pass_evidence' not in target and not result['visual_estimate_ready']
    result,_,_=scene.observe(1.1,.94,.88)
    assert result['visual_estimate_ready'] and result['exit_frames']==3
    assert result['phase']=='visual_estimate' and result['target_id']=='secondary-1'
    assert result['frame_id']==1100 and result['captured_s']==1.1
    assert not result['tail_verified']


def test_stationary_outer_pixels_do_not_prove_side_departure():
    scene=Scene();scene.candidate()
    for t in (.9,1.,1.1):result,_,_=scene.observe(t,.9,.83)
    assert result['exit_frames']==3 and not result['visual_estimate_ready']


def test_right_side_uses_the_opposite_current_pixel_exit_zone():
    scene=Scene((.7,.7,.7),.3);scene.candidate(side='right')
    for t,x,y in ((.9,.20,.86),(1.,.16,.90),(1.1,.12,.94)):
        result,_,_=scene.observe(t,y,x,side='right')
    assert result['visual_estimate_ready'] and not result['tail_verified']


def test_current_single_edge_prediction_cannot_generate_new_ground_pass_evidence():
    scene=Scene();scene.row(.3)
    target,nav=scene.row(.4,.73,single=True)
    assert nav['line_state']=='single' and not nav['boundary_observed']
    assert 'pass_evidence' not in target
    result=scene.passage.observe(target,nav,'left',.4)
    assert result['gate_hits']==0 and not result['visual_pass_candidate']


def test_pixel_departure_without_previous_ground_candidate_cannot_count():
    scene=Scene()
    for t,x,y in ((.9,.80,.86),(1.,.84,.90),(1.1,.88,.94)):
        result,target,_=scene.observe(t,y,x)
        assert 'pass_evidence' not in target
    assert result['gate_hits']==0 and not result['visual_estimate_ready']


@pytest.mark.parametrize('change',[{'camera':'primary'},{'frame_id':1},{'captured_s':.1},
    {'within_observed_band':False},{'target_id':'secondary-other'},
    {'reference_fraction_at_foot':float('nan')}])
def test_stale_cross_target_or_outside_proof_cannot_add_a_gate(change):
    scene=Scene()
    for t,y in ((.4,.55),(.5,.63),(.6,.73),(.7,.75),(.8,.76)):
        target,nav=scene.row(t,y);target['pass_evidence'].update(change)
        result=scene.passage.observe(target,nav,'left',t)
    assert result['gate_hits']==0 and not result['visual_pass_candidate']


def test_duplicate_current_frame_does_not_add_exit_or_renew_deadline():
    scene=Scene();deadline=scene.candidate()['observation_deadline_s']
    result,target,nav=scene.observe(.9,.86,.8);before=copy.deepcopy((target,nav))
    repeated=scene.passage.observe(target,nav,'left',.95)
    assert not repeated['fresh_evidence'] and repeated['exit_frames']==result['exit_frames']==1
    assert repeated['observation_deadline_s']==deadline and (target,nav)==before


def recovery_event(t,anchor=800,camera='secondary'):
    return {'source_camera':camera,'anchor_frame_id':anchor,
        'evidence_frame_ids':[round(t*1000)-50,round(t*1000)],
        'recovered_s':t,'ground_evidence':False}


def test_unique_same_id_recovery_freezes_candidate_but_requires_fresh_exit_sequence():
    scene=Scene();deadline=scene.candidate()['observation_deadline_s']
    result,_,_=scene.observe(.9,.86,.8,identity_revision=1,identity_recovered=True,
                             identity_recovery=recovery_event(.9))
    assert result['recovery_candidate_frozen'] and result['visual_pass_candidate']
    assert result['gate_hits']==3 and result['exit_frames']==0
    assert result['observation_deadline_s']==deadline and not result['visual_estimate_ready']
    for t,x,y in ((1.,.81,.87),(1.1,.85,.91),(1.2,.89,.95)):
        result,_,_=scene.observe(t,y,x,identity_revision=1)
    assert result['visual_estimate_ready'] and not result['tail_verified']


@pytest.mark.parametrize('event',[None,[],recovery_event(.9,camera='primary'),
    {**recovery_event(.9),'evidence_frame_ids':[900,900]}])
def test_unproved_identity_change_clears_candidate(event):
    scene=Scene();scene.candidate()
    result,_,_=scene.observe(.9,.86,.8,identity_revision=1,identity_recovered=True,identity_recovery=event)
    assert not result['visual_pass_candidate'] and result['gate_hits']==0
    assert not result['visual_estimate_ready']


def test_side_or_camera_conflict_cannot_carry_old_candidate():
    scene=Scene();scene.candidate()
    result,_,_=scene.observe(.9,.86,.8,side='right')
    assert not result['visual_pass_candidate'] and not result['visual_estimate_ready']


def test_recovery_cannot_revive_expired_observation_candidate():
    scene=Scene();scene.candidate()
    result,_,_=scene.observe(5.51,.94,.88,identity_revision=1,identity_recovered=True,
                             identity_recovery=recovery_event(5.51))
    assert result['phase']=='unverified' and not result['visual_pass_candidate']
    assert not result['visual_estimate_ready'] and not result['tail_verified']


def test_gap_in_current_exit_pixels_requires_a_new_exit_sequence():
    scene=Scene();scene.candidate()
    scene.observe(.9,.86,.8);scene.observe(1.,.9,.84);scene.observe(1.1)
    for t,x,y in ((1.2,.81,.87),(1.3,.85,.91)):
        result,_,_=scene.observe(t,y,x)
        assert not result['visual_estimate_ready']
    result,_,_=scene.observe(1.4,.95,.89)
    assert result['visual_estimate_ready']


@pytest.mark.parametrize('change',[{'y_fraction':1.1},{'y_fraction':float('nan')},
    {'bbox_xyxy':[10,10,5,20]},{'bbox_xyxy':[0,0,900,360]},
    {'bbox_xyxy':None}])
def test_malformed_current_pixel_geometry_cannot_complete_candidate(change):
    scene=Scene();scene.candidate()
    for t,x,y in ((.9,.80,.86),(1.,.84,.90),(1.1,.88,.94)):
        target,nav=scene.row(t,y,x);nav['visual_cones'][0].update(change)
        result=scene.passage.observe(target,nav,'left',t)
    assert not result['visual_estimate_ready'] and result['exit_frames']==0


def test_expired_episode_cannot_restart_from_same_id_frames_or_replayed_proof():
    scene=Scene();scene.candidate()
    old_target,old_nav=scene.row(.81,.76)
    target,nav=scene.row(5.5)
    expired=scene.passage.observe(target,nav,'left',5.5)
    assert expired['phase']=='unverified'
    for t in (5.6,5.7):
        target,nav=scene.row(t);target=copy.deepcopy(old_target)
        result=scene.passage.observe(target,nav,'left',t)
        assert result['gate_hits']==0 and not result['visual_pass_candidate']
    for t,y in ((5.8,.55),(5.9,.63),(6.,.73),(6.1,.75),(6.2,.76)):
        result,_,_=scene.observe(t,y)
        assert result['phase']=='unverified' and not result['anchor_valid']
        assert result['observation_deadline_s']==pytest.approx(5.5)
    # Only an explicit new task/target reset starts a new observation episode;
    # even then it requires fresh same-slice frames rather than replayed proof.
    scene.passage.reset('new_task')
    for t,y in ((6.3,.55),(6.4,.63),(6.5,.73),(6.6,.75),(6.7,.76)):
        result,_,_=scene.observe(t,y)
    assert result['visual_pass_candidate'] and result['observation_deadline_s']==pytest.approx(11.4)
    assert not result['visual_estimate_ready']
