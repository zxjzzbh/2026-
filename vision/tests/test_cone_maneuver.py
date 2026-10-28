"""Bounded obstacle requests using synthetic inputs and fake hardware only."""
import copy

import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_maneuver import ObservedLaneRecovery,TargetGap
from carvision.cone_navigation import actuator_settings
from carvision.shared_executor import BorrowedActuator
from test_cone_turn_controller import lane,settings,started
from test_shared_executor import CLIENT,Worker,rig


def cone(identifier='secondary-A',fraction=.535,*,y=.67):
    # Keep the actual observed foot inside a current image box at every y.
    x=.42*480;foot_y=y*360
    return {'id':identifier,'fraction':fraction,'half_width':.03,'y_fraction':y,
            'near':False,'bbox_xyxy':[x-9,foot_y-18,x+9,foot_y+4],'image_fraction':.42,
            'half_image_width':18/960,'confirmed':True,'identity_revision':0}


def observed(t,*,target=None,state='paired',near=.64,far=None,camera='secondary',**changes):
    target=cone() if target is None else target
    nav=lane(t,state=state,near=near,far=near if far is None else far,
             camera=camera,cones=[target],**changes)
    nav['obstacle_diagnostics']=[{'id':target['id'],'reason':'current_observed_lane_position'}]
    nav['visual_cones']=[{'id':target['id'],'confirmed':True,'current_pixel':True,
        'captured_s':nav['captured_s'],'frame_id':nav['frame_id'],'source_camera':camera,
        'image_fraction':target.get('image_fraction',.42),'half_image_width':target.get('half_image_width',18/960),
        'bbox_xyxy':copy.deepcopy(target['bbox_xyxy']),'y_fraction':target['y_fraction'],
        'image_reference_fraction':.50,'identity_revision':target['identity_revision']}]
    return nav


def avoiding():
    controller=started(gain=1.5)
    for t in (.3,.4,.5):
        result=controller.step(observed(t),t)
    assert result['target_id']=='secondary-A' and result['steering_us']>1690
    return controller


def test_current_confirmed_small_lane_target_gets_meaningful_request_without_bypassing_slew():
    controller=started(gain=1.5);previous=controller.actual_steering
    result=controller.step(observed(.3),.3)
    assert result['target_space']=='lane' and result['maneuver_phase']=='avoid'
    assert 1727<result['requested_steering_us']<=1870
    assert abs(result['steering_us']-previous)<=60
    assert result['effective_steering_gain']==1.5 and not result['small_target_limited']
    assert controller.count==0 and not controller.passed_ids


@pytest.mark.parametrize('missing_evidence',['visual','lane_provenance','visual_frame_match'])
def test_small_target_release_requires_current_pixels_and_observed_lane_provenance(missing_evidence):
    controller=started(gain=1.5);nav=observed(.3)
    if missing_evidence=='visual':nav['visual_cones']=[]
    elif missing_evidence=='lane_provenance':nav['obstacle_diagnostics']=[]
    else:nav['visual_cones'][0]['frame_id']=1
    result=controller.step(nav,.3)
    assert result['small_target_limited'] and result['requested_steering_us']==1727
    assert controller.count==0


def test_same_pd_error_uses_gain_one_for_lane_follow_and_saved_gain_only_for_avoidance():
    following=started(gain=1.5);avoiding_controller=started(gain=1.5)
    following.pd.reset();avoiding_controller.pd.reset()
    normal=following.step(lane(.3,near=.6,far=.6),.3)
    # This cone's only lane target is .50, matching ordinary lane following.
    avoidance=avoiding_controller.step(observed(.3,target=cone(fraction=.76),near=.6),.3)
    assert following.pd.error==pytest.approx(avoiding_controller.pd.error)
    assert following.pd.error==pytest.approx(-.1)
    assert normal['effective_steering_gain']==1.
    assert avoidance['effective_steering_gain']==1.5
    assert avoidance['requested_steering_us']>normal['requested_steering_us']>1610


def test_boundary_correction_ignores_saved_avoidance_gain_even_with_a_locked_cone():
    results=[]
    for gain in (1.,1.5,3.):
        controller=started(gain=gain);controller.pd.reset()
        result=controller.step(observed(.3,near=.1,boundary_correction='right'),.3)
        assert result['boundary_correction']=='right'
        assert result['effective_steering_gain']==1.
        results.append(result['requested_steering_us'])
    assert len(set(results))==1


def test_image_continuation_uses_avoidance_gain_but_has_no_new_pass_evidence():
    controller=avoiding()
    previous=controller.actual_steering
    nav=observed(.6,target={**cone(),'near_field_continuation':True},state='missing')
    result=controller.step(nav,.6)
    assert result['target_space']=='image' and result['effective_steering_gain']==1.5
    # Stable current target pixels are feedback even without road boundaries;
    # they retain the ordinary slew limit rather than the no-feedback cap.
    assert result['guidance_mode']=='target_only' and result['target_feedback_steering']
    assert result['target_observation']['target_stable']
    assert abs(result['steering_us']-previous)<=60
    assert result['passage_evidence']['gate_hits']==0
    assert result['passage_evidence']['exit_frames']==0
    assert controller.count==0 and not controller.passed_ids


def test_two_current_continuous_pairs_resume_known_demand_without_resetting_recovery_timer():
    controller=avoiding()
    missing=controller.step(lane(.6,state='missing'),.6)
    since=controller.line_missing_since
    assert abs(missing['steering_us']-1610)<=80
    first=controller.step(observed(.7),.7)
    assert not first['paired_stable'] and abs(first['steering_us']-1610)<=80
    second=controller.step(observed(.75),.75)
    assert not second['paired_stable'] and second['steering_us']>1690
    assert controller.line_missing_since==since
    # Another genuine loss still withdraws the stronger demand immediately.
    lost_again=controller.step(lane(.8,state='missing'),.8)
    assert abs(lost_again['steering_us']-1610)<=80


@pytest.mark.parametrize('change',[{'near':.85},{'camera':'primary'},
                                   {'target':cone('secondary-B')},{'boundary_correction':'right'}])
def test_geometric_source_or_target_change_cannot_borrow_old_full_demand(change):
    controller=avoiding()
    controller.step(lane(.6,state='missing'),.6)
    controller.step(observed(.7),.7)
    result=controller.step(observed(.75,**change),.75)
    assert not result['paired_stable']
    assert abs(result['steering_us']-1610)<=80


def test_pending_has_one_capture_anchored_deadline_and_decays_without_reversing_or_extra_gain():
    controller=avoiding()
    gap=lane(.6,near=.5,far=.5)
    result=controller.step(gap,.65)
    assert result['target_space']=='pending' and result['maneuver_phase']=='target_pending'
    assert result['effective_steering_gain']==1.
    assert result['target_pending_deadline_s']==pytest.approx(.85)
    assert 1610<=result['requested_steering_us']<=1690
    before=result['requested_steering_us']
    cached=controller.step(gap,.7)
    assert cached['fresh_control'] is False
    result=controller.step(lane(.8),.8)
    assert result['target_pending_deadline_s']==pytest.approx(.85)
    assert 1610<=result['requested_steering_us']<before
    expired=controller.step(lane(.85),.85)
    assert expired['maneuver_phase']=='unverified_recovery'
    assert expired['phase']!='complete' and not expired['lane_return_confirmed']
    assert controller.count==0 and not controller.passed_ids


def test_pending_reacquisition_uses_current_pixels_and_does_not_invent_passes():
    controller=avoiding();controller.step(lane(.6),.6)
    result=controller.step(observed(.7),.7)
    assert result['target_id']=='secondary-A' and result['maneuver_phase']=='avoid'
    assert result['target_pending_deadline_s'] is None
    assert result['effective_steering_gain']==1.5
    assert controller.count==0 and not controller.passed_ids


def test_unverified_new_lock_cannot_complete_the_race_using_two_earlier_pass_records():
    from cone_mission_fixture import estimate_one,ground_nav
    controller=started(gain=1.5,handoff=True)
    estimate_one(controller,.3,'secondary-first');estimate_one(controller,1.1,'secondary-second')
    assert controller.count==2
    for t,y in ((1.9,.55),(2.,.60)):controller.step(ground_nav(t,'secondary-current',y=y),t)
    assert controller.mission.registered('secondary','secondary-current')
    for i in range(21,39):
        result=controller.step(lane(i/10),i/10)
        assert result['phase']!='complete' and not result['lane_return_confirmed']
    assert result['maneuver_phase']=='unverified_recovery' and controller.count==2
    assert result['mission_unresolved'][0]['id']=='secondary-current'
    assert not controller.mission.completion_ready()


def test_empty_frames_cannot_count_but_three_current_exit_observations_can_estimate_one_pass():
    from cone_mission_fixture import ground_nav,exit_nav
    controller=started(gain=1.5,handoff=True)
    for i,y in enumerate((.55,.63,.71,.73,.76)):
        t=round(.3+i*.1,8);controller.step(ground_nav(t,y=y),t)
    assert controller.near_seen and controller.pass_gate_hits==3
    phases=[]
    for i in range(8,12):
        result=controller.step(lane(i/10),i/10)
        phases.append(result['maneuver_phase'])
        assert result['passage_evidence']['visual_pass_candidate']
        assert result['passage_evidence']['exit_frames']==0
    assert set(phases)=={'target_pending','unverified_recovery'}
    assert controller.count==0  # A timer/empty view cannot supply exit evidence.
    for index,t in enumerate((1.2,1.3,1.4)):
        result=controller.step(exit_nav(t,index=index),t)
    result=controller.step(lane(1.5,near=.3,far=.3),1.5)
    assert controller.count==1 and controller.auto_count==1
    assert controller.passed_ids=={'secondary-A'}
    assert result['phase']!='complete' and not result['lane_return_confirmed']


def test_target_gap_helper_includes_first_processing_age_and_never_renews_from_missing_frames():
    gap=TargetGap();nav=lane(.4)
    assert gap.update('A',False,nav,.5,1870)
    assert gap.diagnostics(.5)['target_pending_deadline_s']==pytest.approx(.65)
    assert 1610<gap.request(.5)<=1690
    assert gap.update('A',False,lane(.6),.6,1350)
    assert gap.diagnostics(.6)['target_pending_deadline_s']==pytest.approx(.65)
    assert gap.request(.6)>1610  # New missing data cannot reverse the old hold.
    assert not gap.update('A',False,lane(.65),.65,1350)
    assert gap.request(.65)==1610


def test_continuous_pair_helper_does_not_refresh_last_reliable_clock_or_request():
    recovery=ObservedLaneRecovery()
    assert recovery.limit(observed(.2),.2,'secondary-A',True,True,1870,None)==(False,80.)
    recovery.limit(lane(.3,state='missing'),.3,'secondary-A',False,False,1690,None)
    assert recovery.limit(observed(.4),.4,'secondary-A',True,False,1870,None)==(False,80.)
    resumed,limit=recovery.limit(observed(.5),.5,'secondary-A',True,False,1870,None)
    assert resumed and limit>80
    assert recovery.trusted['captured_s']==.2
    resumed,limit=recovery.limit(observed(1.21),1.21,'secondary-A',True,False,1870,None)
    assert not resumed and limit==80.


@pytest.mark.parametrize('stop',['lease','emergency'])
def test_pending_does_not_renew_real_worker_lease_or_bypass_global_emergency(rig,stop):
    now,clock,backend,manual,owner,_=rig;controller=avoiding()
    handle=BorrowedActuator(Worker(owner),actuator_settings(controller.s),CLIENT,clock=clock)
    now[0]=.6;pending=controller.step(lane(.6),.6);handle.send(pending)
    assert owner.active and pending['effective_steering_gain']==1.
    deadline=owner.guard.deadline
    now[0]=.65;cached=controller.step(lane(.6),.65)
    assert cached['fresh_control'] is False and owner.guard.deadline==deadline
    # The caller must skip cached output; status alone cannot renew the worker.
    owner.status();now[0]=.801 if stop=='lease' else .7
    if stop=='lease':owner.tick()
    else:owner.dispatch('emergency',{})
    assert not owner.active and owner.guard.esc==1400
    with pytest.raises((ValueError,RuntimeError)):handle.send(pending)
    now[0]+=.351;owner.tick();assert owner.guard is None


@pytest.mark.parametrize('state',['single','missing'])
def test_stronger_maneuver_does_not_authorize_initial_degraded_lane(state):
    controller=ContinuousConeController(settings(gain=1.5),0.,initialized=True)
    for t in (0.,.1,.2,.3):
        result=controller.step(lane(t,state=state),t)
        assert result['esc_us']==1500 and not controller.has_forward_command


def test_no_forced_alternation_overrides_the_shorter_valid_second_cone_route():
    controller=started(gain=1.5)
    controller.last_pass_side='left'
    # Both sides fit; left is nearer the current car reference. The previous
    # pass being left is not a reason to force the farther right candidate.
    result=controller.step(observed(.3,target=cone(fraction=.5),near=.35),.3)
    assert controller.side=='left' and result['requested_steering_us']>1610
    assert controller.count==0
