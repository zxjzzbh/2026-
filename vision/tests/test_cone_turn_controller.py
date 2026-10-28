"""Actual obstacle commands from synthetic lane observations; no hardware I/O."""
import copy
from pathlib import Path

import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_low_speed import trial_settings
from carvision.cone_navigation import actuator_settings
from carvision.cone_planning import load_settings
from carvision.cone_trial_output import ConePulseGuard


ROOT=Path(__file__).resolve().parents[2]


def settings(forward=1600,gain=1.):
    source=load_settings(ROOT/'vision/tests/fixtures/configs/cone-trial-field-20261026.json')
    return trial_settings(source,{'test_forward_us':forward,'test_steering_gain':gain})


def lane(t,*,state='paired',near=.5,far=.5,cones=None,camera='secondary',fid=None,**changes):
    nav={'experimental':True,'ready':True,'captured_s':t,
         'frame_id':round(t*1000) if fid is None else fid,'source_camera':camera,
         'line_state':state,'line_held':state=='single','line_missing':state=='missing',
         'boundary_observed':state=='paired','ego_fraction':(near+far)/2,
         'ego_near_fraction':near,'ego_far_fraction':far,'heading_error':near-far,
         'half_envelope':.20,'lane_bounds_px':[[100,380],[50,430]],
         'cones':[] if cones is None else cones,'uncertain':[],'uncertain_cones':[]}
    if state=='single':nav.update(prediction_age_s=.1,line_age_s=.1,boundary_uncertainty_fraction=.05)
    if state=='missing':
        for key in ('lane_bounds_px','ego_fraction','ego_near_fraction','ego_far_fraction','heading_error'):
            nav.pop(key)
    nav.update(changes)
    if 'visual_cones' in nav:
        nav['visual_cones']=[dict(entry) for entry in nav['visual_cones']]
        for entry in nav['visual_cones']:
            for key in ('captured_s','frame_id','source_camera'):
                entry.setdefault(key,nav[key])
    return nav


def started(*,forward=1600,gain=1.,handoff=False,stop_for_cones=False):
    controller=ContinuousConeController(settings(forward,gain),0.,initialized=True,handoff_after_two=handoff,
                                        stop_for_cones=stop_for_cones)
    for t in (0.,.1,.2):output=controller.step(lane(t),t)
    assert output['esc_us']==forward and output['fresh_control']
    return controller


def test_only_three_distinct_same_camera_paired_frames_over_150ms_can_start():
    controller=ContinuousConeController(settings(),0.,initialized=True)
    first=lane(0.,fid=1)
    assert controller.step(first,0.)['esc_us']==1500
    for now in (.02,.04,.06):
        output=controller.step(first,now)
        assert output['esc_us']==1500 and not output['fresh_control']
    assert controller.lane_evidence.count==1
    assert controller.step(lane(.10,fid=2),.10)['esc_us']==1500
    output=controller.step(lane(.20,fid=3),.20)
    assert output['action']=='search' and output['esc_us']==1600


@pytest.mark.parametrize('state',['single','missing'])
def test_initial_degraded_lane_never_moves_or_spends_a_moving_recovery_budget(state):
    controller=ContinuousConeController(settings(),0.,initialized=True)
    for i in range(30):
        t=i*.1
        output=controller.step(lane(t,state=state),t)
        assert output['action']=='neutral' and output['steering_us']==1610
        assert controller.line_missing_since is None
    with pytest.raises(ValueError,match='画面'):
        controller.step(lane(2.9,state=state),3.2)


def test_camera_switches_cannot_pool_initial_paired_frames():
    controller=ContinuousConeController(settings(),0.,initialized=True)
    for i in range(8):
        t=i*.1
        assert controller.step(lane(t,camera='primary' if i%2 else 'secondary'),t)['esc_us']==1500


def test_legacy_nav_is_paired_only_when_its_two_edges_are_fresh():
    controller=ContinuousConeController(settings(),0.,initialized=True)
    for t in (0.,.1,.2):
        data=lane(t)
        for key in ('line_state','boundary_observed','ego_near_fraction','ego_far_fraction','heading_error'):
            data.pop(key)
        output=controller.step(data,t)
    assert output['esc_us']==1600
    data=lane(.3,line_held=True)
    data.pop('line_state')
    output=controller.step(data,.3)
    assert output['esc_us']<=1570 and abs(output['steering_us']-1610)<=80


def test_duplicate_control_does_not_recenter_or_refresh_the_motion_lease():
    controller=started();writes=[]
    guard=ConePulseGuard(actuator_settings(controller.s),lambda *args:writes.append(args),mode='F/B')
    sample=lane(.3,near=.65,far=.50)
    output=controller.step(sample,.3)
    guard.accept({**output,'sequence':1,'sent_s':.3},.3)
    before=(controller.pd.error,controller.pd.derivative,controller.actual_steering)
    for now in (.32,.34,.36):
        cached=controller.step(sample,now)
        assert not cached['fresh_control'] and cached['steering_us']==output['steering_us']
        assert (controller.pd.error,controller.pd.derivative,controller.actual_steering)==before
    guard.tick(.51)
    assert guard.fault and guard.esc==1400


@pytest.mark.parametrize('state',['single','missing'])
def test_first_degraded_frame_withdraws_old_full_lock_and_lowers_throttle(state):
    controller=started();controller.actual_steering=1870.
    output=controller.step(lane(.3,state=state),.3)
    assert output['esc_us']<=1570 and output['action']=='cone_'+str(output['esc_us'])
    assert abs(output['steering_us']-1610)<=80
    assert output['steering_phase']=='limited_recovery' and not output['paired_stable']


def test_one_good_pair_cannot_restart_the_one_second_no_guidance_budget():
    controller=started()
    controller.step(lane(.3,state='missing'),.3)
    controller.step(lane(.4),.4)
    controller.step(lane(.5,state='missing'),.5)
    for t in (.6,.7,.8,.9,1.,1.1,1.299):
        output=controller.step(lane(t,state='missing'),t)
        assert 1500<output['esc_us']<=1570 and abs(output['steering_us']-1610)<=80
        assert not output['paired_stable'] and controller.line_missing_since==pytest.approx(.3)
    with pytest.raises(ValueError,match='均无可靠引导超过 1 秒'):
        controller.step(lane(1.3,state='missing'),1.3)


def test_expired_single_prediction_enters_none_budget_from_its_first_current_capture():
    controller=started()
    controller.step(lane(.3,state='single',line_observed_s=.2,prediction_age_s=.16),.36)
    assert controller.line_missing_since is None  # A valid current edge is guidance.
    for now in (.59,.79,.99,1.19,1.39,1.529):
        captured=round(now-.06,8)
        output=controller.step(lane(captured,state='single',line_observed_s=.2,
                                    prediction_age_s=now-.2),now)
        assert output['line_state']=='missing'  # A one-second budget never extends single prediction.
        assert output['esc_us']<=1570 and controller.line_missing_since==pytest.approx(.53)
    with pytest.raises(ValueError,match='均无可靠引导超过 1 秒'):
        controller.step(lane(1.47,state='missing'),1.53)


def test_stable_paired_recovery_resets_the_budget_but_keeps_limits_until_confirmed():
    controller=started()
    for t in (.3,.4,.5,.6,.7,.8):
        output=controller.step(lane(t,state='missing'),t)
        assert output['esc_us']<=1570 and abs(output['steering_us']-1610)<=80
    for t in (.9,1.):
        output=controller.step(lane(t),t)
        assert output['esc_us']<=1570 and not output['paired_stable']
        assert controller.line_missing_since==pytest.approx(.3)
    output=controller.step(lane(1.1),1.1)
    assert output['paired_stable'] and controller.line_missing_since is None
    output=controller.step(lane(1.2,state='single',line_observed_s=1.1),1.2)
    assert output['esc_us']<=1570 and controller.line_missing_since is None
    output=controller.step(lane(1.4,state='single',line_observed_s=1.1,prediction_age_s=.3),1.4)
    assert output['line_state']=='missing' and controller.line_missing_since==pytest.approx(1.4)


def test_one_second_recovery_does_not_extend_camera_freshness_or_single_prediction():
    controller=started()
    output=controller.step(lane(.45,state='single',line_observed_s=.2,prediction_age_s=.25),.45)
    assert output['line_state']=='single' and output['esc_us']<=1570
    output=controller.step(lane(.451,state='single',line_observed_s=.2,prediction_age_s=.251),.451)
    assert output['line_state']=='missing' and output['esc_us']<=1570
    assert controller.progress()['line_recovery_s']==1.
    with pytest.raises(ValueError,match='画面'):
        controller.step(lane(.46,state='missing'),.711)


def test_single_lane_cannot_refresh_or_commit_an_old_cone_pass():
    from cone_mission_fixture import ground_nav
    controller=started()
    for i,y in enumerate((.55,.63,.71,.73,.76)):
        t=round(.3+i*.1,8);controller.step(ground_nav(t,y=y),t)
    assert controller.pass_gate_hits==3 and controller.pass_evidence['visual_pass_candidate']
    output=controller.step(lane(.8,state='single'),.8)
    assert controller.count==0 and controller.target=='secondary-A'
    assert controller.pass_gate_hits==0 and not output['tail_clearance_verified']
    assert controller.pass_evidence['anchor_valid']
    assert controller.pass_evidence['exit_frames']==0 and not controller.pass_evidence['visual_estimate_ready']
    # Neither a single edge nor later blank paired frames are current visual
    # exit evidence. Expiring the observation budget cannot create a pass.
    for t in (.9,1.,1.1,1.2,1.3,1.4,1.5,1.6,1.7):output=controller.step(lane(t),t)
    assert controller.count==0 and not controller.passed_ids
    assert output['mission_unresolved'][0]['id']=='secondary-A'
    assert output['pass_evidence_kind']=='image_sequence_estimate'
    assert not output['tail_clearance_verified']


def test_single_lane_does_not_add_a_third_exit_gate_frame():
    from cone_mission_fixture import ground_nav
    controller=started()
    for i,y in enumerate((.55,.63,.71,.73)):
        t=round(.3+i*.1,8);controller.step(ground_nav(t,y=y),t)
    assert controller.pass_gate_hits==2
    data=ground_nav(.7,y=.76)
    data.update(line_state='single',line_held=True,boundary_observed=False,prediction_age_s=.1)
    controller.step(data,.7)
    assert controller.pass_gate_hits==0 and controller.count==0
    # The earlier same-slice anchor survives; the single edge itself supplies
    # neither a new ground gate nor the required later bottom/side exit.
    assert controller.pass_evidence['anchor_valid']
    assert controller.pass_evidence['exit_frames']==0
    assert not controller.pass_evidence['visual_estimate_ready']


def test_small_distant_target_never_demands_immediate_endpoint_even_with_high_gain():
    controller=started(gain=3.)
    cone={'id':'far-small','fraction':.55,'half_width':.03,'near':False,
          'y_fraction':.65,'bbox_xyxy':[240,210,258,234],'image_close':False}
    for t in (.3,.4,.5,.6,.7):
        output=controller.step(lane(t,cones=[copy.deepcopy(cone)]),t)
        assert 1350<output['steering_us']<1870
        assert abs(output['requested_steering_us']-1610)<=117
        assert output['small_target_limited']


@pytest.mark.parametrize('forward',[1501,1550,1575,1600,1625])
def test_turning_and_degraded_commands_never_exceed_selected_throttle_and_use_frozen_guard(forward):
    controller=started(forward=forward)
    guard=ConePulseGuard(actuator_settings(controller.s),lambda *args:None,mode='F/B')
    turning=controller.step(lane(.3,near=.5,far=.2),.3)
    assert turning['esc_us']<=min(forward,1570)
    guard.accept({**turning,'sequence':1,'sent_s':.3},.3)
    recovery=controller.step(lane(.4,state='single'),.4)
    assert recovery['esc_us']<=min(forward,1570)
    guard.accept({**recovery,'sequence':2,'sent_s':.4},.4)
    assert not guard.fault


def test_existing_close_cone_clearance_veto_still_stops_the_run():
    controller=started(stop_for_cones=True)
    cone={'id':'danger','fraction':.62,'half_width':.03,'near':True,
          'y_fraction':.75,'image_close':True}
    with pytest.raises(ValueError,match='净空'):
        controller.step(lane(.3,cones=[cone]),.3)


def test_s3_center_and_two_passes_do_not_mean_vehicle_has_returned_to_lane():
    from cone_mission_fixture import estimate_one
    controller=started(handoff=True)
    estimate_one(controller,.3,'secondary-A');estimate_one(controller,1.1,'secondary-B')
    assert controller.count==2 and controller.mission.completion_ready()
    # Stored command state is not vehicle pose or wheel-angle feedback.
    controller.actual_steering=1610.
    assert controller.actual_steering==1610
    for t in (1.9,2.,2.1,2.2):
        output=controller.step(lane(t,near=.68,far=.68),t)
        assert output['phase']!='complete' and not output['lane_return_confirmed']
    for t in (2.3,2.4,2.5):
        output=controller.step(lane(t),t)
        assert output['phase']!='complete'
    output=controller.step(lane(2.6),2.6)
    assert output['phase']=='complete' and output['lane_return_confirmed']
    assert not output['tail_clearance_verified']


def test_centered_near_position_cannot_hide_bad_heading_or_single_lane():
    from cone_mission_fixture import estimate_one
    controller=started(handoff=True)
    estimate_one(controller,.3,'secondary-A');estimate_one(controller,1.1,'secondary-B')
    assert controller.count==2 and controller.mission.completion_ready()
    for t in (1.9,2.,2.1,2.2):
        assert controller.step(lane(t,near=.5,far=.25),t)['phase']!='complete'
    output=controller.step(lane(2.3,state='single'),2.3)
    assert output['phase']!='complete' and not output['lane_return_confirmed']


@pytest.mark.parametrize('danger',[{'boundary_danger':True},{'ready':False},{'action':'stop'}])
def test_fatal_navigation_requests_are_not_hidden_by_waiting_or_recovery(danger):
    controller=ContinuousConeController(settings(),0.,initialized=True)
    with pytest.raises(ValueError):controller.step(lane(0.,**danger),0.)
