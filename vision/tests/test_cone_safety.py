"""Shared obstacle stop gates and ordered physical-target passage regressions."""
import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_low_speed import LowSpeedController, trial_settings
from carvision.cone_navigation import actuator_settings
from carvision.shared_executor import BorrowedActuator
from test_cone_low_speed import settings, observation
from test_cone_turn_controller import lane
from cone_mission_fixture import ground_nav,exit_nav
from test_shared_executor import CLIENT,Worker,rig


def warm_continuous(controller):
    """Consume actual paired frames; do not bypass the production start gate."""
    for t in (.1,.2,.3):
        result=controller.step(observation(t,cones=[]),t)
    assert result['esc_us']>1500 and controller.has_forward_command
    return .3


@pytest.mark.parametrize('kind', ['bounded_trial', 'continuous_trial', 'race'])
def test_all_modes_consume_explicit_navigation_stop_and_readiness(kind, settings):
    for change in ({'boundary_danger': True}, {'ready': False}, {'action': 'stop'}):
        options=trial_settings(settings)
        if kind=='bounded_trial':controller=LowSpeedController(options,0.,initialized=True)
        else:
            options['low_speed_test'].update(manual_stop_only=True,max_run_s=None)
            controller=ContinuousConeController(options,0.,initialized=True,handoff_after_two=kind=='race')
        controller.step(observation(.1,cones=[]),.1)
        with pytest.raises(ValueError):controller.step(observation(.2,cones=[],**change),.2)


@pytest.mark.parametrize('controller_type', [LowSpeedController, ContinuousConeController])
def test_no_side_with_observed_lane_space_stops_before_any_forward_output(controller_type, settings):
    # The explicitly restored collision-stop option must keep the old veto;
    # the bounded legacy controller retains its original default behavior.
    options={'stop_for_cones':True} if controller_type is ContinuousConeController else {}
    controller=controller_type(trial_settings(settings),0.,initialized=True,**options)
    t=warm_continuous(controller)+.1 if controller_type is ContinuousConeController else .1
    blocked={'id':'blocked','fraction':.5,'half_width':.35,'y_fraction':.55,'near':False}
    with pytest.raises(ValueError,match='间隙'):
        controller.step(observation(t,cones=[blocked]),t)
    assert controller.count==0


@pytest.mark.parametrize('controller_type', [LowSpeedController, ContinuousConeController])
def test_old_held_lane_cannot_restart_recovery_budget_by_changing_to_image_only(controller_type, settings):
    controller=controller_type(trial_settings(settings),0.,initialized=True)
    t=warm_continuous(controller) if controller_type is ContinuousConeController else .1
    if controller_type is LowSpeedController:controller.step(observation(t,cones=[]),t)
    source={'source_camera':'secondary'} if controller_type is ContinuousConeController else {}
    result=controller.step(observation(t+.1,cones=[],line_held=True,line_age_s=.1,**source),t+.1)
    if controller_type is ContinuousConeController:
        assert result['guidance_mode']=='none'
        assert result['guidance_budget']['none_since_s']==pytest.approx(t+.1)
    lost=observation(t+.35,cones=[],line_missing=True,**source)
    lost.pop('lane_bounds_px');lost.pop('ego_fraction')
    result=controller.step(lost,t+.35)
    assert result['action']==('cone_1570' if controller_type is ContinuousConeController else 'search')
    # Continuous now measures the no-current-road/no-current-target episode
    # from this held frame, never from the cached lane's claimed pixel age.
    deadline=t+1.1 if controller_type is ContinuousConeController else t+.4
    # Keep camera captures fresh throughout recovery; the shared bounded
    # controller retains .4 s while obstacle continuous recovery gets 1 s.
    if controller_type is ContinuousConeController:
        for elapsed in (.4,.5,.6,.7,.8,.9,1.,1.099):
            lost=observation(t+elapsed,cones=[],line_missing=True,**source)
            lost.pop('lane_bounds_px');lost.pop('ego_fraction')
            result=controller.step(lost,t+elapsed)
            assert result['action']=='cone_1570' and abs(result['steering_us']-1610)<=80
    lost=observation(deadline,cones=[],line_missing=True,**source)
    lost.pop('lane_bounds_px');lost.pop('ego_fraction')
    message='均无可靠引导超过 1 秒' if controller_type is ContinuousConeController else '恢复预算'
    with pytest.raises(ValueError,match=message):controller.step(lost,deadline)
    assert controller.count==0


@pytest.mark.parametrize('controller_type', [LowSpeedController, ContinuousConeController])
def test_stale_held_lane_is_not_current_guidance_and_uses_each_controller_budget(controller_type, settings):
    controller=controller_type(trial_settings(settings),0.,initialized=True)
    t=warm_continuous(controller)+.1 if controller_type is ContinuousConeController else .1
    if controller_type is LowSpeedController:
        with pytest.raises(ValueError,match='恢复预算'):
            controller.step(observation(t,cones=[],line_held=True,line_age_s=.5),t)
        return
    # Old held pixels are not current paired road evidence. They may only use
    # the bounded none episode, with current camera frames throughout.
    for elapsed in (0.,.1,.2,.3,.4,.5,.6,.7,.8,.9,.999):
        captured=round(t+elapsed,8)
        result=controller.step(observation(captured,cones=[],source_camera='secondary',
                              line_held=True,line_age_s=1.1+elapsed),captured)
        assert result['guidance_mode']=='none' and result['action']=='cone_1570'
        assert result['guidance_budget']['none_since_s']==pytest.approx(t)
        assert controller.current_line_state=='missing' and controller.count==0
    with pytest.raises(ValueError,match='均无可靠引导超过 1 秒'):
        controller.step(observation(t+1.,cones=[],source_camera='secondary',
                        line_held=True,line_age_s=2.1),t+1.)


def test_continuous_stale_lane_before_any_motion_waits_in_neutral_without_spending_recovery_budget(settings):
    controller=ContinuousConeController(trial_settings(settings),0.,initialized=True)
    for t in (.1,.3,.5,.7):
        result=controller.step(observation(t,cones=[],line_held=True,line_age_s=.5),t)
        assert result['action']=='neutral' and result['esc_us']==1500
    assert not controller.has_forward_command and controller.line_missing_since is None


@pytest.mark.parametrize('uncertain', [False, True])
@pytest.mark.parametrize('controller_type', [LowSpeedController, ContinuousConeController])
def test_inward_boundary_correction_cannot_enter_near_cone_clearance(controller_type, uncertain, settings):
    options={'stop_for_cones':True} if controller_type is ContinuousConeController else {}
    controller=controller_type(trial_settings(settings),0.,initialized=True,**options)
    t=warm_continuous(controller)+.1 if controller_type is ContinuousConeController else .1
    cone={'id':'near','fraction':.35,'half_width':.03,'y_fraction':.75,'near':True,'image_close':True}
    nav=observation(t,cones=[] if uncertain else [cone],ego_fraction=.15,ego_near_fraction=.15,
                    boundary_correction='right')
    if uncertain:nav.update(uncertain=['near'],uncertain_cones=[{**cone,'provisional':True}])
    with pytest.raises(ValueError,match='净空'):controller.step(nav,t)


@pytest.mark.parametrize('case',['no_gap','near','provisional'])
def test_default_continuous_contact_policy_keeps_moving_with_cone_only_conflicts(case,settings):
    controller=ContinuousConeController(trial_settings(settings),0.,initialized=True)
    t=warm_continuous(controller)+.1
    if case=='no_gap':
        cone={'id':'wide','fraction':.5,'half_width':.35,'y_fraction':.55,'near':False}
        nav=observation(t,cones=[cone])
    else:
        cone={'id':'near','fraction':.35,'half_width':.03,'y_fraction':.75,'near':True,'image_close':True}
        nav=observation(t,cones=[] if case=='provisional' else [cone],ego_fraction=.15,
                        ego_near_fraction=.15,boundary_correction='right')
        if case=='provisional':nav.update(uncertain=['near'],uncertain_cones=[{**cone,'provisional':True}])
    result=controller.step(nav,t)
    assert result['esc_us']>1500 and result['phase']=='driving'
    assert 1350<=result['steering_us']<=1870 and controller.count==0
    assert controller.progress()['cone_contact_stop_enabled'] is False


@pytest.mark.parametrize('change',[{'boundary_danger':True},{'ready':False},{'action':'stop'}])
def test_default_contact_continue_does_not_override_fatal_navigation_while_moving(change,settings):
    controller=ContinuousConeController(trial_settings(settings),0.,initialized=True)
    t=warm_continuous(controller)+.1
    blocked={'id':'wide','fraction':.5,'half_width':.35,'y_fraction':.55,'near':False}
    with pytest.raises(ValueError):controller.step(observation(t,cones=[blocked],**change),t)


def test_default_contact_continue_keeps_the_current_camera_freshness_limit(settings):
    controller=ContinuousConeController(trial_settings(settings),0.,initialized=True)
    warm_continuous(controller)
    with pytest.raises(ValueError,match='画面'):
        controller.step(observation(.4,cones=[]),.651)


@pytest.mark.parametrize('stop',['lease','emergency'])
def test_contact_continue_still_uses_shared_worker_lease_and_global_emergency(stop,settings,rig):
    now,clock,backend,manual,owner,_=rig
    controller=ContinuousConeController(trial_settings(settings),0.,initialized=True)
    t=warm_continuous(controller)+.1
    handle=BorrowedActuator(Worker(owner),actuator_settings(controller.s),CLIENT,clock=clock)
    blocked={'id':'wide','fraction':.5,'half_width':.35,'y_fraction':.55,'near':False}
    now[0]=t;intent=controller.step(observation(t,cones=[blocked]),t);handle.send(intent)
    assert owner.active and owner.guard.esc==intent['esc_us']>1500
    now[0]=.601 if stop=='lease' else .41
    if stop=='lease':owner.tick()
    else:owner.dispatch('emergency',{})
    assert not owner.active and owner.guard.esc==1400
    with pytest.raises((ValueError,RuntimeError)):handle.send(intent)
    now[0]+=.351;owner.tick()
    assert owner.guard is None
    motor=[row for row in backend.writes if row[0] in ('motion','neutral')]
    assert motor[-1][0]=='neutral'
    assert all(row[3]<=.2 for row in motor if row[0]=='motion')


def test_two_distinct_same_side_passes_complete_race_without_counting_reappearing_old_id(settings):
    controller=ContinuousConeController(trial_settings(settings),0.,initialized=True,handoff_after_two=True)
    t=warm_continuous(controller)
    identifiers=('secondary-first','secondary-second')
    for identifier in identifiers:
        for y in (.55,.63,.71,.73,.76):
            t=round(t+.1,8)
            controller.step(ground_nav(t,identifier,side='left',y=y),t)
        assert controller.side=='left' and controller.pass_gate_hits==3
        assert controller.pass_evidence['visual_pass_candidate']
        assert not controller.pass_evidence['visual_estimate_ready']
        assert controller.mission.registered('secondary',identifier)
        for index in range(3):
            t=round(t+.1,8)
            before=controller.count
            result=controller.step(exit_nav(t,identifier,side='left',index=index),t)
            if index<2:assert controller.count==before
        assert controller.count==(1 if identifier==identifiers[0] else 2)
        if identifier==identifiers[0]:
            t=round(t+.1,8);controller.step(ground_nav(t,identifier,side='left',y=.55),t)
            assert controller.target is None and controller.count==1
    assert result['phase']!='complete' and not controller.return_confirmed
    # Counting the second image passage is not enough to hand off. Require
    # current paired lane position and heading to remain centered for .3 s.
    for _ in range(2):
        t=round(t+.1,8)
        result=controller.step(lane(t,near=.55,far=.45),t)
        assert result['phase']!='complete'
    for _ in range(4):
        t=round(t+.1,8);result=controller.step(lane(t),t)
    assert result['phase']=='complete' and controller.auto_count==2
    assert controller.passed_ids==set(identifiers)
    assert len(controller.mission.diagnostics()['mission_passed_estimates'])==2


def test_two_exit_frames_and_elapsed_tail_time_do_not_manufacture_third_clearance_frame(settings):
    controller=ContinuousConeController(trial_settings(settings),0.,initialized=True,handoff_after_two=True)
    start=warm_continuous(controller)
    for i,y in enumerate((.55,.63,.71,.73)):
        t=round(start+.1+i*.1,8)
        controller.step(ground_nav(t,'secondary-one',side='left',y=y),t)
    assert controller.pass_gate_hits==2
    assert controller.pass_evidence['visual_pass_candidate']
    assert controller.pass_evidence['anchor_valid']
    assert not controller.pass_evidence['visual_estimate_ready']
    deadline=controller.pass_evidence['observation_deadline_s']
    # Two current bottom/side exit frames still cannot replace the third;
    # powered tail time has no authority to turn an anchor into a passage.
    for index in range(2):
        t=round(start+.5+index*.1,8)
        controller.step(exit_nav(t,'secondary-one',side='left',index=index),t)
    assert controller.pass_evidence['exit_frames']==2
    assert not controller.pass_evidence['visual_estimate_ready']
    for i in range(14):
        t=round(start+.7+i*.1,8);controller.step(lane(t),t)
    assert controller.count==0 and controller.unverified_passes==0
    assert controller.pass_evidence['anchor_valid']
    assert controller.pass_evidence['exit_frames']==0
    assert controller.pass_evidence['observation_deadline_s']==deadline
    while t<deadline:
        t=min(deadline,round(t+.1,8));controller.step(lane(t),t)
    assert controller.count==0 and controller.unverified_passes==1
    assert not controller.mission.diagnostics()['mission_passed_estimates']
    assert not controller.mission.completion_ready()
