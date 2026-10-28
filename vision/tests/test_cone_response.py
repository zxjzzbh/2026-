"""Motion regressions for live avoidance priority and current target ownership."""
import pytest

from carvision.cone_response import combine_guidance
from test_cone_turn_controller import lane,started


def cone(identifier='secondary-old',fraction=.7,**changes):
    return {'id':identifier,'fraction':fraction,'half_width':.03,'image_fraction':.7,
            'half_image_width':.03,'near':True,'y_fraction':.65,'image_close':False,**changes}


def pixels(identifier='secondary-old',x=.72,reference=.65,**changes):
    return {'id':identifier,'current_pixel':True,'confirmed':True,'image_fraction':x,
            'half_image_width':.03,'image_reference_fraction':reference,
            'y_fraction':.85,'clipped':True,'identity_revision':0,
            'bbox_xyxy':[(x-.03)*480,.79*360,(x+.03)*480,.86*360],**changes}


def seeded():
    controller=started()
    controller.step(lane(.3,cones=[cone()]),.3)
    assert controller.target=='secondary-old' and controller.side=='left'
    return controller


def test_699_live_avoidance_does_not_disappear_inside_heading_deadband():
    controller=started()
    output=controller.step(lane(.3,near=.35,far=.258,half_envelope=.205,
        cones=[cone(fraction=.539,y_fraction=.639)]),.3)
    assert output['target_space']=='lane' and output['lateral_error']<-.07
    assert controller.pd.error<-.05  # Old sum was about -.016, inside .018 deadband.
    assert output['requested_steering_us']>1640 and output['steering_us']>1610
    assert abs(output['heading_compensation'])<=abs(output['lateral_error'])*.25+1e-9


@pytest.mark.parametrize('lateral,heading',[(-.08,.4),(.08,-.4)])
def test_opposing_heading_can_remove_at_most_one_quarter_of_live_lateral_demand(lateral,heading):
    error,correction=combine_guidance(lateral,heading,True)
    assert error*lateral>0 and abs(error)>=.75*abs(lateral)
    assert abs(correction)<=.25*abs(lateral)


def test_pure_heading_guidance_remains_available_without_lateral_avoidance():
    assert combine_guidance(0.,.2,True)==pytest.approx((.13,.13))
    controller=started()
    output=controller.step(lane(.3,near=.5,far=.3),.3)
    assert output['target_id'] is None and output['requested_steering_us']<1610


def test_single_uncertainty_does_not_reverse_an_observed_avoidance_target():
    controller=seeded()
    output=controller.step(lane(.4,state='single',near=.354,far=.292,
        half_envelope=.323,base_half_envelope=.205,boundary_uncertainty_fraction=.118,
        boundary_correction='right',cones=[cone(near_field_continuation=True)],
        visual_cones=[pixels()]),.4)
    assert output['target_space']=='image' and output['boundary_correction'] is None
    assert output['requested_steering_us']>1610 and output['steering_us']>1610
    assert output['soft_boundary_override'] and output['esc_us']<=1570
    assert abs(output['requested_steering_us']-1610)<=80


def test_single_real_near_boundary_and_explicit_fatal_stop_keep_priority():
    controller=seeded()
    output=controller.step(lane(.4,state='single',near=.1,far=.1,base_half_envelope=.2,
        half_envelope=.32,cones=[cone(near_field_continuation=True)],visual_cones=[pixels()]),.4)
    assert output['boundary_correction']=='right' and output['requested_steering_us']<1610
    with pytest.raises(ValueError,match='真实边线停止'):
        controller.step(lane(.5,boundary_danger=True,reason='真实边线停止'),.5)


def test_current_pixels_continue_across_paired_missing_without_coordinate_reuse_or_center_reset():
    controller=seeded();old_lane_goal=controller.target_fraction
    for t,state,x in ((.4,'paired',.72),(.5,'missing',.70),(.6,'paired',.74)):
        output=controller.step(lane(t,state=state,cones=[cone(near_field_continuation=True)],
            visual_cones=[pixels(x=x)]),t)
        assert output['target_id']=='secondary-old' and output['target_source']=='current_pixels'
        assert output['target_space']=='image' and controller.filtered_target==pytest.approx(x-.18)
        assert controller.filtered_target!=old_lane_goal and controller.filtered_target!=.5
        assert output['requested_steering_us']>1610 and controller.count==0


@pytest.mark.parametrize('change',[{'current_pixel':False},{'confirmed':False},
    {'image_reference_fraction':None},{'id':'foreign'},{'image_fraction':float('nan')},
    {'captured_s':.3},{'frame_id':300},{'source_camera':'primary'}])
def test_invalid_or_unknown_pixels_never_turn_historical_lane_fraction_into_a_target(change):
    controller=seeded()
    output=controller.step(lane(.4,cones=[cone(near_field_continuation=True,fraction=.05)],
        visual_cones=[pixels(**change)]),.4)
    assert output['target_source']=='target_pending' and controller.filtered_target is None
    assert abs(output['requested_steering_us']-1610)<=80 and controller.count==0
    assert output['effective_steering_gain']==1.


def test_visual_only_target_cannot_initialize_a_new_lock():
    controller=started()
    output=controller.step(lane(.3,visual_cones=[pixels()]),.3)
    assert output['target_id'] is None and output['target_source']=='fresh_lane_only'
    assert output['steering_us']==1610


def test_missing_target_has_bounded_pending_then_current_lane_without_false_pass():
    controller=seeded();old_goal=controller.target_fraction
    for k in range(16):
        t=round(.4+k*.1,8);output=controller.step(lane(t),t)
        if t<.65:
            assert controller.filtered_target is None and output['target_source']=='target_pending'
            assert output['target_pending_deadline_s']==pytest.approx(.65)
        else:
            assert controller.filtered_target==.5 and output['target_source']=='fresh_lane_only'
        assert controller.count==0
    assert old_goal!=.5 and controller.target is None
    assert 'secondary-old' in output['retired_unverified_ids']
    assert output['maneuver_phase']=='unverified_recovery' and not output['lane_return_confirmed']


def test_new_confirmed_blocker_takes_over_without_old_tail_or_stable_lane_wait():
    from cone_mission_fixture import ground_nav
    controller=started()
    for i,y in enumerate((.55,.63,.71,.73,.76)):
        t=round(.3+i*.1,8);controller.step(ground_nav(t,'secondary-old',y=y),t)
    assert controller.pass_gate_hits==3 and controller.pass_evidence['visual_pass_candidate']
    controller.step(lane(.8,state='missing',visual_cones=[pixels('secondary-old')]),.8)
    nav=ground_nav(.9,'secondary-new',side='right',y=.65)
    nav.update(ego_fraction=.5,ego_near_fraction=.5,ego_far_fraction=.5)
    target=nav['cones'][0];target['fraction']=.44
    target['pass_evidence'].update(reference_fraction_at_foot=.5,cone_fraction_at_foot=.44)
    visual=nav['visual_cones'][0]
    visual.update(image_fraction=.452,image_reference_fraction=.5)
    visual['bbox_xyxy']=[(.452-.02)*480,(.65-.06)*360,(.452+.02)*480,(.65+.01)*360]
    target['bbox_xyxy']=visual['bbox_xyxy'];target['image_fraction']=.452
    output=controller.step(nav,.9)
    assert not output['paired_stable']
    assert output['target_id']=='secondary-new' and output['target_space']=='lane'
    assert output['target_handoffs']==1 and 'secondary-old -> secondary-new' in output['target_handoff_reason']
    assert 'secondary-old' in output['retired_unverified_ids'] and 'secondary-old' not in controller.passed_ids
    assert output['mission_unresolved'][0]['id']=='secondary-old'
    assert controller.count==0 and controller.pass_gate_hits==0 and controller.tail_powered==0
    assert controller.side=='right' and output['requested_steering_us']<1610


@pytest.mark.parametrize('change',[{'provisional':True},{'confirmed':False}])
def test_unconfirmed_blocker_cannot_take_over(change):
    controller=seeded()
    output=controller.step(lane(.4,cones=[cone('new',fraction=.44,**change)]),.4)
    assert output['target_id']=='secondary-old' and output['target_handoffs']==0


def test_farther_cone_cannot_steal_a_current_foreground_target():
    controller=seeded()
    output=controller.step(lane(.4,cones=[cone(y_fraction=.75),cone('far',fraction=.44,y_fraction=.5)]),.4)
    assert output['target_id']=='secondary-old' and output['target_handoffs']==0


def test_duplicate_current_visual_does_not_refresh_target_tail_or_motion_lease():
    controller=seeded()
    nav=lane(.4,cones=[cone(near_field_continuation=True)],visual_cones=[pixels()])
    first=controller.step(nav,.4)
    before=(controller.pd.error,controller.pd.derivative,controller.tail_powered,controller.target_handoffs)
    cached=controller.step(nav,.5)
    assert cached['fresh_control'] is False and cached['steering_us']==first['steering_us']
    assert (controller.pd.error,controller.pd.derivative,controller.tail_powered,controller.target_handoffs)==before


def test_unconfirmed_pixels_do_not_extend_one_second_no_guidance_budget():
    controller=seeded()
    for t in (.4,.5,.6,.7,.8,.9,1.,1.1,1.2,1.3):
        output=controller.step(lane(t,state='missing',visual_cones=[pixels(confirmed=False)]),t)
        assert output['esc_us']<=1570 and abs(output['steering_us']-1610)<=80
    with pytest.raises(ValueError,match='均无可靠引导超过 1 秒'):
        controller.step(lane(1.4,state='missing',visual_cones=[pixels(confirmed=False)]),1.4)

