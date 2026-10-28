"""An explicit avoidance hint retains one actual camera's current target only."""
import copy

import pytest

from carvision.cone_guidance import CurrentTargetEvidence
from test_lane_source_priority import Selection,view


def target_view(camera,t,*,state='missing',fid=None,identifier=None,**changes):
    identifier=identifier or camera+'-target'
    snapshot=view(camera,t,state=state,fid=fid,cones=[],**changes)
    snapshot['visual_cones']=[{'id':identifier,'current_pixel':True,'confirmed':True,
        'identity_revision':0,'source_camera':camera,'frame_id':snapshot['frame_id'],
        'captured_s':t,'image_fraction':300/480,'half_image_width':20/960,
        'image_reference_fraction':.5,'y_fraction':270/360,'bbox_xyxy':[290,250,310,280]}]
    return snapshot


def update(run,camera,snapshot,now):
    other='primary' if camera=='secondary' else 'secondary'
    return run.update(now,**{camera:snapshot,other:view(other,now,correction='right')})


@pytest.mark.parametrize('camera',['primary','secondary'])
@pytest.mark.parametrize('state',['single','missing'])
def test_current_locked_target_keeps_its_camera_when_other_view_has_two_lines(camera,state):
    run=Selection();run.nav.set_avoidance_target(camera,camera+'-target')
    snapshot=target_view(camera,1.,state=state)
    result=update(run,camera,snapshot,1.)
    assert result['source_camera']==camera and result['line_state']==state
    assert result['visual_cones']==snapshot['visual_cones'] and result['cones']==[]
    assert result['boundary_correction'] is None
    assert not result.get('metric_boundary_verified') and 'pass_evidence' not in result
    if state=='missing':assert not result.get('lane_bounds_px') and 'ego_fraction' not in result
    else:
        assert result['lane_bounds_px']==snapshot['lane_bounds_px']
        assert result['ego_near_fraction']==snapshot['ego_near_fraction']


def test_primary_target_hint_suppresses_three_frame_secondary_return_until_cleared():
    run=Selection();run.on_primary();run.nav.set_avoidance_target('primary','primary-target')
    for t in (.1,.2,.3):
        result=run.update(t,view('secondary',t),target_view('primary',t))
        assert result['source_camera']=='primary'
    assert run.nav.secondary_recovery.count==3
    run.nav.set_avoidance_target(None,None)
    assert run.update(.4,view('secondary',.4),view('primary',.4))['source_camera']=='secondary'


@pytest.mark.parametrize('change',[
    {'confirmed':False},{'current_pixel':False},{'id':'secondary-other'},
    {'source_camera':'primary'},{'frame_id':999},{'captured_s':.9},
    {'identity_revision':None},{'image_reference_fraction':None},
    {'half_image_width':.2},{'bbox_xyxy':[1,1,2,2]},
])
def test_incomplete_wrong_id_or_stale_pixel_metadata_does_not_pin_view(change):
    run=Selection();run.nav.set_avoidance_target('secondary','secondary-target')
    snapshot=target_view('secondary',1.)
    snapshot['visual_cones'][0].update(change)
    result=update(run,'secondary',snapshot,1.)
    assert result['source_camera']=='primary'


@pytest.mark.parametrize('change',[
    {'boundary_danger':True},{'ready':False},{'action':'stop'},
])
def test_unsafe_or_explicit_stop_view_is_not_retained_even_with_current_pixels(change):
    run=Selection();run.nav.set_avoidance_target('secondary','secondary-target')
    result=update(run,'secondary',target_view('secondary',1.,**change),1.)
    assert result['source_camera']=='primary' and not result.get('boundary_danger')


@pytest.mark.parametrize('camera',['primary','secondary'])
def test_target_pixels_without_road_cannot_override_other_view_boundary_danger(camera):
    run=Selection();run.nav.set_avoidance_target(camera,camera+'-target')
    other='primary' if camera=='secondary' else 'secondary'
    snapshots={camera:target_view(camera,1.,state='missing'),
        other:view(other,1.,boundary_danger=True,ready=False,action='stop')}
    result=run.update(1.,**snapshots)
    # Current target pixels locate a cone, not the vehicle's road clearance.
    # With no usable road in either view, retain the original fatal boundary.
    assert result['source_camera']==other and result['boundary_danger']
    assert result['ready'] is False and result['action']=='stop'


@pytest.mark.parametrize('state',['paired','single'])
def test_target_hint_preserves_original_usable_road_fallback_when_other_view_is_dangerous(state):
    run=Selection();run.nav.set_avoidance_target('secondary','secondary-target')
    result=run.update(1.,target_view('secondary',1.,state=state),
        view('primary',1.,boundary_danger=True,ready=False,action='stop'))
    assert result['source_camera']=='secondary' and result['line_state']==state
    assert not result.get('boundary_danger') and result['lane_bounds_px']


def test_cached_target_keeps_original_frame_and_expiry_cannot_be_refreshed_by_other_camera():
    run=Selection();run.nav.set_avoidance_target('secondary','secondary-target')
    cached=target_view('secondary',1.,fid=10);evidence=CurrentTargetEvidence()
    for now in (1.,1.1,1.2):
        result=update(run,'secondary',cached,now)
        assert result['source_camera']=='secondary'
        assert result['frame_id']==10 and result['captured_s']==1.
        observed=evidence.update(result,'secondary-target','left',now)
        assert not observed['target_stable'] and observed['frames']==1
    result=update(run,'secondary',cached,1.251)
    assert result['source_camera']=='primary'
    assert result['captured_s']==1.251 and all(item['id']!='secondary-target' for item in result.get('visual_cones',[]))


def test_missing_locked_camera_and_same_image_fraction_in_other_view_do_not_transfer_identity():
    run=Selection();run.nav.set_avoidance_target('secondary','secondary-target')
    result=run.update(1.,primary=target_view('primary',1.,state='paired'))
    assert result['source_camera']=='primary'
    assert result['visual_cones'][0]['id']=='primary-target'
    assert all(item['id']!='secondary-target' for item in result['visual_cones'])


def test_fresh_target_does_not_hide_mismatched_cached_navigator_frame():
    run=Selection();run.nav.set_avoidance_target('secondary','secondary-target')
    snapshot=target_view('secondary',1.,fid=10)
    # The row claims a different source frame. A cached navigator result is not
    # current evidence for that row, even if its own pixel flags are consistent.
    rows={'secondary':{'captured_s':1.1,'frame_id':11,'snapshot':snapshot,
        'road_context':{'valid':False,'turn_aware':True}},
        'primary':{'captured_s':1.1,'frame_id':11,'snapshot':view('primary',1.1,fid=11),
        'road_context':{'valid':True,'turn_aware':True}}}
    result=run.nav.update_views({'secondary':object(),'primary':object()},rows,1.1)
    assert result['source_camera']=='primary'


def test_no_hint_and_explicit_clear_preserve_original_lane_priority_and_recovery_votes():
    plain=Selection();cleared=Selection();plain.on_primary();cleared.on_primary()
    cleared.nav.set_avoidance_target('primary','primary-target')
    cleared.nav.set_avoidance_target(None,None)
    for t in (.1,.2,.3):
        expected=plain.update(t,view('secondary',t),view('primary',t))
        actual=cleared.update(t,view('secondary',t),view('primary',t))
        assert actual==expected
    assert actual['source_camera']=='secondary'


@pytest.mark.parametrize('camera,identifier',[
    ('primary','secondary-target'),('secondary','primary-target'),
    ('unknown','unknown-target'),(None,'secondary-target'),('secondary',None),
])
def test_hint_does_not_accept_foreign_camera_identity(camera,identifier):
    with pytest.raises(ValueError):Selection().nav.set_avoidance_target(camera,identifier)


def test_actual_dual_navigator_visual_contract_can_retain_secondary_without_ground_proof():
    from test_cone_visual_targets import cone,picture,observation
    value=copy.deepcopy(Selection().nav.s)
    from carvision.cone_low_speed import DualLowSpeedNavigator
    navigator=DualLowSpeedNavigator(value);navigator.set_avoidance_target('secondary','secondary-1')
    track=cone(identity_revision=0);image=picture(track)
    secondary=observation(tracks=[track]);secondary['road_context']['valid']=False
    primary=observation(camera='primary',tracks=[])
    result=navigator.update_views({'primary':image,'secondary':image},
        {'primary':primary,'secondary':secondary},1.)
    assert result['source_camera']=='secondary' and result['line_state']=='missing'
    assert result['visual_cones'][0]['id']=='secondary-1'
    assert not result.get('lane_bounds_px') and 'pass_evidence' not in result
