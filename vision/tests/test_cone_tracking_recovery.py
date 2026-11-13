"""Real turning failures and motion safeguards; all output is simulated."""
import copy
import json
from pathlib import Path

import pytest

from test_cone_latest_recovery import settings,nav,geometry
from carvision.cone_latest_engine.cone_vision import ImageConeTracker
from carvision.cone_latest_engine.cone_edge_recovery import TurnEdgeRecovery
from carvision.cone_latest_engine.cone_continuous import ContinuousConeController
from carvision.cone_latest_engine.cone_power import output_request,with_reduced_power
from carvision.cone_latest_engine.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard
from carvision.cone_latest_engine import cone_image_guidance as guidance

ROWS=json.loads((Path(__file__).parent/'fixtures/cone-regression-20261107/turn-identity-and-edge.json').read_text(encoding='utf8'))


def test_recorded_near_cone_keeps_identity_through_fast_turn(settings):
    tracker=ImageConeTracker('secondary',settings);identities=[]
    for row in ROWS:
        if row['camera']!='secondary':continue
        tracks=tracker.update(row['detections'],row['frame_id'],row['captured_s'])
        if .7<=row['t']<=1.85:
            cones=[t for t in tracks if t['visible'] and t['kind']=='cone_candidate' and t['bbox_xyxy'][1]>220]
            if cones:
                near=max(cones,key=lambda t:t['bbox_xyxy'][3])
                identities.append(near['id'])
                assert near['confirmed']
    assert len(identities)>=6 and len(set(identities))==1


def detection(x,y):
    return dict(kind='cone_candidate',bbox_xyxy=[x,y,x+20,y+30],clipped=False)


def test_two_close_cones_are_not_merged_by_motion_matching(settings):
    tracker=ImageConeTracker('secondary',settings)
    a=tracker.update([detection(150,220),detection(210,220)],1,1.)
    b=tracker.update([detection(125,225),detection(185,225)],2,1.15)
    assert [t['id'] for t in b if t['visible']]==[t['id'] for t in a]
    assert len({t['id'] for t in b if t['visible']})==2


def test_ambiguous_match_does_not_inherit_confirmation(settings):
    tracker=ImageConeTracker('secondary',settings)
    for i in range(3):tracker.update([detection(150,220),detection(190,220)],i+1,1.+i*.1)
    rows=tracker.update([detection(170,220)],4,1.3)
    fresh=[t for t in rows if t['visible']]
    assert len(fresh)==1 and not fresh[0]['confirmed']


def test_recorded_31px_stripe_motion_is_current_feedback_not_lost_line():
    recovery=TurnEdgeRecovery();accepted=[];anchor=None
    for row in ROWS:
        if row['road_context'] is None:continue
        context=row['road_context'];result=recovery.update(context,(360,480,3),row['captured_s'])
        if context['valid']:anchor=row['captured_s']
        if 2.12<=row['t']<=2.5:
            assert result is not None
            assert recovery.paired[0]==anchor
            assert result.prediction_age_s==pytest.approx(row['captured_s']-anchor)
            accepted.append(row['t'])
    assert len(accepted)>=3


@pytest.mark.parametrize('side,heading,cone_x',[('left',.073,.55),('right',-.073,.45)])
def test_mid_lane_heading_does_not_reverse_required_cone_clearance(settings,monkeypatch,side,heading,cone_x):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda n,*a,**k:geometry(n['captured_s'],heading,.5))
    guide=guidance.ImageConeGuidance(settings)
    cone=dict(id='one',fraction=cone_x,half_width=.03,image_fraction=cone_x,half_image_width=.03,y_fraction=.75,near=True)
    row={**nav(1.,[cone]),'line_missing':False,'lane_bounds_px':[[100,300],[0,400]],'ego_fraction':.5}
    result=guide.update(row,1.)
    assert result['side']==side
    assert result['command']*(-1 if side=='left' else 1)>0
    assert result['power_scale']>0 and not result['control_conflict']


def test_camera_handover_preserves_identity_without_inventing_old_side_corridor(settings,monkeypatch):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:None)
    guide=guidance.ImageConeGuidance(settings)
    a=dict(id='secondary-5',image_fraction=.55,half_image_width=.03,y_fraction=.75,near=True)
    assert guide.update(nav(1.,[a]),1.)['side']=='left'
    b={**a,'id':'primary-9','image_fraction':.43,'y_fraction':.94}
    result=guide.update({**nav(1.2,[b]),'source_camera':'primary'},1.2)
    assert result['target_id']=='primary-9'
    assert result['side'] is None or result['side'] in result['side_choice']['available_sides']
    assert result['identity_transition']=='view_handover_side_retained'
    assert guide.last_target['image_fraction']==.43


def test_reidentified_target_retains_identity_but_rejects_missing_side(settings,monkeypatch):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:None)
    guide=guidance.ImageConeGuidance(settings)
    a=dict(id='old',image_fraction=.55,half_image_width=.03,y_fraction=.65,near=True)
    guide.update(nav(1.,[a]),1.)
    result=guide.update(nav(1.55,[{**a,'id':'new','image_fraction':.45,'y_fraction':.85}]),1.55)
    assert result['target_id']=='new'
    assert result['side'] is None or result['side'] in result['side_choice']['available_sides']
    assert not result['side_choice']['locked']


def test_new_cone_cannot_resurrect_old_corridor(settings,monkeypatch):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:None)
    guide=guidance.ImageConeGuidance(settings)
    a=dict(id='first',image_fraction=.55,half_image_width=.03,y_fraction=.95,near=True,exit_band=True)
    guide.update(nav(1.,[a]),1.)
    guide.corridor_memory={'captured_s':1.,'side':'left','target_id':'first','motion_samples':0,'remaining_m':1.,'estimated_limit_s':6.}
    b={**a,'id':'second','y_fraction':.65,'image_fraction':.35,'exit_band':False}
    result=guide.update(nav(1.2,[b]),1.2)
    assert result['target_id']=='first'  # Leaving the picture does not clear the vehicle body.
    result=guide.update(nav(1.9,[b]),1.9)
    assert result['target_id']=='second' and guide.corridor_memory is None
    assert guide.update(nav(2.4),2.4)['target_id']!='first'


def test_lost_direction_reduces_power_and_uses_bounded_recent_correction(settings,monkeypatch):
    current=[geometry(1.,-.06,.55)]
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:current[0])
    controller=ContinuousConeController(settings,1.,initialized=True)
    controller.step({**nav(1.),'experimental':True},1.)
    current[0]=None
    first=controller.step({**nav(1.1),'experimental':True},1.1)
    assert first['esc_us']<settings['driving']['forward_us']
    assert first['steering_us']>1610  # Recent inward direction, not instant straight ahead.
    final=controller.step({**nav(1.75),'experimental':True},1.75)
    assert final['esc_us']>=1570
    with pytest.raises(ValueError,match='道路方向'):
        controller.step({**nav(2.11),'experimental':True},2.11)


def test_boundary_conflict_uses_one_bounded_inward_correction(settings,monkeypatch):
    current=[geometry(1.,0.,.5)]
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda n,*a,**k:current[0])
    controller=ContinuousConeController(settings,1.,initialized=True)
    cone=dict(id='one',fraction=.45,half_width=.03,image_fraction=.45,half_image_width=.03,y_fraction=.8,near=True)
    row={**nav(1.,[cone]),'experimental':True,'line_missing':False,'lane_bounds_px':[[100,300],[0,400]],'ego_fraction':.5}
    controller.step(row,1.)
    current[0]=geometry(1.1,-.07,.75)
    intent=controller.step({**row,'captured_s':1.1,'ego_fraction':.75,'cones':[{**cone,'fraction':.68}]},1.1)
    assert intent['esc_us']>=1570
    assert controller.guidance_state['side'] is None
    assert 'right' in controller.guidance_state['side_choice']['blocked_sides']
    assert intent['requested_steering_us']>1610
    assert intent['steering_us']>=1610  # Rate limit finishes unwinding before turning inward.
    assert abs(controller.guidance_state['command'])<=.35
    assert controller.guidance_state['target_id']=='one'


@pytest.mark.parametrize('forward',[1570,1575,1600,1625])
def test_every_power_stage_passes_actual_installed_guard_without_hardware(settings,forward):
    settings['driving']['forward_us']=forward
    writes=[];guard=ConePulseGuard(with_reduced_power(actuator_settings(settings)),
        lambda channel,pulse:writes.append((channel,pulse)),mode='F/B')
    previous=forward
    for sequence,scale in enumerate((1.,.75,.65,.4,.2,0.)):
        action,pulse=output_request(forward,scale)
        now=1.+sequence*.1
        guard.accept(dict(sequence=sequence,sent_s=now,control_profile='cone',action=action,
                          esc_us=pulse,steering_us=1610 if pulse==1500 else 1700),now)
        assert guard.esc==pulse and pulse<=previous and not guard.fault
        previous=pulse
    assert guard.esc==1500 and guard.actual_steering==1610


def test_reduced_power_does_not_expand_original_guard_allowlist(settings):
    guard=ConePulseGuard(actuator_settings(settings),lambda *_:None,mode='F/B')
    with pytest.raises(ValueError):
        guard.accept(dict(sequence=1,sent_s=1.,control_profile='cone',action='cone_1530',esc_us=1530,steering_us=1610),1.)
