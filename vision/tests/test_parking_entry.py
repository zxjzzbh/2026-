"""Pure parking-entry intent tests: never open a device or a network socket."""
import copy
import json
from pathlib import Path

import pytest

from carvision.brake_parking import load_settings
from carvision.parking_entry import ParkingEntryController
from carvision.cone_trial_output import ConePulseGuard


ROOT=Path(__file__).resolve().parents[2]
ALIGNED=[[100,330],[380,330],[320,180],[160,180]]
NEAR=[[100,330],[380,330],[320,220],[160,220]]
RIGHT=[[260,330],[440,330],[400,160],[280,160]]
LEFT=[[40,330],[220,330],[200,160],[80,160]]
STOP=[[100,345],[380,345],[330,235],[150,235]]


def settings():
    return load_settings(ROOT/'vision/configs/brake-parking.json')


def sample(i, *, side='right', q=None, moving=True, valid=True, blocked=False):
    t=round(i*.1,3)
    return {'fresh':True,'frame_id':i,'captured_s':t,'image_size':[480,360],
            'motion':{'valid':valid,'moving':moving,'source':'ground_optical_estimate'},
            'parking':{'slots':[{'id':side,'availability':'blocked' if blocked else 'clear',
                                 'corners_px':copy.deepcopy(ALIGNED if q is None else q),
                                 'occupancy_verified':False}],
                       'metric_geometry':False}}


def tick(c,i,**kwargs):
    return c.step(sample(i,**kwargs),round(i*.1,3))


def armed(c,**kwargs):
    for i in range(30):
        result=tick(c,i,**kwargs)
        assert result['esc_us']==1500 and result['phase']=='arming'
    return tick(c,30,**kwargs)


@pytest.mark.parametrize('side,q,direction',[('left',LEFT,1),('right',RIGHT,-1)])
def test_three_seconds_neutral_then_proportional_turn_to_explicit_side(side,q,direction):
    c=ParkingEntryController(settings(),side)
    result=armed(c,side=side,q=q)
    assert result['phase']=='approach' and result['esc_us']==1575
    assert direction*(result['steering_us']-1610)>0
    assert abs(result['steering_us']-1610)<=26
    assert 1350<=result['steering_us']<=1870
    assert result['action']=='search'
    assert not result['field_verified'] and result['actual_pose_not_verified']
    assert result['metric_pose'] is None and result['distance_estimate_m'] is None


def test_selected_side_stays_fixed_when_recommendation_changes():
    c=ParkingEntryController(settings(),'right');armed(c,q=RIGHT)
    s=sample(31,q=RIGHT);s['parking']['selected_slot']='left'
    result=c.step(s,3.1)
    assert result['selected_side']=='right' and result['esc_us']>1500


def test_fifty_cm_reference_normalization_and_finite_brake_then_optical_stop():
    original=settings();c=ParkingEntryController(original)
    armed(c,q=NEAR)
    result=tick(c,31,q=STOP)
    assert result['phase']=='brake_pulse' and result['esc_us']==1400
    assert result['geometry']['far_edge_y_normalized']==235/359
    assert result['brake_trigger']['distance_m']==.5
    assert not result['reference_transfer_verified']
    assert original['brake_trigger_distance_m']!=.5
    pulses=[]
    for i in range(32,49):
        result=tick(c,i,q=STOP,moving=False)
        pulses.append(result['esc_us'])
    assert pulses[0]==1400 and all(p==1500 for p in pulses[1:])
    assert result['phase']=='done' and result['done']
    assert not result['whole_vehicle_inside_verified']
    assert not result['physical_stop_verified'] and not result['parking_zone_verified']
    assert result['actual_pose_not_verified'] and result['metric_pose'] is None
    assert result['esc_us']==1500
    json.dumps(result,allow_nan=False)


def test_unknown_or_interrupted_stationary_evidence_cannot_finish():
    c=ParkingEntryController(settings());armed(c,q=NEAR);tick(c,31,q=STOP)
    for i in range(32,44):
        result=tick(c,i,q=STOP,moving=False,valid=i%4!=0)
        assert not result['done']
    for i in range(44,69):
        result=tick(c,i,q=STOP,moving=None,valid=False)
    assert result['phase']=='fault' and not result['done']


def test_moving_feedback_requests_only_bounded_number_of_brake_pulses():
    c=ParkingEntryController(settings());armed(c,q=NEAR);tick(c,31,q=STOP)
    output=[tick(c,i,q=STOP,moving=True) for i in range(32,69)]
    assert max(v.get('brake_pulses',0) for v in output)<=3
    assert output[-1]['phase']=='fault' and output[-1]['esc_us']==1500
    assert sum(v['esc_us']==1400 for v in output)<=5


@pytest.mark.parametrize('side',['left','right'])
def test_selected_side_blue_board_stops_entry_and_never_switches_sides(side):
    c=ParkingEntryController(settings(),side);armed(c,side=side)
    result=tick(c,31,side=side,blocked=True)
    assert result['phase']=='safety_brake' and result['esc_us']==1400
    assert result['selected_side']==side
    result=tick(c,33,side=side)
    assert result['phase']=='fault' and result['esc_us']==1500


def test_board_direction_hint_overrides_recent_cached_polygon():
    c=ParkingEntryController(settings());armed(c)
    s=sample(31);s['parking']={'slots':[],'direction_hint':{'blocked_side':'right'}}
    result=c.step(s,3.1)
    assert result['phase']=='safety_brake'


def test_geometry_absent_never_drives_on_default_selection():
    c=ParkingEntryController(settings())
    for i in range(65):
        s=sample(i);s['parking']={'slots':[],'selected_slot':'right','tentative':True}
        result=c.step(s,round(i*.1,3))
        assert result['esc_us']==1500
    assert result['phase']=='fault'


def test_short_geometry_loss_uses_bounded_cache_then_brakes():
    c=ParkingEntryController(settings());armed(c)
    for i in (31,32,33):
        s=sample(i);s['parking']={'slots':[]}
        result=c.step(s,round(i*.1,3))
        assert result['esc_us']==1575 and result['cached_geometry']
        assert result['geometry_age_s']<=.4
    s=sample(35);s['parking']={'slots':[]}
    result=c.step(s,3.5)
    assert result['phase']=='safety_brake' and result['esc_us']==1400


@pytest.mark.parametrize('mutation',['repeat_id','repeat_capture','stale','wrong_size'])
def test_duplicate_stale_or_incompatible_frame_does_not_keep_driving(mutation):
    c=ParkingEntryController(settings());armed(c)
    s=sample(31)
    if mutation=='repeat_id':s['frame_id']=30
    if mutation=='repeat_capture':s['captured_s']=3.
    if mutation=='stale':s['captured_s']=2.
    if mutation=='wrong_size':s['image_size']=[640,480]
    result=c.step(s,3.1)
    assert result['phase']=='safety_brake' and result['esc_us']==1400
    assert c.step(s,3.3)['esc_us']==1500


def test_control_gap_returns_neutral_without_starting_another_brake_pulse():
    c=ParkingEntryController(settings());armed(c)
    result=c.step(sample(40),4.)
    assert result['phase']=='fault' and result['esc_us']==1500


def test_far_edge_does_not_authorize_completion_when_vehicle_envelope_is_outside():
    c=ParkingEntryController(settings());armed(c,q=[[260,345],[440,345],[400,220],[280,220]])
    q=[[260,345],[440,345],[400,235],[280,235]]
    result=tick(c,31,q=q)
    assert result['phase']=='safety_brake'
    assert not result['geometry']['lateral_envelope_estimate_inside']
    assert not result['done']


@pytest.mark.parametrize('q',[
    [[100,330],[380,180],[320,330],[160,180]],
    [[100,330],[380,330],[320,180],[160,100]],
    [[100,330],[380,330],[320,180],[float('nan'),180]],
])
def test_ambiguous_geometry_does_not_start_forward(q):
    c=ParkingEntryController(settings())
    result=armed(c,q=q)
    assert result['esc_us']==1500 and result['phase']=='wait_geometry'


def test_historical_homography_is_only_an_image_centerline_not_pose_verification():
    ground={'image_size':[480,360], 'homography':[[.01,0,-3.],[0,-.01,4.],[0,0,1.]],'verified':True}
    c=ParkingEntryController(settings(),ground=ground)
    result=armed(c,q=RIGHT)
    assert result['centerline_source']=='historical_ground_image_reference'
    assert all(v['center_px']==300 for v in result['geometry']['envelope_checks'])
    assert result['metric_pose'] is None and result['actual_pose_not_verified']
    assert not result['field_verified']


def test_all_entry_settle_and_fault_outputs_match_the_real_pulse_guard():
    c=ParkingEntryController(settings());writes=[]
    guard=ConePulseGuard(c.s,lambda *item:writes.append(item),mode='F/B')
    for i in range(65):
        q=RIGHT
        if 31<=i<=36:
            a=(i-30)/6
            q=[[x*(1-a)+xx*a,y*(1-a)+yy*a] for (x,y),(xx,yy) in zip(RIGHT,ALIGNED)]
        elif 37<=i<=40:
            a=(i-36)/4
            q=[[x*(1-a)+xx*a,y*(1-a)+yy*a] for (x,y),(xx,yy) in zip(ALIGNED,STOP)]
        elif i>40:q=STOP
        result=tick(c,i,q=q,moving=i<42)
        if result['action']=='neutral':assert result['steering_us']==1610
        now=round(i*.1,3)
        guard.accept({**result,'sent_s':now,'sequence':i},now)
    assert not guard.fault and result['done']
    assert writes[-2:]==[(4,1500),(3,1610)]


def guide_sample(i,far_y,*,dx=0,duplicate=False):
    s=sample(i)
    guide={'corners_px':[[90+dx,355],[390+dx,355],[325+dx,far_y],[155+dx,far_y]],
           'near_boundary_observed':False,'far_boundary_observed':True}
    s['parking']={'slots':[], 'entry_guides':[guide,copy.deepcopy(guide)] if duplicate else [guide]}
    return s


def test_observed_quad_then_entrance_leaves_view_guides_continue_until_actual_far_row():
    c=ParkingEntryController(settings());armed(c)
    for i,y in enumerate([187,195,203,211,219,227],31):
        result=c.step(guide_sample(i,y),round(i*.1,3))
        assert result['phase']=='approach' and result['esc_us']>1500
        assert result['geometry_source']=='tracked_far_guide'
        assert not result['near_boundary_observed'] and result['far_boundary_fresh']
        assert not result['cached_geometry']
        assert result['geometry']['far_edge_y_normalized']==y/359
        assert not result['whole_vehicle_inside_verified']
    result=c.step(guide_sample(37,235),3.7)
    assert result['phase']=='brake_pulse' and result['esc_us']==1400
    assert not result['near_boundary_observed']


def test_roi_cut_guide_cannot_be_used_to_choose_side_for_the_first_time():
    c=ParkingEntryController(settings())
    for i in range(64):
        result=c.step(guide_sample(i,180),round(i*.1,3))
        assert result['esc_us']==1500
    assert result['phase']=='fault'


@pytest.mark.parametrize('kind',['adjacent','entrance','ambiguous','missing_far_flag'])
def test_unmatched_or_ambiguous_guide_cannot_refresh_old_far_edge(kind):
    c=ParkingEntryController(settings());armed(c)
    for i in range(31,37):
        s=guide_sample(i,330 if kind=='entrance' else 190,dx=80 if kind=='adjacent' else 0,
                       duplicate=kind=='ambiguous')
        if kind=='missing_far_flag':s['parking']['entry_guides'][0].pop('far_boundary_observed')
        result=c.step(s,round(i*.1,3))
        if result['phase']=='approach':
            assert result['cached_geometry'] and not result['far_boundary_fresh']
            assert result['geometry']['far_edge_y_normalized']==180/359
        else:assert result['phase'] in ('safety_brake','fault')
    assert result['phase'] in ('safety_brake','fault')
    assert not result['done']


def test_guide_chain_cannot_refresh_itself_after_camera_gap_or_move_far_edge_backward():
    c=ParkingEntryController(settings());armed(c)
    good=c.step(guide_sample(31,190),3.1)
    assert good['geometry_source']=='tracked_far_guide'
    for i in range(32,38):
        result=c.step(guide_sample(i,160),round(i*.1,3))
    assert result['phase'] in ('safety_brake','fault')


def candidate_sample(i,*,q=ALIGNED,hint='right',side='right',clear=True,only=True):
    s=sample(i)
    s['parking']={'slots':[], 'direction_hint':{'suggested_slot':hint},
                  'entry_candidates':[{'id':side,'corners_px':copy.deepcopy(q),
                                       'candidate_only':only,'availability':'clear' if clear else 'unknown'}]}
    return s


def test_visible_candidate_initializes_only_as_unverified_actual_geometry():
    c=ParkingEntryController(settings())
    for i in range(31):result=c.step(candidate_sample(i),round(i*.1,3))
    assert result['esc_us']>1500 and result['geometry_source']=='visible_candidate'
    assert result['near_boundary_observed'] and not result['field_verified']
    result=c.step(guide_sample(31,190),3.1)
    assert result['esc_us']>1500 and result['geometry_source']=='tracked_far_guide'


@pytest.mark.parametrize('kwargs',[{'hint':None},{'hint':'left'},{'side':'left'},
                                  {'clear':False},{'only':False},{'q':[[0,0]]}])
def test_candidate_without_consistent_visible_evidence_cannot_initialize(kwargs):
    c=ParkingEntryController(settings())
    for i in range(31):result=c.step(candidate_sample(i,**kwargs),round(i*.1,3))
    assert result['esc_us']==1500 and result['phase']=='wait_geometry'


def test_candidate_does_not_override_blue_board_or_multiple_candidate_ambiguity():
    for blocked in (True,False):
        c=ParkingEntryController(settings())
        for i in range(31):
            s=candidate_sample(i)
            if blocked:s['parking']['direction_hint']['blocked_side']='right'
            else:s['parking']['entry_candidates']*=2
            result=c.step(s,round(i*.1,3))
        assert result['esc_us']==1500
        assert result['phase']=='fault' if blocked else result['phase']=='wait_geometry'


@pytest.mark.parametrize('kwargs',[{'forward_us':1650},{'selected_side':'unknown'},
                                  {'steering_gain':float('nan')},{'half_width_m':.3},
                                  {'ground':{'image_size':[640,480],'homography':[[1,0,0],[0,1,0],[0,0,1]]}}])
def test_invalid_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):ParkingEntryController(settings(),**kwargs)
