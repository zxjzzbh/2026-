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
    assert 26<abs(result['steering_us']-1610)<=60
    assert 1350<=result['steering_us']<=1870
    assert result['action']=='search'
    assert not result['field_verified'] and result['actual_pose_not_verified']
    assert result['metric_pose'] is None and result['distance_estimate_m'] is None


def test_parking_can_reach_selected_turn_within_half_a_second_without_wider_endpoints():
    c=ParkingEntryController(settings(),'right');result=armed(c,q=RIGHT)
    for i in range(31,35):result=tick(c,i,q=RIGHT)
    assert result['steering_us']==1350
    assert result['esc_us']==1575 and result['guidance_phase']=='turn_in'


@pytest.mark.parametrize('mirror',[-1,1])
def test_parking_corrects_heading_even_when_single_lookahead_center_is_aligned(mirror):
    q=[[100+60*mirror,330],[380+60*mirror,330],
       [320-40*mirror,180],[160-40*mirror,180]]
    side='left' if mirror==1 else 'right'
    c=ParkingEntryController(settings(),side);result=armed(c,side=side,q=q)
    assert abs(result['geometry']['lateral_error'])<1e-6
    assert abs(result['geometry']['heading_error'])>.1
    assert mirror*(result['steering_us']-1610)>0
    assert result['guidance_phase']=='align' and not result['done']


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
    assert result['brake_trigger']['distance_m'] is None
    assert not result['brake_trigger']['measured']
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
def test_observed_target_blockage_retargets_without_neutral_or_rearming(side):
    c=ParkingEntryController(settings(),side);armed(c,side=side)
    result=tick(c,31,side=side,blocked=True)
    assert result['phase']=='seek' and result['esc_us']==1550
    other='right' if side=='left' else 'left'
    assert result['selected_side']==other and result['retarget_count']==1
    assert result['target_change']['from']==side
    assert c.last_far_edge is None and c.last_poly is None
    result=tick(c,33,side=other)
    assert result['phase']=='approach' and result['esc_us']==1550


def test_screen_half_hint_cannot_relabel_an_acquired_bay_when_camera_turns():
    c=ParkingEntryController(settings());armed(c)
    s=sample(31);s['parking']={'slots':[],'direction_hint':{'blocked_side':'right'}}
    result=c.step(s,3.1)
    assert result['phase']=='seek' and result['esc_us']==1550
    assert result['selected_side']=='right' and result['retarget_count']==0


def test_no_current_target_has_a_bounded_search_then_latches_stop():
    c=ParkingEntryController(settings());results=[]
    for i in range(200):
        s=sample(i);s['parking']={'slots':[],'selected_slot':'right','tentative':True}
        result=c.step(s,round(i*.1,3));results.append(result)
        assert result['esc_us']==1500 if i<30 else result['esc_us']<=1550
        if i>=30:
            assert result['geometry']['far_edge_y_normalized'] is None
            assert not result['far_boundary_fresh'] and not result['parking_zone_verified']
        if i>=46:assert result['steering_us']==1610
    assert all(r['esc_us']==1550 for r in results[30:80])
    assert any(r['esc_us']==1400 for r in results[80:83])
    assert all(r['esc_us']==1500 for r in results[83:])
    assert result['phase']=='fault' and not result['active']


def test_short_geometry_loss_holds_heading_then_continues_seek_instead_of_braking():
    c=ParkingEntryController(settings());armed(c)
    for i in (31,32,33):
        s=sample(i);s['parking']={'slots':[]}
        result=c.step(s,round(i*.1,3))
        assert result['esc_us']==1550 and result['cached_geometry']
        assert result['geometry_age_s']<=.4
    s=sample(35);s['parking']={'slots':[]}
    result=c.step(s,3.5)
    assert result['phase']=='seek' and result['esc_us']==1550
    assert not result['cached_geometry'] and not result['far_boundary_fresh']
    assert result['geometry']['far_edge_y_normalized'] is None


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
def test_ambiguous_geometry_does_not_get_treated_as_a_far_parking_line(q):
    c=ParkingEntryController(settings())
    result=armed(c,q=q)
    assert result['esc_us']==1550 and result['phase']=='seek'
    assert result['geometry']['far_edge_y_normalized'] is None


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
        assert result['esc_us']==(1500 if i<30 else 1550)
    assert result['phase']=='seek' and not result['far_boundary_fresh']
    assert result['geometry']['far_edge_y_normalized'] is None


@pytest.mark.parametrize('kind',['adjacent','entrance','ambiguous','missing_far_flag'])
def test_unmatched_or_ambiguous_guide_cannot_refresh_old_far_edge(kind):
    c=ParkingEntryController(settings());armed(c)
    for i in range(31,37):
        s=guide_sample(i,330 if kind=='entrance' else 190,dx=80 if kind=='adjacent' else 0,
                       duplicate=kind=='ambiguous')
        if kind=='missing_far_flag':s['parking']['entry_guides'][0].pop('far_boundary_observed')
        result=c.step(s,round(i*.1,3))
        assert result['phase']==('seek' if i<36 else 'safety_brake')
        assert result['esc_us']==(1550 if i<36 else 1400)
        assert not result['far_boundary_fresh']
        assert result['geometry']['far_edge_y_normalized'] is None
    assert not result['done']


def test_guide_chain_cannot_refresh_itself_after_camera_gap_or_move_far_edge_backward():
    c=ParkingEntryController(settings());armed(c)
    good=c.step(guide_sample(31,190),3.1)
    assert good['geometry_source']=='tracked_far_guide'
    for i in range(32,38):
        result=c.step(guide_sample(i,160),round(i*.1,3))
    assert result['phase']=='safety_brake' and result['esc_us']==1400
    assert result['geometry']['far_edge_y_normalized'] is None


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
    assert result['esc_us']==1550 and result['phase']=='seek'
    assert result['geometry']['far_edge_y_normalized'] is None


def test_candidate_does_not_override_blue_board_or_multiple_candidate_ambiguity():
    for blocked in (True,False):
        c=ParkingEntryController(settings())
        for i in range(31):
            s=candidate_sample(i)
            if blocked:s['parking']['direction_hint']['blocked_side']='right'
            else:s['parking']['entry_candidates']*=2
            result=c.step(s,round(i*.1,3))
        assert result['esc_us']==1550 and result['phase']=='seek'
        assert result['geometry']['far_edge_y_normalized'] is None


def test_seek_for_seven_seconds_then_geometry_and_far_line_recover_without_rearming():
    c=ParkingEntryController(settings());writes=[]
    guard=ConePulseGuard(c.s,lambda *p:writes.append(p),mode='F/B')
    results=[]
    for i in range(92):
        s=sample(i,q=NEAR if i==70 else STOP,moving=i<73)
        if i<70:s['parking']={'slots':[]}
        result=c.step(s,round(i*.1,3));results.append(result)
        guard.accept({**result,'sent_s':round(i*.1,3),'sequence':i},round(i*.1,3))
    assert all(r['esc_us']==1550 for r in results[30:71])
    assert results[70]['phase']=='approach' and results[70]['far_boundary_fresh']
    assert results[71]['phase']=='brake_pulse' and results[71]['esc_us']==1400
    assert results[-1]['done'] and not guard.fault
    assert not results[-1]['whole_vehicle_inside_verified']


def test_acquired_target_loss_brakes_instead_of_returning_to_unbounded_seek():
    c=ParkingEntryController(settings());armed(c,q=LEFT)
    for i in range(31,46):
        s=sample(i);s['parking']={'slots':[]}
        result=c.step(s,round(i*.1,3))
        if i<36:assert result['esc_us']==1550 and result['phase']=='seek'
        else:assert result['esc_us']<=1500 and result['phase'] in ('safety_brake','fault')
        assert result['geometry']['far_edge_y_normalized'] is None
    result=tick(c,46,q=NEAR)
    assert result['phase']=='fault' and result['esc_us']==1500
    result=tick(c,47,q=STOP)
    assert result['phase']=='fault' and result['esc_us']==1500 and not result['done']


def test_short_missing_target_then_real_far_fragment_recovers_with_continuous_pwm():
    c=ParkingEntryController(settings());armed(c)
    s=sample(31);s['parking']={'slots':[]}
    assert c.step(s,3.1)['esc_us']==1550
    result=c.step(guide_sample(32,190),3.2)
    assert result['phase']=='approach' and result['esc_us']==1550
    assert result['geometry_source']=='tracked_far_guide'
    assert result['geometry']['far_edge_y_normalized']==190/359


def test_missing_frame_does_not_forget_which_old_edge_was_the_entrance():
    c=ParkingEntryController(settings())
    armed(c,q=[[150,220],[330,220],[320,180],[160,180]])
    for i in (31,32):
        s=sample(i);s['parking']={'slots':[]}
        assert c.step(s,round(i*.1,3))['esc_us']==1550
    result=c.step(guide_sample(33,220),3.3)
    assert result['phase']=='seek' and result['geometry']['far_edge_y_normalized'] is None


@pytest.mark.parametrize('side,board_side,box',[('left','right',[259,78,364,176]),
                                              ('right','left',[116,78,221,176])])
def test_visible_board_keeps_feedback_steering_past_the_old_short_turn_timer(side,board_side,box):
    c=ParkingEntryController(settings(),side)
    for i in range(50):
        s=sample(i,side=side)
        s['parking']={'slots':[], 'direction_hint':{'suggested_slot':side,'blocked_side':board_side,
                                                  'boards':[{'side':board_side,'bbox_px':box}]}}
        result=c.step(s,round(i*.1,3))
        if i>=30:
            assert result['geometry_source']=='blue_board_seek'
            assert result['guidance_target_px'] is not None
            assert result['geometry']['far_edge_y_normalized'] is None
        if i>=46:
            assert result['guidance_target_px'] is not None
            assert result['steering_us']>1675 if side=='left' else result['steering_us']<1545
            assert result['esc_us']==1550


def test_board_guided_seek_recentres_when_visible_target_is_aligned_and_never_fakes_stop_line():
    c=ParkingEntryController(settings(),'left')
    for i in range(50):
        s=sample(i);s['parking']={'slots':[], 'direction_hint':{'suggested_slot':'left','blocked_side':'right',
            'boards':[{'side':'right','bbox_px':[259,78,364,176]}]}}
        result=c.step(s,round(i*.1,3))
    assert result['steering_us']>1675
    for i in range(50,62):
        s['frame_id']=i;s['captured_s']=round(i*.1,3)
        s['parking']['direction_hint']['boards'][0]['bbox_px']=[290,78,390,176]
        result=c.step(s,round(i*.1,3))
    assert result['steering_us']==1610 and result['esc_us']==1550
    assert result['geometry']['far_edge_y_normalized'] is None and not result['done']


def test_seek_still_stops_for_two_blocked_bays_and_camera_fault():
    for blocked in (True,False):
        c=ParkingEntryController(settings())
        for i in range(31):
            s=sample(i);s['parking']={'slots':[]}
            c.step(s,round(i*.1,3))
        s=sample(31);s['parking']={'slots':[]}
        if blocked:s['parking']['slots']=[{'id':side,'availability':'blocked'} for side in ('left','right')]
        else:s['fresh']=False
        result=c.step(s,3.1)
        assert result['phase']=='safety_brake' and result['esc_us']==1400
        assert c.step(sample(33),3.3)['phase']=='fault'


def test_real_overshoot_trace_now_brakes_after_far_target_loss_and_cannot_resume():
    trace=json.loads((ROOT/'vision/tests/fixtures/parking-overshoot-loss-20261013.json').read_text('utf-8'))
    c=ParkingEntryController(settings(),trace['selected_side'],forward_us=trace['forward_us'],steering_gain=1.25)
    braking=[];after=[]
    for row in trace['rows']:
        result=c.step(row['sample'],row['now_s'])
        if result['esc_us']<1500:braking.append(row['now_s'])
        if braking:after.append(result)
    assert braking and 5.8<braking[0]<6.4
    assert all(r['esc_us']<=1500 for r in after)
    assert after[-1]['phase']=='fault' and not after[-1]['done']
    assert all(r['old_esc_us']>1500 for r in trace['rows'] if 6.5<r['now_s']<10.)


def test_increased_gain_is_proportional_and_keeps_existing_endpoints():
    old=ParkingEntryController(settings(),steering_gain=1.)
    new=ParkingEntryController(settings(),steering_gain=1.25)
    q=[[x+25,y] for x,y in ALIGNED]
    for i in range(45):a=tick(old,i,q=q);b=tick(new,i,q=q)
    assert abs(b['steering_us']-1610)>abs(a['steering_us']-1610)
    assert abs(abs(b['steering_us']-1610)-1.25*abs(a['steering_us']-1610))<=1.
    assert 1350<=b['steering_us']<=1870


@pytest.mark.parametrize('side',['left','right'])
def test_target_crossing_camera_centre_does_not_cause_large_opposite_steering(side):
    c=ParkingEntryController(settings(),side,steering_gain=1.25)
    outputs=[]
    for i in range(80):
        shift=-60+min(40,max(0,i-30))*3
        if side=='right':shift=-shift
        q=[[x+shift,y] for x,y in ALIGNED]
        result=tick(c,i,side=side,q=q)
        if i>=30:outputs.append(result)
    assert all(r['esc_us'] in (1550,1575) for r in outputs)
    if side=='left':assert min(r['steering_us'] for r in outputs)>=1530
    else:assert max(r['steering_us'] for r in outputs)<=1690
    assert any(r['countersteer_limited'] for r in outputs)


def test_initial_board_cue_retargets_once_and_image_half_flicker_does_not_toggle():
    c=ParkingEntryController(settings(),'right')
    for i in range(31):
        s=sample(i);s['parking']={'slots':[]}
        c.step(s,round(i*.1,3))
    for i in range(31,46):
        s=sample(i);s['parking']={'slots':[],'direction_hint':{
            'blocked_side':'right' if i%2 else 'left','suggested_slot':'left' if i%2 else 'right'}}
        r=c.step(s,round(i*.1,3))
        assert r['selected_side']=='left' and r['retarget_count']==1
        assert r['esc_us']==1550 and r['phase']=='seek'
    assert c.last_far_at is None


def test_actual_board_overlap_replans_but_unrelated_screen_side_board_does_not():
    for overlapping in (False,True):
        c=ParkingEntryController(settings(),'left');armed(c,side='left')
        box=[200,230,300,330] if overlapping else [10,230,70,330]
        s=sample(31,side='left');s['parking']={'slots':[],'direction_hint':{
            'blocked_side':'left','suggested_slot':'right','boards':[{'side':'left','bbox_px':box}]}}
        r=c.step(s,3.1)
        assert r['esc_us']==1550
        assert r['selected_side']==('right' if overlapping else 'left')
        assert r['retarget_count']==int(overlapping)


def test_retarget_cannot_reuse_old_half_far_line_or_old_loss_timer():
    c=ParkingEntryController(settings(),'right');armed(c)
    r=tick(c,31,blocked=True)
    assert r['selected_side']=='left' and r['retarget_count']==1
    for i in range(32,43):
        s=sample(i);s['parking']={'slots':[],'tracked_far_lines':[{
            'id':'right','source':'optical_flow_far_edge','frame_id':i,'captured_s':round(i*.1,3),
            'edge_px':[[160,235],[320,235]],'tracked_points':20,'marking_support':1.}]}
        r=c.step(s,round(i*.1,3))
        assert r['esc_us']==1550 and r['geometry']['far_edge_y_normalized'] is None
    assert not r['done'] and c.core is None


def road(left,right):
    return {'white_lines_detected':True,
            'left_inner':[[left+(240-y)*.2,y] for y in range(120,241,15)],
            'right_inner':[[right-(240-y)*.2,y] for y in range(120,241,15)]}


@pytest.mark.parametrize('side,left,right,correction',[('left',200,450,'right'),('right',20,290,'left')])
def test_white_edge_inward_correction_overrides_selected_side_limit(side,left,right,correction):
    c=ParkingEntryController(settings(),side);armed(c,side=side,q=LEFT if side=='left' else RIGHT)
    s=sample(31,side=side,q=LEFT if side=='left' else RIGHT);s['parking']['road_context']=road(left,right)
    r=c.step(s,3.1)
    assert r['esc_us']==1550 and r['white_boundary']['correction']==correction
    assert r['steering_us']<1610 if correction=='right' else r['steering_us']>1610


def test_persistent_body_margin_breach_brakes_instead_of_continuing_outside_white_line():
    c=ParkingEntryController(settings(),'left');armed(c,side='left',q=LEFT)
    for i in range(31,34):
        s=sample(i,side='left',q=LEFT);s['parking']['road_context']=road(210,460)
        r=c.step(s,round(i*.1,3))
    assert r['phase']=='safety_brake' and r['esc_us']==1400
    assert '白线' in r['reason'] and not r['done']


def test_aligned_target_unwinds_to_centre_gradually_without_opposite_swing():
    c=ParkingEntryController(settings(),'left');last=None
    for i in range(55):
        dx=-60 if i<35 else min(0,-60+(i-35)*5)
        q=[[x+dx,y] for x,y in ALIGNED]
        r=tick(c,i,side='left',q=q)
        if last is not None and i>=36:
            assert abs(r['steering_us']-last)<=61
        assert r['steering_us']>=1530
        last=r['steering_us']
    assert r['steering_us']==1610 and r['esc_us']==1550


@pytest.mark.parametrize('kwargs',[{'forward_us':1650},{'selected_side':'unknown'},
                                  {'steering_gain':float('nan')},{'half_width_m':.3},
                                  {'ground':{'image_size':[640,480],'homography':[[1,0,0],[0,1,0],[0,0,1]]}}])
def test_invalid_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):ParkingEntryController(settings(),**kwargs)
