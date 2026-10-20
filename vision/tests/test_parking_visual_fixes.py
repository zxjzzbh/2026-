"""Failure-driven visual parking regressions; fake observations/output only."""
import copy
import json
from pathlib import Path

import pytest

from carvision.parking_entry import ParkingEntryController
from carvision.parking_visual import validate_visual_options
from carvision.steering import from_offset
from carvision.cone_trial_output import ConePulseGuard
from test_parking_entry import settings, sample, road, ALIGNED, LEFT, RIGHT, STOP
from test_parking_entry_run import ParkingEntryRun, configurations, base, preparation, Actuator, command


def frame(i, *, side='left', quad=None, white=True, moving=True, empty=False):
    row=sample(i,side=side,q=ALIGNED if quad is None else quad,moving=moving)
    if empty:row['parking']['slots']=[]
    if white:row['parking']['road_context']=road(70,410)
    return row


def step(controller,i,**kwargs):
    return controller.step(frame(i,**kwargs),round(i*.1,3))


def moving_controller(**kwargs):
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True,**kwargs)
    for i in range(36):out=step(controller,i)
    assert out['esc_us']==1550
    return controller


def test_fresh_continuous_bay_covers_white_dropout_without_ending_session():
    controller=moving_controller()
    for i in range(36,45):
        out=step(controller,i,white=False)
        assert out['phase']=='approach' and out['esc_us']==1550
        assert out['white_boundary']['support_source']=='current_bay_envelope'
        assert out['white_boundary']['usable'] and not out['white_boundary']['observed']
        assert not out['whole_vehicle_inside_verified'] and not out['reference_transfer_verified']


def test_start_with_only_current_bay_waits_for_several_fresh_observations():
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True)
    for i in range(30):step(controller,i,white=False,moving=False)
    first=step(controller,30,white=False,moving=False)
    second=step(controller,31,white=False,moving=False)
    confirmed=step(controller,32,white=False,moving=False)
    assert first['phase']==second['phase']=='boundary_wait'
    assert first['esc_us']==second['esc_us']==1500
    assert confirmed['esc_us']==1550 and confirmed['phase']=='approach'


def test_short_white_grace_requires_current_bay_not_a_cached_target():
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True)
    for i in range(31):step(controller,i)
    blink=step(controller,31,white=False)
    assert blink['esc_us']==1550
    assert blink['white_boundary']['support_source']=='recent_white_and_current_bay'
    lost=step(controller,32,white=False,empty=True)
    assert lost['phase']=='boundary_wait' and lost['esc_us']==1400


def test_all_evidence_loss_pauses_with_finite_brake_then_times_out_and_latches():
    controller=moving_controller();outputs=[];writes=[]
    guard=ConePulseGuard(controller.s,lambda *values:writes.append(values),mode='F/B')
    # Begin the guard at the last known forward output, then replay the loss.
    guard.accept({**step(controller,36),'sent_s':3.6,'sequence':0},3.6)
    for i in range(37,65):
        out=step(controller,i,white=False,empty=True);outputs.append(out)
        guard.accept({**out,'sent_s':i*.1,'sequence':i-36},i*.1)
    assert outputs[0]['phase']=='boundary_wait' and outputs[0]['esc_us']==1400
    assert sum(out['esc_us']==1400 for out in outputs)<=2
    assert all(out['esc_us']<=1500 for out in outputs)
    assert outputs[-1]['phase']=='fault' and not outputs[-1]['done']
    assert not guard.fault


def test_boundary_recovery_waits_for_brake_release_and_stopped_feedback():
    controller=moving_controller()
    assert step(controller,36,white=False,empty=True)['esc_us']==1400
    assert step(controller,37,moving=True)['esc_us']<=1500
    assert step(controller,38,moving=True)['esc_us']==1500
    resumed=step(controller,39,moving=False)
    assert resumed['esc_us']==1550 and resumed['phase']=='approach'
    assert resumed['steering_us']==1610


def test_current_single_dangerous_edge_vetoes_bay_fallback():
    controller=moving_controller()
    row=frame(36,white=False)
    row['parking']['road_context']={'white_lines_detected':False,
        'left_inner':[[230.,float(y)] for y in range(120,241,10)]}
    out=controller.step(row,3.6)
    assert out['phase']=='boundary_wait' and out['esc_us']==1400


def test_single_dangerous_edge_also_vetoes_bay_before_any_full_white_history():
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True)
    for i in range(45):
        row=frame(i,white=False,moving=False)
        row['parking']['road_context']={'white_lines_detected':False,
            'left_inner':[[230.,float(y)] for y in range(120,241,10)]}
        out=controller.step(row,round(i*.1,3))
        assert out['esc_us']==1500
    assert out['phase']=='fault'


def test_unknown_occupancy_never_substitutes_for_missing_white_boundaries():
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True)
    for i in range(45):
        row=frame(i,white=False,moving=False)
        row['parking']['slots'][0]['availability']='unknown'
        out=controller.step(row,round(i*.1,3))
        assert out['esc_us']==1500
    assert out['phase']=='fault'


@pytest.mark.parametrize('side,initial,shift,direction',[
    ('left',LEFT,25,-1),('right',RIGHT,-25,1),
])
def test_alignment_can_correct_opposite_direction_before_already_aligned(side,initial,shift,direction):
    controller=ParkingEntryController(settings(),side,require_white_boundary=True)
    previous=1610
    for i in range(90):
        fraction=max(0.,min(1.,(i-35)/20))
        target=[[x+shift,y] for x,y in ALIGNED]
        quad=[[a+(b-a)*fraction for a,b in zip(start,end)] for start,end in zip(initial,target)]
        out=step(controller,i,side=side,quad=quad)
        assert abs(out['steering_us']-previous)<=61
        previous=out['steering_us']
    assert not out['geometry']['image_aligned']
    assert direction*(out['steering_us']-1610)>30
    assert out['steering_us']==from_offset(controller.pd.output)
    assert 1530<=out['steering_us']<=1690 and out['esc_us']==1550


def test_seek_and_total_deadlines_survive_new_geometry_and_replanning():
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True)
    outputs=[step(controller,i,empty=True) for i in range(101)]
    assert all(o['esc_us']==1550 for o in outputs[30:80])
    assert outputs[80]['esc_us']==1400
    assert all(o['esc_us']==1500 for o in outputs[83:])
    assert '寻位超时' in outputs[-1]['reason']
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True,
        visual_options={'max_run_s':5.,'seek_timeout_s':2.})
    for i in range(101):out=step(controller,i)
    assert out['phase']=='fault' and out['esc_us']==1500 and '总时限' in out['reason']


def test_slowdown_precedes_stop_row_and_matches_frozen_actuator_pulses():
    controller=ParkingEntryController(settings(),'right',require_white_boundary=True,
        visual_options={'slow_line_y_normalized':.5,'stop_line_y_normalized':.7})
    writes=[];guard=ConePulseGuard(controller.s,lambda *values:writes.append(values),mode='F/B')
    for i in range(33):
        y=160 if i<=30 else 175 if i==31 else 190
        q=[[260,330],[440,330],[400,y],[280,y]]
        out=step(controller,i,side='right',quad=q)
        guard.accept({**out,'sent_s':i*.1,'sequence':i},i*.1)
        if i==30:assert out['esc_us']==1575 and out['action']=='search'
    assert out['esc_us']==1550 and out['action']=='creep'
    assert not guard.fault and (4,1550) in writes
    assert controller.s['pwm']['search_us']==1575 and controller.s['pwm']['creep_us']==1550


def test_custom_parking_stop_row_does_not_rewrite_or_claim_legacy_distances():
    brake=settings();original=copy.deepcopy(brake)
    controller=ParkingEntryController(brake,'left',require_white_boundary=True,
        visual_options={'slow_line_y_normalized':.6,'stop_line_y_normalized':.75})
    for i in range(31):out=step(controller,i)
    for i,y in enumerate((195,210,225,235,250,265,270),31):
        q=[[100,345],[380,345],[330,y],[150,y]]
        out=step(controller,i,quad=q)
        if y==235:assert out['esc_us']==1550
    assert out['phase']=='brake_pulse' and out['esc_us']==1400
    assert out['brake_trigger']['far_edge_y_normalized']==.75
    assert out['brake_trigger']['distance_m'] is None and not out['brake_trigger']['measured']
    assert out['distance_estimate_m'] is None and not out['reference_transfer_verified']
    assert brake==original


@pytest.mark.parametrize('options',[
    {'slow_forward_us':1500},{'slow_forward_us':True},{'slow_forward_us':1600.5},
    {'seek_timeout_s':float('nan')},{'seek_timeout_s':11},
    {'max_run_s':4},{'boundary_grace_s':.5},{'boundary_wait_s':.1},
    {'countersteer_limit_us':260},{'countersteer_limit_us':True},
    {'stop_line_y_normalized':.7},{'slow_line_y_normalized':.7,'stop_line_y_normalized':.6},
    {'slow_line_y_normalized':.65,'stop_line_y_normalized':.66},
    {'slow_line_y_normalized':True,'stop_line_y_normalized':.7}, {'unreviewed':1},
])
def test_invalid_visual_settings_are_rejected_before_hardware(options):
    with pytest.raises(ValueError):validate_visual_options(options)
    parking,brake,cone=configurations();parking['visual_entry']=options
    run=ParkingEntryRun(lambda _:pytest.fail('invalid settings must not open hardware'))
    with pytest.raises(ValueError):run.prepare(parking,brake,cone,base(),preparation(),0.)
    assert not run.active


def test_visual_slow_pulse_is_frozen_before_actuator_is_created():
    parking,brake,cone=configurations();parking['visual_entry']={'slow_forward_us':1545}
    opened=[];actuator=Actuator()
    def factory(s):opened.append(copy.deepcopy(s));return actuator
    run=ParkingEntryRun(factory)
    run.prepare(parking,brake,cone,base(),preparation(),0.)
    assert not opened
    run.tick(frame(1,side='right',moving=False),.1)
    assert opened[0]['pwm']['creep_us']==1545
    parking['visual_entry']['slow_forward_us']=1600
    assert run.s['brake']['pwm']['creep_us']==1545
    run.cancel('test ended')


def test_real_white_loss_failures_pause_instead_of_ending_on_the_first_missing_frame():
    fixture=json.loads((Path(__file__).parent/'fixtures/parking-boundary-failures-20261020.json').read_text(encoding='utf-8'))
    assert len(fixture['cases'])==3
    for case in fixture['cases']:
        controller=ParkingEntryRun._controller(case['settings'])
        for row in case['rows']:
            out=controller.step(row['sample'],row['now_s'])
            assert out['phase']!='fault'
        assert out['phase']=='boundary_wait' and out['esc_us']<=1500
        assert not out['done'] and not out['whole_vehicle_inside_verified']
