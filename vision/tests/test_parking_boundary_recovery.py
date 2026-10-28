"""Real failure replay and recovery guards; no network or physical output."""
import json
from pathlib import Path

import pytest

from carvision.parking_entry import ParkingEntryController
from carvision.parking_entry_run import ParkingEntryRun
from carvision.cone_trial_output import ConePulseGuard
from test_parking_entry import settings, ALIGNED
from test_parking_visual_fixes import frame, step, moving_controller


@pytest.mark.parametrize('mirrored',[False,True])
def test_contained_but_not_aligned_bay_can_supply_boundaries(mirrored):
    quad=[[140,330],[420,330],[280,180],[120,180]]
    if mirrored:quad=[[480-x,y] for x,y in (quad[1],quad[0],quad[3],quad[2])]
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True)
    for i in range(50):
        out=step(controller,i,quad=quad,white=False,moving=False)
        if i<32:assert out['esc_us']==1500
        else:
            assert out['esc_us']==1550 and out['phase']=='approach'
            assert abs(out['geometry']['heading_error'])>.35
            assert out['geometry']['lateral_envelope_estimate_inside']
            assert out['white_boundary']['support_source']=='current_bay_envelope'
            assert out['guidance_phase']=='align' and not out['done']


def test_bay_boundary_margin_hysteresis_never_authorizes_an_outside_envelope():
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True)
    for i in range(36):step(controller,i,white=False,moving=False)
    for i,shift in ((36,20),(37,40)):
        out=step(controller,i,white=False,quad=[[x+shift,y] for x,y in ALIGNED])
        assert out['esc_us']==1550
    assert 0<out['white_boundary']['bay_clearance_fraction']<.025
    outside=step(controller,38,white=False,quad=[[x+45,y] for x,y in ALIGNED])
    assert outside['phase']=='boundary_wait' and outside['esc_us']==1400
    assert outside['boundary_wait_cause']=='bay_envelope_outside'
    # The same narrow margin is insufficient to acquire new bay-only support.
    new=ParkingEntryController(settings(),'left',require_white_boundary=True)
    for i in range(45):
        out=step(new,i,white=False,moving=False,quad=[[x+40,y] for x,y in ALIGNED])
        assert out['esc_us']==1500
    assert out['phase']=='fault' and out['boundary_wait_cause']=='bay_margin_pending'


def test_recovery_requires_continuous_stopped_frames_and_resets_on_motion_or_unknown():
    controller=moving_controller()
    assert step(controller,36,white=False,empty=True)['esc_us']==1400
    for i,valid,moving in ((37,True,False),(38,True,False),(39,True,False),
                            (40,True,True),(41,True,False),(42,False,False),
                            (43,True,False),(44,True,False)):
        row=frame(i,moving=moving);row['motion']['valid']=valid
        out=controller.step(row,round(i*.1,3))
        assert out['esc_us']<=1500
    resumed=step(controller,45,moving=False)
    assert resumed['phase']=='approach' and resumed['esc_us']==1550
    assert resumed['boundary_wait_cause'] is None


@pytest.mark.parametrize('valid,moving,cause,text',[
    (False,False,'stop_feedback_invalid','缺少有效停稳反馈'),
    (True,True,'still_moving','仍检测到移动'),
    (True,None,'stop_feedback_invalid','缺少有效停稳反馈'),
])
def test_timeout_names_the_stop_feedback_problem_after_boundaries_recover(valid,moving,cause,text):
    controller=moving_controller()
    step(controller,36,white=False,empty=True)
    for i in range(37,51):
        row=frame(i,moving=moving);row['motion']['valid']=valid
        out=controller.step(row,round(i*.1,3))
        assert out['esc_us']<=1500
    assert out['phase']=='fault' and out['boundary_wait_cause']==cause
    assert text in out['reason'] and '停车边界持续不足' not in out['reason']
    assert out['white_boundary']['evidence_usable']


def test_boundary_timeout_does_not_misreport_missing_stop_feedback_when_already_stopped():
    controller=moving_controller()
    for i in range(36,51):out=step(controller,i,white=False,empty=True,moving=False)
    assert out['phase']=='fault' and '缺少当前有效车位和白线边界' in out['reason']
    assert out['boundary_wait_cause']=='boundary_evidence_missing'
    assert '停稳反馈' not in out['reason']


def test_bay_only_support_does_not_bypass_the_final_misaligned_stop_guard():
    controller=ParkingEntryController(settings(),'left',require_white_boundary=True)
    quad=[[140,345],[420,345],[280,236],[120,236]]
    for i in range(36):out=step(controller,i,quad=quad,white=False,moving=False)
    assert out['phase']=='fault' and out['esc_us']==1500
    assert '远端停车参考' in out['reason'] and not out['done']


def test_real_white_dropout_and_heading_jitter_recovers_once_without_false_completion():
    fixture=json.loads((Path(__file__).parent/'fixtures/parking-boundary-recovery-20261020.json').read_text(encoding='utf-8'))
    controller=ParkingEntryRun._controller(fixture['settings'])
    guard=ConePulseGuard(controller.s,lambda *args:None,mode='F/B')
    outputs=[]
    for seq,row in enumerate(fixture['rows']):
        out=controller.step(row['sample'],row['now_s']);outputs.append(out)
        guard.accept({**out,'sent_s':row['now_s'],'sequence':seq},row['now_s'])
        assert out['phase']!='fault' and not out['done']
        assert not out['whole_vehicle_inside_verified']
    assert not guard.fault
    first_pause=next(i for i,out in enumerate(outputs) if out['phase']=='boundary_wait')
    recovered=next(i for i in range(first_pause+1,len(outputs)) if outputs[i]['esc_us']>1500)
    assert all(out['esc_us']<=1500 for out in outputs[first_pause:recovered])
    assert all(out['phase']=='approach' and out['esc_us']==1550 for out in outputs[recovered:])
    assert sum(out['esc_us']==1400 for out in outputs)<=2
    assert outputs[-1]['white_boundary']['support_source']=='current_bay_envelope'
    assert outputs[-1]['geometry']['heading_error']>.35
    assert outputs[-1]['logic_revision']=='parking_closed_loop_v3'
