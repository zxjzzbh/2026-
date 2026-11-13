"""Regression of the Nov 8 recorded failure; no hardware or networking."""
import copy
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from carvision.parking_bay import detect_parking
from carvision.parking_entry_run import ParkingEntryRun
from carvision.parking_visual import resolve_visual_options
from carvision.parking_log import ParkingFrames
from test_parking_entry_run import prepared, command, sample
from test_parking_entry import ParkingEntryController, settings, tick, RIGHT
from test_parking_boundary_guard import frame, road

DATA=Path(__file__).parent/'fixtures/parking-20261108'


@pytest.mark.parametrize('mirror',[False,True])
def test_actual_slanted_bay_and_border_board_remain_observed(mirror):
    image=cv2.imdecode(np.frombuffer((DATA/'post-run.jpg').read_bytes(),np.uint8),1)
    if mirror:image=cv2.flip(image,1)
    s=json.loads((DATA/'settings.json').read_text(encoding='utf8'))['parking']
    observation,_=detect_parking(image,s)
    side='right' if mirror else 'left'
    assert observation['direction_hint']['suggested_slot']==side
    candidates=observation['entry_candidates']
    assert len(candidates)==1 and candidates[0]['id']==side
    assert min(candidates[0]['edge_support'])>=.9
    assert candidates[0]['ground_fraction']>.9
    assert candidates[0]['blue_inside_fraction']<.005
    assert not observation['metric_geometry'] and not candidates[0]['occupancy_verified']


@pytest.mark.parametrize('forward',[1570,1575,1600])
@pytest.mark.parametrize('stored',[1501,1545,1550,1570,1600])
def test_all_legacy_slow_values_respect_measured_motor_floor(forward,stored):
    value=resolve_visual_options({'slow_forward_us':stored},settings(),forward)
    assert 1570<=value['slow_forward_us']<=forward


def test_actual_frame_timing_survives_all_three_original_expirations():
    records=json.loads((DATA/'frame-timing.json').read_text())
    origin=records[0]['captured_s']
    publications=[(r['captured_s']-origin+r['frame_age_ms']/1000,r) for r in records]
    run,actuator=prepared();sequence=0;index=0;current=sample(.1)
    for step in range(1,int(publications[-1][0]/.04)+1):
        now=.1+step*.04
        run.command('parking_trial_keepalive',command(run,sequence),now);sequence+=1
        while index<len(publications) and publications[index][0]+.1<=now:
            r=publications[index][1]
            current=sample(r['captured_s']-origin+.1);current['frame_id']=r['frame_id']
            index+=1
        run.tick(current,now)
        assert run.active and run.phase=='ready'
    assert all(c['esc_us']==1500 for c in actuator.commands)
    run.cancel('test finished')


def test_recorded_turn_keeps_current_target_through_white_dropout():
    stored=json.loads((DATA/'settings.json').read_text(encoding='utf8'))
    controller=ParkingEntryRun._controller(stored)
    rows=[json.loads(x) for x in (DATA/'decisions.jsonl').read_text(encoding='utf8').splitlines()]
    outputs=[]
    for row in rows:
        if row['intent']['phase']=='ready':continue
        out=controller.step(row['sample'],row['now_s']);outputs.append(out)
        assert out['phase']!='fault'
        assert out['esc_us']<=1500 or out['esc_us']>=1570
        assert abs(out['steering_us']-1610)<=160
        assert not out['whole_vehicle_inside_verified'] and not out['done']
    assert outputs[-1]['white_boundary']['support_source']=='observed_entry_transition'
    assert outputs[-1]['esc_us']==1570
    # This is recorded input replay, not a simulation of the changed trajectory.


def test_white_handoff_is_bounded_and_cannot_keep_driving_blind():
    c=ParkingEntryController(settings(),require_white_boundary=True)
    for i in range(36):c.step(frame(i,road()),i*.1)
    for i in range(36,43):
        out=c.step(frame(i,{}),i*.1)
        assert out['esc_us']==1570
        assert out['white_boundary']['support_source']=='observed_entry_transition'
    out=c.step(frame(44,{}),4.4)
    assert out['phase']=='boundary_wait' and out['esc_us']==1400
    for i in range(45,60):out=c.step(frame(i,{}),i*.1)
    assert out['phase']=='fault' and out['esc_us']==1500


def test_missing_target_is_not_covered_by_white_handoff():
    c=ParkingEntryController(settings(),require_white_boundary=True)
    for i in range(36):c.step(frame(i,road()),i*.1)
    s=frame(36,{});s['parking']['slots']=[]
    out=c.step(s,3.6)
    assert out['esc_us']==1400 and out['phase']=='boundary_wait'


def test_large_sample_interval_cannot_cause_a_steering_jump():
    c=ParkingEntryController(settings())
    for i in range(31):out=tick(c,i,q=RIGHT)
    before=out['steering_us'];s=sample(3.45);s['parking']=frame(34,road())['parking']
    out=c.step(s,3.45)
    assert out['phase']!='fault'
    assert abs(out['steering_us']-before)<=90


def test_frame_recording_indexes_the_exact_raw_bytes_and_caps_disk(tmp_path):
    recorder=ParkingFrames(tmp_path,max_bytes=6)
    recorder.add(b'abc',1,1.);recorder.add(b'def',2,1.1);recorder.add(b'ghi',3,1.2)
    recorder.close()
    assert (tmp_path/'frames.mjpg').read_bytes()==b'abcdef'
    rows=[json.loads(r) for r in (tmp_path/'frames.jsonl').read_text().splitlines()]
    assert [(r['frame_id'],r['offset'],r['length']) for r in rows]==[(1,0,3),(2,3,3)]
    status=json.loads((tmp_path/'recording.json').read_text())
    assert status['written']==2 and status['dropped']==1 and status['error'] is None
