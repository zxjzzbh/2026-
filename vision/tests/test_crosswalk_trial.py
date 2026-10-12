import json
from pathlib import Path

import pytest

from carvision.crosswalk_trial import ParkingTrialLogic, CrosswalkTrialDrive, load_reference


def reference():
    return {'schema_version':1, 'measured':True, 'camera':'secondary', 'image_size':[480,360],
            'front_distance_m':.25, 'far_edge_y_normalized':.75}


def sample(**changes):
    return {'fresh':True, 'image_size':[480,360], 'presence':'present',
            'far_edge_y_normalized':.65, 'lane_valid':True,'lane_offset':0, **changes}


def test_measured_reference_required(tmp_path):
    p=tmp_path/'reference.json'
    p.write_text(json.dumps(reference()|{'measured':False}))
    with pytest.raises(ValueError):load_reference(p)
    p.write_text(json.dumps(reference()))
    assert load_reference(p)['front_distance_m']==.25


def test_early_neutral_reference_is_distinct_from_final_parking_target(tmp_path):
    p=tmp_path/'reference.json'
    d=reference()|{'schema_version':2,'trigger_distance_m':.65,'parking_target_distance_m':.15}
    d.pop('front_distance_m')
    p.write_text(json.dumps(d))
    loaded=load_reference(p)
    assert loaded['trigger_distance_m']==.65 and loaded['parking_target_distance_m']==.15
    c=ParkingTrialLogic(loaded,mode='straight');c.start(0)
    assert c.step(sample(far_edge_y_normalized=.7),.1)['motor']=='forward'
    assert c.step(sample(far_edge_y_normalized=.76),.2)['motor']=='stop'


@pytest.mark.parametrize('bad',[{'parking_target_distance_m':.4}, {'trigger_distance_m':.1},
                               {'trigger_distance_m':float('nan')}, {'trigger_distance_m':2}])
def test_unmeasured_or_inconsistent_compensation_is_rejected(tmp_path,bad):
    p=tmp_path/'reference.json'
    d=reference()|{'schema_version':2,'trigger_distance_m':.65,'parking_target_distance_m':.15}|bad
    d.pop('front_distance_m');p.write_text(json.dumps(d))
    with pytest.raises(ValueError):load_reference(p)


def test_trial_searches_before_detection_and_lane_mode_still_requires_lane():
    c=ParkingTrialLogic(reference());c.start(0)
    assert c.step(sample(presence='unknown',far_edge_y_normalized=None),.05)['motor']=='forward'
    assert c.step(sample(lane_valid=False),.1)['motor']=='stop'
    assert c.step(sample(),.15)['motor']=='forward'


def test_explicit_straight_trial_does_not_claim_lane_following():
    c=ParkingTrialLogic(reference(),mode='straight');c.start(0)
    d=c.step(sample(lane_valid=False),.1)
    assert d['motor']=='forward' and d['steering']=='center' and c.mode=='straight'


@pytest.mark.parametrize('changes',[{'fresh':False},{'image_size':[640,480]}])
def test_stale_or_lost_measurement_stops_and_latches(changes):
    c=ParkingTrialLogic(reference());c.start(0);c.step(sample(),.1)
    d=c.step(sample(**changes),.2)
    assert d['motor']=='stop' and d['phase']=='fault'
    assert c.step(sample(),.3)['motor']=='stop'


def test_short_detection_dropout_does_not_stop_search_but_long_loss_latches():
    c=ParkingTrialLogic(reference(),mode='straight');c.start(0)
    assert c.step(sample(presence='unknown',far_edge_y_normalized=None),.05)['motor']=='forward'
    assert c.step(sample(candidate=True,presence='unknown'),.1)['motor']=='forward'
    assert c.step(sample(candidate=False,presence='unknown',far_edge_y_normalized=None),.2)['motor']=='forward'
    assert c.step(sample(candidate=False,presence='unknown',far_edge_y_normalized=None),.4)['phase']=='fault'


def test_geometric_row_at_stop_reference_stops_without_waiting_for_display_timer():
    c=ParkingTrialLogic(reference(),mode='straight');c.start(0)
    d=c.step(sample(candidate=True,presence='unknown',far_edge_y_normalized=.76),.1)
    assert d['phase']=='braking' and d['motor']=='stop'


def test_no_timer_starts_from_neutral_command_alone():
    c=ParkingTrialLogic(reference());c.start(0)
    c.step(sample(far_edge_y_normalized=.76),.1)
    for i in range(2,61):
        assert c.step(sample(),i/10)['motor']=='stop'
    assert c.phase=='braking' and c.hold_started is None


def test_operator_stop_then_three_seconds_holds_until_explicit_continue():
    c=ParkingTrialLogic(reference());c.start(0)
    c.step(sample(far_edge_y_normalized=.76),.1)
    c.confirm_stopped(.15)
    for i in range(2,32):
        d=c.step(sample(),i/10)
        assert d['motor']=='stop' and d['phase']=='hold'
    assert c.step(sample(),3.2)['phase']=='stopped'
    assert c.step(sample(),10)['motor']=='stop'
    assert c.step(sample(),60)['motor']=='stop'
    c.continue_after_measurement(61)
    assert c.step(sample(),61.1)['motor']=='forward'
    assert c.step(sample(),61.4)['motor']=='forward'
    assert c.step(sample(),61.5)['phase']=='done'
    assert c.step(sample(),61.6)['motor']=='stop'


def test_telemetry_stopped_feedback_must_remain_fresh():
    c=ParkingTrialLogic(reference(),feedback='telemetry');c.start(0)
    c.step(sample(far_edge_y_normalized=.76),.1)
    ready=sample(telemetry_verified=True,speed_mps=0,telemetry_age_s=.01)
    c.step(ready,.2)
    c.step(ready,.4)
    d=c.step(sample(telemetry_verified=True,speed_mps=0,telemetry_age_s=1),.6)
    assert d['phase']=='braking' and c.hold_started is None
    with pytest.raises(ValueError):c.confirm_stopped(.7)


def test_clock_gap_and_approach_deadline_end_motion():
    c=ParkingTrialLogic(reference());c.start(0);c.step(sample(),.1)
    assert c.step(sample(),.7)['phase']=='fault'
    c=ParkingTrialLogic(reference(),maximum_approach_s=.2);c.start(0)
    assert c.step(sample(),.1)['motor']=='forward'
    assert c.step(sample(),.3)['phase']=='fault'


class Driver:
    owner='current-console-client'
    def __init__(self):self.commands=[]
    def status(self):
        return {'setup_token':'current-boot-token','mode':'enabled','motor_pulse_us':1500,
                'steering_center_us':1610,'test_mode':'driving','last_command_sequence':5}
    def request(self,action,payload):
        self.commands.append((action,payload));return self.status()


def test_status_and_invalid_start_never_create_motor_command(tmp_path):
    d=Driver();w=CrosswalkTrialDrive(d,{},tmp_path/'missing.json',tmp_path/'log.jsonl')
    assert w.status()['crosswalk_trial']['active'] is False
    with pytest.raises(ValueError):w.request('crosswalk_start',{'client':'wrong'})
    with pytest.raises(ValueError):
        w.request('crosswalk_start',{'client':d.owner,'setup_token':'current-boot-token'})
    with pytest.raises(FileNotFoundError):
        w.request('crosswalk_start',{'client':d.owner,'setup_token':'current-boot-token','field_ready':True})
    assert d.commands==[]


def test_manual_input_cancels_active_trial_without_mixing_motion(tmp_path):
    d=Driver();w=CrosswalkTrialDrive(d,{},tmp_path/'unused.json',tmp_path/'log.jsonl')
    w.logic=ParkingTrialLogic(reference());w.logic.start(0)
    w.request('command',{'motor':'forward'})
    assert w.logic.phase=='cancelled'
    assert [action for action,_ in d.commands]==['stop']


def test_any_operator_can_still_emergency_stop(tmp_path):
    d=Driver();w=CrosswalkTrialDrive(d,{},tmp_path/'unused.json',tmp_path/'log.jsonl')
    w.logic=ParkingTrialLogic(reference());w.logic.start(0)
    w.request('emergency',{'client':'different'})
    assert d.commands[-1][0]=='emergency'


def test_early_operator_confirmation_is_rejected():
    c=ParkingTrialLogic(reference());c.start(0)
    with pytest.raises(ValueError):c.confirm_stopped(.1)


def test_continue_cannot_skip_the_stop_or_hold():
    c=ParkingTrialLogic(reference());c.start(0)
    with pytest.raises(ValueError):c.continue_after_measurement(.1)
    c.step(sample(far_edge_y_normalized=.76),.1)
    with pytest.raises(ValueError):c.continue_after_measurement(.2)


def test_current_500ms_lease_is_not_changed_by_explicit_continue(tmp_path):
    d=Driver();w=CrosswalkTrialDrive(d,{},tmp_path/'unused.json',tmp_path/'log.jsonl')
    w.logic=ParkingTrialLogic(reference());w.logic.start(0)
    with pytest.raises(ValueError):
        w.request('crosswalk_continue',{'client':d.owner,'setup_token':'current-boot-token'})
    assert d.commands==[]


def test_terminal_failure_is_logged_and_neutral_is_sent(tmp_path):
    d=Driver();p=tmp_path/'log.jsonl'
    w=CrosswalkTrialDrive(d,{},tmp_path/'unused.json',p)
    w.logic=ParkingTrialLogic(reference());w.logic.cancel('camera stale',True)
    w._work()
    terminal=json.loads(p.read_text(encoding='utf-8'))
    assert terminal['record_type']=='terminal' and terminal['intent']['phase']=='fault'
    assert terminal['stop_command_sent'] is True and d.commands[-1][0]=='stop'


def test_retired_fixed_throttle_trial_cannot_be_started(tmp_path):
    p=tmp_path/'reference.json';p.write_text(json.dumps(reference()|{'trial_enabled':False,'disabled_reason':'closed loop required'}))
    d=Driver();w=CrosswalkTrialDrive(d,{},p,tmp_path/'log.jsonl')
    state=w.status()['crosswalk_trial']
    assert state['reference_available'] and not state['trial_enabled']
    with pytest.raises(ValueError,match='closed loop required'):
        w.request('crosswalk_start',{'client':d.owner,'setup_token':'current-boot-token','field_ready':True})
    assert d.commands==[]
