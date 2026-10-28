"""First-sighting stop, using sensor fixtures and no hardware output."""
import copy
import os
from pathlib import Path

import pytest

from carvision.brake_parking import BrakeParkingController, load_settings
from carvision.race_flow import RaceFlowController, load_bundle
from flow_calibration_fixture import current_crosswalk_sample

ROOT=Path(os.environ.get('CARVISION_FLOW_TEST_ROOT',Path(__file__).resolve().parents[2]))


def sample(t,y=.53,moving=False,**change):
    result=dict(fresh=True,frame_id=round(t*100)+1,captured_s=t,image_size=[480,360],
                candidate=y is not None,far_edge_y_normalized=y,
                motion={'valid':moving is not None,'moving':moving})
    result.update(change);return result


def controller():
    return BrakeParkingController(load_settings(ROOT/'vision/configs/brake-parking.json'),first_detection_stop=True)


def finish_first_stop(c,start=0.,y=.53):
    events=[]
    for i in range(1,30):
        t=round(start+i*.1,8);d=c.step(sample(t,y),t);events+=d['events']
        if any(e['event']=='crosswalk_first_detection_resume' for e in d['events']):
            return t,d,events
    pytest.fail('initial stop did not complete: '+str(d))


def test_first_detection_brakes_then_once_only_resumes_original_stop_and_speech():
    c=controller()
    assert c.step(sample(0,None,True),0)['action']=='search'
    d=c.step(sample(.1,.53,True),.1)
    assert d['action']=='brake' and d['stop_stage']=='first_detection'
    t,d,events=finish_first_stop(c,.1)
    assert t>=.1+c.s['brake_pulse_s']+c.s['stop_confirm_s']+.5
    assert not c.spoken and not any(e['event']=='speak' for e in events)
    for i in range(1,15):
        now=round(t+i*.1,8);d=c.step(sample(now,.54,True),now)
        assert d['esc_us']>1500 and d['stop_stage']=='original_operation'
    t=round(now+.1,8)
    d=c.step(sample(t,.75,True),t)
    assert d['action']=='brake' and d['brake_pulses']==1
    spoken=[];held_at=None
    for i in range(1,125):
        now=round(t+i*.1,8);d=c.step(sample(now,.75,False),now);spoken+=d['events']
        if c.hold_since is not None:held_at=c.hold_since
        if c.phase=='done':break
    assert c.phase=='done' and now-held_at>=10
    assert spoken==[{'event':'speak','text':c.s['speech_text']}]


def test_stop_confirmation_restarts_if_motion_reappears():
    c=controller();c.step(sample(0,.53,True),0)
    for i in range(1,9):c.step(sample(i/10),i/10)
    assert c.phase=='hold'
    d=c.step(sample(.9,.53,True),.9)
    assert d['action']=='brake' and c.hold_since is None
    t,d,_=finish_first_stop(c,.9)
    assert t>=1.9 and d['action']=='neutral'


def test_missing_stationarity_never_claims_a_completed_short_stop():
    c=controller();c.step(sample(0,.53,True),0)
    for i in range(1,34):d=c.step(sample(i/10,.53,None),i/10)
    assert d['phase']=='fault' and d['esc_us']==1500 and not c.spoken


@pytest.mark.parametrize('at_first_detection',[True,False])
def test_near_reference_goes_directly_to_original_hold_without_forward_resume(at_first_detection):
    c=controller();c.step(sample(0,.75 if at_first_detection else .53,True),0)
    outputs=[]
    for i in range(1,120):outputs.append(c.step(sample(i/10,.75,False),i/10))
    assert c.phase=='done' and all(d['esc_us']<=1500 for d in outputs)
    assert not any(e['event']=='crosswalk_first_detection_resume' for d in outputs for e in d['events'])
    assert sum(e['event']=='speak' for d in outputs for e in d['events'])==1


def test_valid_detection_without_distance_still_stops_and_line_loss_during_pause_is_not_retriggered():
    c=controller()
    d=c.step(sample(0,None,True,candidate=True),0)
    assert d['action']=='brake'
    t,_,_=finish_first_stop(c,y=None)
    assert c.step(sample(round(t+.1,8),None,True),round(t+.1,8))['action']=='search'


@pytest.mark.parametrize('change',[{'fresh':False},{'captured_s':-1},{'frame_id':1}])
def test_stale_frames_cannot_resume_after_initial_stop(change):
    c=controller();c.step(sample(0),0)
    s=sample(.1);s.update(change)
    assert c.step(s,.1)['phase']=='fault'


def test_shared_parking_core_stays_opt_in_and_new_crosswalk_task_rearms():
    s=load_settings(ROOT/'vision/configs/brake-parking.json');original=copy.deepcopy(s)
    parking=BrakeParkingController(s)
    assert parking.step(sample(0,.53,True),0)['action']=='search'
    assert not parking.first_detection_stop and s==original
    assert controller().step(sample(0,.53,True),0)['action']=='brake'


def test_full_race_flow_uses_new_stop_and_preserves_original_ten_second_hold():
    c=RaceFlowController(load_bundle(ROOT),0)
    c._go('crosswalk',0,'test')
    assert c.controller.first_detection_stop and c.controller.s['hold_s']==10
    tick=0
    events=[]
    def step(y=.53,moving=False,audio=None):
        nonlocal tick
        tick+=1;t=round(tick*.1,8)
        motion={'valid':True,'moving':moving,'captured_s':t}
        s=current_crosswalk_sample(c,sample(t,y,moving))
        navigation={'experimental':True,'ready':True,'captured_s':t,'frame_id':tick,
                    'ego_fraction':.5,'half_envelope':.20,'lane_bounds_px':[[100,380],[50,430]],
                    'cones':[],'uncertain':[],'source_camera':'secondary'}
        decision=c.step({'sequence':tick,'captured_s':t,'motion':motion,
                         'crosswalk':s,'navigation':navigation},t,audio or {})
        assert c.stage!='fault',decision
        events.extend(decision['events'])
        return t,decision
    _,d=step(moving=True)
    assert d['action']=='brake' and d['phase']=='crosswalk' and d['stop_stage']=='first_detection'
    for _ in range(30):
        t,d=step()
        if d.get('stop_stage')=='original_operation':break
    else:pytest.fail('first-detection stop did not resume with fresh continuous frames')
    assert t>=.1+c.controller.s['brake_pulse_s']+c.controller.s['stop_confirm_s']+.5
    assert not any(e['event']=='speak' for e in events)
    _,d=step(.54,True)
    assert d['esc_us']>1500 and d['stop_stage']=='original_operation'
    _,d=step(.75,True)
    assert d['action']=='brake' and d['stop_stage']=='original_operation'
    held_at=None
    for _ in range(130):
        t,d=step(.75,False)
        if c.controller.hold_since is not None:held_at=c.controller.hold_since
        if c.controller.phase=='done':break
    else:pytest.fail('original crosswalk hold never completed')
    assert held_at is not None and t-held_at>=10
    speech=[e for e in events if e['event']=='speak']
    assert len(speech)==1 and speech[0]['id']=='crosswalk'
    assert c.stage=='crosswalk' and c.audio_wait is not None
    _,d=step(.75,False,{'crosswalk':'command_sent'})
    assert c.stage=='traffic' and d['esc_us']>1500
