"""Production camera/controller and trial worker; simulated time, no motor I/O."""
import copy

import pytest

from carvision.race_flow import FlowTrafficController
from carvision.traffic_driving import load_settings
from carvision.traffic_trial import TrafficTrialDrive
from test_traffic_driving import CONFIG
from test_traffic_tracking import BOX, FAR, fake_camera, publish_read, signal
from test_traffic_trial import Driver, DeferredThread, Speech, PROFILE, prepare, owned


def event(**changes):
    row={'camera_id':'primary','frame_id':1,'captured_s':1.,'detected_s':1.,
         'state':'red','bbox_xyxy':BOX,'stop_only':True}
    return {**row,**changes}


def test_primary_stop_reaches_controller_before_slow_second_camera():
    reader,states,now=fake_camera([signal('red')]*2,[signal('unknown')]*2)
    reader.s['max_frame_age_s']=.5
    controller=FlowTrafficController(load_settings(CONFIG),0.,120.,external_cadence=True)
    calls=[]
    def stop(e):
        intent=controller.accept_early_stop(e,now[0])
        if intent is not None:calls.append((now[0],intent))
    reader.stop_callback=stop
    secondary=reader.detectors['secondary'].detect
    def delayed(*a,**kw):
        result=secondary(*a,**kw);now[0]+=.4;return result
    reader.detectors['secondary'].detect=delayed
    try:
        sample=publish_read(reader,states,now,1,0.)
        assert len(calls)==1 and calls[0][0]==0. and calls[0][1]['esc_us']==1400
        assert now[0]==.4 and controller.pulse_until==.35
        assert not controller.red_seen and not controller.stop_sequence_complete
        sample['motion']={'valid':True,'moving':False,'captured_s':now[0],'frame_id':1}
        result=controller.step(sample,now[0])
        assert result['action']=='neutral' and result['phase']!='fault'
        assert controller.pulse_until==.35 and controller.brake_count==1
        assert controller.accept_early_stop(event(frame_id=2,captured_s=.4,detected_s=.4),.4) is None
        assert controller.pulse_until==.35
    finally:reader.close()


@pytest.mark.parametrize('changes',[
    {'state':'green'},{'stop_only':False},{'captured_s':.8},{'captured_s':1.1},
    {'detected_s':1.1},{'camera_id':'other'},{'frame_id':True},{'bbox_xyxy':[0,0,0,2]},
])
def test_early_stop_rejects_invalid_evidence_without_granting_any_motion(changes):
    controller=FlowTrafficController(load_settings(CONFIG),0.,120.,external_cadence=True)
    assert controller.accept_early_stop(event(**changes),1.) is None
    assert controller.phase=='approach' and controller.brake_count==0 and not controller.red_seen


def test_unrelated_fixture_cannot_issue_early_stop_after_identity_locked():
    controller=FlowTrafficController(load_settings(CONFIG),0.,120.,external_cadence=True)
    track=controller.approach_guard.tracks['primary'];track.box=list(BOX)
    assert controller.accept_early_stop(event(bbox_xyxy=FAR),1.) is None


def test_early_stop_does_not_override_arming_or_unlock_green():
    from carvision.traffic_driving import TrafficDrivingController
    controller=TrafficDrivingController(load_settings(CONFIG))
    assert controller.accept_early_stop(event(),1.) is None
    assert controller.phase=='arming' and controller.brake_count==0
    assert controller.stop_signal_seen and not controller.red_seen


@pytest.mark.parametrize('moment',[1.1,1.5])
def test_early_red_during_feedback_recovery_adopts_existing_finite_pulse(moment):
    controller=FlowTrafficController(load_settings(CONFIG),0.,120.,external_cadence=True)
    controller._begin_motion_recovery(1.)
    assert controller.brake_count==1
    result=controller.accept_early_stop(event(captured_s=moment,detected_s=moment),moment)
    assert result['phase']=='brake_pulse' and controller.pulse_until==1.35
    assert result['action']==('brake' if moment<1.35 else 'neutral')
    assert controller.brake_count==1 and not controller.red_seen


@pytest.mark.parametrize('cancel_during_primary',[False,True])
def test_actual_trial_worker_sends_early_brake_and_honors_cancel(tmp_path,monkeypatch,cancel_during_primary):
    monkeypatch.setattr('carvision.traffic_trial.threading.Thread',DeferredThread)
    reader,states,now=fake_camera([],[])
    now[0]=1.
    writes=[];sample_number=[0];red_at=[None]
    class Actuator:
        def send(self,intent):
            writes.append((now[0],copy.deepcopy(intent)))
            if red_at[0] is not None and now[0]>red_at[0]:drive.stop.set()
            return {'hardware_output':False}
        def close(self):pass
    drive=TrafficTrialDrive(Driver(),states,CONFIG,PROFILE,tmp_path,clock=lambda:now[0],
        actuator_factory=Actuator,reader_factory=lambda *a:reader,speech_factory=Speech)
    def primary(*a,**kw):
        if drive.phase=='approach':
            red_at[0]=now[0]
            if cancel_during_primary:drive.stop.set()
            return signal('red')
        return signal('off')
    def secondary(*a,**kw):
        if red_at[0] is not None:now[0]=round(now[0]+.3,8)
        return signal('unknown')
    reader.detectors['primary'].detect=primary
    reader.detectors['secondary'].detect=secondary
    original_read=reader.read
    def read(_):
        sample_number[0]+=1;now[0]=round(1.+sample_number[0]*.1,8)
        if sample_number[0]>60:raise AssertionError('worker did not reach red')
        drive.lease_until=now[0]+3.
        if drive.phase=='ready':
            drive.request('traffic_start',owned(drive,fb_mode_confirmed=True,esc_on_static=True,field_ready=True))
        for state in states.values():state.publish(sample_number[0],now[0])
        sample=original_read(now[0])
        sample['motion']={'valid':True,'moving':True,'captured_s':now[0],'frame_id':sample_number[0]}
        return sample
    reader.read=read
    prepare(drive);drive._work()
    assert red_at[0] is not None
    brakes=[(t,row) for t,row in writes if row['action']=='brake']
    if cancel_during_primary:assert not brakes
    else:
        assert brakes and brakes[0][0]==red_at[0]
        assert drive.controller.brake_count==1 and drive.phase!='fault'
        assert drive.controller.pulse_until==pytest.approx(red_at[0]+.35)


def test_native_traffic_frame_does_not_require_matching_jpeg():
    from types import SimpleNamespace
    import numpy as np
    reader,states,now=fake_camera([signal('red')],[signal('unknown')])
    events=[];reader.stop_callback=events.append
    now[0]=1.
    for state in states.values():
        state.raw_jpeg=None
        state.capture_frame=SimpleNamespace(image=np.zeros((360,480,3),dtype=np.uint8),
            frame_id=10,receive_monotonic_ms=1000.)
    try:
        sample=reader.read(1.)
        assert sample['fresh'] and events[0]['frame_id']==10 and events[0]['captured_s']==1.
        assert reader.jpegs['primary'] is None
    finally:reader.close()
