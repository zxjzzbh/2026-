"""Real guards with fake output/clock; no camera, UART or vehicle access."""
import time

import pytest

from carvision.brake_output import PulseGuard
from carvision.cone_trial_output import ConePulseGuard
from carvision.cone_command import ConeCommandSender
from carvision.cone_low_speed import LowSpeedNavigator, DualLowSpeedNavigator
from carvision.cone_road import ConeRoadContext
from test_cone_turn_corridor import track, observe
from test_cone_low_speed import settings
from test_shared_executor import rig, Worker, CLIENT
from carvision.shared_executor import BorrowedActuator


def profile():
    return {'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':1600,'creep_us':1570,'brake_us':1400},
            'brake_pulse_s':.12,'brake_release_s':.1,'actuator_lease_s':.2}


class FakeActuator:
    run=True
    def __init__(self,clock):
        self.clock=clock;self.rows=[];self.closed=False
        self.guard=ConePulseGuard(profile(),lambda *args:None,mode='F/B')
    def send(self,intent):
        command={k:intent[k] for k in ('action','esc_us','steering_us')}
        command.update(sequence=len(self.rows),sent_s=self.clock())
        if 'control_profile' in intent:command['control_profile']=intent['control_profile']
        self.guard.accept(command,self.clock());self.rows.append(command)
        return {'hardware_output':True,'command':command}
    def close(self):self.closed=True


def intent(t=0.,camera='secondary',fid=1):
    return {'action':'search','esc_us':1600,'steering_us':1500,'control_profile':'cone',
            'observation_s':t,'observation_camera':camera,'observation_frame_id':fid}


def test_201ms_and_499ms_command_gaps_are_accepted_then_500ms_expires():
    guard=ConePulseGuard(profile(),lambda *args:None,mode='F/B')
    def command(t,seq):return {**intent(t),'sequence':seq,'sent_s':t}
    guard.accept(command(0.,0),0.)
    guard.accept(command(.2013,1),.2013)
    guard.accept(command(.7,2),.7)
    guard.tick(1.1999);assert guard.fault is None
    guard.tick(1.2);assert guard.fault=='command lease expired'


def test_other_stages_return_to_original_lease_immediately():
    guard=ConePulseGuard(profile(),lambda *args:None,mode='F/B')
    guard.accept({**intent(),'sequence':0,'sent_s':0.},0.)
    guard.accept({'action':'neutral','esc_us':1500,'steering_us':1610,'sequence':1,'sent_s':.1},.1)
    assert guard.deadline==pytest.approx(.3)
    guard.tick(.301);assert guard.fault=='command lease expired'
    assert PulseGuard({**profile(),'actuator_lease_s':.5},lambda *args:None,mode='F/B',forward_limit_us=1625)
    with pytest.raises(ValueError):
        PulseGuard({**profile(),'actuator_lease_s':.501},lambda *args:None,mode='F/B',forward_limit_us=1625)


def test_live_shared_worker_uses_cone_500ms_and_original_segment_envelope(rig):
    now,clock,backend,manual,owner,config=rig
    actuator=BorrowedActuator(Worker(owner),config,CLIENT,clock=clock)
    sender=ConeCommandSender(actuator,clock=clock,threaded=False)
    try:
        sender.send(intent())
        assert owner.guard.deadline==.5
        now[0]=.2013;owner.tick();assert owner.active
        sender.tick();assert owner.guard.deadline==pytest.approx(.7013)
        # Switching back to the traffic/parking caller restores 200 ms.
        now[0]=.25;sender.send({'action':'neutral','esc_us':1500,'steering_us':1610})
        assert owner.guard.deadline==pytest.approx(.45)
        now[0]=.451;owner.tick();assert not owner.active
        assert all(row[3]<=.2 for row in backend.writes if row[0]=='motion')
    finally:sender.close()


def test_sender_keeps_cadence_during_visual_gap_but_does_not_extend_source_life():
    now=[0.];actuator=FakeActuator(lambda:now[0])
    sender=ConeCommandSender(actuator,clock=lambda:now[0],threaded=False)
    sender.send(intent())
    for t in (.05,.1,.15,.2,.25,.3,.35,.4,.45):now[0]=t;sender.tick()
    assert len(actuator.rows)==10 and not actuator.guard.fault
    # Re-submitting this old frame does not grant another 500 ms.
    now[0]=.49;sender.send(intent());assert sender.deadline==.5
    now[0]=.5;sender.tick()
    assert sender.fault and actuator.rows[-1]['action']=='brake'
    count=len(actuator.rows);now[0]=.6;sender.tick();assert len(actuator.rows)==count
    actuator.guard.tick(.621);assert actuator.guard.esc==1500
    with pytest.raises(RuntimeError,match='500 ms'):sender.send(intent(.6,fid=2))
    sender.close();assert actuator.closed


def test_stage_change_retires_cone_replay_and_stop_is_sent_immediately():
    now=[0.];actuator=FakeActuator(lambda:now[0]);sender=ConeCommandSender(actuator,clock=lambda:now[0],threaded=False)
    sender.send(intent());now[0]=.05
    sender.send({'action':'brake','esc_us':1400,'steering_us':1610})
    assert sender.slot is None and actuator.guard.brake_until==pytest.approx(.17)
    n=len(actuator.rows);now[0]=.1;sender.tick();assert len(actuator.rows)==n
    sender.close();now[0]=.15;sender.tick();assert len(actuator.rows)==n


def test_page_ownership_expiry_stops_replay_even_when_pixels_are_still_fresh():
    now=[0.];actuator=FakeActuator(lambda:now[0]);sender=ConeCommandSender(actuator,clock=lambda:now[0],threaded=False)
    sender.send({**intent(),'page_deadline_s':.3})
    now[0]=.29;sender.tick();assert sender.fault is None
    now[0]=.3;sender.tick()
    assert '心跳' in sender.fault and actuator.rows[-1]['action']=='brake'
    sender.refresh_page_deadline(.8);assert sender.fault and sender.slot is None
    sender.close()


def test_fresh_page_heartbeats_cannot_extend_old_visual_result_lifetime():
    now=[0.];actuator=FakeActuator(lambda:now[0]);sender=ConeCommandSender(actuator,clock=lambda:now[0],threaded=False)
    sender.send({**intent(),'page_deadline_s':.3})
    now[0]=.2;sender.refresh_page_deadline(.7)
    assert sender.deadline==.5
    now[0]=.4;sender.refresh_page_deadline(.9);sender.tick()
    now[0]=.5;sender.tick()
    assert '视觉' in sender.fault and actuator.rows[-1]['action']=='brake'
    sender.close()


def test_camera_switch_accepts_fresh_view_but_reused_frame_cannot_renew():
    now=[.2];actuator=FakeActuator(lambda:now[0]);sender=ConeCommandSender(actuator,clock=lambda:now[0],threaded=False)
    sender.send(intent(.2))
    now[0]=.25;sender.send(intent(.19,'primary',7));assert sender.deadline==pytest.approx(.69)
    with pytest.raises(ValueError,match='reused'):sender.send(intent(.24,'primary',7))
    sender.close()


def test_background_thread_sends_while_vision_caller_is_blocked():
    actuator=FakeActuator(time.monotonic);sender=ConeCommandSender(actuator)
    try:
        sender.send(intent(time.monotonic()))
        time.sleep(.24)  # Represents a slow vision call, not a new observation.
        assert len(actuator.rows)>=3 and not actuator.guard.fault
    finally:sender.close()
    count=len(actuator.rows);time.sleep(.07);assert len(actuator.rows)==count


@pytest.mark.parametrize('side',['left','right'])
def test_opted_in_single_edge_uses_current_pixels_until_one_second(settings,side):
    settings['scene'].update(surface='indoor_white_line_track',white_v_min=200)
    nav=LowSpeedNavigator(settings);nav.single_recovery_s=1.
    image=track();assert nav.update(image,observe(settings,image),1.)['line_state']=='paired'
    image=track(sides=(side,),shift=8.)
    for t,fid in ((1.4,2),(1.8,3),(1.99,4)):
        result=nav.update(image,observe(settings,image,t,fid),t)
        assert result['line_state']=='single' and result['line_observed_s']==1.
        assert not result['boundary_observed'] and nav.paired_corridor[0]==1.
    result=nav.update(image,observe(settings,image,2.001,5),2.001)
    assert result['line_state']=='missing'


def test_single_limit_is_explicit_and_other_stages_keep_250ms(settings):
    dual=DualLowSpeedNavigator(settings)
    assert all(v.single_recovery_s==.25 for v in dual.views.values())
    dual.set_single_recovery_limit(1.);assert all(v.single_recovery_s==1. for v in dual.views.values())
    dual.set_single_recovery_limit(.25);assert all(v.single_recovery_s==.25 for v in dual.views.values())


def test_successful_threshold_is_tried_first_without_changing_saved_setting(settings):
    import numpy as np
    settings['scene'].update(surface='indoor_white_line_track',white_v_min=255)
    road=ConeRoadContext(settings,turn_aware=True)
    image=track(value=205);mask=np.zeros(image.shape[:2],dtype=np.uint8)
    first=road.inspect(image,mask,frame_id=1,captured_s=1.)
    assert first['valid'] and first['boundary_diagnostics']['effective_white_v_min']==200
    again=road.inspect(image,mask,frame_id=2,captured_s=1.1)
    assert again['valid'] and len(again['boundary_diagnostics']['threshold_attempts'])==1
    assert settings['scene']['white_v_min']==255
    fresh=ConeRoadContext(settings,turn_aware=True).inspect(image,mask,frame_id=3,captured_s=1.2)
    assert fresh['boundary_diagnostics']['threshold_attempts'][0]['white_v_min']==255
