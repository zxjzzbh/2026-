"""Actual race camera, console callback, command sender and guard; no hardware.

Only the camera cache, clock and final PWM transport are simulated. The primary
lamp is classified from pixels by the production TrafficSignalDetector.
"""
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from carvision.cone_command import ConeCommandSender
from carvision.race_flow import RaceFlowController
from carvision.race_flow_camera import RaceFlowCamera
from carvision.race_flow_console import RaceFlowDrive
from test_race_flow import Base
from test_race_sender_cadence import GuardActuator,advance
from test_traffic_signal import scene


ROOT=Path(__file__).resolve().parents[2]


def states():
    image=np.full((360,480,3),25,np.uint8)
    image[40:190,60:420]=scene(0)
    return {camera:SimpleNamespace(condition=threading.RLock(),state='ok',raw_jpeg=None,
        raw_frame_id=None,raw_received_ms=None,
        capture_frame=SimpleNamespace(frame_id=1,receive_monotonic_ms=0.,
            image=image.copy() if camera=='primary' else np.full_like(image,25)))
        for camera in ('primary','secondary')}


@pytest.fixture
def rig():
    now=[0.]
    actuator=GuardActuator(lambda:now[0])
    sender=ConeCommandSender(actuator,clock=lambda:now[0],threaded=False)
    cache=states()
    drive=RaceFlowDrive(Base(),cache,ROOT,clock=lambda:now[0],offline_competition=True)
    assert drive.bundle is not None,drive.error
    drive.token='current-round';drive.active=True;drive.actuator=sender
    drive.controller=RaceFlowController(drive.bundle,0.,recover_gaps=True)
    drive.controller._go('traffic',0.,'fixture enters the real traffic stage')
    drive.stage='traffic'
    reader=RaceFlowCamera(cache,drive.bundle,ROOT,lambda:now[0])
    yield now,actuator,sender,drive,reader
    reader.close()
    reader.crosswalk.close()
    sender.close()


def bind(rig):
    now,actuator,sender,drive,reader=rig
    cancel,token=drive.cancel_event,drive.token
    callback=lambda event:drive._early_traffic_stop(event,cancel,token,sender)
    reader.set_traffic_stop_callback(callback)
    return callback


def add_processing_delays(rig,*,secondary_delay=.4,road_delay=.3):
    now,actuator,sender,drive,reader=rig
    order=[]
    detect=reader.traffic.detectors['primary'].detect
    def primary(*a,**kw):
        result=detect(*a,**kw)
        assert result['state']=='red' and result['fixture_detected']
        order.append(('primary_detected_red',now[0]))
        return result
    reader.traffic.detectors['primary'].detect=primary
    secondary=reader.traffic.detectors['secondary'].detect
    def slower_secondary(*a,**kw):
        order.append(('secondary_begin',now[0],actuator.guard.esc))
        advance((now,actuator,sender),now[0]+secondary_delay)
        return secondary(*a,**kw)
    reader.traffic.detectors['secondary'].detect=slower_secondary
    inspect=reader.cones.roads['primary'].inspect
    def slower_road(*a,**kw):
        order.append(('road_begin',now[0],actuator.guard.esc))
        advance((now,actuator,sender),now[0]+road_delay)
        return inspect(*a,**kw)
    reader.cones.roads['primary'].inspect=slower_road
    return order


def test_real_race_camera_stops_via_console_and_sender_before_other_camera_and_road(rig):
    now,actuator,sender,drive,reader=rig
    # Previously accepted moving output; no new forward authorization is part
    # of the stop callback or the timing simulation below.
    sender.send_race({'action':'search','esc_us':1600,'steering_us':1610},
        run_until=120.,evidence_until=.5)
    drive.controller.last_drive=True
    bind(rig)
    order=add_processing_delays(rig)
    sample=reader.read('traffic',0.)
    assert sample is not None
    assert order[0]==('primary_detected_red',0.)
    assert order[1]==('secondary_begin',0.,1400)
    assert order[2]==('road_begin',.4,1500)
    brakes=[row for row in actuator.commands if row['action']=='brake']
    assert len(brakes)==1 and brakes[0]['sent_s']==0.
    assert now[0]==pytest.approx(.7) and actuator.guard.esc==1500
    assert actuator.guard.fault is None and not sender.race_interrupted()
    assert not drive.controller.last_drive and drive.controller.controller.brake_count==1
    assert drive.controller.controller.pulse_until==.35
    assert sample['traffic']['captured_s']==0.  # Processing never forges freshness.
    assert all(row['esc_us']<=1500 for row in actuator.commands[1:])
    # An overdue aggregate may pause/reconfirm, never undo the red stop.
    intent=drive.controller.step(sample,now[0],motion=reader.ground_motion())
    assert intent['esc_us']<=1500 and drive.controller.stage=='traffic'
    drive._send_intent(sender,intent,sample,'traffic')
    assert actuator.guard.esc<=1500


def test_repeated_actual_camera_callback_never_restarts_the_350ms_pulse(rig):
    now,actuator,sender,drive,reader=rig
    bind(rig)
    reader.read('traffic',0.)
    assert actuator.guard.esc==1400 and sender.race_brake_until==.35
    for fid,stamp in enumerate((.1,.2,.3,.4),2):
        advance((now,actuator,sender),stamp)
        for state in reader.states.values():
            state.capture_frame=SimpleNamespace(image=state.capture_frame.image,
                frame_id=fid,receive_monotonic_ms=stamp*1000)
        reader.read('traffic',stamp)
        assert drive.controller.controller.brake_count==1
    assert actuator.guard.esc==1500 and sender.race_brake_until is None
    assert len([row for row in actuator.commands if row['action']=='brake'])==1
    assert actuator.guard.fault is None and not drive.controller.controller.red_seen


@pytest.mark.parametrize('changed',['cancel','cancel_identity','token','inactive','handle','stage'])
def test_old_cancelled_or_unowned_callback_cannot_write_to_real_sender(rig,changed):
    _,actuator,sender,drive,reader=rig
    bind(rig)
    if changed=='cancel':drive.cancel_event.set()
    elif changed=='cancel_identity':drive.cancel_event=threading.Event()
    elif changed=='token':drive.token='new-round'
    elif changed=='inactive':drive.active=False
    elif changed=='handle':drive.actuator=object()
    elif changed=='stage':drive.stage='cones'
    sample=reader.read('traffic',0.)
    assert sample['traffic']['camera_observations']['primary']['signal']['state']=='red'
    assert not actuator.commands and drive.controller.controller.brake_count==0


@pytest.mark.parametrize('cancel_in_detector',[False,True])
def test_real_race_worker_binds_stop_callback_to_its_current_run(rig,cancel_in_detector):
    now,actuator,sender,drive,reader=rig
    order=add_processing_delays(rig,secondary_delay=.2,road_delay=.2)
    # Run the production worker and its callback registration. Advance directly
    # to the traffic stage because board/zebra transitions have their own tests.
    read=reader.read
    def read_stage(stage,stamp):
        drive.controller._go('traffic',stamp,'fixture skips already tested prior tasks')
        drive.stage='traffic'
        result=read('traffic',stamp)
        drive.cancel_event.set()
        return result
    reader.read=read_stage
    primary=reader.traffic.detectors['primary'].detect
    def detect(*a,**kw):
        result=primary(*a,**kw)
        if cancel_in_detector:drive.cancel_event.set()
        return result
    reader.traffic.detectors['primary'].detect=detect
    drive.reader_factory=lambda *args:reader
    drive.factory=lambda bundle:sender
    drive._work()
    brakes=[row for row in actuator.commands if row['action']=='brake']
    if cancel_in_detector:
        assert not brakes and order[1][2]==1500
    else:
        assert len(brakes)==1 and brakes[0]['sent_s']==0.
        assert order[1][2]==1400 and drive.controller.controller.brake_count==1
    assert sender.closed and actuator.closed and not drive.active
    assert all(row['esc_us']<=1500 for row in actuator.commands)
    count=len(actuator.commands)
    # The previously installed closure cannot write after cleanup/new run.
    drive.active=True;drive.token='new-round';drive.cancel_event=threading.Event()
    reader.traffic.stop_callback({'stop_only':True,'camera_id':'primary','frame_id':99,
        'captured_s':now[0],'detected_s':now[0],'state':'red','bbox_xyxy':[94,79,386,151]})
    assert len(actuator.commands)==count
