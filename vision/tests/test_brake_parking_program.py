import json
import struct
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.brake_parking import load_settings,BrakeParkingController,GroundMotionEstimate,brake_trigger_reference
from carvision.brake_output import BrakeActuator,PulseGuard,write_parking_pulse,verify_neutral_readback
from carvision.rasadapter5 import RasAdapter
from carvision.parking_speech import TEAM_PARKING_PHRASE
from carvision.brake_runtime import replay
from carvision import brake_runtime


CONFIG=Path(__file__).parents[1]/'configs/brake-parking.json'


def sample(t,y=.55,moving=False,**change):
    return {'fresh':True,'frame_id':round(t*100)+1,'captured_s':t,'image_size':[480,360],
            'candidate':y is not None,'far_edge_y_normalized':y,'lane_valid':True,'lane_offset':0,
            'motion':{'valid':moving is not None,'moving':moving},**change}


def cmd(sequence,t,action,s=None):
    s=s or load_settings(CONFIG);p=s['pwm']
    return {'sequence':sequence,'sent_s':t,'action':action,'steering_us':1610,
            'esc_us':{'neutral':1500,'search':p['search_us'],'creep':p['creep_us'],'brake':p['brake_us']}[action]}


def test_existing_markers_are_preserved_exactly():
    s=load_settings(CONFIG)
    assert [(x['distance_m'],x['far_edge_y_normalized']) for x in s['references']]==[
        (.65,.6016713091922006),(.5,.6545961002785515),(.25,.7465181058495822)]
    assert not s['tuning_verified']


def test_forward_uses_the_previous_working_manual_point_and_short_arming():
    s=load_settings(CONFIG)
    assert s['arming_s']==3
    assert s['pwm']['search_us']==s['pwm']['creep_us']==1575


def test_no_observed_progress_ends_drive_before_the_old_ten_second_limit():
    c=BrakeParkingController(load_settings(CONFIG))
    for i in range(23):
        d=c.step(sample(i/10,.53,False),i/10)
    assert d['phase']=='fault' and d['action']=='neutral'
    assert '未观察到移动' in d['reason']


def test_known_forward_and_neutral_packets_match_original_manual_driver():
    manual=RasAdapter();new=RasAdapter();original_packets=[];new_packets=[]
    manual.send=lambda *args:original_packets.append(args)
    new.send=lambda *args:new_packets.append(args)
    for pulse in (1500,1575):
        manual.set_esc(4,pulse);write_parking_pulse(new,4,pulse)
    assert new_packets==original_packets
    with pytest.raises(ValueError):write_parking_pulse(new,1,1500)


@pytest.mark.parametrize('pulse',[1600,1605,1625])
def test_higher_traffic_forward_requires_explicit_limit_at_guard_and_transport(pulse):
    s=load_settings(CONFIG);s['pwm']['search_us']=s['pwm']['creep_us']=pulse
    writes=[]
    with pytest.raises(ValueError):PulseGuard(s,lambda *a:writes.append(a),mode='F/B')
    board=RasAdapter();board.send=lambda *a:writes.append(a)
    with pytest.raises(ValueError):write_parking_pulse(board,4,pulse)
    assert not writes
    write_parking_pulse(board,4,pulse,forward_limit_us=1625)
    assert writes==[(4,struct.pack('<BHBBH',1,20,1,4,pulse))]
    writes.clear()
    guard=PulseGuard(s,lambda *a:writes.append(a),mode='F/B',forward_limit_us=1625)
    guard.accept(cmd(0,0,'search',s),0)
    assert (4,pulse) in writes
    guard.tick(.26)  # Original 250-ms independent lease remains enforced.
    assert writes[-1]==(4,1400)
    guard.tick(.4)
    assert writes[-2:]==[(4,1500),(3,1610)]


def test_traffic_envelope_cannot_exceed_1625_or_change_default_parking_profile():
    s=load_settings(CONFIG);writes=[];board=RasAdapter();board.send=lambda *a:writes.append(a)
    for pulse in (1626,1700):
        with pytest.raises(ValueError):write_parking_pulse(board,4,pulse,forward_limit_us=1625)
    for limit in (1700,2000,True):
        with pytest.raises(ValueError):PulseGuard(s,lambda *a:writes.append(a),mode='F/B',forward_limit_us=limit)
        with pytest.raises(ValueError):BrakeActuator(s,forward_limit_us=limit)
    assert not writes
    assert s['pwm']['search_us']==s['pwm']['creep_us']==1575


def test_startup_requires_board_neutral_readback_but_does_not_claim_esc_ready():
    board=SimpleNamespace(read_position=lambda ch:{3:1610,4:1500}[ch])
    result=verify_neutral_readback(board)
    assert not result['esc_ready_verified'] and not result['physical_stop_verified']
    board.read_position=lambda ch:1575
    with pytest.raises(RuntimeError):verify_neutral_readback(board)


def test_search_creep_brake_and_hold_are_different_actions():
    settings=load_settings(CONFIG);settings['brake_trigger_distance_m']=.25
    c=BrakeParkingController(settings)
    assert c.step(sample(0,None),0)['action']=='search'
    assert c.step(sample(.1,.61),.1)['action']=='search'
    assert c.step(sample(.2,.67),.2)['action']=='creep'
    d=c.step(sample(.3,.75,True),.3)
    assert d['action']=='brake' and d['esc_us']<1500
    assert c.step(sample(.4,.77,True),.4)['action']=='brake'
    events=[]
    for i in range(5,112):
        d=c.step(sample(i/10,.78,False),i/10);events+=d['events']
        assert d['esc_us']==1500
        if i<110:assert d['phase']!='done'
    assert c.phase=='done'
    assert events==[{'event':'speak','text':TEAM_PARKING_PHRASE}]
    assert d['physical_stop_verified'] is False and d['parking_zone_verified'] is False


def test_brake_30cm_earlier_uses_55cm_interpolation_not_a_new_measurement():
    s=load_settings(CONFIG);trigger=brake_trigger_reference(s)
    expected=s['references'][1]['far_edge_y_normalized']+(s['references'][0]['far_edge_y_normalized']-s['references'][1]['far_edge_y_normalized'])/3
    assert trigger['distance_m']==.55
    assert trigger['far_edge_y_normalized']==pytest.approx(expected)
    assert trigger['measured'] is False
    c=BrakeParkingController(s)
    assert c.step(sample(0,expected-.002,True),0)['action']=='search'
    intent=c.step(sample(.1,trigger['far_edge_y_normalized'],True),.1)
    assert intent['action']=='brake' and intent['brake_pulses']==1
    assert intent['distance_estimate_m']==pytest.approx(.55)
    assert intent['brake_trigger']==trigger


@pytest.mark.parametrize('distance',[.2,.7,float('nan'),True])
def test_brake_trigger_cannot_extrapolate_outside_original_calibration(distance):
    settings=load_settings(CONFIG);settings['brake_trigger_distance_m']=distance
    with pytest.raises(ValueError):brake_trigger_reference(settings)


def test_unknown_motion_does_not_start_parking_timer():
    c=BrakeParkingController(load_settings(CONFIG))
    c.step(sample(0,.75,True),0)
    for i in range(1,33):d=c.step(sample(i/10,.78,None),i/10)
    assert d['phase']=='fault' and not c.spoken


def test_second_brake_only_if_motion_is_still_observed_and_release_finished():
    c=BrakeParkingController(load_settings(CONFIG))
    c.step(sample(0,.75,True),0)
    c.step(sample(.1,.77,True),.1)
    assert c.step(sample(.2,.77,True),.2)['action']=='neutral'
    d=c.step(sample(.3,.78,True),.3)
    assert d['action']=='brake' and d['brake_pulses']==2


def test_brake_retries_are_bounded():
    c=BrakeParkingController(load_settings(CONFIG));c.step(sample(0,.75,True),0)
    for i in range(1,35):d=c.step(sample(i/10,.8,True),i/10)
    assert c.phase=='fault' and c.brake_count==3 and d['esc_us']==1500


@pytest.mark.parametrize('change',[{'fresh':False},{'frame_id':0},{'captured_s':-1},{'image_size':[640,480]}])
def test_invalid_camera_input_never_keeps_driving(change):
    c=BrakeParkingController(load_settings(CONFIG));c.step(sample(0),0)
    d=c.step(sample(.1,**change),.1)
    assert d['phase']=='fault' and d['action']=='neutral'


def test_no_distance_extrapolation_inside_25cm():
    c=BrakeParkingController(load_settings(CONFIG))
    d=c.step(sample(0,.81),0)
    assert d['action']=='brake' and d['distance_estimate_m'] is None


def test_optical_estimator_rejects_textureless_and_repeated_frames():
    m=GroundMotionEstimate();image=np.zeros((360,480,3),np.uint8)
    assert not m.update(image,1,0)['valid']
    assert not m.update(image,2,.1)['valid']
    rng=np.random.default_rng(10);image=rng.integers(0,256,(360,480,3),dtype=np.uint8)
    m=GroundMotionEstimate();m.update(image,1,0)
    assert not m.update(image,1,.1)['valid']


def test_optical_stationarity_is_labeled_estimate_and_detects_translation():
    rng=np.random.default_rng(4);image=rng.integers(0,256,(360,480,3),dtype=np.uint8)
    m=GroundMotionEstimate();m.update(image,1,0)
    d=m.update(image,2,.1)
    assert d['valid'] and not d['moving'] and not d['physical_stop_verified']
    shifted=cv2.warpAffine(image,np.float32([[1,0,2],[0,1,0]]),(480,360))
    d=m.update(shifted,3,.2)
    assert d['valid'] and d['moving']


@pytest.mark.parametrize('mode',['F/R','F/B/R',None])
def test_actuator_rejects_other_modes_before_any_output(mode):
    writes=[]
    with pytest.raises(ValueError):PulseGuard(load_settings(CONFIG),lambda *a:writes.append(a),mode=mode)
    assert writes==[]


def test_command_lease_uses_finite_brake_then_neutral():
    writes=[];g=PulseGuard(load_settings(CONFIG),lambda *a:writes.append(a),mode='F/B')
    g.accept(cmd(0,0,'search'),0);g.tick(.26)
    assert g.fault and g.esc==1400
    g.tick(.4);assert g.esc==1500
    with pytest.raises(RuntimeError):g.accept(cmd(1,.41,'search'),.41)


def test_renewed_brake_commands_do_not_extend_a_pulse():
    writes=[];g=PulseGuard(load_settings(CONFIG),lambda *a:writes.append(a),mode='F/B')
    g.accept(cmd(0,0,'brake'),0);g.accept(cmd(1,.1,'brake'),.1);g.tick(.13)
    assert g.esc==1500
    g.accept(cmd(2,.14,'brake'),.14);assert g.esc==1500
    g.accept(cmd(3,.15,'neutral'),.15)
    g.accept(cmd(4,.24,'brake'),.24);assert g.esc==1400


def test_mismatched_pulse_faults_instead_of_executing_arbitrary_output():
    writes=[];g=PulseGuard(load_settings(CONFIG),lambda *a:writes.append(a),mode='F/B')
    with pytest.raises(ValueError):g.accept(cmd(0,0,'search')|{'esc_us':1900},0)
    assert not any(pulse==1900 for _,pulse in writes)


def test_dry_actuator_never_opens_a_worker():
    actuator=BrakeActuator(load_settings(CONFIG),run=False)
    assert actuator.process is None
    result=actuator.send({'action':'brake','esc_us':1400,'steering_us':1610})
    assert result['hardware_output'] is False
    actuator.close()


def test_replay_is_file_backed_and_output_free(tmp_path):
    inp=tmp_path/'observations.jsonl'
    inp.write_text('\n'.join(json.dumps({'now_s':i/10,'sample':sample(i/10,.75,False)}) for i in range(121)),encoding='utf-8')
    args=SimpleNamespace(config=CONFIG,input=inp,output=tmp_path/'out',path_mode='straight')
    d=replay(args)
    assert d['phase']=='done' and not d['hardware_output']
    assert not list(args.output.glob('*.html'))
    with pytest.raises(FileExistsError):replay(args)


def test_motion_during_hold_restarts_the_full_ten_seconds():
    c=BrakeParkingController(load_settings(CONFIG))
    events=[]
    for i in range(31):
        d=c.step(sample(i/10,.75,False),i/10);events+=d['events']
    assert c.phase=='hold'
    assert c.step(sample(3.1,.78,True),3.1)['action']=='brake'
    for i in range(32,138):
        d=c.step(sample(i/10,.78,False),i/10);events+=d['events']
        assert d['phase']!='done'
    for i in range(138,143):
        d=c.step(sample(i/10,.78,False),i/10);events+=d['events']
    assert d['phase']=='done'
    assert events==[{'event':'speak','text':TEAM_PARKING_PHRASE}]


def test_after_three_pulses_observe_settling_then_complete_ten_second_hold():
    c=BrakeParkingController(load_settings(CONFIG));events=[]
    for i in range(120):
        d=c.step(sample(i/10,.75,i<10),i/10);events+=d['events']
        assert d['phase']!='fault'
    assert c.brake_count==3 and c.phase=='done'
    assert events==[{'event':'speak','text':TEAM_PARKING_PHRASE}]


def test_stale_explicit_speed_feedback_does_not_fall_back_to_optical_stop():
    c=BrakeParkingController(load_settings(CONFIG))
    d=c.step(sample(0,.6,False,speed_feedback_valid=True,speed_mps=0,speed_age_s=1),0)
    assert d['phase']=='fault' and d['action']=='neutral'


def test_new_frame_id_with_reversed_timestamp_is_rejected():
    c=BrakeParkingController(load_settings(CONFIG))
    c.step(sample(.1),.1)
    assert c.step(sample(.2,captured_s=.09),.2)['phase']=='fault'


def test_configured_shorter_command_lease_is_applied():
    s=load_settings(CONFIG);s['actuator_lease_s']=.1
    g=PulseGuard(s,lambda *_:None,mode='F/B')
    g.accept(cmd(0,0,'search',s),0);g.tick(.11)
    assert g.fault=='command lease expired' and g.esc==1400


def test_camera_reader_retries_when_jpeg_and_status_cross_frames(monkeypatch):
    reader=brake_runtime.PreviewReader('http://127.0.0.1:8080',load_settings(CONFIG))
    ids=iter([1,2,2,2])
    ok,jpeg=cv2.imencode('.jpg',np.zeros((360,480,3),np.uint8));assert ok
    def get(path,limit):
        if path.startswith('/frame.jpg'):return jpeg.tobytes()
        return json.dumps({'raw_preview':{'state':'ok','frame_id':next(ids),'host_frame_age_ms':0}}).encode()
    monkeypatch.setattr(reader,'get',get)
    image,frame_id,_=reader.next()
    assert frame_id==2 and image.shape==(360,480,3)


@pytest.mark.parametrize('failure',['slow_processing','camera_disconnected'])
def test_camera_runtime_closes_output_on_stale_processing_or_capture_error(tmp_path,monkeypatch,failure):
    clock=[0.0];actuators=[]
    class Reader:
        def __init__(self,*_):pass
        def next(self):
            if failure=='camera_disconnected':raise RuntimeError('camera disconnected')
            clock[0]=.01
            return np.zeros((360,480,3),np.uint8),1,.01
    class Actuator:
        def __init__(self,*_,**__):self.closed=False;self.commands=[];actuators.append(self)
        def send(self,intent):self.commands.append(intent);return {'hardware_output':False}
        def close(self):self.closed=True
    class Detector:
        def detect(self,image,t):
            clock[0]+=.3
            return {'stripes_xywh':[],'candidate':False,'far_edge_y_normalized':None,'bbox_xyxy':None},None
    monkeypatch.setattr(brake_runtime,'time',SimpleNamespace(monotonic=lambda:clock[0]))
    monkeypatch.setattr(brake_runtime,'PreviewReader',Reader)
    monkeypatch.setattr(brake_runtime,'BrakeActuator',Actuator)
    monkeypatch.setattr(brake_runtime,'CrosswalkDetector',Detector)
    args=SimpleNamespace(config=CONFIG,seconds=1,run=False,preview='http://127.0.0.1:8080',
                         esc_mode=None,output=tmp_path/'out',path_mode='straight',port='/unused',feedback=None,show=False)
    if failure=='camera_disconnected':
        with pytest.raises(RuntimeError,match='camera disconnected'):brake_runtime.camera(args)
    else:
        assert brake_runtime.camera(args)['phase']=='fault'
    assert actuators[0].closed and not actuators[0].commands
    rows=[json.loads(line) for line in (args.output/'decisions.jsonl').read_text(encoding='utf-8').splitlines()]
    assert rows[-1]['event']=='session_ended' and rows[-1]['actuator_closed']
