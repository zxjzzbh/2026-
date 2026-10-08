"""Reviewed board interpolation, axes release, and unchanged stop protection."""
import hashlib,json
import pytest
from carvision.manual_drive import ManualDrive, SimulationBackend
from carvision.pi5_pwm import Pi5BenchBackend, RasAdapterPWM

class Clock:
    now=0.
    def __call__(self):return self.now

class Hardware(SimulationBackend):
    hardware_output=True
    ground_held_reviewed=True
    steering_single_target=True
    steering_settle_s=.3
    steering_center_us=1715
    combined_trial=True
    reverse_available=False
    def __init__(self):self.calls=[];self.steering_active=False
    def check(self):pass
    def motion(self,pulse,seconds):self.calls.append(('motor',pulse,seconds))
    def neutral(self):self.calls.append(('neutral',))
    def steering(self,pulse):self.calls.append(('turn',pulse));self.steering_active=True
    def steering_idle(self):
        if self.steering_active:self.calls.append(('idle',));self.steering_active=False
    def close(self):self.calls.append(('close',))

def ready():
    clock,backend=Clock(),Hardware()
    drive=ManualDrive(backend,clock=clock,settling_s=0,session_s=180,ground_held=True)
    drive.request('enable',{'client':'target-test','bench_ready':True});drive.tick()
    backend.calls.clear()
    return drive,backend,clock

def command(drive,sequence,motor='stop',turn='right'):
    drive.request('command',{'client':'target-test','sequence':sequence,'motor':motor,'steering':turn})

def test_held_right_emits_one_target_then_center_and_waits_for_board_interpolation():
    drive,backend,clock=ready()
    for seq in range(1,22):
        clock.now=(seq-1)*.05;command(drive,seq);drive.tick()
    assert [c for c in backend.calls if c[0]=='turn']==[('turn',1550)]
    assert not [c for c in backend.calls if c[0]=='motor']
    drive.request('stop',{});clock.now+=.05;drive.tick()
    assert [c for c in backend.calls if c[0]=='turn']==[('turn',1550),('turn',1715)]
    clock.now+=.21;drive.tick();assert not [c for c in backend.calls if c[0]=='idle']
    clock.now+=.1;drive.tick();assert [c for c in backend.calls if c[0]=='idle']

def test_forward_right_and_axis_release_do_not_replay_or_cancel_each_other():
    drive,backend,clock=ready()
    for seq in range(1,26):
        clock.now=(seq-1)*.05;command(drive,seq,'forward');drive.tick()
    assert drive.pulse==1575 and drive.steering_us==1550
    clock.now+=.05;command(drive,26,'forward','center');drive.tick()
    assert drive.pulse==1575 and drive.steering_us==1715
    clock.now+=.05;command(drive,27,'stop','left');drive.tick()
    assert drive.pulse==1500 and drive.steering_us==1750

@pytest.mark.parametrize('lost_input',[False,True])
def test_target_steering_preserves_zero_throttle_and_recenter_on_stop_or_lost_input(lost_input):
    drive,backend,clock=ready();command(drive,1,'forward');drive.tick()
    if lost_input:clock.now=.21;drive.tick()
    else:drive.request('emergency',{'stop_source':'keyboard_space'});clock.now=.03;drive.tick()
    assert drive.mode=='emergency' and drive.pulse==1500 and drive.steering_us==1715
    assert drive.status()['lease_ms']==200

def test_uart_target_uses_verified_300ms_frame_and_only_s3(monkeypatch):
    class Board:
        def __init__(self):self.calls=[]
        def set_position(self,channel,pulse,seconds):self.calls.append((channel,pulse,seconds))
    monkeypatch.setattr('carvision.rasadapter5.RasAdapter',Board)
    pwm=RasAdapterPWM(motor=True,channel=3,esc_channel=4,center_us=1715,single_target=True)
    pwm.turn(1550);pwm.turn(1715)
    assert pwm.board.calls==[(3,1550,.3),(3,1715,.3)]
    with pytest.raises(ValueError):RasAdapterPWM(motor=False,channel=None,single_target=True)

@pytest.mark.parametrize('bad_hash',[False,True])
def test_unverified_or_changed_target_evidence_is_rejected_before_guardian_start(tmp_path,monkeypatch,bad_hash):
    boot='test-boot'
    (tmp_path/'baseline.json').write_text('{}')
    evidence={'boot_id':'other-boot','right_turn_physically_verified':True,
              'returned_to_center_physically_verified':True,'target_us':1550,'center_us':1715}
    raw=json.dumps(evidence).encode();(tmp_path/'observation.json').write_bytes(raw)
    review={'model':'Raspberry Pi 5 Model B Rev 1.0','boot_id':boot,
            'wheels_raised_confirmed_by_user':True,'esc_off_confirmed_by_user':True,
            'steering_backend':'rasadapter5a_uart','steering_channel':3,'steering_center_us':1715,
            'baseline_record':'baseline.json','steering_single_target_verified':True,
            'steering_single_target_evidence_record':'observation.json',
            'steering_single_target_evidence_sha256':'0'*64 if bad_hash else hashlib.sha256(raw).hexdigest()}
    backend=Pi5BenchBackend(tmp_path,tmp_path/'out',boot,review,steering_only=True)
    monkeypatch.setattr('carvision.pi5_pwm.inspect_pi5',lambda:{})
    monkeypatch.setattr('carvision.pi5_pwm.subprocess.Popen',lambda *a,**k:pytest.fail('guardian must not start'))
    with pytest.raises(ValueError,match='single target'):backend.open()
