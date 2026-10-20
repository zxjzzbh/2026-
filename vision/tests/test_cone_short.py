"""Finite output and ownership tests; no UART is opened in this suite."""
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.cone_short_output import ShortGuard,validate_parameters
from carvision.cone_short_trial import ConeShortRun
from carvision.cone_observer import ConeObserverDrive

CONFIG=Path(__file__).parents[1]/'configs/cone-trial.json'


class Harness:
    def __init__(self):
        self.now=0.;self.seq=0;self.writes=[]
        self.guard=ShortGuard(lambda c,p:self.writes.append((self.now,c,p)),self.now)
    def send(self,action='keepalive',**kw):
        self.guard.accept({'action':action,'sequence':self.seq,'sent_s':self.now,**kw},self.now);self.seq+=1
    def advance(self,end):
        while self.now+.001<end:
            self.now=round(min(end,self.now+.025),6);self.send()
    def start(self,mode='turn_1350',duration=.8):
        self.send('start',mode=mode,forward_us=1575,duration_s=duration,expected_round=self.guard.round)


@pytest.mark.parametrize('mode,steer',[('turn_1350',1350),('turn_1870',1870),('brake',1610)])
def test_child_owns_alignment_forward_deadline_braking_and_recenter(mode,steer):
    h=Harness();h.start(mode)
    h.advance(3.2)
    assert h.guard.phase=='aligning' and h.guard.esc_us==1500 and h.guard.steering_us==steer
    h.advance(3.5)
    assert h.guard.phase=='driving' and h.guard.esc_us==1575 and h.guard.steering_us==steer
    h.advance(8.)
    assert h.guard.phase=='complete' and h.guard.esc_us==1500 and h.guard.steering_us==1610
    assert .8<=h.guard.brake_started_s-h.guard.drive_started_s<=.826
    pulses=[r for r in h.writes if r[1:]==(4,1400)]
    assert len(pulses)==3
    assert all(channel in (3,4) for _,channel,_ in h.writes)
    assert all(p in (1350,1610,1870) for _,ch,p in h.writes if ch==3)
    positive=sum(ch==4 and p>1500 for _,ch,p in h.writes)
    h.advance(15.)
    assert sum(ch==4 and p>1500 for _,ch,p in h.writes)==positive


def test_only_an_explicit_new_round_repeats_a_short_test():
    h=Harness();h.start();h.advance(8.)
    h.start('turn_1870',.2);h.advance(13.)
    assert h.guard.round==2 and h.guard.phase=='complete'


def test_repeated_start_cannot_extend_forward():
    h=Harness();h.start();h.advance(3.5)
    with pytest.raises(ValueError):h.start()
    assert h.guard.fault and h.guard.esc_us==1400
    h.now=3.7;h.guard.tick(h.now)
    assert h.guard.esc_us==1500


@pytest.mark.parametrize('elapsed',[.21,.5,3.])
def test_lease_loss_while_forward_always_enters_finite_braking(elapsed):
    h=Harness();h.start();h.advance(3.5);h.now+=elapsed;h.guard.tick(h.now)
    assert h.guard.fault and h.guard.esc_us==1400
    h.now+=.121;h.guard.tick(h.now)
    assert h.guard.esc_us==1500 and h.guard.emergency_until is None


def test_abort_during_release_observes_brake_release_gap():
    h=Harness();h.start(duration=.2);h.advance(3.76)
    assert h.guard.phase=='releasing'
    h.guard.abort('stop',h.now)
    assert h.guard.esc_us==1500
    h.now+=.11;h.guard.tick(h.now);assert h.guard.esc_us==1400
    h.now+=.13;h.guard.tick(h.now);assert h.guard.esc_us==1500


def test_expired_neutral_prepare_never_generates_a_brake_or_drive_pulse():
    h=Harness();h.now=2.1;h.guard.tick(h.now)
    assert h.guard.fault and all(p==1500 for _,ch,p in h.writes if ch==4)


@pytest.mark.parametrize('mode,pulse,duration',[('reverse',1575,.8),('brake',1500,.8),('brake',1626,.8),
    ('brake',True,.8),('brake',1575,0),('brake',1575,2.1),('brake',1575,float('nan'))])
def test_unreviewed_parameters_rejected(mode,pulse,duration):
    with pytest.raises(ValueError):validate_parameters(mode,pulse,duration)


def test_late_or_replayed_round_command_cannot_drive():
    h=Harness()
    with pytest.raises(ValueError):h.guard.accept({'action':'start','sequence':0,'sent_s':-1,'mode':'brake','forward_us':1575,'duration_s':.8,'expected_round':0},0.)
    assert all(p==1500 for _,ch,p in h.writes if ch==4)


def base():return {'mode':'disabled','boot_test_phase':'esc_off','setup_token':'boot','hardware_output':False}
def prep():return {'client':'page','setup_token':'boot','esc_off':True,'fb_mode_confirmed':True}


class FakeActuator:
    run=False
    def __init__(self,clock):
        self.clock=clock;self.sequence=0;self.closed=False;self.commands=[]
        self.guard=ShortGuard(lambda ch,p:self.commands.append((ch,p)),clock())
    def send(self,action,**kw):
        self.guard.accept({'action':action,'sequence':self.sequence,'sent_s':self.clock(),**kw},self.clock());self.sequence+=1
    def poll(self):return self.guard.snapshot(self.clock())
    def close(self):self.closed=True


@pytest.fixture
def session(tmp_path):
    now=[0.];opened=[]
    def factory():a=FakeActuator(lambda:now[0]);opened.append(a);return a
    run=ConeShortRun(tmp_path,factory=factory,clock=lambda:now[0])
    return run,now,opened


def owned(run,seq,**kw):return {'client':'page','run_token':run.token,'trial_sequence':seq,**kw}


def start(run,seq=0,mode='turn_1350'):
    run.command('cone_short_start',owned(run,seq,mode=mode,forward_us=1575,duration_s=.8,expected_round=run.round,
        fb_mode_confirmed=True,esc_on_static=True,field_ready=True))


def test_prepare_is_neutral_and_no_measurements_are_needed(session):
    run,now,opened=session
    assert not opened and run.status(base())['can_prepare']
    run.prepare(base(),prep());assert not opened
    run.tick(True);assert len(opened)==1 and all(p==1500 for ch,p in opened[0].commands if ch==4)
    assert run.status(base())['can_start']
    start(run)
    for i in range(1,170):
        now[0]=round(i*.05,6)
        run.command('cone_short_keepalive',owned(run,i));run.tick(True)
    assert run.phase=='complete' and run.active and run.status(base())['can_start']
    assert any(ch==4 and p>1500 for ch,p in opened[0].commands)
    run.cancel('done');assert opened[0].closed and not run.active


@pytest.mark.parametrize('failure',['camera','lease','stop','startup_cancel','startup_cancel_error'])
def test_cancel_and_failure_release_exclusive_hardware(session,failure):
    run,now,opened=session
    if failure.startswith('startup_cancel'):
        old=run.factory
        def factory():
            run.cancel('during startup')
            if failure.endswith('error'):raise ValueError('startup failed')
            return old()
        run.factory=factory
    run.prepare(base(),prep());run.tick(True)
    if failure=='camera':run.tick(False)
    elif failure=='lease':now[0]=1.;run.tick(True)
    elif failure=='stop':run.cancel('operator stop')
    assert not run.active and all(a.closed for a in opened)


def test_foreign_page_and_duplicate_commands_cannot_start(session):
    run,now,opened=session;run.prepare(base(),prep());run.tick(True)
    with pytest.raises(ValueError):run.command('cone_short_keepalive',owned(run,0,client='other'))
    start(run)
    with pytest.raises(ValueError):start(run,1)
    with pytest.raises(ValueError):run.command('cone_short_keepalive',owned(run,0))
    run.cancel('end')


def test_other_active_controls_and_missing_declarations_reject_prepare(session):
    run,_,opened=session
    for state,payload in [({**base(),'traffic_trial':{'active':True}},prep()),(base(),{**prep(),'esc_off':False}),
        ({**base(),'mode':'enabled'},prep()),(base(),{**prep(),'setup_token':'old'})]:
        with pytest.raises(ValueError):run.prepare(state,payload)
    assert not opened


def test_wrapper_keeps_unrelated_commands_and_settings_unchanged(tmp_path,monkeypatch):
    class Driver:
        def __init__(self):self.calls=[]
        def status(self):return base()
        def request(self,a,p):self.calls.append(a)
        def close(self):pass
    config=tmp_path/'isolated-cone.json';config.write_bytes(CONFIG.read_bytes())
    driver=Driver();w=ConeObserverDrive(driver,{},config,tmp_path,clock=lambda:0.,short_actuator_factory=lambda:FakeActuator(lambda:0.))
    monkeypatch.setattr(w,'_start_observer',lambda:None)
    w.request('command',{});assert driver.calls==['command']
    w.request('cone_short_prepare',prep());w.short.tick(True)
    s=w.status();assert s['cone_trial']['short_test']['active'] and not s['cone_trial']['avoidance']['can_prepare']
    assert not s['traffic_trial']['can_prepare'] and not s['crosswalk_trial']['can_prepare']
    for action in ('command','traffic_prepare','cone_prepare','cone_settings'):
        with pytest.raises(ValueError):w.request(action,{})
    w.request('emergency',{})
    assert not w.short.active and driver.calls==['command'] and not w.local_config.exists()
    w.request('command',{});assert driver.calls==['command','command']
