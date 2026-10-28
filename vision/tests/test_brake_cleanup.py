"""Actual worker shutdown paths with fake UART, pipe, flock and clock."""
import copy
import importlib
import json
import sys
from types import SimpleNamespace

import pytest

from carvision.brake_timing import brake_cleanup_window_s,MAX_BRAKE_PULSE_S
from carvision.brake_output import PulseGuard
from carvision.brake_parking import load_settings
from test_crosswalk_profile_integration import profile,legacy,load_active


@pytest.mark.parametrize('value',[.500001,.6,True,float('nan'),float('inf'),0.,-.1])
def test_pulses_over_500ms_and_invalid_values_are_still_rejected(tmp_path,legacy,value):
    assert MAX_BRAKE_PULSE_S==.5
    settings=copy.deepcopy(legacy);settings['brake_pulse_s']=value
    with pytest.raises(ValueError):PulseGuard(settings,lambda ch,p:None,mode='F/B')
    path=tmp_path/'invalid.json';path.write_text(json.dumps(settings),encoding='utf-8')
    with pytest.raises(ValueError):load_settings(path)


def test_cleanup_grace_covers_one_full_brake_and_pending_release_only():
    assert brake_cleanup_window_s(.5)==pytest.approx(.55)
    assert brake_cleanup_window_s(.5,.1)==pytest.approx(.65)
    assert brake_cleanup_window_s(.12)==pytest.approx(.35)
    assert brake_cleanup_window_s(.35)==pytest.approx(.40)
    assert brake_cleanup_window_s(.35,.1)==pytest.approx(.50)
    with pytest.raises(ValueError):brake_cleanup_window_s(.5,float('inf'))


@pytest.mark.parametrize('module_name,stop_during_release',[
    ('brake_output',False),('cone_trial_output',False),
    ('cone_short_output',False),('cone_short_output',True)])
def test_real_worker_shutdown_completes_350ms_pulse_before_final_neutral(
        tmp_path,monkeypatch,legacy,profile,module_name,stop_during_release):
    module=importlib.import_module('carvision.'+module_name)
    settings=load_active(tmp_path,legacy,profile);clock=[0.];writes=[];closed=[]
    def sleep(seconds):clock[0]=round(clock[0]+seconds,6)
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock[0],sleep=sleep))
    monkeypatch.setitem(sys.modules,'fcntl',SimpleNamespace(LOCK_EX=1,LOCK_NB=2,flock=lambda *a:None))
    monkeypatch.setattr(module,'open',lambda *a:SimpleNamespace(fileno=lambda:0,close=lambda:None),raising=False)
    class Adapter:
        def __init__(self,*args):pass
        def open(self):pass
        def receive(self,*args):pass
        def close(self):closed.append(True)
    monkeypatch.setattr('carvision.rasadapter5.RasAdapter',Adapter)
    monkeypatch.setattr(module,'verify_neutral_readback',lambda adapter:{})
    monkeypatch.setattr(module,'write_parking_pulse',lambda adapter,ch,p,**kw:writes.append((clock[0],ch,p)))
    class Pipe:
        def __init__(self):self.sequence=0;self.last_phase=None;self.is_closed=False
        def send(self,row):
            if row.get('status'):self.last_phase=row['status'].get('phase')
        def poll(self,timeout):sleep(timeout);return True
        def recv(self):
            seq=self.sequence;self.sequence+=1
            if module_name=='cone_short_output':
                if (stop_during_release and self.last_phase=='releasing'
                        or not stop_during_release and clock[0]>=3.6):return {'close':True}
                return {'action':'start' if seq==0 else 'keepalive','sequence':seq,'sent_s':clock[0],
                        'mode':'brake','forward_us':1600,'duration_s':2.,'expected_round':0}
            if seq>0:return {'close':True}
            return {'action':'search','esc_us':settings['pwm']['search_us'],'steering_us':1610,
                    'sequence':seq,'sent_s':clock[0]}
        def close(self):self.is_closed=True
    pipe=Pipe()
    if module_name=='brake_output':module._worker(pipe,settings,'fake','F/B',1575)
    elif module_name=='cone_trial_output':module._worker(pipe,settings,'fake','F/B',1350,1870)
    else:module._worker(pipe,'fake')
    assert closed and pipe.is_closed
    esc=[(t,p) for t,ch,p in writes if ch==4]
    assert any(p>1500 for t,p in esc)
    starts=[i for i,(t,p) in enumerate(esc) if p==1400]
    assert len(starts)==(2 if stop_during_release else 1)
    for i in starts:
        end=next(t for t,p in esc[i+1:] if p==1500)
        assert .35-1e-9<=end-esc[i][0]<=.356
    if stop_during_release:
        first_end=next(t for t,p in esc[starts[0]+1:] if p==1500)
        assert esc[starts[1]][0]-first_end>=.1-1e-9
    assert esc[-1][1]==1500
    assert not any(p>1500 for t,p in esc[starts[0]+1:])
