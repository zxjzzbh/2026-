"""Use the real guardian state and loop with fake I/O, never a vehicle."""
import json
from types import SimpleNamespace

import pytest

from carvision import pi5_pwm
from test_pi5_pwm import Clock, PWM
from test_shared_executor import borrow, rig


def continuous_state():
    clock=Clock();pwm=PWM();pwm.manual_forward_speed_enabled=True
    return pi5_pwm.GuardState(pwm,clock=clock,manual_forward_speed=True),clock,pwm


def test_shared_handoff_switches_guardian_before_motion_and_restores_after_braking(rig):
    backend=rig[2];modes=[]
    backend.set_autonomous=lambda enabled:modes.append(enabled)
    actuator,_=borrow(rig)
    assert modes==[True]
    actuator.send({'action':'creep','esc_us':1575,'steering_us':1610})
    actuator.close()
    assert modes==[True]  # Finite braking still belongs to autonomy.
    rig[0][0]=.4;rig[4].tick()
    assert modes==[True,False] and rig[3].mode=='emergency'


def test_mode_switch_requires_neutral_and_keeps_command_expiry():
    state,clock,pwm=continuous_state()
    state.request({'action':'motion','pulse':1575,'seconds':.2})
    with pytest.raises(ValueError):state.request({'action':'control_mode','autonomous':True})
    state.request({'action':'neutral'})
    state.request({'action':'control_mode','autonomous':True})
    assert state.autonomous
    clock.now=.26;state.tick()
    assert state.fault and pwm.calls[-2:]==[('neutral',),('steering_idle',)]


@pytest.mark.parametrize('value',[1,'true',None])
def test_mode_switch_rejects_non_boolean(value):
    state,_,_=continuous_state()
    with pytest.raises(ValueError):state.request({'action':'control_mode','autonomous':value})


def test_bench_guard_cannot_enable_autonomy():
    state=pi5_pwm.GuardState(PWM())
    with pytest.raises(ValueError):state.request({'action':'control_mode','autonomous':True})


def test_autonomous_guard_cannot_use_manual_high_speed_range():
    state,_,pwm=continuous_state()
    state.request({'action':'control_mode','autonomous':True})
    for pulse in (1501,1569,1626,2000,1300):
        with pytest.raises(ValueError):state.request({'action':'motion','pulse':pulse,'seconds':.1})
    assert not any(x[0]=='motion' for x in pwm.calls)
    state.request({'action':'motion','pulse':1570,'seconds':.1})
    state.request({'action':'neutral'})
    state.request({'action':'control_mode','autonomous':False})
    state.request({'action':'motion','pulse':2000,'seconds':.1})
    assert pwm.calls[-1]==('motion',2000)


@pytest.mark.parametrize('switch,overheat,expected_error',[
    (True,False,None),(False,False,'5G'),(True,True,'temperature')])
def test_real_loop_survives_link_loss_only_after_handoff_and_keeps_temperature_check(
        monkeypatch,capsys,switch,overheat,expected_error):
    now=[0.];pwm=PWM();pwm.manual_forward_speed_enabled=True
    pwm.open=lambda:None;pwm.close=lambda:[]
    real_state=pi5_pwm.GuardState
    monkeypatch.setattr(pi5_pwm,'RasAdapterPWM',lambda **kw:pwm)
    monkeypatch.setattr(pi5_pwm,'GuardState',lambda *a,**kw:real_state(*a,clock=lambda:now[0],**kw))
    monkeypatch.setattr(pi5_pwm.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(pi5_pwm.signal,'SIGHUP',1,raising=False)
    monkeypatch.setattr(pi5_pwm.signal,'signal',lambda *a:None)
    monkeypatch.setattr(pi5_pwm.Path,'read_text',lambda path,**kw:'boot' if str(path).endswith('boot_id') else '1')
    monkeypatch.setattr(pi5_pwm,'interface_is_inactive',lambda name:True)
    def command(cmd,**kw):
        if cmd[0]=='ip':
            return json.dumps([{'dev':'usb0' if now[0]<.5 else 'wlan0'}])
        if cmd[-1]=='get_throttled':return 'throttled=0x0'
        if cmd[-1]=='measure_temp':return "temp=75.0'C" if overheat and now[0]>.5 else "temp=45.0'C"
        raise AssertionError(cmd)
    monkeypatch.setattr(pi5_pwm.subprocess,'check_output',command)
    requests=([{'action':'control_mode','autonomous':True}] if switch else [])+[
        {'action':'check'} for _ in range(31)]+[{'action':'shutdown'}]
    def readable(*args):
        now[0]=round(now[0]+.1,8)
        return [0],[],[]
    monkeypatch.setattr(pi5_pwm.select,'select',readable)
    monkeypatch.setattr(pi5_pwm.sys,'stdin',SimpleNamespace(fileno=lambda:0))
    monkeypatch.setattr(pi5_pwm.os,'read',lambda *args:(json.dumps(requests.pop(0))+'\n').encode())
    args=SimpleNamespace(motor=True,steering=True,steering_channel=3,esc_channel=4,
        steering_center_us=1610,continuous_manual=True,expected_boot_id='boot',neutral_only=False,result=None)
    pi5_pwm.guard_main(args)
    replies=[json.loads(line) for line in capsys.readouterr().out.splitlines()]
    errors=[row['error'] for row in replies if 'error' in row]
    if expected_error:assert errors and expected_error in errors[-1]
    else:assert not errors and now[0]>3
