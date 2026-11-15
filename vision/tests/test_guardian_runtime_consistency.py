"""Guardian control/probe isolation with fake hardware only; never opens a vehicle."""
import json
import subprocess
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from carvision import pi5_pwm as pwm_module
from carvision.pi5_pwm import EnvironmentFault, EnvironmentMonitor, GuardState, Pi5BenchBackend
from test_shared_executor import CLIENT, borrow, rig


class PWM:
    manual_forward_speed_enabled = True

    def __init__(self, **kwargs): self.actions = []
    def open(self): pass
    def neutral(self): self.actions.append(('neutral',))
    def steering_idle(self): self.actions.append(('steering_idle',))
    def motion(self, pulse): self.actions.append(('motion', pulse))
    def turn(self, pulse): self.actions.append(('turn', pulse))
    def close(self): self.neutral(); return []


def test_continuous_guard_has_half_second_lease_but_keeps_finite_motion():
    now = [0.]
    pwm = PWM()
    guard = GuardState(pwm, clock=lambda: now[0], manual_forward_speed=True, lease_s=.5)
    guard.request({'action':'motion','pulse':1600,'seconds':.2})
    now[0] = .21; guard.tick()
    assert guard.motion_until is None and guard.fault is None
    assert pwm.actions[-1] == ('neutral',)
    now[0] = .499; guard.tick()
    assert guard.fault is None
    now[0] = .5; guard.tick()
    assert guard.fault == 'Pi 5 guardian command lease expired'


def test_guardian_mode_change_requires_continuous_scope_and_neutral():
    guard = GuardState(PWM(), manual_forward_speed=False)
    with pytest.raises(ValueError, match='continuous'):
        guard.request({'action':'control_mode','mode':'autonomous'})
    guard = GuardState(PWM(), manual_forward_speed=True, lease_s=.5)
    with pytest.raises(ValueError, match='continuous'):
        guard.request({'action':'control_mode','mode':'unknown'})
    guard.request({'action':'motion','pulse':1600,'seconds':.2})
    with pytest.raises(ValueError, match='neutral'):
        guard.request({'action':'control_mode','mode':'autonomous'})
    guard.request({'action':'neutral'})
    assert guard.request({'action':'control_mode','mode':'autonomous'})['control_mode']=='autonomous'
    assert guard.request({'action':'control_mode','mode':'manual'})['control_mode']=='manual'
    assert guard.motion_until is None


@pytest.mark.parametrize('failure', ['power', 'temperature', 'network'])
def test_only_network_restriction_is_removed_during_autonomy(failure):
    monitor = EnvironmentMonitor('boot', True)
    monitor.errors[failure] = 'detected '+failure
    assert monitor.failure('manual') == 'detected '+failure
    assert monitor.failure('autonomous') == (None if failure=='network' else 'detected '+failure)


@pytest.mark.parametrize('flags,boot,temperature,expected', [
    ('0x1', 'boot', 45, 'power/boot'),
    ('0x0', 'different-boot', 45, 'power/boot'),
    ('0x0', 'boot', 70, 'temperature'),
    ('0x0', 'boot', float('nan'), 'temperature'),
])
def test_real_probe_failures_remain_fatal_in_autonomous_mode(
        monkeypatch, flags, boot, temperature, expected):
    def output(command, **kwargs):
        if command[-1]=='get_throttled': return 'throttled='+flags
        if command[-1]=='measure_temp': return f"temp={temperature}'C"
        return '[{"dev":"usb0"}]'
    monkeypatch.setattr(pwm_module.subprocess, 'check_output', output)
    monkeypatch.setattr(Path, 'read_text', lambda path, **kwargs:
        boot if 'boot_id' in path.as_posix() else '1')
    monkeypatch.setattr(pwm_module, 'interface_is_inactive', lambda name: True)
    monitor = EnvironmentMonitor('boot', True)
    monitor.poll()
    assert expected in monitor.failure('autonomous')


def test_monitor_runs_slow_probes_without_holding_its_result_lock(monkeypatch):
    entered, release = threading.Event(), threading.Event()
    monitor = EnvironmentMonitor('boot', False)
    def blocked_power():
        entered.set()
        assert release.wait(2)
    monkeypatch.setattr(monitor, '_power', blocked_power)
    monitor.start()
    try:
        assert entered.wait(1)
        started = time.monotonic()
        for _ in range(50): assert monitor.failure('autonomous') is None
        assert time.monotonic()-started < .2
    finally:
        release.set(); monitor.close()
    assert not monitor.thread.is_alive()


def test_transient_probe_failure_has_bounded_grace_and_diagnostics():
    now = [0.]
    monitor = EnvironmentMonitor('boot', False, clock=lambda:now[0])
    monitor._probe('power', lambda:None)
    def timeout(): raise subprocess.TimeoutExpired('vcgencmd', .2)
    now[0] = .4; monitor._probe('power', timeout)
    now[0] = .9
    assert monitor.failure('autonomous') is None
    assert monitor.diagnostics()['unavailable']['power']['failed_for_s'] == pytest.approx(.5)
    monitor._probe('power', timeout)
    now[0] = 1.41
    assert 'probe unavailable' in monitor.failure('autonomous')
    monitor._probe('power', lambda:None)
    assert monitor.failure('autonomous') is None
    assert monitor.diagnostics()['unavailable'] == {}
    now[0] = 4.42
    assert 'sample expired' in monitor.failure('autonomous')


def test_unavailable_first_sample_is_not_treated_as_healthy():
    monitor = EnvironmentMonitor('boot', True)
    def timeout(): raise subprocess.TimeoutExpired('vcgencmd', .2)
    monitor._probe('power', timeout)
    assert 'probe unavailable' in monitor.failure('autonomous')


@pytest.mark.parametrize('stale_bad', [False,True])
def test_network_mode_generation_discards_inflight_old_results(stale_bad):
    monitor = EnvironmentMonitor('boot', True)
    entered, release = threading.Event(), threading.Event()
    monitor.switch_mode('autonomous')
    def previous():
        entered.set()
        assert release.wait(1)
        if stale_bad: raise EnvironmentFault('old route failure')
    thread = threading.Thread(target=lambda:monitor._probe('network', previous))
    thread.start(); assert entered.wait(1)
    monitor.switch_mode('manual')
    release.set(); thread.join(1)
    assert not thread.is_alive()
    assert not monitor.manual_ready() and monitor.failure('manual') is None
    assert 'network' not in monitor.last_good and 'network' not in monitor.errors
    monitor._probe('network', lambda:None)
    assert monitor.manual_ready() and monitor.failure('manual') is None


def test_manual_pending_check_is_bounded_and_cannot_accept_motion():
    now = [0.]
    monitor = EnvironmentMonitor('boot', True, clock=lambda:now[0])
    monitor.switch_mode('manual')
    guard = GuardState(PWM(), clock=lambda:now[0], manual_forward_speed=True, lease_s=.5)
    guard.manual_ready = monitor.manual_ready()
    with pytest.raises(ValueError, match='fresh manual network check pending'):
        guard.request({'action':'motion','pulse':1600,'seconds':.2})
    assert guard.motion_until is None
    guard.request({'action':'neutral'})
    assert not guard.request({'action':'check'})['manual_ready']
    now[0] = 1.01
    assert 'fresh manual network check timed out' in monitor.failure('manual')


def test_backend_waits_for_fresh_manual_ack_with_check_requests(monkeypatch):
    backend = Pi5BenchBackend.__new__(Pi5BenchBackend)
    calls = []
    def rpc(action, **kwargs):
        calls.append(action)
        return {'control_mode':'manual','manual_ready':len(calls)>=3,
                'environment':{'network_generation':2}}
    backend.rpc = rpc
    monkeypatch.setattr(pwm_module.time, 'sleep', lambda delay:None)
    backend.set_control_mode('manual')
    assert calls == ['control_mode','check','check']


def test_backend_rejects_changed_mode_generation_during_manual_wait(monkeypatch):
    backend = Pi5BenchBackend.__new__(Pi5BenchBackend)
    calls = []
    def rpc(action, **kwargs):
        calls.append(action)
        return {'control_mode':'manual','manual_ready':action=='check',
                'environment':{'network_generation':len(calls)}}
    backend.rpc = rpc
    monkeypatch.setattr(pwm_module.time, 'sleep', lambda delay:None)
    with pytest.raises(RuntimeError, match='mode changed'):
        backend.set_control_mode('manual')


def test_backend_requires_the_guardian_mode_acknowledgment():
    backend = Pi5BenchBackend.__new__(Pi5BenchBackend)
    calls = []
    backend.rpc = lambda *args, **kwargs: calls.append((args,kwargs)) or {'ok':True}
    with pytest.raises(RuntimeError, match='acknowledged'):
        backend.set_control_mode('autonomous')
    assert calls == [(('control_mode',), {'mode':'autonomous'})]


def test_owner_status_exposes_probe_unavailability_without_renewing_motion(rig):
    _, _, backend, _, owner, _ = rig
    backend.environment_diagnostics = {'unavailable':{'power':{'reason':'timeout','failed_for_s':.5}}}
    before = list(backend.writes)
    assert owner.status()['guardian_environment'] == backend.environment_diagnostics
    assert backend.writes == before


def test_handoff_checks_owner_before_mode_rpc_and_switches_before_pausing(rig):
    _, _, backend, controller, owner, settings = rig
    changes = []
    def change(mode): changes.append((mode, controller.stop_event.is_set(), controller.mode))
    backend.set_control_mode = change
    with pytest.raises(ValueError):
        owner.dispatch('autonomy_acquire', {'client':'foreign-owner','settings':settings,'fb_mode_confirmed':True})
    assert changes == []
    actuator, _ = borrow(rig)
    assert changes[0] == ('autonomous',False,'enabled')
    actuator.close()
    assert len(changes)==1 and owner.guardian_mode=='autonomous'
    assert controller.mode == 'emergency' and not owner.active
    owner.dispatch('reset',{})
    owner.dispatch('enable',{'client':CLIENT,'bench_ready':True})
    assert changes[-1][0] == 'manual'
    assert all(row[0]!='motion' for row in backend.writes)


@pytest.mark.parametrize('restore_fails', [False, True])
def test_failed_mode_switch_never_retires_the_manual_watcher_first(rig, restore_fails):
    _, _, backend, controller, owner, _ = rig
    controller.start(); watcher = controller.thread
    modes = []
    def change(mode):
        modes.append(mode)
        if mode=='autonomous' or restore_fails:
            raise RuntimeError('mode ACK lost')
    backend.set_control_mode = change
    with pytest.raises(RuntimeError, match='mode ACK lost'):
        borrow(rig)
    assert modes == ['autonomous','manual']
    assert watcher.is_alive() and not controller.stop_event.is_set()
    assert not owner.active
    assert bool(owner.handoff_fault) is restore_fails
    # Allow fixture cleanup without simulating a permanently broken backend.
    backend.set_control_mode = lambda mode: None


def test_release_finishes_brake_without_restoring_manual_mode(rig):
    now, _, backend, controller, owner, _ = rig
    modes = []
    backend.set_control_mode = lambda mode: modes.append((mode,now[0],owner.guard.esc if owner.guard else 1500))
    actuator, _ = borrow(rig)
    actuator.send({'action':'search','esc_us':1600,'steering_us':1610})
    now[0]=.05; actuator.close()
    assert [mode for mode,_,_ in modes]==['autonomous']
    now[0]=.41; owner.tick()
    assert [mode for mode,_,_ in modes]==['autonomous']
    assert owner.guard is None and controller.mode=='emergency'
    owner.dispatch('reset',{})
    owner.dispatch('enable',{'client':CLIENT,'bench_ready':True})
    assert modes[-1]==('manual',.41,1500)


def test_mode_restore_failure_cannot_reenable_manual_motion(rig):
    _, _, backend, controller, owner, _ = rig
    backend.set_control_mode = lambda mode: None
    actuator, _ = borrow(rig)
    def fail(mode): raise RuntimeError('manual mode ACK lost')
    backend.set_control_mode = fail
    owner.dispatch('autonomy_release', {'client':CLIENT,'worker_token':actuator.token})
    assert not owner.active and not owner.handoff_fault and controller.mode=='emergency'
    owner.dispatch('reset',{})
    with pytest.raises(RuntimeError, match='manual mode ACK lost'):
        owner.dispatch('enable',{'client':CLIENT,'bench_ready':True})
    assert not owner.active and owner.handoff_fault and controller.mode=='fault'
    with pytest.raises(ValueError): owner.dispatch('enable',{'client':CLIENT,'bench_ready':True})
    backend.set_control_mode = lambda mode: None


@pytest.mark.parametrize('autonomous,route_change,transient_timeout,restore_manual', [
    (False,False,False,False),(True,True,False,False),(False,False,True,False),
    (True,True,False,True)])
def test_real_guardian_loop_services_commands_during_slow_environment_probes(
        monkeypatch, capsys, autonomous, route_change, transient_timeout, restore_manual):
    """The original 150-ms-per-probe reproduction now completes without fault."""
    states, probe_threads, route_calls, power_calls, reads = [], [], [0], [0], [0]
    original_guard = pwm_module.GuardState
    def guard(pwm, **kwargs):
        state=original_guard(pwm,**kwargs);states.append(state);return state
    monkeypatch.setattr(pwm_module,'RasAdapterPWM',PWM)
    monkeypatch.setattr(pwm_module,'GuardState',guard)
    monkeypatch.setattr(pwm_module.signal,'signal',lambda *args:None)
    monkeypatch.setattr(pwm_module.signal,'SIGHUP',1,raising=False)
    original_read = Path.read_text
    def read(path,*args,**kwargs):
        if path.as_posix()=='/proc/sys/kernel/random/boot_id': return 'boot'
        if path.as_posix()=='/sys/class/net/usb0/carrier': return '1'
        return original_read(path,*args,**kwargs)
    monkeypatch.setattr(Path,'read_text',read)
    monkeypatch.setattr(pwm_module,'interface_is_inactive',lambda name:True)
    def output(command,**kwargs):
        if kwargs.get('timeout')==.2:
            probe_threads.append(threading.current_thread().name)
            time.sleep(.15)
        if command[-1]=='get_throttled':
            power_calls[0]+=1
            if transient_timeout and power_calls[0]==4:
                raise subprocess.TimeoutExpired(command,.2)
            return 'throttled=0x0'
        if command[-1]=='measure_temp': return "temp=45.0'C"
        route_calls[0]+=1
        changed=route_change and route_calls[0]>1 and (not restore_manual or route_calls[0]==2)
        return json.dumps([{'dev':'eth0' if changed else 'usb0'}])
    monkeypatch.setattr(pwm_module.subprocess,'check_output',output)
    monkeypatch.setattr(pwm_module.sys,'stdin',SimpleNamespace(fileno=lambda:42))
    def ready(*args): time.sleep(.02);return ([42],[],[])
    monkeypatch.setattr(pwm_module.select,'select',ready)
    def receive(*args):
        reads[0]+=1
        if reads[0]==1 and autonomous: return b'{"action":"control_mode","mode":"autonomous"}\n'
        if reads[0]>=150: return b'{"action":"shutdown"}\n'
        if restore_manual and reads[0]==120: return b'{"action":"neutral"}\n'
        if restore_manual and reads[0]==121: return b'{"action":"control_mode","mode":"manual"}\n'
        if restore_manual and reads[0]>121: return b'{"action":"check"}\n'
        return b'{"action":"motion","pulse":1600,"seconds":0.2}\n'
    monkeypatch.setattr(pwm_module.os,'read',receive)
    args=SimpleNamespace(motor=True,steering=True,steering_channel=3,esc_channel=4,
        steering_center_us=1610,neutral_only=False,expected_boot_id='boot',continuous_manual=True,result=None)
    pwm_module.guard_main(args)
    replies=[json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert reads[0]==150 and not any('error' in row or row.get('fault') for row in replies)
    assert states[0].fault is None and states[0].lease_s==.5
    assert states[0].control_mode==('autonomous' if autonomous and not restore_manual else 'manual')
    assert route_calls[0]>=2
    assert 'guardian-environment' in probe_threads
    if transient_timeout:
        assert any((row.get('environment') or {}).get('unavailable') for row in replies)
    if restore_manual:
        assert any((row.get('environment') or {}).get('manual_network_pending') for row in replies)
        assert route_calls[0]>=3 and states[0].manual_ready and states[0].motion_until is None
