"""Desktop mode selection never interrupts an active vehicle or starts motion."""
import importlib.util
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('vehicle_desktop_launcher',ROOT/'pc-launcher/launch_vehicle.py')
launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)


def idle():
    return {'application_mode':'debug','mode':'disabled','boot_test_phase':'esc_off',
            'hardware_output':False,'worker_started':False,'pending_hardware_start':False}


@pytest.mark.parametrize('extra',[{'mode':'enabled'},{'hardware_output':True},{'hardware_output':None},
    {'pending_hardware_start':True},{'race_flow':{'active':True}},
    {'cone_trial':{'avoidance':{'active':True}}},{'parking_payment':{'motion_locked':True}}])
def test_mode_switch_requires_released_hardware_and_no_active_task(extra):
    assert not launcher.can_switch({**idle(),**extra})


def test_opening_same_mode_does_not_stop_or_restart_active_control(monkeypatch):
    state={**idle(),'mode':'enabled','hardware_output':True}
    monkeypatch.setattr(launcher,'read_state',lambda _:state)
    monkeypatch.setattr(launcher,'command',lambda *a,**k:pytest.fail('must not change service'))
    assert launcher.select_mode(object(),{},'debug') is state


def test_different_mode_refuses_active_control_before_sftp_or_service_commands(monkeypatch):
    monkeypatch.setattr(launcher,'read_state',lambda _:{**idle(),'mode':'enabled'})
    monkeypatch.setattr(launcher,'command',lambda *a,**k:pytest.fail('must not change service'))
    with pytest.raises(RuntimeError,match='正在准备'):launcher.select_mode(object(),{},'race')


@pytest.mark.parametrize('mode',['race','debug'])
def test_desktop_selects_the_explicit_entry_with_no_motion_command(mode):
    text=launcher.unit_content(mode).decode('utf-8')
    assert '/vision/tools/run_'+mode+'.py' in text
    assert 'serve_plan_b' not in text and '/api/drive/command' not in text
