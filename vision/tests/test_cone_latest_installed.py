"""Installed debug-registry checks. All hardware calls remain fake. Author: 13246."""
import copy
import os
from pathlib import Path

import pytest

application=pytest.importorskip('carvision.application')
from carvision.cone_latest_task import install
from carvision.cone_latest_page import extend_controls
from carvision.cone_latest_engine.cone_low_speed import trial_settings
from carvision.cone_latest_engine.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard

ROOT=Path(os.environ.get('CARVISION_LATEST_TEST_ROOT',Path(__file__).resolve().parents[2]))


class Manual:
    def __init__(self):self.calls=[]
    def status(self):return {'mode':'disabled','boot_test_phase':'esc_off','setup_token':'software-only',
        'hardware_output':False,'worker_started':False,'pending_hardware_start':False}
    def request(self,a,p):self.calls.append(a);return self.status()
    def close(self):pass


@pytest.fixture
def app():
    install(application);manual=Manual();instance=application.DebugApplication(manual,{},ROOT)
    yield instance
    instance.close()


def test_original_and_latest_states_are_both_present(app):
    s=app.status()
    assert s['cone_trial']['program']=='blue-cone-avoidance-v2'
    assert s['cone_latest']['program']=='latest-cone-7fb9989'
    assert not s['cone_latest']['avoidance']['active'] and s['hardware_output'] is False


@pytest.mark.parametrize('action',['cone_prepare','traffic_prepare','enable'])
def test_latest_owner_blocks_other_movement_tasks(app,action):
    run=app.tasks['cone_latest'].inner.run;run.active=True;run.owner='software-owner'
    with pytest.raises(ValueError):app.request(action,{'client':'software-owner'})
    assert not app.manual.calls
    run.active=False


def test_existing_cone_owner_blocks_latest_prepare(app):
    run=app.tasks['cones'].run;run.active=True
    with pytest.raises(ValueError):app.request('latest_cone_prepare',{'client':'software-owner'})
    assert not app.tasks['cone_latest'].inner.run.active and not app.manual.calls
    run.active=False


def test_global_emergency_releases_latest_task(app):
    run=app.tasks['cone_latest'].inner.run;run.active=True;run.owner='software-owner'
    app.request('emergency',{'client':'software-check'})
    assert not run.active


def test_actual_frontend_can_add_latest_without_removing_old_tasks():
    from carvision.debug_page import build_controls
    before=build_controls(root=ROOT);original=before.SCRIPT
    after=extend_controls(before)
    assert before.SCRIPT==original
    for name in ('crosswalk','traffic','cone','parking','cone_latest'):
        assert 'value="'+name+'"' in after.HTML
    assert 'latest_cone_keepalive' in after.SCRIPT and 'cone_keepalive' in after.SCRIPT


def test_installed_output_guard_accepts_latest_pulses_without_real_hardware(app):
    settings=trial_settings(app.tasks['cone_latest'].settings)
    writes=[];guard=ConePulseGuard(actuator_settings(settings),lambda channel,pulse:writes.append((channel,pulse)),mode='F/B')
    for i,steer in enumerate((1350,1870,1610)):
        now=1.+i*.1
        guard.accept({'sequence':i,'sent_s':now,'action':'search','esc_us':1600,'steering_us':steer},now)
    assert (4,1600) in writes
    guard.abort('software check completed',1.25);guard.tick(2.)
    assert guard.esc==1500
