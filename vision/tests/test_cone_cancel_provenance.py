"""Record automatic stop provenance without changing the browser's stop policy."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from carvision.cone_execution import ConeRun
from carvision.cone_low_speed import PROFILE
from carvision.cone_observer import ConeObserverDrive
from carvision.cone_planning import load_settings

ROOT = Path(__file__).resolve().parents[2]


class ClosingActuator:
    run = True

    def __init__(self,fail=False):
        self.fail = fail
        self.close_calls = 0

    def close(self):
        self.close_calls += 1
        if self.fail:raise RuntimeError('synthetic close failure')


def active_run(tmp_path,fail_close=False):
    run = ConeRun(log_root=tmp_path)
    run.active,run.phase,run.reason = True,'driving','offline active run'
    run.owner,run.token = 'synthetic-private-owner','synthetic-private-run-token'
    run.navigation = {'captured_s':1.,'ready':False,'reason':'original navigation failure'}
    run.intent = {'phase':'driving','action':'search','esc_us':1600,'steering_us':1610}
    run.folder = tmp_path/'active-record'
    run.folder.mkdir()
    run.log = (run.folder/'decisions.jsonl').open('w',encoding='utf-8')
    actuator = ClosingActuator(fail_close)
    run.actuator = actuator
    return run,actuator


@pytest.mark.parametrize('source',['stop_button','window_blur','page_hidden','page_exit','connection_lost'])
def test_real_cone_cancel_handler_preserves_existing_browser_stop_source(tmp_path,source):
    base = {'mode':'disabled','boot_test_phase':'esc_off','hardware_output':False,
            'worker_started':False,'pending_hardware_start':False,'setup_token':'offline-setup'}
    observer = ConeObserverDrive(SimpleNamespace(status=lambda:base),{},
        ROOT/'vision/configs/cone-trial.json',tmp_path,clock=lambda:12.5)
    observer.run,actuator = active_run(tmp_path)
    # The existing browser stop request has a client/source, not a run_token.
    result = observer.request('cone_cancel',{'client':observer.run.owner,'stop_source':source})
    first = observer.run.first_stop
    assert first['source']==source and first['phase']=='cancelled' and first['at_s']==12.5
    assert result['cone_trial']['avoidance']['first_stop']==first
    summary = json.loads((observer.run.folder/'summary.json').read_text(encoding='utf-8'))
    assert summary['first_stop']==first and summary['stop_history'][-1]['source']==source
    assert actuator.close_calls==1 and not observer.run.active


def test_first_fault_and_navigation_evidence_survive_later_automatic_cancels(tmp_path):
    run,actuator = active_run(tmp_path)
    original_navigation = copy.deepcopy(run.navigation)
    run.cancel('original control failure',phase='fault',source='internal',now=10.)
    first = copy.deepcopy(run.first_stop)
    run.navigation = {'captured_s':10.1,'ready':True,'reason':'later observation'}
    run.cancel('page stopped the task',source='connection_lost',now=10.1)
    run.cancel('page became hidden',source='page_hidden',now=10.2)
    summary = json.loads((run.folder/'summary.json').read_text(encoding='utf-8'))
    assert run.first_stop==first=={'phase':'fault','reason':'original control failure','source':'internal','at_s':10.}
    assert run.phase=='fault' and run.reason=='original control failure'
    assert summary['phase']=='fault' and summary['reason']=='original control failure'
    assert summary['failure_navigation']==original_navigation
    assert summary['first_stop']==first
    assert summary['stop_history'][-1]['source']=='page_hidden'
    assert actuator.close_calls==1 and run.actuator is None and run.log.closed


def test_repeated_stops_have_bounded_history_without_control_identity_data(tmp_path):
    run,actuator = active_run(tmp_path)
    run.cancel('first failure',phase='fault',now=1.)
    for index in range(20):
        run.cancel('later stop',source='window_blur',now=2.+index)
    assert len(run.stop_history)==8
    assert run.first_stop['at_s']==1. and run.first_stop['reason']=='first failure'
    assert run.stop_history[-1]['at_s']==21.
    history = json.dumps({'first_stop':run.first_stop,'stop_history':run.stop_history})
    assert 'synthetic-private-owner' not in history and 'synthetic-private-run-token' not in history
    for event in run.stop_history:
        assert not {'client','owner','owner_client','token','run_token'}.intersection(event)
    assert actuator.close_calls==1


def test_cleanup_error_is_separate_from_first_fault_and_repeated_close_is_idempotent(tmp_path):
    run,actuator = active_run(tmp_path,fail_close=True)
    # Propagation of a close failure is an API choice. It must never erase the
    # primary stop evidence or make a later page cancel close the handle twice.
    try:run.cancel('navigation stopped the run',phase='fault',source='internal',now=4.)
    except RuntimeError as error:assert 'synthetic close failure' in str(error)
    run.cancel('later page cancel',source='page_exit',now=4.1)
    summary = json.loads((run.folder/'summary.json').read_text(encoding='utf-8'))
    assert run.first_stop['reason']==run.reason=='navigation stopped the run'
    assert run.phase=='fault' and 'synthetic close failure' in run.cleanup_error
    assert summary['reason']=='navigation stopped the run'
    assert 'synthetic close failure' in summary['cleanup_error']
    assert actuator.close_calls==1 and not run.active and run.actuator is None


def test_only_accepted_new_preparation_resets_stop_provenance(tmp_path):
    run,actuator = active_run(tmp_path)
    run.cancel('previous failure',phase='fault',source='internal',now=1.)
    previous = copy.deepcopy(run.first_stop)
    old_record = run.folder/'summary.json'
    settings = load_settings(ROOT/'vision/tests/fixtures/configs/cone-trial-field-20261026.json')
    base = {'mode':'disabled','boot_test_phase':'esc_off','hardware_output':False,
            'worker_started':False,'pending_hardware_start':False,'setup_token':'offline-setup'}
    payload = {'profile':PROFILE,'client':'new-offline-page','setup_token':'offline-setup',
               'esc_off':True,'fb_mode_confirmed':True}
    with pytest.raises(ValueError):
        run.prepare(settings,base,{**payload,'esc_off':False},5.)
    assert run.first_stop==previous and run.phase=='fault'
    run.prepare(settings,base,payload,6.)
    assert run.active and run.phase=='preparing' and run.first_stop is None and run.stop_history==[]
    assert run.actuator is None and actuator.close_calls==1
    assert json.loads(old_record.read_text(encoding='utf-8'))['first_stop']==previous
    run.cancel('new round stop',source='stop_button',now=7.)
    assert run.first_stop['at_s']==7. and run.first_stop['source']=='stop_button'


def test_unconfirmed_cleanup_blocks_repreparation_without_erasing_first_fault(tmp_path):
    run,actuator = active_run(tmp_path,fail_close=True)
    try:run.cancel('original navigation failure',phase='fault',source='internal',now=1.)
    except RuntimeError:pass
    first = copy.deepcopy(run.first_stop)
    settings = load_settings(ROOT/'vision/tests/fixtures/configs/cone-trial-field-20261026.json')
    base = {'mode':'disabled','boot_test_phase':'esc_off','hardware_output':False,
            'worker_started':False,'pending_hardware_start':False,'setup_token':'offline-setup'}
    payload = {'profile':PROFILE,'client':'new-offline-page','setup_token':'offline-setup',
               'esc_off':True,'fb_mode_confirmed':True}
    with pytest.raises(ValueError):
        run.prepare(settings,base,payload,6.)
    assert run.first_stop==first and run.phase=='fault' and not run.active
    assert 'synthetic close failure' in run.cleanup_error and actuator.close_calls==1
