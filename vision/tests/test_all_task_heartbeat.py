"""Network-delay regressions with fake time/output, never a live car."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
from types import SimpleNamespace

import pytest

from carvision.application import DebugApplication
from carvision.brake_trial import BrakeTrialDrive
from carvision.traffic_trial import TrafficTrialDrive
from carvision.page_heartbeat import PageHeartbeat, PAGE_LEASE_S


@pytest.fixture(params=[BrakeTrialDrive, TrafficTrialDrive])
def task(request):
    value = object.__new__(request.param)
    now = [0.]
    value.clock = lambda: now[0]
    value.lock = threading.RLock()
    value.page_lock = threading.RLock()
    value.page_link = PageHeartbeat()
    value.lease_until = value.page_link.reset(0.)
    value.closed = False
    value.busy = True
    value.stop = threading.Event()
    value.client, value.token = 'page', 'current'
    value.last_sequence = 17
    value.status = lambda: pytest.fail('heartbeat cannot gather camera/worker status')
    return value, now


def packet(seq=0, **extra):
    return {'client': 'page', 'run_token': 'current', 'heartbeat_sequence': seq, **extra}


def test_slow_status_lock_and_2500ms_network_gap_do_not_stop_supervision(task):
    value, now = task
    now[0] = 2.5
    replies = []
    with value.lock:
        worker = threading.Thread(target=lambda: replies.append(value.page_heartbeat(packet())))
        worker.start()
        worker.join(.5)
        assert replies, 'status/vision lock must not block heartbeat'
    worker.join(1)
    assert replies[0]['accepted'] and replies[0]['page_lease_s'] == 3.
    assert value.lease_until == 5.5 and value.last_sequence == 17
    assert not value.stop.is_set()
    assert value.page_link.status(now[0])['maximum_receive_gap_s'] == 2.5


def test_only_new_authorized_heartbeats_extend_current_run(task):
    value, now = task
    now[0] = .4
    value.page_heartbeat(packet(7))
    deadline = value.lease_until
    now[0] = .8
    for seq in (7, 6):
        assert not value.page_heartbeat(packet(seq))['accepted']
    for bad in (packet(8, client='observer'), packet(8, run_token='previous'), packet(True), packet(-1)):
        with pytest.raises(ValueError):
            value.page_heartbeat(bad)
    assert value.lease_until == deadline
    now[0] = deadline + .001
    with pytest.raises(ValueError, match='3 秒'):
        value.page_heartbeat(packet(8))
    assert value.lease_until == deadline


def test_stop_and_new_round_cannot_be_undone_by_late_ack(task):
    value, now = task
    value.stop.set()
    with pytest.raises(ValueError):
        value.page_heartbeat(packet())
    value.stop.clear()
    value.token = 'new'
    value.lease_until = value.page_link.reset(0.)
    with pytest.raises(ValueError):
        value.page_heartbeat(packet())
    assert value.page_link.received == 0


@pytest.mark.parametrize('endpoint,key', [('crosswalk_page_heartbeat','brake'), ('traffic_page_heartbeat','traffic')])
def test_application_routes_heartbeat_without_global_status_or_task_gate(endpoint, key):
    app = object.__new__(DebugApplication)
    app.closed = False
    app.status = lambda: pytest.fail('no full status on compact path')
    app.tasks = {key: SimpleNamespace(page_heartbeat=lambda body: {'echo': body})}
    assert app.request(endpoint, packet()) == {'echo': packet()}


def test_browser_retries_for_every_autonomous_debug_task():
    node = os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:
        pytest.skip('Node required for browser behavior replay')
    root = Path(__file__).resolve().parents[2]
    result = subprocess.run([node, str(root/'vision/tests/js/all_task_heartbeat.cjs'),
                             str(root/'vision/web/debug/app.js')],
                            input=json.dumps({'html': (root/'vision/web/debug/index.html').read_text(encoding='utf-8')}),
                            text=True, encoding='utf-8', capture_output=True, timeout=15)
    assert result.returncode == 0, result.stdout+result.stderr
    assert json.loads(result.stdout)['hardware_output'] is False


def test_renewal_cannot_shorten_a_newer_heartbeat():
    link = PageHeartbeat()
    link.reset(0.)
    link.renew(.8)
    assert link.renew(.2) == .8 + PAGE_LEASE_S


def test_race_uses_500ms_local_lease_and_still_expires_independently():
    from carvision.race_flow import load_bundle
    from carvision.race_flow_console import RaceFlowDrive
    from carvision.cone_trial_output import ConePulseGuard
    root = Path(__file__).resolve().parents[2]
    flow = object.__new__(RaceFlowDrive)
    flow.owner = 'page'
    flow.drive = SimpleNamespace(borrow_actuator=lambda settings, client: settings)
    settings = flow._make_actuator(load_bundle(root))
    assert settings['actuator_lease_s'] == .5
    writes = []
    guard = ConePulseGuard(settings, lambda *args: writes.append(args), mode='F/B')
    guard.accept({'action':'neutral','esc_us':1500,'steering_us':1610,'sequence':0,'sent_s':0.}, 0.)
    guard.tick(.49)
    assert not guard.fault
    guard.tick(.501)
    assert guard.fault == 'command lease expired'
    assert all(channel != 4 or pulse == 1500 for channel, pulse in writes)
