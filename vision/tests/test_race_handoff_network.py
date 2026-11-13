"""Exercise production ownership/HTTP with fake outputs and adverse timing."""
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
from http.client import HTTPConnection
from types import SimpleNamespace

import pytest

from carvision.race_flow import load_bundle
from carvision.race_flow_console import RaceFlowDrive,RACE_PAGE_LEASE_S
from carvision.web_preview import PreviewState,make_server

ROOT=Path(__file__).resolve().parents[2]


@pytest.fixture
def active_flow(tmp_path,monkeypatch):
    bundle=load_bundle(ROOT)
    monkeypatch.setattr('carvision.race_flow_console.load_bundle',lambda *a:copy.deepcopy(bundle))
    calls=[];now=[0.]
    base=SimpleNamespace(status=lambda:{'mode':'enabled','owner_client':'page','setup_token':'setup'},
                         request=lambda action,payload:calls.append((action,dict(payload))))
    flow=RaceFlowDrive(base,{},tmp_path,live=True,clock=lambda:now[0],
                       actuator_factory=lambda _:pytest.fail('test must not acquire hardware'))
    flow.active=True;flow.stage='handoff';flow.owner='page';flow.token='current-auto'
    flow.lease_until=RACE_PAGE_LEASE_S;flow.last_heartbeat_s=0.
    yield flow,now,calls
    flow.cancel('test finished')


@pytest.mark.parametrize('stage',['handoff','wait_board','traffic','cones'])
@pytest.mark.parametrize('source',['key_release','gamepad_release','status_connection_lost','stop_button'])
def test_late_manual_stop_cannot_cancel_autonomy_or_reach_worker(active_flow,stage,source):
    flow,_,calls=active_flow;flow.stage=stage
    flow.request('stop',{'client':'page','control_scope':'manual','stop_source':source})
    assert flow.active and flow.stage==stage and not flow.cancel_event.is_set()
    assert not calls and flow.ignored_manual_stops==1


def test_old_client_key_release_is_also_ignored_after_handoff(active_flow):
    flow,_,calls=active_flow
    flow.request('stop',{'client':'page','stop_source':'key_release'})
    assert flow.active and not calls


@pytest.mark.parametrize('action',['race_cancel','race_exit','stop'])
def test_old_autonomous_round_cannot_cancel_current_round(active_flow,action):
    flow,_,calls=active_flow
    flow.request(action,{'client':'page','control_scope':'race','run_token':'old-auto'})
    assert flow.active and not calls


def test_current_explicit_stop_and_global_emergency_still_stop(active_flow):
    flow,_,calls=active_flow
    flow.request('race_cancel',{'client':'page','control_scope':'race','run_token':'current-auto','stop_source':'exit_auto'})
    assert not flow.active and flow.stage=='cancelled'
    assert flow.last_stop['source']=='exit_auto'
    # The emergency path is global, including from another observer and with
    # stale/mislabelled scope. It must never depend on a current run token.
    flow.active=True;flow.stage='handoff';flow.cancel_event=threading.Event()
    flow.request('emergency',{'client':'observer','control_scope':'manual','run_token':'old'})
    assert not flow.active and [c[0] for c in calls]==['emergency']


def heartbeat(seq=0,**extra):
    return {'client':'page','run_token':'current-auto','heartbeat_sequence':seq,**extra}


def test_slow_1740ms_heartbeat_only_renews_supervision(active_flow):
    flow,now,calls=active_flow;now[0]=1.74
    reply=flow.request('race_page_heartbeat',heartbeat())
    assert reply['accepted'] and reply['page_lease_s']==3.
    assert flow.lease_until==pytest.approx(4.74) and not calls
    assert flow.sequence==-1 and flow.heartbeat_count==1


def test_duplicate_reordered_foreign_and_expired_heartbeats_never_revive(active_flow):
    flow,now,_=active_flow;now[0]=.3
    flow.request('race_page_heartbeat',heartbeat(2));deadline=flow.lease_until
    now[0]=.4
    for seq in (1,2):assert not flow.request('race_page_heartbeat',heartbeat(seq))['accepted']
    for bad in (heartbeat(3,client='other'),heartbeat(3,run_token='old'),heartbeat(True),heartbeat(-1)):
        with pytest.raises(ValueError):flow.request('race_page_heartbeat',bad)
    assert flow.lease_until==deadline
    now[0]=deadline+.001
    with pytest.raises(ValueError,match='3 秒'):flow.request('race_page_heartbeat',heartbeat(3))
    assert flow.lease_until==deadline
    flow.cancel('page expired')
    with pytest.raises(ValueError):flow.request('race_page_heartbeat',heartbeat(4))
    assert not flow.active and flow.cancel_event.is_set()


def test_worker_stops_when_three_second_page_lease_really_expires(active_flow):
    flow,now,calls=active_flow;sent=[];closed=[]
    flow.factory=lambda _:SimpleNamespace(run=True,send=lambda intent:sent.append(intent),close=lambda:closed.append(True))
    flow.reader_factory=lambda *a:SimpleNamespace(read=lambda *a:pytest.fail('expired session cannot drive'),preview=None)
    now[0]=3.001
    flow._work()
    assert not flow.active and flow.stage=='fault' and '3 秒' in flow.reason
    assert closed==[True] and not sent and not calls


def test_heartbeat_does_not_wait_for_status_or_operator_transition_locks(active_flow):
    flow,now,_=active_flow;now[0]=.2;result=[]
    with flow.lock,flow.transition_lock:
        t=threading.Thread(target=lambda:result.append(flow.request('race_page_heartbeat',heartbeat())))
        t.start();t.join(.5)
        assert result and result[0]['accepted']
    t.join(1)


def test_authenticated_lightweight_http_path_never_reads_full_status(active_flow):
    flow,now,_=active_flow;now[0]=.4
    flow.status=lambda:pytest.fail('heartbeat must not assemble status or query worker')
    camera=PreviewState();server=make_server(('127.0.0.1',0),camera,drive=flow,build_controls=False)
    worker=threading.Thread(target=server.serve_forever,kwargs={'poll_interval':.02},daemon=True);worker.start()
    connection=HTTPConnection('127.0.0.1',server.server_port,timeout=2)
    try:
        headers={'Content-Type':'application/json','X-Drive-Key':server.drive_key}
        connection.request('POST','/api/drive/race_page_heartbeat',json.dumps(heartbeat()),headers)
        response=connection.getresponse();data=response.read()
        assert response.status==200 and len(data)<300 and json.loads(data)['heartbeat_ack']
        headers['X-Drive-Key']='wrong'
        connection.request('POST','/api/drive/race_page_heartbeat',json.dumps(heartbeat(1)),headers)
        response=connection.getresponse();response.read()
        assert response.status==403 and flow.heartbeat_count==1
    finally:
        connection.close();camera.finish();server.shutdown();server.server_close();worker.join(2)


def test_production_browser_handles_request_reordering_and_network_failures():
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node required for production browser tests')
    result=subprocess.run([node,str(ROOT/'vision/tests/js/competition_network.cjs'),str(ROOT/'vision/web/competition/app.js')],
        input=json.dumps({'html':(ROOT/'vision/web/competition/index.html').read_text(encoding='utf-8')}),
        text=True,encoding='utf-8',capture_output=True,timeout=20)
    assert result.returncode==0,result.stdout+result.stderr
    assert json.loads(result.stdout)['hardware_output'] is False
