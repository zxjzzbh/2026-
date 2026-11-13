"""Remote-supervision lease; all output is fake and HTTP stays on loopback."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
from http.client import HTTPConnection
from types import SimpleNamespace

import pytest

from carvision.application import DebugApplication
from carvision.cone_execution import ConeRun,CONE_PAGE_LEASE_S
from carvision.web_preview import PreviewState,make_server

ROOT=Path(__file__).resolve().parents[2]


def run():
    value=ConeRun();value.active=True;value.phase='ready';value.owner='page';value.token='test-token'
    value.lease_until=CONE_PAGE_LEASE_S;value.last_heartbeat_s=0.
    return value


def body(seq=0,**changes):
    return {'client':'page','run_token':'test-token','heartbeat_sequence':seq,**changes}


def test_1740ms_network_delay_is_tolerated_without_changing_actuator_commands():
    value=run();reply=value.heartbeat(body(),1.74)
    assert reply['accepted'] and reply['page_lease_s']==3.
    assert value.lease_until==pytest.approx(4.74) and value.active and value.actuator is None
    assert value.sequence==-1 and value.heartbeat_count==1


def test_out_of_order_duplicate_and_foreign_heartbeats_never_extend_ownership():
    value=run();value.heartbeat(body(2),.3);deadline=value.lease_until
    assert value.heartbeat(body(1),.4)['accepted'] is False
    assert value.heartbeat(body(2),.5)['accepted'] is False
    assert value.lease_until==deadline
    for bad in (body(3,client='other'),body(3,run_token='old'),body(-1),body(True)):
        with pytest.raises(ValueError):value.heartbeat(bad,.6)
        assert value.active and value.lease_until==deadline


def test_three_seconds_without_valid_heartbeat_stops_and_late_packets_cannot_revive():
    value=run()
    with pytest.raises(ValueError,match='3 秒'):value.heartbeat(body(),3.001)
    assert not value.active and value.phase=='fault' and value.first_stop['source']=='page_lease'
    with pytest.raises(ValueError):value.heartbeat(body(1),3.01)
    assert not value.active


def test_start_command_sequence_is_independent_from_heartbeat_sequence():
    value=run();value.sequence=8
    value.heartbeat(body(500),.2)
    assert value.sequence==8
    value.command('cone_keepalive',{'client':'page','run_token':value.token,'trial_sequence':9},.3)
    assert value.sequence==9 and value.heartbeat_sequence==500
    value.heartbeat(body(501),.4);assert value.sequence==9


def test_lightweight_http_path_bypasses_full_status_and_uses_real_owner_validation():
    value=run();app=object.__new__(DebugApplication);app.closed=False
    app.tasks={'cones':SimpleNamespace(page_heartbeat=lambda payload:value.heartbeat(payload,.4))}
    app.status=lambda:pytest.fail('heartbeat must not gather full task/camera status')
    state=PreviewState();server=make_server(('127.0.0.1',0),state,drive=app,build_controls=False)
    worker=threading.Thread(target=server.serve_forever,kwargs={'poll_interval':.02},daemon=True);worker.start()
    connection=HTTPConnection('127.0.0.1',server.server_port,timeout=2)
    try:
        headers={'Content-Type':'application/json','X-Drive-Key':server.drive_key}
        connection.request('POST','/api/drive/cone_page_heartbeat',json.dumps(body()),headers)
        response=connection.getresponse();raw=response.read()
        assert response.status==200 and len(raw)<300
        ack=json.loads(raw);assert ack['heartbeat_ack'] and ack['heartbeat_sequence']==0
        assert value.lease_until==3.4
        value.cancel('explicit operator stop',now=.5)
        with pytest.raises(ValueError):value.heartbeat(body(1),.6)
    finally:
        connection.close();state.finish();server.shutdown();server.server_close();worker.join(timeout=2)


def test_browser_heartbeat_survives_slow_status_and_reply_reordering():
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node required for real browser-client logic')
    result=subprocess.run([node,str(ROOT/'vision/tests/js/debug_heartbeat.cjs'),str(ROOT/'vision/web/debug/app.js')],
                          input=json.dumps({'html':(ROOT/'vision/web/debug/index.html').read_text(encoding='utf-8')}),
                          text=True,encoding='utf-8',capture_output=True,timeout=15)
    assert result.returncode==0,result.stdout+result.stderr
    assert json.loads(result.stdout)['hardware_output'] is False
