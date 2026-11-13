"""Parking supervision faults use fake outputs and loopback HTTP only."""
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
from carvision.parking_entry_run import PAGE_LEASE_S
from carvision.web_preview import PreviewState,make_server
from test_parking_entry_run import prepared,base,command,sample

ROOT=Path(__file__).resolve().parents[2]


def body(run,seq=0,**changes):
    return {'client':'page','run_token':run.token,'heartbeat_sequence':seq,**changes}


def test_delay_tolerance_does_not_relax_actuator_or_freshness():
    run,actuator=prepared()
    ack=run.heartbeat(body(run),1.74)
    assert ack['accepted'] and PAGE_LEASE_S==3.
    assert run.lease_until==pytest.approx(4.74)
    assert run.s['brake']['actuator_lease_s']==.5
    assert run.s['brake']['max_frame_age_s']==.5
    assert all(c['esc_us']==1500 for c in actuator.commands)
    run.tick(sample(1.74),2.25)
    assert not run.active and actuator.closed and '画面' in run.reason


def test_duplicate_foreign_and_out_of_order_heartbeats_do_not_renew():
    run,_=prepared();run.heartbeat(body(run,20),.2);deadline=run.lease_until
    for seq in (19,20):assert not run.heartbeat(body(run,seq),.3)['accepted']
    for bad in (body(run,21,client='foreign'),body(run,21,run_token='old'),body(run,-1),body(run,True)):
        with pytest.raises(ValueError):run.heartbeat(bad,.4)
    assert run.lease_until==deadline and run.sequence==-1
    run.command('parking_trial_keepalive',command(run,0),.5)
    run.heartbeat(body(run,21),.6)
    assert run.sequence==0 and run.heartbeat_sequence==21


def test_control_request_queued_before_heartbeat_cannot_shorten_lease():
    run,_=prepared();run.heartbeat(body(run),1.2)
    run.command('parking_trial_keepalive',command(run,0),.4)
    assert run.lease_until==4.2 and run.last_heartbeat_s==1.2


def test_missing_heartbeat_stops_and_summary_records_actual_receive_gap(tmp_path):
    run,actuator=prepared(tmp_path);payload=body(run)
    run.heartbeat(payload,.4)
    run.tick(sample(3.401),3.401)
    assert not run.active and actuator.closed and run.phase=='fault'
    summary=json.loads((run.folder/'summary.json').read_text(encoding='utf8'))
    assert summary['page_heartbeat']['last_received_s']==.4
    assert summary['page_heartbeat']['age_s']==pytest.approx(3.001)
    with pytest.raises(ValueError):run.heartbeat(payload,3.402)


def test_heartbeat_cannot_revive_an_expired_or_explicitly_cancelled_run():
    run,actuator=prepared();payload=body(run)
    with pytest.raises(ValueError,match='3 秒'):run.heartbeat(payload,3.001)
    assert actuator.closed and not run.active
    run,actuator=prepared();payload=body(run);run.cancel('operator stop')
    with pytest.raises(ValueError):run.heartbeat(payload,.2)
    assert actuator.closed and not run.active


def test_http_heartbeat_bypasses_status_and_global_queue():
    run,_=prepared();app=object.__new__(DebugApplication);app.closed=False
    app.tasks={'parking':SimpleNamespace(page_heartbeat=lambda payload:run.heartbeat(payload,.4))}
    app.status=lambda:pytest.fail('heartbeat must not collect full status')
    # No application lock is installed: entering the old serialized path fails.
    state=PreviewState();server=make_server(('127.0.0.1',0),state,drive=app,build_controls=False)
    worker=threading.Thread(target=server.serve_forever,kwargs={'poll_interval':.02},daemon=True);worker.start()
    connection=HTTPConnection('127.0.0.1',server.server_port,timeout=2)
    try:
        headers={'Content-Type':'application/json','X-Drive-Key':server.drive_key}
        connection.request('POST','/api/drive/parking_page_heartbeat',json.dumps(body(run)),headers)
        response=connection.getresponse();raw=response.read()
        assert response.status==200 and len(raw)<300 and json.loads(raw)['heartbeat_ack']
        assert run.lease_until==3.4
    finally:
        connection.close();state.finish();server.shutdown();server.server_close();worker.join(timeout=2)
        run.cancel('test ended')


@pytest.mark.parametrize('mode',['parking','visual'])
def test_actual_browser_parking_heartbeat_survives_slow_status(mode):
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node required for actual browser client logic')
    result=subprocess.run([node,str(ROOT/'vision/tests/js/parking_heartbeat.cjs'),str(ROOT/'vision/web/debug/app.js')],
        input=json.dumps({'html':(ROOT/'vision/web/debug/index.html').read_text(encoding='utf8'),'mode':mode}),
        text=True,encoding='utf8',capture_output=True,timeout=15)
    assert result.returncode==0,result.stdout+result.stderr
    assert json.loads(result.stdout)['hardware_output'] is False
