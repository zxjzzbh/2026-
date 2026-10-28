"""Independent competition resources and asynchronous client contracts."""
import json
import os
import re
import shutil
import subprocess
import threading
from pathlib import Path
from types import SimpleNamespace
from urllib.request import urlopen

import pytest
from carvision import manual_controls
from carvision.competition_page import build_controls, render_page, add_page
from carvision.web_preview import make_server

ROOT=Path(__file__).resolve().parents[2]


def test_independent_resources_do_not_compose_legacy_controls():
    before=(manual_controls.HTML,manual_controls.SCRIPT,manual_controls.STYLE)
    controls=build_controls(manual_controls)
    page=render_page('const key="test-key", pageSession="session";',controls)
    assert before==(manual_controls.HTML,manual_controls.SCRIPT,manual_controls.STYLE)
    for marker in ('crosswalk-test','traffic-test','cone-test','parking-bay-test','departure-test','__DRIVE_KEY__'):
        assert marker not in page
    assert '/competition/assets/app.js' in page and '/competition/assets/style.css' in page
    bootstrap=re.search(r'<script id="control-bootstrap" type="application/json">(.*?)</script>',page).group(1)
    assert json.loads(bootstrap)=={'key':'test-key','session':'session'}
    assert 'function render' not in page
    ids=re.findall(r'\bid="([^"]+)"',page)
    assert len(ids)==len(set(ids))
    with pytest.raises(ValueError):render_page('missing bootstrap',controls)


def test_page_and_static_routes_share_auth_without_control_requests():
    calls=[]
    drive=SimpleNamespace(status=lambda:{'mode':'disabled'},request=lambda action,payload:calls.append(action) or {'mode':'disabled'})
    camera=SimpleNamespace(status=lambda:{'state':'starting'})
    server=add_page(make_server(('127.0.0.1',0),camera,camera,drive),root=ROOT,at_root=True)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    url='http://127.0.0.1:'+str(server.server_port)
    try:
        page=urlopen(url+'/').read().decode()
        assert '比赛操作台' in page and 'crosswalk-test' not in page
        assert urlopen(url+'/competition/').read().decode()==page
        js=urlopen(url+'/competition/assets/app.js')
        assert js.headers['Content-Type'].startswith('text/javascript')
        assert b'createCompetitionClient' in js.read()
        assert urlopen(url+'/competition/assets/style.css').headers['Content-Type'].startswith('text/css')
        assert not calls
    finally:server.shutdown();server.server_close();thread.join(timeout=2)


def test_explicit_server_credentials_need_no_legacy_page_or_script():
    class Original:
        def do_GET(self):pytest.fail('independent race route must not build or read the debug page')
        def send_body(self,content,mime,status=200):self.result=(content,mime,status)
    server=SimpleNamespace(RequestHandlerClass=Original,drive_key='direct-key',drive_session='direct-session')
    add_page(server,root=ROOT,at_root=True)
    handler=server.RequestHandlerClass();handler.path='/'
    handler.do_GET()
    content,mime,status=handler.result
    assert status==200 and mime.startswith('text/html')
    assert b'direct-key' in content and b'direct-session' in content
    assert b'crosswalk-test' not in content


def test_standalone_client_ownership_queue_handoff_and_manual_api():
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node is required for offline asynchronous browser contracts')
    controls=build_controls()
    result=subprocess.run([node,str(ROOT/'vision/tests/js/competition_static.cjs'),str(ROOT/'vision/web/competition/app.js')],
        input=json.dumps({'html':controls.HTML}),text=True,encoding='utf-8',capture_output=True,timeout=20)
    assert result.returncode==0,result.stdout+result.stderr
    assert json.loads(result.stdout)['hardware_output'] is False


def test_debug_page_background_events_stop_only_its_current_owner():
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node is required for the debug page ownership contract')
    source=manual_controls.SCRIPT
    start=source.index('function ownsPageControl(){')
    end=source.index("window.addEventListener('blur'",start)
    harness="""
const assert=require('node:assert/strict'),vm=require('node:vm');
const calls=[];
const env={client:'page',lastStatus:{race_flow:{active:true,owner_client:'other'}},pageManualOwned:false,pageManualTrialToken:null,calls,stop(...args){calls.push(args)}};
vm.createContext(env);vm.runInContext(require('node:fs').readFileSync(0,'utf8'),env);
env.stopForPageEvent('window_blur');assert.equal(env.calls.length,0);
env.lastStatus={race_flow:{active:true,owner_client:'page'}};env.stopForPageEvent('window_blur');assert.equal(env.calls.length,1);
env.lastStatus={trial_token:'new'};env.pageManualOwned=true;env.pageManualTrialToken='old';env.stopForPageEvent('page_hidden');assert.equal(env.calls.length,1);
env.lastStatus={trial_token:'old'};env.stopForPageEvent('page_hidden');assert.equal(env.calls.length,2);
"""
    result=subprocess.run([node,'-e',harness],input=source[start:end],text=True,encoding='utf-8',capture_output=True,timeout=10)
    assert result.returncode==0,result.stdout+result.stderr
