"""Standalone debug resources and real-client request contracts."""
import json
import os
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import pytest

from carvision.debug_page import add_page, build_controls

ROOT=Path(__file__).resolve().parents[2]


def test_debug_page_uses_static_resources_and_direct_server_bootstrap():
    class Original:
        def do_GET(self):pytest.fail('debug route cannot fall back to the old composed page')
        def send_body(self,data,mime,status=200):self.output=(data,mime,status)
    server=SimpleNamespace(RequestHandlerClass=Original,drive_key='debug-key',drive_session='debug-session')
    add_page(server,root=ROOT)
    h=server.RequestHandlerClass();h.path='/';h.do_GET()
    html,mime,status=h.output
    assert status==200 and mime.startswith('text/html')
    assert b'debug-key' in html and b'/debug/assets/app.js' in html
    assert b'crosswalk-test' not in html and b'/manual' in html
    h.path='/debug/assets/app.js';h.do_GET()
    assert b'createDebugClient' in h.output[0] and h.output[1].startswith('text/javascript')


def test_debug_client_validates_real_confirmation_and_task_payloads():
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node is required for offline debug client checks')
    controls=build_controls(root=ROOT)
    result=subprocess.run([node,str(ROOT/'vision/tests/js/debug_static.cjs'),str(ROOT/'vision/web/debug/app.js')],
        input=json.dumps({'html':controls.HTML}),text=True,encoding='utf-8',capture_output=True,timeout=20)
    assert result.returncode==0,result.stdout+result.stderr
    assert json.loads(result.stdout)['hardware_output'] is False
