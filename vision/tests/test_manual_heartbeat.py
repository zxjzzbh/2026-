import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest
from carvision.manual_controls import SCRIPT


def test_delayed_reply_sends_due_heartbeat_without_replaying_released_keys():
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:
        pytest.skip('Node.js is required for delayed browser-response checks')
    result=subprocess.run([node,str(Path(__file__).parent/'js/manual_heartbeat.cjs')],
                          input=json.dumps({'dashboard':SCRIPT}),capture_output=True,
                          text=True,encoding='utf-8',timeout=15)
    assert result.returncode==0,result.stdout+result.stderr
    record=json.loads(result.stdout)
    assert record['checks_passed']==24 and record['hardware_output'] is False
