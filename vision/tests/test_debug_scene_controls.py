"""Debug scene selection and bounded-recovery presentation contracts."""
import copy
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
from types import SimpleNamespace

import pytest

from carvision.cone_controls import HTML, SCRIPT, STATE
from carvision.parking_bay_controls import HTML as PARKING_HTML
from carvision.cone_observer import ConeObserverDrive


ROOT=Path(__file__).resolve().parents[2]


def test_blue_track_can_be_saved_without_motion_or_calibration_changes(tmp_path):
    config=tmp_path/'cone-trial.json'
    config.write_bytes((ROOT/'vision/configs/cone-trial.json').read_bytes())
    calls=[]
    base=SimpleNamespace(status=lambda:{'mode':'disabled','boot_test_phase':'esc_off'},
                         request=lambda *args:calls.append(args))
    drive=ConeObserverDrive(base,{},config,tmp_path)
    before=copy.deepcopy(drive.settings)
    drive.request('cone_settings',{'scene_surface':'blue_running_track'})
    assert drive.settings['scene']['surface']=='blue_running_track'
    for key in ('geometry','ground','motion','driving'):
        assert drive.settings[key]==before[key]
    assert calls==[]
    assert ConeObserverDrive(base,{},config,tmp_path).settings['scene']['surface']=='blue_running_track'
    with pytest.raises(ValueError):drive.request('cone_settings',{'scene_surface':'ignore_boundaries'})


def test_debug_scene_and_parking_modes_are_clear():
    assert '<option value="blue_running_track">蓝色跑道（红桶）</option>' in HTML
    options=re.search(r'<select id="parking-trial-controller">(.*?)</select>',PARKING_HTML).group(1)
    assert options.startswith('<option value="metric_path">')
    assert '<option value="visual_trial">历史视觉诊断（不计正式入位）</option>' in options


def test_debug_render_reports_bounded_recovery_and_no_route_stop():
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node is required for real client rendering')
    harness=r'''
const assert=require('node:assert/strict'),vm=require('node:vm'),nodes=new Map();
const env={stale:false,client:'page',held:new Map(),gamepadControl:null,latency:0,
 document:{querySelectorAll(){return[]}},el(id){if(!nodes.has(id))nodes.set(id,{dataset:{},value:'',textContent:''});return nodes.get(id)}};
vm.createContext(env);vm.runInContext(require('node:fs').readFileSync(0,'utf8'),env);
const state={cone_trial:{settings:{scene:{surface:'blue_running_track'}},
 avoidance:{profile:'legacy_low_speed',low_speed_test:{best_effort_route:true}},
 latest:{navigation:{experimental:true,line_sources:[],line_missing:true}}}};
env.renderCone(state);
assert.equal(env.el('cone-surface').value,'blue_running_track');
assert.match(env.el('cone-test-line-state').textContent,/总时限1秒.*停止/);
assert.match(env.el('cone-test-progress').textContent,/间隙未明确，按白线内目标继续/);
assert(!env.el('cone-test-progress').textContent.includes('尝试避让'));
state.cone_trial.latest.navigation={experimental:true,line_sources:[],line_held:true};env.renderCone(state);
assert.match(env.el('cone-test-line-state').textContent,/单边预测最多0.25秒.*总时限1秒/);
'''
    result=subprocess.run([node,'-e',harness],input=STATE+SCRIPT.split('async function coneAction')[0],
        encoding='utf-8',text=True,capture_output=True,timeout=10)
    assert result.returncode==0,result.stdout+result.stderr
