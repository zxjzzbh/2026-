"""Render the shipped client offline; current light and vote history differ."""
import json
import os
import shutil
import subprocess

import pytest

from carvision.traffic_controls import SCRIPT


@pytest.mark.parametrize('current,confirmed,fresh,use_vote_raw', [
    ('red', False, True, True),
    ('yellow', False, True, True),
    ('off', False, True, True),
    ('unknown', False, True, True),
    ('green', False, True, True),
    ('green', True, True, True),
    ('red', False, True, False),
    ('green', True, False, True),
])
def test_current_signal_is_not_replaced_by_historical_green(current, confirmed, fresh, use_vote_raw):
    node = os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:
        pytest.skip('Node.js is needed to execute the real client render function')
    harness = r'''
const vm=require('node:vm'),assert=require('node:assert/strict');
const input=JSON.parse(require('node:fs').readFileSync(0,'utf8'));
const nodes=new Map();
const env={trafficActive:false,trafficSaveBusy:false,trafficStartBusy:false,trafficStartError:'',
 trafficDirty:false,stale:false,client:'page',held:new Map(),gamepadControl:null,latency:0,
 feedback(){},document:{querySelectorAll(){return [];}},
 el(id){if(!nodes.has(id))nodes.set(id,{dataset:{},value:'',textContent:''});return nodes.get(id);}};
vm.createContext(env);vm.runInContext(input.script,env);
const row={valid:true,red_frames:1,total_frames:10,red_percent:10,ready:true,collected_s:1,
 green_frames:15,green_total_frames:20,green_percent:75,green_ready:true,green_collected_s:2};
// Deliberately retain a historical green state and stale stable summary.
const votes={state:'green',green_confirmed:input.confirmed,window_s:1,green_window_s:2,
 threshold_percent:50,cameras:{secondary:row}};
if(input.use_vote_raw)votes.raw_state=input.current;
const cfg={pwm:{search_us:1570},red_window_s:1,green_window_s:2,red_confirm_percent:50,
 brake_pulse_s:.35,max_approach_s:10};
env.renderTraffic({traffic_trial:{active:true,owner_client:'page',settings:cfg,phase:'wait_green',
 frame_age_s:input.fresh?0:1,observation:{signal:{state:input.use_vote_raw?'green':input.current},
 camera_observations:{secondary:{frame_id:1,signal:{state:input.current}}}},
 intent:{phase:'wait_green',action:'neutral',esc_us:1500,signal_votes:votes,
 stable:{state:'green',green_confirmed:true}}}});
const badge=env.el('traffic-color'),detail=env.el('traffic-red-vote').textContent;
assert.equal(badge.dataset.signal,input.fresh?input.current:'unknown');
assert.match(detail,/历史窗口投票/);
assert.match(detail,/红 1\/10.*绿 15\/20/);
const ready=input.fresh&&input.confirmed;
assert.equal(env.el('traffic-intent').textContent.includes('当前绿灯条件已确认'),ready);
if(!ready){
 assert.match(detail,/保持停止/);
 assert.match(detail,/历史绿票不能单独授权通行/);
 assert(!badge.textContent.includes('绿灯 · 已确认'));
}else{
 assert.equal(badge.textContent,'当前绿灯 · 已确认');
 assert(!detail.includes('保持停止'));
}
if(input.fresh&&input.current==='green'&&!input.confirmed)assert.match(badge.textContent,/待确认/);
console.log(JSON.stringify({badge:badge.textContent,hardware_output:false}));
'''
    result = subprocess.run([node, '-e', harness], input=json.dumps({
        'script': SCRIPT.split('async function trafficSend')[0],
        'current': current, 'confirmed': confirmed, 'fresh': fresh, 'use_vote_raw': use_vote_raw,
    }), encoding='utf-8', text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
