"""Real cone settings adapter and production JS, using no camera/hardware I/O."""
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
from types import SimpleNamespace

import pytest

from carvision.cone_observer import ConeObserverDrive


ROOT=Path(__file__).resolve().parents[2]
META={'client':'page','setup_token':'setup','trial_token':'trial'}


@pytest.fixture
def observer(tmp_path,monkeypatch):
    source=ROOT/'vision/tests/fixtures/configs/cone-trial-field-20261026.json'
    if not source.exists():source=ROOT/'vision/configs/cone-trial.json'
    settings=json.loads(source.read_text(encoding='utf-8-sig'))
    settings.setdefault('scene',{}).update(surface='indoor_white_line_track',white_v_min=255)
    config=tmp_path/'cone-trial.json'
    config.write_text(json.dumps(settings),encoding='utf-8')
    def forbidden(*args,**kwargs):pytest.fail('Settings test must never request movement or start hardware')
    base=SimpleNamespace(status=lambda:{'mode':'disabled','boot_test_phase':'esc_off','hardware_output':False},
                         request=forbidden)
    drive=ConeObserverDrive(base,{},config,tmp_path,clock=lambda:10.,
                            actuator_factory=forbidden,short_actuator_factory=forbidden)
    monkeypatch.setattr(drive,'_start_observer',forbidden)
    drive.observing=True
    drive.latest={'sentinel':'old-scene'};drive.preview=b'old-preview';drive.updated=9.9
    writes=[];original_save=drive._save
    def save(candidate):
        writes.append(copy.deepcopy(candidate));return original_save(candidate)
    monkeypatch.setattr(drive,'_save',save)
    return drive,writes


def unchanged(drive,before):
    assert drive.settings==before and not drive.local_config.exists()
    assert drive.observing and not drive.reset_reader
    assert drive.latest=={'sentinel':'old-scene'} and drive.preview==b'old-preview' and drive.updated==9.9
    assert not drive.run.active and not drive.short.active and drive.thread is None


def test_readonly_same_scene_and_protocol_metadata_are_a_true_noop(observer):
    drive,writes=observer;before=copy.deepcopy(drive.settings)
    result=drive.request('cone_settings',{**META,'scene_surface':'indoor_white_line_track'})
    assert result['cone_trial']['settings']==before and writes==[]
    unchanged(drive,before)


def test_readonly_scene_change_persists_and_invalidates_old_reader_evidence(observer):
    drive,writes=observer;before=copy.deepcopy(drive.settings)
    result=drive.request('cone_settings',{**META,'scene_surface':'blue_running_track'})
    assert len(writes)==1 and drive.local_config.exists()
    expected=copy.deepcopy(before);expected['scene']['surface']='blue_running_track'
    assert drive.settings==expected and result['cone_trial']['settings']==expected
    assert not set(META)&set(drive.settings)
    assert drive.reset_reader and drive.latest is drive.preview is drive.updated is None
    assert drive.observing and not drive.run.active and not drive.short.active and drive.thread is None


@pytest.mark.parametrize('surface',['ignore_boundaries',None,True,[]])
def test_invalid_scene_does_not_write_or_invalidate_readonly_observer(observer,surface):
    drive,writes=observer;before=copy.deepcopy(drive.settings)
    with pytest.raises(ValueError):drive.request('cone_settings',{**META,'scene_surface':surface})
    assert writes==[];unchanged(drive,before)


@pytest.mark.parametrize('extra',[
    {'geometry_cm':{'lane_width':123}}, {'blue_hsv_lower':[100,110,50]},
    {'blue_hsv_upper':[130,255,255]}, {'confirm_frames':3},
    {'motion':{'braking_distance':25}}, {'driving':{'steering_gain':1.5}},
    {'red_ground_percent':15},
])
def test_other_settings_still_require_stopping_observation_and_are_atomic(observer,extra):
    drive,writes=observer;before=copy.deepcopy(drive.settings)
    with pytest.raises(ValueError):
        drive.request('cone_settings',{**META,'scene_surface':'blue_running_track',**extra})
    assert writes==[];unchanged(drive,before)


@pytest.mark.parametrize('active',['run','short'])
@pytest.mark.parametrize('scene',['indoor_white_line_track','blue_running_track'])
def test_any_active_cone_task_rejects_even_scene_only_noop(observer,active,scene):
    drive,writes=observer;before=copy.deepcopy(drive.settings)
    getattr(drive,active).active=True
    with pytest.raises(ValueError):drive.request('cone_settings',{**META,'scene_surface':scene})
    assert drive.settings==before and writes==[] and not drive.reset_reader
    assert drive.latest=={'sentinel':'old-scene'}
    with pytest.raises(ValueError):drive.request('cone_white_threshold',{**META,'white_v_min':160})
    assert writes==[]


def test_readonly_white_threshold_keeps_existing_safe_save_path(observer):
    drive,writes=observer;before=copy.deepcopy(drive.settings)
    drive.request('cone_white_threshold',{**META,'white_v_min':160})
    expected=copy.deepcopy(before);expected['scene']['white_v_min']=160
    assert drive.settings==expected and len(writes)==1
    assert drive.reset_reader and drive.latest is drive.preview is drive.updated is None
    assert drive.observing and not drive.run.active and not drive.short.active


def test_real_debug_save_crosses_real_readonly_backend_and_clears_dirty_without_preparing(observer,tmp_path):
    node=os.environ.get('SMARTCAR_NODE') or shutil.which('node')
    if not node:pytest.skip('Node is required for the production debug client check')
    drive,writes=observer
    # Reuse the repository's fake DOM verbatim. Only fetch is replaced by a
    # line protocol that dispatches to this actual ConeObserverDrive instance.
    harness=(ROOT/'vision/tests/js/debug_static.cjs').read_text(encoding='utf-8')
    fake_dom=harness[harness.index('function element('):harness.index('let rev=0')]
    source=r'''
const fs=require('node:fs'),assert=require('node:assert/strict'),readline=require('node:readline');
const {createDebugClient}=require(process.argv[2]);
const input=JSON.parse(fs.readFileSync(process.argv[3],'utf8')),nodes=new Map(),events={},documentEvents={},requests=[];
__FAKE_DOM__
let serial=0;const waiting=new Map(),lines=readline.createInterface({input:process.stdin});
lines.on('line',line=>{const reply=JSON.parse(line);waiting.get(reply.id)(reply);waiting.delete(reply.id);});
const fetch=async(url,options={})=>{
  if(options.method!=='POST')return {ok:true,json:async()=>structuredClone(input.status)};
  const action=url.split('/').at(-1),body=JSON.parse(options.body);requests.push({action,body});
  assert(['cone_settings','cone_white_threshold'].includes(action),'no prepare/start/stop/control POST');
  const id=++serial,result=await new Promise(resolve=>{
    waiting.set(id,resolve);process.stdout.write(JSON.stringify({type:'request',id,action,body})+'\n');
  });
  return {ok:result.ok,json:async()=>result.body};
};
const api=createDebugClient({document,window,fetch,bootstrap:{key:'key',session:'session'},clientId:'page',
  setTimeout:()=>1,clearTimeout(){},setInterval(){}});
const flush=async()=>{for(let i=0;i<5;i++)await new Promise(setImmediate)};
(async()=>{
  api.init();await flush();assert.equal(requests.length,0);assert(api.choose('cone'));
  for(const id of ['confirm-esc-off','confirm-fb'])nodes.get(id).checked=true;
  nodes.get('field-white_v_min').value='160';await nodes.get('field-white_v_min').fire('input');
  assert.equal(nodes.get('field-test_steering_gain').value,'1.5');
  assert(api.inspect().dirty&&nodes.get('prepare-task').disabled);
  await api.save();
  assert.deepEqual(requests.map(row=>row.action),['cone_settings','cone_white_threshold']);
  assert.equal(requests[0].body.scene_surface,'indoor_white_line_track');
  assert.equal(requests[1].body.white_v_min,160);
  assert(!api.inspect().dirty&&!nodes.get('prepare-task').disabled);
  assert(!nodes.get('save-settings').disabled&&!nodes.get('message').textContent);
  assert(nodes.get('observe-task').disabled&&nodes.get('stop-task').disabled,
    'save neither acquires nor cancels an existing readonly observer');
  assert.equal(api.inspect().status.cone_trial.settings.scene.white_v_min,160);
  process.stdout.write(JSON.stringify({type:'result',posts:requests,dirty:api.inspect().dirty,
    prepare_disabled:nodes.get('prepare-task').disabled,hardware_output:false})+'\n');
  lines.close();
})().catch(error=>{console.error(error);process.exitCode=1;lines.close();});
'''.replace('__FAKE_DOM__',fake_dom)
    script=tmp_path/'debug-save-client.cjs';script.write_text(source,encoding='utf-8')
    def status():
        return {**drive.status(),'application_mode':'debug','control_session_id':'session',
                'setup_token':'setup','trial_token':'trial'}
    client_input=tmp_path/'client-input.json'
    client_input.write_text(json.dumps({'html':(ROOT/'vision/web/debug/index.html').read_text(encoding='utf-8'),
                                       'status':status()}),encoding='utf-8')
    process=subprocess.Popen([node,str(script),str(ROOT/'vision/web/debug/app.js'),str(client_input)],
                              stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                              text=True,encoding='utf-8')
    timeout=threading.Timer(15,process.kill);timeout.start();result=None;calls=[]
    try:
        for line in process.stdout:
            message=json.loads(line)
            if message['type']=='result':result=message;continue
            assert message['type']=='request' and message['action'] in ('cone_settings','cone_white_threshold')
            calls.append(message)
            try:
                drive.request(message['action'],message['body']);reply={'ok':True,'body':status()}
            except ValueError as exc:reply={'ok':False,'body':{'error':str(exc)}}
            process.stdin.write(json.dumps({'id':message['id'],**reply})+'\n');process.stdin.flush()
        code=process.wait(timeout=2);stderr=process.stderr.read()
        assert code==0,stderr
    finally:
        timeout.cancel()
        if process.poll() is None:process.kill();process.wait(timeout=2)
        for stream in (process.stdin,process.stdout,process.stderr):stream.close()
    assert result and result['dirty'] is False and result['prepare_disabled'] is False
    assert [row['action'] for row in calls]==['cone_settings','cone_white_threshold']
    assert len(writes)==1  # Same-scene save is a no-op; only white writes.
    assert drive.settings['scene']['white_v_min']==160 and drive.observing
    assert not drive.run.active and not drive.short.active and drive.thread is None
