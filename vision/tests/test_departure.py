import json
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.departure import PHRASE,DepartureSequence,detect_board
from carvision.departure_trial import DepartureTrialDrive
from carvision.departure_controls import extend_controls


def scene():
    rng=np.random.default_rng(5)
    return rng.integers(60,190,(360,480,1),dtype=np.uint8).repeat(3,axis=2)


def rows(t,i,primary='present',secondary='absent'):
    return {c:{'state':v,'frame_id':i,'captured_s':t,'boxes':[],'camera_id':c}
            for c,v in [('primary',primary),('secondary',secondary)]}


def arm(s):
    for i in range(4):assert not s.update(rows(i*.2,i),i*.2)
    assert s.phase=='wait_removed'


def test_rectangle_and_near_clipped_panel_are_present_but_small_cone_is_not():
    image=scene();cv2.rectangle(image,(170,100),(290,250),(255,0,0),-1)
    result=detect_board(image);assert result['state']=='present'
    assert result['dimensions_cm']=={'width':40,'height':50} and not result['metric_distance_available']
    json.dumps(result,allow_nan=False)
    assert detect_board(np.full((360,480,3),(255,0,0),np.uint8))['state']=='present'
    image=scene();cv2.fillPoly(image,[np.array([[350,310],[340,339],[360,339]])],(255,0,0))
    assert detect_board(image)['state']=='absent'


def test_real_historical_close_board_is_recognized():
    image=cv2.imdecode(np.frombuffer((Path(__file__).parent/'fixtures/blue-board-close.jpg').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    assert detect_board(image)['state']=='present'


@pytest.mark.parametrize('value',[0,15,255])
def test_dark_or_covered_or_overexposed_camera_is_unknown(value):
    assert detect_board(np.full((360,480,3),value,np.uint8))['state']=='unknown'


def test_empty_scene_never_triggers_without_first_seeing_board():
    s=DepartureSequence()
    for i in range(40):assert not s.update(rows(i*.2,i,'absent','absent'),i*.2)
    assert s.phase=='wait_board'


def test_confirmed_board_then_both_clear_triggers_once():
    s=DepartureSequence();arm(s);events=[]
    for i in range(4,20):events.append(s.update(rows(i*.2,i,'absent','absent'),i*.2))
    assert sum(events)==1 and s.phase=='complete'


def test_secondary_alone_can_establish_presence_and_blocks_removal():
    s=DepartureSequence()
    for i in range(15):assert not s.update(rows(i*.2,i,'absent','present'),i*.2)
    assert s.phase=='wait_removed' and s.seen_cameras==['secondary']


def test_brief_miss_and_reappearance_restart_clear_count():
    s=DepartureSequence();arm(s)
    for i in range(4,7):assert not s.update(rows(i*.2,i,'absent','absent'),i*.2)
    assert not s.update(rows(1.4,7),1.4) and s.clear_elapsed==0
    for i in range(8,12):assert not s.update(rows(i*.2,i,'absent','absent'),i*.2)
    assert s.phase=='wait_removed'


def test_moving_board_shape_unknown_keeps_confirmed_presence_but_does_not_count_as_clear():
    s=DepartureSequence();arm(s)
    for i in range(4,9):
        moving=rows(i*.2,i,'present','unknown')
        moving['secondary']['scene_valid']=True
        assert not s.update(moving,i*.2)
        assert s.phase=='wait_removed' and s.clear_elapsed==0
    fired=[]
    for i in range(9,18):fired.append(s.update(rows(i*.2,i,'absent','absent'),i*.2))
    assert sum(fired)==1


def test_shape_unknown_before_confirming_board_does_not_arm():
    s=DepartureSequence()
    for i in range(10):
        moving=rows(i*.2,i,'unknown','unknown')
        for row in moving.values():row['scene_valid']=True
        assert not s.update(moving,i*.2) and s.phase=='wait_board'
    for i in range(10,30):assert not s.update(rows(i*.2,i,'absent','absent'),i*.2)


def test_one_clear_board_view_can_confirm_presence_while_other_has_ambiguous_shape():
    s=DepartureSequence()
    for i in range(5):
        r=rows(i*.2,i,'unknown','present');r['primary']['scene_valid']=True
        assert not s.update(r,i*.2)
    assert s.phase=='wait_removed' and s.seen_cameras==['secondary']


def test_tunable_area_threshold_rejects_small_blue_panel_at_higher_setting():
    image=scene();cv2.rectangle(image,(210,160),(260,240),(255,0,0),-1)
    assert detect_board(image,min_blue_percent=1.5)['state']=='present'
    assert detect_board(image,min_blue_percent=3.)['state']=='absent'
    assert detect_board(image)['min_blue_percent']==3.


@pytest.mark.parametrize('area',[True,0,.4,31,float('nan'),'3'])
def test_area_threshold_validation(area):
    with pytest.raises(ValueError):detect_board(scene(),min_blue_percent=area)


def test_repeated_cached_frames_cannot_confirm_removal():
    s=DepartureSequence();arm(s)
    one=rows(.8,4,'absent','absent')
    for now in np.arange(.8,3.,.1):assert not s.update(one,float(now))
    assert s.phase=='wait_board' # Old cache eventually becomes invalid, requiring the board again.


@pytest.mark.parametrize('failure',['unknown','stale','missing','gap'])
def test_camera_failure_requires_board_to_be_seen_again(failure):
    s=DepartureSequence();arm(s);r=rows(.8,4,'absent','absent');now=.8
    if failure=='unknown':r['secondary']['state']='unknown'
    if failure=='stale':r['secondary']['captured_s']=0
    if failure=='missing':r.pop('secondary')
    if failure=='gap':r=rows(2.,4,'absent','absent');now=2.
    assert not s.update(r,now) and s.phase=='wait_board'
    for i in range(20,35):assert not s.update(rows(i*.2,i,'absent','absent'),i*.2)


@pytest.mark.parametrize('value',[True,0,.4,4,float('nan')])
def test_invalid_confirmation_times_rejected(value):
    with pytest.raises(ValueError):DepartureSequence(value)


class Base:
    def __init__(self):
        self.s={'mode':'disabled','hardware_output':False,'worker_started':False,'setup_token':'boot','parking_payment':{'motion_locked':False}}
        self.calls=[]
    def status(self):return dict(self.s)
    def request(self,*args):self.calls.append(args)


class Voice:
    def __init__(self):self.calls=0
    def start(self):self.calls+=1;return True
    def close(self):pass
    def status(self):return {'text':PHRASE,'command_sent':self.calls>0,'playback_heard':None}


def test_real_sequence_dispatches_exact_phrase_once_without_any_drive_call(tmp_path):
    base=Base();voice=Voice();now=[0.];trial=DepartureTrialDrive(base,{},tmp_path,clock=lambda:now[0],speech_factory=lambda:voice)
    feed=[rows(i*.2,i) for i in range(4)]+[rows(i*.2,i,'absent','absent') for i in range(4,16)]
    def observe():
        row=feed.pop(0);now[0]=row['primary']['captured_s'];return row,None
    trial._observe=observe;trial.active=True;trial.started=0.;trial.folder=tmp_path/'round';trial.folder.mkdir()
    trial._run();trial._run()
    assert voice.calls==1 and voice.status()['text']=='我发车啦'
    assert not base.calls and trial.phase=='complete' and not trial.active
    result=json.loads((trial.folder/'events.jsonl').read_text(encoding='utf-8').splitlines()[-1])
    assert result['event']=='speech_requested' and result['vehicle_commands_sent'] is False


def test_lifted_board_shape_then_clear_dispatches_speech_once(tmp_path):
    base=Base();voice=Voice();now=[0.]
    trial=DepartureTrialDrive(base,{},tmp_path,clock=lambda:now[0],speech_factory=lambda:voice)
    feed=[rows(i*.2,i) for i in range(4)]
    for i in range(4,7):
        r=rows(i*.2,i,'present','unknown');r['secondary']['scene_valid']=True;feed.append(r)
    feed += [rows(i*.2,i,'absent','absent') for i in range(7,17)]
    def observe():
        row=feed.pop(0);now[0]=row['primary']['captured_s'];return row,None
    trial._observe=observe;trial.active=True;trial.started=0.;trial.folder=tmp_path/'round';trial.folder.mkdir()
    trial._run()
    assert voice.calls==1 and not base.calls and trial.phase=='complete'
    records=[json.loads(line) for line in (trial.folder/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    assert any(r['observations']['secondary']['state']=='unknown' and r['phase']=='wait_removed' for r in records)


def test_parameters_persist_without_changing_motion_or_while_running(tmp_path):
    base=Base();trial=DepartureTrialDrive(base,{},tmp_path)
    payload={'setup_token':'boot','settings_revision':trial.settings_revision,'min_blue_percent':5,'clear_confirm_s':1.5}
    trial.request('departure_save',payload)
    restored=DepartureTrialDrive(base,{},tmp_path)
    assert restored.settings=={'min_blue_percent':5.,'clear_confirm_s':1.5} and not base.calls
    with pytest.raises(ValueError):trial.request('departure_save',payload) # old revision
    trial.active=True
    with pytest.raises(ValueError):trial.request('departure_save',{**payload,'settings_revision':trial.settings_revision})
    assert restored.settings==trial.settings


def test_speech_self_test_does_not_complete_or_start_departure(tmp_path):
    base=Base();voice=Voice();trial=DepartureTrialDrive(base,{},tmp_path,speech_factory=lambda:voice)
    result=trial.request('departure_test_speech',{'setup_token':'boot'})['departure_trial']
    assert voice.calls==1 and not base.calls
    assert result['speech_test']['command_sent'] and result['speech'] is None
    assert result['phase']=='idle' and not result['active'] and result['forward_output_enabled'] is False


def test_invalid_saved_settings_can_be_repaired_with_save(tmp_path):
    file=tmp_path/'vision/configs/departure.local.json';file.parent.mkdir(parents=True);file.write_text('{bad',encoding='utf-8')
    base=Base();trial=DepartureTrialDrive(base,{},tmp_path)
    state=trial.status()['departure_trial'];assert not state['can_start'] and state['can_save']
    trial.request('departure_save',{'setup_token':'boot','settings_revision':trial.settings_revision,'min_blue_percent':3,'clear_confirm_s':1})
    assert trial.status()['departure_trial']['can_start']


def test_active_test_blocks_motion_and_payment_but_cancel_does_not_touch_driver(tmp_path):
    base=Base();trial=DepartureTrialDrive(base,{},tmp_path);trial.active=True
    for action in ('enable','command','crosswalk_prepare','traffic_start','cone_short_start','payment_begin'):
        with pytest.raises(ValueError):trial.request(action,{})
    trial.request('stop',{});assert not base.calls and not trial.active
    trial.request('command',{'motor':'forward'});assert base.calls[-1][0]=='command'


@pytest.mark.parametrize('busy',['manual','payment'])
def test_start_rejected_when_another_control_session_is_active(tmp_path,busy):
    base=Base()
    if busy=='manual':base.s['hardware_output']=True;base.s['mode']='enabled'
    else:base.s['parking_payment']['motion_locked']=True
    trial=DepartureTrialDrive(base,{},tmp_path)
    with pytest.raises(ValueError):trial.request('departure_start',{'client':'a','setup_token':'boot'})
    assert not base.calls and not trial.active


def test_start_without_visible_board_is_allowed_but_only_waits(tmp_path):
    base=Base();trial=DepartureTrialDrive(base,{},tmp_path)
    try:
        trial.request('departure_start',{'client':'a','setup_token':'boot','clear_confirm_s':1.})
        assert trial.active and trial.phase=='wait_board' and trial.speech is None and not base.calls
        json.dumps(trial.status(),allow_nan=False)
    finally:trial.close()


def test_existing_camera_cache_is_used_and_stale_image_not_marked_clear(tmp_path):
    base=Base();_,jpg=cv2.imencode('.jpg',scene());now=[1.]
    state=SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg.tobytes(),raw_frame_id=3,raw_received_ms=1000,state='ok')
    trial=DepartureTrialDrive(base,{'primary':state,'secondary':state},tmp_path,clock=lambda:now[0])
    observations,preview=trial._observe();trial.observations=observations
    assert all(r['state']=='absent' for r in observations.values()) and preview and not base.calls
    now[0]=2.;observations,_=trial._observe();assert all(r['state']=='unknown' for r in observations.values())


def test_ui_preserves_original_render_and_inserts_before_crosswalk():
    controls=SimpleNamespace(HTML='<div id="autonomy-tests"><section id="crosswalk-test"></section></div>',STYLE='',
        SCRIPT='<script>(()=>{let gamepadControl=null,pwmControl=null;function render(s){existing(s);}async function heartbeat(){} })();</script>')
    extend_controls(controls)
    assert controls.HTML.index('id="departure-test"')<controls.HTML.index('id="crosswalk-test"')
    assert 'renderBeforeDeparture(s);renderDeparture(s)' in controls.SCRIPT and 'existing(s)' in controls.SCRIPT
    assert '我发车啦' in controls.SCRIPT and '仅播报' in controls.HTML
