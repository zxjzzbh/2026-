"""Operator-ended trial behavior through the actual session/output interfaces."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_low_speed import trial_settings,LowSpeedNavigator
from carvision.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard
from carvision.cone_execution import ConeRun
from test_cone_low_speed import settings,observation,row,Actuator


def continuous(settings):
    s=trial_settings(settings);s['low_speed_test'].update(manual_stop_only=True,max_run_s=None)
    return s


def test_visual_warnings_and_elapsed_time_never_emit_stop(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    assert ctl.step(observation(1.,cones=[]),1.)['action']=='neutral'
    for t,change in [(3.1,{}),(10.,{'boundary_danger':True,'ego_near_fraction':-.1}),
                     (120.,{'uncertain':['shape']}),(900.,{})]:
        data=observation(t,**change)
        data['cones'][0].update(fraction=.5,y_fraction=.75,near=True,image_close=True)
        result=ctl.step(data,t)
        assert result['action']=='search' and result['esc_us']==1600
        assert result['phase']=='driving' and result['manual_stop_only']
    assert ctl.progress()['max_run_s'] is None


def test_target_disappearance_and_two_pass_count_do_not_end_trial(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    ctl.step(observation(3.1),3.1)
    for t in np.arange(3.2,6.,.1):
        assert ctl.step(observation(float(t),cones=[]),float(t))['action']=='search'
    assert ctl.count==0 and ctl.unverified_passes==1
    ctl.count=2
    for t in (6.1,7.,121.):assert ctl.step(observation(t,cones=[]),t)['action']=='search'


def test_missing_lines_and_ambiguous_camera_handoff_do_not_stop(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    ctl.step(observation(3.1,source_camera='secondary'),3.1)
    for t in (3.2,3.3,4.,8.):
        data=observation(t,cones=[],source_camera='primary');data.pop('lane_bounds_px')
        assert ctl.step(data,t)['action']=='search'
    assert ctl.count==0
    data=observation(8.1,source_camera='primary');data['cones'][0]['id']='new-view'
    assert ctl.step(data,8.1)['action']=='search'


def test_steering_is_proportional_and_rate_limited_instead_of_bang_bang(settings):
    ctl=ContinuousConeController(continuous(settings),0.);last=1610;values=[]
    for i,t in enumerate(np.arange(3.1,4.,.05)):
        data=observation(float(t));data['cones'][0].update(fraction=.30 if i<8 else .70,y_fraction=.55,near=False)
        result=ctl.step(data,float(t));pulse=result['steering_us'];values.append(pulse)
        assert abs(pulse-last)<=121 if i==0 else abs(pulse-last)<=61
        assert 1350<=pulse<=1870
        last=pulse
    assert any(1350<v<1610 for v in values) and ctl.progress()['manual_stop_only']


@pytest.mark.parametrize('cone_fraction',[.27,.73])
def test_selected_gain_amplifies_partial_avoidance_without_changing_direction(settings,cone_fraction):
    pulses=[]
    for gain in (1.,1.6):
        s=trial_settings(settings,{'test_steering_gain':gain})
        ctl=ContinuousConeController(s,0.)
        for t in np.arange(3.1,5.,.1):
            data=observation(float(t));data['cones'][0].update(fraction=cone_fraction,near=False,y_fraction=.55)
            intent=ctl.step(data,float(t))
            assert intent['action']=='search' and intent['esc_us']==1600
        pulses.append(intent['steering_us']-1610)
        assert ctl.progress()['steering_gain']==gain
    assert pulses[0]*pulses[1]>0
    assert abs(pulses[1])==pytest.approx(abs(pulses[0])*1.6,abs=1.)


def test_maximum_gain_preserves_selected_endpoints_slew_limit_and_frame_guard(settings):
    s=trial_settings(settings,{'test_steering_gain':3.,'test_left_us':1810,'test_right_us':1410})
    ctl=ContinuousConeController(s,0.);last=1610;endpoints=set()
    for i in range(30):
        t=3.1+i*.05;near=.1 if i<15 else .9
        intent=ctl.step(observation(t,cones=[],ego_fraction=near,ego_near_fraction=near),t)
        pulse=intent['steering_us'];endpoints.add(pulse)
        assert 1410<=pulse<=1810 and abs(pulse-last)<=(121 if i==0 else 61)
        assert intent['action']=='search'
        last=pulse
    assert {1410,1810}<=endpoints
    with pytest.raises(ValueError,match='相机'):ctl.step(observation(4.55),4.9)


def test_older_settings_keep_original_gain_without_rewriting_input(settings):
    s=continuous(settings);s['low_speed_test'].pop('steering_gain')
    ctl=ContinuousConeController(s,0.,handoff_after_two=True)
    assert ctl.progress()['steering_gain']==1.0
    assert 'steering_gain' not in s['low_speed_test']


@pytest.mark.parametrize('cone_fraction,expected',[
    (.27,[1610,1569,1559,1553,1550,1548]),
    (.73,[1610,1651,1661,1667,1670,1672]),
])
@pytest.mark.parametrize('historical_settings',[False,True])
def test_default_matches_original_controller_steering_trace(settings,cone_fraction,expected,historical_settings):
    # Captured from the pre-gain controller, covering both partial turns and
    # its initial deadband. Adding a knob alone must not alter these outputs.
    s=continuous(settings)
    if historical_settings:s['low_speed_test'].pop('steering_gain')
    ctl=ContinuousConeController(s,0.);pulses=[]
    for t in (3.1,3.2,3.3,3.4,3.5,3.6):
        data=observation(t);data['cones'][0].update(fraction=cone_fraction,near=False,y_fraction=.55)
        intent=ctl.step(data,t);pulses.append(intent['steering_us'])
        assert intent['action']=='search' and intent['esc_us']==1600
    assert pulses==expected


@pytest.mark.parametrize('value',[True,0.,3.1,float('nan'),float('inf'),'2'])
def test_direct_controller_settings_cannot_bypass_gain_validation(settings,value):
    s=continuous(settings);s['low_speed_test']['steering_gain']=value
    with pytest.raises(ValueError,match='转向增益'):ContinuousConeController(s,0.)


def test_second_cone_position_changes_the_first_avoidance_choice(settings):
    chosen=[]
    for second_x in (.15,.85):
        ctl=ContinuousConeController(continuous(settings),0.)
        first={'id':'front','fraction':.5,'half_width':.03,'near':True,'y_fraction':.6}
        second={**first,'id':'next','fraction':second_x,'y_fraction':.45,'near':False}
        assert ctl.step(observation(3.1,cones=[first,second]),3.1)['action']=='search'
        chosen.append(ctl.side)
    assert chosen==['right','left']


def test_sustained_near_boundary_requests_correction_not_session_exit(settings):
    nav=LowSpeedNavigator(continuous(settings));ctl=ContinuousConeController(continuous(settings),0.)
    for i,t in enumerate((3.1,3.2,3.3,3.4)):
        data=row(t,i);data['tracks']=[]
        data['road_context']['left_inner']=[[280.,y] for y in np.linspace(180,260,12)]
        data['road_context']['right_inner']=[[470.,y] for y in np.linspace(180,260,12)]
        guide=nav.update(np.zeros((360,480,3),np.uint8),data,t)
        assert not guide.get('boundary_danger')
        result=ctl.step(guide,t)
        assert result['action']=='search' and result['steering_us']<1610
    assert guide['boundary_warning']


def test_blue_background_does_not_become_a_new_target_when_lines_are_missing(settings):
    data=row(3.1,1);data['road_context']['valid']=False
    data['tracks'][0].update(kind='unresolved_blue',confirmed=True,ground_contact_usable=False,
        clipped=True,bbox_xyxy=[260,0,480,360],foot_pixel=[380,220])
    nav=LowSpeedNavigator(continuous(settings)).update(np.zeros((360,480,3),np.uint8),data,3.1)
    assert nav['uncertain']==['one'] and not nav['cones']
    assert ContinuousConeController(continuous(settings),0.).step(nav,3.1)['steering_us']==1610


def test_camera_and_clock_failures_and_worker_watchdog_are_not_bypassed(settings):
    s=continuous(settings);ctl=ContinuousConeController(s,0.)
    result=ctl.step(observation(3.1,cones=[]),3.1)
    with pytest.raises(ValueError,match='相机'):ctl.step(observation(3.1),3.5)
    with pytest.raises(ValueError,match='时钟'):ctl.step(observation(2.),2.)
    writes=[];guard=ConePulseGuard(actuator_settings(s),lambda *v:writes.append(v),mode='F/B')
    guard.accept({**result,'sequence':0,'sent_s':3.1},3.1);guard.tick(3.31)
    assert guard.fault and guard.esc==1400


def test_actual_session_has_no_motion_timeout_and_stops_on_operator_cancel(settings,tmp_path):
    made=[]
    def factory(s):
        a=Actuator();made.append(a);return a
    run=ConeRun(factory,tmp_path)
    run.prepare(settings,{'mode':'disabled','boot_test_phase':'esc_off','setup_token':'boot'},
        {'profile':'legacy_low_speed','client':'page','setup_token':'boot','esc_off':True,'fb_mode_confirmed':True},0.)
    run.tick(observation(.1,cones=[]),.1)
    run.command('cone_start',{'client':'page','run_token':run.token,'trial_sequence':1,
        'fb_mode_confirmed':True,'esc_on_static':True,'field_ready':True},.2)
    assert isinstance(run.controller,ContinuousConeController)
    for i in range(660):
        t=.3+i*.2
        run.command('cone_keepalive',{'client':'page','run_token':run.token,'trial_sequence':i+2},t)
        run.tick(observation(t,cones=[]),t)
        assert run.active
        if t>3.2:assert made[0].intents[-1]['action']=='search'
    run.cancel('operator stop')
    assert not run.active and made[0].closed
    run.tick(observation(140.,cones=[]),140.)
    assert not run.active


def test_actual_failure_trace_runs_without_perception_stop_commands(settings):
    path=Path(__file__).parent/'fixtures/cone-continuity-20261013.json'
    trace=json.loads(path.read_text(encoding='utf-8'));s=copy.deepcopy(trace['settings'])
    s['low_speed_test'].update(manual_stop_only=True,max_run_s=None)
    ctl=ContinuousConeController(s,trace['started_s']);moving=False
    for row_data in trace['steps']:
        intent=ctl.step(row_data['navigation'],row_data['now_s']);moving|=intent['action']=='search'
        if moving:assert intent['action']=='search'
