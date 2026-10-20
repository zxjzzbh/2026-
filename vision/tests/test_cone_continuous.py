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


@pytest.mark.parametrize('fraction',[.45,.55])
def test_retired_near_field_cache_cannot_reacquire_and_hold_old_turn(settings,fraction):
    ctl=ContinuousConeController(continuous(settings),0.);trace=[]
    for i in range(24):
        t=3.1+i*.1
        cone={'id':'late-near','fraction':fraction,'half_width':.03,'near':True,'y_fraction':.74,'exit_band':True}
        if i>=5:cone['near_field_continuation']=True
        trace.append(ctl.step(observation(t,cones=[cone]),t)['steering_us'])
    # Existing one-second tail allowance expires, then bounded slew reaches
    # centre within three 100-ms updates, even while the cached edge remains.
    assert all(p==1610 for p in trace[19:])
    assert ctl.target is None and ctl.unverified_passes==1 and ctl.count==0
    assert all(abs(a-b)<=121 for a,b in zip(trace,trace[1:]))


def test_near_field_cache_alone_never_creates_a_fresh_avoidance_target(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    cone={'id':'old-edge','fraction':.5,'half_width':.04,'near':True,'y_fraction':.8,'near_field_continuation':True}
    for i in range(5):
        t=3.1+i*.1
        assert ctl.step(observation(t,cones=[cone]),t)['steering_us']==1610
        assert ctl.target is None


def exiting_cone(t,*,continuation=False):
    return {'id':'late-near','fraction':.5,'image_fraction':.5,'half_width':.03,
            'near':True,'y_fraction':.74,'exit_band':True,'near_field_continuation':continuation}


def retire_cached_target(ctl):
    result=None
    for i in range(17):
        t=3.1+i*.1
        result=ctl.step(observation(t,cones=[exiting_cone(t,continuation=i>=5)]),t)
    assert ctl.target is None and ctl.recentering
    return result


def test_recenter_sends_centre_before_resuming_lane_offset_correction(settings):
    ctl=ContinuousConeController(continuous(settings),0.);retire_cached_target(ctl)
    outputs=[]
    for t in (4.8,4.9,5.,5.1,5.2):
        outputs.append(ctl.step(observation(t,cones=[],ego_fraction=.6,ego_near_fraction=.6),t))
    assert any(x['steering_us']==1610 and x['steering_phase']=='recentering' for x in outputs)
    assert outputs[-1]['steering_phase']=='lane_follow'
    assert all(x['action']=='search' and x['esc_us']==1600 for x in outputs)


def test_fresh_new_obstacle_can_interrupt_recenter_but_cached_edge_cannot(settings):
    ctl=ContinuousConeController(continuous(settings),0.);retire_cached_target(ctl)
    fresh={**exiting_cone(4.8),'id':'next-real','fraction':.6,'near_field_continuation':False}
    result=ctl.step(observation(4.8,cones=[fresh]),4.8)
    assert ctl.target=='next-real' and not ctl.recentering
    assert result['steering_phase']=='avoiding' and ctl.count==0


def test_near_boundary_correction_is_not_overridden_by_recenter(settings):
    ctl=ContinuousConeController(continuous(settings),0.);retire_cached_target(ctl)
    for t in (4.8,4.9,5.):
        result=ctl.step(observation(t,cones=[],ego_near_fraction=.1),t)
        assert result['steering_phase']=='boundary_correction' and result['requested_steering_us']<1610
    for t in (5.1,5.2,5.3):result=ctl.step(observation(t,cones=[]),t)
    assert result['steering_us']==1610 and not ctl.recentering


def test_cache_is_not_acquired_as_new_target_after_camera_switch(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    ctl.step(observation(3.1,source_camera='secondary'),3.1)
    old={**exiting_cone(3.2,continuation=True),'id':'primary-edge'}
    for t in (3.2,3.3,3.4,3.5):
        result=ctl.step(observation(t,cones=[old],source_camera='primary'),t)
        assert ctl.target is None
    assert result['steering_us']==1610 and ctl.count==0


def test_recenter_command_reaches_actual_session_output_interface(settings,tmp_path):
    made=[]
    def factory(s):
        a=Actuator();made.append(a);return a
    run=ConeRun(factory,tmp_path)
    run.prepare(settings,{'mode':'disabled','boot_test_phase':'esc_off','setup_token':'boot'},
        {'profile':'legacy_low_speed','client':'page','setup_token':'boot','esc_off':True,'fb_mode_confirmed':True},0.)
    run.tick(observation(.1,cones=[]),.1)
    run.command('cone_start',{'client':'page','run_token':run.token,'trial_sequence':1,
        'fb_mode_confirmed':True,'esc_on_static':True,'field_ready':True},.2)
    for i in range(56):
        t=.3+i*.1
        run.command('cone_keepalive',{'client':'page','run_token':run.token,'trial_sequence':i+2},t)
        cone=exiting_cone(t,continuation=t>=3.8)
        run.tick(observation(t,cones=[cone]),t)
    sent=made[0].intents
    assert any(x['steering_us']!=1610 for x in sent)
    assert sent[-1]['steering_us']==1610 and sent[-1]['esc_us']==1600
    assert any(x.get('steering_phase')=='recentering' for x in sent)
    run.cancel('test ended');assert made[0].closed


def test_recenter_passes_through_output_guard_as_s3_center_without_braking(settings):
    s=continuous(settings);ctl=ContinuousConeController(s,0.);writes=[]
    guard=ConePulseGuard(actuator_settings(s),lambda *args:writes.append(args),mode='F/B')
    for i in range(24):
        t=3.1+i*.1;intent=ctl.step(observation(t,cones=[exiting_cone(t,continuation=i>=5)]),t)
        guard.accept({**intent,'sequence':i,'sent_s':t},t)
    steering=[pulse for channel,pulse in writes if channel==3]
    throttle=[pulse for channel,pulse in writes if channel==4]
    assert any(p!=1610 for p in steering) and steering[-1]==1610
    assert throttle and set(throttle)=={1600} and not guard.fault


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


@pytest.mark.parametrize('cone_fraction,direction',[(.27,-1),(.73,1)])
@pytest.mark.parametrize('historical_settings',[False,True])
def test_edge_target_responds_on_first_frame_and_old_settings_still_load(settings,cone_fraction,direction,historical_settings):
    s=continuous(settings)
    if historical_settings:s['low_speed_test'].pop('steering_gain')
    ctl=ContinuousConeController(s,0.);pulses=[]
    for t in (3.1,3.2,3.3,3.4,3.5,3.6):
        data=observation(t);data['cones'][0].update(fraction=cone_fraction,near=False,y_fraction=.55)
        intent=ctl.step(data,t);pulses.append(intent['steering_us'])
        assert intent['action']=='search' and intent['esc_us']==1600
    assert direction*(pulses[0]-1610)>0
    assert max(pulses)-min(pulses)<=1  # A stationary target must not drift.
    assert ctl.progress()['steering_policy']=='locked_cone_edge_pd'


def test_one_cone_never_changes_its_selected_side_under_image_jitter(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    for i,x in enumerate((.5,.48,.52,.30,.70,.49,.51)):
        t=3.1+i*.1;data=observation(t)
        data['cones'][0].update(fraction=x,near=False,y_fraction=.55)
        ctl.step(data,t)
        if i==0:side=ctl.side
        assert ctl.side==side and ctl.target=='one' and ctl.count==0


def test_occluded_cone_keeps_its_edge_target_during_tail_clearance(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    ctl.step(observation(3.1),3.1);target=ctl.target_fraction;side=ctl.side
    ctl.step(observation(3.2,cones=[]),3.2)
    assert ctl.target_fraction==target!=.5 and ctl.side==side
    assert ctl.review_pending and ctl.count==0


def test_second_cone_prefers_opposite_side_after_observed_first_pass(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    for i,y in enumerate((.55,.63,.71,.73,.75)):
        t=3.1+i*.1;data=observation(t,ego_fraction=.38)
        data['cones'][0].update(fraction=.70,near=y>=.6,y_fraction=y,exit_band=y>=.7)
        ctl.step(data,t)
    assert ctl.side=='left'
    for i in range(14):
        t=3.6+i*.1;ctl.step(observation(t,cones=[],ego_fraction=.38),t)
    assert ctl.count==1
    data=observation(5.1);data['cones'][0].update(id='second',fraction=.5,near=False,y_fraction=.5)
    ctl.step(data,5.1)
    assert ctl.side=='right' and ctl.target=='second'


def test_image_exit_without_observed_side_clearance_cannot_count_as_pass(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    for i,y in enumerate((.55,.63,.71,.73,.75)):
        t=3.1+i*.1;data=observation(t)
        data['cones'][0].update(fraction=.5,near=y>=.6,y_fraction=y,exit_band=y>=.7)
        ctl.step(data,t)
    assert ctl.escape_exit_hits>=3 and ctl.pass_gate_hits==0
    for i in range(14):
        t=3.6+i*.1;ctl.step(observation(t,cones=[]),t)
    assert ctl.count==0 and ctl.unverified_passes==1


def test_five_and_twenty_hz_have_same_elapsed_time_steering_budget(settings):
    end=[]
    for dt in (.2,.05):
        ctl=ContinuousConeController(continuous(settings),0.,initialized=True)
        for t in (0.,.2,.4):ctl.step(observation(t,cones=[],ego_fraction=.1),t)
        for t in np.arange(.4+dt,.801,dt):
            pulse=ctl.step(observation(float(t),cones=[],ego_fraction=.9),float(t))['steering_us']
        end.append(pulse)
    assert end[0]==pytest.approx(end[1],abs=1)
    assert end[0]>1800  # The old 0.1 s cap left the 5 Hz result around 1590.


def test_cached_frame_cannot_repeat_steering_or_refresh_pass_evidence(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    data=observation(3.1);first=ctl.step(data,3.1);before=ctl.progress()
    assert ctl.step(data,3.2)==first and ctl.progress()==before


def test_fresh_camera_handoff_allows_older_capture_without_reversing_side(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    ctl.step(observation(3.1,source_camera='secondary'),3.1);side=ctl.side
    data=observation(3.08,source_camera='primary');data['cones'][0]['id']='primary-alias'
    out=ctl.step(data,3.2)
    assert out['action']=='search' and ctl.side==side and ctl.target=='primary-alias'
    assert ctl.count==0 and ctl.pd.derivative==0.
    with pytest.raises(ValueError,match='相机时间'):
        ctl.step(observation(3.07,source_camera='primary'),3.22)


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
