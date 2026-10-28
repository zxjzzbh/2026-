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
from test_cone_low_speed import settings,observation as legacy_observation,row,Actuator


def observation(t,cones=None,**changes):
    # These are current frames from one camera, rather than anonymous legacy
    # lane dictionaries. Empty-cone frames still contain real road feedback.
    changes.setdefault('frame_id',round(t*1000))
    changes.setdefault('source_camera','secondary')
    return legacy_observation(t,cones=cones,**changes)


def continuous(settings):
    s=trial_settings(settings);s['low_speed_test'].update(manual_stop_only=True,max_run_s=None)
    return s


def warmed(settings,**kwargs):
    ctl=ContinuousConeController(settings,0.,**kwargs)
    # Keep real, distinct lane observations through arming; do not bypass the
    # three-frame startup/recovery gate or fabricate a calibrated profile.
    for t in np.arange(0.,3.01,.1):
        ctl.step(observation(round(float(t),8),cones=[]),round(float(t),8))
    return ctl


def mission_observation(t,**changes):
    # cone_mission_fixture uses millisecond frame IDs. Keep the same camera
    # and increasing frame sequence when returning to ordinary observations.
    changes.setdefault('frame_id',round(t*1000))
    changes.setdefault('source_camera','secondary')
    return observation(t,**changes)


def test_safe_guidance_has_no_periodic_stop_but_boundary_hazard_is_not_bypassed(settings):
    ctl=ContinuousConeController(continuous(settings),0.)
    assert ctl.step(observation(1.,cones=[]),1.)['action']=='neutral'
    for t in np.arange(1.1,900.01,.1):
        t=round(float(t),8)
        data=observation(t,cones=[])
        result=ctl.step(data,t)
        if t<ctl.arm_until:
            assert result['esc_us']==1500
            continue
        assert result['action']=='search' and result['esc_us']==1600
        assert result['phase']=='driving' and result['manual_stop_only']
    assert ctl.progress()['max_run_s'] is None
    with pytest.raises(ValueError,match='停止'):
        ctl.step(observation(900.1,cones=[],boundary_danger=True),900.1)


def test_target_disappearance_and_two_pass_count_do_not_end_trial(settings):
    from cone_mission_fixture import estimate_one
    ctl=warmed(continuous(settings))
    ctl.step(observation(3.1),3.1)
    for t in np.arange(3.2,6.,.1):
        assert ctl.step(observation(float(t),cones=[]),float(t))['esc_us']>1500
    assert ctl.count==0 and ctl.unverified_passes==1
    estimate_one(ctl,6.,'secondary-first');estimate_one(ctl,7.,'secondary-second')
    assert ctl.count==2 and ctl.mission.completion_ready()
    for t in np.arange(7.8,121.01,.1):
        t=round(float(t),8)
        assert ctl.step(mission_observation(t,cones=[]),t)['esc_us']>1500


@pytest.mark.parametrize('fraction',[.30,.70])
def test_retired_near_field_cache_cannot_reacquire_and_hold_old_turn(settings,fraction):
    ctl=warmed(continuous(settings));trace=[]
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
    ctl=warmed(continuous(settings))
    cone={'id':'old-edge','fraction':.5,'half_width':.04,'near':True,'y_fraction':.8,'near_field_continuation':True}
    for i in range(5):
        t=3.1+i*.1
        assert ctl.step(observation(t,cones=[cone]),t)['steering_us']==1610
        assert ctl.target is None


def exiting_cone(t,*,continuation=False):
    return {'id':'late-near','fraction':.70,'image_fraction':.70,'half_width':.03,
            'near':True,'y_fraction':.74,'exit_band':True,'near_field_continuation':continuation}


def retire_cached_target(ctl):
    from cone_mission_fixture import estimate_one
    _,result=estimate_one(ctl,3.1,'secondary-late-near')
    # Keep real current frames until the callers' next timestamp. An offset
    # remains after the visual estimate, so lane return is not complete yet.
    for t in np.arange(3.9,4.71,.1):
        t=round(float(t),8)
        result=ctl.step(mission_observation(t,cones=[],ego_fraction=.38,ego_near_fraction=.38,
                                           ego_far_fraction=.38),t)
    assert ctl.target is None and ctl.recentering
    assert ctl.count==1  # Actual ordered image-clearance evidence, not timeout.
    return result


def test_recenter_corrects_vehicle_offset_before_finishing_lane_return(settings):
    ctl=warmed(continuous(settings));retire_cached_target(ctl)
    outputs=[]
    for t in (4.8,4.9,5.,5.1,5.2):
        outputs.append(ctl.step(mission_observation(t,cones=[],ego_fraction=.6,ego_near_fraction=.6),t))
    assert all(x['steering_phase']=='return_to_lane' and not x['lane_return_confirmed'] for x in outputs)
    assert outputs[-1]['steering_us']>1610  # Correct the persistent right offset.
    assert all(1500<x['esc_us']<=1600 for x in outputs)
    for t in (5.3,5.4,5.5,5.6,5.7):result=ctl.step(mission_observation(t,cones=[]),t)
    assert result['lane_return_confirmed'] and not ctl.recentering
    assert result['steering_phase']=='lane_follow'


def test_fresh_new_obstacle_can_interrupt_recenter_but_cached_edge_cannot(settings):
    ctl=warmed(continuous(settings));retire_cached_target(ctl)
    fresh={**exiting_cone(4.8),'id':'next-real','fraction':.3,'near_field_continuation':False}
    result=ctl.step(mission_observation(4.8,cones=[fresh]),4.8)
    assert ctl.target=='next-real' and not ctl.recentering
    assert result['steering_phase']=='avoiding' and ctl.count==1


def test_near_boundary_correction_is_not_overridden_by_recenter(settings):
    ctl=warmed(continuous(settings));retire_cached_target(ctl)
    for t in (4.8,4.9,5.):
        result=ctl.step(mission_observation(t,cones=[],ego_near_fraction=.1),t)
        assert result['steering_phase']=='boundary_correction' and result['requested_steering_us']<1610
    for t in (5.1,5.2,5.3,5.4,5.5):result=ctl.step(mission_observation(t,cones=[]),t)
    assert result['steering_us']==1610 and not ctl.recentering


def test_cache_is_not_acquired_as_new_target_after_camera_switch(settings):
    ctl=warmed(continuous(settings))
    ctl.step(observation(3.1,source_camera='secondary'),3.1)
    old={**exiting_cone(3.2,continuation=True),'id':'primary-edge'}
    for t in (3.2,3.3,3.4,3.5):
        result=ctl.step(observation(t,cones=[old],source_camera='primary'),t)
        assert ctl.target is None
    assert result['steering_us']==1610 and ctl.count==0


def test_recenter_command_reaches_actual_session_output_interface(settings,tmp_path):
    from cone_mission_fixture import ground_nav,exit_nav
    made=[]
    def factory(s):
        a=Actuator();made.append(a);return a
    run=ConeRun(factory,tmp_path)
    run.prepare(settings,{'mode':'disabled','boot_test_phase':'esc_off','setup_token':'boot'},
        {'profile':'legacy_low_speed','client':'page','setup_token':'boot','esc_off':True,'fb_mode_confirmed':True},0.)
    for t in (.1,.2,.3):run.tick(observation(t,cones=[]),t)
    run.command('cone_start',{'client':'page','run_token':run.token,'trial_sequence':1,
        'fb_mode_confirmed':True,'esc_on_static':True,'field_ready':True},.4)
    for i in range(56):
        t=.5+i*.1
        run.command('cone_keepalive',{'client':'page','run_token':run.token,'trial_sequence':i+2},t)
        t=round(t,8)
        if 3.5<=t<4.:
            index=round((t-3.5)/.1)
            nav=ground_nav(t,'secondary-session',y=(.55,.63,.71,.73,.76)[index])
        elif 4.<=t<=4.2:
            nav=exit_nav(t,'secondary-session',index=round((t-4.)/.1))
        else:
            near=.38 if 4.2<t<5.3 else .5
            nav=mission_observation(t,cones=[],ego_fraction=near,ego_near_fraction=near,ego_far_fraction=near)
        run.tick(nav,t)
    sent=made[0].intents
    assert any(x['steering_us']!=1610 for x in sent)
    assert sent[-1]['steering_us']==1610 and sent[-1]['esc_us']==1600
    assert any(x.get('steering_phase')=='return_to_lane' for x in sent)
    assert run.controller.count==1 and run.controller.mission.diagnostics()['mission_passed_estimates']
    run.cancel('test ended');assert made[0].closed


def test_recenter_passes_through_output_guard_as_s3_center_without_braking(settings):
    from cone_mission_fixture import ground_nav,exit_nav
    s=continuous(settings);ctl=warmed(s);writes=[]
    guard=ConePulseGuard(actuator_settings(s),lambda *args:writes.append(args),mode='F/B')
    for i in range(24):
        t=round(3.1+i*.1,8)
        if i<5:nav=ground_nav(t,'secondary-guard',y=(.55,.63,.71,.73,.76)[i])
        elif i<8:nav=exit_nav(t,'secondary-guard',index=i-5)
        else:nav=mission_observation(t,cones=[])
        intent=ctl.step(nav,t)
        guard.accept({**intent,'sequence':i,'sent_s':t},t)
    steering=[pulse for channel,pulse in writes if channel==3]
    throttle=[pulse for channel,pulse in writes if channel==4]
    assert any(p!=1610 for p in steering) and steering[-1]==1610
    assert throttle and set(throttle)<={1570,1580,1600} and 1400 not in throttle and not guard.fault
    assert ctl.count==1


def test_brief_line_gap_and_camera_handoff_recover_within_budget(settings):
    ctl=warmed(continuous(settings))
    ctl.step(observation(3.1,cones=[],source_camera='secondary'),3.1)
    for t in (3.2,3.3):
        data=observation(t,cones=[],source_camera='primary');data.pop('lane_bounds_px')
        out=ctl.step(data,t)
        assert 1500<out['esc_us']<=1570 and abs(out['steering_us']-1610)<=80
    assert ctl.count==0
    for t in (3.35,3.45):
        out=ctl.step(observation(t,cones=[],source_camera='primary'),t)
        assert not out['paired_stable'] and ctl.line_missing_since is not None
    out=ctl.step(observation(3.5,cones=[],source_camera='primary'),3.5)
    assert out['paired_stable'] and out['esc_us']==1600
    assert ctl.line_missing_since is None
    assert ctl.progress()['line_recovery_s']==1.


def test_steering_is_proportional_and_rate_limited_instead_of_bang_bang(settings):
    ctl=warmed(continuous(settings));last=1610;values=[]
    for i,t in enumerate(np.arange(3.1,4.,.05)):
        data=observation(float(t));data['cones'][0].update(fraction=.30 if i<8 else .35,y_fraction=.55,near=False)
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
        ctl=warmed(s)
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
    ctl=warmed(s);last=1610;endpoints=set()
    for i in range(30):
        t=3.1+i*.05;near=.1 if i<15 else .9
        intent=ctl.step(observation(t,cones=[],ego_fraction=near,ego_near_fraction=near),t)
        pulse=intent['steering_us'];endpoints.add(pulse)
        assert 1410<=pulse<=1810
        if intent.get('boundary_output_withdrawal'):
            assert pulse==1610 and intent['esc_us']==1500 and intent['action']=='neutral'
        else:
            assert abs(pulse-last)<=(121 if i==0 else 61)
            assert 1500<intent['esc_us']<=1600
        last=pulse
    assert {1410,1810}<=endpoints
    with pytest.raises(ValueError,match='相机'):ctl.step(observation(4.55),4.9)


def test_older_settings_use_current_default_gain_without_rewriting_input(settings):
    s=continuous(settings);s['low_speed_test'].pop('steering_gain')
    ctl=ContinuousConeController(s,0.,handoff_after_two=True)
    assert ctl.progress()['steering_gain']==1.5
    assert 'steering_gain' not in s['low_speed_test']


@pytest.mark.parametrize('cone_fraction,direction',[(.27,-1),(.73,1)])
@pytest.mark.parametrize('historical_settings',[False,True])
def test_edge_target_responds_on_first_frame_and_old_settings_still_load(settings,cone_fraction,direction,historical_settings):
    s=continuous(settings)
    if historical_settings:s['low_speed_test'].pop('steering_gain')
    ctl=warmed(s);pulses=[]
    for t in (3.1,3.2,3.3,3.4,3.5,3.6):
        data=observation(t);data['cones'][0].update(fraction=cone_fraction,near=False,y_fraction=.55)
        intent=ctl.step(data,t);pulses.append(intent['steering_us'])
        assert intent['action']=='search' and intent['esc_us']==1600
    assert direction*(pulses[0]-1610)>0
    assert max(pulses)-min(pulses)<=1  # A stationary target must not drift.
    assert ctl.progress()['steering_policy']=='shared_multi_row_pd'


def test_one_cone_locks_its_side_and_stops_if_that_gap_disappears(settings):
    ctl=warmed(continuous(settings),stop_for_cones=True)
    for i,x in enumerate((.5,.51,.52,.495,.51)):
        t=3.1+i*.1;data=observation(t)
        data['cones'][0].update(fraction=x,near=False,y_fraction=.55)
        ctl.step(data,t)
        if i==0:side=ctl.side
        assert ctl.side==side and ctl.target=='one' and ctl.count==0
    data=observation(3.7);data['cones'][0].update(fraction=.30,near=False,y_fraction=.55)
    with pytest.raises(ValueError,match='间隙'):ctl.step(data,3.7)
    assert ctl.side==side


def test_occluded_cone_keeps_pass_bookkeeping_but_does_not_steer_to_old_position(settings):
    ctl=warmed(continuous(settings))
    ctl.step(observation(3.1),3.1);target=ctl.target_fraction;side=ctl.side
    output=ctl.step(observation(3.2,cones=[]),3.2)
    assert ctl.target_fraction==target!=.5 and ctl.side==side
    assert ctl.review_pending and ctl.count==0
    assert ctl.filtered_target is None and output['target_source']=='target_pending'
    assert abs(output['requested_steering_us']-1610)<=80


def test_second_cone_uses_current_position_after_observed_first_pass(settings):
    from cone_mission_fixture import estimate_one
    ctl=warmed(continuous(settings))
    estimate_one(ctl,3.1,'secondary-first',side='left')
    assert ctl.count==1
    data=mission_observation(3.9,ego_fraction=.65,ego_near_fraction=.65)
    data['cones'][0].update(id='secondary-second',fraction=.5,near=False,y_fraction=.5)
    ctl.step(data,3.9)
    assert ctl.side=='right' and ctl.target=='secondary-second'


def test_image_exit_without_observed_side_clearance_cannot_count_as_pass(settings):
    ctl=warmed(continuous(settings),stop_for_cones=True)
    for i,y in enumerate((.55,.63)):
        t=3.1+i*.1;data=observation(t)
        data['cones'][0].update(fraction=.5,near=y>=.6,y_fraction=y,exit_band=y>=.7)
        ctl.step(data,t)
    data=observation(3.3);data['cones'][0].update(fraction=.5,near=True,y_fraction=.71,exit_band=True)
    with pytest.raises(ValueError,match='净空'):ctl.step(data,3.3)
    assert ctl.pass_gate_hits==0 and ctl.count==0


def test_five_and_twenty_hz_have_same_elapsed_time_steering_budget(settings):
    end=[]
    for dt in (.2,.05):
        ctl=ContinuousConeController(continuous(settings),0.,initialized=True)
        for t in (0.,.2,.4):ctl.step(observation(t,cones=[],ego_fraction=.3),t)
        for t in np.arange(.4+dt,.801,dt):
            intent=ctl.step(observation(float(t),cones=[],ego_fraction=.7),float(t))
            assert not intent.get('boundary_output_withdrawal')
            pulse=intent['steering_us']
        end.append(pulse)
    assert end[0]==pytest.approx(end[1],abs=1)
    # The first two lane frames are neutral; only the third may start steering.
    # Both rates then receive the same real 400 ms reversal budget.
    assert end[0]>1610


def test_cached_frame_cannot_repeat_steering_or_refresh_pass_evidence(settings):
    ctl=warmed(continuous(settings))
    data=observation(3.1);first=ctl.step(data,3.1);before=ctl.progress()
    pd_before=(ctl.pd.error,ctl.pd.derivative)
    passage_before=(ctl.count,ctl.pass_gate_hits,ctl.auto_count,copy.deepcopy(ctl.passed_ids))
    repeated=ctl.step(data,3.2)
    assert not repeated['fresh_control']
    assert repeated['esc_us']==first['esc_us'] and repeated['steering_us']==first['steering_us']
    after=ctl.progress()
    assert (ctl.pd.error,ctl.pd.derivative)==pd_before
    # Remaining deadline time is status, not new control or passage evidence.
    for key in ('output_steering_us','requested_steering_us',
                'passage_evidence','paired_frames','target_fraction'):
        assert after.get(key)==before.get(key)
    assert (ctl.count,ctl.pass_gate_hits,ctl.auto_count,ctl.passed_ids)==passage_before
    assert after['target_observation'].get('frames',0)==before['target_observation'].get('frames',0)
    assert not after['target_observation']['fresh_evidence']


def test_fresh_camera_handoff_allows_older_capture_without_reversing_side(settings):
    ctl=warmed(continuous(settings))
    ctl.step(observation(3.1,source_camera='secondary'),3.1);side=ctl.side
    data=observation(3.08,source_camera='primary');data['cones'][0]['id']='primary-alias'
    out=ctl.step(data,3.2)
    assert 1500<out['esc_us']<=1570 and ctl.side==side and ctl.target=='primary-alias'
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
        ctl=warmed(continuous(settings))
        first={'id':'front','fraction':.5,'half_width':.03,'near':True,'y_fraction':.6}
        second={**first,'id':'next','fraction':second_x,'y_fraction':.45,'near':False}
        assert 1500<ctl.step(observation(3.1,cones=[first,second]),3.1)['esc_us']<=1600
        chosen.append(ctl.side)
    assert chosen==['right','left']


def test_sustained_outside_boundary_stops_even_in_operator_ended_trial(settings):
    nav=LowSpeedNavigator(continuous(settings));ctl=warmed(continuous(settings))
    for i,t in enumerate((3.1,3.2,3.3,3.4)):
        data=row(t,310+i);data['tracks']=[]
        data['road_context']['left_inner']=[[280.,y] for y in np.linspace(180,260,12)]
        data['road_context']['right_inner']=[[470.,y] for y in np.linspace(180,260,12)]
        guide=nav.update(np.zeros((360,480,3),np.uint8),data,t)
        if guide.get('boundary_danger'):
            with pytest.raises(ValueError,match='白线外'):ctl.step(guide,t)
            assert guide['action']=='stop' and not guide['ready']
            break
        result=ctl.step(guide,t)
        assert 1500<result['esc_us']<=1600 and result['steering_us']<1610
    else:pytest.fail('persistent departure did not stop')


def test_blue_background_does_not_become_a_new_target_when_lines_are_missing(settings):
    data=row(3.1,1);data['road_context']['valid']=False
    data['tracks'][0].update(kind='unresolved_blue',confirmed=True,ground_contact_usable=False,
        clipped=True,bbox_xyxy=[260,0,480,360],foot_pixel=[380,220])
    nav=LowSpeedNavigator(continuous(settings)).update(np.zeros((360,480,3),np.uint8),data,3.1)
    assert nav['uncertain']==['one'] and not nav['cones']
    assert ContinuousConeController(continuous(settings),0.).step(nav,3.1)['steering_us']==1610


def test_camera_and_clock_failures_and_worker_watchdog_are_not_bypassed(settings):
    s=continuous(settings);ctl=warmed(s)
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
    for t in (.1,.2,.3):run.tick(observation(t,cones=[]),t)
    run.command('cone_start',{'client':'page','run_token':run.token,'trial_sequence':1,
        'fb_mode_confirmed':True,'esc_on_static':True,'field_ready':True},.4)
    assert isinstance(run.controller,ContinuousConeController)
    for i in range(660):
        t=.5+i*.2
        run.command('cone_keepalive',{'client':'page','run_token':run.token,'trial_sequence':i+2},t)
        run.tick(observation(t,cones=[]),t)
        assert run.active
        if t>run.controller.arm_until:assert made[0].intents[-1]['action']=='search'
    run.cancel('operator stop')
    assert not run.active and made[0].closed
    run.tick(observation(140.,cones=[]),140.)
    assert not run.active


def test_actual_failure_trace_cannot_override_navigation_stop(settings):
    path=Path(__file__).parent/'fixtures/cone-continuity-20261013.json'
    trace=json.loads(path.read_text(encoding='utf-8'));s=copy.deepcopy(trace['settings'])
    s['low_speed_test'].update(manual_stop_only=True,max_run_s=None)
    ctl=ContinuousConeController(s,trace['started_s']);moving=False
    for row_data in trace['steps']:
        nav=row_data['navigation']
        if nav.get('ready') is False or nav.get('action')=='stop' or nav.get('boundary_danger'):
            with pytest.raises(ValueError):ctl.step(nav,row_data['now_s'])
            break
        intent=ctl.step(row_data['navigation'],row_data['now_s']);moving|=intent['esc_us']>1500
        if moving:assert intent['esc_us']>1500
    else:pytest.fail('trace stop request was not exercised')
