"""Distinct sensor captures at realistic processing cadence; no hardware."""
import pytest

from carvision.traffic_voting import DualSignalVotes
from carvision.traffic_driving import TrafficDrivingController,load_settings
from test_traffic_driving import Run,CONFIG
from test_traffic_voting import frame


@pytest.mark.parametrize('interval,gap',[(None,.4),(None,.6),(None,.75),(.5,.4)])
@pytest.mark.parametrize('color,window',[('red',1.),('green',2.)])
def test_formal_vote_uses_real_window_at_supported_cadence(interval,gap,color,window):
    controller=TrafficDrivingController(load_settings(CONFIG),control_interval_s=interval)
    vote=controller.votes
    for fid in range(10):
        now=round(fid*gap,8)
        result=vote.update({'secondary':frame('secondary',color,fid,now)},now)
        if now<window:assert not result[color+'_confirmed']
    assert result[color+'_confirmed']
    native=vote.votes['secondary'] if color=='red' else vote.green_votes['secondary']
    assert len(native.frames)<=int(window/gap)+1
    assert all(a[0]<b[0] for a,b in zip(native.frames,list(native.frames)[1:]))


@pytest.mark.parametrize('bad',['stale','different_fixture','long_gap'])
def test_camera_discontinuity_still_resets_formal_vote(bad):
    vote=DualSignalVotes(1,50,.5,green_window_s=2,maximum_gap_s=.75)
    for fid in range(7):
        now=round(fid*.4,8);result=vote.update({'secondary':frame('secondary','green',fid,now)},now)
    assert result['green_confirmed']
    stamp=2.8 if bad!='long_gap' else 3.2
    row=frame('secondary','green',7,stamp,
              [300,200,470,300] if bad=='different_fixture' else None)
    result=vote.update({'secondary':row},stamp+.51 if bad=='stale' else stamp)
    assert not result['green_confirmed']
    assert result['cameras']['secondary']['green_collected_s']==0


@pytest.mark.parametrize('fixture_present',[False,True])
def test_fresh_unknown_is_a_negative_vote_and_never_current_green_authority(fixture_present):
    vote=DualSignalVotes(1,50,.5,green_window_s=2,maximum_gap_s=.75)
    for fid in range(21):
        now=fid/10
        result=vote.update({'secondary':frame('secondary','green',fid,now)},now)
    row=frame('secondary','unknown',21,2.1)
    row['signal']['fixture_detected']=fixture_present
    result=vote.update({'secondary':row},2.1)
    assert result['cameras']['secondary']['green_percent']==95
    assert result['cameras']['secondary']['green_total_frames']==20
    assert result['current_green_sources']==[] and not result['green_confirmed']
    result=vote.update({'secondary':frame('secondary','green',22,2.2)},2.2)
    if fixture_present:
        assert result['green_confirmed'] and result['cameras']['secondary']['green_percent']==95
    else:
        assert not result['green_confirmed']
        assert result['cameras']['secondary']['green_total_frames']==1
        assert result['cameras']['secondary']['green_collected_s']==0


def test_unknown_color_with_changed_fixture_does_not_retain_old_green_history():
    vote=DualSignalVotes(1,50,.5,green_window_s=2,maximum_gap_s=.75)
    for fid in range(21):
        result=vote.update({'secondary':frame('secondary','green',fid,fid/10)},fid/10)
    row=frame('secondary','unknown',21,2.1,[300,200,470,300])
    row['signal']['fixture_detected']=True
    result=vote.update({'secondary':row},2.1)
    assert not result['green_sources'] and not result['green_confirmed']
    assert result['cameras']['secondary']['green_total_frames']==1


def test_duplicate_images_never_supply_votes_and_fast_red_budget_stays_short():
    vote=DualSignalVotes(1,50,.5,green_window_s=2,maximum_gap_s=.75)
    row=frame('secondary','green',1,1.)
    for now in (1.,1.1,1.2,1.3):result=vote.update({'secondary':row},now)
    assert result['cameras']['secondary']['total_frames']==1 and not result['green_confirmed']
    vote=DualSignalVotes(1,50,.5,green_window_s=2,maximum_gap_s=.75)
    for fid in range(3):
        now=fid*.4
        result=vote.update({'secondary':frame('secondary','red',fid,now)},now)
    assert not result['fast_red_sources']


def test_slow_race_red_brakes_then_confirms_and_waits_actual_green_window():
    run=Run('off');run.c.control_interval_s=None
    run.c.votes=TrafficDrivingController(load_settings(CONFIG),control_interval_s=None).votes
    run.until('approach',moving=True)
    for _ in range(10):
        output=run.step('red',False,dt=.4)
        assert output['phase']!='fault'
        if output['phase']=='wait_green':break
    assert output['phase']=='wait_green' and run.c.red_seen
    first_green=run.t+.4
    for _ in range(10):
        output=run.step('green',False,dt=.4)
        if run.t-first_green<2.-1e-9:assert output['phase']!='done'
        if output['phase']=='done':break
    assert output['phase']=='done' and output['esc_us']==1500


def motion_run(interval=None):
    run=Run('off');run.c.control_interval_s=interval
    run.until('approach',moving=False)
    since=run.t-.5
    run.step(moving=None)
    assert run.c.phase=='motion_recovery'
    return run,since


def snapshot(run,since,dt=.4,**changes):
    now=round(run.t+dt,8)
    motion={'valid':True,'moving':False,'captured_s':now,'continuous_still_since_s':since,**changes}
    return run.step(dt=dt,motion=motion)


@pytest.mark.parametrize('interval',[None,.5])
def test_continuous_motion_interval_can_start_before_recovery(interval):
    run,since=motion_run(interval)
    snapshot(run,since)
    output=snapshot(run,since)
    assert output['phase']=='approach' and output['esc_us']==1500


def test_stale_snapshot_does_not_discard_later_independent_continuity():
    run,since=motion_run()
    snapshot(run,since,dt=.1,valid=False,moving=None,captured_s=run.t-.3,reason='ground_motion_stale')
    snapshot(run,since,dt=.4)
    output=snapshot(run,since,dt=.4)
    assert output['phase']=='approach'


@pytest.mark.parametrize('conflict',[{'moving':True},{'valid':False,'moving':None,'reason':'insufficient_features'}])
def test_real_movement_or_sampler_unknown_cannot_be_erased_by_old_stillness(conflict):
    run,since=motion_run()
    snapshot(run,since,dt=.4,**conflict)
    output=snapshot(run,since)
    assert output['phase']=='motion_recovery'
    output=snapshot(run,since)
    assert output['phase']=='motion_recovery'
    output=snapshot(run,since)
    assert output['phase']=='fault'


@pytest.mark.parametrize('conflict',[{'moving':True},{'valid':False,'moving':None,'reason':'insufficient_features'}])
def test_new_continuous_interval_after_real_conflict_can_recover(conflict):
    run,since=motion_run()
    snapshot(run,since,dt=.4,**conflict)
    output=snapshot(run,run.t+.01)
    assert output['phase']=='approach' and output['esc_us']==1500
