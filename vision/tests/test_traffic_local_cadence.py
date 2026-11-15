"""Actual traffic consumers under slow result cadence; no camera or actuator I/O."""
import pytest

from carvision.race_flow import FlowTrafficController
from carvision.traffic_driving import load_settings
from carvision.traffic_stop_motion import TrafficStopMotion
from carvision.traffic_tracking import TrafficApproachGuard
from test_traffic_driving import CONFIG
from test_traffic_voting import frame
from test_traffic_tracking import fake_camera,publish_read,signal,SETTINGS,row as lamp_row


class Replay:
    def __init__(self, external=True):
        self.c=FlowTrafficController(load_settings(CONFIG),0.,120.,external_cadence=external)
        self.fid=0;self.t=-.1

    def step(self, color='red', *, dt=.1, moving=False, since=None):
        self.t=round(self.t+dt,8);t=self.t;self.fid+=1
        motion={'valid':moving is not None,'moving':moving,'captured_s':t,'frame_id':self.fid}
        if since is not None:motion['continuous_still_since_s']=since
        return self.c.step({'fresh':True,'camera_id':'dual','frame_id':self.fid,'captured_s':t,
            'image_size':[480,360],'camera_observations':{'secondary':frame('secondary',color,self.fid,t)},
            'motion':motion},t)


@pytest.mark.parametrize('external,gap',[(True,.4),(True,.6),(True,.75),(False,.4)])
def test_new_traffic_stage_locks_red_at_slow_cadence_then_waits_real_green_window(external,gap):
    run=Replay(external)
    for _ in range(15):
        row=run.step(dt=gap)
        assert row['phase']!='fault'
        if row['phase']=='wait_green':break
    assert run.c.phase=='wait_green' and run.c.red_seen
    assert run.c.approach_guard.tracks['secondary'].box is not None
    first_green=run.t+gap
    for _ in range(15):
        row=run.step('green',dt=gap)
        assert row['phase']!='fault'
        if run.t-first_green<2.-1e-9:assert row['phase']!='done'
        if row['phase']=='done':break
    assert row['phase']=='done' and row['esc_us']==1500


@pytest.mark.parametrize('gap',[.4,.6,.75])
def test_confirmed_sliding_clears_on_slow_current_continuous_stillness_then_green(gap):
    run=Replay();run.step('off');run.step('off')
    for _ in range(25):
        row=run.step()
        if row['phase']=='wait_green':break
    assert run.c.phase=='wait_green'
    for _ in range(3):row=run.step(moving=True)
    assert row['phase']=='brake_pulse' and run.c.stop_motion.sliding
    since=round(run.t+gap,8)
    for _ in range(10):
        row=run.step(dt=gap,since=since)
        assert row['phase']!='fault'
        if row['phase']=='wait_green':break
    assert row['phase']=='wait_green' and not run.c.stop_motion.sliding
    first_green=run.t+gap
    for _ in range(12):
        row=run.step('green',dt=gap,since=since)
        if run.t-first_green<2.-1e-9:assert row['phase']!='done'
        if row['phase']=='done':break
    assert row['phase']=='done' and row['brake_pulses']==2


@pytest.mark.parametrize('gap',[.4,.6,.75])
def test_camera_and_guard_both_lock_two_native_frames_at_real_result_cadence(gap):
    reader,states,now=fake_camera([signal()]*4,[signal('unknown')]*4)
    guard=TrafficApproachGuard(SETTINGS)
    try:
        for fid in range(2):
            batch=publish_read(reader,states,now,fid,fid*gap)
            filtered,info=guard.update(batch['camera_observations'],now[0],approaching=True)
        assert batch['camera_observations']['primary']['fixture_track']['locked']
        assert info['cameras']['primary']['locked'] and filtered['primary']['signal']['state']=='off'
        reader.begin_round()
        for fid in range(2,4):batch=publish_read(reader,states,now,fid,fid*gap)
        assert batch['camera_observations']['primary']['fixture_track']['locked']
    finally:reader.close()


def test_production_guard_still_rejects_long_gap_unknown_and_cross_camera_pooling():
    guard=TrafficApproachGuard(SETTINGS)
    guard.update({'primary':lamp_row('primary',1,0.)},0.,approaching=True)
    _,info=guard.update({'primary':lamp_row('primary',2,.76),
        'secondary':lamp_row('secondary',1,.76)},.76,approaching=True)
    assert not info['any_locked']
    guard.update({'primary':lamp_row('primary',3,.8,'unknown')},.8,approaching=True)
    _,info=guard.update({'primary':lamp_row('primary',4,1.)},1.,approaching=True)
    assert not info['cameras']['primary']['locked']


def sliding_after_neutral():
    detector=TrafficStopMotion(.5,max_sample_gap_s=.75)
    detector.begin_neutral(0.)
    for fid,t in enumerate((.2,.3,.4)):detector.update(True,t,fid,t)
    assert detector.sliding
    detector.begin_brake(preserve_slide=True);detector.begin_neutral(1.)
    return detector


def test_independent_stillness_only_counts_after_actual_neutral_settle():
    detector=sliding_after_neutral()
    detector.update(False,1.49,10,1.49,continuous_still_since_s=.5)
    assert detector.sliding and not detector.stationary
    detector.update(False,1.5,11,1.5,continuous_still_since_s=.5)
    assert not detector.sliding and not detector.stationary
    detector.update(False,1.7,12,1.7,continuous_still_since_s=.5)
    assert detector.stationary and detector.still_since==1.2


@pytest.mark.parametrize('moving,captured',[(True,1.3),(None,1.3),(None,None)])
def test_independent_stillness_cannot_cross_real_moving_or_sampler_unknown(moving,captured):
    detector=sliding_after_neutral()
    detector.update(moving,captured,10,1.3)
    detector.update(False,1.8,11,1.8,continuous_still_since_s=.5)
    assert detector.sliding and not detector.stationary
    detector.update(False,2.,12,2.,continuous_still_since_s=1.31)
    assert not detector.sliding and detector.stationary


def test_stale_snapshot_can_be_covered_by_new_independent_stillness():
    detector=sliding_after_neutral()
    detector.update(None,.9,10,1.4)
    detector.update(False,1.8,11,1.8,continuous_still_since_s=.5)
    assert not detector.sliding and detector.stationary


@pytest.mark.parametrize('since',[None,float('nan'),float('inf'),2.,.3])
def test_invalid_or_contradictory_still_intervals_never_clear_sliding(since):
    detector=sliding_after_neutral()
    for fid,t in enumerate((1.3,1.5,1.8),10):
        detector.update(False,t,fid,t,continuous_still_since_s=since)
        assert detector.sliding and not detector.stationary


@pytest.mark.parametrize('captured,now',[(1.3,1.6),(1.9,1.8)])
def test_stale_or_future_motion_cannot_clear_sliding_with_long_interval(captured,now):
    detector=sliding_after_neutral()
    detector.update(False,captured,10,now,continuous_still_since_s=.5)
    assert detector.sliding and not detector.stationary


def test_duplicate_capture_cannot_replace_its_stillness_interval():
    detector=sliding_after_neutral()
    detector.update(False,1.6,10,1.6,continuous_still_since_s=1.6)
    detector.update(False,1.6,10,1.65,continuous_still_since_s=.5)
    assert detector.sliding and not detector.stationary


def test_moving_streak_uses_result_cadence_but_preserves_three_frame_minimum():
    detector=TrafficStopMotion(.5,max_sample_gap_s=.75);detector.begin_neutral(0.)
    for fid,t in enumerate((.2,.6,1.)):
        detector.update(True,t,fid,t)
        assert detector.rebrake_ready==(fid==2)
    detector.begin_brake(preserve_slide=True);detector.begin_neutral(1.4)
    for fid,t in enumerate((1.6,2.4,3.2),10):
        detector.update(True,t,fid,t)
        assert not detector.rebrake_ready  # Results beyond the supported cadence cannot pool.
