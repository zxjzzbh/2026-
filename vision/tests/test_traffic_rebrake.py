"""Traffic supplementary braking: transient rejection and finite recovery."""
import copy

import pytest

from carvision.traffic_driving import load_settings, validate_settings
from carvision.traffic_stop_motion import TrafficStopMotion
from test_traffic_driving import CONFIG, Run
from test_traffic_stop_hold import stop_with_unknown_motion


def detector():
    motion = TrafficStopMotion(.5)
    motion.begin_neutral(0.)
    return motion


@pytest.mark.parametrize('value', [0,7,-1,True,6.5,'6',None])
def test_invalid_brake_budgets_remain_rejected(value):
    settings = load_settings(CONFIG)
    with pytest.raises(ValueError):
        validate_settings({**settings,'max_brake_pulses':value})


def test_default_budget_is_six_and_custom_lower_budget_remains_valid():
    assert load_settings(CONFIG)['max_brake_pulses'] == 6
    assert validate_settings({**load_settings(CONFIG),'max_brake_pulses':2})['max_brake_pulses'] == 2


def test_post_brake_transients_are_ignored_before_actual_neutral_settle_end():
    motion = detector()
    for frame,t in enumerate((.01,.07,.14,.19)):
        motion.update(True,t,frame,t)
        assert not motion.blocking and not motion.rebrake_ready
    for frame,t in enumerate((.2,.3,.4),10):
        motion.update(True,t,frame,t)
        assert motion.rebrake_ready == (t==.4)


def test_three_frames_also_need_two_hundred_ms_of_evidence():
    motion=detector()
    for frame,t in enumerate((.2,.23,.26)):
        motion.update(True,t,frame,t)
        assert not motion.rebrake_ready
    motion.update(True,.4,3,.4)
    assert motion.rebrake_ready and motion.sliding


def test_same_frame_cannot_accumulate_votes_even_if_outer_timestamp_changes():
    motion=detector()
    for t in (.2,.3,.4,.5):
        motion.update(True,t,10,t)
        assert motion.moving_frames==1 and not motion.rebrake_ready
    motion.update(True,.6,11,.6)
    assert motion.moving_frames==1  # Gap from last accepted observation resets votes.


def test_duplicate_confirmed_frame_cannot_issue_another_brake_request():
    motion=detector()
    for frame,t in enumerate((.2,.3,.4)): motion.update(True,t,frame,t)
    assert motion.rebrake_ready
    motion.update(True,.4,2,.45)
    assert motion.sliding and not motion.rebrake_ready


@pytest.mark.parametrize('separator', [False,None])
def test_still_or_unknown_frame_breaks_moving_streak(separator):
    motion=detector()
    motion.update(True,.2,1,.2)
    motion.update(True,.3,2,.3)
    motion.update(separator,.4,3,.4)
    motion.update(True,.5,4,.5)
    assert motion.moving_frames==1 and not motion.sliding


def test_stale_or_reversed_frame_never_confirms_motion():
    motion=detector()
    motion.update(True,.2,2,.2)
    motion.update(True,.1,1,.3)
    motion.update(True,.2,2,.6)
    assert not motion.rebrake_ready and not motion.sliding


def test_confirmed_motion_survives_unknown_and_requires_continuous_still_to_clear():
    motion=detector()
    for frame,t in enumerate((.2,.3,.4)): motion.update(True,t,frame,t)
    motion.update(None,.5,3,.5)
    assert motion.sliding and not motion.rebrake_ready
    for frame,t in enumerate((.6,.7,.8),4):
        motion.update(False,t,frame,t)
        assert motion.sliding
    motion.update(False,.9,7,.9)
    assert not motion.sliding and not motion.stationary
    motion.update(False,1.,8,1.)
    motion.update(False,1.1,9,1.1)
    assert motion.stationary


def test_every_pulse_needs_new_post_neutral_evidence():
    motion=detector()
    for frame,t in enumerate((.2,.3,.4)): motion.update(True,t,frame,t)
    motion.begin_brake(preserve_slide=True)
    for frame,t in enumerate((.5,.6,.7),3):
        motion.update(True,t,frame,t)
        assert motion.sliding and not motion.rebrake_ready
    motion.begin_neutral(.8)
    motion.update(True,.9,6,.9)
    assert not motion.rebrake_ready
    for frame,t in enumerate((1.,1.1,1.2),7): motion.update(True,t,frame,t)
    assert motion.rebrake_ready


def test_controller_does_not_rebrake_on_cached_frame_or_short_jitter():
    run=stop_with_unknown_motion()
    stamp=run.t+.05
    cached={'valid':True,'moving':True,'captured_s':stamp,'frame_id':55}
    for _ in range(4):
        row=run.step('red',dt=.05,motion=copy.deepcopy(cached))
        assert row['action']=='neutral' and row['brake_pulses']==1
    for _ in range(10):
        for moving in (True,False):
            row=run.step('red',moving=moving)
            assert row['phase']=='wait_green' and row['brake_pulses']==1
    assert run.until('done','green',moving=False)['brake_pulses']==1


def test_pending_moving_evidence_blocks_green_then_confirms_a_brake():
    run=stop_with_unknown_motion()
    for _ in range(20):
        row=run.step('green',moving=False)
        assert row['phase']=='wait_green'
    for _ in range(2):
        row=run.step('green',moving=True)
        assert row['phase']=='wait_green' and not row['events']
        assert row['supplementary_brake_evidence']['blocking']
    row=run.step('green',moving=True)
    assert row['action']=='brake' and row['brake_pulses']==2
    evidence=row['events'][0]['motion_evidence']
    assert evidence['moving_frames']==3 and evidence['observed_span_s']>=.2-1e-9


def test_only_six_pulses_with_continuous_motion_and_no_release_on_green():
    run=Run('red')
    pulses=[]
    for _ in range(130):
        row=run.step('red',moving=True,dt=.1)
        if any(e['event']=='brake_requested' for e in row['events']):pulses.append(run.t)
        assert row['esc_us']<=1500
        if row['phase']=='fault':break
    assert len(pulses)==6 and row['brake_pulses']==6
    assert row['phase']=='fault' and '次数上限' in row['reason']
    assert all(b-a>=.75-1e-9 for a,b in zip(pulses,pulses[1:]))
    for _ in range(25):
        row=run.step('green',moving=False)
        assert row['phase']=='fault' and row['esc_us']==1500


def test_first_red_still_brakes_in_same_control_batch():
    run=Run('off');run.until('approach',moving=True)
    assert run.step('red',moving=True)['action']=='brake'
