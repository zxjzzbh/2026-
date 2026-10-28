"""Fresh-image quality flashes must not erase a confirmed start board.

All frames/outputs are in memory or recorded files. Never open real hardware.
"""
import copy
import json
from pathlib import Path

import pytest

from carvision.departure import DepartureSequence
from test_departure import rows,arm
from test_race_flow import bundle


def quality(t,i,camera='primary'):
    result=rows(t,i,'absent','absent')
    result[camera].update(state='unknown',scene_valid=False,unknown_cause='image_quality')
    return result


@pytest.mark.parametrize('camera',['primary','secondary'])
def test_short_quality_flash_keeps_presence_but_requires_a_full_fresh_clear_interval(camera):
    s=DepartureSequence();arm(s)
    assert not s.update(rows(.8,4,'absent','absent'),.8)
    for i,t in enumerate((1.,1.1),5):
        assert not s.update(quality(t,i,camera),t)
        assert s.phase=='wait_removed' and s.clear_frames==0 and s.clear_elapsed==0
        assert s.diagnostics()['input_issues']=={camera:'image_quality'}
    fired=[]
    for i,t in enumerate((1.2,1.4,1.6,1.8,2.),7):
        event=s.update(rows(t,i,'absent','absent'),t);fired.append(event)
        if t<1.8:assert not event
    assert sum(fired)==1 and s.phase=='complete'


def test_persistent_quality_failure_forgets_presence_and_empty_recovery_cannot_start():
    s=DepartureSequence();arm(s)
    for i in range(4,11):assert not s.update(quality(i*.2,i),i*.2)
    assert s.phase=='wait_board'
    for i in range(11,24):assert not s.update(rows(i*.2,i,'absent','absent'),i*.2)


@pytest.mark.parametrize('failure',['missing','stale','future','invalid_scene'])
def test_missing_stale_or_invalid_input_still_invalidates_immediately(failure):
    s=DepartureSequence();arm(s);sample=quality(.8,4)
    if failure=='missing':sample.pop('primary')
    elif failure=='stale':sample['primary']['captured_s']=0.
    elif failure=='future':sample['primary']['captured_s']=2.
    else:sample['primary'].pop('unknown_cause')
    assert not s.update(sample,.8) and s.phase=='wait_board'
    for i in range(5,20):assert not s.update(rows(i*.2,i,'absent','absent'),i*.2)


def test_quality_without_prior_board_confirmation_never_arms():
    s=DepartureSequence()
    for i in range(10):assert not s.update(quality(i*.1,i),i*.1)
    for i in range(10,30):assert not s.update(rows(i*.1,i,'absent','absent'),i*.1)
    assert s.phase=='wait_board'


def test_returning_after_the_quality_grace_expires_requires_a_new_board():
    s=DepartureSequence();arm(s)
    assert not s.update(quality(.8,4),.8)
    assert not s.update(rows(1.9,9,'absent','absent'),1.9)
    assert s.phase=='wait_board'


def test_duplicate_clear_frames_after_quality_recovery_never_complete():
    s=DepartureSequence();arm(s);s.update(quality(.8,4),.8)
    cached=rows(1.,5,'absent','absent')
    for t in (1.,1.1,1.2,1.3,1.4,1.5):assert not s.update(cached,t)
    assert s.phase=='wait_removed' and s.clear_frames==1


def test_capture_clock_rollback_after_quality_recovery_still_resets():
    s=DepartureSequence();arm(s);s.update(quality(.8,4),.8)
    # Fresh enough at now=.9 but older than the last accepted capture at .6.
    assert not s.update(rows(.5,5,'absent','absent'),.9)
    assert s.phase=='wait_board'


def test_real_two_frame_quality_failure_completes_once_after_valid_recovery():
    fixture=json.loads((Path(__file__).parent/'fixtures/departure-quality-dropout-20261024.json').read_text(encoding='utf-8'))
    s=DepartureSequence(fixture['clear_s']);events=[]
    for sample in fixture['rows']:
        events.append(s.update(sample['observations'],sample['now']))
        json.dumps(s.diagnostics(),allow_nan=False)
    assert sum(events)==1 and s.phase=='complete'


def test_full_flow_waits_through_quality_flash_then_hands_off_once(bundle,monkeypatch):
    from carvision.race_flow import RaceFlowController
    from test_race_flow import Frames
    c=RaceFlowController(bundle,0.);f=Frames(c);f.until('wait_board')
    for _ in range(8):f.step(board='present')
    assert c.departure.phase=='wait_removed'
    original=c.departure.update
    def bad(observations,now):
        observations=copy.deepcopy(observations)
        observations['primary'].update(state='unknown',scene_valid=False,unknown_cause='image_quality')
        return original(observations,now)
    monkeypatch.setattr(c.departure,'update',bad)
    for _ in range(2):
        out=f.step(board='absent')
        assert out['phase']=='wait_board' and out['esc_us']==1500
        assert out['departure_diagnostics']['input_issues']=={'primary':'image_quality'}
    monkeypatch.setattr(c.departure,'update',original)
    start=f.t
    out=f.until('crosswalk',board='absent')
    assert f.t-start>=.6 and c.departure.phase=='complete'
