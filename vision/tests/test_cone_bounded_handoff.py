"""Actual stalled handoffs, coherent-view recovery and early steering unwind."""
import json
from pathlib import Path

import pytest

from test_cone_latest_recovery import geometry, settings
from carvision.cone_latest_engine.cone_continuous import ContinuousConeController
from carvision.cone_latest_engine.cone_low_speed import DualLowSpeedNavigator
from carvision.cone_latest_engine.cone_steering_policy import ConeHandoff, ApproachPreview, approach_command

CASES=json.loads((Path(__file__).parent/'fixtures/cone-regression-20261107/bounded-handoff.json').read_text(encoding='utf8'))


@pytest.mark.parametrize('case',CASES,ids=['22-54-06','22-54-44','22-55-20'])
def test_recorded_next_cone_cannot_stay_in_heading_handoff_for_seconds(case):
    start=case['rows'][0]['now_s'];control=ContinuousConeController(case['settings'],start,initialized=True)
    assist_start=None;previous=None;following=[]
    for row in case['rows']:
        now=row['now_s'];intent=control.step(row['navigation'],now);g=control.progress()['image_guidance']
        if g['handoff_recovery']:
            if assist_start is None:assist_start=now
            assert now-assist_start<=.45
        elif assist_start is not None:
            assert now-assist_start<.7  # Includes one recorded control-loop interval.
            assist_start=None
        if now-start>2 and not g['handoff_recovery'] and g['target_id']:
            following.append(g['phase'])
        assert intent['esc_us']>=1570 and control.progress()['passed_count']==0
        if previous:
            t,steer=previous
            assert abs(intent['steering_us']-steer)<=600*min(.15,now-t)+1
        previous=(now,intent['steering_us'])
    assert 'avoiding' in following or 'passing' in following


@pytest.mark.parametrize('camera,y',[('primary',.71),('secondary',.61)])
def test_approaching_next_cone_releases_heading_assist_immediately(camera,y):
    handoff=ConeHandoff();handoff.begin(1.)
    handoff.advance(1.01,{'y_fraction':y},camera,fresh_target=True)
    assert not handoff.active and handoff.reason=='next_cone_approaching'


def test_no_target_or_perfect_lane_cannot_create_unbounded_handoff():
    handoff=ConeHandoff();handoff.begin(1.)
    handoff.advance(1.44,None,'secondary',fresh_target=False);assert handoff.active
    handoff.advance(1.46,None,'secondary',fresh_target=False);assert not handoff.active


def test_cached_near_target_does_not_force_urgent_release():
    handoff=ConeHandoff();handoff.begin(1.)
    handoff.advance(1.1,{'near':True},'primary',fresh_target=False)
    assert handoff.active


def test_lateral_progress_tapers_before_corridor_and_accepts_countersteering():
    preview=ApproachPreview()
    preview.update(('secondary','lane','one'),1.,.13,enabled=True)
    remaining,settling=preview.update(('secondary','lane','one'),1.1,.07,enabled=True)
    assert 0<=remaining<.07 and settling
    assert approach_command(remaining*9,-.2,settling=settling)<0
    # A prediction is not permission to switch sides or count the cone passed.
    assert remaining>=0


def test_duplicate_camera_frame_cannot_advance_approach_prediction():
    preview=ApproachPreview();key=('secondary','lane','one')
    preview.update(key,1.,.13,enabled=True)
    expected=preview.update(key,1.1,.07,enabled=True)
    for _ in range(5):assert preview.update(key,1.1,.07,enabled=True)==expected


def test_camera_target_changes_and_image_only_feedback_reset_lateral_prediction():
    preview=ApproachPreview();preview.update(('secondary','one'),1.,.13,enabled=True)
    assert preview.update(('primary','one'),1.1,.07,enabled=True)==(.07,False)
    assert preview.update(('primary','two'),1.2,.06,enabled=True)==(.06,False)
    assert preview.update(('primary','two'),1.3,.04,enabled=False)==(.04,False)


def view_case():
    views={'primary':{'cones':[{'id':'p','near':False}]},'secondary':{'cones':[{'id':'s'}]}}
    course={'cones':[{'camera':'primary','alias':'p','x':-.1,'y':1.3,'captured_s':1.},
                     {'camera':'secondary','alias':'s','x':-.06,'y':1.8,'captured_s':1.03}]}
    return views,course


def test_fresh_paired_camera_can_recover_same_estimated_target():
    views,course=view_case();navigator=object.__new__(DualLowSpeedNavigator)
    assert navigator._paired_view(views,['secondary'],'primary',course)==('primary','p','secondary','s')


@pytest.mark.parametrize('fault',['near','two_cones','different_position','stale_pair','no_pair'])
def test_ambiguous_or_near_views_cannot_invent_cross_camera_target_alias(fault):
    views,course=view_case();fresh=['secondary'];navigator=object.__new__(DualLowSpeedNavigator)
    if fault=='near':views['primary']['cones'][0]['near']=True
    if fault=='two_cones':views['secondary']['cones'].append({'id':'other'})
    if fault=='different_position':course['cones'][1]['x']=.8
    if fault=='stale_pair':course['cones'][1]['captured_s']=1.5
    if fault=='no_pair':fresh=[]
    assert navigator._paired_view(views,fresh,'primary',course) is None
