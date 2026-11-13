"""A route lock must never turn an excluded gap into a drivable corridor."""
import json
from pathlib import Path

import pytest

from test_cone_latest_recovery import settings
from carvision.cone_latest_engine.cone_side_choice import ConeRouteChoice
from carvision.cone_latest_engine.cone_continuous import ContinuousConeController

CHOICES=[(.1,'left',.18,.35,.35),(.1,'right',.65,.82,.65)]
CASES=json.loads((Path(__file__).parent/'fixtures/cone-regression-20261107/route-lock-right-gap.json').read_text(encoding='utf8'))


def choose(route,t,left=.6,right=.6,*,choices=CHOICES,spaces=True,**kw):
    args=dict(camera='secondary',captured=t,target=dict(x=.5,y_fraction=.55),ego=.5,applied=.5)
    args.update(kw)
    return route.choose(choices,dict(left_gap_m=left,right_gap_m=right) if spaces else None,**args)


def test_first_choice_is_provisional_and_duplicate_frame_cannot_lock(settings):
    route=ConeRouteChoice(settings)
    c,r=choose(route,1.)
    assert c[1]=='right' and r['provisional'] and not r['locked']
    for _ in range(4):
        c,r=choose(route,1.)
        assert not r['locked'] and r['confirmation_hits']==1
    c,r=choose(route,1.1)
    assert c[1]=='right' and r['locked'] and not r['provisional']


def test_narrow_right_gap_cannot_win_twenty_centimetre_tie(settings):
    route=ConeRouteChoice(settings)
    c,r=choose(route,1.,left=.46,right=.29)
    assert c[1]=='left' and r['preferred_side']=='right'
    assert r['blocked_sides']==['right'] and r['available_sides']==['left']


def test_invalid_route_revokes_commitment_then_confirms_left(settings):
    route=ConeRouteChoice(settings)
    choose(route,1.);choose(route,1.1)
    assert route.committed
    c,r=choose(route,1.2,left=.89,right=.29)
    assert c is None and not route.committed and not r['locked']
    c,r=choose(route,1.3,left=.89,right=.29)
    assert c[1]=='left' and r['selected_side']=='left'
    # Uncalibrated fallback cannot reopen the proven-too-narrow right gap.
    for t in (1.4,1.5,1.6):
        c,r=choose(route,t,spaces=False)
        assert c[1]=='left' and 'right' not in r['available_sides']


def test_image_only_preview_without_issued_steering_remains_reviewable(settings):
    route=ConeRouteChoice(settings)
    choose(route,1.,spaces=False,applied=0.);choose(route,1.1,spaces=False,applied=0.)
    assert not route.committed
    c,r=choose(route,1.2,left=.89,right=.29)
    assert c is None and not r['locked']
    c,r=choose(route,1.3,left=.89,right=.29)
    assert c[1]=='left'


def test_late_opposite_path_cannot_cross_cone_or_retain_excluded_side(settings):
    route=ConeRouteChoice(settings)
    choose(route,1.);choose(route,1.1)
    for t in (1.2,1.3):
        c,r=choose(route,t,left=.89,right=.29,target=dict(x=.5,y_fraction=.95),ego=.8)
        assert c is None and r['selected_side'] is None
    assert r['reason']=='near_cone_crossing_not_supported'


def test_no_corridor_does_not_fabricate_lane_edge_goal(settings):
    route=ConeRouteChoice(settings);choose(route,1.);choose(route,1.1)
    c,r=choose(route,1.2,choices=[])
    assert c is None and r['available_sides']==[] and not route.committed


def test_held_target_cannot_confirm_new_side(settings):
    route=ConeRouteChoice(settings);choose(route,1.)
    c,r=choose(route,1.1,held=True)
    assert not route.confirmed and r['confirmation_hits']==1


def test_new_physical_target_discards_previous_gap_exclusions(settings):
    route=ConeRouteChoice(settings);choose(route,1.,left=.89,right=.29)
    route.reset()
    c,r=choose(route,1.1,left=.29,right=.89)
    assert c[1]=='right' and r['blocked_sides']==['left']


@pytest.mark.parametrize('case',CASES,ids=['23-27-07','23-27-45','23-28-52','23-29-27'])
def test_recorded_choices_are_feasible_and_outputs_keep_floor_and_slew(case):
    start=case['rows'][0]['now_s'];control=ContinuousConeController(case['settings'],start,initialized=True)
    previous=None;corrected=[]
    for row in case['rows']:
        now=row['now_s'];intent=control.step(row['navigation'],now);g=control.progress()['image_guidance']
        selection=g.get('side_choice') or {}
        if g['side'] is not None:
            assert g['side'] in selection['available_sides']
            assert g['side'] not in selection['blocked_sides']
        assert intent['esc_us']>=1570 and control.progress()['passed_count']==0
        if previous:
            t,steer=previous
            assert abs(intent['steering_us']-steer)<=600*min(.15,now-t)+1
        previous=(now,intent['steering_us'])
        if case['name'].endswith('1794065367294961667') and 3.5<now-start<4.05:
            corrected.append(intent)
            assert g['side']=='left' and 'right' in selection['blocked_sides']
            assert intent['requested_steering_us']>1610 and intent['steering_us']>1610
    if case['name'].endswith('1794065367294961667'):assert len(corrected)>=3
