"""Real first-cone reversals, neutral priming and physical-target continuity."""
import json
from pathlib import Path

import pytest

from carvision.cone_latest_engine.cone_maneuver_state import ConeAliases,ConeReturnGuard
from carvision.cone_latest_engine.cone_continuous import ContinuousConeController
from carvision.cone_latest_engine.cone_side_choice import ConeRouteChoice
from test_cone_latest_recovery import settings,nav
from test_cone_route_lock import choose,CHOICES

CASES=json.loads((Path(__file__).parent/'fixtures/cone-regression-20261108/first-cone-continuity.json').read_text(encoding='utf8'))


def cone(name,x=.55,y=.65):
    return dict(id=name,image_fraction=x,half_image_width=.03,y_fraction=y)


def test_known_physical_target_survives_camera_round_trip_and_new_alias():
    aliases=ConeAliases();p=cone('p');s=cone('s')
    aliases.remember('primary',p,1.)
    hint=dict(from_camera='primary',from_id='p',to_camera='secondary',to_id='s')
    assert aliases.match([s],'secondary',1.2,hint)==(s,'paired_view_alias_retained')
    aliases.remember('secondary',s,1.2)
    assert aliases.match([p],'primary',1.9)[0]==p
    aliases.remember('primary',p,1.9)
    next_s=cone('s-new')
    assert aliases.match([next_s],'secondary',2.,{**hint,'to_id':'s-new'})[0]==next_s


def test_ambiguous_reidentification_and_expired_aliases_are_not_merged():
    aliases=ConeAliases();aliases.remember('secondary',cone('old'),1.)
    assert aliases.match([cone('a',.50),cone('b',.60)],'secondary',1.2)==(None,None)
    assert aliases.match([cone('old')],'secondary',6.1)==(None,None)
    aliases.reset()
    assert aliases.match([cone('old')],'secondary',1.3)==(None,None)


def test_issued_image_space_turn_is_not_reversed_by_later_metric_tie(settings):
    route=ConeRouteChoice(settings)
    choose(route,1.,choices=CHOICES[:1],spaces=False,applied=0.)
    choose(route,1.1,choices=CHOICES[:1],spaces=False,applied=-.3)
    assert route.committed and route.side=='left'
    c,r=choose(route,1.2,left=.65,right=.52,applied=-.5)
    assert c[1]=='left' and r['locked'] and r['preferred_side']=='right'
    # A physically excluded route still loses its lock.
    c,r=choose(route,1.3,left=.25,right=.9,applied=-.5)
    assert c is None and 'left' in r['blocked_sides'] and not r['locked']


@pytest.mark.parametrize('side,sign',[('left',1),('right',-1)])
def test_image_side_clearance_and_early_exit_flag_do_not_unlock_strong_return(side,sign):
    guard=ConeReturnGuard()
    target={**cone('one',.95 if side=='left' else .05,.76),'near':True,'exit_band':True}
    for t in (1.,1.1,1.2):
        report=guard.observe(target,'secondary','image_fraction',t,t,side,fresh=True,clear=True)
        assert guard.constrain(sign*.65,side)==pytest.approx(sign*.20)
        assert not report['near_side_observed'] and not report['body_passed']


def test_duplicate_or_missing_frames_never_unlock_more_countersteering():
    guard=ConeReturnGuard();target=cone('one',.95,.99)
    guard.observe(target,'secondary','image',1.,1.,'left',fresh=True,clear=True)
    for t in (1.05,1.1,1.2):
        report=guard.observe(target,'secondary','image',1.,t,'left',fresh=True,clear=True)
        assert report['counter_limit']==.20 and not report['near_side_observed']
    report=guard.observe(target,'secondary','image',1.3,1.3,'left',fresh=True,clear=True)
    assert report['counter_limit']==pytest.approx(.45) and report['near_side_observed']
    report=guard.observe(None,'secondary','image',2.2,2.2,'left',fresh=False,clear=False)
    assert report['counter_limit']==.20 and not report['body_passed']


def test_current_close_white_line_can_override_return_limit_but_preview_alone_cannot():
    guard=ConeReturnGuard()
    guard.observe(cone('one'),'secondary','lane',1.,1.,'left',fresh=True,clear=False)
    far=dict(near_fraction=.40,preview_fraction=.10,left_risk=1.,center_margin_fraction=.20)
    assert guard.constrain(.65,'left',far)==pytest.approx(.20)
    assert guard.constrain(.65,'left',{**far,'near_fraction':.21})==.65
    assert guard.report['current_boundary_override']
    assert guard.constrain(-.65,'right',dict(observed_edge_side='right',edge_clearance_fraction=.10))==-.65


def test_neutral_arming_primes_choice_without_outputs_or_motion_progress(settings):
    control=ContinuousConeController(settings,1.)
    target=cone('one',.60)
    for t in (1.,1.1,1.2,2.,3.):
        result=control.step({**nav(t,[target]),'experimental':True},t)
        assert result['phase']=='arming' and (result['esc_us'],result['steering_us'])==(1500,1610)
        assert not control.guidance.route_choice.committed
        assert control.guidance.corridor_memory is None
        assert control.guidance.no_direction_since is None
    assert control.guidance.route_choice.confirmed
    result=control.step({**nav(4.,[target]),'experimental':True},4.)
    assert result['phase']=='driving' and result['esc_us']>=1570
    assert control.progress()['side']=='left'


def replay(case):
    driving=[r for r in case['rows'] if r['phase']=='driving'];start=driving[0]['now_s']
    control=ContinuousConeController(case['settings'],case['rows'][0]['now_s'])
    control.arm_until=start
    results=[];previous=None
    for row in case['rows']:
        intent=control.step(row['navigation'],row['now_s'])
        if row['phase']=='arming':
            assert (intent['esc_us'],intent['steering_us'])==(1500,1610)
            continue
        result=control.progress()['image_guidance']
        assert intent['esc_us']>=1570 and control.progress()['passed_count']==0
        if previous:
            t,steer=previous
            assert abs(intent['steering_us']-steer)<=600*min(.15,row['now_s']-t)+1
        previous=(row['now_s'],intent['steering_us'])
        results.append((row['now_s']-start,intent,result))
    return results


def test_recorded_first_cone_keeps_left_route_through_aliases_and_twenty_cm_tie():
    results=replay(CASES[0])
    early=[g for t,_,g in results if t<2.2]
    assert early and all(g['side']=='left' for g in early)
    assert sum(g['identity_transition']=='new_target' for g in early)<=1
    lost=[(intent,g) for t,intent,g in results if .5<t<.95]
    assert lost and all(g['phase']!='clearance_tail' for _,g in lost)
    assert all(intent['requested_steering_us']>=1558 for intent,_ in lost)
    assert any(len(g['target_aliases'])>=3 for g in early)


def test_recorded_first_cone_does_not_release_strong_return_while_still_ahead():
    results=replay(CASES[1])
    passing=[(intent,g) for t,intent,g in results if 1.35<t<1.7]
    assert passing and all(g['side']=='left' for _,g in passing)
    assert all(intent['requested_steering_us']>=1558 for intent,_ in passing)
    assert all(not g['return_guard']['body_passed'] for _,g in passing)
