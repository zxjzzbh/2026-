"""Regressions for late avoidance, early return and phase-induced steering jumps."""
import json
from pathlib import Path

import pytest

from test_cone_latest_recovery import settings, nav, geometry
from carvision.cone_latest_engine import cone_image_guidance as guidance
from carvision.cone_latest_engine.cone_continuous import ContinuousConeController
from carvision.cone_latest_engine.cone_steering_policy import RoadHeadingControl, passing_command

RUNS=json.loads((Path(__file__).parent/'fixtures/cone-regression-20261107/late-turn-and-hold-side.json').read_text(encoding='utf8'))


def replay(record):
    start=record['rows'][0]['now_s']
    controller=ContinuousConeController(record['settings'],start,initialized=True)
    results=[]
    for row in record['rows']:
        intent=controller.step(row['navigation'],row['now_s'])
        results.append((row['now_s']-start,intent,controller.progress()))
    return results


def test_recorded_first_cone_has_effective_early_avoidance():
    rows=replay(RUNS[1])
    initial=[v for t,v,_ in rows if .1<t<.5]
    assert initial and all(v['steering_us']>=1700 for v in initial)
    assert all(v['esc_us']>=1570 for _,v,_ in rows)


@pytest.mark.parametrize('record',RUNS,ids=['21-16-58','21-18-01'])
def test_recorded_near_exit_cannot_unlock_full_right_endpoint(record):
    rows=replay(record)
    for row,(t,intent,progress) in zip(record['rows'],rows):
        if row['old_intent']['steering_phase']=='hold_side':
            assert intent['requested_steering_us']>=1441
            assert intent['steering_us']>=1441
            assert progress['passed_count']==0
        if intent['steering_phase'] in ('passing','hold_side','align','clearance_tail','recenter','boundary_recovery','transition_align'):
            assert abs(intent['requested_steering_us']-1610)<=169
    for (ta,a,_),(tb,b,_) in zip(rows,rows[1:]):
        assert abs(b['steering_us']-a['steering_us'])<=600*min(.15,tb-ta)+1


@pytest.mark.parametrize('side,cone_x,heading',[('left',.95,.1),('right',.05,-.1)])
def test_visible_side_clearance_never_requests_full_return_to_center(settings,monkeypatch,side,cone_x,heading):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda n,*a,**k:geometry(n['captured_s'],heading,.5))
    guide=guidance.ImageConeGuidance(settings)
    cone=dict(id='near',image_fraction=cone_x,half_image_width=.03,y_fraction=.96,near=True,exit_band=True)
    for t in (1.,1.1,1.2,1.3):
        result=guide.update(nav(t,[cone]),t)
        assert result['phase']=='passing' and result['side']==side
        assert abs(result['command'])<=.65 and result['target_fraction']==.5
        assert result['command']*heading>0  # Road alignment is not lateral return.
    # Missing current cone and changing coordinate domains retains the side;
    # it cannot activate a different unbounded heading calculation.
    result=guide.update({**nav(1.45),'line_missing':False,'lane_bounds_px':[[100,300],[0,400]],'ego_fraction':.5},1.45)
    assert result['side']==side and abs(result['command'])<=.65
    assert result['phase']!='recenter'


def test_direction_filter_cannot_be_advanced_by_duplicate_observations():
    control=RoadHeadingControl();g=geometry(1.,.04,.5)
    original=control.update(g)
    for _ in range(10):assert control.update({**g,'heading_correction_fraction':-.3})==original
    revised=control.update(geometry(1.1,-.3,.5))
    assert abs(revised-original)<.30


def test_camera_change_resets_derivative_without_turning_to_endpoint():
    control=RoadHeadingControl();control.update(geometry(1.,.04,.5))
    g={**geometry(1.1,-.3,.5),'camera':'primary'}
    assert abs(control.update(g))<=.65
    assert abs(passing_command(control.command,'right'))<=.65


def test_missing_current_target_does_not_count_as_passage(settings,monkeypatch):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda n,*a,**k:geometry(n['captured_s'],0.,.5))
    control=ContinuousConeController(settings,1.,initialized=True)
    cone=dict(id='first',image_fraction=.92,half_image_width=.03,y_fraction=.96,near=True,exit_band=True)
    control.step({**nav(1.,[cone]),'experimental':True},1.)
    for t in (1.1,1.3,1.5,1.7):
        result=control.step({**nav(t),'experimental':True},t)
        assert result['steering_phase']!='recenter'
        assert control.progress()['passed_count']==0
