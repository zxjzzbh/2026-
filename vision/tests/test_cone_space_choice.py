"""Operator's 117 cm lane, wider-side selection and strict 20 cm right tie."""
import pytest

from test_cone_latest_recovery import settings,geometry,nav
from carvision.cone_latest_engine import cone_image_guidance as guidance
from carvision.cone_latest_engine.cone_side_choice import side_spaces,choose_corridor

CHOICES=[(.1,'left',.2,.3,.3),(.4,'right',.7,.8,.7)]


@pytest.mark.parametrize('left,right,expected',[(.60,.60,'right'),(.65,.4501,'right'),
    (.65,.45,'left'),(.6501,.45,'left'),(.45,.65,'right'),(.6,.5,'right')])
def test_wider_gap_and_strict_twenty_centimetre_rule(left,right,expected):
    choice,report=choose_corridor(CHOICES,dict(left_gap_m=left,right_gap_m=right))
    assert choice[1]==expected and report['selected_side']==expected


def test_right_preference_never_selects_a_missing_vehicle_corridor():
    choice,report=choose_corridor(CHOICES[:1],dict(left_gap_m=.55,right_gap_m=.50))
    assert choice[1]=='left' and report['preferred_side']=='right'
    assert report['reason']=='preferred_side_has_no_vehicle_corridor'


def test_unknown_scale_does_not_compare_pixels_with_twenty_centimetres():
    choice,report=choose_corridor([(.1,'left',.1,.3,.3),(.4,'right',.7,.8,.7)],None)
    assert choice[1]=='left' and report['difference_m'] is None
    assert report['left_gap_m'] is None


def lane(t,cone):
    return {**nav(t,[cone]),'line_missing':False,'lane_bounds_px':[[100,380],[0,480]],
            'lane_rows':[200,300],'ego_fraction':.45}


def cone(x=.5):
    return dict(id='one',fraction=x,half_width=.03,image_fraction=x,
                half_image_width=.03,y_fraction=.75,near=True)


def test_current_pair_uses_operator_track_width(settings):
    assert settings['geometry']['lane_width_m']==1.17
    result=side_spaces(lane(1.,cone()),cone(),settings,1.)
    assert result['left_gap_m']==pytest.approx(.47*1.17)
    assert result['right_gap_m']==pytest.approx(.47*1.17)
    assert not result['metric_verified']


@pytest.mark.parametrize('flags',[{'line_held':True},{'single_edge_observed':True},{'line_missing':True}])
def test_held_or_single_edge_cannot_invent_two_metric_gaps(settings,flags):
    assert side_spaces({**lane(1.,cone()),**flags},cone(),settings,1.) is None


def test_stale_pair_cannot_set_new_side(settings):
    assert side_spaces(lane(1.,cone()),cone(),settings,1.6) is None


def test_raw_lane_and_cone_must_come_from_same_camera(settings):
    n=nav(1.,[cone()]);n['local_course']={'image_lane':[dict(camera='primary',captured_s=1.,
        left=[[100,200+i*15] for i in range(7)],right=[[380,200+i*15] for i in range(7)])]}
    assert side_spaces(n,cone(),settings,1.) is None
    n['local_course']['image_lane'][0]['camera']='secondary'
    result=side_spaces(n,cone(),settings,1.)
    assert result is not None and result['left_gap_m']==pytest.approx(result['right_gap_m'])


def test_new_cone_chooses_space_not_smallest_steering_displacement(settings,monkeypatch):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda n,*a,**k:geometry(n['captured_s'],0.,.45))
    guide=guidance.ImageConeGuidance(settings)
    result=guide.update(lane(1.,cone(.52)),1.)
    assert result['side']=='right'  # Left is nearer, but gap difference is <20 cm.
    assert result['side_choice']['preference_reason']=='under_20cm_prefer_right'
    assert not result['side_choice']['locked']
    assert result['side_choice']['difference_m']==pytest.approx(.04*1.17)


def test_started_manoeuvre_keeps_side_and_next_cone_is_reassessed(settings,monkeypatch):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda n,*a,**k:geometry(n['captured_s'],0.,.5))
    guide=guidance.ImageConeGuidance(settings)
    first=cone(.45);first['y_fraction']=.95;first['exit_band']=True
    assert guide.update(lane(1.,first),1.)['side']=='right'
    guide.applied_command=.5
    changed={**first,'fraction':.46,'image_fraction':.46}
    locked=guide.update(lane(1.1,changed),1.1)
    assert locked['side']=='right' and locked['side_choice']['locked']
    second={**changed,'id':'second','fraction':.7,'image_fraction':.7,'y_fraction':.65,'exit_band':False}
    result=guide.update(lane(2.,second),2.)
    assert result['target_id']=='second' and result['side']=='left'
    assert result['side_choice']['preference_reason']=='larger_gap'
