"""Production race/debug policy, with independent stripe pixels and no I/O."""
import copy
import numpy as np
import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_low_speed import LowSpeedNavigator
from carvision.race_flow import RaceFlowController
from test_cone_turn_controller import lane, settings
from test_cone_low_speed import row
from test_race_flow import Frames, bundle


def cone(x=.5, **changes):
    return dict(id='secondary-one', fraction=x, half_width=.03,
                near=False, y_fraction=.55, **changes)


def warmed(s=None):
    c=ContinuousConeController(s or settings(),0.,initialized=True,stop_for_cones=False)
    for t in (0.,.1,.2):c.step(lane(t),t)
    return c


@pytest.mark.parametrize('x,expected',[(.5,'right'),(.51,'right'),(.55,'left'),(.60,'left'),(.40,'right')])
def test_real_race_flow_uses_current_space_and_20cm_right_tie(bundle,x,expected):
    bundle=copy.deepcopy(bundle)
    bundle['cones']['geometry']['lane_width_m']=1.17
    c=RaceFlowController(bundle,0.);c._go('cones',0.,'test');frames=Frames(c)
    for _ in range(3):frames.step(nav_override=lambda t,i:lane(t,fid=i))
    out=frames.step(nav_override=lambda t,i:lane(t,fid=i,cones=[cone(x)]))
    assert c.controller.side==expected
    assert out['esc_us']>=1570
    report=c.controller.progress()['route_choice']
    assert not report['metric_verified']
    assert report['source']=='current_same_camera_edges_scaled_by_track_width'


def mapped_settings():
    s=settings();s['geometry']['lane_width_m']=1.17
    s['ground'].update(image_size=[480,360],coordinate_frame='rear_axle_x_right_y_forward',
        homography=[[.01,0,-2.4],[0,-.01,4],[0,0,1]])
    return s


def stripe(t,*,camera='secondary',right_x=280):
    return {'camera':camera,'captured_s':t,'line_held':False,
            'right':[[right_x,float(y)] for y in np.linspace(180,260,12)],
            'left':[[163,float(y)] for y in np.linspace(180,260,12)]}


def test_right_under_50cm_uses_current_secondary_pixels_to_prefer_left():
    c=warmed(mapped_settings())
    data=lane(.3,cones=[cone()],near=.65,far=.65,
              local_course={'image_lane':[stripe(.3)]})
    out=c.step(data,.3)
    assert c.side=='left' and out['steering_us']>1610
    report=c.progress()['right_boundary']
    assert report['available'] and report['right_distance_m']==pytest.approx(.4)
    assert not report['metric_verified']  # retained mapping is not new validation


@pytest.mark.parametrize('edge',[None,stripe(.3,camera='primary'),stripe(-1.)])
def test_fraction_or_other_camera_or_old_stripe_cannot_fake_50cm(edge):
    c=warmed(mapped_settings())
    data=lane(.3,cones=[cone()],near=.65,far=.65,
              local_course={'image_lane':[] if edge is None else [edge]})
    c.step(data,.3)
    assert c.side=='right'
    assert not c.progress()['right_boundary']['available']


def test_right_warning_without_feasible_left_does_not_drive_outward():
    c=warmed(mapped_settings())
    out=c.step(lane(.3,cones=[cone(.30)],local_course={'image_lane':[stripe(.3)]}),.3)
    assert out['esc_us']==1500
    assert c.progress()['route_choice']['reason']=='right_boundary_no_feasible_left'


def test_lost_locked_corridor_pauses_and_reconfirms_far_side_without_old_throttle():
    c=warmed();outputs=[]
    for i,x in enumerate((.45,.48,.51,.54,.57,.60)):
        t=.3+.1*i
        outputs.append(c.step(lane(t,cones=[cone(x)],near=.65,far=.65),t))
    assert outputs[2]['esc_us']>=1570
    assert outputs[3]['esc_us']==1500  # right endpoint ceased to exist
    assert c.side=='left' and c.count==0
    assert all(o['esc_us']==1500 or o['esc_us']>=1570 for o in outputs)
    assert all(o['esc_us']==1500 for o in outputs[3:] if o['steering_us']<1610)
    assert not any(o['best_effort_route'] for o in outputs)


def test_near_cone_cannot_trigger_crossing_to_new_side_or_count_as_pass():
    c=warmed()
    c.step(lane(.3,cones=[cone(.45)],near=.65,far=.65),.3)
    for i in range(6):
        t=.4+i*.1
        out=c.step(lane(t,cones=[cone(.57, image_close=True)],near=.65,far=.65),t)
        assert out['esc_us']==1500
        assert c.progress()['route_choice']['reason']=='near_cone_crossing_not_supported'
    assert c.side=='right' and c.count==0


def test_same_capture_does_not_confirm_replacement_corridor():
    c=warmed()
    c.step(lane(.3,cones=[cone(.45)],near=.65,far=.65),.3)
    data=lane(.4,cones=[cone(.57)],near=.65,far=.65)
    c.step(data,.4)
    for t in (.42,.44,.46):c.step(data,t)
    assert c.side=='right' and c.progress()['route_choice']['confirmation_hits']==1


def test_raw_paired_stripes_are_preserved_as_current_camera_measurements():
    s=mapped_settings();source=row(1.,1)
    nav=LowSpeedNavigator(s).update(np.zeros((360,480,3),np.uint8),source,1.)
    observed=nav['local_course']['image_lane'][0]
    assert observed['camera']=='secondary' and observed['captured_s']==1.
    assert observed['right']==source['road_context']['right_inner']
    assert observed['left']==source['road_context']['left_inner']
