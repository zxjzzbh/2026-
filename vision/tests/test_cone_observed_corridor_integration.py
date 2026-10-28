"""Code-derived geometry counterexamples, not reconstructed accident imagery.

943/948 did not record raw tracks/images. These synthetic observed edge points
exercise the reported fitting mechanism without inventing either session's data.
"""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from carvision.cone_corridor import measured_corridor
from carvision.cone_continuous import ContinuousConeController
from carvision.cone_low_speed import DualLowSpeedNavigator,LowSpeedNavigator,trial_settings
from carvision.cone_planning import load_settings
from carvision.race_flow import RaceFlowController,load_bundle
from carvision.race_flow_console import RaceFlowDrive
from carvision.shared_executor import BorrowedActuator
from test_shared_executor import CLIENT,Worker,rig

ROOT=Path(__file__).resolve().parents[2]
IMAGE=np.zeros((360,480,3),np.uint8)


def settings():
    return trial_settings(load_settings(ROOT/'vision/tests/fixtures/configs/cone-trial-field-20261026.json'))


def x0(value,y):
    H=np.asarray(value['ground']['homography'],float)
    return -(H[0,1]*np.asarray(y)+H[0,2])/H[0,0]


def observed_context(value,*,wide=False,legacy=False):
    ys=np.linspace(180,260 if wide else 220,12)
    widths=np.linspace(80,350,len(ys)) if wide else 170-.04*(ys-180)**2
    middle=x0(value,ys)
    return {'valid':True,'turn_aware':not legacy,'ground_supported':True,
        'left_inner':np.column_stack((middle-widths/2,ys)).tolist(),
        'right_inner':np.column_stack((middle+widths/2,ys)).tolist(),
        'boundary_candidates':[]}


def row(value,t,fid,*,foot_y=None,wide=False,legacy=False):
    tracks=[]
    if foot_y is not None:
        x=float(x0(value,foot_y)+20)
        tracks=[{'id':'secondary-A','visible':True,'confirmed':True,'seen_s':t,
                 'identity_revision':0,
                 'kind':'cone_candidate','ground_contact_usable':True,'clipped':False,
                 'foot_pixel':[x,foot_y],'bbox_xyxy':[x-10,foot_y-20,x+10,foot_y+5]}]
    return {'captured_s':t,'frame_id':fid,'tracks':tracks,
            'road_context':observed_context(value,wide=wide,legacy=legacy)}


def test_valid_observed_curve_never_extrapolates_a_cone_foot_into_whole_lane_failure():
    value=settings();data=row(value,1.,1,foot_y=260.)
    model=measured_corridor(data['road_context'],IMAGE.shape)
    # The old short extension admitted y260; its quadratic extrapolation has
    # width -86 there, although every actual observed width is 106..170 px.
    assert 170-.04*(260-180)**2==-86
    with pytest.raises(ValueError):model.bounds(260.)
    blank=LowSpeedNavigator(value).update(IMAGE,row(value,1.,1),1.)
    cone=LowSpeedNavigator(value).update(IMAGE,data,1.)
    assert blank['line_state']==cone['line_state']=='paired'
    assert blank['lane_bounds_px']==cone['lane_bounds_px']
    assert cone['boundary_observed'] and not cone.get('line_missing')
    assert not cone.get('boundary_failure_reason') and cone['cones']==[]
    assert cone['obstacle_diagnostics'][0]['reason']=='out_of_observed_band'
    pixel=cone['visual_cones'][0]
    assert pixel['id']=='secondary-A' and pixel['current_pixel'] and pixel['confirmed']
    assert pixel['frame_id']==1 and pixel['captured_s']==1. and pixel['source_camera']=='secondary'
    assert 'fraction' not in pixel and 'reference_distance_m' not in pixel
    assert not cone['metric_boundary_verified']


def test_smooth_positive_observed_perspective_width_above_three_is_a_valid_pair():
    value=settings();data=row(value,1.,1,wide=True)
    model=measured_corridor(data['road_context'],IMAGE.shape)
    widths=[model.bounds(y)[1]-model.bounds(y)[0] for y in np.linspace(model.low,model.high,9)]
    assert max(widths)/min(widths)==pytest.approx(350/80)
    nav=LowSpeedNavigator(value).update(IMAGE,data,1.)
    assert nav['line_state']=='paired' and nav['boundary_observed']
    assert not nav.get('line_missing') and not nav.get('boundary_failure_reason')


def test_legacy_context_keeps_its_existing_short_extension_behavior():
    value=settings();nav=LowSpeedNavigator(value).update(IMAGE,row(value,1.,1,foot_y=260.,legacy=True),1.)
    assert nav['line_state']=='paired' and not nav.get('line_missing')
    assert nav['cones'] and nav['cones'][0]['id']=='secondary-A'
    assert 'visual_cones' not in nav


@pytest.mark.parametrize('invalid',['cross','nonfinite','too_narrow','outside_image','unconfirmed_raw'])
def test_removing_perspective_ratio_does_not_accept_invalid_or_unknown_boundaries(invalid):
    value=settings();data=row(value,1.,1)
    road=data['road_context']
    if invalid=='cross':road['right_inner']=[[x-200,y] for x,y in road['left_inner']]
    elif invalid=='nonfinite':road['left_inner'][5][0]=float('nan')
    elif invalid=='too_narrow':road['right_inner']=[[x+60,y] for x,y in road['left_inner']]
    elif invalid=='outside_image':road['left_inner']=[[x-500,y] for x,y in road['left_inner']]
    else:road.update(valid=False,ground_supported=False)
    with pytest.raises(ValueError):measured_corridor(road,IMAGE.shape)
    nav=LowSpeedNavigator(value).update(IMAGE,data,1.)
    assert nav['line_state']=='missing' and nav.get('line_missing') and not nav['boundary_observed']
    controller=ContinuousConeController(value,1.,initialized=True)
    assert controller.step(nav,1.)['esc_us']==1500
    assert not controller.has_forward_command and controller.count==0


def test_outside_observed_band_can_extend_only_existing_lock_through_real_race_worker(rig):
    now,clock,backend,manual,owner,_=rig
    value=settings();navigator=DualLowSpeedNavigator(value)
    worker=Worker(owner)
    view=SimpleNamespace(borrow_actuator=lambda options,client:BorrowedActuator(worker,options,client,clock=clock))
    bundle=load_bundle(ROOT)
    handle=RaceFlowDrive._make_actuator(SimpleNamespace(drive=view,owner=CLIENT,live=False),bundle)
    flow=RaceFlowController(bundle,0.,initialized=True);flow._go('cones',0.,'offline observed-band integration')
    for fid,t in enumerate((0.,.1,.2,.3,.4)):
        data=row(value,t,fid,foot_y=210. if fid==3 else 260. if fid==4 else None)
        nav=navigator.update_views({'secondary':IMAGE},{'secondary':data},t)
        output=flow.step({'sequence':fid,'captured_s':t,'navigation':nav},t)
        now[0]=t;handle.send(output)
        assert flow.stage=='cones' and not owner.guard.fault
        assert flow.controller.count==0 and not flow.controller.passed_ids
    assert nav['line_state']=='paired' and not nav['cones']
    assert output['target_id']=='secondary-A' and output['target_space']=='image'
    assert output['esc_us']>1500 and not output['tail_clearance_verified']
    assert owner.guard.esc==output['esc_us']
    # Changing projection scope does not extend the independently frozen lease.
    now[0]=.601;owner.tick()
    assert not owner.active and owner.guard.esc==1400
    now[0]=.952;owner.tick();assert owner.guard is None
    assert all(entry[3]<=.2 for entry in backend.writes if entry[0]=='motion')


def test_an_unacquired_outside_band_pixel_is_never_a_new_lane_target_or_pass():
    value=settings();navigator=DualLowSpeedNavigator(value)
    controller=ContinuousConeController(value,0.,initialized=True,handoff_after_two=True)
    for fid,t in enumerate(np.arange(0,1.4,.1)):
        data=row(value,float(t),fid,foot_y=260.)
        nav=navigator.update_views({'secondary':IMAGE},{'secondary':data},float(t))
        output=controller.step(nav,float(t))
        assert nav['line_state']=='paired' and nav['cones']==[]
        assert controller.target is None and controller.count==0 and not controller.passed_ids
        assert output['phase']!='complete'
