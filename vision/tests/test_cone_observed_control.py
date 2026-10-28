"""Synthetic observed-band mechanisms, not reconstruction of the real accident.

Road/track observations below are constructed explicitly. No raw images of the
incident are available; these tests check navigator/controller compatibility.
"""
import copy
from pathlib import Path

import numpy as np
import pytest

from carvision.cone_continuous import ContinuousConeController
from carvision.cone_low_speed import DualLowSpeedNavigator,trial_settings
from carvision.cone_planning import load_settings
from carvision.cone_navigation import actuator_settings
from carvision.cone_trial_output import ConePulseGuard


ROOT=Path(__file__).parents[1]


def system():
    settings=trial_settings(load_settings(ROOT/'tests/fixtures/configs/cone-trial-field-20261026.json'))
    navigator=DualLowSpeedNavigator(settings)
    controller=ContinuousConeController(settings,0.,initialized=True)
    for t in (0.,.1,.2):
        result=controller.step(observe(navigator,t),t)
    assert result['esc_us']>1500
    return navigator,controller


def reference(navigator,y):
    H=navigator.views['secondary'].projection.h
    return float(-(H[0,1]*y+H[0,2])/H[0,0])


def target(navigator,t,y,identifier='secondary-1',**changes):
    x=reference(navigator,y)+40
    return {'id':identifier,'visible':True,'confirmed':True,'seen_s':t,
            'identity_revision':0,
            'bbox_xyxy':[x-15,y-25,x+15,min(360,y+5)],'foot_pixel':[x,y],
            'kind':'cone_candidate','ground_contact_usable':True,'clipped':False,**changes}


def observe(navigator,t,tracks=None):
    ys=np.linspace(180,260,12)
    row={'captured_s':t,'frame_id':round(t*1000),'tracks':tracks or [],
         'road_context':{'valid':True,'ground_supported':True,'turn_aware':True,
             'boundary_candidates':[],
             'left_inner':[[reference(navigator,y)-140,float(y)] for y in ys],
             'right_inner':[[reference(navigator,y)+140,float(y)] for y in ys]}}
    image=np.zeros((360,480,3),np.uint8)
    return navigator.update_views({'secondary':image},{'secondary':row},t)


def locked():
    navigator,controller=system()
    nav=observe(navigator,.3,[target(navigator,.3,230)])
    output=controller.step(nav,.3)
    assert output['target_id']=='secondary-1' and output['target_space']=='lane'
    return navigator,controller


@pytest.mark.parametrize('y',[145.,300.,340.])
def test_target_outside_observed_band_does_not_invalidate_the_current_paired_lane(y):
    navigator,controller=system()
    nav=observe(navigator,.3,[target(navigator,.3,y)])
    assert nav['line_state']=='paired' and nav['boundary_observed'] and not nav.get('line_missing')
    assert nav['lane_rows']==pytest.approx([180,260])
    assert not any(not c.get('near_field_continuation') for c in nav['cones'])
    output=controller.step(nav,.3)
    assert output['paired_stable'] and output['target_id'] is None
    assert output['target_source']=='fresh_lane_only' and output['esc_us']>1500
    assert controller.line_missing_since is None and controller.count==0


def test_existing_lock_continues_current_pixels_beyond_band_without_extending_lane_geometry():
    navigator,controller=locked();old_fraction=controller.target_fraction
    nav=observe(navigator,.4,[target(navigator,.4,300)])
    assert nav['line_state']=='paired' and nav['lane_rows']==pytest.approx([180,260])
    assert nav['visual_cones'][0]['current_pixel']
    output=controller.step(nav,.4)
    current=nav['visual_cones'][0]
    expected=current['image_fraction']-(current['half_image_width']+.15)
    assert output['target_source']=='current_pixels' and output['target_space']=='image'
    assert controller.filtered_target==pytest.approx(expected)
    assert controller.filtered_target!=pytest.approx(old_fraction)
    assert output['requested_steering_us']>1610 and output['esc_us']>1500
    assert controller.count==0 and not output['tail_clearance_verified']


def test_legacy_continuation_metadata_cannot_reactivate_historical_lane_geometry():
    navigator,controller=locked()
    nav=observe(navigator,.4,[target(navigator,.4,300)])
    # The history container remains compatible, but its old geometry must not
    # become a current planning target or a new pass-clearance observation.
    nav['cones']=[{'id':'secondary-1','fraction':-10.,'half_width':100.,
        'near':True,'y_fraction':.95,'exit_band':True,'near_field_continuation':True,
        'position_source':'last_valid_ground_observation'}]
    before=controller.pass_gate_hits
    output=controller.step(nav,.4)
    assert output['target_space']=='image' and output['esc_us']>1500
    assert controller.pass_gate_hits==before and controller.count==0
    assert controller.target_fraction!=-10. and not output['best_effort_route']


@pytest.mark.parametrize('pixels_remain',[False,True])
def test_outside_or_disappeared_target_cannot_manufacture_missing_third_pass_frame(pixels_remain):
    navigator,controller=locked()
    controller.pass_gate_hits=2;controller.near_seen=True
    for k in range(16):
        t=round(.4+k*.1,8)
        tracks=[target(navigator,t,300)] if pixels_remain else []
        nav=observe(navigator,t,tracks)
        # New diagnostic metadata must never count as target visibility.
        nav.setdefault('obstacle_diagnostics',[])
        output=controller.step(nav,t)
        assert controller.count==0 and controller.auto_count==0
        assert 'secondary-1' not in controller.passed_ids
    if pixels_remain:
        # Real paired road and current target pixels remain available. Keeping
        # that identity is allowed; neither its duration nor the injected old
        # gate counter supplies a third observed clearance/pass frame.
        assert controller.target=='secondary-1' and controller.unverified_passes==0
        assert output['target_source']=='current_pixels'
        assert not output['passage_evidence']['visual_estimate_ready']
        assert output['passage_evidence']['exit_frames']==0
    else:
        assert controller.target is None and controller.unverified_passes==1
        assert 'secondary-1' in output['retired_unverified_ids']
        assert output['target_source']=='fresh_lane_only'


def test_current_lane_and_obstacle_diagnostics_do_not_renew_cached_frame_motion():
    navigator,controller=locked()
    nav=observe(navigator,.4,[target(navigator,.4,300)])
    first=controller.step(nav,.4)
    before=(controller.pass_gate_hits,controller.tail_powered,controller.pd.error,controller.pd.derivative)
    repeated=copy.deepcopy(nav)
    repeated['obstacle_diagnostics']=[{'id':'secondary-1','reason':'out_of_observed_band'}]
    cached=controller.step(repeated,.5)
    assert cached['fresh_control'] is False and cached['steering_us']==first['steering_us']
    assert (controller.pass_gate_hits,controller.tail_powered,controller.pd.error,controller.pd.derivative)==before
    guard=ConePulseGuard(actuator_settings(controller.s),lambda *args:None,mode='F/B')
    guard.accept({**first,'sequence':0,'sent_s':.4},.4)
    guard.tick(.61)
    assert guard.fault and guard.esc==1400


def test_new_band_fields_do_not_change_gain_contact_or_camera_freshness_policy():
    navigator,controller=system()
    assert controller.steering_gain==1.5 and controller.stop_for_cones is False
    assert controller.progress()['line_recovery_s']==1.
    nav=observe(navigator,.3,[target(navigator,.3,300)])
    with pytest.raises(ValueError,match='画面'):
        controller.step(nav,.551)
