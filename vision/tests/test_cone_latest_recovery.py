"""Latest-cone failure regressions. Hardware is always fake."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from carvision.cone_latest_engine import cone_image_guidance as guidance
from carvision.cone_latest_engine.cone_execution import ConeRun
from carvision.cone_latest_engine.cone_low_speed import trial_settings
from carvision.cone_latest_engine.cone_road import ConeRoadContext
from carvision.cone_latest_task import install

ROOT=Path(__file__).resolve().parents[2]


@pytest.fixture
def settings():
    value=json.loads((ROOT/'vision/configs/cone-latest-20261029.json').read_text(encoding='utf8'))
    return trial_settings(value,{'test_steering_gain':1.5})


def nav(t,cones=()):
    return dict(source_camera='secondary',captured_s=t,line_missing=True,cones=list(cones))


def geometry(t,heading,near=.2):
    return dict(camera='secondary',lane_observed_s=t,line_held=False,fresh_lane=True,
                near_ego_fraction=near,heading_correction_fraction=heading,nav_goal_compatible=True)


def test_heading_is_corrected_before_lateral_recentering(monkeypatch,settings):
    value=guidance.ImageConeGuidance(settings)
    current=[geometry(1.,-.06)]
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:current[0])
    first=value.update(nav(1.),1.)
    assert first['phase']=='align' and first['command']<0
    # Position demands right steering, but yaw must first be corrected left.
    for i in range(3):
        t=1.1+i*.1;current[0]=geometry(t,0.)
        result=value.update(nav(t),t)
    assert result['phase']=='recenter' and 0<result['command']<=.5


def test_repeated_frame_cannot_confirm_vehicle_alignment(monkeypatch,settings):
    value=guidance.ImageConeGuidance(settings)
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:geometry(1.,0.))
    for t in (1.,1.03,1.06,1.09):
        assert value.update(nav(1.),t)['phase']=='align'
    assert value.recenter_aligned_hits==1


def test_visible_cone_clearance_survives_alignment(monkeypatch,settings):
    settings['geometry']['lane_width_m']=1.2
    value=guidance.ImageConeGuidance(settings)
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda n,*a,**k:geometry(n['captured_s'],0.,.22))
    cone=dict(id='real',fraction=.5,half_width=.03,image_fraction=.5,
              half_image_width=.03,y_fraction=.8,near=True)
    for i in range(8):
        t=1+i*.1
        n={**nav(t,[cone]),'line_missing':False,'lane_bounds_px':[[100,300],[0,400]],'ego_fraction':.22}
        result=value.update(n,t)
        assert result['side']=='right'  # Equal free space now follows the operator's right preference.
        assert result['target_fraction']>.7  # Never request the blocked road center.
        assert result['phase']!='recenter'


def test_no_road_no_target_stops_after_one_second(monkeypatch,settings):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:None)
    value=guidance.ImageConeGuidance(settings)
    assert not value.update(nav(1.),1.)['stop_required']
    assert not value.update(nav(1.99),1.99)['stop_required']
    result=value.update(nav(2.01),2.01)
    assert result['stop_required'] and result['command']==0


def test_target_only_feedback_is_bounded_at_five_seconds(monkeypatch,settings):
    monkeypatch.setattr(guidance,'image_lane_geometry',lambda *a,**k:None)
    value=guidance.ImageConeGuidance(settings)
    cone=dict(id='real',image_fraction=.5,half_image_width=.03,y_fraction=.7)
    assert not value.update(nav(1.,[cone]),1.)['stop_required']
    assert not value.update(nav(5.99,[cone]),5.99)['stop_required']
    assert value.update(nav(6.01,[cone]),6.01)['stop_required']


def test_single_edge_recovery_weight_expires_without_refreshing_anchor():
    assert guidance.ImageConeGuidance._weight(dict(single_edge_observed=True,line_age_s=.4))==pytest.approx(.36)
    assert guidance.ImageConeGuidance._weight(dict(single_edge_observed=True,line_age_s=1.))==0


def test_current_single_edge_never_refreshes_the_old_pair(settings):
    from test_cone_turn_corridor import track
    settings['scene']['surface']='indoor_white_line_track'
    context=ConeRoadContext(settings)
    def read(t,sides=('left','right'),shift=0):
        image=track(sides=sides,shift=shift)
        return context.inspect(image,np.zeros(image.shape[:2],np.uint8),frame_id=round(t*100),captured_s=t)
    assert read(1.)['valid']
    for t in (1.1,1.3,1.5,1.7,1.9):
        result=read(t,('left',),8.)
        assert result['valid'] and result['single_edge_observed']
        assert result['paired_observed_s']==1.
        assert result['prediction_age_s']==pytest.approx(t-1.)
    assert not read(2.05,('left',),8.)['valid']


def test_background_blue_is_not_new_target_without_floor_or_lane_support():
    from carvision.cone_latest_engine import cone_road
    from unittest.mock import patch
    def enrich(rows,image,context):return rows
    with patch.object(cone_road.PaintContext,'enrich',enrich):
        for color,expected in [((110,110,110),False),((50,130,50),True)]:
            image=np.full((80,100,3),color,dtype=np.uint8)
            row=dict(bbox_xyxy=[40,20,55,50],boundary_relation='unknown',scene_supported=False)
            ConeRoadContext.enrich([row],image,{})
            assert row['new_target_supported'] is expected
        row['boundary_relation']='outside'
        ConeRoadContext.enrich([row],image,{})
        assert not row['new_target_supported']


def running():
    value=ConeRun();value.active=True;value.phase='ready';value.owner='page';value.token='test'
    value.lease_until=2.;value.last_heartbeat_s=0.
    return value


def heartbeat(seq=0,**changes):
    return {'client':'page','run_token':'test','heartbeat_sequence':seq,**changes}


def test_latest_heartbeat_has_independent_sequence_and_three_second_lease():
    value=running();value.sequence=12
    assert value.heartbeat(heartbeat(50),1.74)['accepted']
    assert value.lease_until==pytest.approx(4.74) and value.sequence==12
    assert not value.heartbeat(heartbeat(49),1.8)['accepted']
    assert value.lease_until==pytest.approx(4.74)
    with pytest.raises(ValueError):value.heartbeat(heartbeat(51,client='wrong'),1.9)
    with pytest.raises(ValueError):value.heartbeat(heartbeat(51),4.75)
    assert not value.active
    with pytest.raises(ValueError):value.heartbeat(heartbeat(52),4.76)


def test_latest_heartbeat_bypasses_status_and_observer_lock():
    from carvision import application
    install(application)
    app=object.__new__(application.DebugApplication);app.closed=False
    value=running()
    app.tasks={'cone_latest':SimpleNamespace(inner=SimpleNamespace(run=value),clock=lambda:1.5)}
    app.status=lambda:pytest.fail('heartbeat must bypass status collection')
    result=app.request('latest_cone_page_heartbeat',heartbeat())
    assert result['accepted'] and value.lease_until==4.5


def test_extended_frontend_uses_independent_latest_heartbeat():
    from carvision.debug_page import build_controls
    from carvision.cone_latest_page import extend_controls
    script=extend_controls(build_controls(root=ROOT)).SCRIPT
    assert "task==='cone_latest'?'latest_cone_page_heartbeat':'cone_page_heartbeat'" in script
    assert "if(['cone','cone_latest'].includes(task))return coneHeartbeat()" in script
