"""Independent camera distance selection; synthetic pixels and fake output only."""
import copy
from types import SimpleNamespace

import pytest

from carvision.crosswalk_observation import CalibratedTrialCamera
from carvision.brake_parking import BrakeParkingController,load_settings
from carvision.race_flow_console import RaceFlowDrive
from test_crosswalk_observation import settings,states,publish,paper_row,background,advance,lock_primary


def control_settings(settings):
    from pathlib import Path
    result=load_settings(Path(__file__).parents[1]/'configs/brake-parking.json')
    result.update(copy.deepcopy(settings))
    return result


def test_large_distance_difference_records_native_measurements_without_averaging(settings,states):
    publish(states['primary'],paper_row(190),10001,9.98)
    publish(states['secondary'],paper_row(280),5,10.)
    sample=CalibratedTrialCamera(states,settings).read(10.)
    diag=sample['distance_disagreement'];rows=sample['camera_observations']
    assert sample['selection_revision']=='crosswalk_single_source_v1'
    assert diag['primary_distance_m']==rows['primary']['distance_estimate_m']
    assert diag['secondary_distance_m']==sample['distance_estimate_m']
    assert diag['difference_m']==pytest.approx(abs(diag['primary_distance_m']-diag['secondary_distance_m']))
    assert diag['difference_m']>.05 and diag['capture_skew_ms']==pytest.approx(20.)
    assert diag['captured_s']=={'primary':9.98,'secondary':10.}
    assert diag['raw_frame_ids']=={'primary':10001,'secondary':5}
    assert not diag['target_association_verified']
    assert sample['captured_s']==10. and sample['raw_frame_id']==5


def test_warning_clears_on_new_consistent_measurements(settings,states):
    publish(states['primary'],paper_row(190),10001,10.)
    publish(states['secondary'],paper_row(280),5,10.)
    reader=CalibratedTrialCamera(states,settings)
    assert reader.read(10.)['distance_disagreement']
    advance(states,10.1)
    assert reader.read(10.1)['distance_disagreement'] is None


def test_two_new_primary_frames_in_overlap_can_select_primary_despite_difference(settings,states):
    publish(states['primary'],paper_row(245),10001,10.)
    publish(states['secondary'],paper_row(185),5,10.)
    reader=CalibratedTrialCamera(states,settings)
    assert reader.read(10.)['source_camera']=='secondary'
    advance(states,10.1,primary=245,secondary=185)
    sample=reader.read(10.1)
    assert sample['source_camera']=='primary' and sample['distance_disagreement']['difference_m']>.05
    assert sample['distance_estimate_m']==sample['camera_observations']['primary']['distance_estimate_m']


@pytest.mark.parametrize('camera,bottom',[('primary',300),('secondary',290),('primary',330),('secondary',330)])
def test_either_near_camera_causes_actual_brake_instead_of_disagreement_fault(settings,states,camera,bottom):
    reader=lock_primary(settings,states)
    advance(states,10.2,primary=bottom if camera=='primary' else 190,
            secondary=bottom if camera=='secondary' else 180)
    sample=reader.read(10.2)
    assert sample['source_camera']==camera
    controller=BrakeParkingController(control_settings(settings),crosswalk=True)
    result=controller.step(sample,10.2)
    assert result['action']=='brake' and result['phase']=='brake_pulse'
    assert result['source_camera']==camera
    assert result['distance_disagreement']==sample['distance_disagreement']


def test_primary_preference_returns_after_secondary_stop_risk_clears(settings,states):
    reader=lock_primary(settings,states)
    advance(states,10.2,secondary=330)
    assert reader.read(10.2)['source_camera']=='secondary'
    advance(states,10.3)
    result=reader.read(10.3)
    assert result['source_camera']=='primary' and result['source_changed']


@pytest.mark.parametrize('camera',['primary','secondary'])
def test_only_one_camera_needs_to_recognize_the_target(settings,states,camera):
    for name in states:
        publish(states[name],paper_row(245 if name=='primary' else 235) if name==camera else background(),
                states[name].raw_frame_id+1,10.)
    sample=CalibratedTrialCamera(states,settings).read(10.)
    assert sample['candidate'] and sample['source_camera']==camera
    assert not sample['camera_observations']['secondary' if camera=='primary' else 'primary']['candidate']


def test_primary_losing_target_can_use_secondary_recognition_after_handoff(settings,states):
    reader=lock_primary(settings,states)
    advance(states,10.2,primary=None)
    result=reader.read(10.2)
    assert result['candidate'] and result['source_camera']=='secondary'
    assert result['captured_s']==10.2 and result['source_changed']


def test_braking_override_does_not_relabel_an_older_secondary_frame(settings,states):
    reader=lock_primary(settings,states)
    publish(states['primary'],paper_row(245),10002,10.2)
    publish(states['secondary'],paper_row(235),6,10.15)
    assert reader.read(10.2)['captured_s']==10.2
    publish(states['secondary'],paper_row(330),7,10.18)
    assert reader.read(10.21) is None
    publish(states['secondary'],paper_row(330),8,10.22)
    result=reader.read(10.22)
    assert result['captured_s']==10.22 and result['source_camera']=='secondary'


def test_selected_warning_survives_full_flow_navigation_and_is_logged(settings,states):
    from test_race_flow import ROOT,Frames
    from carvision.race_flow import RaceFlowController,load_bundle
    bundle=load_bundle(ROOT);bundle['brake'].update(control_settings(settings))
    controller=RaceFlowController(bundle,9.9);controller._go('crosswalk',9.9,'test')
    controller.controller.first_detection_handled=True
    publish(states['primary'],paper_row(190),10001,10.)
    publish(states['secondary'],paper_row(280),5,10.)
    sample=CalibratedTrialCamera(states,settings).read(10.)
    nav={'experimental':True,'ready':True,'captured_s':10.,'frame_id':1,
         'ego_fraction':.5,'half_envelope':.20,'lane_bounds_px':[[100,380],[50,430]],
         'cones':[],'uncertain':[],'source_camera':'secondary'}
    result=controller.step({'sequence':1,'captured_s':10.,'crosswalk':sample,'navigation':nav},10.)
    assert result['phase']=='crosswalk' and result['esc_us']>1500
    assert result['distance_disagreement']==sample['distance_disagreement']


@pytest.mark.parametrize('stage',['fault','cancelled'])
def test_stopped_page_shows_neutral_and_preserves_last_sent_intent(tmp_path,stage):
    flow=RaceFlowDrive(SimpleNamespace(status=lambda:{}),{},tmp_path,live=False)
    flow.error=None;flow.stage=stage;flow.reason='test stop'
    last={'action':'search','esc_us':1600,'steering_us':1650,'reason':'old motion'}
    flow.last_intent=copy.deepcopy(last)
    status=flow.status()['race_flow']
    assert status['intent']['action']=='neutral' and status['intent']['esc_us']==1500
    assert status['intent']['steering_us']==1610 and not status['hardware_output']
    assert status['last_intent_before_stop']==last and flow.last_intent==last


def test_warning_metadata_is_copied_and_not_changed_by_the_next_batch(settings,states):
    publish(states['primary'],paper_row(190),10001,10.)
    publish(states['secondary'],paper_row(280),5,10.)
    reader=CalibratedTrialCamera(states,settings);sample=reader.read(10.)
    controller=BrakeParkingController(control_settings(settings),crosswalk=True)
    result=controller.step(sample,10.)
    original=copy.deepcopy(result['distance_disagreement'])
    sample['distance_disagreement']['difference_m']=99.
    assert result['distance_disagreement']==original
