"""A usable calibrated camera survives a peer's temporary loss."""
import numpy as np
import pytest
from carvision.crosswalk_observation import CalibratedTrialCamera
from test_crosswalk_observation import settings,states,publish,paper_row,processed_views


@pytest.mark.parametrize('failed',['primary','secondary'])
@pytest.mark.parametrize('failure',['stale','error','bad_jpeg','black'])
def test_one_temporary_failure_keeps_other_camera_native_evidence(settings,states,failed,failure):
    healthy='secondary' if failed=='primary' else 'primary'
    state=states[failed]
    if failure=='stale':state.raw_received_ms=9000.
    elif failure=='error':state.state='error'
    elif failure=='bad_jpeg':state.raw_jpeg=b'bad jpeg'
    else:publish(state,np.zeros((360,480,3),np.uint8),state.raw_frame_id+1,10.)
    reader=CalibratedTrialCamera(states,settings)
    result=reader.read(10.)
    assert result['source_camera']==healthy and result['captured_s']==10.
    assert result['raw_frame_id']==states[healthy].raw_frame_id
    assert result['candidate'] and result['metric_valid'] and result['degraded']
    assert set(result['camera_observations'])=={healthy}
    assert set(result['camera_errors'])=={failed}
    assert reader.read(10.1) is None


def test_processed_race_batch_may_contain_one_matched_camera(settings,states):
    reader=CalibratedTrialCamera(states,settings);views=processed_views(reader,states)
    result=reader.read(10.,processed_views={'secondary':views['secondary']})
    assert result['source_camera']=='secondary' and result['captured_s']==10.
    assert result['degraded'] and 'primary' in result['camera_errors']


def test_both_missing_fails_and_recovered_camera_cannot_send_older_cross_camera_time(settings,states):
    reader=CalibratedTrialCamera(states,settings)
    states['primary'].state=states['secondary'].state='error'
    with pytest.raises(ValueError,match='双摄画面失效'):reader.read(10.)
    states['secondary'].state='ok'
    publish(states['secondary'],paper_row(235),5,10.2)
    first=reader.read(10.2)
    states['primary'].state='ok';states['secondary'].state='error'
    publish(states['primary'],paper_row(245),10001,10.1)
    assert reader.read(10.2) is None
    publish(states['primary'],paper_row(245),10002,10.3)
    result=reader.read(10.3)
    assert result['source_changed'] and result['captured_s']>first['captured_s']


def test_near_edge_brake_source_has_priority_again_after_camera_recovers(settings,states):
    reader=CalibratedTrialCamera(states,settings)
    states['primary'].state='error';reader.read(10.)
    states['primary'].state='ok'
    publish(states['primary'],paper_row(330),10001,10.1)
    publish(states['secondary'],paper_row(190),5,10.1)
    result=reader.read(10.1)
    assert result['source_camera']=='primary' and result['near_limit_reached']
    assert not result['degraded'] and not result['camera_errors']


def test_identity_and_future_clock_are_not_soft_camera_losses(settings,states):
    reader=CalibratedTrialCamera(states,settings)
    states['primary'].raw_received_ms=10200.
    with pytest.raises(ValueError,match='未来'):reader.read(10.)
