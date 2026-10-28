"""Per-camera wall-clock checks and 500ms crosswalk-only latency; fake IO."""
import copy
from types import SimpleNamespace

import pytest

from carvision.crosswalk_observation import CalibratedTrialCamera
from carvision.brake_parking import BrakeParkingController
from test_crosswalk_observation import settings,states,publish,paper_row


def test_second_camera_can_publish_during_primary_processing_without_false_future_error(settings,states,monkeypatch):
    settings['crosswalk_max_frame_age_s']=.5;clock=[10.]
    reader=CalibratedTrialCamera(states,settings,clock=lambda:clock[0])
    original=reader.detectors['primary'].detect
    def detect(*args):
        clock[0]=10.08
        publish(states['secondary'],paper_row(235),5,10.06)
        return original(*args)
    monkeypatch.setattr(reader.detectors['primary'],'detect',detect)
    sample=reader.read(10.)
    assert sample['captured_s']==10.06 and sample['raw_frame_id']==5
    assert sample['camera_ages_ms']=={'primary':80.,'secondary':20.}
    assert sample['frame_age_limit_ms']==500


@pytest.mark.parametrize('age',[.3,.49,.5])
def test_crosswalk_camera_accepts_new_distinct_frames_within_500ms(settings,states,age):
    settings['crosswalk_max_frame_age_s']=.5
    reader=CalibratedTrialCamera(states,settings)
    out=reader.read(10.+age)
    assert out['fresh'] and out['captured_s']==10.
    assert reader.read(10.+age) is None


def test_crosswalk_camera_rejects_older_than_500ms(settings,states):
    settings['crosswalk_max_frame_age_s']=.5
    with pytest.raises(ValueError,match='500毫秒'):
        CalibratedTrialCamera(states,settings).read(10.501)


def test_a_truly_future_frame_is_rejected_even_with_the_runtime_clock(settings,states):
    settings['crosswalk_max_frame_age_s']=.5
    publish(states['secondary'],paper_row(235),5,10.2)
    with pytest.raises(ValueError,match='采样时间异常'):
        CalibratedTrialCamera(states,settings,clock=lambda:10.1).read(10.)


def test_processing_cannot_hide_a_stale_first_camera(settings,states,monkeypatch):
    settings['crosswalk_max_frame_age_s']=.5;clock=[10.]
    reader=CalibratedTrialCamera(states,settings,clock=lambda:clock[0])
    original=reader.detectors['primary'].detect
    def detect(*args):
        clock[0]=10.6
        publish(states['secondary'],paper_row(235),5,10.59)
        return original(*args)
    monkeypatch.setattr(reader.detectors['primary'],'detect',detect)
    with pytest.raises(ValueError,match='primary 摄像头处理后帧龄'):
        reader.read(10.)


def test_camera_error_remains_immediate_even_if_frame_is_recent(settings,states):
    settings['crosswalk_max_frame_age_s']=.5;states['secondary'].state='error'
    with pytest.raises(ValueError,match='缓存不可用'):
        CalibratedTrialCamera(states,settings).read(10.)


def test_real_clock_rollback_is_rejected(settings,states):
    with pytest.raises(ValueError,match='校验时钟无效或倒退'):
        CalibratedTrialCamera(states,settings,clock=lambda:9.9).read(10.)


def test_crosswalk_limit_is_separate_from_shared_parking_and_actuator_deadlines(tmp_path):
    from test_crosswalk_profile_integration import profile,legacy,load_active,observation
    # Reuse the pure in-memory fixture factories without opening hardware.
    p=profile.__wrapped__();old=legacy.__wrapped__()
    active=load_active(tmp_path,old,p,.5)
    assert active['max_frame_age_s']==.25 and active['actuator_lease_s']<=.25
    assert active['crosswalk_max_frame_age_s']==.5
    crosswalk=BrakeParkingController(active,crosswalk=True)
    sample=observation(0.,moving=True,candidate=False,near_edge_y_normalized=None)
    assert crosswalk.step(sample,.4)['phase']!='fault'
    parking=BrakeParkingController(active)
    assert parking.step(sample,.4)['phase']=='fault'


def test_full_flow_uses_500ms_only_in_crosswalk_stage():
    from test_race_flow import ROOT
    from carvision.race_flow import RaceFlowController,load_bundle
    c=RaceFlowController(load_bundle(ROOT),0.)
    assert c.frame_age_limit()==.25
    c._go('crosswalk',.1,'test')
    assert c.frame_age_limit()==.5
    c._go('traffic',.2,'test')
    assert c.frame_age_limit()==.25


def test_cancel_keeps_the_original_fault_reason_visible(tmp_path):
    from carvision.race_flow_console import RaceFlowDrive
    flow=RaceFlowDrive(SimpleNamespace(status=lambda:{}),{},tmp_path,live=False)
    flow.stage='fault';flow.reason='secondary 帧龄501毫秒，超出500毫秒期限'
    flow.cancel('已停止全流程，车辆控制已释放')
    assert flow.stage=='fault' and flow.reason.startswith('secondary 帧龄501')


def test_independent_worker_accepts_350ms_frame_without_changing_actuator_lease(tmp_path):
    from test_crosswalk_profile_integration import profile,write_profile
    from test_brake_trial import Driver,Speech,CONFIG
    from carvision.brake_trial import BrakeTrialDrive
    write_profile(tmp_path,profile.__wrapped__());clock=[10.];writes=[]
    class Reader:
        jpeg=None
        def __init__(self,*args):self.frame=0
        def read(self,now):
            self.frame+=1;clock[0]+=.1
            return {'captured_s':clock[0]-.35,'frame_id':self.frame,'image_size':[480,360],'fresh':True}
    class Actuator:
        def send(self,intent):writes.append(intent);wrapper.stop.set();return {'hardware_output':False}
        def close(self):pass
    wrapper=BrakeTrialDrive(Driver(),{},CONFIG,tmp_path,clock=lambda:clock[0],
        reader_factory=Reader,actuator_factory=Actuator,speech_factory=Speech)
    wrapper.folder=tmp_path/'session';wrapper.phase='preparing';wrapper.busy=True;wrapper.lease_until=100.
    wrapper._work()
    assert writes and writes[0]['phase']=='ready' and wrapper.phase!='fault', (wrapper.phase,wrapper.reason)
    assert wrapper.settings['actuator_lease_s']<=.25


def test_idle_distance_view_rechecks_clock_after_processing(tmp_path,monkeypatch):
    from test_crosswalk_profile_integration import profile,write_profile
    from test_brake_trial import Driver,CONFIG
    from carvision.brake_trial import BrakeTrialDrive
    write_profile(tmp_path,profile.__wrapped__());clock=[10.]
    class Reader:
        def __init__(self,*args,**kwargs):pass
        def read(self,now):clock[0]=10.03;return {'captured_s':10.02,'candidate':False}
    monkeypatch.setattr('carvision.brake_trial.TrialCamera',Reader)
    wrapper=BrakeTrialDrive(Driver(),{},CONFIG,tmp_path,clock=lambda:clock[0])
    assert wrapper._distance_status()['state']=='ok'
