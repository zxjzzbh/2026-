"""A boot or page poll must never authorize motion or reuse an old preparation."""
import hashlib
import json
import threading

import pytest

from carvision.boot_test import BootTestDrive, reviewed_manifest
from carvision.manual_drive import DriveError


def setup(prepare=None, confirm=None):
    events=[]
    class Repeated:
        def status(self):return {'mode':'disabled','trial_token':'fresh-round','hardware_output':False}
        def request(self, action, payload):
            events.append((action,payload))
            return self.status()
        def close(self):events.append('close')
    drive=BootTestDrive(prepare or (lambda cancelled: events.append('prepare') or {'boot':'new'}),
                        confirm or (lambda context: events.append('confirm')),
                        lambda context: events.append('factory') or Repeated(), {})
    return drive,events


def request(drive, **values):
    return drive.request('enable', {'client':'operator','bench_ready':True,
                                   'setup_token':drive.status()['setup_token'],**values})


def test_boot_and_polling_never_prepare_or_create_hardware():
    drive,events=setup()
    for _ in range(30):
        assert drive.status()['boot_test_phase']=='esc_off'
        assert drive.status()['hardware_output'] is False
    assert events==[]
    with pytest.raises(DriveError):drive.request('command',{'motor':'forward'})
    drive.close()
    assert events==[]


def test_two_fresh_physical_confirmations_required_without_held_direction_replay():
    drive,events=setup()
    first_token=drive.status()['setup_token']
    with pytest.raises(DriveError):request(drive,bench_ready=False)
    request(drive,motor='forward',steering='right');drive.thread.join(2)
    assert events==['prepare'] and drive.status()['boot_test_phase']=='esc_on'
    assert drive.status()['setup_token']!=first_token
    with pytest.raises(DriveError):request(drive,setup_token=first_token)
    with pytest.raises(DriveError):request(drive,bench_ready=False)
    on_token=drive.status()['setup_token']
    request(drive,motor='forward',steering='right');drive.thread.join(2)
    assert drive.status()['boot_test_phase']=='ready'
    assert drive.status()['setup_token']!=on_token
    with pytest.raises(DriveError):
        drive.request('command',{'setup_token':on_token,'trial_token':'fresh-round','motor':'forward'})
    assert events[:3]==['prepare','confirm','factory']
    assert events[3]==('enable',{'client':'operator','bench_ready':True,'trial_token':'fresh-round'})
    drive.close()


def test_reloading_page_can_confirm_next_setup_step_with_fresh_token():
    drive,events=setup()
    request(drive);drive.thread.join(2)
    request(drive,client='new-page');drive.thread.join(2)
    assert events[-1][1]['client']=='new-page'
    drive.close()


def test_continuous_delegate_without_round_token_still_needs_two_fresh_confirmations():
    events=[]
    class Continuous:
        def status(self):return {'mode':'disabled','ground_continuous_trial':True}
        def request(self, action, payload):events.append((action,payload));return self.status()
        def close(self):pass
    drive=BootTestDrive(lambda cancelled:{},lambda context:None,lambda context:Continuous(),{})
    request(drive);drive.thread.join(2)
    assert drive.status()['boot_test_phase']=='esc_on' and events==[]
    request(drive,motor='forward');drive.thread.join(2)
    assert drive.status()['boot_test_phase']=='ready'
    assert events==[('enable',{'client':'operator','bench_ready':True,'trial_token':None})]
    drive.close()


def test_mode_selection_cannot_cross_preparation_or_replay_old_tokens():
    drive, events = setup()
    drive.profiles = {'driving':{}, 'steering_pwm':{'steering_pwm_available':True,'motor_available':False}}
    old = drive.status()['setup_token']
    drive.request('select_mode',{'setup_token':old,'test_mode':'steering_pwm'})
    assert events == [] and drive.status()['motor_available'] is False
    assert drive.status()['setup_token'] != old
    with pytest.raises(DriveError):request(drive,setup_token=old)
    request(drive);drive.thread.join(2)
    assert drive.status()['boot_test_phase']=='steering_ready'
    with pytest.raises(DriveError):
        drive.request('select_mode',{'setup_token':drive.status()['setup_token'],'test_mode':'driving'})
    request(drive);drive.thread.join(2)
    assert drive.status()['boot_test_phase']=='ready'
    token = drive.status()['setup_token']
    drive.request('finish_test',{'setup_token':token})
    drive.thread.join(2)
    assert drive.status()['boot_test_phase']=='esc_off' and drive.delegate is None
    assert events[-2][0]=='emergency' and events[-1]=='close'
    with pytest.raises(DriveError):
        drive.request('steering_pwm',{'setup_token':token,'pulse_us':1700})
    drive.request('select_mode',{'setup_token':drive.status()['setup_token'],'test_mode':'driving'})
    assert drive.status()['test_mode']=='driving'
    drive.close()


def test_unhashable_mode_is_a_control_error_and_never_starts_hardware():
    drive, events = setup()
    with pytest.raises(DriveError):
        drive.request('select_mode',{'setup_token':drive.status()['setup_token'],'test_mode':[]})
    assert events == []
    drive.close()


@pytest.mark.parametrize('step', ['prepare','confirm'])
def test_failed_preparation_never_creates_a_drive(step):
    def fail(*args):raise RuntimeError('test failure')
    drive,events=setup(prepare=fail if step=='prepare' else None,
                       confirm=fail if step=='confirm' else None)
    request(drive);drive.thread.join(2)
    if step=='confirm':request(drive);drive.thread.join(2)
    assert drive.status()['mode']=='fault' and 'factory' not in events
    with pytest.raises(DriveError):request(drive)
    drive.close()


def test_stop_during_preparation_cancels_without_starting_a_worker():
    entered=threading.Event();done=threading.Event()
    def prepare(cancelled):
        entered.set();done.wait(2)
        return {'boot':'new'}
    drive,events=setup(prepare=prepare)
    request(drive);assert entered.wait(2)
    drive.request('emergency',{})
    done.set();drive.thread.join(2)
    assert drive.status()['mode']=='fault' and events==[]
    drive.close()


def test_passive_focus_change_during_zero_only_baseline_cannot_arm_or_cancel_motion():
    entered=threading.Event();done=threading.Event()
    def prepare(cancelled):entered.set();done.wait(2);return {'boot':'new'}
    drive,events=setup(prepare=prepare)
    request(drive);assert entered.wait(2)
    drive.request('stop',{'stop_source':'window_blur'})
    drive.request('stop',{'stop_source':'page_hidden'})
    assert not drive.cancelled.is_set() and drive.status()['boot_test_phase']=='baseline'
    done.set();drive.thread.join(2)
    assert drive.status()['boot_test_phase']=='esc_on' and events==[]
    drive.close()


def test_preparation_failure_reset_needs_new_esc_off_confirmation_and_starts_nothing():
    def fail(*args):raise RuntimeError('prepare failed')
    drive,events=setup(prepare=fail)
    request(drive);drive.thread.join(2)
    token=drive.status()['setup_token']
    with pytest.raises(DriveError):drive.request('reset',{'setup_token':token})
    drive.request('reset',{'setup_token':token,'bench_ready':True})
    assert drive.status()['boot_test_phase']=='esc_off' and events==[]
    assert drive.status()['setup_token']!=token
    drive.close()


def test_close_during_confirmation_cannot_publish_a_new_drive():
    entered=threading.Event();done=threading.Event()
    def confirm(context):entered.set();done.wait(2)
    drive,events=setup(confirm=confirm)
    request(drive);drive.thread.join(2)
    request(drive);assert entered.wait(2)
    closing=threading.Thread(target=drive.close)
    closing.start();assert drive.cancelled.wait(2)
    done.set();closing.join(2)
    assert not closing.is_alive()
    assert 'factory' not in events


def test_manifest_rejects_source_drift_and_workspace_escape(tmp_path):
    (tmp_path/'code.py').write_text('current')
    manifest={'pi_checks':{'rc':0},'files':[{'path':'code.py','sha256':hashlib.sha256(b'current').hexdigest()}]}
    (tmp_path/'checks.json').write_text(json.dumps(manifest))
    template={'software_checks_record':'checks.json'}
    assert reviewed_manifest(tmp_path,template)==manifest
    (tmp_path/'code.py').write_text('changed')
    with pytest.raises(ValueError):reviewed_manifest(tmp_path,template)
    with pytest.raises(ValueError):reviewed_manifest(tmp_path,{'software_checks_record':'../checks.json'})
