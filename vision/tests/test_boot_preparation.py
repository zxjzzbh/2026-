"""Exercise fresh neutral preparation and the physical/static evidence boundary."""
import builtins
import hashlib
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest
import carvision.boot_test as module


@pytest.fixture
def prepared_environment(tmp_path, monkeypatch):
    events=[]
    (tmp_path/'code.py').write_text('current')
    (tmp_path/'checks.json').write_text(json.dumps({'pi_checks':{'rc':0},'files':[
        {'path':'code.py','sha256':hashlib.sha256(b'current').hexdigest()}]}))
    template={'software_checks_record':'checks.json','boot_id':'old',
              'neutral_physically_verified':True,'neutral_evidence_record':'old-proof',
              'neutral_handoff_verified_monotonic_s':0,'neutral_handoff_boot_id':'old'}
    template_path=tmp_path/'template.json';template_path.write_text(json.dumps(template))
    monkeypatch.setattr(module.PiBootPreparation,'current_boot',staticmethod(lambda:'fresh-boot'))
    original_read=Path.read_bytes
    monkeypatch.setattr(Path,'read_bytes',lambda path: b'Raspberry Pi 5 Model B Rev 1.0\0'
                        if path.as_posix()=='/proc/device-tree/model' else original_read(path))
    original_open=builtins.open
    monkeypatch.setattr(builtins,'open',lambda path,*args,**kw:
                        original_open(tmp_path/'ownership' if path=='/tmp/carvision-pi-pwm.lock' else path,*args,**kw))
    monkeypatch.setitem(sys.modules,'fcntl',SimpleNamespace(LOCK_EX=1,LOCK_NB=2,flock=lambda *args:None))
    class Board:
        position=1500
        def open(self):events.append('open')
        def close(self):events.append('close')
        def set_esc(self,channel,pulse):events.append(('esc',channel,pulse))
        def read_position(self,channel):events.append(('read',channel));return self.position
    monkeypatch.setattr('carvision.rasadapter5.RasAdapter',Board)
    monkeypatch.setattr(module.subprocess,'check_output',lambda *args,**kw:
                        json.dumps({'boot_id':'fresh-boot','modem':{'nr5g_registration':1}}))
    clock=[0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.time,'sleep',lambda seconds:clock.__setitem__(0,clock[0]+seconds))
    prep=module.PiBootPreparation(tmp_path,template_path)
    monkeypatch.setattr(prep,'health',lambda:{'boot_id':'fresh-boot','power_flags':0})
    return prep,events,Board


def test_only_neutral_is_written_and_old_physical_proof_is_cleared(prepared_environment):
    prep,events,_=prepared_environment
    assert events==[]
    context=prep.prepare(threading.Event())
    assert [e for e in events if isinstance(e,tuple) and e[0]=='esc']==[('esc',4,1500)]
    baseline=json.loads((Path(context['folder'])/'baseline.json').read_text())
    assert len(baseline['samples'])==30 and baseline['duration_s']>=30
    assert context['review']['boot_id']=='fresh-boot'
    assert context['review']['neutral_physically_verified'] is False
    assert 'neutral_evidence_record' not in context['review']
    before=list(events);prep.confirm(context)
    assert ('esc',4,1500) not in events[len(before):]
    assert context['review']['neutral_physically_verified'] is True


def test_cancelled_setup_cannot_send_neutral(prepared_environment):
    prep,events,_=prepared_environment
    cancelled=threading.Event();cancelled.set()
    with pytest.raises(RuntimeError,match='取消'):prep.prepare(cancelled)
    assert events==[]


def test_health_failure_prevents_any_board_command(prepared_environment,monkeypatch):
    prep,events,_=prepared_environment
    def fail():raise RuntimeError('power failure')
    monkeypatch.setattr(prep,'health',fail)
    with pytest.raises(RuntimeError):prep.prepare(threading.Event())
    assert events==[]


def test_changed_neutral_cannot_be_recorded_as_physical_static_proof(prepared_environment):
    prep,events,board=prepared_environment
    context=prep.prepare(threading.Event())
    board.position=1575
    with pytest.raises(RuntimeError,match='零油门'):prep.confirm(context)
    assert context['review']['neutral_physically_verified'] is False
    assert not (Path(context['folder'])/'neutral-observation.json').exists()


def test_old_boot_context_cannot_be_confirmed(prepared_environment):
    prep,events,_=prepared_environment
    context=prep.prepare(threading.Event());before=list(events)
    context['boot_id']='old-boot'
    with pytest.raises(RuntimeError):prep.confirm(context)
    assert events==before


def test_long_manual_preparation_does_not_invent_reverse_or_current_stop_proof(prepared_environment):
    prep, events, _ = prepared_environment
    template = json.loads(prep.template_path.read_text())
    template.update(ground_continuous_requested_by_user=True,
                    drivetrain_issue_resolved_reported_by_user=True)
    prep.template_path.write_text(json.dumps(template))
    context = prep.prepare(threading.Event())
    review = context['review']
    assert review['continuous_ground_driving_enabled'] is True
    assert review['manual_session_limit_s'] is review['hold_limit_s'] is None
    assert review['reverse_revalidation_prepared'] is True
    assert review['reverse_physically_verified'] is False
    assert review['current_boot_keyboard_stop_reverified'] is False
    assert review['neutral_physically_verified'] is False
    assert [e for e in events if isinstance(e, tuple) and e[0]=='esc'] == [('esc', 4, 1500)]


def test_s3_preparation_and_confirmation_never_open_uart_or_touch_s4(prepared_environment):
    prep, events, _ = prepared_environment
    context = prep.prepare(threading.Event(), mode='steering_pwm')
    assert events == []
    review = context['review']
    assert review['steering_pwm_mode'] is True and review['wheels_raised_confirmed_by_user'] is True
    assert review['neutral_physically_verified'] is False
    assert review['ground_continuous_requested_by_user'] is False
    prep.confirm(context)
    assert events == [] and review['neutral_physically_verified'] is False
    assert (Path(context['folder'])/'steering-ready.json').exists()
    assert not (Path(context['folder'])/'neutral-observation.json').exists()
