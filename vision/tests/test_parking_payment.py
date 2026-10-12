import hashlib
import json
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from carvision.parking_payment import PaymentWindow,detect_payment_codes,public_candidate
from carvision.payment_console import PaymentConsoleDrive,outputs_released,attach_payment_capture
from carvision.payment_controls import extend_controls,add_endpoints


def test_payment_window_is_exactly_one_cent_and_duplicate_begin_does_not_extend():
    now=[10.];p=PaymentWindow(lambda:now[0]);p.begin('a',True);sid=p.session_id
    assert p.deadline==40 and p.status()['expected_amount_cents']==1
    now[0]=20;assert not p.begin('a',True) and p.deadline==40 and p.session_id==sid
    assert not p.status()['provider_verified'] and not p.status()['money_transferred_by_program']


@pytest.mark.parametrize('time,phase',[(29.99,'paid_reported'),(30.,'paid_reported'),(30.01,'late_reported')])
def test_payment_report_boundary_is_receipt_arrival_not_provider_verification(time,phase):
    now=[0.];p=PaymentWindow(lambda:now[0]);p.begin('a',True);now[0]=time
    assert p.report_paid('a',p.session_id,1,True)
    assert p.phase==phase and p.locked and not p.status()['provider_verified']
    assert not p.report_paid('a',p.session_id,1,True)


@pytest.mark.parametrize('amount',[0,2,.01,True,'1',None])
def test_amount_must_be_integer_one_cent(amount):
    p=PaymentWindow(lambda:0);p.begin('a',True)
    with pytest.raises(ValueError):p.report_paid('a',p.session_id,amount,True)
    assert p.reported_at is None


def test_expiry_never_releases_vehicle_or_becomes_paid():
    now=[0.];p=PaymentWindow(lambda:now[0]);p.begin('a',True);now[0]=100
    s=p.status();assert s['phase']=='expired' and s['motion_locked'] and not s['operator_reported_paid']
    with pytest.raises(ValueError):p.report_paid('other',p.session_id,1,True)
    with pytest.raises(ValueError):p.report_paid('a','old',1,True)
    p.end('a',p.session_id);assert not p.locked


def qr_jpeg(value='TEST_ONLY_PARKING_NOT_A_PAYMENT',rotation=False):
    qr=cv2.QRCodeEncoder_create().encode(value)
    qr=cv2.copyMakeBorder(qr,4,4,4,4,cv2.BORDER_CONSTANT,value=255)
    qr=cv2.resize(qr,(232,232),interpolation=cv2.INTER_NEAREST)
    if rotation:qr=cv2.rotate(qr,cv2.ROTATE_90_CLOCKWISE)
    image=np.full((360,480,3),210,np.uint8);image[50:282,100:332]=cv2.cvtColor(qr,cv2.COLOR_GRAY2BGR)
    ok,jpg=cv2.imencode('.jpg',image);assert ok
    return jpg.tobytes()


@pytest.mark.parametrize('rotation',[False,True])
def test_actual_qr_detector_decodes_original_pixels_without_creating_payment_url(rotation):
    jpg=qr_jpeg(rotation=rotation)
    rows=detect_payment_codes(jpg,'secondary',7,1.)
    codes=[r for r in rows if r['kind']=='detected_qr']
    assert len(codes)==1 and codes[0]['payload_text']=='TEST_ONLY_PARKING_NOT_A_PAYMENT'
    assert codes[0]['original_jpeg']==jpg and codes[0]['frame_sha256']==hashlib.sha256(jpg).hexdigest()
    assert codes[0]['image_jpeg'] and 'image_jpeg' not in public_candidate(codes[0])
    assert rows[-1]['kind']=='full_frame' and rows[-1]['image_jpeg']==jpg
    # The real HTTP handler serializes these rows without a NumPy encoder.
    json.dumps([public_candidate(r) for r in rows],allow_nan=False)


def test_unknown_frame_has_real_full_frame_fallback_not_a_generated_code():
    ok,jpg=cv2.imencode('.jpg',np.full((360,480,3),200,np.uint8));assert ok
    rows=detect_payment_codes(jpg.tobytes(),'primary',1,1.)
    assert len(rows)==1 and rows[0]['kind']=='full_frame' and not rows[0]['decoded']


def test_qr_text_is_returned_as_untrusted_text_even_if_it_looks_like_script():
    text='javascript:alert(1)'
    rows=detect_payment_codes(qr_jpeg(text),'primary',1,1.)
    assert any(r['payload_text']==text for r in rows)  # No HTTP/browser call exists in this module.


class Driver:
    def __init__(self):
        self.state={'mode':'disabled','hardware_output':False,'worker_started':False,'setup_token':'boot',
                    'crosswalk_trial':{'active':False},'traffic_trial':{'active':False},
                    'cone_trial':{'avoidance':{'active':False}}}
        self.calls=[]
    def status(self):return dict(self.state)
    def request(self,action,payload):
        self.calls.append((action,payload))
        if action=='finish_test':self.state.update(mode='disabled',hardware_output=False,worker_started=False)


@pytest.fixture
def console(tmp_path):
    now=[0.];base=Driver();w=PaymentConsoleDrive(base,{},tmp_path,clock=lambda:now[0])
    return w,base,now


def begin(w):
    if w.selected is None:
        w.selected=next(c for c in detect_payment_codes(qr_jpeg(),'secondary',1,w.clock()) if c['decoded'])
    return w.request('payment_begin',{'client':'page','setup_token':'boot','parked_confirmed':True,'selected_id':w.selected['id']})


def report(w,**kw):
    return w.request('payment_report',{'client':'page','payment_id':w.window.session_id,
                      'amount_cents':1,'paid_confirmed':True,'selected_id':w.selected['id'] if w.selected else None,**kw})


def test_idle_scan_and_status_never_open_motor_and_manual_passthrough_still_works(console):
    w,base,_=console
    assert not w.status()['parking_payment']['motion_locked']
    w._scan_once();assert not base.calls and not w.candidates
    w.request('command',{'motor':'forward'})
    assert base.calls[-1][0]=='command'


def test_begin_ends_manual_and_blocks_all_new_motion_while_recording(console):
    w,base,now=console;base.state.update(mode='enabled',hardware_output=True,worker_started=True)
    begin(w);assert base.calls[-1][0]=='finish_test' and w.window.locked
    for action in ['command','enable','crosswalk_start','traffic_start','cone_prepare','cone_start','cone_short_prepare','cone_short_start']:
        with pytest.raises(ValueError):w.request(action,{'client':'page'})
    now[0]=31;assert w.status()['parking_payment']['phase']=='expired'
    assert w.window.locked and outputs_released(base.status())


def test_payment_waits_for_cone_short_output_and_disables_both_cone_paths(console):
    w,base,_=console
    base.state['cone_trial']['short_test']={'active':True,'can_start':True,'can_prepare':True}
    assert not outputs_released(base.status())
    begin(w)
    assert base.calls[-1][0]=='finish_test'
    state=w.status()
    assert not state['cone_trial']['short_test']['can_start']
    with pytest.raises(ValueError):report(w)
    base.state['cone_trial']['short_test']['active']=False
    report(w)
    assert w.window.status()['operator_reported_paid']


def test_window_does_not_reset_on_tab_blur_stop_or_scan(console):
    w,base,now=console;begin(w);deadline=w.window.deadline
    now[0]=10;w.request('stop',{'stop_source':'window_blur'});w._scan_once();begin(w)
    assert w.window.deadline==deadline and w.window.phase=='awaiting_payment'
    assert not base.calls


def test_stale_page_unconfirmed_parked_and_wrong_amount_never_start_fake_receipt(console):
    w,_,_=console
    with pytest.raises(ValueError):w.request('payment_begin',{'client':'page','setup_token':'old','parked_confirmed':True})
    with pytest.raises(ValueError):w.request('payment_begin',{'client':'page','setup_token':'boot','parked_confirmed':False})
    assert not w.window.locked
    begin(w)
    with pytest.raises(ValueError):report(w,paid_confirmed=False)
    assert not w.window.status()['operator_reported_paid']


def test_operator_report_persists_without_qr_payload_or_claiming_verified_payment(console):
    w,_,now=console;begin(w);now[0]=4;report(w)
    data=json.loads((w.record/'summary.json').read_text(encoding='utf-8'))
    assert data['payment']['operator_reported_paid'] and data['payment']['reported_within_window']
    assert not data['payment']['provider_verified'] and not data['payment']['money_transferred_by_program']
    assert 'owner_client' not in data['payment']
    w.request('payment_end',{'client':'page','setup_token':'boot','payment_id':w.window.session_id})
    assert not w.window.locked


def test_payment_restart_does_not_restart_thirty_seconds_or_report_success(console):
    w,base,now=console;begin(w);now[0]=10
    restored=PaymentConsoleDrive(base,{},w.root,clock=lambda:100)
    assert restored.window.phase=='interrupted' and restored.window.deadline is None
    assert restored.window.locked and not restored.window.status()['operator_reported_paid']
    with pytest.raises(ValueError):begin(restored)
    restored.request('payment_end',{'client':'new-page','setup_token':'boot','payment_id':restored.window.session_id})
    assert not restored.window.locked


def test_paid_report_survives_restart_as_history_and_old_image_clears_on_end(console):
    w,base,now=console;begin(w);report(w)
    restored=PaymentConsoleDrive(base,{},w.root,clock=lambda:100)
    assert restored.status()['parking_payment']['previous_report']['operator_reported_paid'] is True
    restored.request('payment_end',{'client':'new','setup_token':'boot','payment_id':restored.window.session_id})
    data=json.loads((restored.record/'summary.json').read_text(encoding='utf-8'))
    assert data['previous_report']['operator_reported_paid'] and restored.selected is None


def test_current_authorized_page_can_end_idle_window_without_auto_driving(console):
    w,base,_=console;begin(w)
    with pytest.raises(ValueError):w.request('payment_end',{'client':'other','setup_token':'old','payment_id':w.window.session_id})
    w.request('payment_end',{'client':'other','setup_token':'boot','payment_id':w.window.session_id})
    assert not w.window.locked and not base.calls


def test_scan_freezes_selected_code_and_rescan_does_not_replace_it(console):
    w,base,now=console;now[0]=1
    state=SimpleNamespace(condition=threading.RLock(),raw_jpeg=qr_jpeg(),raw_frame_id=9,raw_received_ms=1000,state='ok')
    w.states={'secondary':state};w._scan_once()
    assert w.selected and w.selected['decoded'] and not base.calls
    selected=w.selected['id'];original=w.image(selected);begin(w)
    state.raw_jpeg=qr_jpeg('SECOND_TEST_NOT_PAYMENT');state.raw_frame_id=10;w._scan_once()
    assert w.selected['id']==selected and w.image(selected)==original
    assert not base.calls
    begin(w)
    with pytest.raises(ValueError):report(w,selected_id='another-code')


def test_ui_keeps_original_renderer_and_never_executes_decoded_text():
    c=SimpleNamespace(SCRIPT='''<script>(()=>{let gamepadControl=null,pwmControl=null;
function render(s){originalRender(s);}
async function heartbeat(){originalHeartbeat();}
})();</script>''',HTML='<p>existing</p>',STYLE='')
    extend_controls(c)
    assert 'renderBeforePayment(s);renderParkingPayment(s)' in c.SCRIPT
    assert 'originalRender(s)' in c.SCRIPT and 'originalHeartbeat()' in c.SCRIPT
    assert "el('payment-payload').textContent" in c.SCRIPT and '.innerHTML' not in c.SCRIPT
    assert 'window.open' not in c.SCRIPT and 'sessionStorage' in c.SCRIPT
    assert '0.01' in c.HTML and 'payment-report' in c.HTML


def test_image_endpoint_only_returns_selected_memory_images(console):
    w,_,_=console
    w.candidates={'known':{'image_jpeg':b'jpeg'}}
    class Base:
        def do_GET(self):self.reply='original'
        def send_body(self,*args):self.reply=args
    server=SimpleNamespace(RequestHandlerClass=Base);add_endpoints(server,w)
    h=server.RequestHandlerClass();h.path='/api/payment/image.jpg?id=known';h.do_GET();assert h.reply==(b'jpeg','image/jpeg',200)
    h.path='/api/payment/image.jpg?id=../../secret';h.do_GET();assert h.reply[2]==404
    h.path='/api/status';h.do_GET();assert h.reply=='original'


def test_no_code_cannot_freeze_or_lock_and_primary_cannot_supply_payment(console):
    w,base,now=console;now[0]=1
    _,blank=cv2.imencode('.jpg',np.full((360,480,3),200,np.uint8))
    def state(jpg):return SimpleNamespace(condition=threading.RLock(),raw_jpeg=jpg,raw_frame_id=1,raw_received_ms=1000,state='ok')
    w.states={'secondary':state(blank.tobytes()),'primary':state(qr_jpeg())}
    w._scan_once()
    assert w.selected is None and not w.window.locked and not base.calls
    assert all(c['camera_id']=='secondary' for c in w.candidates.values())
    c=next(iter(w.candidates.values()))
    with pytest.raises(ValueError):w.request('payment_select',{'client':'page','candidate_id':c['id']})
    with pytest.raises(ValueError):w.request('payment_begin',{'client':'page','setup_token':'boot','parked_confirmed':True})
    assert not w.window.locked and not base.calls and not w.status()['parking_payment']['can_begin']


def test_selected_secondary_real_code_survives_status_and_record_json(console):
    w,base,now=console;now[0]=1
    w.states={'secondary':SimpleNamespace(condition=threading.RLock(),raw_jpeg=qr_jpeg(),raw_frame_id=1,raw_received_ms=1000,state='ok')}
    w._scan_once()
    state=json.loads(json.dumps(w.status(),allow_nan=False))
    assert state['parking_payment']['selected']['camera_id']=='secondary'
    assert state['parking_payment']['can_begin'] and not w.window.locked and not base.calls
    w.request('payment_begin',{'client':'page','setup_token':'boot','parked_confirmed':True,'selected_id':w.selected['id']})
    assert (w.record/'summary.json').is_file()


def test_missing_secondary_never_uses_primary_or_locks(console):
    w,base,now=console;now[0]=1
    w.states={'primary':SimpleNamespace(condition=threading.RLock(),raw_jpeg=qr_jpeg(),raw_frame_id=1,raw_received_ms=1000,state='ok')}
    assert w._scan_once() is False and w.selected is None
    assert 'secondary' in w.scan_errors and not w.window.locked and not base.calls


def test_missed_rescan_clears_old_selection_before_payment_but_not_during_payment(console):
    w,base,now=console;now[0]=1
    state=SimpleNamespace(condition=threading.RLock(),raw_jpeg=qr_jpeg(),raw_frame_id=1,raw_received_ms=1000,state='ok')
    w.states={'secondary':state};w._scan_once();assert w.selected
    _,blank=cv2.imencode('.jpg',np.full((360,480,3),200,np.uint8))
    state.raw_jpeg=blank.tobytes();w._scan_once()
    assert w.selected is None and not w.window.locked
    begin(w);chosen=w.selected;w._scan_once()
    assert w.window.locked and w.selected is chosen


def test_search_keeps_retrying_without_freezing_and_can_stop_without_motor_commands(console):
    w,base,now=console;now[0]=1;done=threading.Event();count=[0]
    def detector(*args):
        count[0]+=1
        if count[0]>=2:done.set()
        return []
    w.detector=detector
    w.states={'secondary':SimpleNamespace(condition=threading.RLock(),raw_jpeg=qr_jpeg(),raw_frame_id=1,raw_received_ms=1000,state='ok')}
    w.request('payment_scan',{'client':'page'})
    assert done.wait(2)
    assert w.scanning and w.selected is None and not w.window.locked and not base.calls
    w.request('payment_scan_stop',{'client':'page'});w.thread.join(2)
    assert not w.scanning and not w.states['secondary'].payment_capture_requested


def test_full_resolution_capture_preserves_original_preview_and_uses_existing_owner(console):
    w,base,now=console;now[0]=1;calls=[]
    small=cv2.imdecode(np.frombuffer(qr_jpeg(),np.uint8),cv2.IMREAD_COLOR)
    pixels=cv2.resize(small,(640,480),interpolation=cv2.INTER_NEAREST)
    state=SimpleNamespace(condition=threading.RLock(),state='ok')
    def original(jpg,frame):
        calls.append(jpg);state.raw_jpeg=jpg;state.raw_frame_id=frame.frame_id;state.raw_received_ms=frame.receive_monotonic_ms
    state.publish_raw=original;attach_payment_capture(state);attach_payment_capture(state)
    frame=SimpleNamespace(image=pixels,frame_id=9,receive_monotonic_ms=1000)
    state.publish_raw(qr_jpeg(),frame)
    assert not hasattr(state,'payment_full_frame')
    state.payment_capture_requested=True;state.publish_raw(qr_jpeg(),frame)
    assert len(calls)==2 and state.raw_jpeg==calls[-1]
    assert state.payment_full_frame[0] is not frame.image
    w.states={'secondary':state};w._scan_once()
    assert w.selected and w.selected['frame_id']==9
    assert cv2.imdecode(np.frombuffer(w.selected['original_jpeg'],np.uint8),cv2.IMREAD_COLOR).shape[:2]==(480,640)
    assert not base.calls
