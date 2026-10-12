"""Secondary-camera QR search and explicit parking/payment UI state.

This wrapper never opens UART/PWM or calls a payment API. During a payment
window it ends existing driving tasks and rejects new movement requests.
"""
import json
import re
import threading
import time
from pathlib import Path
import cv2

from .brake_parking import numeric
from .parking_payment import PaymentWindow, detect_payment_codes, public_candidate


def outputs_released(s):
    return (s.get('mode')=='disabled' and not any(s.get(k) for k in ('hardware_output','worker_started','pending_hardware_start'))
            and not (s.get('traffic_trial') or {}).get('active')
            and not (s.get('crosswalk_trial') or {}).get('active')
            and not any(((s.get('cone_trial') or {}).get(k) or {}).get('active') for k in ('avoidance','short_test')))


def usable_code(c):
    return bool(c and c.get('camera_id')=='secondary' and c.get('kind')=='detected_qr'
                and c.get('decoded') is True and c.get('payload_text'))


def attach_payment_capture(state):
    """Keep full-size pixels only while searching, using the existing owner.

    The ordinary preview JPEG and every other perception input stay unchanged.
    """
    if getattr(state,'payment_capture_attached',False):return
    original=state.publish_raw
    def publish(jpeg,frame):
        original(jpeg,frame)
        if getattr(state,'payment_capture_requested',False):
            with state.condition:
                state.payment_full_frame=(frame.image.copy(),frame.frame_id,frame.receive_monotonic_ms)
    state.publish_raw=publish
    state.payment_capture_attached=True


class PaymentConsoleDrive:
    def __init__(self, drive, states, root, clock=time.monotonic, detector=detect_payment_codes):
        self.drive,self.states,self.root,self.clock,self.detector = drive,states,Path(root),clock,detector
        self.lock = threading.RLock()
        self.window = PaymentWindow(clock)
        self.closed = self.scanning = False
        self.thread = None
        self.scan_stop = threading.Event()
        self.scan_revision = 0
        self.scan_reason = '点击识别付款码，持续读取第二路 H65；未解出码不冻结、不锁车'
        self.candidates, self.selected = {}, None
        self.scan_errors = {}
        self.release_error = None
        self.record = None
        self.last_recorded_phase = 'idle'
        self.previous_report = None
        self.current_record = self.root/'run/parking-payment-current.json'
        self.scan_client = None
        if self.current_record.exists():
            try:
                saved=json.loads(self.current_record.read_text(encoding='utf-8'))
                old=saved.get('payment',{})
                if old.get('motion_locked') and re.fullmatch('[0-9a-f]{32}',old.get('session_id','')):
                    self.window.session_id=old['session_id'];self.window.locked=True
                    self.window.phase='interrupted'
                    self.window.reason='上次付款流程因服务重启中断；先核对微信结果，再结束旧记录，避免重复付款'
                    self.record=self.root/'run'/('parking-payment-'+old['session_id'])
                    self.last_recorded_phase='interrupted'
                    self.previous_report=saved.get('previous_report') or {k:old.get(k) for k in
                        ('operator_reported_paid','reported_within_window','provider_verified')}
            except (OSError,ValueError,TypeError):
                self.scan_reason='上次付款记录无法读取；开始前请核对微信是否已支付'

    def _record(self, event):
        if self.record is None: return
        self.record.mkdir(parents=True,exist_ok=True)
        s = self.window.status()
        # No QR payload, bank identifiers, control tokens or owner IDs in logs.
        s.pop('owner_client',None)
        row = {'event':event,'utc_unix_s':time.time(),'payment':s,
               'selected':None if not self.selected else {k:self.selected.get(k) for k in
                   ('id','camera_id','frame_id','captured_s','kind','decoded','payload_sha256','frame_sha256')},
               'release_error':self.release_error,'previous_report':self.previous_report}
        with (self.record/'events.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        temp = self.record/'summary.json.tmp'
        temp.write_text(json.dumps(row,ensure_ascii=False,indent=2),encoding='utf-8')
        temp.replace(self.record/'summary.json')
        current=self.current_record.with_suffix('.json.tmp')
        current.write_text(json.dumps(row,ensure_ascii=False,indent=2),encoding='utf-8')
        current.replace(self.current_record)
        if self.selected:
            (self.record/'selected-code.jpg').write_bytes(self.selected['image_jpeg'])
            (self.record/'selected-frame.jpg').write_bytes(self.selected['original_jpeg'])
        self.last_recorded_phase = s['phase']

    def status(self):
        with self.lock:
            base = self.drive.status()
            payment = self.window.status()
            if payment['phase'] != self.last_recorded_phase and self.record:
                self._record('window_state_changed')
            if payment['motion_locked']:
                for key in ('traffic_trial','crosswalk_trial'):
                    if key in base:
                        base[key] = {**base[key],'can_prepare':False,'can_start':False,'can_request_start':False}
                if 'cone_trial' in base:
                    base['cone_trial'] = {**base['cone_trial']}
                    for key in ('avoidance','short_test'):
                        if key in base['cone_trial']:
                            base['cone_trial'][key]={**base['cone_trial'][key],
                                'can_prepare':False,'can_start':False,'can_request_start':False}
            payment.update({'program':'parking-payment-v1','scanning':self.scanning,'scan_revision':self.scan_revision,
                'recognition_camera':'secondary','recognition_policy':'secondary_decoded_only',
                'can_begin':not self.scanning and not self.closed and not self.window.locked and usable_code(self.selected),
                'scan_reason':self.scan_reason,'scan_errors':dict(self.scan_errors),'can_scan':not self.scanning and not self.closed,
                'candidates':[public_candidate(v) for v in self.candidates.values()],
                'selected':public_candidate(self.selected) if self.selected else None,
                'selected_age_s':None if self.selected is None else max(0,self.clock()-self.selected['captured_s']),
                'previous_report':self.previous_report,
                'outputs_released':outputs_released(base),'release_error':self.release_error,
                'record_directory':str(self.record) if self.record else None})
            return {**base,'parking_payment':payment,'payment_status_s':self.clock()}

    def _finish_control(self, payload):
        if outputs_released(self.drive.status()): return
        try:
            self.drive.request('finish_test',payload)
        except (ValueError,RuntimeError) as exc:
            self.release_error = str(exc)
            # Emergency/stop is still allowed even if a preparation cannot finish.
            try: self.drive.request('stop',{'stop_source':'parking_payment'})
            except (ValueError,RuntimeError): pass

    def request(self, action, payload):
        if not isinstance(payload,dict): raise ValueError('JSON object required')
        with self.lock:
            if action=='status': return self.status()
            if not action.startswith('payment_'):
                if self.window.locked:
                    if action in ('stop','emergency','finish_test','traffic_cancel','crosswalk_cancel','cone_cancel','cone_stop','cone_short_stop'):
                        self._finish_control(payload)
                        return self.status() # Switching to WeChat must not reset/cancel the timer.
                    raise ValueError('停车缴费期间车辆已锁止；先结束本轮记录，才能重新准备行驶')
                self.drive.request(action,payload)
                return self.status()
            if self.closed: raise ValueError('服务已停止')
            client = payload.get('client')
            if not isinstance(client,str) or not 1 <= len(client) <= 128: raise ValueError('付款页面身份无效')
            if action=='payment_scan':
                if self.scanning: return self.status()
                self.scanning=True;self.scan_reason='正在持续识别第二路 H65；未解出码不冻结、不锁车';self.scan_errors={}
                if not self.window.locked:
                    self.selected=None;self.candidates={};self.scan_revision+=1
                self.scan_stop.clear()
                if 'secondary' in self.states:self.states['secondary'].payment_capture_requested=True
                self.scan_client=client
                self.thread=threading.Thread(target=self._scan,name='parking-qr',daemon=True)
                self.thread.start()
            elif action=='payment_scan_stop':
                self.scan_stop.set()
                self.scan_reason='已停止识别；未识别的画面不会被冻结或启动缴费锁止'
            elif action=='payment_select':
                if self.window.locked: self.window.check_owner(client,payload.get('payment_id'))
                candidate=self.candidates.get(payload.get('candidate_id'))
                if candidate is None: raise ValueError('此截图已更新，请选择当前批次中的付款码')
                if not usable_code(candidate):raise ValueError('第二路尚未解出二维码；此画面仅供观察，不冻结、不锁车')
                if self.scanning:raise ValueError('正在识别，请等待结果')
                self.selected=candidate
                if self.window.locked:self._record('operator_selected_original_code_image')
            elif action=='payment_begin':
                if payload.get('setup_token') != self.drive.status().get('setup_token'):
                    raise ValueError('页面已更新，请刷新后开始付款计时')
                if not self.window.locked:
                    if self.scanning or not usable_code(self.selected):
                        raise ValueError('第二路尚未解出并选定付款码，不开始计时或锁车')
                    if payload.get('selected_id') != self.selected['id']:
                        raise ValueError('付款码已变化，请核对当前图片后再开始')
                if self.window.begin(client,payload.get('parked_confirmed')):
                    self.record=self.root/'run'/('parking-payment-'+self.window.session_id)
                    self.previous_report=None
                    self.release_error=None
                    self._finish_control(payload)
                    self._record('operator_confirmed_four_wheels_stopped')
            elif action=='payment_report':
                if not outputs_released(self.drive.status()):
                    self._finish_control(payload)
                    raise ValueError('车辆控制还未完全释放，请保持停车后再记录回执')
                selected_id=self.selected['id'] if self.selected else None
                if payload.get('selected_id') != selected_id:
                    raise ValueError('付款截图已变化，请先核对当前显示的付款码')
                if self.window.report_paid(client,payload.get('payment_id'),payload.get('amount_cents'),payload.get('paid_confirmed')):
                    self._record('operator_reported_payment_1_cent')
            elif action=='payment_end':
                if payload.get('setup_token') != self.drive.status().get('setup_token'):
                    raise ValueError('请刷新当前控制页后结束付款记录')
                # Any current authorized control page can release an idle
                # payment interlock if the original browser tab was closed.
                if payload.get('payment_id') != self.window.session_id:
                    raise ValueError('付款记录已变化，请刷新后操作')
                if not self.window.locked: return self.status()
                if not outputs_released(self.drive.status()):
                    self._finish_control(payload)
                    raise ValueError('控制仍在结束中，请等待输出释放后再结束本轮')
                self.window.owner=client
                self.window.end(client,payload.get('payment_id'))
                self._record('operator_ended_payment_record')
                self.selected=None;self.candidates={};self.scan_revision+=1
            else: raise ValueError('未知付款动作')
            return self.status()

    def _scan(self):
        try:
            while not self.closed and not self.scan_stop.is_set():
                if self._scan_once():break
                if self.scan_stop.wait(.4):break
        finally:
            with self.lock:
                self.scanning=False
                if 'secondary' in self.states:self.states['secondary'].payment_capture_requested=False

    def _scan_once(self):
        candidates={};errors={}
        try:
            # Copy frames first; QR work never competes for USB ownership.
            frames=[]
            for camera in ('secondary',):
                try:
                    state=self.states[camera]
                    with state.condition:
                        jpg,fid,ms=state.raw_jpeg,state.raw_frame_id,state.raw_received_ms
                        full=getattr(state,'payment_full_frame',None)
                        failed=state.state in ('error','stopped')
                    if full is not None and numeric(full[2]) and 0<=self.clock()-full[2]/1000<=.8:
                        ok,encoded=cv2.imencode('.jpg',full[0],[cv2.IMWRITE_JPEG_QUALITY,95])
                        if ok:jpg,fid,ms=encoded.tobytes(),full[1],full[2]
                    if failed or not jpg or type(fid) is not int or not numeric(ms) or not 0 <= self.clock()-ms/1000 <= .8:
                        raise ValueError('没有新鲜画面')
                    frames.append((jpg,camera,fid,ms/1000))
                except (KeyError,ValueError) as exc: errors[camera]=str(exc)
            for row in frames:
                if self.closed: break
                try:
                    for candidate in self.detector(*row): candidates[candidate['id']]=candidate
                except Exception as exc: errors[row[1]]=str(exc)
            with self.lock:
                if self.closed or self.scan_stop.is_set(): return False
                self.candidates,self.scan_errors=candidates,errors
                self.scan_revision+=1
                codes=[c for c in candidates.values() if usable_code(c)]
                if not self.window.locked:self.selected=None
                # One uniquely decoded destination may be selected on first scan;
                # rescans never replace an already frozen payment image.
                hashes={c['payload_sha256'] for c in codes if c['decoded']}
                if (self.selected is None and len(hashes)==1 and all(c['decoded'] for c in codes)
                        and (not self.window.locked or self.scan_client==self.window.owner)):
                    self.selected=max(codes,key=lambda c:len(c['image_jpeg']))
                    if self.window.locked:self._record('single_decoded_code_frozen_after_scan')
                self.scan_reason=(f'第二路已解出{len(codes)}张二维码；请核对收款方，尚未锁车' if codes else
                                  '第二路未解出二维码；继续刷新识别，不冻结、不锁车' if candidates else
                                  '第二路暂无新鲜画面，继续等待；不切换第一路、不锁车')
                return bool(codes)
        except Exception:
            with self.lock:
                self.scan_reason='第二路识别发生异常，请重新识别；未开始缴费锁止'
            raise

    def image(self, candidate_id):
        with self.lock:
            c=self.selected if self.selected and self.selected['id']==candidate_id else self.candidates.get(candidate_id)
            return c['image_jpeg'] if c else None

    def close(self):
        with self.lock:
            self.closed=True
            self.scan_stop.set()
            if self.window.locked: self._record('service_stopped_during_payment_record')
            thread=self.thread
        if thread and thread is not threading.current_thread(): thread.join(timeout=2)
        # The existing launcher remains responsible for closing its own driver.
