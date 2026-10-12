"""Real QR capture and an operator-reported 1-cent payment window.

Never contacts a decoded URL, a payment provider, or a motion driver. A paid
report is a human assertion, not confirmation from WeChat or the recipient.
"""
import hashlib
import secrets

import cv2
import numpy as np


class PaymentWindow:
    window_s = 30.0
    amount_cents = 1

    def __init__(self, clock):
        self.clock = clock
        self.phase = 'idle'
        self.session_id = self.owner = None
        self.started = self.deadline = self.reported_at = None
        self.locked = False
        self.reason = '先识别付款码；车辆四轮停稳后再开始30秒缴费计时'

    def tick(self):
        if self.phase == 'awaiting_payment' and self.clock() > self.deadline:
            self.phase, self.reason = 'expired', '30秒窗口已结束；车辆保持锁止，可记录迟到回执'

    def begin(self, client, parked_confirmed):
        self.tick()
        if parked_confirmed is not True:
            raise ValueError('需要队员确认四轮已经停稳且位于停车区')
        if self.locked:
            if client != self.owner: raise ValueError('请在开始本轮付款的页面操作')
            return False  # Double click / retry cannot extend the deadline.
        self.session_id, self.owner = secrets.token_hex(16), client
        self.started = self.clock()
        self.deadline = self.started+self.window_s
        self.reported_at = None
        self.locked = True
        self.phase, self.reason = 'awaiting_payment', '请用微信扫描真实付款码，核对收款方与0.01元金额，完成后记录回执'
        return True

    def check_owner(self, client, session_id):
        if not self.locked or client != self.owner or session_id != self.session_id:
            raise ValueError('付款记录已变化，请使用开始本轮付款的页面')

    def report_paid(self, client, session_id, amount_cents, paid_confirmed):
        self.check_owner(client,session_id)
        self.tick()
        if type(amount_cents) is not int or amount_cents != 1 or paid_confirmed is not True:
            raise ValueError('仅记录队员明确报告已支付0.01元；本按钮不会发起扣款')
        if self.reported_at is not None: return False
        self.reported_at = self.clock()
        timely = self.reported_at <= self.deadline
        self.phase = 'paid_reported' if timely else 'late_reported'
        self.reason = ('队员已报告支付0.01元，请向裁判展示微信支付成功页；尚未自动核验到账' if timely else
                       '已记录迟到的支付回执，不能据此认定获得限时加分；车辆保持锁止')
        return True

    def end(self, client, session_id):
        self.check_owner(client,session_id)
        self.tick()
        self.locked = False
        self.phase, self.reason = 'ended', '本轮记录已结束；不会自动恢复行驶'

    def status(self):
        self.tick()
        return {'phase':self.phase,'reason':self.reason,'session_id':self.session_id,'owner_client':self.owner,
                'motion_locked':self.locked,'expected_amount_cents':1,'window_s':self.window_s,
                'remaining_s':None if self.deadline is None else max(0,self.deadline-self.clock()),
                'started_monotonic_s':self.started,'deadline_monotonic_s':self.deadline,
                'operator_reported_paid':self.reported_at is not None,
                'reported_within_window':None if self.reported_at is None else self.reported_at <= self.deadline,
                'provider_verified':False,'parking_verified_by_sensor':False,
                'timer_source':'operator_four_wheels_stopped_confirmation',
                'money_transferred_by_program':False}


def detect_payment_codes(jpeg, camera, frame_id, captured_s):
    """Return actual frame/crops. QR text is untrusted data and is never opened."""
    if not jpeg or len(jpeg) > 2_000_000: raise ValueError('付款码图像为空或过大')
    image = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    if image is None or image.shape[0]*image.shape[1] > 1_500_000:
        raise ValueError('付款码图像解码失败或尺寸过大')
    detector = cv2.QRCodeDetector()
    points, values = [], []
    try:
        found, decoded, quads, _ = detector.detectAndDecodeMulti(image)
        if found and quads is not None: points, values = list(quads), list(decoded)
    except (cv2.error,UnicodeError):
        pass
    if not points:
        try:
            value, quad, _ = detector.detectAndDecode(image)
            if quad is not None: points, values = [quad.reshape(4,2)], [value]
        except (cv2.error,UnicodeError):
            pass
    method='original'
    # Small printed modules are often lost by the detector's sampling. A
    # bounded cubic upsample helps decoding; all crop coordinates are mapped
    # back to the actual original pixels, and no QR is regenerated.
    if not any(values) and image.shape[0]*image.shape[1] <= 375_000:
        enlarged=cv2.resize(image,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
        try:
            found,decoded,quads,_=detector.detectAndDecodeMulti(enlarged)
            if found and quads is not None and any(decoded):
                points,values=list(quads/2),list(decoded);method='cubic_2x'
        except (cv2.error,UnicodeError):pass
    # Even failed decoding can still give a real QR crop for the phone scanner.
    if not points:
        try:
            found, quads = detector.detectMulti(image)
            if found and quads is not None: points, values = list(quads), ['']*len(quads)
        except cv2.error:
            pass
    h,w = image.shape[:2]
    result = []
    for i, quad in enumerate(points[:6]):
        quad = np.asarray(quad,dtype=float).reshape(4,2)
        if not np.isfinite(quad).all() or abs(cv2.contourArea(quad.astype(np.float32))) < 25: continue
        x1,y1 = map(int,np.floor(quad.min(axis=0)))
        x2,y2 = map(int,np.ceil(quad.max(axis=0)))
        margin = max(10,round(max(x2-x1,y2-y1)*.18))
        x1,y1,x2,y2 = max(0,x1-margin),max(0,y1-margin),min(w,x2+margin+1),min(h,y2+margin+1)
        if x2 <= x1 or y2 <= y1: continue
        crop = image[y1:y2,x1:x2]
        factor = max(1,min(4,800/max(crop.shape[:2])))
        if factor > 1: crop = cv2.resize(crop,None,fx=factor,fy=factor,interpolation=cv2.INTER_NEAREST)
        ok, encoded = cv2.imencode('.jpg',crop,[cv2.IMWRITE_JPEG_QUALITY,95])
        if not ok: continue
        value = values[i] if i < len(values) and isinstance(values[i],str) else ''
        value = value[:4096]
        result.append({'id':secrets.token_hex(12),'camera_id':camera,'frame_id':frame_id,'captured_s':captured_s,
                       'kind':'detected_qr','decoded':bool(value),'payload_text':value,
                       'decode_method':method,'source_size':[int(w),int(h)],
                       'payload_sha256':hashlib.sha256(value.encode()).hexdigest() if value else None,
                       'frame_sha256':hashlib.sha256(jpeg).hexdigest(),'bbox_xyxy':[int(v) for v in (x1,y1,x2,y2)],
                       'image_jpeg':encoded.tobytes(),'original_jpeg':jpeg,
                       'description':'真实二维码截图' if value else '检测到二维码，内容未解出；可用微信扫描截图'})
    # A real whole-frame fallback avoids pretending detection always succeeds.
    result.append({'id':secrets.token_hex(12),'camera_id':camera,'frame_id':frame_id,'captured_s':captured_s,
                   'kind':'full_frame','decoded':False,'payload_text':'','payload_sha256':None,
                   'frame_sha256':hashlib.sha256(jpeg).hexdigest(),'bbox_xyxy':None,
                   'image_jpeg':jpeg,'original_jpeg':jpeg,'description':'整幅实拍画面（手动查看付款码）'})
    return result


def public_candidate(row):
    return {k:v for k,v in row.items() if k not in ('image_jpeg','original_jpeg')}
