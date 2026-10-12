"""Human-operated parking payment controls; decoded QR text stays inert."""

HTML = '''<section id="parking-payment" class="drive-panel">
<h3>停车缴费 · ¥0.01</h3><p>第二路 H65 识别付款码 · 未解出码不冻结、不锁车</p>
<p id="payment-state" role="status">尚未开始付款计时</p>
<div class="payment-top"><strong id="payment-time">30 秒</strong><span>从队员确认四轮停稳时开始；裁判以实际停车时刻为准。</span></div>
<div class="payment-actions"><button id="payment-scan" type="button">识别付款码</button>
<button id="payment-begin" type="button">四轮已停稳 · 开始计时</button></div>
<p id="payment-scan-state"></p><div id="payment-candidates" class="payment-candidates"></div>
<div id="payment-frozen" hidden><p id="payment-code-source"></p>
<img id="payment-code" alt="已冻结的真实付款码截图" width="480" height="480">
<p>请用手机微信扫一扫此图，核对收款方和金额 <b>0.01 元</b>。本程序不会自动打开二维码地址或扣款。</p>
<div class="payment-actions"><button id="payment-large" type="button">放大付款码</button>
<a id="payment-download" download="parking-payment-code.jpg">下载真实截图</a></div>
<details><summary>识别内容（仅显示）</summary><pre id="payment-payload"></pre></details></div>
<div class="payment-actions"><button id="payment-report" type="button" disabled>我已支付0.01元 · 记录回执</button>
<button id="payment-end" type="button" disabled>结束本轮记录</button></div>
<p id="payment-receipt" role="status"></p><p id="payment-error" role="alert"></p>
<details><summary>操作与状态说明</summary>
<p>点击识别后持续读取第二路原始画面；未解出码时继续刷新，不冻结、不锁车。解出后停止搜索，多个二维码时选择正确的一张。未解码的整幅画面仅供观察。</p>
<p>开始计时前可以重新识别，旧选择会清除。计时开始后的付款图保持冻结，重新识别失败不会取消已开始的缴费锁止。</p>
<p>车已停入规定区域且四轮静止后，点“开始计时”。30秒内由队员在微信完成支付，并向裁判展示支付结果。</p>
<p>“记录回执”只表示队员报告支付完成，不能自动核验到账或判定加分。迟到的回执会单独标记。无需在本页输入微信密码。</p>
<p>计时期间关闭行驶入口，付款后不会自动前进。切换到微信或刷新页面不重置计时；服务重启时不自动宣称支付成功。</p></details>
<dialog id="payment-dialog"><button id="payment-close-dialog" type="button">关闭放大图</button>
<img id="payment-large-code" alt="放大的真实付款码截图"><p>付款金额：0.01 元；请在微信中核对收款方。</p></dialog>
</section>'''

SCRIPT = '''
function renderParkingPayment(s){
 if(s.control_session_id&&s.control_session_id!==pageSession){expiredPage();return;}
 const at=s.payment_status_s;
 if(typeof at==='number'&&at<paymentLastStatusAt)return;
 if(typeof at==='number')paymentLastStatusAt=at;
 const p=s.parking_payment||{},own=p.owner_client===paymentClient;
 paymentLocked=!!p.motion_locked;paymentId=p.session_id;paymentSelectedId=p.selected?.id||null;
 el('payment-state').textContent=p.reason||'付款模块未加载';
 el('payment-time').textContent=p.remaining_s==null?'30 秒':p.phase==='paid_reported'?'已记录回执':p.remaining_s>0?p.remaining_s.toFixed(1)+' 秒':'窗口已结束';
 el('payment-scan').textContent=p.scanning?'停止识别':'识别付款码';
 el('payment-scan').disabled=stale||paymentBusy||!p.can_scan&&!p.scanning;
 el('payment-begin').disabled=stale||paymentBusy||!p.can_begin;
 el('payment-begin').title=p.can_begin?'第二路已解出付款码；确认四轮停稳后开始计时':'第二路尚未解出并选定付款码，不开始计时或锁车';
 el('payment-report').disabled=stale||paymentBusy||!paymentLocked||!own||p.operator_reported_paid||!p.outputs_released||!['awaiting_payment','expired'].includes(p.phase);
 el('payment-end').disabled=stale||paymentBusy||!paymentLocked||!p.outputs_released;
 el('payment-scan-state').textContent=p.scan_reason||'';
 const chosen=p.selected;
 el('payment-frozen').hidden=!chosen;
 if(chosen){
  const url='/api/payment/image.jpg?id='+encodeURIComponent(chosen.id);
  if(el('payment-code').getAttribute('src')!==url){el('payment-code').src=url;el('payment-large-code').src=url;el('payment-download').href=url;}
  el('payment-code-source').textContent='已冻结 '+Math.floor(p.selected_age_s||0)+' 秒 · '+(chosen.camera_id==='primary'?'第一路':'第二路')+' #'+chosen.frame_id+' · '+chosen.description;
  el('payment-payload').textContent=chosen.payload_text||'未解出内容；仍可扫描上方原始截图。';
 }
 const revision=String(p.scan_revision)+'/'+(chosen?.id||'')+'/'+paymentLocked+'/'+own+'/'+!!p.scanning;
 if(revision!==paymentGalleryRevision){
  paymentGalleryRevision=revision;el('payment-candidates').replaceChildren();
  for(const c of p.candidates||[]){
   const button=document.createElement('button');button.type='button';button.className='payment-candidate';
   const picture=document.createElement('img');picture.src='/api/payment/image.jpg?id='+encodeURIComponent(c.id);picture.alt=c.description;
   const label=document.createElement('span');label.textContent='第二路 · '+(c.kind==='detected_qr'?'二维码截图':'观察画面')+(c.decoded?'（已解码）':'（未解码，不锁定）');
   button.append(picture,label);button.disabled=stale||p.scanning||paymentLocked&&!own||!c.decoded||c.camera_id!=='secondary'||c.kind!=='detected_qr';
   button.addEventListener('click',()=>paymentAction('payment_select',{candidate_id:c.id}));
   el('payment-candidates').append(button);
  }
 }
 el('payment-receipt').textContent=p.operator_reported_paid?(p.reported_within_window?'队员已报告支付0.01元；到账未自动核验，请展示微信成功页。':'已记录迟到回执；不据此认定限时加分。'):p.previous_report?.operator_reported_paid?'上次记录中队员已报告支付；请先核对微信结果，避免重复支付。':'';
 if(paymentLocked&&!own&&p.phase!=='interrupted')el('payment-error').textContent='本轮由另一页面开始，请回原页面记录回执；原窗口已关闭时，可先核对结果再结束旧记录。';
 else if(p.release_error)el('payment-error').textContent='控制结束提示：'+p.release_error;
 if(!paymentLocked)return;
 lastStatus=s;allowed=false;turnAvailable=false;motorAvailable=false;reverseAvailable=false;
 held.clear();needsRelease=true;heartbeatDue=false;gamepadControl?.requireNeutral();
 el('drive-title').textContent='停车缴费 · 车辆锁止';el('drive-mode').textContent=p.reason;
 el('drive-action').textContent=p.outputs_released?'控制输出已释放':'正在结束车辆控制';
 for(const id of ['drive-enable','drive-reset','drive-next','drive-focus','test-mode','gamepad-arm','crosswalk-prepare','crosswalk-start','traffic-prepare','traffic-start','cone-prepare','cone-drive-start','cone-start','cone-short-prepare'])if(el(id))el(id).disabled=true;
 document.querySelectorAll('.cone-short-starts button').forEach(b=>b.disabled=true);
 document.querySelectorAll('[data-drive],[data-speed]').forEach(b=>b.disabled=true);
}
async function paymentAction(action,body={}){
 if(paymentBusy)return;
 paymentBusy=true;el('payment-error').textContent='';if(lastStatus)render(lastStatus);
 try{render(await send(action,{client:paymentClient,payment_id:paymentId,selected_id:paymentSelectedId,...body}));}
 catch(e){el('payment-error').textContent=e.message;}
 finally{paymentBusy=false;if(lastStatus)render(lastStatus);}
}
el('payment-scan').addEventListener('click',()=>paymentAction(lastStatus?.parking_payment?.scanning?'payment_scan_stop':'payment_scan'));
el('payment-begin').addEventListener('click',()=>paymentAction('payment_begin',{parked_confirmed:true}));
el('payment-report').addEventListener('click',()=>paymentAction('payment_report',{amount_cents:1,paid_confirmed:true}));
el('payment-end').addEventListener('click',()=>paymentAction('payment_end'));
el('payment-large').addEventListener('click',()=>el('payment-dialog').showModal());
el('payment-close-dialog').addEventListener('click',()=>el('payment-dialog').close());
'''


def extend_controls(controls):
    script=controls.SCRIPT
    sentinel='let gamepadControl=null,pwmControl=null;'
    if script.count(sentinel)!=1 or script.count('function render(s){')!=1 or script.count('async function heartbeat(){')!=1:
        raise ValueError('付款扩展需要当前控制台脚本，未匹配时不覆盖原控制')
    script=script.replace(sentinel,sentinel+'''
let paymentLocked=false,paymentId=null,paymentSelectedId=null,paymentBusy=false,paymentGalleryRevision=null,paymentLastStatusAt=-Infinity;
let paymentClient=client;
try{paymentClient=sessionStorage.getItem('smartcar-payment-client')||client;sessionStorage.setItem('smartcar-payment-client',paymentClient);}catch(e){}
''',1)
    script=script.replace('function render(s){','function renderBeforePayment(s){',1)
    script=script.replace('async function heartbeat(){','async function heartbeat(){\n if(paymentLocked){heartbeatDue=false;return;}',1)
    wrapper='''
function render(s){
 if(typeof acceptTrafficSnapshot==='function'&&!acceptTrafficSnapshot(s))return;
 if(typeof s.payment_status_s==='number'&&s.payment_status_s<paymentLastStatusAt)return;
 renderBeforePayment(s);renderParkingPayment(s);
}
'''
    end=script.rfind('})();')
    if end<0:raise ValueError('控制台闭包未找到')
    controls.SCRIPT=script[:end]+wrapper+SCRIPT+script[end:]
    controls.HTML+=HTML
    controls.STYLE=getattr(controls,'STYLE','')+'''<style>
#parking-payment{grid-column:1;background:white;border-radius:18px;padding:20px;display:grid;gap:12px;min-width:0;color:#24212b}
#parking-payment h3,#parking-payment p{margin:0;line-height:1.6}#parking-payment p{color:#75677e}
.payment-top{display:flex;align-items:center;gap:15px}.payment-top strong{font-size:28px;color:#8e35e8;white-space:nowrap}.payment-top span{font-size:12px;color:#75677e}
.payment-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px}#parking-payment button,#parking-payment a{min-height:42px;border:1px solid #d8d8e4;border-radius:10px;padding:8px;text-align:center}
#parking-payment button:enabled{background:#8e35e8;color:white;border-color:#8e35e8}#parking-payment button:disabled{opacity:.45}
#parking-payment #payment-end{background:#fff1f2;color:#b42335;border-color:#fecdd3}#parking-payment a{display:grid;place-items:center;color:#8e35e8;text-decoration:none}
.payment-candidates{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.payment-candidate{display:grid;gap:6px;background:#faf8fc!important;color:#24212b!important;font-size:11px}
.payment-candidate img{width:100%;height:100px;object-fit:contain}.payment-candidate span{overflow-wrap:anywhere}
#payment-code{display:block;width:100%;max-width:480px;height:auto;max-height:500px;object-fit:contain;background:#fff;margin:8px auto;border:1px solid #eee;border-radius:8px}
#payment-payload{white-space:pre-wrap;overflow-wrap:anywhere;max-height:150px;overflow:auto;font-size:11px}#payment-error{color:#b42335!important}
#payment-dialog{max-width:min(90vw,850px);border:1px solid #ddd;border-radius:14px;background:white;color:#24212b}#payment-dialog::backdrop{background:#0008}
#payment-large-code{display:block;max-width:85vw;max-height:75vh;object-fit:contain;background:white;margin:12px auto}
@media(max-width:500px){.payment-candidates{grid-template-columns:1fr 1fr}.payment-actions{grid-template-columns:1fr}.payment-top{align-items:flex-start}}
</style>'''


def add_endpoints(server, drive):
    from urllib.parse import urlsplit,parse_qs
    original=server.RequestHandlerClass
    class Handler(original):
        def do_GET(self):
            url=urlsplit(self.path)
            if url.path!='/api/payment/image.jpg':return super().do_GET()
            image=drive.image(parse_qs(url.query).get('id',[''])[0])
            try:return self.send_body(image or b'Unknown payment snapshot','image/jpeg' if image else 'text/plain',200 if image else 404)
            except (OSError,ConnectionError):self.close_connection=True
    server.RequestHandlerClass=Handler
    return server
