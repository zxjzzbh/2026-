"""A whole-race entry above the existing, retained single-task panels."""
HTML='''<section id="race-flow-panel" class="drive-panel">
<h3>完整比赛流程 · 手动切入自动驾驶</h3>
<p>遥控到切换区并停稳，保持电调 F/B，再点击下方入口。接管后先等待蓝色挡板移开，不会立即前进。</p>
<p>挡板移开后前进并寻找斑马线（不播报发车） → 斑马线停 10 秒并播报 → 红灯停、绿灯继续 → 两桶避障 → 空闲半区停车 → 30 秒扫码缴费。</p>
<button id="race-enter" type="button">已在切换区停稳 · 进入自动模式（F/B）</button>
<div class="race-actions"><button id="race-stop" type="button">急停总流程</button><button id="race-exit" type="button">退出总流程，恢复单项测试</button></div>
<p id="race-stage" role="status">完整流程尚未启用</p><p id="race-reason" role="status"></p>
<p id="race-progress"></p><p id="race-outputs"></p>
<div class="race-preview"><img id="race-preview" alt="当前比赛任务的真实相机画面" hidden></div>
<p id="race-payment"></p><button id="race-paid" type="button" disabled>我已实际支付 0.01 元 · 记录付款结果</button>
<p>扫码支付由队员完成，二维码沿用下方缴费面板。程序不自动付款，也不把按钮声明当作支付平台到账证明。</p>
<p id="race-readiness"></p>
<p>右转 1350 · 中位 1610 · 左转 1870。急停退出后可继续使用下方各任务单测。</p>
</section>'''
STATE='''
let raceActive=false,raceOwned=false,raceToken=null,raceSequence=0,raceBusy=false,raceHeartbeatBusy=false,raceImageBusy=false;
let raceQueue=Promise.resolve(),raceLastSnapshot=-Infinity,raceImageURL=null;
const raceDisabled=new Map();
'''
SCRIPT='''
function renderRaceFlow(s){
 const r=s.race_flow||{};raceActive=!!r.active;raceOwned=r.owner_client===client;raceToken=r.run_token;
 el('race-enter').disabled=stale||raceBusy||!r.can_enter;
 el('race-paid').disabled=stale||raceBusy||!raceOwned||!r.can_report_paid;
 el('race-stage').textContent='当前阶段：'+(r.stage_label||'未加载全流程服务');
 el('race-reason').textContent=r.reason||'请使用完整流程启动入口加载程序';
 el('race-progress').textContent=(r.transitions||[]).map(x=>({wait_board:'等挡板',crosswalk:'斑马线',traffic:'红绿灯',cones:'绕桶',parking_search:'找车位',parking_entry:'入位',payment:'缴费',complete:'结束'}[x.to]||x.to)).join(' → ');
 el('race-outputs').textContent=r.intent?('本轮指令：'+r.intent.action+' · 油门 '+r.intent.esc_us+' μs · 转向 '+r.intent.steering_us+' μs'):'等待手动进入自动模式';
 el('race-readiness').textContent=r.parking_missing?.length?'停车入位反馈尚未齐备，届时会停车提示；其余任务可联调。':'停车反馈配置已载入，仍需场地验证。';
 el('race-payment').textContent=r.payment_remaining_s==null?'入位停稳后开始 30 秒缴费窗口':('缴费剩余 '+r.payment_remaining_s.toFixed(1)+' 秒'+(r.operator_reported_paid?' · 已记录付款声明，未核验平台到账':''));
 if(raceActive){
  held.clear();heartbeatDue=false;allowed=false;needsRelease=true;motorAvailable=false;turnAvailable=false;gamepadControl?.requireNeutral();
  for(const n of document.querySelectorAll('#crosswalk-test button,#traffic-test button,#cone-test button,#departure-test button,#parking-bay-test button,[data-drive],[data-speed]')){
   raceDisabled.set(n,n.disabled);n.disabled=true;
  }
  for(const id of ['drive-enable','drive-reset','drive-next','gamepad-arm']){const n=el(id);if(n){raceDisabled.set(n,n.disabled);n.disabled=true;}}
 }
}
async function raceRequest(action,body={}){
 const token=raceToken;
 const run=()=>send(action,{...body,run_token:token,race_sequence:++raceSequence});
 const result=raceQueue.then(run,run);raceQueue=result.catch(()=>{});return result;
}
el('race-enter').addEventListener('click',async()=>{
 if(raceBusy)return;raceBusy=true;
 try{render(await send('race_enter',{setup_token:lastStatus?.setup_token,stationary_confirmed:true,in_switch_zone:true,fb_mode_confirmed:true}));}
 catch(e){feedback(e.message,'error');}finally{raceBusy=false;if(lastStatus)render(lastStatus);}
});
el('race-stop').addEventListener('click',()=>stop('emergency',false,'race_stop'));
el('race-exit').addEventListener('click',async()=>{try{render(await send('race_exit'));}catch(e){feedback(e.message,'error');}});
el('race-paid').addEventListener('click',async()=>{
 if(raceBusy)return;raceBusy=true;
 try{render(await raceRequest('race_report_paid',{operator_paid:true}));}
 catch(e){feedback(e.message,'error');}finally{raceBusy=false;if(lastStatus)render(lastStatus);}
});
setInterval(async()=>{
 if(!raceActive||!raceOwned||stale||document.hidden||raceHeartbeatBusy)return;
 raceHeartbeatBusy=true;try{render(await raceRequest('race_keepalive'));}
 catch(e){feedback(e.message,'error');stop('stop',false,'race_connection_lost');}finally{raceHeartbeatBusy=false;}
},100);
setInterval(async()=>{
 if(!raceActive||stale||document.hidden||raceImageBusy)return;
 raceImageBusy=true;const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1500);
 try{const response=await fetch('/api/race-flow/frame.jpg',{cache:'no-store',signal:controller.signal});
  if(!response.ok)throw new Error('no fresh image');const blob=await response.blob(),url=URL.createObjectURL(blob),old=raceImageURL;
  raceImageURL=url;el('race-preview').src=url;el('race-preview').hidden=false;if(old)URL.revokeObjectURL(old);
 }catch(e){el('race-preview').hidden=true;}finally{clearTimeout(timer);raceImageBusy=false;}
},125);
'''


def extend_controls(controls):
    script=controls.SCRIPT;marker='function render(s){';sentinel='let gamepadControl=null,pwmControl=null;'
    if script.count(marker)!=1 or script.count(sentinel)!=1 or controls.HTML.count('<div id="autonomy-tests">')!=1:
        raise ValueError('完整流程需要既有遥控、斑马线、红绿灯、锥桶、发车和停车组合控制台')
    script=script.replace(marker,'function renderBeforeRaceFlow(s){',1).replace(sentinel,sentinel+STATE,1)
    script=script.replace('async function heartbeat(){','async function heartbeat(){\n if(raceActive){heartbeatDue=false;return;}',1)
    wrapper='''function render(s){
 if(typeof s.race_flow_status_s==='number'){if(s.race_flow_status_s<raceLastSnapshot)return;raceLastSnapshot=s.race_flow_status_s;}
 for(const [node,disabled] of raceDisabled)node.disabled=disabled;raceDisabled.clear();
 renderBeforeRaceFlow(s);renderRaceFlow(s);
}
'''
    end=script.rfind('})();')
    if end<0:raise ValueError('控制台闭包缺失')
    controls.SCRIPT=script[:end]+wrapper+SCRIPT+script[end:]
    controls.HTML=controls.HTML.replace('<div id="autonomy-tests">','<div id="autonomy-tests">'+HTML,1)
    controls.STYLE=getattr(controls,'STYLE','')+'''<style>
#race-flow-panel{background:#fff;border-radius:18px;padding:20px;display:grid;gap:12px;min-width:0}
#race-flow-panel p,#race-flow-panel h3{margin:0;line-height:1.65}#race-flow-panel p{color:#75677e}
#race-flow-panel button{min-height:44px;border:1px solid #d8d8e4;border-radius:10px;padding:10px;background:#8e35e8;color:white}
#race-flow-panel button:disabled{opacity:.45}#race-flow-panel #race-stop{background:#b42335}.race-actions{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.race-preview img{width:100%;max-height:400px;object-fit:contain;border-radius:10px}
</style>'''


def add_endpoints(server,drive):
    from urllib.parse import urlsplit
    original=server.RequestHandlerClass
    class Handler(original):
        def do_GET(self):
            if urlsplit(self.path).path!='/api/race-flow/frame.jpg':return super().do_GET()
            with drive.lock:
                fresh=drive.updated is not None and 0<=drive.clock()-drive.updated<=.25
                data=drive.preview if fresh else None
            try:return self.send_body(data or b'No fresh race frame','image/jpeg' if data else 'text/plain',200 if data else 503)
            except (OSError,ConnectionError):self.close_connection=True
    server.RequestHandlerClass=Handler;return server
