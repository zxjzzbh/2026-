"""Parking observation and entry trial on the existing real-car console."""
HTML='''<section class="drive-panel" id="parking-bay-test">
<h3>停车区 · 选区与入位判断</h3>
<p>停车区长 1 m，沿赛道中线分成两个半区。左／右按车辆前进方向定义，选择入口没有挡板的一侧。</p>
<p id="parking-bay-state" role="status">等待停车观察服务</p>
<div class="parking-bay-actions"><button id="parking-bay-observe" type="button" disabled>开始停车区识别</button><button id="parking-bay-stop" type="button">停止识别</button></div>
<p id="parking-bay-selection" role="status" aria-live="polite" data-state="waiting">等待开始识别</p><p id="parking-bay-geometry"></p><p id="parking-bay-slots"></p>
<div class="parking-bay-viewer"><img id="parking-bay-preview" alt="第二路实际黄线、挡板和停车半区识别" width="480" height="360" hidden><span id="parking-bay-placeholder">开始后显示实际摄像头识别图</span></div>
<details><summary>识别参数</summary>
<label>测试地面<select id="parking-bay-surface"><option value="auto">自动识别红／绿地面</option><option value="red_running_track">室外红色跑道</option><option value="indoor_green">室内绿色地面</option></select></label>
<label>赛道总宽（cm）<input id="parking-bay-width" type="number" min="60" max="200" step="0.5" value="123"></label>
<label>入口蓝色占比阈值（%）<input id="parking-bay-blue" type="number" min="0.5" max="30" step="0.5" value="3"></label>
<label>空闲半区可见地面（%）<input id="parking-bay-floor" type="number" min="50" max="95" step="1" value="65"></label>
<label>视觉入位末段油门（μs）<input id="parking-visual-slow" type="number" min="1501" max="1625" step="1" value="1550"></label>
<label>自定义减速线高度（%）<input id="parking-visual-slow-row" type="number" min="30" max="93" step="0.1" placeholder="留空沿用原图像参考"></label>
<label>自定义停止线高度（%）<input id="parking-visual-stop-row" type="number" min="32" max="95" step="0.1" placeholder="留空沿用原图像参考"></label>
<p>两条图像线须一起设置或一起留空；高度从图像顶部向下计算，减速线至少早于停止线2%。请结合实际停点调整；保存图像参考不代表完成距离或制动标定。</p>
<button id="parking-bay-save" type="button">保存识别参数</button><p>地面类型默认为自动；室内绿地面与室外红色跑道也可分别指定。未见蓝色不直接等于空闲；黄线、边界和车位内部还需明确。</p></details>
<p id="parking-bay-readiness"></p><p id="parking-bay-error" role="alert"></p>
<p>识别过程中持续给出左／右建议；依据不足时标为暂定。选区建议不代表已确认可安全入位。</p>
<div id="parking-trial-test">
<h4>低速停车入位</h4>
<label>入库方式<select id="parking-trial-controller"><option value="metric_path">路径跟踪入库</option><option value="visual_trial">原视觉入位试验（对照）</option></select></label>
<p id="parking-trial-mode-help"></p>
<p id="parking-trial-missing" role="status"></p>
<label>入位前进油门（μs）<input id="parking-trial-forward" type="number" min="1501" max="1625" step="1" value="1575"></label>
<label>入位转向增益<input id="parking-trial-gain" type="number" min="1" max="3" step="0.05" value="1.25"></label>
<p>油门在下次准备时生效。准备后打开电调、保持车辆静止，再点击开始；本项会实际行驶并自动刹停。</p>
<p>路径入库：确认空位 → 规划入口前转弯 → 跟随路径 → 末段减速 → 停稳确认。原视觉试验保留原有寻位行为，仅供对照。</p>
<p>视觉对照模式：白线短暂缺失时结合连续车位边界低速修正；证据不足则暂停等待，持续不足退出。寻位默认最多5秒，总入位最多30秒；末段降低油门输出。</p>
<div class="parking-bay-actions"><button id="parking-trial-prepare" type="button" disabled>准备停车（电调关闭、F/B）</button><button id="parking-trial-start" type="button" disabled>开始低速入位</button></div>
<button id="parking-trial-cancel" type="button">停止入位／结束本轮</button>
<p id="parking-trial-state" role="status">等待停车入位服务</p><p id="parking-trial-output"></p>
<p>路径入库需要当前有效的车位、位姿和运动标定。原视觉试验沿用50 cm图像停车参考，不代表四轮物理入位验证。空格／Esc可急停。</p>
</div>
<a href="#parking-payment">入位停稳后，前往下方停车缴费</a>
<p>这里不自动开始30秒付款计时，不把选区成功当作四轮已经入位。</p>
</section>'''

SCRIPT='''
function renderParkingBay(s){
 const p=s.parking_bay||{},cfg=p.settings||{},obs=p.observation;parkingBayActive=!!p.active;
 el('parking-bay-state').textContent=p.settings_error||((obs&&!obs.layout_found)?obs.reason:null)||p.reason||'停车观察模块未加载';
 el('parking-bay-observe').disabled=stale||parkingBayBusy||!!s.parking_trial?.active||!p.can_observe;
 el('parking-bay-stop').disabled=stale||parkingBayBusy||!!s.parking_trial?.active;
 for(const id of ['parking-bay-surface','parking-bay-width','parking-bay-blue','parking-bay-floor','parking-visual-slow','parking-visual-slow-row','parking-visual-stop-row','parking-bay-save'])el(id).disabled=stale||parkingBayBusy||parkingBayActive||!!s.parking_trial?.active;
 if(!parkingBayDirty&&!parkingBayBusy){
  el('parking-bay-surface').value=cfg.surface||'auto';
  el('parking-bay-width').value=(cfg.lane_width_m||1.23)*100;
  el('parking-bay-blue').value=(cfg.blocked_blue_fraction||.03)*100;
  el('parking-bay-floor').value=(cfg.clear_ground_fraction||.65)*100;
  el('parking-visual-slow').value=cfg.visual_entry?.slow_forward_us??1550;
  el('parking-visual-slow-row').value=cfg.visual_entry?.slow_line_y_normalized==null?'':cfg.visual_entry.slow_line_y_normalized*100;
  el('parking-visual-stop-row').value=cfg.visual_entry?.stop_line_y_normalized==null?'':cfg.visual_entry.stop_line_y_normalized*100;
 }
 const selected=p.selection||{},names={left:'左半区',right:'右半区'},labels={clear:'可见空闲候选',blocked:'入口遮挡',unknown:'未知'},surfaces={red_running_track:'红色地面',indoor_green:'绿色地面',auto:'地面'};
 const pct=value=>typeof value==='number'&&Number.isFinite(value)?(value*100).toFixed(1)+'%':'待判断';
 const hasSelection=Object.prototype.hasOwnProperty.call(names,selected.selected_slot);
 if(hasSelection)parkingBayLastSelection={...selected};
 else if(!parkingBayActive)parkingBayLastSelection=null;
 const display=hasSelection?selected:(parkingBayActive?parkingBayLastSelection:null),selectionNode=el('parking-bay-selection');
 if(display){
  const expired=!!stale||!!display.stale,retained=!hasSelection;
  const confirmed=parkingBayActive&&hasSelection&&selected.confirmed===true&&!selected.tentative&&!expired;
  const basis=display.suggestion_basis||display.reason||'当前画面的左右入口对比';
  const detail=expired?' · 画面过期，仅保留建议':retained?' · 接口刷新中，仅保留上次建议':!parkingBayActive?' · 识别已停止，仅保留建议':'';
  selectionNode.textContent=(confirmed?'推荐':'暂定')+names[display.selected_slot]+(confirmed?'（视觉已确认）':'（尚未确认）')+detail+' · 依据：'+basis;
  selectionNode.dataset.state=confirmed?'confirmed':'tentative';
 }else{
  selectionNode.textContent=parkingBayActive?'正在读取选区建议':'等待开始识别';
  selectionNode.dataset.state='waiting';
 }
 el('parking-bay-geometry').textContent=!obs?'等待第二路新鲜画面':!obs.layout_found?'停车结构尚未确认':obs.layout_complete===false?'已见部分停车结构；缺失边界不当作实测车位':'已见两个停车半区；'+(obs.metric_geometry?'已关联米制几何':'当前为图像位置，尚无米制几何');
 el('parking-bay-slots').textContent=(obs?.slots||[]).map(r=>names[r.id]+'：'+labels[r.availability]+(r.geometry_visible===false?' · 挡板已见，边界被遮挡':' · 入口蓝色'+pct(r.blue_entrance_fraction)+' · '+(surfaces[r.ground_surface]||'地面')+pct(r.ground_fraction??r.red_ground_fraction))).join(' / ');
 el('parking-bay-readiness').textContent=p.readiness?.ready?'停车几何配置已载入，仍需检查运动反馈和场地验证。':'米制入位反馈尚未齐备；下方会显示具体缺项，选区建议仍可观察。';
 parkingBayFrame=p.observation?.frame_id??null;
 renderParkingTrial(s);
}
function renderParkingTrial(s){
 const t=s.parking_trial||{},nextToken=t.run_token??null;
 if(nextToken!==parkingTrialToken){parkingTrialSequence=0;parkingTrialGeneration++;}
 parkingTrialActive=!!t.active;parkingTrialOwned=t.owner_client===client;parkingTrialToken=nextToken;
 if(!parkingTrialActive)parkingTrialStopping=false;
 const locked=stale||parkingTrialBusy||parkingTrialStopping;
 const selector=el('parking-trial-controller');
 if(parkingTrialActive)selector.value=t.controller_mode||'visual_trial';
 selector.disabled=locked||parkingTrialActive;
 const metric=selector.value==='metric_path',missing=t.metric_missing||[];
 el('parking-trial-mode-help').textContent=metric?'速度采用实测档位表，按剩余路径减速；下方手动油门不参与路径模式。':'视觉入位：白线与连续车位边界共同判断；证据不足暂停，恢复且停稳后继续。寻位有时限，末段降低油门；实际停点仍需实测。';
 el('parking-trial-missing').textContent=metric?(missing.length?'待补齐：'+missing.join('；'):'参数已齐备；开始后还须有新鲜米制车位和位姿反馈。'):'当前为原视觉对照试验。';
 el('parking-trial-prepare').disabled=locked||parkingTrialActive||!t.can_prepare||(metric&&missing.length>0);
 el('parking-trial-start').disabled=locked||!parkingTrialActive||!parkingTrialOwned||!t.can_request_start;
 el('parking-trial-forward').disabled=locked||parkingTrialActive||metric;
 el('parking-trial-gain').disabled=locked||parkingTrialActive||metric;
 el('parking-trial-state').textContent=parkingTrialError||t.reason||'等待停车入位服务';
 const side=t.selected_slot==='left'?'左半区':t.selected_slot==='right'?'右半区':'待选择';
 const wb=t.intent?.white_boundary,whiteText=wb?.observed?(wb.correction==='right'?'向右纠偏':wb.correction==='left'?'向左纠偏':'边界可用于纠偏'):wb?.usable?'当前车位边界提供补充判断':(wb?.reason||'等待有效边界');
 el('parking-trial-output').textContent=t.intent?'当前目标：'+side+(t.retarget_count?' · 已改选 '+t.retarget_count+' 次':'')+' · 油门 '+t.intent.esc_us+' μs · 转向 '+t.intent.steering_us+' μs · 白线：'+whiteText+(t.hardware_output?' · 实车输出':' · 无实车输出'):'开始前采用最新有效选区；本项独立于绕桶测试。';
 const tracking=t.intent?.parking_decision?.tracking;
 if(tracking&&typeof tracking.remaining_m==='number')el('parking-trial-output').textContent+=' · 剩余路径 '+tracking.remaining_m.toFixed(2)+' m · 路径偏差 '+(tracking.tracking_error_m*100).toFixed(1)+' cm';
 if(!parkingTrialActive)return;
 lastStatus=s;held.clear();heartbeatDue=false;needsRelease=true;allowed=false;turnAvailable=false;motorAvailable=false;reverseAvailable=false;gamepadControl?.requireNeutral();
 for(const id of ['drive-enable','drive-reset','drive-next','drive-speed','test-mode','gamepad-arm','drive-focus','crosswalk-prepare','crosswalk-start','traffic-prepare','traffic-start','cone-test-prepare','cone-test-start','departure-start','race-enter','parking-bay-observe','parking-bay-stop']){
  const n=document.getElementById(id);if(n){parkingTrialDisabled.set(n,n.disabled);n.disabled=true;}
 }
 for(const n of document.querySelectorAll('[data-drive],[data-speed]')){parkingTrialDisabled.set(n,n.disabled);n.disabled=true;}
 const title=document.getElementById('drive-title'),mode=document.getElementById('drive-mode'),action=document.getElementById('drive-action');
 if(title)title.textContent='停车入位单项试验';if(mode)mode.textContent=t.reason||'准备停车';if(action)action.textContent=t.intent?.action||t.phase||'准备';
}
function parkingTrialRunSend(action,body={}){
 const token=parkingTrialToken,generation=parkingTrialGeneration;
 const run=()=>{
  if(!parkingTrialActive||!parkingTrialOwned||stale||document.hidden||parkingTrialStopping||token!==parkingTrialToken||generation!==parkingTrialGeneration)throw new Error('本轮已结束或页面不可控制');
  return send(action,{run_token:token,trial_sequence:++parkingTrialSequence,...body});
 };
 const result=parkingTrialQueue.then(run,run);parkingTrialQueue=result.catch(()=>{});return result;
}
async function parkingTrialPrepare(){
 if(parkingTrialBusy||parkingTrialActive||parkingTrialStopping||stale)return;
 const input=el('parking-trial-forward');if(!input.checkValidity()){input.reportValidity();return;}
 const gain=el('parking-trial-gain');if(!gain.checkValidity()){gain.reportValidity();return;}
 parkingTrialBusy=true;parkingTrialError='';if(lastStatus)render(lastStatus);
 try{render(await send('parking_trial_prepare',{setup_token:lastStatus?.setup_token,esc_off:true,fb_mode_confirmed:true,
  selected_slot:lastStatus?.parking_bay?.selection?.selected_slot||'right',forward_us:Number(input.value),
  steering_gain:Number(gain.value),controller_mode:el('parking-trial-controller').value}));}
 catch(e){parkingTrialError=e.message;}
 finally{parkingTrialBusy=false;if(lastStatus)render(lastStatus);}
}
async function parkingTrialStart(){
 if(parkingTrialBusy||parkingTrialStopping)return;parkingTrialBusy=true;parkingTrialError='';if(lastStatus)render(lastStatus);
 try{render(await parkingTrialRunSend('parking_trial_start',{esc_on_static:true,field_ready:true,fb_mode_confirmed:true}));}
 catch(e){parkingTrialError=e.message;}
 finally{parkingTrialBusy=false;if(lastStatus)render(lastStatus);}
}
async function parkingTrialCancel(action='parking_trial_cancel',keepalive=false){
 parkingTrialGeneration++;parkingTrialStopping=true;parkingTrialError='';
 held.clear();heartbeatDue=false;allowed=false;needsRelease=true;gamepadControl?.requireNeutral();
 if(lastStatus)render(lastStatus);
 try{render(await send(action,{},keepalive));}
 catch(e){parkingTrialError='停车确认未收到：'+e.message;}
 finally{if(!parkingTrialActive)parkingTrialStopping=false;if(lastStatus)render(lastStatus);}
}
async function parkingBayAction(action){
 if(parkingBayBusy||parkingTrialActive)return;parkingBayBusy=true;el('parking-bay-error').textContent='';
 const slowRow=el('parking-visual-slow-row'),stopRow=el('parking-visual-stop-row'),slowPulse=el('parking-visual-slow');
 for(const input of [slowRow,stopRow,slowPulse])if(!input.checkValidity()){input.reportValidity();parkingBayBusy=false;return;}
 const visual={...(lastStatus?.parking_bay?.settings?.visual_entry||{}),slow_forward_us:Number(slowPulse.value),
  slow_line_y_normalized:slowRow.value===''?null:Number(slowRow.value)/100,
  stop_line_y_normalized:stopRow.value===''?null:Number(stopRow.value)/100};
 const body={settings_revision:lastStatus?.parking_bay?.settings_revision,surface:el('parking-bay-surface').value,lane_width_m:Number(el('parking-bay-width').value)/100,
  blocked_blue_fraction:Number(el('parking-bay-blue').value)/100,clear_ground_fraction:Number(el('parking-bay-floor').value)/100,visual_entry:visual};
 if(lastStatus)render(lastStatus);
 try{const s=await send(action,body);if(action==='parking_bay_save')parkingBayDirty=false;render(s);}
 catch(e){el('parking-bay-error').textContent=e.message;}
 finally{parkingBayBusy=false;if(lastStatus)render(lastStatus);}
}
el('parking-bay-observe').addEventListener('click',()=>parkingBayAction('parking_bay_observe'));
el('parking-bay-stop').addEventListener('click',()=>parkingBayAction('parking_bay_stop'));
el('parking-bay-save').addEventListener('click',()=>parkingBayAction('parking_bay_save'));
el('parking-trial-prepare').addEventListener('click',()=>parkingTrialPrepare());
el('parking-trial-start').addEventListener('click',()=>parkingTrialStart());
el('parking-trial-cancel').addEventListener('click',()=>parkingTrialCancel());
el('parking-trial-controller').addEventListener('change',()=>{if(lastStatus)renderParkingTrial(lastStatus);});
for(const id of ['parking-bay-surface','parking-bay-width','parking-bay-blue','parking-bay-floor','parking-visual-slow','parking-visual-slow-row','parking-visual-stop-row'])el(id).addEventListener('input',()=>parkingBayDirty=true);
setInterval(async()=>{
 if(!parkingTrialActive||!parkingTrialOwned||stale||document.hidden||parkingTrialPulseBusy||parkingTrialStopping)return;
 parkingTrialPulseBusy=true;
 try{await parkingTrialRunSend('parking_trial_keepalive',{compact_ack:true});}
 catch(e){if(parkingTrialActive&&!parkingTrialStopping){parkingTrialError=e.message;await parkingTrialCancel();}}
 finally{parkingTrialPulseBusy=false;}
},100);
setInterval(async()=>{
 if(stale||document.hidden||!parkingBayActive||parkingBayImageBusy||parkingBayFrame==null||parkingBayLoaded===parkingBayFrame)return;
 parkingBayImageBusy=true;const fid=parkingBayFrame,controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1500);
 try{
  const r=await fetch('/api/parking-bay/frame.jpg',{cache:'no-store',signal:controller.signal});if(!r.ok)throw new Error('No fresh frame');
  const url=URL.createObjectURL(await r.blob()),old=parkingBayURL;parkingBayURL=url;parkingBayLoaded=fid;
  el('parking-bay-preview').src=url;el('parking-bay-preview').hidden=false;el('parking-bay-placeholder').hidden=true;if(old)URL.revokeObjectURL(old);
 }catch(e){el('parking-bay-placeholder').textContent='等待停车区实际画面';}
 finally{clearTimeout(timer);parkingBayImageBusy=false;}
},400);
'''


def extend_controls(controls):
    marker='function render(s){';sentinel='let gamepadControl=null,pwmControl=null;';html='<section id="parking-payment"'
    script=controls.SCRIPT
    if script.count(marker)!=1 or script.count(sentinel)!=1 or controls.HTML.count(html)!=1:raise ValueError('停车观察需要现有缴费控制台')
    script=script.replace(marker,'function renderBeforeParkingBay(s){',1)
    script=script.replace(sentinel,sentinel+'''let parkingBayActive=false,parkingBayBusy=false,parkingBayDirty=false,parkingBayLast=-Infinity;
let parkingBayFrame=null,parkingBayLoaded=null,parkingBayURL=null,parkingBayImageBusy=false,parkingBayLastSelection=null;
let parkingTrialActive=false,parkingTrialOwned=false,parkingTrialToken=null,parkingTrialSequence=0,parkingTrialGeneration=0,parkingTrialLastSnapshot=-Infinity;
let parkingTrialBusy=false,parkingTrialStopping=false,parkingTrialPulseBusy=false,parkingTrialError='',parkingTrialQueue=Promise.resolve();
const parkingTrialDisabled=new Map();
''',1)
    script=script.replace('async function heartbeat(){','async function heartbeat(){\n if(parkingTrialActive){heartbeatDue=false;return;}',1)
    stop_marker="function stop(action='stop',keepalive=false,stopSource='unknown'){"
    script=script.replace(stop_marker,stop_marker+"\n if(parkingTrialActive){void parkingTrialCancel(action==='emergency'?'emergency':'parking_trial_cancel',keepalive);return;}",1)
    wrapper='''function render(s){
 if(typeof acceptTrafficSnapshot==='function'&&!acceptTrafficSnapshot(s))return;
 if(typeof s.parking_bay_status_s==='number'){if(s.parking_bay_status_s<parkingBayLast)return;parkingBayLast=s.parking_bay_status_s;}
 if(typeof s.parking_trial_status_s==='number'){if(s.parking_trial_status_s<parkingTrialLastSnapshot)return;parkingTrialLastSnapshot=s.parking_trial_status_s;}
 for(const [node,disabled] of parkingTrialDisabled)node.disabled=disabled;parkingTrialDisabled.clear();
 renderBeforeParkingBay(s);renderParkingBay(s);
}
'''
    end=script.rfind('})();')
    if end<0:raise ValueError('控制台闭包缺失')
    controls.SCRIPT=script[:end]+wrapper+SCRIPT+script[end:]
    controls.HTML=controls.HTML.replace(html,HTML+html,1)
    controls.STYLE=getattr(controls,'STYLE','')+'''<style>
#parking-bay-test{grid-column:1;background:white;border-radius:18px;padding:20px;display:grid;gap:12px;min-width:0}
#parking-bay-test h3,#parking-bay-test p{margin:0;line-height:1.6}#parking-bay-test p{color:#75677e}
#parking-bay-test #parking-bay-selection{padding:12px;border-radius:10px;background:#f7f4fa;font-weight:600;color:#55465f}
#parking-bay-test #parking-bay-selection[data-state="tentative"]{background:#fff5dc;color:#7a5100}
#parking-bay-test #parking-bay-selection[data-state="confirmed"]{background:#eaf7ed;color:#236436}
#parking-trial-test{display:grid;gap:10px;padding:14px;border:1px solid #ddd1eb;border-radius:12px}#parking-trial-test h4{margin:0}#parking-bay-test #parking-trial-cancel{background:#b42335;color:white}
.parking-bay-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px}#parking-bay-test button,#parking-bay-test input,#parking-bay-test select{min-height:42px;border:1px solid #d8d8e4;border-radius:10px;padding:8px}
#parking-bay-test button:enabled{background:#8e35e8;color:white}#parking-bay-test button:disabled{opacity:.45}
#parking-bay-test label{display:flex;justify-content:space-between;gap:10px;margin:8px 0}#parking-bay-test input{width:90px}
.parking-bay-viewer{position:relative;aspect-ratio:4/3;background:#f7f4fa;display:grid;place-items:center;overflow:hidden;border-radius:10px}
#parking-bay-preview{position:absolute;width:100%;height:100%;object-fit:contain}#parking-bay-error{color:#b42335!important}
</style>'''


def add_endpoints(server,drive):
    from urllib.parse import urlsplit
    original=server.RequestHandlerClass
    class Handler(original):
        def do_GET(self):
            if urlsplit(self.path).path!='/api/parking-bay/frame.jpg':return super().do_GET()
            jpg=drive.image()
            try:return self.send_body(jpg or b'No fresh parking frame','image/jpeg' if jpg else 'text/plain',200 if jpg else 503)
            except (OSError,ConnectionError):self.close_connection=True
    server.RequestHandlerClass=Handler;return server
