"""Traffic controls inserted into the existing real-camera driving console."""

HTML = '''<section class="drive-panel" id="traffic-test">
<h3>红绿灯测试 · F/B</h3>
<p>双路使用同一识别算法 · 任一路达标即确认 · 红灯优先</p>
<p id="traffic-test-state" role="status">正在读取车端状态</p>
<div class="traffic-throttle"><label for="traffic-forward">接近油门（μs）</label>
<div class="traffic-throttle-steps"><button id="traffic-forward-down" type="button" aria-label="接近油门降低 5 微秒">−5</button>
<input id="traffic-forward" type="number" min="1501" max="1625" step="1" value="1600">
<button id="traffic-forward-up" type="button" aria-label="接近油门增加 5 微秒">+5</button></div></div>
<p>接近档默认 1600 μs；可调 1501–1625 μs，建议每次 5 μs。停止后调整并保存，下次准备生效。</p>
<div class="traffic-threshold"><label for="traffic-red-percent">红绿灯共用比例（%）</label>
<input id="traffic-red-percent" type="number" min="1" max="100" step="1" value="50">
<button id="traffic-save" type="button">保存参数</button></div>
<p id="traffic-settings-state"></p>
<div class="traffic-actions"><button id="traffic-prepare" type="button" disabled>准备（电调关闭）</button>
<button id="traffic-start" type="button" disabled>开始</button>
<button id="traffic-cancel" type="button">停止</button></div>
<p id="traffic-start-hint" role="status">先准备，再开始。</p>
<div class="traffic-observation"><strong id="traffic-color">等待准备</strong><span id="traffic-vision"></span></div>
<p id="traffic-intent" hidden></p><p id="traffic-red-vote" role="status"></p><p id="traffic-speech-state" role="status" hidden></p>
<p>首次：电调保持 F/B，关闭电调点“准备”；打开电调、确认静止且前方空旷后点“开始”。</p>
<p>3 秒后按已保存油门搜索，无需先看见灯具。两路分别统计最近 2 秒，任一路红灯占比达标时制动；停稳后任一路绿灯占比达标且无红灯达标时播报，保持停车。</p>
<details id="traffic-preview-section"><summary>实时识别画面</summary>
<div class="traffic-viewer"><img id="traffic-preview" alt="双路实际识别画面：左第一路，右第二路H65" width="960" height="360" hidden>
<span id="traffic-preview-placeholder">准备后显示双路识别画面：左第一路，右第二路 H65</span></div></details>
<details><summary>参数与播报说明</summary>
<div class="traffic-parameters">
<label for="traffic-approach">接近最长时间（秒）</label><input id="traffic-approach" type="number" min="0.5" max="10" step="0.5" value="5">
</div>
<p>红、绿共用 1–100% 阈值和 2 秒窗口；分别对两路计票，不混合帧数，重复／过期图像不累计。停止后修改并保存。准备完成即可开始，不检查是否看见灯具。</p>
<p>两路都运行原第一路的灯具定位与发光判断算法，第二路继续提供地面运动估计。任一路达红灯条件即停车；若同时有红、绿达标，红灯优先。黄灯、运动反馈失效、两路断帧、失联或接近超时会停车。</p>
<p>本轮确认红灯并取得停稳估计后重新收集 2 秒窗口，任一路绿灯占比达共用阈值且无红灯／黄灯阻挡时，播报“红绿灯结束，开始前行”，车辆保持停止。取消原连续绿灯0.6秒规则。</p>
<p>光电传感器由场地控制灯具，未接入车端。本程序不猜测随机倒计时，也不把灯色当作前轮位置。</p>
<p>首次在灯具正前方直线路段测试，让摄像头始终看清灯具；核对前轮停车位置是否位于传感器与灯具之间。油门数值是控制指令，不等于实际车速；提高后需重新检查停车距离。持续前进但无运动反馈仍会退出。</p>
<p>此入口测试红绿灯阶段，不自动从斑马线切入。前进搜索不会自动延长时限，未确认绿灯不会完成停车流程。参数只在停止后可改，遥控与斑马线测试互斥。</p></details>
</section>'''

SCRIPT = '''
function acceptTrafficSnapshot(s){
 if(s.control_session_id&&s.control_session_id!==pageSession){expiredPage();return false;}
 const at=s.traffic_status_s;
 if(typeof at==='number'&&Number.isFinite(at)){
  if(at<trafficLastStatusAt)return false;
  trafficLastStatusAt=at;
 }
 return true;
}
function renderTraffic(s){
 const t=s.traffic_trial||{}, cfg=t.settings||{}, own=t.owner_client===client;
 const wasActive=trafficActive;
 trafficActive=!!t.active;trafficOwned=own;trafficToken=t.run_token;trafficSettingsRevision=t.settings_revision;
 el('traffic-test-state').textContent=t.settings_error||t.reason||'红绿灯程序未加载';
 el('traffic-prepare').disabled=stale||trafficSaveBusy||!t.can_prepare;
 const startDisabled=stale||trafficSaveBusy||trafficStartBusy||!own||!(t.can_request_start??t.can_start);
 if(el('traffic-start').disabled!==startDisabled)el('traffic-start').disabled=startDisabled;
 const startLabel=trafficStartBusy?'正在开始…':t.phase==='complete'?'开始下一轮':'开始';
 if(el('traffic-start').textContent!==startLabel)el('traffic-start').textContent=startLabel;
 const startReason=stale?'页面已过期，请刷新后重新准备':trafficSaveBusy?'正在保存参数':
  t.active&&!own?'这一轮由另一页面准备；请使用原页面，或先停止本轮':t.start_blocked_reason||'';
 el('traffic-start').title=startReason;
 el('traffic-start-hint').textContent=trafficStartError||startReason||(t.camera_ready?'准备完成：无需看见灯具，打开电调并确认静止后点开始。':'等待准备');
 for(const id of ['traffic-forward','traffic-approach','traffic-red-percent','traffic-save'])el(id).disabled=stale||trafficSaveBusy||!!t.active;
 const forward=el('traffic-forward');
 if(t.forward_limits_us){forward.min=t.forward_limits_us[0];forward.max=t.forward_limits_us[1];}
 if(!trafficDirty&&!trafficSaveBusy&&cfg.pwm){
  el('traffic-forward').value=cfg.pwm.search_us;el('traffic-approach').value=cfg.max_approach_s;
  el('traffic-red-percent').value=cfg.red_confirm_percent;
  el('traffic-settings-state').textContent='已保存 · '+cfg.pwm.search_us+' μs · 两路各统计 2 秒，红／绿共用 '+cfg.red_confirm_percent+'% · 红灯优先。';
 }
 el('traffic-forward-down').disabled=forward.disabled||Number(forward.value)<=Number(forward.min);
 el('traffic-forward-up').disabled=forward.disabled||Number(forward.value)>=Number(forward.max);
 const obs=t.observation, raw=obs?.signal, p=t.intent;
 const fresh=t.active&&t.frame_age_s!=null&&t.frame_age_s>=0&&t.frame_age_s<=.25;
 const color=fresh?(p?.signal_votes?.state||raw?.state):'unknown';
 el('traffic-color').textContent=({red:p?.red_seen_this_round?'红灯已确认 · 停车':'红灯候选 · 待比例确认',yellow:'黄灯 · 停车',green:'绿灯',off:t.can_start?'灯具未亮 · 可开始搜索':p?.phase==='approach'?'未亮 · 正在接近':'熄灯 · 等待',unknown:t.can_start?'未见灯具 · 可开始搜索':p?.phase==='approach'?'未见灯具 · 搜索中':'未确认灯色'})[color]||'等待画面';
 el('traffic-color').dataset.signal=color;
 el('traffic-vision').textContent=fresh?Object.entries(obs.camera_observations||{}).map(([c,v])=>(c==='primary'?'第一路':'第二路 H65')+' #'+v.frame_id+' '+v.signal.state).join(' · '):'准备后显示双路识别；旧画面不作为依据';
 el('traffic-intent').textContent=t.active&&p?('阶段 '+p.phase+' · 输出 '+p.esc_us+' μs'+(p.red_seen_this_round?' · 已见本轮红灯':'')+(p.stable?.green_confirmed?' · 绿灯占比已达标':'')):'';
 el('traffic-intent').hidden=!t.active||!p;
 const votes=p?.signal_votes;
 el('traffic-red-vote').textContent=t.active&&votes?Object.entries(votes.cameras).map(([c,v])=>(c==='primary'?'第一路':'第二路')+'：'+(!v.valid?'画面不可用，不参与计票':('红 '+v.red_frames+'/'+v.total_frames+' = '+v.red_percent.toFixed(1)+'% · 绿 '+v.green_frames+'/'+v.total_frames+' = '+v.green_percent.toFixed(1)+'%'+(!v.ready?' · 收集窗口 '+v.collected_s.toFixed(1)+'/2 秒':'')))).join(String.fromCharCode(10))+String.fromCharCode(10)+'共用阈值 '+votes.threshold_percent+'%'+(votes.conflict?' · 红绿同时达标，红灯优先':''):'开始后显示两路各自的红／绿帧占比；绿灯也使用相同2秒窗口及比例。';
 el('traffic-speech-state').textContent=({sending:'正在发送播报',command_sent:'已发送：红绿灯结束，开始前行',failed:'语音发送失败；车辆仍保持停车',cancelled:'播报已结束'})[t.speech?.state]||'';
 el('traffic-speech-state').hidden=!el('traffic-speech-state').textContent;
 if(t.active&&!wasActive)el('traffic-preview-section').open=true;
 if(!fresh){el('traffic-preview').hidden=true;el('traffic-preview-placeholder').hidden=false;el('traffic-preview-placeholder').textContent=t.active?'画面暂未更新，等待新帧':'准备后显示双路识别：左第一路，右第二路 H65';}
 if(!t.active)return false;
 lastStatus=s;held.clear();heartbeatDue=false;needsRelease=true;allowed=false;
 turnAvailable=false;motorAvailable=false;reverseAvailable=false;gamepadControl?.requireNeutral();
 el('drive-title').textContent='红绿灯场地测试';el('drive-mode').textContent=t.reason;
 el('drive-action').textContent=p?.action==='brake'?'主动制动':p?.action==='search'?'前进':'中位';
 el('drive-active-speed').textContent=p?p.esc_us+' μs':'准备中';
 el('drive-time').textContent=own?'本页持有红绿灯控制权':'另一页面正在测试';
 el('drive-latency').textContent=latency==null?'—':latency+' ms';
 for(const id of ['drive-enable','drive-reset','drive-next','drive-speed','test-mode','gamepad-arm','drive-focus']){if(el(id))el(id).disabled=true;}
 document.querySelectorAll('[data-drive],[data-speed]').forEach(b=>b.disabled=true);
 feedback(t.reason,t.phase==='fault'?'error':'sending');return true;
}
async function trafficSend(action,body={}){
 return send(action,{run_token:trafficToken,trial_sequence:++trafficSequence,...body});
}
for(const id of ['traffic-forward','traffic-approach','traffic-red-percent'])el(id).addEventListener('input',()=>{
 trafficDirty=true;el('traffic-settings-state').textContent='尚未保存；准备前会自动保存。';
});
function adjustTrafficForward(delta){
 const input=el('traffic-forward');if(input.disabled)return;
 const value=Number(input.value);
 input.value=Math.min(Number(input.max),Math.max(Number(input.min),(Number.isFinite(value)?value:1600)+delta));
 input.dispatchEvent(new Event('input',{bubbles:true}));
 if(lastStatus)render(lastStatus);
}
el('traffic-forward-down').addEventListener('click',()=>adjustTrafficForward(-5));
el('traffic-forward-up').addEventListener('click',()=>adjustTrafficForward(5));
async function saveTrafficSettings(force=false){
 if(!trafficDirty&&!force)return true;
 for(const id of ['traffic-forward','traffic-approach','traffic-red-percent']){if(!el(id).checkValidity()){el(id).reportValidity();return false;}}
 trafficSaveBusy=true;if(lastStatus)render(lastStatus);
 try{
  const s=await send('traffic_settings',{settings_revision:trafficSettingsRevision,forward_us:Number(el('traffic-forward').value),max_approach_s:Number(el('traffic-approach').value),red_confirm_percent:Number(el('traffic-red-percent').value)});
  trafficDirty=false;render(s);return true;
 }catch(e){el('traffic-settings-state').textContent=e.message;return false;}
 finally{trafficSaveBusy=false;if(lastStatus)render(lastStatus);}
}
el('traffic-save').addEventListener('click',()=>saveTrafficSettings(true));
el('traffic-prepare').addEventListener('click',async()=>{
 held.clear();needsRelease=true;gamepadControl?.requireNeutral();
 try{
  if(!await saveTrafficSettings())return;
  requestError='';localHint='';trafficStartError='';trafficSequence=0;
  render(await send('traffic_prepare',{esc_off:true,fb_mode_confirmed:true}));
 }catch(e){feedback(e.message,'error');}
});
el('traffic-start').addEventListener('click',async()=>{
 if(trafficStartBusy)return;
 trafficStartBusy=true;trafficStartError='';if(lastStatus)render(lastStatus);
 try{requestError='';localHint='';render(await trafficSend('traffic_start',{fb_mode_confirmed:true,esc_on_static:true,field_ready:true}));}
 catch(e){trafficStartError=e.message;feedback(e.message,'error');}
 finally{trafficStartBusy=false;if(lastStatus)render(lastStatus);}
});
el('traffic-cancel').addEventListener('click',()=>stop('stop',false,'stop_button'));
setInterval(async()=>{
 if(!trafficActive||!trafficOwned||stale||document.hidden||trafficPulseBusy>=2)return;
 trafficPulseBusy++;
 try{render(await trafficSend('traffic_keepalive'));}
 catch(e){if(e.message!=='old or invalid traffic sequence'){feedback(e.message,'error');stop('stop',false,'connection_recovery');}}
 finally{trafficPulseBusy--;}
},100);
let trafficImageBusy=false,trafficImageURL=null;
setInterval(async()=>{
 if(!trafficActive||stale||document.hidden||trafficImageBusy)return;
 trafficImageBusy=true;const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1500);
 try{
  const r=await fetch('/api/traffic/frame.jpg',{cache:'no-store',signal:controller.signal});if(!r.ok)throw new Error('no fresh frame');
  const url=URL.createObjectURL(await r.blob()),old=trafficImageURL;trafficImageURL=url;
  el('traffic-preview').src=url;el('traffic-preview').hidden=false;el('traffic-preview-placeholder').hidden=true;if(old)URL.revokeObjectURL(old);
 }catch(e){el('traffic-preview').hidden=true;el('traffic-preview-placeholder').hidden=false;}
 finally{clearTimeout(timer);trafficImageBusy=false;}
},400);
'''


def extend_controls(controls):
    script = controls.SCRIPT
    sentinel = 'let gamepadControl=null,pwmControl=null;'
    brake_render = 'if(renderBrake(s))return;'
    heartbeat = 'async function heartbeat(){'
    if any(script.count(x) != 1 for x in (sentinel, brake_render, heartbeat)) or script.rfind('})();') < 0:
        raise ValueError('红绿灯扩展需要现有 F/B 控制台，停止加载不匹配的遥控版本')
    script = script.replace(sentinel, sentinel+'\nlet trafficActive=false,trafficOwned=false,trafficToken=null,trafficSequence=0,trafficPulseBusy=0,trafficSettingsRevision=null,trafficSaveBusy=false,trafficDirty=false,trafficStartBusy=false,trafficStartError="",trafficLastStatusAt=-Infinity;')
    script = script.replace(brake_render, 'if(!acceptTrafficSnapshot(s))return;\n const brakeBusy=renderBrake(s),trafficBusy=renderTraffic(s); if(brakeBusy||trafficBusy)return;')
    script = script.replace(heartbeat, heartbeat+'\n if(trafficActive){heartbeatDue=false;return;}')
    end = script.rfind('})();')
    controls.SCRIPT = script[:end]+SCRIPT+script[end:]
    from .brake_controls import HTML as brake_html
    if not controls.HTML.endswith(brake_html):
        raise ValueError('需要斑马线面板作为最后一个原有面板，才能组合任务布局')
    controls.HTML = controls.HTML[:-len(brake_html)]+'<div id="autonomy-tests">'+brake_html+HTML+'</div>'
    controls.STYLE = getattr(controls, 'STYLE', '')+'''<style>
#autonomy-tests{grid-column:1;display:grid;grid-template-columns:minmax(0,1fr);gap:18px;min-width:0;align-items:start}
#traffic-test{background:white;color:var(--text-primary,#24212b);border-radius:18px;padding:20px;display:grid;gap:12px;min-width:0}
#traffic-test p,#traffic-test h3{margin:0;line-height:1.6}#traffic-test p{color:var(--text-secondary,#75677e)}
#traffic-test label{display:flex;gap:8px;align-items:center;line-height:1.6}
#traffic-test button,#traffic-test input{min-height:42px}
#traffic-test input{width:100%;min-width:0;box-sizing:border-box;border:1px solid #d8d8e4;border-radius:10px;padding:8px;background:#fff;color:#24212b}
#traffic-test .traffic-threshold{display:grid;grid-template-columns:1fr 85px auto;gap:8px;align-items:center}
#traffic-test .traffic-throttle{display:grid;grid-template-columns:1fr 210px;gap:8px;align-items:center}
#traffic-test .traffic-throttle-steps{display:grid;grid-template-columns:50px 1fr 50px;gap:6px}
#traffic-test .traffic-actions{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
#traffic-test button:enabled{background:#8e35e8;color:white;border-color:#8e35e8}
#traffic-test button:disabled{opacity:.45;cursor:not-allowed}#traffic-test button{transition:none}
#traffic-test #traffic-cancel{background:#fff1f2;color:#b42335;border-color:#fecdd3}
#traffic-test #traffic-start-hint{min-height:3.2em}
#traffic-test .traffic-parameters{display:grid;grid-template-columns:1fr 110px;gap:10px;align-items:center;margin:12px 0}
.traffic-observation{display:flex;gap:12px;align-items:center;flex-wrap:wrap;font-size:12px}
#traffic-color{padding:5px 9px;border:1px solid #ddd5e6;border-radius:8px;color:#75677e;background:#faf8fc}
#traffic-color[data-signal=red]{color:#b42335;border-color:#fecdd3;background:#fff1f2}
#traffic-color[data-signal=green]{color:#17603c;border-color:#b6e3cb;background:#effbf4}
#traffic-color[data-signal=yellow]{color:#825b08;border-color:#ead69b;background:#fffbeb}
#traffic-test .traffic-viewer{position:relative;aspect-ratio:8/3;width:100%;max-width:960px;border-radius:10px;overflow:hidden;background:#f7f4fa;margin-top:12px;display:grid;place-items:center}
#traffic-red-vote{white-space:pre-line;font-variant-numeric:tabular-nums}
#traffic-preview{position:absolute;inset:0;width:100%;height:100%;object-fit:contain}
#traffic-preview-placeholder{padding:16px;color:#75677e;text-align:center}
@media(max-width:390px){#traffic-test .traffic-threshold{grid-template-columns:1fr 70px}#traffic-save{grid-column:1 / -1}#traffic-test .traffic-throttle{grid-template-columns:1fr}#traffic-test .traffic-throttle-steps{max-width:240px}}
</style>'''


def add_preview_endpoint(server, drive):
    """Serve the exact annotated decision image; never opens a camera or motor."""
    from urllib.parse import urlsplit
    original = server.RequestHandlerClass

    class Handler(original):
        def do_GET(self):
            if urlsplit(self.path).path != '/api/traffic/frame.jpg':
                return super().do_GET()
            with drive.lock:
                sample, jpeg = drive.observation or {}, drive.preview_jpeg
                fresh = drive.busy and 0 <= drive.clock()-sample.get('captured_s', -1e9) <= drive.settings['max_frame_age_s']
            try:
                return self.send_body(jpeg if fresh and jpeg else b'No fresh traffic frame',
                                      'image/jpeg' if fresh and jpeg else 'text/plain', 200 if fresh and jpeg else 503)
            except (OSError, ConnectionError):
                self.close_connection = True
    server.RequestHandlerClass = Handler
    return server
