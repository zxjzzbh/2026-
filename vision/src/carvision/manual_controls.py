"""Browser controls for finite raised-wheel trials."""

from .gamepad_controls import SCRIPT as GAMEPAD_SCRIPT

HTML = '''<section id="drive-panel" class="drive-console" tabindex="-1">
<div class="console-heading"><h2 id="drive-title">架空驾驶短测</h2><span id="drive-link" class="link-pill">连接中</span></div>
<p id="drive-mode">正在连接控制服务……</p>
<p id="drive-safety" class="safety-note" hidden></p>
<p id="drive-feedback" role="status" aria-live="polite" style="padding:14px;border:2px solid #94a3b8;border-radius:8px;font-size:20px;font-weight:bold">正在核对控制服务，请稍候……</p>
<div class="speed-control"><label for="drive-speed">速度指令 <output id="drive-speed-value">20%</output></label>
<input id="drive-speed" type="range" min="0" max="100" step="1" value="20" aria-label="速度指令比例">
<div class="speed-presets"><button data-speed="20">慢速 20%</button><button data-speed="40">中速 40%</button><button data-speed="60">较快 60%</button></div>
<p id="drive-speed-note">油门指令比例；实际轮速受电池、负载和电调影响。</p></div>
<div class="drive-readouts"><div><span>当前动作</span><strong id="drive-action">停车 · 回正</strong></div><div><span>当前指令</span><strong id="drive-active-speed">0%</strong></div><div><span>控制延迟</span><strong id="drive-latency">—</strong></div></div>
<p id="drive-time">正在读取本轮剩余时间……</p>
<button id="drive-refresh" hidden>刷新控制页面</button>
<button id="drive-reconnect" hidden>重新连接控制</button>
<button id="drive-next" hidden>开始下一轮</button>
<p id="drive-limits">前进每次最多 0.8 秒；倒车请按住约 3 秒，包含刹车和回零等待。一直按住不会重复试转，需全部松开后再按。</p>
<label><input id="bench-ready" type="checkbox"> <span id="bench-confirm-text">核对当前测试模式后启用</span></label>
<nav class="drive-start"><button id="drive-enable">启用短测</button><button id="drive-focus">点击这里，再用键盘</button><button id="drive-reset" hidden>解除急停</button></nav>
<button id="drive-emergency" class="emergency-control">急停 · 空格</button>
<div id="direction-pad" class="direction-pad">
<span></span><button data-drive="forward">前进 ↑ / W</button><span></span>
<button data-drive="left">左转 ← / A</button><button id="drive-stop">停车</button><button data-drive="right">右转 → / D</button>
<span></span><button data-drive="reverse">倒车 ↓ / S</button><span></span></div>
<p id="drive-message">W/S 前进倒车 · A/D 转向 · 空格急停。支持方向键和按住屏幕按钮。</p>
<section class="gamepad-panel"><h3>USB 游戏手柄</h3>
<p id="gamepad-info" role="status">仅检测手柄；尚未接管方向。</p>
<label><input id="gamepad-arm" type="checkbox"> 使用手柄方向输入</label>
<p>标准映射：按住 RB 允许操作；左摇杆左右转向，RT 前进、LT 倒车，B 急停。松开 RB 停车。沿用本轮固定档位和短测时限，扳机暂不调速。</p></section>
</section>'''

STYLE = '''<style>
main{max-width:1320px;padding:24px}h1{font-size:30px;letter-spacing:.02em}
body{background:#0b1120}button{transition:background .12s,border-color .12s}button:disabled{opacity:.4;cursor:not-allowed}button:focus-visible,input:focus-visible{outline:3px solid #60a5fa;outline-offset:3px}
.driving-layout{display:grid;grid-template-columns:minmax(0,1fr) 370px;gap:20px;align-items:start;margin-top:24px}
.camera-zone,.drive-console{background:#111c30;border:1px solid #27364f;border-radius:16px;padding:20px;min-width:0}
.camera-zone .viewer{margin-top:14px;min-height:200px;aspect-ratio:4/3;display:grid;place-items:center;border:1px solid #27364f}
.camera-zone .viewer img{width:100%;height:100%;object-fit:contain}.camera-zone #state{font-size:14px}
.camera-zone nav{margin:16px 0}.aux-camera{margin-top:18px;max-width:330px}.aux-camera .viewer{min-height:120px}.aux-camera h3{font-size:16px;margin:0}.aux-camera p{font-size:13px}
.console-heading{display:flex;align-items:center;justify-content:space-between;gap:12px}.console-heading h2{font-size:22px;margin:0}.link-pill{background:#1e3a35;border-radius:20px;padding:6px 10px;font-size:12px;color:#86efac;white-space:nowrap}
.drive-console p{font-size:13px}.safety-note{background:#47241e;border:1px solid #b4533d;border-radius:8px;padding:12px;color:#fed7aa}
.speed-control{border-top:1px solid #27364f;border-bottom:1px solid #27364f;padding:18px 0}.speed-control label{display:flex;justify-content:space-between;font-weight:600}.speed-control output{font-size:26px;color:#93c5fd;font-variant-numeric:tabular-nums}
#drive-speed{width:100%;accent-color:#60a5fa;margin:16px 0 12px;height:26px;cursor:pointer}.speed-presets{display:flex;gap:8px}.speed-presets button{padding:8px;font-size:12px;flex:1}.speed-control p{margin-bottom:0}
.drive-readouts{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:18px 0}.drive-readouts div:first-child{grid-column:1/-1}.drive-readouts span{display:block;color:#94a3b8;font-size:12px}.drive-readouts strong{display:block;font-size:18px;margin-top:4px;font-variant-numeric:tabular-nums}
.drive-start{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:14px 0}.drive-start button{padding:10px;font-size:13px}#drive-enable{background:#2563eb;border-color:#3b82f6}#drive-reset{grid-column:1/-1}
.emergency-control{width:100%;background:#b91c1c;border-color:#ef4444;font-size:19px;font-weight:700;padding:16px;margin-bottom:18px}.direction-pad{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.direction-pad button{min-height:58px;padding:8px;font-size:14px;user-select:none}.direction-pad button.active{background:#2563eb}
.vision-details{margin-top:24px;border-top:1px solid #27364f;padding-top:16px}.vision-details summary{cursor:pointer;color:#94a3b8;font-size:14px}
@media(max-width:900px){main{padding:16px}.driving-layout{grid-template-columns:1fr}.drive-console{order:-1}.drive-readouts{grid-template-columns:2fr 1fr 1fr}.drive-readouts div:first-child{grid-column:auto}.camera-zone .viewer{max-height:65vh}.aux-camera{max-width:100%}.aux-camera .viewer{max-height:40vh}}
</style>'''

SCRIPT = '''<script>
(()=>{
const key=__DRIVE_KEY__, pageSession=__DRIVE_SESSION__, el=id=>document.getElementById(id);
const client=Array.from(crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join('');
let held=new Map(), sequence=0, busy=false, heartbeatDue=false, allowed=false, turnAvailable=true, motorAvailable=true, reverseAvailable=true, urgentGeneration=0, needsRelease=true, stopPending=false;
let stale=false, enabling=false, lastStatus=null, requestError='', localHint='', latency=null, speedWanted=null, speedBusy=false;
let gamepadControl=null;
const directionNames={forward:'前进',reverse:'倒车',left:'左转',right:'右转'};
const keys={ArrowUp:'forward',w:'forward',W:'forward',ArrowDown:'reverse',s:'reverse',S:'reverse',ArrowLeft:'left',a:'left',A:'left',ArrowRight:'right',d:'right',D:'right'};
const keySource=e=>'key:'+(e.code||e.key.toLowerCase());
function feedback(message,tone='idle'){
 const colors={idle:'#94a3b8',ready:'#22c55e',sending:'#fbbf24',error:'#f87171'};
 el('drive-feedback').textContent=message;el('drive-feedback').style.borderColor=colors[tone];
}
function expiredPage(){
 stale=true;held.clear();allowed=false;urgentGeneration++;
 el('drive-mode').textContent='控制页面已过期，请刷新后核对当前模式。';
 el('drive-time').textContent='本页已禁止发送方向请求。';
 el('drive-refresh').hidden=false;el('drive-enable').disabled=true;
 document.querySelectorAll('[data-drive]').forEach(b=>b.disabled=true);
 feedback('控制服务已更新：请刷新页面，再重新启用短测。','error');
}
async function send(action,body={},keepalive=false){
 const started=performance.now(),controller=new AbortController(), timer=setTimeout(()=>controller.abort(),2000);
 try{
 const response=await fetch('/api/drive/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Drive-Key':key},
 body:JSON.stringify({client,trial_token:lastStatus?.trial_token,...body}),cache:'no-store',keepalive,signal:controller.signal});
 const result=await response.json();latency=Math.round(performance.now()-started);if(!response.ok){if(result.code==='control_page_rejected')expiredPage();throw new Error(result.error||'控制请求失败');}return result;
 }catch(e){if(e.name==='AbortError')throw new Error('2秒内未收到控制确认');throw e;}
 finally{clearTimeout(timer);}
}
function render(s){
 const previousStatus=lastStatus;
 if(previousStatus?.trial_token&&s.trial_token&&previousStatus.trial_token!==s.trial_token){
  held.clear();heartbeatDue=false;urgentGeneration++;needsRelease=true;stopPending=false;
  gamepadControl?.requireNeutral();requestError='';localHint='';el('bench-ready').checked=false;
 }
 lastStatus=s;
 if(s.control_session_id&&s.control_session_id!==pageSession){expiredPage();return;}
 if(stale)return;
 if(s.controls_paused){
  allowed=false;held.clear();turnAvailable=false;motorAvailable=false;reverseAvailable=false;
  el('drive-title').textContent='实车控制';el('drive-link').textContent='已连接';
  el('drive-mode').textContent='模拟已关闭 · 实车暂停';
  el('drive-safety').hidden=false;el('drive-safety').textContent=s.reason;
  el('drive-action').textContent='未输出控制信号';el('drive-active-speed').textContent='0%';
  el('drive-latency').textContent=latency==null?'—':latency+' ms';
  el('drive-speed').disabled=true;el('drive-speed').value=0;el('drive-speed-value').textContent='0%';
  el('drive-speed-note').textContent='模拟操作已关闭，当前无法启用车轮或转向输出。';
  el('drive-time').textContent='供电检查通过后，重新启动实车测试。';
  el('drive-limits').textContent='电调保持关闭，摄像头预览仍可使用。';
  el('bench-ready').checked=false;el('bench-ready').disabled=true;
  el('bench-confirm-text').textContent='当前实车测试尚未就绪';
  el('drive-enable').textContent='实车暂停';el('drive-reset').hidden=true;
  ['drive-enable','drive-focus','drive-emergency','drive-stop'].forEach(id=>el(id).disabled=true);
  document.querySelectorAll('[data-drive],[data-speed]').forEach(b=>{b.disabled=true;b.classList.remove('active');});
  feedback('模拟已关闭。'+s.reason,'error');return;
 }
 const finished=['expired','closed','fault'].includes(s.mode)||s.worker_available===false;
 const nextRound=s.can_start_next_round===true;
 el('drive-next').hidden=!nextRound;el('drive-next').disabled=enabling||!el('bench-ready').checked;
 if(finished){requestError='';localHint='';held.clear();needsRelease=true;}
 const wasAllowed=allowed;allowed=s.mode==='enabled'&&!requestError&&!finished;
 turnAvailable=s.steering_available!==false;
 motorAvailable=s.motor_available!==false;
 reverseAvailable=s.reverse_available!==false;
 const continuous=s.continuous_simulation===true;
 const ground=s.ground_short_trial===true, groundHeld=s.ground_held_trial===true, testSeconds=s.test_session_s??180;
 el('drive-title').textContent=groundHeld?'低速遥控测试':ground?'地面负载短测':continuous?'手动驾驶':'架空驾驶短测';
 el('drive-link').textContent='已连接';el('drive-link').style.color='#86efac';
 el('drive-latency').textContent=latency==null?'—':latency+' ms';
 el('drive-safety').hidden=!s.real_output_paused_reason;el('drive-safety').textContent='实车未启用：'+(s.real_output_paused_reason||'')+' 电调保持关闭，当前仅验证网页控制。';
 el('drive-speed').disabled=!s.speed_adjustable||stale||['closed','fault','expired'].includes(s.mode);
 document.querySelectorAll('[data-speed]').forEach(b=>b.disabled=el('drive-speed').disabled);
 if(!speedBusy&&speedWanted==null){el('drive-speed').value=s.speed_percent??100;el('drive-speed-value').textContent=el('drive-speed').value+'%';}
 el('drive-speed-note').textContent=s.speed_adjustable?'油门指令比例；实际转速尚未测量。先以低档位检查控制反馈。':'本轮实车架空短测使用已验证固定档位，速度调节暂不开放。';
 el('drive-action').textContent=s.mode==='emergency'?'已急停':(s.motor_direction==='forward'?'前进':s.motor_direction==='reverse'?'倒车':'停车')+' · '+(s.steering_direction==='left'?'左转':s.steering_direction==='right'?'右转':'回正');
 el('drive-active-speed').textContent=(s.active_speed_percent??0)+'%'+(!s.hardware_output?'（模拟）':'');
 el('bench-confirm-text').textContent=s.hardware_output?(motorAvailable?'车轮已架空，电调已打开且轮子静止，开关在手边':'车轮已架空，电调保持关闭，只测试前轮转向'):'我确认这是模拟模式，电调保持关闭';
 if(ground){el('bench-confirm-text').textContent='场地空旷，电调已打开且车轮静止，开关在手边';el('drive-active-speed').textContent=s.motor_pulse_us===1500?'零油门':'固定短测档';el('drive-speed-value').textContent='固定档位';el('drive-speed-note').textContent='使用已验证的固定油门档位，每段转动最长0.8秒。';}
 if(ground&&s.load_probe){el('drive-speed-value').textContent='起步提高一档';el('drive-speed-note').textContent='前进油门提高一档，等待本次负载验证；每段最长0.8秒，松键停车。';el('drive-active-speed').textContent=s.motor_pulse_us===1500?'零油门':'起步试验档';}
 if(s.raised_load_probe){el('drive-title').textContent='架空起步油门短测';el('drive-speed-value').textContent='前进提高一档';el('drive-speed-note').textContent='只测架空前进与停车，每段最多0.8秒；倒车和舵机信号关闭。';el('drive-active-speed').textContent=s.motor_pulse_us===1500?'零油门':'提高一档';}
 if(groundHeld){el('bench-confirm-text').textContent='车已落地，路线空旷，有人跟车，电调打开后车轮静止';el('drive-speed-value').textContent='固定低档位';el('drive-speed-note').textContent='使用已验证的低档位，松开行驶键停车。';el('drive-active-speed').textContent=s.motor_pulse_us===1500?'零油门':'固定低档位';}
 el('drive-limits').textContent=s.parking_protection_trial?'只测试 W 前进和停车保护，每次最多 0.8 秒。轮子开始转动后立即触发指定保护；先不测试倒车和转向。':motorAvailable?'前进每次最多 0.8 秒；倒车请按住约 3 秒，包含刹车和回零等待。一直按住不会重复试转，需全部松开后再按。':'只测试 A/D 或左右键，每次最多 2 秒；松键后回正并关闭舵机信号。前进、倒车和云台信号保持关闭。';
 if(continuous)el('drive-limits').textContent='按住 W/S 或上下键持续发送行驶请求，A/D 可同时转向。松开行驶键停车，空格急停；切换窗口后停止发送。';
 if(ground)el('drive-limits').textContent='有限地面负载短测：W前进最多0.8秒，S请按住约3秒完成倒车准备。每组之间松开全部方向键；松键回零、空格急停。';
 if(groundHeld)el('drive-limits').textContent='按住 W 前进，A/D 转向；松开 W 停车，松开转向键回正。空格急停，失联停车；单次按住最多 '+s.motion_hold_max_s+' 秒，本轮 '+testSeconds+' 秒。';
 el('drive-mode').textContent=(s.hardware_output?(s.parking_protection_trial?'实车停车保护短测（只测前进，舵机信号关闭）':s.combined_trial?'实车联动短测（云台信号关闭）':!motorAvailable?'实车转向短测（驱动和云台信号关闭）':turnAvailable?'实车短测':'实车车轮短测（转向和云台信号关闭）'):'模拟模式（按钮不会驱动车轮）')+' · '+s.reason+
 (s.mode==='settling'?'（'+Math.ceil(s.ready_in_s)+'秒）':'');
 if(ground)el('drive-mode').textContent='实车地面负载短测 · '+s.reason+(s.mode==='settling'?'（'+Math.ceil(s.ready_in_s)+'秒）':'');
 if(groundHeld)el('drive-mode').textContent='实车低速遥控 · '+s.reason+(s.mode==='settling'?'（'+Math.ceil(s.ready_in_s)+'秒）':'');
 if(s.raised_load_probe){el('drive-mode').textContent='实车架空前进短测 · '+s.reason+(s.mode==='settling'?'（'+Math.ceil(s.ready_in_s)+'秒）':'');el('drive-limits').textContent='只测 W 前进和停车：每次最多0.8秒，松键或空格停车；先不测试倒车和转向。';}
 el('drive-enable').disabled=enabling||s.mode!=='disabled';el('drive-enable').textContent=enabling||s.mode==='starting'?'正在启用……':groundHeld?'启用低速遥控':continuous?'启用模拟驾驶':'启用短测';el('drive-reset').hidden=s.mode!=='emergency';
 el('drive-time').textContent=s.session_remaining_s==null?(s.hardware_output?'测试剩余时间暂不可用，请关闭电调。':'当前为模拟测试'):(s.session_phase==='preparing'?'准备剩余 '+Math.ceil(s.session_remaining_s)+' 秒；启用后另有 3 分钟测试时间':(s.combined_trial||s.parking_protection_trial)&&s.mode==='settling'?'正在等待就绪；就绪后有 3 分钟测试时间':('本轮剩余 '+Math.ceil(s.session_remaining_s)+' 秒；结束后需重新准备'));
 if((ground||groundHeld)&&s.session_remaining_s!=null)el('drive-time').textContent=s.session_phase==='preparing'?'准备剩余 '+Math.ceil(s.session_remaining_s)+' 秒；启用就绪后有 '+testSeconds+' 秒测试时间':s.mode==='settling'?'正在等待就绪；就绪后有 '+testSeconds+' 秒测试时间':'本轮剩余 '+Math.ceil(s.session_remaining_s)+' 秒';
 if(groundHeld&&s.pending_hardware_start)el('drive-time').textContent='准备就绪，等待你点击启用；就绪后有 '+Math.round(testSeconds/60)+' 分钟测试时间。';
 if(groundHeld&&s.mode==='starting')el('drive-time').textContent='正在准备控制信号，请保持车辆静止；行驶倒计时尚未开始。';
 if(s.repeatable_trials&&s.pending_hardware_start)el('drive-time').textContent='等待你点击启用；每轮 '+testSeconds+' 秒，正常结束后可开始下一轮。';
 if(s.repeatable_trials&&s.mode==='starting')el('drive-time').textContent='正在准备本轮，请保持车辆静止；倒计时尚未开始。';
 if(s.raised_load_probe&&s.session_remaining_s!=null)el('drive-time').textContent=s.session_phase==='preparing'?'准备剩余 '+Math.ceil(s.session_remaining_s)+' 秒；启用就绪后有 '+testSeconds+' 秒测试时间':s.mode==='settling'?'正在等待就绪；就绪后有 '+testSeconds+' 秒测试时间':'本轮剩余 '+Math.ceil(s.session_remaining_s)+' 秒';
 if(finished){
  el('drive-link').textContent=s.mode==='expired'?'本轮已结束':'控制已停止';el('drive-link').style.color='#fbbf24';
  el('drive-time').textContent=s.mode==='expired'&&s.session_phase==='preparing'?'准备窗口已结束，尚未启用本轮测试。请关闭电调，重新准备。':'本轮控制已结束，请关闭电调，重新准备。';
  el('drive-active-speed').textContent='输出已结束';
  if(!nextRound||previousStatus?.mode!=='expired')el('bench-ready').checked=false;
 }
 if(nextRound){el('drive-time').textContent='第 '+s.trial_number+' 轮已结束，确认停稳后点击“开始下一轮”。';el('bench-confirm-text').textContent='车已停稳，场地空旷，电调开关在手边';}
 el('bench-ready').disabled=finished&&!nextRound;el('drive-focus').disabled=finished;
 el('drive-next').disabled=enabling||!el('bench-ready').checked;
 document.querySelectorAll('[data-drive]').forEach(b=>b.disabled=!allowed||!turnAvailable&&['left','right'].includes(b.dataset.drive)||!motorAvailable&&['forward','reverse'].includes(b.dataset.drive)||!reverseAvailable&&b.dataset.drive==='reverse');
 document.querySelectorAll('[data-drive]').forEach(b=>b.classList.toggle('active',Array.from(held.values()).includes(b.dataset.drive)));
 if(wasAllowed&&!allowed)held.clear();
 if(requestError){feedback(requestError,'error');return;}
 if(enabling){feedback('启用请求已发出，等待树莓派确认……','sending');return;}
 if(s.mode==='starting'){feedback('正在准备遥控，请保持车辆静止；完成后再等待就绪倒计时。','sending');return;}
 if(s.mode==='settling'){feedback('启用成功，'+(!s.hardware_output?'模拟模式，电调保持关闭':motorAvailable?'保持零油门':'电调保持关闭')+'：'+Math.ceil(s.ready_in_s)+' 秒后可以按键。','sending');return;}
 if(nextRound){feedback('本轮已结束。确认车已停稳，勾选就绪后点击“开始下一轮”。','ready');return;}
 if(s.mode==='expired'||s.mode==='closed'||s.mode==='fault'){feedback(s.reason+'；本轮不能继续按键，请关闭电调。','error');return;}
 if(s.mode==='emergency'){feedback('已急停：'+s.reason+'。需要解除急停并重新启用。','error');return;}
 if(localHint){feedback(localHint,'sending');return;}
 if(stopPending){feedback('已收到松键／停车操作，正在等待停车确认……','sending');return;}
 if(s.motor_stage==='brake'){feedback((s.hardware_output?'倒车准备：正在输出刹车信号':'模拟倒车准备：刹车阶段')+'；请继续按住 S，松键会取消。','sending');return;}
 if(s.motor_stage==='neutral_gap'){feedback('倒车准备：回零等待 '+Math.max(0,s.reverse_wait_remaining_s).toFixed(1)+' 秒；请继续按住 S。','sending');return;}
 if(s.motor_pulse_us!==1500&&s.motor_pulse_us!=null){
  const centerUs=s.steering_center_us??1650;
  const turn=s.combined_trial&&s.steering_signal_active&&s.steering_pulse_us!==centerUs?'＋'+(s.steering_pulse_us>centerUs?'左转':'右转'):'';
  feedback(s.hardware_output?('树莓派已输出'+(s.motor_pulse_us>1500?'前进':'倒车')+turn+'信号；松键停车、回正。'):('模拟驾驶：'+(s.motor_direction==='forward'?'前进':'倒车')+' '+s.speed_percent+'%'+(s.steering_direction!=='center'?' ＋ '+directionNames[s.steering_direction]:'')+'；松开行驶键停车。'),'ready');return;
 }
 if((!motorAvailable||s.combined_trial)&&s.steering_signal_active){
  const centerUs=s.steering_center_us??1650;
  feedback(held.size&&!s.release_required?'树莓派已输出'+(s.steering_pulse_us===centerUs?'回正':s.steering_pulse_us>centerUs?'左转':'右转')+'信号；松键回正。':'已收到回正请求，前轮正在回正……','ready');return;
 }
 if(continuous&&s.steering_direction!=='center'&&!s.release_required){feedback('模拟驾驶：停车 ＋ '+directionNames[s.steering_direction]+'；车轮不会转动。','ready');return;}
 if(held.size&&s.release_required){feedback(continuous?'已停止行驶请求，请松开按键后再按。':'本次短测已回零，请松开按键后再测。','ready');return;}
 if(held.size){feedback('按键已收到，等待树莓派响应／回零准备……','sending');return;}
 if(allowed&&s.reverse_cancelled){feedback('倒车准备已因松键取消；已回零，请观察车轮是否停下。再测时按住 S 约 3 秒。','sending');return;}
 if(allowed){feedback(continuous?(['松键或停车请求','按键已松开','速度指令为零'].includes(s.reason)?'模拟停车已确认：行驶请求已回零。':'模拟驾驶已就绪：按住 W/S 行驶，A/D 转向；当前没有实车输出。'):motorAvailable?(['松键或停车请求','按键已松开'].includes(s.reason)?'已收到回零确认；请观察车轮是否停下。':'已就绪：可以按住 W 前进；当前零油门。'):['松键或停车请求','按键已松开','短测结束，请松开全部方向键后再按'].includes(s.reason)?'已输出回正信号并关闭舵机信号；请观察前轮是否回正。':'已就绪：按住 A 左转、D 右转；松键回正。','ready');return;}
 feedback('尚未启用：先勾选就绪确认，再点击“'+(groundHeld?'启用低速遥控':continuous?'启用模拟驾驶':'启用短测')+'”。');
}
function stop(action='stop',keepalive=false,stopSource='unknown'){
 gamepadControl?.requireNeutral();
 held.clear();heartbeatDue=false;localHint='';urgentGeneration++;needsRelease=true;
 if(stale)return;
 if(lastStatus&&(['expired','closed','fault'].includes(lastStatus.mode)||lastStatus.worker_available===false)){render(lastStatus);return;}
 stopPending=true;const generation=urgentGeneration;
 feedback('已收到'+(action==='emergency'?'急停':'松键／停车')+'操作，等待树莓派确认……','sending');
 document.querySelectorAll('[data-drive]').forEach(b=>b.classList.remove('active'));
 send(action,{stop_source:stopSource},keepalive).then(s=>{if(generation===urgentGeneration){requestError='';lastStatus=s;}}).catch(()=>{allowed=false;requestError='停车确认未收到：请关闭电调，检查页面连接。';feedback(requestError,'error');})
 .finally(()=>{if(generation===urgentGeneration){stopPending=false;if(lastStatus)render(lastStatus);}});
}
async function heartbeat(){
 if(stopPending||!allowed||held.size===0||stale){heartbeatDue=false;return;}
 // Keep one request in flight and one due flag, rather than lose a timer
 // tick while waiting for a cellular response. Never queue old directions.
 if(busy){heartbeatDue=true;return;}
 const values=new Set(held.values());let motor=values.has('forward')?'forward':values.has('reverse')?'reverse':'stop';
 let steering=values.has('left')?'left':values.has('right')?'right':'center';
 if(values.has('forward')&&values.has('reverse')||values.has('left')&&values.has('right')){stop('stop',false,'conflicting_keys');return;}
 busy=true;heartbeatDue=false;const generation=urgentGeneration;
 try{
  if(needsRelease){const s=await send('command',{sequence:++sequence,motor:'stop',steering:'center'});
   if(generation!==urgentGeneration||held.size===0)return;render(s);needsRelease=false;}
  const sources=Array.from(held.keys());
  const s=await send('command',{sequence:++sequence,motor,steering,input_source:sources.some(x=>x.startsWith('pad:'))?'gamepad':sources.some(x=>x.startsWith('key:'))?'keyboard':'pointer'});if(generation===urgentGeneration){requestError='';localHint='';render(s);}
 }
 catch(e){if(generation===urgentGeneration&&!stale){held.clear();heartbeatDue=false;allowed=false;requestError=e.message+'；控制请求失败，请关闭电调后检查。';feedback(requestError,'error');}}
 finally{busy=false;if(heartbeatDue&&!stopPending&&allowed&&held.size&&!stale){heartbeatDue=false;heartbeat();}}
}
function press(source,direction){
 if(gamepadControl?.armed){localHint='手柄方向模式已选中；键盘空格仍可急停。使用键盘或屏幕方向前，请取消手柄勾选。';if(lastStatus)render(lastStatus);return;}
 if(stale){expiredPage();return;}
 if(!allowed){localHint='已收到'+directionNames[direction]+'按键，但尚未就绪；请先启用并等待倒计时。';if(lastStatus)render(lastStatus);return;}
 if(!turnAvailable&&['left','right'].includes(direction)){localHint='本轮只测试车轮，左右转向信号关闭。';render(lastStatus);return;}
 if(!motorAvailable&&['forward','reverse'].includes(direction)){localHint='本轮只测试转向，前进和倒车信号关闭；请按 A 或 D。';render(lastStatus);return;}
 if(!reverseAvailable&&direction==='reverse'){localHint='本轮只测试前进停车保护，倒车信号关闭；请按 W。';render(lastStatus);return;}
 if(held.has(source))return;localHint='';requestError='';held.set(source,direction);feedback('已收到'+directionNames[direction]+'按键，正在发出控制请求……','sending');heartbeat();
}
function release(source){
 if(!held.has(source))return;held.delete(source);
 if((lastStatus?.continuous_simulation||lastStatus?.raised_held_trial||lastStatus?.ground_held_trial)&&held.size){urgentGeneration++;localHint='';heartbeat();return;}
 stop('stop',false,source.startsWith('key:')?'key_release':'pointer_release');
}
async function changeSpeed(value){
 if(stale||!lastStatus?.speed_adjustable)return;
 speedWanted=Number(value);el('drive-speed').value=speedWanted;el('drive-speed-value').textContent=speedWanted+'%';
 if(speedWanted===0)stop('stop',false,'speed_zero');
 if(speedBusy)return;speedBusy=true;
 try{
  while(speedWanted!=null){const value=speedWanted;speedWanted=null;lastStatus=await send('speed',{speed_percent:value});}
 }catch(e){speedWanted=null;requestError='速度设置未确认：'+e.message;stop('stop',false,'speed_failure');}
 finally{speedBusy=false;if(lastStatus)render(lastStatus);}
}
el('drive-speed').oninput=e=>changeSpeed(e.target.value);
el('drive-speed').onpointerup=()=>el('drive-panel').focus({preventScroll:true});
document.querySelectorAll('[data-speed]').forEach(b=>b.onclick=()=>{changeSpeed(b.dataset.speed);el('drive-panel').focus({preventScroll:true});});
document.querySelectorAll('[data-drive]').forEach(b=>{
 b.style.touchAction='none';b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);b.classList.add('active');press('pointer:'+e.pointerId,b.dataset.drive);};
 b.onpointerup=e=>release('pointer:'+e.pointerId);b.onpointercancel=e=>release('pointer:'+e.pointerId);
 b.onlostpointercapture=e=>release('pointer:'+e.pointerId);
});
window.addEventListener('keydown',e=>{
 if(e.code==='Space'||e.key===' '||e.key==='Spacebar'){e.preventDefault();if(!e.repeat)stop('emergency',false,'keyboard_space');return;}
 if(keys[e.key]){
  if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName)){localHint='方向键已收到，请先点击页面空白处，再按键。';if(lastStatus)render(lastStatus);return;}
  e.preventDefault();if(!e.repeat)press(keySource(e),keys[e.key]);}
});
window.addEventListener('keyup',e=>{if(keys[e.key]){e.preventDefault();release(keySource(e));}});
el('drive-stop').onclick=()=>stop('stop',false,'stop_button');el('drive-emergency').onclick=()=>stop('emergency',false,'emergency_button');
el('drive-focus').onclick=()=>{el('drive-panel').focus();localHint=allowed?(motorAvailable?'页面已获得键盘焦点：按住 W 前进，松开停车。':'页面已获得键盘焦点：按住 A 左转、D 右转，松键回正。'):'页面已获得键盘焦点；先启用短测，再按方向键。';if(lastStatus)render(lastStatus);};
el('drive-refresh').onclick=()=>window.location.reload();
el('drive-reconnect').onclick=async()=>{
 if(stale){window.location.reload();return;}
 if(lastStatus&&(['expired','closed','fault'].includes(lastStatus.mode)||lastStatus.worker_available===false)){render(lastStatus);return;}
 held.clear();urgentGeneration++;needsRelease=true;
 try{await send('emergency',{stop_source:'connection_recovery'});lastStatus=await send('reset');requestError='';localHint='';sequence=0;el('bench-ready').checked=false;render(lastStatus);}
 catch(e){requestError='重新连接失败：'+e.message;feedback(requestError,'error');}
};
el('bench-ready').onchange=()=>{if((lastStatus?.mode==='disabled'||lastStatus?.can_start_next_round)&&!enabling){requestError='';localHint='';render(lastStatus);}};
el('drive-next').onclick=async()=>{
 if(stale||enabling||!lastStatus?.can_start_next_round)return;
 if(!el('bench-ready').checked){feedback('请先确认车已停稳且现场就绪。','error');return;}
 enabling=true;allowed=false;held.clear();heartbeatDue=false;urgentGeneration++;needsRelease=true;
 gamepadControl?.requireNeutral();requestError='';localHint='';render(lastStatus);
 try{sequence=0;lastStatus=await send('next_trial',{bench_ready:true});el('bench-ready').checked=false;render(lastStatus);}
 catch(e){requestError='下一轮准备失败：'+e.message;feedback(requestError,'error');}
 finally{enabling=false;if(lastStatus)render(lastStatus);}
};
el('drive-enable').onclick=async()=>{
 if(stale||enabling)return;
 if(!el('bench-ready').checked){requestError=lastStatus?.ground_short_trial||lastStatus?.ground_held_trial?'启用未发送：请先勾选场地和电调就绪确认。':lastStatus?.continuous_simulation?'启用未发送：请先勾选模拟模式确认。':'启用未发送：请先勾选车轮架空和当前电调状态确认。';if(lastStatus)render(lastStatus);return;}
 enabling=true;requestError='';localHint='';if(lastStatus)render(lastStatus);
 try{sequence=0;held.clear();needsRelease=true;lastStatus=await send('enable',{bench_ready:true});}
 catch(e){requestError='启用失败：'+e.message;feedback(requestError,'error');}
 finally{enabling=false;if(lastStatus)render(lastStatus);}};
el('drive-reset').onclick=async()=>{try{requestError='';localHint='';render(await send('reset'));el('bench-ready').checked=false;}catch(e){requestError=e.message;feedback(requestError,'error');}};
window.addEventListener('blur',()=>stop('stop',false,'window_blur'));
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop('stop',false,'page_hidden');});
window.addEventListener('pagehide',()=>stop('stop',true,'page_exit'));
async function poll(){const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1500),started=performance.now();
 try{const response=await fetch('/api/drive/status',{cache:'no-store',signal:controller.signal});if(!response.ok)throw new Error('状态读取失败');const status=await response.json();latency=Math.round(performance.now()-started);render(status);}
 catch(e){held.clear();allowed=false;requestError='控制服务连接中断，请关闭电调后检查网络。';el('drive-link').textContent='连接中断';el('drive-link').style.color='#fca5a5';feedback(requestError,'error');}
 finally{clearTimeout(timer);el('drive-reconnect').hidden=!requestError||stale||lastStatus&&(['expired','closed','fault'].includes(lastStatus.mode)||lastStatus.worker_available===false);setTimeout(poll,800);}}
__GAMEPAD_CONTROLS__
gamepadControl=createUSBGamepadControls({arm:el('gamepad-arm'),info:el('gamepad-info'),
 getPads:typeof navigator.getGamepads==='function'?()=>navigator.getGamepads():null,
 now:()=>performance.now(),state:()=>({allowed:allowed&&!stopPending,stale,
 focused:!document.hidden&&document.hasFocus(),otherInput:Array.from(held.keys()).some(x=>!x.startsWith('pad:')),
 capabilities:{forward:motorAvailable,reverse:motorAvailable&&reverseAvailable,left:turnAvailable,right:turnAvailable}}),
 stop:(reason,emergency)=>stop(emergency?'emergency':'stop',false,reason),
 directions:directions=>{for(const source of Array.from(held.keys()))if(source.startsWith('pad:'))held.delete(source);
  for(const direction of directions)held.set('pad:'+direction,direction);heartbeat();}
});
setInterval(()=>gamepadControl.sample(),50);
setInterval(heartbeat,70);poll();
})();
</script>'''.replace('__GAMEPAD_CONTROLS__', GAMEPAD_SCRIPT)

AUX_SCRIPT = '''<script>
(()=>{
 const el=id=>document.getElementById(id);let attached=false,source=null;
 const image=el('aux-video');image.onerror=()=>{attached=false;};
 document.addEventListener('visibilitychange',()=>{if(document.hidden){image.removeAttribute('src');image.hidden=true;attached=false;}});
 el('camera').addEventListener('change',()=>{attached=false;image.hidden=true;image.removeAttribute('src');});
 async function poll(){const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1500);
  try{
   if(document.hidden){image.removeAttribute('src');image.hidden=true;attached=false;return;}
   if(el('camera').options.length<2){el('aux-camera').hidden=true;return;}
   el('aux-camera').hidden=false;const camera=el('camera').value==='primary'?'secondary':'primary';
   if(camera!==source){image.removeAttribute('src');attached=false;source=camera;}
   const response=await fetch('/api/status?camera='+camera,{cache:'no-store',signal:controller.signal});
   if(!response.ok)throw new Error('视频状态不可用');const status=await response.json();
   if(document.hidden)return;
   el('aux-title').textContent=camera==='secondary'?'第二路摄像头':'主摄像头';
   el('camera-info').textContent='两路视频持续采集，主摄优先；另一画面保留在下方。';
   const ready=status.state==='ok';el('aux-state').textContent=ready?'画面已连接':status.state==='starting'?'等待摄像头画面……':'画面暂不可用，请检查摄像头连接。';
   image.hidden=!ready;
   if(ready&&!attached){image.src='/stream.mjpg?camera='+camera+'&view=raw&t='+Date.now();attached=true;}
   if(!ready){image.removeAttribute('src');attached=false;}
  }catch(e){image.hidden=true;image.removeAttribute('src');attached=false;el('aux-state').textContent='第二路画面连接中断。';}
  finally{clearTimeout(timer);setTimeout(poll,1000);}
 }
 poll();
})();
</script>'''
