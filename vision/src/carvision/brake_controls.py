"""F/B trial controls added to the real car's existing video/control page."""

HTML = '''<section class="drive-panel" id="crosswalk-test">
<h3>斑马线测试 · F/B</h3>
<p id="crosswalk-program-summary">使用第二路 H65，停稳等待 10 秒并播报。</p>
<p id="crosswalk-test-state" role="status">正在读取状态</p>
<div class="crosswalk-distance"><label for="crosswalk-distance">刹车触发距离（cm）</label>
<input id="crosswalk-distance" type="number" min="25" max="65" step="1" value="55">
<button id="crosswalk-save-distance" type="button">保存距离</button></div>
<p id="crosswalk-settings-state" role="status">支持 25–65 cm，数值越大越早刹车。</p>
<select id="crosswalk-path-mode"><option value="straight">直线前进搜索斑马线</option>
<option value="lane">沿已识别赛道线接近</option></select>
<div class="crosswalk-actions"><button id="crosswalk-prepare" type="button" disabled>准备（电调关闭）</button>
<button id="crosswalk-start" type="button" disabled>开始</button>
<button id="crosswalk-cancel" type="button">停止</button></div>
<p id="crosswalk-intent"></p><p id="crosswalk-speech-state"></p>
<p>首次：电调保持 F/B，关闭电调点“准备”；打开电调、确认静止且前方空旷后点“开始”。</p>
<p>3 秒后前进；停稳后等待 10 秒并播报。完成后可直接开始下一轮。空格 / Esc 停止。</p>
<details><summary>参数与播报说明</summary><p id="crosswalk-reference-state"></p>
<p>距离保存后长期保留；行驶中修改用于下一轮。手动遥控会在准备时自动结束。</p>
<p>播报：我是山东大学启航队的智能车，请为我加油</p>
<p>F/B 模式倒车禁用；停稳判断使用视觉估计。</p></details>
</section>'''

SCRIPT = '''
function renderBrake(s){
 const t=s.crosswalk_trial||{};
 if(t.brake_trigger)el('crosswalk-program-summary').textContent='第二路 H65 · 约 '+Math.round(t.brake_trigger.distance_m*100)+' cm 刹车 · 停稳 '+t.hold_s+' 秒并播报';
 if(s.control_session_id&&s.control_session_id!==pageSession){expiredPage();return true;}
 const owned=t.owner_client===client;
 brakeActive=!!t.active;
 brakeOwned=owned;
 brakeToken=t.run_token;
 brakeSettingsRevision=t.settings_revision;
 el('crosswalk-test-state').textContent=t.prepare_blocked_reason||t.reason||'未加载刹车测试程序';
 el('crosswalk-reference-state').textContent=t.reference_summary||'';
 el('crosswalk-prepare').disabled=stale||!t.can_prepare||brakeSaveBusy;
 el('crosswalk-start').disabled=stale||!owned||!t.can_start||brakeSaveBusy;
 el('crosswalk-start').textContent=t.phase==='complete'?'开始下一轮':'开始';
 el('crosswalk-path-mode').disabled=!!t.active&&!['ready','complete'].includes(t.phase);
 el('crosswalk-distance').disabled=stale||brakeSaveBusy||t.active&&!owned;
 el('crosswalk-save-distance').disabled=el('crosswalk-distance').disabled;
 if(t.brake_trigger&&!brakeSettingsDirty&&document.activeElement!==el('crosswalk-distance'))el('crosswalk-distance').value=Math.round(t.brake_trigger.distance_m*100);
 if(!brakeSettingsDirty&&!brakeSaveBusy){
  const next=Math.round((t.brake_trigger?.distance_m||0)*100),current=Math.round((t.running_brake_trigger?.distance_m||0)*100);
  el('crosswalk-settings-state').textContent=t.settings_error||(t.active&&!['ready','complete','preparing'].includes(t.phase)&&next!==current?
   '本轮保持 '+current+' cm；已保存 '+next+' cm，下轮生效。':'已保存 '+next+' cm · 可改 25–65 cm，数值越大越早刹车。');
 }
 const p=t.intent;
 el('crosswalk-intent').textContent=t.phase==='arming'?'中位初始化：'+Math.ceil(t.arming_remaining_s)+' 秒':
  p?('阶段 '+p.phase+' · 输出 '+p.esc_us+' μs'+(p.hold_remaining_s!=null?' · 等待 '+p.hold_remaining_s.toFixed(1)+' 秒':'')):'';
 el('crosswalk-speech-state').textContent=({sending:'正在发送播报',command_sent:'已发送：'+(t.speech?.text||t.speech_text||''),failed:'语音发送失败',cancelled:'语音请求已结束'})[t.speech?.state]||'';
 if(!t.active)return false;
 lastStatus=s;held.clear();heartbeatDue=false;needsRelease=true;allowed=false;
 turnAvailable=false;motorAvailable=false;reverseAvailable=false;
 gamepadControl?.requireNeutral();
 el('drive-title').textContent='斑马线刹车测试';
 el('drive-mode').textContent=t.reason;
 el('drive-action').textContent=t.phase==='brake_pulse'?'主动制动':p?.action==='search'||p?.action==='creep'?'前进':'中位';
 el('drive-active-speed').textContent=p?p.esc_us+' μs':'准备中';
 el('drive-time').textContent=owned?'本页持有测试控制权':'测试由另一页面持有';
 el('drive-latency').textContent=latency==null?'—':latency+' ms';
 for(const id of ['drive-enable','drive-reset','drive-next','drive-speed','test-mode','gamepad-arm']){if(el(id))el(id).disabled=true;}
 document.querySelectorAll('[data-drive],[data-speed]').forEach(b=>b.disabled=true);
 feedback(t.reason,t.phase==='fault'?'error':'sending');
 return true;
}
async function brakeSend(action,body={}){
 return send(action,{run_token:brakeToken,trial_sequence:++brakeSequence,...body});
}
el('crosswalk-distance').addEventListener('input',()=>{
 brakeSettingsDirty=true;el('crosswalk-settings-state').textContent='尚未保存；点击保存，或开始时自动保存。';
});
async function saveBrakeSettings(force=false){
 if(!brakeSettingsDirty&&!force)return true;
 const input=el('crosswalk-distance');
 if(!input.checkValidity()){input.reportValidity();return false;}
 brakeSaveBusy=true;if(lastStatus)render(lastStatus);
 try{
  const state=await send('crosswalk_settings',{brake_distance_cm:Number(input.value),settings_revision:brakeSettingsRevision});
  brakeSettingsDirty=false;render(state);return true;
 }catch(e){el('crosswalk-settings-state').textContent=e.message;return false;}
 finally{brakeSaveBusy=false;if(lastStatus)render(lastStatus);}
}
el('crosswalk-save-distance').addEventListener('click',()=>saveBrakeSettings(true));
el('crosswalk-prepare').addEventListener('click',async()=>{
 held.clear();needsRelease=true;gamepadControl?.requireNeutral();
 try{
  if(!await saveBrakeSettings())return;
  requestError='';localHint='';brakeSequence=0;
  render(await send('crosswalk_prepare',{esc_off:true,fb_mode_confirmed:true}));
 }catch(e){feedback(e.message,'error');}
});
el('crosswalk-start').addEventListener('click',async()=>{
 try{
  if(!await saveBrakeSettings())return;
  requestError='';localHint='';
  render(await brakeSend('crosswalk_start',{field_ready:true,esc_on_static:true,fb_mode_confirmed:true,path_mode:el('crosswalk-path-mode').value}));
 }catch(e){feedback(e.message,'error');}
});
el('crosswalk-cancel').addEventListener('click',()=>stop('stop',false,'stop_button'));
setInterval(async()=>{
 if(!brakeActive||!brakeOwned||stale||document.hidden||brakePulseBusy>=2)return;
 brakePulseBusy++;
 try{render(await brakeSend('crosswalk_keepalive'));}
 catch(e){if(e.message!=='old or invalid brake sequence'){feedback(e.message,'error');stop('stop',false,'connection_recovery');}}
 finally{brakePulseBusy--;}
},100);
'''


def extend_controls(controls):
    script = controls.SCRIPT
    sentinel = 'let gamepadControl=null,pwmControl=null;'
    render = 'function render(s){'
    heartbeat = 'async function heartbeat(){'
    ending = script.rfind('})();')
    if any(script.count(s) != 1 for s in (sentinel, render, heartbeat)) or ending < 0:
        raise ValueError('需要已对齐的 10 月 9 日真实控制台，不能套用旧仓库遥控脚本')
    script = script.replace(sentinel, sentinel+'\nlet brakeActive=false,brakeOwned=false,brakeToken=null,brakeSequence=0,brakePulseBusy=0,brakeSettingsRevision=null,brakeSettingsDirty=false,brakeSaveBusy=false;')
    script = script.replace(render, render+'\n if(renderBrake(s))return;\n')
    # Keep the main console's manual status honest in F/B mode too.
    script = script.replace('renderLegacy(s);', '''renderLegacy(s);
 if(s.esc_mode==='F/B'){
  el('drive-limits').textContent='F/B 模式：W 前进、A/D 转向；S / 下键及手柄倒车已禁用。遥控松键回中位；斑马线入口使用主动制动。';
  el('gamepad-arm').disabled=false;
 }''', 1)
    script = script.replace(heartbeat, heartbeat+'\n if(brakeActive){heartbeatDue=false;return;}')
    ending = script.rfind('})();')
    controls.SCRIPT = script[:ending]+SCRIPT+script[ending:]
    controls.HTML = controls.HTML.replace('<b>LT</b> 倒车', '<b>LT</b> 已禁用（F/B）')
    controls.HTML += HTML
    controls.STYLE = getattr(controls, 'STYLE', '')+'''<style>
#crosswalk-test{background:white;border-radius:18px;padding:20px;display:grid;gap:12px}
#crosswalk-test p,#crosswalk-test h3{margin:0}
#crosswalk-test label{display:flex;gap:8px;align-items:center;line-height:1.6}
#crosswalk-test button,#crosswalk-test select,#crosswalk-distance{min-height:42px}
#crosswalk-test select{width:100%}
#crosswalk-test .crosswalk-distance{display:grid;grid-template-columns:1fr 85px auto;gap:8px;align-items:center}
#crosswalk-distance{width:100%;box-sizing:border-box;border:1px solid #d8d8e4;border-radius:10px;padding:8px}
#crosswalk-test .crosswalk-actions{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
#crosswalk-test button:enabled{background:#8e35e8;color:white;border-color:#8e35e8}
#crosswalk-test button:disabled{opacity:.45}
#crosswalk-test #crosswalk-cancel{background:#fff1f2;color:#b42335;border-color:#fecdd3}
#crosswalk-test button[hidden]{display:none}
</style>
'''
