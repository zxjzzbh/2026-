"""Speech-only blue-board departure card before the crosswalk task."""
HTML='''<section class="drive-panel" id="departure-test">
<h3>发车挡板测试 · 仅播报</h3>
<p>蓝色挡板：宽 40 cm × 高 50 cm。先确认挡板存在，再确认移开；本轮不输出前进。</p>
<p id="departure-state" role="status">等待车端状态</p>
<label>出现所需蓝色面积占比（%）<input id="departure-area" type="number" min="0.5" max="100" step="0.5" value="19.53125"></label>
<p>默认值对应参考算法的 640×480 图像 60000 像素。移开判断使用独立的低面积阈值；这是蓝色轮廓总面积比例，不是识别置信度。</p>
<label>移开确认时间（秒）<input id="departure-clear" type="number" min="0.5" max="3" step="0.1" value="0.6"></label>
<div class="departure-actions"><button id="departure-save" type="button" disabled>保存参数</button>
<button id="departure-test-speech" type="button" disabled>试听“我发车啦”</button></div>
<p id="departure-settings-state"></p><p id="departure-speech-test-state" role="status"></p>
<div class="departure-actions"><button id="departure-start" type="button" disabled>开始发车测试</button>
<button id="departure-stop" type="button">停止测试</button></div>
<p id="departure-observation"></p><p id="departure-speech" role="status"></p><p id="departure-error" role="alert"></p>
<div class="departure-viewer"><img id="departure-preview" alt="挡板识别实拍：左第一路，右第二路 H65" width="960" height="360" hidden>
<span id="departure-placeholder">开始后显示两路实际识别画面</span></div>
<p>任一路稳定看到挡板 0.5 秒后，进入等待移开；抬起造成的蓝色形状变化保留此前确认记录，暂停移开计时。两路画面明确清空并达到设定时间后，播报一次“我发车啦”。已确认后，1秒内的短时画面质量异常暂停移开计时；恢复后重新完整确认清空。缺帧、过期或持续异常需重新确认挡板。</p>
<p>停止后可修改参数，点击保存或开始测试都会保存到车端，下次打开网页仍保留。试听仅验证语音，不表示检测到挡板移开。</p>
<p>开始后保持车和镜头不动，由人移开挡板。完成后保留最后识别图；再次点击开始可测下一轮。停止／空格／Esc 结束测试。两分钟未完成自动结束。</p>
</section>'''

SCRIPT='''
function renderDeparture(s){
 const d=s.departure_trial||{},cfg=d.settings||{};departureActive=!!d.active;
 el('departure-state').textContent=d.reason||'发车模块未加载';
 el('departure-start').disabled=stale||departureBusy||!d.can_start;
 for(const id of ['departure-area','departure-clear','departure-save'])el(id).disabled=stale||departureBusy||!d.can_save;
 el('departure-test-speech').disabled=stale||departureBusy||!d.can_test_speech;
 if(!departureSettingsDirty&&!departureBusy){
 el('departure-area').value=cfg.min_blue_percent??19.53125;el('departure-clear').value=cfg.clear_confirm_s??0.6;
 }
 el('departure-settings-state').textContent=d.settings_error||'已保存：出现蓝色面积 '+(cfg.min_blue_percent??19.53125)+'% · 移开确认 '+(cfg.clear_confirm_s??0.6)+' 秒'+(departureSettingsDirty?'（输入有修改，保存或开始时生效）':'');
 el('departure-start').textContent=d.phase==='complete'?'开始下一轮':'开始发车测试';
 const rows=d.observations||{},names={primary:'第一路',secondary:'第二路'},states={present:'有挡板',absent:'未见挡板',unknown:'画面不明确'};
 el('departure-observation').textContent=Object.entries(rows).map(([c,r])=>names[c]+'：'+states[r.state]+(r.frame_id!=null?' #'+r.frame_id:'')+(r.largest_blue_fraction!=null?' · 蓝色 '+(r.largest_blue_fraction*100).toFixed(1)+'%':'')).join(' / ')+(d.phase==='wait_removed'?' · 移开确认 '+(d.clear_elapsed_s||0).toFixed(1)+' / '+d.clear_confirm_s+' 秒':'');
 const voice=d.speech;
 el('departure-speech').textContent=voice?voice.command_sent?'“我发车啦”播报指令已发送，请现场确认声音；车辆不前进。':voice.state==='failed'?'播报失败：'+voice.error:'正在发送“我发车啦”播报指令':'';
 const selftest=d.speech_test;
 el('departure-speech-test-state').textContent=selftest?selftest.command_sent?'试听指令已发送；请现场确认声音，这不是移开判定结果。':selftest.state==='failed'?'试听失败：'+selftest.error:selftest.state==='cancelled'?'试听已取消':'正在发送试听指令':'';
 departurePreviewKey=Object.entries(rows).map(([c,r])=>c+':'+r.frame_id+':'+r.state).join('/')+'/'+d.phase;
 departurePreviewWanted=departureActive||d.phase==='complete';
 if(!departureActive)return;
 allowed=false;turnAvailable=false;motorAvailable=false;reverseAvailable=false;held.clear();needsRelease=true;heartbeatDue=false;gamepadControl?.requireNeutral();
 el('drive-title').textContent='发车识别测试 · 仅播报';el('drive-mode').textContent='测试中不输出前进，先停止测试再切换驾驶任务';
 for(const id of ['drive-enable','drive-reset','drive-next','drive-focus','test-mode','gamepad-arm','crosswalk-prepare','crosswalk-start','traffic-prepare','traffic-start','cone-prepare','cone-drive-start','cone-short-prepare','payment-begin'])if(el(id))el(id).disabled=true;
 document.querySelectorAll('[data-drive],[data-speed],.cone-short-starts button').forEach(b=>b.disabled=true);
}
async function departureAction(action){
 if(departureBusy)return;departureBusy=true;el('departure-error').textContent='';
 const clear=Number(el('departure-clear').value),area=Number(el('departure-area').value),revision=lastStatus?.departure_trial?.settings_revision;
 if(lastStatus)render(lastStatus);
 try{
  const response=await send(action,{clear_confirm_s:clear,min_blue_percent:area,settings_revision:revision});
  if(action==='departure_save'||action==='departure_start')departureSettingsDirty=false;
  render(response);
 }catch(e){el('departure-error').textContent=e.message;}
 finally{departureBusy=false;if(lastStatus)render(lastStatus);}
}
el('departure-start').addEventListener('click',()=>departureAction('departure_start'));
el('departure-stop').addEventListener('click',()=>departureAction('departure_stop'));
el('departure-save').addEventListener('click',()=>departureAction('departure_save'));
el('departure-test-speech').addEventListener('click',()=>departureAction('departure_test_speech'));
for(const id of ['departure-area','departure-clear'])el(id).addEventListener('input',()=>departureSettingsDirty=true);
setInterval(async()=>{
 if(stale||document.hidden||!departurePreviewWanted||departureImageBusy||departureLoadedKey===departurePreviewKey)return;
 departureImageBusy=true;const key=departurePreviewKey,controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1500);
 try{
  const r=await fetch('/api/departure/frame.jpg',{cache:'no-store',signal:controller.signal});if(!r.ok)throw new Error('no frame');
  const url=URL.createObjectURL(await r.blob()),old=departureImageURL;departureImageURL=url;departureLoadedKey=key;
  el('departure-preview').src=url;el('departure-preview').hidden=false;el('departure-placeholder').hidden=true;
  if(old)URL.revokeObjectURL(old);
 }catch(e){el('departure-placeholder').textContent='等待新的实际画面';}
 finally{clearTimeout(timer);departureImageBusy=false;}
},400);
'''


def extend_controls(controls):
    script=controls.SCRIPT;marker='function render(s){';sentinel='let gamepadControl=null,pwmControl=null;'
    if script.count(marker)!=1 or script.count(sentinel)!=1 or controls.HTML.count('<div id="autonomy-tests">')!=1:
        raise ValueError('发车测试需要当前已对齐的控制台，不覆盖未知界面')
    script=script.replace(sentinel,sentinel+'''
let departureActive=false,departureBusy=false,departureSettingsDirty=false,departureLastStatus=-Infinity,departurePreviewWanted=false;
let departureImageBusy=false,departureImageURL=null,departurePreviewKey=null,departureLoadedKey=null;
''',1).replace(marker,'function renderBeforeDeparture(s){',1)
    script=script.replace('async function heartbeat(){','async function heartbeat(){\n if(departureActive){heartbeatDue=false;return;}',1)
    wrapper='''
function render(s){
 if(typeof acceptTrafficSnapshot==='function'&&!acceptTrafficSnapshot(s))return;
 if(typeof s.departure_status_s==='number'){
  if(s.departure_status_s<departureLastStatus)return;departureLastStatus=s.departure_status_s;
 }
 renderBeforeDeparture(s);renderDeparture(s);
}
'''
    end=script.rfind('})();')
    if end<0:raise ValueError('控制台闭包缺失')
    controls.SCRIPT=script[:end]+wrapper+SCRIPT+script[end:]
    controls.HTML=controls.HTML.replace('<div id="autonomy-tests">','<div id="autonomy-tests">'+HTML,1)
    controls.STYLE=getattr(controls,'STYLE','')+'''<style>
#departure-test{background:#fff;border-radius:18px;padding:20px;display:grid;gap:12px;min-width:0}
#departure-test h3,#departure-test p{margin:0;line-height:1.6}#departure-test p{color:#75677e}
#departure-test label{display:flex;align-items:center;gap:10px}.departure-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px}
#departure-test button,#departure-clear,#departure-area{min-height:42px;border:1px solid #d8d8e4;border-radius:10px;padding:8px}#departure-clear,#departure-area{width:90px}
#departure-test button:enabled{background:#8e35e8;color:white}#departure-test button:disabled{opacity:.45}
#departure-test #departure-stop{background:#fff1f2;color:#b42335;border-color:#fecdd3}
.departure-viewer{position:relative;aspect-ratio:8/3;background:#f7f4fa;display:grid;place-items:center;overflow:hidden;border-radius:10px}
#departure-preview{position:absolute;width:100%;height:100%;object-fit:contain}#departure-placeholder{padding:15px;text-align:center}
#departure-error{color:#b42335!important}
</style>'''


def add_endpoints(server,drive):
    from urllib.parse import urlsplit
    original=server.RequestHandlerClass
    class Handler(original):
        def do_GET(self):
            if urlsplit(self.path).path!='/api/departure/frame.jpg':return super().do_GET()
            jpg=drive.image()
            try:return self.send_body(jpg or b'No departure frame','image/jpeg' if jpg else 'text/plain',200 if jpg else 503)
            except (OSError,ConnectionError):self.close_connection=True
    server.RequestHandlerClass=Handler
    return server
