"""Concise cone task panel; calibration data and replay APIs stay in files."""

HTML = '''<section class="drive-panel" id="cone-test">
<h3>锥桶持续绕行 · 手动结束</h3><p id="cone-state" role="status">等待识别服务</p>
<div class="cone-actions"><button id="cone-observe" type="button">开始识别</button><button id="cone-stop" type="button">停止识别</button></div>
<label>场地<select id="cone-surface"><option value="red_running_track">红色跑道</option><option value="indoor_white_line_track">室内白线赛道</option></select></label><button id="cone-save-surface" type="button">保存场地</button>
<div class="cone-viewer"><img id="cone-preview" alt="真实双摄锥桶识别画面" width="960" height="360" hidden><span id="cone-placeholder">开始识别后显示实际画面</span></div><p id="cone-cameras"></p>
<div id="cone-low-speed-section">
<div class="cone-settings"><label>白线亮度（0–255）<input id="cone-white-v" type="number" min="0" max="255" step="1" value="200"></label><button id="cone-white-save" type="button">应用亮度</button></div><p id="cone-white-status"></p>
<div class="cone-settings"><label>前进油门（μs）<input id="cone-test-forward" type="number" min="1501" max="1625" step="1" value="1600"></label><span>持续运行，不按时间或通过数量自动结束</span></div>
<div class="cone-settings"><label>左转端点（μs）<input id="cone-test-left" type="number" min="1650" max="1870" step="5" value="1870"></label><label>右转端点（μs）<input id="cone-test-right" type="number" min="1350" max="1570" step="5" value="1350"></label></div>
<p id="cone-test-steering-state">右 1350 · 中位 1610 · 左 1870</p>
<p>持续低速绕桶，由你手动停止。白线预警、目标漏检、通过两桶都不自动刹停；转向随偏差逐步调整。断流、失联和硬件故障保护保留。</p>
<p id="cone-test-line-state"></p>
<div class="cone-actions"><button id="cone-test-prepare" type="button">准备（电调关闭、F/B）</button><button id="cone-test-start" type="button" disabled>开始绕桶测试</button></div>
<button id="cone-test-stop" type="button">停止／急停并结束</button>
<p id="cone-test-progress" role="status">准备后打开电调并确认静止，再开始。空格／Esc 随时急停。</p>
</div></section>'''

STATE = '''
let coneActive=false,coneDirty=false,coneWhiteDirty=false,coneBusy=false,coneImageBusy=false,coneImageURL=null,coneError='';
let coneDriveActive=false,coneDriveOwned=false,coneToken=null,coneSequence=0,conePulseBusy=false,coneRunQueue=Promise.resolve();
let coneLastSnapshot=-Infinity,coneShortActive=false;
'''

SCRIPT = '''
function renderCone(s){
 const c=s.cone_trial||{},a=c.avoidance||{},q=c.short_test||{};
 coneActive=!!c.observing;coneDriveActive=!!a.active;coneDriveOwned=a.owner_client===client;coneToken=a.run_token;coneShortActive=!!q.active;
 const locked=stale||coneBusy||coneDriveActive||coneShortActive;
 el('cone-state').textContent=coneError||c.reason||'锥桶服务未加载';
 el('cone-observe').disabled=locked||!c.can_observe;el('cone-stop').disabled=stale||coneBusy||!coneActive;
 el('cone-surface').disabled=el('cone-save-surface').disabled=locked||coneActive;
 if(!coneDirty&&!coneBusy&&c.settings)el('cone-surface').value=c.settings.scene?.surface||'red_running_track';
 el('cone-white-v').disabled=el('cone-white-save').disabled=locked;
 const white=c.settings?.scene?.white_v_min??200;
 if(!coneWhiteDirty&&!coneBusy)el('cone-white-v').value=white;
 el('cone-white-status').textContent='已保存 '+white+' / 255；越大越只保留亮白。';
 const trial=a.low_speed_test||{},oldTrial=a.profile==='legacy_low_speed';
 el('cone-test-prepare').disabled=stale||coneBusy||!a.can_prepare||!c.low_speed_trial_available;
 el('cone-test-start').disabled=stale||coneBusy||!coneDriveOwned||!a.can_request_start||!oldTrial;
 for(const id of ['cone-test-forward','cone-test-left','cone-test-right'])el(id).disabled=locked;
 el('cone-test-steering-state').textContent=a.steering_limits_us?('本轮：右 '+a.steering_limits_us[0]+' · 中位 1610 · 左 '+a.steering_limits_us[1]):'右 1350 · 中位 1610 · 左 1870；参数在下次准备生效';
 if(oldTrial)el('cone-test-progress').textContent=(a.reason||'')+' · 已过 '+(trial.passed_count??trial.operator_confirmed_count??0)+' 个（视觉估计） · 持续运行，由你停止'+(trial.best_effort_route?' · 当前间隙未明确，正在尝试避让':'');
 const nav=c.latest?.navigation;
 el('cone-test-line-state').textContent=nav?.experimental?('白线：'+(nav.line_sources?.length?nav.line_sources.map(x=>x==='primary'?'第一路':'第二路').join('、'):(nav.line_missing?'未确认，低速前进搜索':nav.line_held?'沿用最近边界，持续前进':'相机未就绪'))+(nav.reason?' · '+nav.reason:'')):'';
 const fresh=coneActive&&c.frame_age_s!=null&&c.frame_age_s>=0&&c.frame_age_s<=.25;
 el('cone-cameras').textContent=fresh?Object.entries(c.latest?.cameras||{}).map(([name,v])=>{
  const bd=v.road_context?.boundary_diagnostics,visible=(v.tracks||[]).filter(t=>t.visible&&t.confirmed&&t.kind==='cone_candidate');
  return (name==='secondary'?'第二路 H65':'第一路')+'：锥桶 '+visible.length+' 个，白线'+(v.lane_valid?'已确认':'未确认')+(bd?'（'+bd.support_rows+' 行）':'');
 }).join(' / '):'等待新鲜识别画面';
 if(!fresh){el('cone-preview').hidden=true;el('cone-placeholder').hidden=false;}
 if(coneDriveActive||coneShortActive){
  lastStatus=s;held.clear();heartbeatDue=false;needsRelease=true;allowed=false;turnAvailable=false;motorAvailable=false;reverseAvailable=false;gamepadControl?.requireNeutral();
  for(const id of ['drive-enable','drive-reset','drive-next','drive-speed','test-mode','gamepad-arm','drive-focus'])if(el(id))el(id).disabled=true;
  document.querySelectorAll('[data-drive],[data-speed]').forEach(b=>b.disabled=true);
  el('drive-title').textContent='锥桶单项测试';el('drive-mode').textContent=a.reason||q.reason;el('drive-action').textContent=a.intent?.action||'准备';
 }
 return coneDriveActive||coneShortActive;
}
async function coneAction(action,body={}){
 if(coneBusy)return;coneError='';coneBusy=true;if(lastStatus)render(lastStatus);
 try{const s=await send(action,{setup_token:lastStatus?.setup_token,...body});if(action==='cone_settings')coneDirty=false;if(action==='cone_white_threshold')coneWhiteDirty=false;render(s);}
 catch(e){coneError=e.message;feedback(e.message,'error');}
 finally{coneBusy=false;if(lastStatus)render(lastStatus);}
}
el('cone-surface').addEventListener('change',()=>coneDirty=true);
el('cone-white-v').addEventListener('input',()=>coneWhiteDirty=true);
el('cone-white-save').addEventListener('click',()=>{const n=el('cone-white-v');if(!n.checkValidity()){n.reportValidity();return;}coneAction('cone_white_threshold',{white_v_min:Number(n.value)});});
el('cone-save-surface').addEventListener('click',()=>coneAction('cone_settings',{scene_surface:el('cone-surface').value}));
el('cone-observe').addEventListener('click',()=>coneAction('cone_observe'));
el('cone-stop').addEventListener('click',()=>coneAction('cone_stop'));
function coneRunSend(action,body={}){
 const token=coneToken;
 const run=()=>{if(!coneDriveActive||stale||token!==coneToken)throw new Error('本轮已结束');return send(action,{run_token:token,trial_sequence:++coneSequence,...body});};
 const p=coneRunQueue.then(run,run);coneRunQueue=p.catch(()=>{});return p;
}
el('cone-test-prepare').addEventListener('click',()=>{
 for(const id of ['cone-test-forward','cone-test-left','cone-test-right'])if(!el(id).checkValidity()){el(id).reportValidity();return;}
 coneSequence=0;coneAction('cone_prepare',{profile:'legacy_low_speed',esc_off:true,fb_mode_confirmed:true,
  test_forward_us:Number(el('cone-test-forward').value),
  test_left_us:Number(el('cone-test-left').value),test_right_us:Number(el('cone-test-right').value)});
});
async function sendConeTest(action,body){
 if(coneBusy)return;coneBusy=true;coneError='';if(lastStatus)render(lastStatus);
 try{render(await coneRunSend(action,body));}catch(e){coneError=e.message;}
 finally{coneBusy=false;if(lastStatus)render(lastStatus);}
}
el('cone-test-start').addEventListener('click',()=>sendConeTest('cone_start',{fb_mode_confirmed:true,esc_on_static:true,field_ready:true}));
el('cone-test-stop').addEventListener('click',()=>stop('stop',false,'stop_button'));
setInterval(async()=>{
 if(!coneDriveActive||!coneDriveOwned||stale||document.hidden||conePulseBusy)return;
 conePulseBusy=true;
 try{render(await coneRunSend('cone_keepalive'));}catch(e){if(coneDriveActive){coneError=e.message;stop('stop',false,'connection_recovery');}}
 finally{conePulseBusy=false;}
},100);
setInterval(async()=>{
 if(!coneActive||stale||document.hidden||coneImageBusy)return;
 coneImageBusy=true;const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1500);
 try{
  const r=await fetch('/api/cone/frame.jpg',{cache:'no-store',signal:controller.signal});if(!r.ok)throw new Error('no fresh frame');
  const blob=await r.blob();if(!coneActive)return;
  const url=URL.createObjectURL(blob),old=coneImageURL;coneImageURL=url;
  el('cone-preview').src=url;el('cone-preview').hidden=false;el('cone-placeholder').hidden=true;if(old)URL.revokeObjectURL(old);
 }catch(e){el('cone-preview').hidden=true;el('cone-placeholder').hidden=false;}
 finally{clearTimeout(timer);coneImageBusy=false;}
},400);
'''


def extend_controls(controls):
    marker='if(!acceptTrafficSnapshot(s))return;'
    sentinel='let gamepadControl=null,pwmControl=null;'
    script=controls.SCRIPT
    if script.count(marker)!=1 or script.count(sentinel)!=1 or not controls.HTML.endswith('</div>') or 'id="autonomy-tests"' not in controls.HTML:
        raise ValueError('锥桶观察扩展需要现有斑马线／红绿灯组合控制台')
    script=script.replace(marker,marker+'\n if(typeof s.cone_status_s===\'number\'){if(s.cone_status_s<coneLastSnapshot)return;coneLastSnapshot=s.cone_status_s;}\n const coneDrivingBusy=renderCone(s);')
    script=script.replace('if(brakeBusy||trafficBusy)return;','if(brakeBusy||trafficBusy||coneDrivingBusy)return;')
    script=script.replace('async function heartbeat(){','async function heartbeat(){\n if(coneDriveActive||coneShortActive){heartbeatDue=false;return;}')
    script=script.replace(sentinel,sentinel+STATE)
    end=script.rfind('})();')
    if end<0:raise ValueError('console script ending missing')
    controls.SCRIPT=script[:end]+SCRIPT+script[end:]
    controls.HTML=controls.HTML[:-6]+HTML+'</div>'
    controls.STYLE=getattr(controls,'STYLE','')+'''<style>
#cone-test{background:#fff;border-radius:18px;padding:20px;display:grid;gap:12px;min-width:0}
#cone-test p,#cone-test h3{margin:0;line-height:1.6}#cone-test p{color:var(--text-secondary,#75677e)}
#cone-test .cone-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px}
#cone-test button,#cone-test input{min-height:42px;border-radius:10px;border:1px solid #d8d8e4}
#cone-test select{min-height:42px;border-radius:10px;border:1px solid #d8d8e4;background:#fff;color:#24212b;padding:8px}
#cone-test button:enabled{background:#8e35e8;color:#fff}#cone-test button:disabled{opacity:.45}
#cone-test .cone-settings{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
#cone-test label{display:grid;gap:6px}#cone-test input{width:100%;min-width:0;box-sizing:border-box;padding:8px;background:#fff;color:#24212b}
#cone-test input[type=checkbox]{width:auto;min-height:0}#cone-test textarea{width:100%;box-sizing:border-box;font:12px monospace;border:1px solid #d8d8e4;border-radius:8px;padding:8px}
#cone-test .cone-viewer{position:relative;aspect-ratio:8/3;background:#f7f4fa;display:grid;place-items:center;border-radius:10px;overflow:hidden}
#cone-preview{position:absolute;width:100%;height:100%;object-fit:contain}#cone-placeholder{padding:16px;text-align:center;color:#75677e}
</style>'''


def add_preview_endpoint(server,drive):
    from urllib.parse import urlsplit
    original=server.RequestHandlerClass
    class Handler(original):
        def do_GET(self):
            if urlsplit(self.path).path!='/api/cone/frame.jpg':return super().do_GET()
            with drive.lock:
                jpg=drive.preview
                fresh=drive.observing and drive.updated is not None and 0<=drive.clock()-drive.updated<=drive.settings['max_frame_age_s']
            try:
                return self.send_body(jpg if fresh and jpg else b'No fresh cone frame',
                    'image/jpeg' if fresh and jpg else 'text/plain',200 if fresh and jpg else 503)
            except (OSError,ConnectionError):self.close_connection=True
    server.RequestHandlerClass=Handler
    return server
