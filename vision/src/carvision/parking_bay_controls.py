"""Parking observation on the real console, before the existing payment card."""
HTML='''<section class="drive-panel" id="parking-bay-test">
<h3>停车区 · 选区与入位判断</h3>
<p>停车区长 1 m，沿赛道中线分成两个半区。左／右按车辆前进方向定义，选择入口没有挡板的一侧。</p>
<p id="parking-bay-state" role="status">等待停车观察服务</p>
<div class="parking-bay-actions"><button id="parking-bay-observe" type="button" disabled>开始停车区识别</button><button id="parking-bay-stop" type="button">停止识别</button></div>
<p id="parking-bay-selection"></p><p id="parking-bay-slots"></p>
<div class="parking-bay-viewer"><img id="parking-bay-preview" alt="第二路实际黄线、挡板和停车半区识别" width="480" height="360" hidden><span id="parking-bay-placeholder">开始后显示实际摄像头识别图</span></div>
<details><summary>识别参数</summary>
<label>赛道总宽（cm）<input id="parking-bay-width" type="number" min="60" max="200" step="0.5" value="123"></label>
<label>入口蓝色占比阈值（%）<input id="parking-bay-blue" type="number" min="0.5" max="30" step="0.5" value="3"></label>
<label>空闲半区可见红色地面（%）<input id="parking-bay-floor" type="number" min="50" max="95" step="1" value="65"></label>
<button id="parking-bay-save" type="button">保存识别参数</button><p>按现有记录预填赛道宽123 cm，现场变化时修改。未见蓝色不直接等于空闲；黄线、边界和车位内部还需明确。</p></details>
<p id="parking-bay-readiness"></p><p id="parking-bay-error" role="alert"></p>
<p>当前网页入口只观察和推荐选区，不输出入位油门。自动入位决策需要已验证的近场地面标定、连续位姿及制动距离；缺失项不会用像素距离代替。</p>
<a href="#parking-payment">入位停稳后，前往下方停车缴费</a>
<p>这里不自动开始30秒付款计时，不把选区成功当作四轮已经入位。</p>
</section>'''

SCRIPT='''
function renderParkingBay(s){
 const p=s.parking_bay||{},cfg=p.settings||{};parkingBayActive=!!p.active;
 el('parking-bay-state').textContent=p.reason||'停车观察模块未加载';
 el('parking-bay-observe').disabled=stale||parkingBayBusy||!p.can_observe;
 for(const id of ['parking-bay-width','parking-bay-blue','parking-bay-floor','parking-bay-save'])el(id).disabled=stale||parkingBayBusy||parkingBayActive;
 if(!parkingBayDirty&&!parkingBayBusy){
  el('parking-bay-width').value=(cfg.lane_width_m||1.23)*100;
  el('parking-bay-blue').value=(cfg.blocked_blue_fraction||.03)*100;
  el('parking-bay-floor').value=(cfg.clear_ground_fraction||.65)*100;
 }
 const selected=p.selection||{},names={left:'左半区',right:'右半区'},labels={clear:'可见空闲候选',blocked:'入口遮挡',unknown:'未知'};
 el('parking-bay-selection').textContent=selected.confirmed?'推荐'+names[selected.selected_slot]+'（候选，尚未实际入位）':selected.reason||'等待新鲜画面和完整停车结构';
 el('parking-bay-slots').textContent=(p.observation?.slots||[]).map(r=>names[r.id]+'：'+labels[r.availability]+' · 入口蓝色'+(r.blue_entrance_fraction*100).toFixed(1)+'%').join(' / ');
 el('parking-bay-readiness').textContent='实车入位待补：'+(p.readiness?.missing||[]).join('、')+'。本入口电机输出关闭。';
 parkingBayFrame=p.observation?.frame_id??null;
}
async function parkingBayAction(action){
 if(parkingBayBusy)return;parkingBayBusy=true;el('parking-bay-error').textContent='';
 const body={settings_revision:lastStatus?.parking_bay?.settings_revision,lane_width_m:Number(el('parking-bay-width').value)/100,
  blocked_blue_fraction:Number(el('parking-bay-blue').value)/100,clear_ground_fraction:Number(el('parking-bay-floor').value)/100};
 if(lastStatus)render(lastStatus);
 try{const s=await send(action,body);if(action==='parking_bay_save')parkingBayDirty=false;render(s);}
 catch(e){el('parking-bay-error').textContent=e.message;}
 finally{parkingBayBusy=false;if(lastStatus)render(lastStatus);}
}
el('parking-bay-observe').addEventListener('click',()=>parkingBayAction('parking_bay_observe'));
el('parking-bay-stop').addEventListener('click',()=>parkingBayAction('parking_bay_stop'));
el('parking-bay-save').addEventListener('click',()=>parkingBayAction('parking_bay_save'));
for(const id of ['parking-bay-width','parking-bay-blue','parking-bay-floor'])el(id).addEventListener('input',()=>parkingBayDirty=true);
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
let parkingBayFrame=null,parkingBayLoaded=null,parkingBayURL=null,parkingBayImageBusy=false;
''',1)
    wrapper='''function render(s){
 if(typeof acceptTrafficSnapshot==='function'&&!acceptTrafficSnapshot(s))return;
 if(typeof s.parking_bay_status_s==='number'){if(s.parking_bay_status_s<parkingBayLast)return;parkingBayLast=s.parking_bay_status_s;}
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
.parking-bay-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px}#parking-bay-test button,#parking-bay-test input{min-height:42px;border:1px solid #d8d8e4;border-radius:10px;padding:8px}
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
