"""Plan B layout around the existing manual and authentic payment controls."""
HTML='''<section id="plan-b-panel" class="drive-panel" hidden>
<h3>Plan B · 全程5G遥控</h3>
<p>W/↑前进，S/↓倒车，A/D或左右键转向，可组合按键；松键停车，空格/Esc急停。</p>
<p id="plan-b-state" role="status">先选择Plan B，再在驾驶面板准备遥控</p>
<label><input id="plan-b-ready" type="checkbox">电调关闭，路线空旷，有人能立即断动力</label>
<button id="plan-b-begin" type="button">选择全程遥控模式</button>
<fieldset id="plan-b-speed-panel"><legend>自行调节油门</legend>
<label>前进比例（0～100%）<input id="plan-b-forward-percent" type="number" min="0" max="100" step="1" value="15"></label>
<label>倒车比例（0～100%）<input id="plan-b-reverse-percent" type="number" min="0" max="100" step="1" value="40"></label>
<button id="plan-b-speed-apply" type="button" disabled>应用前进／倒车油门</button>
<button id="plan-b-speed-max" type="button" disabled>前进设为100% · 2000 μs</button>
<p id="plan-b-speed-state">启用遥控后可调节；默认前进1575、倒车1300 μs。</p>
<p>端点由你提供：前进2000、停车1500、倒车1000 μs。键盘和手柄使用选定档位，扳机深浅暂不调速。比例不是实测车速；倒车要求电调处于支持倒车的模式。</p></fieldset>
<div class="plan-b-actions">
<button id="plan-b-crosswalk" type="button">已在斑马线前停稳 · 等10秒</button>
<button id="plan-b-team" type="button">播报队伍加油语</button>
<button id="plan-b-red" type="button">红灯停车</button>
<button id="plan-b-green" type="button">已看到绿灯 · 解锁</button>
<button id="plan-b-park-speech" type="button">播报“我停车了啊”</button>
<button id="plan-b-exit" type="button">结束Plan B并停车</button>
</div>
<p id="plan-b-countdown"></p><p id="plan-b-speech"></p>
<label><input id="plan-b-parked" type="checkbox">我确认四轮已在停车区域内，车辆已停稳</label>
<button id="plan-b-park" type="button">停车锁止并识别缴费二维码</button>
<p>绕桶与入位由你遥控。停车后在下方核对真实二维码，用手机支付0.01元，再记录已支付。程序不自动扣款。</p>
<p id="plan-b-error" role="alert"></p></section>'''

SCRIPT='''
function renderPlanB(s){
 if(typeof s.plan_b_status_s==='number'){if(s.plan_b_status_s<planBLastSnapshot)return;planBLastSnapshot=s.plan_b_status_s;}
 const p=s.plan_b||{},own=p.owner_client===client;
 el('plan-b-panel').hidden=!planBWanted&&!p.active;
 document.body.classList.toggle('plan-b-layout',planBWanted||p.active);
 el('plan-b-state').textContent=p.reason||'车端尚未安装Plan B';
 el('plan-b-begin').disabled=stale||planBBusy||(!p.can_begin&&!p.can_finish_manual);
 el('plan-b-begin').textContent=p.can_finish_manual?'结束普通遥控并选择全程遥控模式':'选择全程遥控模式';
 const blocked=!p.active||!own||stale||planBBusy;
 const speedReady=!blocked&&p.phase==='manual'&&s.mode==='enabled'&&s.manual_speed_adjustable;
 for(const id of ['plan-b-forward-percent','plan-b-reverse-percent','plan-b-speed-apply','plan-b-speed-max'])el(id).disabled=!speedReady;
 if(s.manual_speed_adjustable){
  if(!planBSpeedDirty&&!speedBusy)el('plan-b-forward-percent').value=s.speed_percent;
  if(!planBSpeedDirty&&!speedBusy)el('plan-b-reverse-percent').value=s.reverse_speed_percent;
  el('plan-b-speed-state').textContent='已选：前进 '+s.speed_percent+'% / '+s.selected_forward_pulse_us+' μs，倒车 '+s.reverse_speed_percent+'% / '+s.selected_reverse_pulse_us+' μs；当前输出 '+(s.motor_pulse_us==null?'尚未启用':s.motor_pulse_us+' μs');
  if(Number.isInteger(s.manual_steering_gain_percent))el('plan-b-speed-state').textContent+='；遥控转向幅度 '+s.manual_steering_gain_percent+'%（左 '+s.manual_steering_left_target_us+' / 右 '+s.manual_steering_right_target_us+' μs）';
 }else el('plan-b-speed-state').textContent=p.version==='plan-b-manual-speed-v2'?'调速版已安装；完成准备并启用持续遥控后，这个入口会开放。':'先安装调速版，再完成准备并启用持续遥控；当前入口尚未开放。';
 for(const id of ['plan-b-crosswalk','plan-b-red','plan-b-team','plan-b-park-speech','plan-b-park'])el(id).disabled=blocked;
 el('plan-b-green').disabled=blocked||p.phase!=='red_hold';
 el('plan-b-exit').disabled=stale||planBBusy||!p.active;
 el('plan-b-countdown').textContent=p.hold_remaining_s==null?'':('停车保持剩余 '+p.hold_remaining_s.toFixed(1)+' 秒');
 const speech=p.speech||{},names={not_requested:'尚未播报',sending:'正在发送',command_sent:'播报指令已发送',failed:'播报失败',cancelled:'已取消'};
 el('plan-b-speech').textContent=(names[speech.state]||speech.state||'')+(speech.error?'：'+speech.error:'');
 if(p.active&&p.payment_remaining_s!=null){
  el('plan-b-countdown').textContent=p.phase==='complete'?'已记录支付报告':('从确认停稳起，缴费剩余 '+p.payment_remaining_s.toFixed(1)+' 秒');
  el('payment-time').textContent=p.phase==='complete'?'已记录回执':p.payment_remaining_s.toFixed(1)+' 秒';
  el('payment-begin').textContent='付款码已核对 · 打开支付记录';
  if(!s.parking_payment?.outputs_released)el('payment-begin').disabled=true;
  if(p.operator_reported_within_30s!=null)el('payment-receipt').textContent=p.operator_reported_within_30s?'队员在确认停车后30秒内报告支付；到账需核对手机成功页。':'已记录超过停车30秒后的支付报告；不能据此认定限时加分。';
 }
 if(!p.active)return;
 if(!own||p.movement_blocked){
  held.clear();heartbeatDue=false;allowed=false;needsRelease=true;gamepadControl?.requireNeutral();
  for(const id of ['drive-enable','drive-reset','drive-next','drive-focus','gamepad-arm'])if(el(id))el(id).disabled=true;
  document.querySelectorAll('[data-drive]').forEach(n=>n.disabled=true);
 }
 if(el('test-mode'))el('test-mode').disabled=true;
 if(el('drive-title'))el('drive-title').textContent='Plan B · 全程手动遥控';
 if(p.phase==='manual'&&own&&s.mode==='enabled')el('drive-limits').textContent='持续遥控，松键停车；不设轮次或单次行驶时限。';
}
async function planBAction(action,body={}){
 if(planBBusy||stale)return;
 if(action==='plan_b_begin'&&!el('plan-b-ready').checked){el('plan-b-error').textContent='请先确认电调关闭和现场条件';return;}
 if(action==='plan_b_park'&&!el('plan-b-parked').checked){el('plan-b-error').textContent='请先确认四轮入位且已停稳';return;}
 planBBusy=true;el('plan-b-error').textContent='';
 if(['plan_b_begin','plan_b_hold','plan_b_park','plan_b_exit'].includes(action)){held.clear();heartbeatDue=false;urgentGeneration++;needsRelease=true;allowed=false;gamepadControl?.requireNeutral();}
 try{
  if(action==='plan_b_begin'&&lastStatus?.plan_b?.can_finish_manual){
   render(await send('finish_test'));
   const deadline=Date.now()+6000;
   while(lastStatus?.boot_test_phase==='closing'&&Date.now()<deadline){
    await new Promise(resolve=>setTimeout(resolve,200));
    render(await send('status'));
   }
   if(stale||lastStatus?.mode!=='disabled'||lastStatus?.boot_test_phase!=='esc_off')throw new Error('旧控制尚未结束，请保持电调关闭，稍后再点选择模式');
  }
  render(await send(action,{...body,esc_off:action==='plan_b_begin',payment_client:paymentClient}));
  if(action==='plan_b_begin')el('drive-panel').scrollIntoView({behavior:'smooth',block:'start'});
  if(action==='plan_b_park')el('parking-payment').scrollIntoView({behavior:'smooth',block:'start'});
 }catch(e){el('plan-b-error').textContent=e.message;}
 finally{planBBusy=false;if(lastStatus)render(lastStatus);}
}
el('plan-b-begin').onclick=()=>planBAction('plan_b_begin');
el('plan-b-crosswalk').onclick=()=>planBAction('plan_b_hold',{kind:'crosswalk',stationary_confirmed:true});
el('plan-b-red').onclick=()=>planBAction('plan_b_hold',{kind:'red'});
el('plan-b-green').onclick=()=>planBAction('plan_b_green',{green_confirmed:true});
el('plan-b-team').onclick=()=>planBAction('plan_b_announce',{phrase:'team'});
el('plan-b-park-speech').onclick=()=>planBAction('plan_b_announce',{phrase:'park'});
el('plan-b-park').onclick=()=>planBAction('plan_b_park',{parked_confirmed:true});
el('plan-b-exit').onclick=()=>planBAction('plan_b_exit');
el('plan-b-speed-apply').onclick=()=>{
 if(el('plan-b-speed-apply').disabled)return;
 const f=el('plan-b-forward-percent').valueAsNumber,r=el('plan-b-reverse-percent').valueAsNumber;
 if(!Number.isInteger(f)||!Number.isInteger(r)||f<0||f>100||r<0||r>100){el('plan-b-error').textContent='前进和倒车比例都填写0到100的整数';return;}
 el('plan-b-error').textContent='';planBSpeedDirty=false;changeSpeed(f,r);
};
for(const id of ['plan-b-forward-percent','plan-b-reverse-percent'])el(id).oninput=()=>{planBSpeedDirty=true;};
el('plan-b-speed-max').onclick=()=>{if(el('plan-b-speed-max').disabled)return;planBSpeedDirty=false;changeSpeed(100);};
'''


def extend_controls(c):
    marker='function render(s){';sentinel='let gamepadControl=null,pwmControl=null;'
    if c.SCRIPT.count(marker)!=1 or c.SCRIPT.count(sentinel)!=1:
        raise ValueError('Plan B需要既有完整控制台，不覆盖未知网页')
    script=c.SCRIPT.replace(marker,'function renderBeforePlanB(s){',1)
    script=script.replace(sentinel,sentinel+"let planBBusy=false,planBSpeedDirty=false,planBLastSnapshot=-Infinity;const planBWanted=new URLSearchParams(location.search).get('mode')==='planb';",1)
    end=script.rfind('})();')
    if end<0:raise ValueError('控制台闭包缺失')
    wrapper='function render(s){renderBeforePlanB(s);renderPlanB(s);}\n'
    c.SCRIPT=script[:end]+wrapper+SCRIPT+script[end:]
    c.HTML=HTML+c.HTML
    c.STYLE=getattr(c,'STYLE','')+'''<style>
#plan-b-panel{background:#17233a;border:2px solid #60a5fa;border-radius:16px;padding:20px;display:grid;gap:12px;margin:18px 0}
#plan-b-panel[hidden]{display:none}#plan-b-panel h3,#plan-b-panel p{margin:0}#plan-b-panel p{color:#d5e0ee}
#plan-b-panel button{min-height:48px}#plan-b-panel button:disabled{opacity:.4}#plan-b-panel label{line-height:1.6}
#plan-b-begin{background:#2464c7}.plan-b-actions{display:grid;grid-template-columns:1fr 1fr;gap:10px}
#plan-b-speed-panel{border:1px solid #60a5fa;border-radius:10px;display:grid;gap:10px;padding:14px}
#plan-b-speed-panel input{max-width:140px}#plan-b-speed-max{background:#805120}
#plan-b-exit,#plan-b-red{background:#a62232}#plan-b-countdown{font-size:24px;font-weight:bold}#plan-b-error{color:#fda4af!important}
.plan-b-layout #race-flow-panel,.plan-b-layout #crosswalk-test,.plan-b-layout #traffic-test,.plan-b-layout #cone-test,
.plan-b-layout #departure-test,.plan-b-layout #parking-bay-test,.plan-b-layout .vision-details{display:none!important}
@media(max-width:600px){.plan-b-actions{grid-template-columns:1fr}}
</style>'''
