"""Raw S3 pulse tuning for the existing isolated, finite steering trial."""

HTML = '''<section id="pwm-panel" class="pwm-panel" aria-labelledby="pwm-title">
<h3 id="pwm-title">S3 转向 PWM 试调</h3>
<p id="pwm-note" role="status">正在读取转向测试状态……</p>
<label for="pwm-slider">目标脉宽 <output id="pwm-wanted">—</output></label>
<input id="pwm-slider" type="range" min="1550" max="1750" step="1" value="1650" disabled>
<div class="pwm-steps"><button data-pwm-step="-5" disabled>−5</button><button data-pwm-step="-1" disabled>−1</button>
<input id="pwm-number" type="number" min="1550" max="1750" step="1" value="1650" aria-label="S3 目标脉宽，微秒" disabled>
<button data-pwm-step="1" disabled>+1</button><button data-pwm-step="5" disabled>+5</button></div>
<p class="pwm-values">已下发：<strong id="pwm-applied">—</strong> · 原中位：<span id="pwm-center">—</span></p>
<nav class="pwm-actions"><button id="pwm-try" disabled>试调当前值</button><button id="pwm-stop" disabled>停止试调并回正</button></nav>
<p>调节即试用，单次最多 2 秒；可连续拖动更新目标。停止、切出页面或失联后回原中位。电调保持关闭，参数不会自动保存。</p>
<p class="pwm-small">已下发表示软件输出记录，不是测得的车轮角度。</p>
</section>'''

STYLE = '''<style>
.pwm-panel{margin:18px 0;padding:16px 0;border-block:1px solid #27364f}
.pwm-panel h3{margin:0 0 8px;font-size:18px}.pwm-panel label{display:flex;justify-content:space-between;align-items:center}
#pwm-wanted{font-size:23px;color:#93c5fd;font-variant-numeric:tabular-nums}
#pwm-slider{width:100%;height:30px;margin:12px 0;accent-color:#60a5fa}
.pwm-steps{display:grid;grid-template-columns:1fr 1fr 86px 1fr 1fr;gap:5px}
.pwm-steps button{padding:8px 3px}.pwm-steps input{min-width:0;text-align:center;background:#0b1120;color:#e2e8f0;border:1px solid #475569;border-radius:6px}
.pwm-actions{display:flex;gap:8px}.pwm-actions button{flex:1;font-size:12px;padding:10px 5px}
.pwm-values{font-variant-numeric:tabular-nums}.pwm-small{color:#94a3b8}
</style>'''

SCRIPT = r'''
function createSteeringPWMControls({el,request,status,render,stop,onError}){
 let snapshot=null,active=false,starting=false,busy=false,due=false,epoch=0,timer=null,initialized=false,wanted=1650;
 const steps=Array.from(document.querySelectorAll('[data-pwm-step]'));
 const inputs=[el('pwm-slider'),el('pwm-number'),el('pwm-try'),...steps];
 function cancel(){active=false;starting=false;due=false;epoch++;if(timer!==null)clearTimeout(timer);timer=null;}
 function available(){return snapshot?.steering_pwm_available===true&&snapshot?.motor_available===false;}
 function ready(){return available()&&snapshot.mode==='enabled'&&status().allowed&&!status().otherInput;}
 function displayWanted(){el('pwm-slider').value=wanted;el('pwm-number').value=wanted;el('pwm-wanted').textContent=wanted+' μs';}
 function update(s){
  snapshot=s;
  el('pwm-panel').hidden=!available()&&s.test_mode!=='steering_pwm';
  if(!initialized&&available()){wanted=s.steering_center_us;initialized=true;displayWanted();}
  if(active&&(!available()||s.mode!=='enabled'||s.controls_paused||s.worker_available===false||(!starting&&s.release_required)))cancel();
  const enabled=ready();inputs.forEach(b=>b.disabled=!enabled);el('pwm-stop').disabled=!active&&s.steering_pwm_active!==true;
  const prefix=s.hardware_output?'':'模拟 ';
  el('pwm-applied').textContent=(s.worker_available===false||s.controls_paused||s.steering_pulse_us==null)?'—':prefix+s.steering_pulse_us+' μs';
  el('pwm-center').textContent=s.steering_center_us==null?'—':s.steering_center_us+' μs';
  el('pwm-note').textContent=!available()?'仅在单独 S3 转向测试模式开放；暂停预览和行驶模式不能调节。':s.mode!=='enabled'?'先确认现场就绪并启用本轮转向测试。':active?'试调中 · 只更新最新目标 · 2 秒后回原中位':s.hardware_output?'实车 S3 · 拖动即可开始一次试调':'模拟 S3 · 拖动可检查界面，不输出硬件信号';
 }
 async function flush(){
  if(!active||starting)return;
  if(!ready()){stop();return;}
  if(busy){due=true;return;}
  busy=true;due=false;const id=epoch,pulse=wanted;
  try{
   const s=await request('steering_pwm',{pulse_us:pulse});
   if(id===epoch&&active)render(s);
  }catch(e){if(id===epoch&&active){cancel();onError(e);stop();}}
  finally{busy=false;if(id===epoch&&active&&(due||wanted!==pulse))flush();}
 }
 async function tune(value){
  const pulse=Number(value);
  if(!Number.isInteger(pulse)||pulse<1550||pulse>1750){displayWanted();onError(new Error('请输入 1550–1750 范围内的整数微秒值'));return;}
  if(!ready())return;
  wanted=pulse;displayWanted();
  if(active){due=true;flush();return;}
  // Do not start a second trial while an old in-flight request is unresolved.
  if(busy)return;
  active=true;starting=true;busy=true;const id=++epoch;
  timer=setTimeout(()=>{if(id===epoch&&active)stop();},2000);
  update(snapshot);
  try{
   const s=await request('command',{motor:'stop',steering:'center'});
   if(id!==epoch||!active)return;
   render(s);starting=false;
  }catch(e){if(id===epoch&&active){cancel();onError(e);stop();}}
  finally{busy=false;if(id===epoch&&active&&!starting)flush();}
 }
 el('pwm-slider').oninput=e=>tune(e.target.value);
 el('pwm-number').onchange=e=>tune(e.target.value);
 steps.forEach(b=>b.onclick=()=>tune(Math.max(1550,Math.min(1750,wanted+Number(b.dataset.pwmStep)))));
 el('pwm-try').onclick=()=>tune(wanted);
 el('pwm-stop').onclick=()=>stop();
 setInterval(()=>{if(active&&!starting)flush();},70);
 return {update,cancel,isActive:()=>active};
}
'''
