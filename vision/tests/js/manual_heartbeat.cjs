// Delayed control replies and keyboard release, without network or hardware.
const assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const script=JSON.parse(fs.readFileSync(0,'utf8')).dashboard
 .replace(/^<script>\s*/,'').replace(/\s*<\/script>$/,'')
 .replace('__DRIVE_KEY__','"key"').replace('__DRIVE_SESSION__','"session"');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
async function setup(ground=false,initial={}){
 const elements=new Map(),events={},intervals={},timeouts={},requests=[],pending=[];
 const directions=['forward','reverse','left','right'].map(d=>({dataset:{drive:d},style:{},disabled:false,
  classList:{add(){},remove(){},toggle(){}},setPointerCapture(){}}));
 function el(id){if(!elements.has(id))elements.set(id,{textContent:'',style:{},checked:false,hidden:false,
  disabled:false,value:20,focus(){},classList:{add(){},remove(){},toggle(){}}});return elements.get(id);}
 let now=0,inFlight=0,maxInFlight=0;
 let status={mode:'enabled',reason:'mock',hardware_output:ground,controls_paused:false,
  combined_trial:ground,steering_signal_active:ground,steering_center_us:1715,steering_pulse_us:1715,
  continuous_simulation:false,ground_held_trial:ground,motion_hold_max_s:ground?60:null,
  test_session_s:ground?180:120,motor_available:true,steering_available:ground,reverse_available:false,
  motor_direction:'stop',steering_direction:'center',motor_pulse_us:1500,session_remaining_s:60,
  control_session_id:'session',release_required:false,...initial};
 const context={navigator:{getGamepads:()=>[]},document:{hidden:false,hasFocus:()=>true,
  getElementById:el,querySelectorAll:selector=>selector==='[data-drive]'?directions:[],
  addEventListener:(name,fn)=>{events[name]=fn;}},window:{addEventListener:(name,fn)=>{events[name]=fn;},location:{reload(){}}},
  crypto:{getRandomValues:array=>array.fill(1)},Uint8Array,AbortController,performance:{now:()=>now},
  setTimeout:(fn,ms)=>{timeouts[ms]=fn;return 1;},clearTimeout(){},setInterval:(fn,ms)=>{intervals[ms]=fn;},
  fetch:async(url,options={})=>{
   if(!options.body)return {ok:true,json:async()=>({...status})};
   const body=JSON.parse(options.body);requests.push({url,body,at:now});
   if(url.endsWith('/command')&&body.motor==='forward'){
    inFlight++;maxInFlight=Math.max(inFlight,maxInFlight);
    const snapshot={...status,motor_direction:'forward',motor_pulse_us:1575,last_command_sequence:body.sequence,
     steering_direction:body.steering??'center',
     steering_pulse_us:body.steering==='left'?1750:body.steering==='right'?1550:1715};
    return new Promise((resolve,reject)=>pending.push({
     resolve(){inFlight--;resolve({ok:true,json:async()=>snapshot});},
     reject(message='late reply failed'){inFlight--;reject(new Error(message));}}));
   }
   if(url.endsWith('/emergency'))status={...status,mode:'emergency',motor_direction:'stop',motor_pulse_us:1500};
   if(url.endsWith('/stop'))status={...status,motor_direction:'stop',motor_pulse_us:1500};
   if(url.endsWith('/next_trial'))status={...status,mode:'starting',trial_token:'trial-new',
     trial_number:2,can_start_next_round:false,motor_direction:'stop',motor_pulse_us:1500};
   return {ok:true,json:async()=>({...status})};
  }};
 vm.createContext(context);vm.runInContext(script,context);await flush();
 const key=(type)=>events[type]({code:'KeyW',key:'w',repeat:false,target:{tagName:'DIV'},preventDefault(){}});
 return {el,events,intervals,requests,pending,key,getMaxInFlight:()=>maxInFlight,
  setStatus:update=>{status={...status,...update};},poll:()=>timeouts[800](),
  setTime:(time)=>{now=time;},countMotion:()=>requests.filter(r=>r.url.endsWith('/command')&&r.body.motor==='forward').length};
}
async function main(){
 let checks=0;
 // A 150-ms reply crosses two timer ticks. Send the current key immediately
 // after the reply, with exactly one in-flight request and no old-key queue.
 const h=await setup();h.key('keydown');await flush();assert.equal(h.countMotion(),1);checks++;
 h.setTime(70);h.intervals[70]();h.setTime(140);h.intervals[70]();
 assert.equal(h.countMotion(),1);checks++;
 h.setTime(150);h.pending.shift().resolve();await flush();
 assert.equal(h.countMotion(),2);assert.equal(h.requests.at(-1).at,150);checks++;
 assert.equal(h.getMaxInFlight(),1);checks++;
 h.key('keyup');await flush();assert.equal(h.requests.at(-1).url,'/api/drive/stop');checks++;
 h.pending.shift().resolve();await flush();h.intervals[70]();await flush();
 assert.equal(h.countMotion(),2);checks++;
 assert.equal(h.el('drive-action').textContent.startsWith('停车'),true);checks++;
 // A late failure after key release must not replace an already confirmed
 // stop or re-arm movement; a new press still starts with neutral release.
 const r=await setup();r.key('keydown');await flush();r.setTime(70);r.intervals[70]();
 r.key('keyup');await flush();r.pending.shift().reject();await flush();
 assert.equal(r.el('drive-feedback').textContent.includes('控制请求失败'),false);checks++;
 r.key('keydown');await flush();assert.equal(r.countMotion(),2);checks++;
 assert.equal(r.requests.at(-2).body.motor,'stop');checks++;
 // Emergency while a response is pending invalidates the reply and any due tick.
 const e=await setup();e.key('keydown');await flush();e.setTime(70);e.intervals[70]();
 e.events.keydown({code:'Space',key:' ',repeat:false,target:{tagName:'DIV'},preventDefault(){}});await flush();
 e.pending.shift().resolve();await flush();e.intervals[70]();await flush();
 assert.equal(e.countMotion(),1);assert.equal(e.el('drive-action').textContent,'已急停');checks++;
 // Ground W+A chord: releasing A keeps W, but the follow-up uses center;
 // releasing W then cancels any pending reply without resending forward.
 const g=await setup(true);g.key('keydown');await flush();g.pending.shift().resolve();await flush();
 assert.equal(g.el('drive-feedback').textContent.includes('＋左转'),false);checks++;
 const a=(type)=>g.events[type]({code:'KeyA',key:'a',repeat:false,target:{tagName:'DIV'},preventDefault(){}});
 a('keydown');await flush();assert.equal(g.requests.at(-1).body.steering,'left');
 a('keyup');await flush();g.pending.shift().resolve();await flush();
 assert.equal(g.requests.at(-1).body.motor,'forward');assert.equal(g.requests.at(-1).body.steering,'center');checks++;
 const count=g.countMotion();g.key('keyup');await flush();assert.equal(g.requests.at(-1).url,'/api/drive/stop');
 g.pending.shift().resolve();await flush();g.intervals[70]();await flush();assert.equal(g.countMotion(),count);checks++;
 // Waiting for the operator is a real ground trial with an available enable
 // button, no preparation countdown and no direction requests before enable.
 const w=await setup(true,{mode:'disabled',hardware_output:false,pending_hardware_start:true,
  worker_started:false,session_remaining_s:null,test_session_s:600,motion_hold_max_s:600});
 assert.equal(w.el('drive-enable').disabled,false);
 assert.equal(w.el('bench-confirm-text').textContent.includes('车已落地'),true);
 assert.equal(w.el('drive-time').textContent.includes('10 分钟'),true);checks++;
 w.key('keydown');await flush();w.intervals[70]();await flush();
 assert.equal(w.countMotion(),0);assert.equal(w.requests.length,0);checks++;
 // An old forward reply must not reactivate the next round. Starting it needs
 // a fresh ready checkbox, sends only an explicit round request, and waits
 // for the new worker before accepting a new keyboard press.
 const n=await setup(true,{ground_held_trial:false,ground_short_trial:true,
  repeatable_trials:true,trial_token:'trial-old',trial_number:1});
 n.key('keydown');await flush();assert.equal(n.countMotion(),1);checks++;
 n.setStatus({mode:'expired',can_start_next_round:true,trial_completed_normally:true});
 await n.poll();await flush();
 assert.equal(n.el('drive-next').hidden,false);assert.equal(n.el('drive-next').disabled,true);checks++;
 const before=n.requests.length;await n.el('drive-next').onclick();
 assert.equal(n.requests.length,before);checks++;
 n.el('bench-ready').checked=true;n.el('bench-ready').onchange();
 assert.equal(n.el('bench-ready').checked,true);assert.equal(n.el('drive-next').disabled,false);checks++;
 await n.el('drive-next').onclick();await flush();
 assert.equal(n.requests.at(-1).url,'/api/drive/next_trial');
 assert.equal(n.requests.at(-1).body.trial_token,'trial-old');assert.equal(n.requests.at(-1).body.bench_ready,true);checks++;
 n.pending.shift().resolve();await flush();n.intervals[70]();await flush();
 assert.equal(n.countMotion(),1);assert.equal(n.el('drive-next').hidden,true);checks++;
 n.key('keyup');n.key('keydown');await flush();assert.equal(n.countMotion(),1);checks++;
 n.setStatus({mode:'enabled',can_start_next_round:false,release_required:false});await n.poll();await flush();
 n.key('keyup');n.key('keydown');await flush();
 assert.equal(n.countMotion(),2);assert.equal(n.requests.at(-1).body.trial_token,'trial-new');checks++;
 // The boot page needs separate ESC-off and ESC-on confirmations. A previous
 // checkbox/token cannot cross the baseline step or send motion at startup.
 const b=await setup(true,{ground_held_trial:false,ground_short_trial:true,
  mode:'disabled',hardware_output:false,boot_test:true,boot_test_phase:'esc_off',setup_token:'off-token'});
 assert.equal(b.el('drive-enable').textContent,'准备测试');
 assert.equal(b.el('bench-confirm-text').textContent.includes('电调已关闭'),true);checks++;
 b.key('keydown');await flush();b.intervals[70]();await flush();assert.equal(b.requests.length,0);checks++;
 b.el('bench-ready').checked=true;await b.el('drive-enable').onclick();await flush();
 assert.equal(b.requests.at(-1).url,'/api/drive/enable');assert.equal(b.requests.at(-1).body.setup_token,'off-token');checks++;
 b.setStatus({mode:'starting',boot_test_phase:'baseline'});await b.poll();await flush();
 assert.equal(b.el('drive-enable').disabled,true);assert.equal(b.el('bench-ready').disabled,true);checks++;
 b.setStatus({mode:'disabled',boot_test_phase:'esc_on',setup_token:'on-token'});await b.poll();await flush();
 assert.equal(b.el('bench-ready').checked,false);
 assert.equal(b.el('bench-confirm-text').textContent.includes('车轮静止'),true);checks++;
 b.el('bench-ready').checked=true;await b.el('drive-enable').onclick();await flush();
 assert.equal(b.requests.at(-1).body.setup_token,'on-token');assert.equal(b.countMotion(),0);checks++;
 // Long control has no misleading finite countdown, and reverse/turn keys
 // remain available with physical keyboard codes (including IME key names).
 const l=await setup(true,{ground_continuous_trial:true,test_session_s:null,motion_hold_max_s:null,
  reverse_available:true,session_remaining_s:null});
 assert.equal(l.el('drive-title').textContent,'5G 长距离遥控');checks++;
 assert.equal(l.el('drive-time').textContent.includes('不设轮次'),true);checks++;
 assert.equal(l.el('drive-limits').textContent.includes('W/S'),true);checks++;
 l.events.keydown({code:'KeyS',key:'Process',repeat:false,target:{tagName:'DIV'},preventDefault(){}});await flush();
 assert.equal(l.requests.at(-1).body.motor,'reverse');checks++;
 l.events.keydown({code:'ArrowRight',key:'ArrowRight',repeat:false,target:{tagName:'DIV'},preventDefault(){}});await flush();
 assert.equal(l.requests.at(-1).body.steering,'right');checks++;
 l.events.keydown({code:'Escape',key:'Escape',repeat:false,target:{tagName:'DIV'},preventDefault(){}});await flush();
 assert.equal(l.requests.at(-1).url,'/api/drive/emergency');assert.equal(l.el('drive-action').textContent,'已急停');checks++;
 const lb=await setup(true,{mode:'disabled',hardware_output:false,boot_test:true,boot_test_phase:'esc_off',
  ground_continuous_trial:true,setup_token:'long-off-token'});
 assert.equal(lb.el('drive-speed-note').textContent.includes('不设轮次'),true);checks++;
 const p=await setup(false,{mode:'disabled',hardware_output:false,boot_test:true,boot_test_phase:'esc_off',
  test_mode:'driving',setup_token:'pwm-off-token',steering_pwm_available:false,motor_available:true});
 p.el('test-mode').value='steering_pwm';await p.el('test-mode').onchange();await flush();
 assert.equal(p.requests.at(-1).url,'/api/drive/select_mode');
 assert.equal(p.requests.at(-1).body.test_mode,'steering_pwm');checks++;
 p.setStatus({test_mode:'steering_pwm',motor_available:false,steering_pwm_available:true,setup_token:'pwm-mode-token'});
 await p.poll();await flush();
 assert.equal(p.el('drive-enable').textContent,'准备 S3 调试');
 assert.equal(p.el('bench-confirm-text').textContent.includes('电调已关闭'),true);checks++;
 assert.equal(p.el('pwm-slider').disabled,true);assert.equal(p.countMotion(),0);checks++;
 p.setStatus({boot_test_phase:'steering_ready',setup_token:'pwm-ready-token'});await p.poll();await flush();
 assert.equal(p.el('drive-enable').textContent,'启用 S3 调试');
 assert.equal(p.el('bench-confirm-text').textContent.includes('电调已关闭'),true);checks++;
 p.setStatus({boot_test_phase:'ready',mode:'enabled',hardware_output:true,release_required:false,steering_pulse_us:1715});
 await p.poll();await flush();
 assert.equal(p.el('pwm-slider').disabled,false);assert.equal(p.el('drive-title').textContent,'S3 实时 PWM 调试');checks++;
 await p.el('pwm-slider').oninput({target:{value:'1700'}});await flush();
 assert.equal(p.requests.at(-1).url,'/api/drive/steering_pwm');
 assert.equal(p.requests.at(-1).body.pulse_us,1700);assert.equal(p.requests.at(-1).body.setup_token,'pwm-ready-token');checks++;
 p.events.keydown({code:'Escape',key:'Escape',repeat:false,target:{tagName:'DIV'},preventDefault(){}});await flush();
 assert.equal(p.requests.at(-1).url,'/api/drive/emergency');assert.equal(p.countMotion(),0);checks++;
 // Cellular RTT must not serialize the next fresh input behind the reply.
 const c=await setup(true,{ground_continuous_trial:true});c.key('keydown');await flush();
 c.setTime(70);c.intervals[70]();await flush();
 assert.equal(c.countMotion(),2);assert.equal(c.getMaxInFlight(),2);checks++;
 c.setTime(140);c.intervals[70]();await flush();assert.equal(c.countMotion(),2);checks++;
 c.setTime(170);c.pending.shift().resolve();await flush();
 assert.equal(c.countMotion(),3);assert.equal(c.requests.at(-1).at,170);checks++;
 c.key('keyup');await flush();assert.equal(c.requests.at(-1).url,'/api/drive/stop');
 while(c.pending.length)c.pending.shift().resolve();await flush();c.intervals[70]();await flush();
 assert.equal(c.countMotion(),3);assert.equal(c.el('drive-action').textContent.startsWith('停车'),true);checks++;
 // Newer steering replies win; delayed and rejected old sequences are not replayed.
 const o=await setup(true,{ground_continuous_trial:true});o.key('keydown');await flush();
 o.events.keydown({code:'KeyA',key:'a',repeat:false,target:{tagName:'DIV'},preventDefault(){}});await flush();
 o.pending.splice(1,1)[0].resolve();await flush();
 assert.equal(o.el('drive-action').textContent.includes('左转'),true);
 o.pending.shift().resolve();await flush();
 assert.equal(o.el('drive-action').textContent.includes('左转'),true);checks++;
 o.setTime(70);o.intervals[70]();await flush();o.setTime(140);o.intervals[70]();await flush();
 o.pending.shift().reject('old or invalid command sequence');await flush();
 assert.equal(o.el('drive-feedback').textContent.includes('控制请求失败'),false);checks++;
 o.events.keydown({code:'Space',key:' ',repeat:false,target:{tagName:'DIV'},preventDefault(){}});await flush();
 const oc=o.countMotion();while(o.pending.length)o.pending.shift().resolve();await flush();o.intervals[70]();
 assert.equal(o.countMotion(),oc);assert.equal(o.el('drive-action').textContent,'已急停');checks++;
 // A new press waits for all old motion requests before its release handshake.
 const q=await setup(true,{ground_continuous_trial:true});q.key('keydown');await flush();q.intervals[70]();await flush();
 q.key('keyup');await flush();q.key('keydown');await flush();assert.equal(q.countMotion(),2);
 q.pending.shift().resolve();await flush();assert.equal(q.countMotion(),2);
 q.pending.shift().resolve();await flush();assert.equal(q.countMotion(),3);
 assert.equal(q.requests.at(-2).body.motor,'stop');checks++;
 // One failed in-flight request latches the UI; a later success cannot re-arm it.
 const f=await setup(true,{ground_continuous_trial:true});f.key('keydown');await flush();f.intervals[70]();await flush();
 f.pending.shift().reject();await flush();f.pending.shift().resolve();await flush();f.intervals[70]();await flush();
 assert.equal(f.countMotion(),2);assert.equal(f.el('drive-feedback').textContent.includes('控制请求失败'),true);checks++;
 process.stdout.write(JSON.stringify({checks_passed:checks,hardware_output:false,simulated:true})+'\n');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
