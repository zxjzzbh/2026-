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
    const snapshot={...status,motor_direction:'forward',motor_pulse_us:1575,
     steering_direction:body.steering??'center',
     steering_pulse_us:body.steering==='left'?1750:body.steering==='right'?1550:1715};
    return new Promise((resolve,reject)=>pending.push({
     resolve(){inFlight--;resolve({ok:true,json:async()=>snapshot});},
     reject(){inFlight--;reject(new Error('late reply failed'));}}));
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
 process.stdout.write(JSON.stringify({checks_passed:checks,hardware_output:false,simulated:true})+'\n');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
