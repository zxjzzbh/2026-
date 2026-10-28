// Executes the actual composed client against a strict local DOM/API harness.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const input=JSON.parse(fs.readFileSync(0,'utf8'));
const nodes=new Map(),all=[],windowEvents={},documentEvents={},intervals=[],requests=[];
function element(tag,attrs={}){
 const classes=new Set();return {id:attrs.id||'',alt:attrs.alt||'',tagName:tag.toUpperCase(),dataset:Object.fromEntries(Object.entries(attrs).filter(([k])=>k.startsWith('data-')).map(([k,v])=>[k.slice(5),v])),
  disabled:'disabled'in attrs,hidden:'hidden'in attrs,checked:false,value:attrs.value||'',textContent:'',style:{},children:[],attributes:attrs,
  classList:{add(v){classes.add(v)},remove(v){classes.delete(v)},toggle(v,b){b?classes.add(v):classes.delete(v)},contains(v){return classes.has(v)}},
  addEventListener(event,fn){this['on'+event]=fn},setAttribute(k,v){this.attributes[k]=v},getAttribute(k){return this.attributes[k]??null},removeAttribute(k){delete this.attributes[k]},
  replaceWith(other){nodes.set(this.id,other)},replaceChildren(...c){this.children=c},append(...c){this.children.push(...c)},focus(){},setPointerCapture(){},showModal(){this.open=true},close(){this.open=false},checkValidity(){return true},reportValidity(){}};
}
for(const [,tag,body] of input.html.matchAll(/<([a-zA-Z][\w-]*)\b([^>]*)>/g)){
 const attrs={};for(const [,k,v] of body.matchAll(/([\w-]+)(?:="([^"]*)")?/g))attrs[k]=v??'';
 const n=element(tag,attrs);all.push(n);if(attrs.id)nodes.set(attrs.id,n);
}
function query(selector){return all.filter(n=>selector.split(',').some(s=>{const m=s.trim().match(/^\[data-([\w-]+)\]$/);return m&&m[1]in n.dataset}));}
const document={hidden:false,hasFocus:()=>true,getElementById:id=>nodes.get(id)||null,querySelectorAll:query,
 createElement:tag=>element(tag),addEventListener:(k,f)=>{(documentEvents[k]??=[]).push(f)}};
const window={addEventListener:(k,f)=>{(windowEvents[k]??=[]).push(f)},location:{reload(){}}};
let version=1;
const base=()=>({control_session_id:'session',setup_token:'setup',trial_token:'trial',mode:'disabled',reason:'ready',
 boot_test:true,boot_test_phase:'esc_off',test_mode:'driving',worker_available:true,worker_started:false,pending_hardware_start:false,
 hardware_output:false,ground_continuous_trial:true,motor_pulse_us:1500,steering_pulse_us:1610,steering_center_us:1610,
 race_flow_status_s:version++,race_flow:{active:false,can_enter:true,stage:'idle',parking_missing:['not verified']},
 parking_payment:{motion_locked:false,can_scan:true,can_begin:false,candidates:[]},payment_status_s:version++});
let status=base(),deferHeartbeat=false,pendingHeartbeat=null,getFailure=false;
const context={document,window,console,Map,Set,Uint8Array,Array,Number,Math,JSON,Boolean,Promise,AbortController,
 crypto:require('node:crypto').webcrypto,performance:{now:()=>0},navigator:{getGamepads:()=>[]},
 sessionStorage:{getItem:()=>null,setItem(){}},setInterval:(f,ms)=>{intervals.push({f,ms});return intervals.length},
 setTimeout:()=>1,clearTimeout(){},clearInterval(){},fetch:async(url,options={})=>{
  if(options.method!=='POST'&&getFailure)throw new Error('network unavailable');
  if(options.method==='POST'){
   const body=JSON.parse(options.body),action=url.split('/').at(-1);requests.push({action,body});
   assert.equal(options.headers['X-Drive-Key'],'key');
   if(action==='race_enter')status={...base(),race_flow:{active:true,owner_client:body.client,run_token:'race-token',stage:'arming',stage_label:'准备',can_enter:false}};
   if(action==='enable')status={...base(),mode:'enabled',boot_test_phase:'ready',worker_started:true};
   if(action==='race_keepalive'&&deferHeartbeat){const old=structuredClone(status);return new Promise(resolve=>{pendingHeartbeat=()=>resolve({ok:true,json:async()=>old})});}
   if(['emergency','race_cancel','race_exit','finish_test'].includes(action))status=base();
   if(action==='race_report_paid')status={...status,race_flow_status_s:version++,race_flow:{...status.race_flow,stage:'complete',can_report_paid:false,operator_reported_paid:true}};
  }
  return {ok:true,json:async()=>structuredClone(status)};
 }};
vm.createContext(context);
let source=input.script.replace(/^<script>/,'').replace(/<\/script>$/,'').replaceAll('__DRIVE_KEY__','"key"').replaceAll('__DRIVE_SESSION__','"session"');
const end=source.lastIndexOf('})();');
source=source.slice(0,end)+'globalThis.testClient={render,stop,client,press,release,poll};'+source.slice(end);
vm.runInContext(source,context);
const flush=async()=>{for(let i=0;i<4;i++)await new Promise(setImmediate)};
const snapshot=s=>({...s,race_flow_status_s:version++,payment_status_s:version++});
(async()=>{
 await flush();assert.equal(requests.length,0,'opening the page must not POST controls');
 for(const f of windowEvents.blur||[])f();await flush();assert.equal(requests.length,0,'passive page must not stop another owner');
 const api=context.testClient;
 getFailure=true;await api.poll();assert.equal(nodes.get('competition-enter').disabled,true);
 await nodes.get('competition-enter').onclick();assert.equal(requests.length,0,'offline status must not allow auto entry');
 api.render(status);assert.equal(nodes.get('competition-enter').disabled,true,'old cached render must not clear a network failure');
 getFailure=false;await api.poll();assert.equal(nodes.get('competition-enter').disabled,false);
 status=snapshot({...base(),race_flow:{active:true,owner_client:'other',run_token:'other-token',stage:'cones'}});api.render(status);
 assert.equal(nodes.get('competition-exit').disabled,true);
 assert.equal(nodes.get('auto-section').dataset.active,'true','observer still sees the auto preview');
 for(const x of intervals.filter(x=>x.ms===100))await x.f();
 for(const f of windowEvents.blur||[])f();await flush();assert.equal(requests.length,0,'unowned automatic run is observation only');
 nodes.get('competition-emergency').onclick();await flush();assert.equal(requests.at(-1).action,'emergency');
 status=snapshot(base());api.render(status);nodes.get('bench-ready').checked=false;
 await nodes.get('drive-enable').onclick();await flush();
 if(requests.at(-1).action!=='enable')nodes.get('bench-ready').checked=true;
 if(requests.at(-1).action!=='enable'){await nodes.get('drive-enable').onclick();await flush();}
 api.press('key:KeyW','forward');await flush();
 assert(requests.some(r=>r.action==='command'&&r.body.motor==='forward'),'manual forward must keep the original protocol');
 assert.equal(nodes.get('competition-enter').disabled,true,'manual output must be released before handoff');
 api.release('key:KeyW');await flush();assert.equal(requests.at(-1).action,'stop');
 await nodes.get('competition-release').onclick();await flush();assert.equal(requests.at(-1).action,'finish_test');
 status=snapshot(base());api.render(status);assert.equal(nodes.get('competition-enter').disabled,false);
 await nodes.get('competition-enter').onclick();await flush();
 assert.equal(requests.at(-1).action,'race_enter');assert.equal(requests.at(-1).body.in_switch_zone,true);
 assert.equal(nodes.get('drive-enable').disabled,true);assert.equal(nodes.get('competition-exit').disabled,false);
 deferHeartbeat=true;const pulse=intervals.find(x=>x.ms===100).f();await flush();assert(pendingHeartbeat);
 nodes.get('competition-exit').onclick();await flush();assert.equal(requests.at(-1).action,'race_cancel');
 pendingHeartbeat();await pulse;await flush();assert.equal(nodes.get('competition-exit').disabled,true,'late heartbeat must not revive cancelled UI');
 deferHeartbeat=false;
 status=snapshot({...base(),worker_available:false,parking_trial:{active:true}});api.render(status);
 nodes.get('competition-emergency').onclick();await flush();assert.equal(requests.at(-1).action,'emergency','standalone tasks must not bypass global emergency');
 status=snapshot({...base(),plan_b:{active:true}});api.render(status);
 assert.equal(nodes.get('drive-enable').disabled,true,'existing Plan B owns its output');
 const beforeBlur=requests.length;for(const f of windowEvents.blur||[])f();await flush();assert.equal(requests.length,beforeBlur);
 status=snapshot({...base(),race_flow:{active:true,owner_client:api.client,run_token:'pay-token',stage:'payment',can_report_paid:true,payment_remaining_s:12}});api.render(status);
 assert.equal(nodes.get('payment-begin').hidden,true);assert.equal(nodes.get('payment-report').hidden,true);
 assert.equal(nodes.get('payment-time').textContent,'12.0 秒');
 assert(nodes.get('payment-state').textContent.includes('付款'));
 assert.equal(nodes.get('competition-paid').hidden,false);await nodes.get('competition-paid').onclick();await flush();
 assert.equal(requests.at(-1).action,'race_report_paid');assert.equal(requests.at(-1).body.operator_paid,true);
 assert(!requests.some(r=>['parking_trial_start','traffic_start','crosswalk_start'].includes(r.action)));
 if(input.cameras){
  const pending=[],timers=[];document.createElement=tag=>{const n=element(tag);pending.push(n);return n};
  context.setTimeout=(f,ms)=>{timers.push({f,ms});return timers.length};
  vm.runInContext(input.cameras.replace(/^<script>/,'').replace(/<\/script>$/,''),context);
  const old=nodes.get('camera-primary'),loaded=pending[0];assert.equal(nodes.get('camera-primary'),old);
  loaded.onload();assert.equal(nodes.get('camera-primary'),loaded);assert.equal(loaded.hidden,false);
  timers.at(-1).f();assert.equal(nodes.get('camera-primary'),loaded,'keep decoded pixels while downloading the next frame');
  pending.at(-1).onerror();assert.equal(loaded.hidden,true,'do not show a frozen image after a failed read');
 }
 console.log(JSON.stringify({hardware_output:false,checks:'no auto POST, ownership, handoff, stale reply, global stop, payment'}));
})().catch(e=>{console.error(e);process.exitCode=1});
