// Execute the shipped standalone client with deterministic DOM/network peers.
const fs=require('node:fs'),assert=require('node:assert/strict');
const {createCompetitionClient}=require(process.argv[2]);
const input=JSON.parse(fs.readFileSync(0,'utf8'));
const nodes=new Map(),all=[],windowEvents={},documentEvents={},intervals=[],requests=[];
function element(tag,attrs={}){
 const listeners={};return {tagName:tag.toUpperCase(),dataset:Object.fromEntries(Object.entries(attrs).filter(([k])=>k.startsWith('data-')).map(([k,v])=>[k.slice(5),v])),
 id:attrs.id||'',textContent:'',disabled:'disabled'in attrs,hidden:'hidden'in attrs,checked:false,style:{},children:[],src:'',
 classList:{toggle(){}},setPointerCapture(){},removeAttribute(){},append(...items){this.children.push(...items)},replaceChildren(...items){this.children=items},
 addEventListener(k,fn){(listeners[k]??=[]).push(fn)},async fire(k,arg={}){for(const fn of listeners[k]||[])await fn(arg)}};
}
for(const [,tag,body] of input.html.matchAll(/<([a-zA-Z][\w-]*)\b([^>]*)>/g)){
 const attrs={};for(const [,k,v] of body.matchAll(/([\w-]+)(?:="([^"]*)")?/g))attrs[k]=v??'';
 const node=element(tag,attrs);all.push(node);if(attrs.id)nodes.set(attrs.id,node);
}
const document={hidden:false,hasFocus:()=>true,getElementById:k=>nodes.get(k),createElement:tag=>element(tag),
 querySelectorAll:selector=>all.filter(n=>selector==='[data-direction]'?'direction'in n.dataset:'stage'in n.dataset),
 addEventListener:(k,fn)=>(documentEvents[k]??=[]).push(fn)};
const window={addEventListener:(k,fn)=>(windowEvents[k]??=[]).push(fn)};
let revision=0,deferPulse=false,pendingPulse=null,deferEnter=false,pendingEnter=null,failGet=false,lastSequence=-1;
const snapshot=s=>({...structuredClone(s),race_flow_status_s:++revision});
const base=()=>snapshot({control_session_id:'session',mode:'disabled',boot_test_phase:'esc_off',setup_token:'setup',trial_token:'manual-token',reason:'等待',
 race_flow:{active:false,stage:'idle',can_enter:true,readiness:{ready:true,missing:[]},parking_missing:[]},parking_payment:{can_scan:true,candidates:[]}});
let status=base();
status.speed_percent=15;status.reverse_speed_percent=40;status.last_speed_sequence=-1;status.selected_forward_pulse_us=1575;status.motor_pulse_us=1500;
function response(value,ok=true){return {ok,json:async()=>structuredClone(value)};}
const http=async(url,options={})=>{
 if(options.method!=='POST'){if(failGet)throw new Error('offline');return response(status);}
 const action=url.split('/').at(-1),body=JSON.parse(options.body);requests.push({action,body});assert.equal(options.headers['X-Drive-Key'],'key');
 if(action==='race_enter'){
  if(deferEnter)return new Promise(resolve=>{pendingEnter=()=>{status=snapshot({...status,race_flow:{active:true,owner_client:'page',run_token:'token',stage:'handoff',can_enter:false,readiness:{ready:true,missing:[]}}});resolve(response(status));};});
  status=snapshot({...status,race_flow:{active:true,owner_client:'page',run_token:'token',stage:'handoff',can_enter:false}});
 }
 if(action==='race_keepalive'){
  const process=()=>{if(body.race_sequence<=lastSequence)return response({error:'old sequence'},false);lastSequence=body.race_sequence;return response(snapshot(status));};
  if(deferPulse)return new Promise(resolve=>{pendingPulse=()=>resolve(process());});return process();
 }
 if(action==='race_report_paid'){assert(body.race_sequence>lastSequence);lastSequence=body.race_sequence;status=snapshot({...status,race_flow:{...status.race_flow,stage:'complete',operator_reported_paid:true,can_report_paid:false}});}
 if(['race_cancel','emergency'].includes(action))status=base();
 if(action==='enable')status=snapshot({...status,mode:'enabled',boot_test_phase:'ready',manual_owner_client:'page',manual_speed_adjustable:true,speed_percent:15,reverse_speed_percent:40,last_speed_sequence:-1,selected_forward_pulse_us:1575,motor_pulse_us:1500,reverse_available:false});
 if(action==='speed'){
  assert(body.speed_sequence>status.last_speed_sequence,'speed sequence must advance');
  assert.equal(body.reverse_speed_percent,status.reverse_speed_percent,'forward edit preserves current reverse setting');
  status=snapshot({...status,speed_percent:body.speed_percent,last_speed_sequence:body.speed_sequence,selected_forward_pulse_us:1500+body.speed_percent*5});
 }
 return response(status);
};
const api=createCompetitionClient({document,window,fetch:http,bootstrap:{key:'key',session:'session'},clientId:'page',navigator:{getGamepads:()=>[]},
 setTimeout:()=>1,clearTimeout(){},setInterval:(f,ms)=>{intervals.push({f,ms});return intervals.length;}});
const flush=async()=>{for(let i=0;i<5;i++)await new Promise(setImmediate);};
const active=(stage='cones',token='token',owner='page')=>snapshot({...base(),race_flow:{active:true,owner_client:owner,run_token:token,stage,can_report_paid:stage==='payment',readiness:{ready:true,missing:[]},payment_remaining_s:12}});
(async()=>{
 api.start();await flush();assert.equal(requests.length,0,'opening is read-only');
 assert.equal(nodes.get('forward-speed-slider').value,'15');
 await api.changeSpeed(100);assert.equal(requests.length,0,'disabled speed controls never change the stored setting');
 for(const fn of windowEvents.blur)fn();await flush();assert.equal(requests.length,0,'passive blur is inert');
 status=active('cones','other','other-page');api.applyStatus(status);for(const fn of windowEvents.blur)fn();await flush();
 assert.equal(requests.length,0,'observer cannot cancel another owner');
 await nodes.get('emergency').fire('click');assert.equal(requests.at(-1).action,'emergency','explicit emergency is global');

 status=base();api.applyStatus(status);deferEnter=true;
 const enter=nodes.get('enter-auto').fire('click');await flush();await nodes.get('enter-auto').fire('click');
 assert.equal(requests.filter(x=>x.action==='race_enter').length,1,'double click starts one handoff');
 assert.equal(requests.filter(x=>x.action==='finish_test').length,0,'handoff is one API action');
 pendingEnter();await enter;deferEnter=false;
 assert(nodes.get('handoff-state').textContent.includes('控制'),'handoff has its own status');
 await api.pulse();assert.equal(requests.at(-1).action,'race_keepalive','handoff retains ownership lease');

 status=active('cones');api.applyStatus(status);deferPulse=true;
 const pulse=api.pulse();await flush();assert(pendingPulse);
 status=active('payment');api.applyStatus(status);
 const paid=nodes.get('report-paid').fire('click');await flush();
 assert.notEqual(requests.at(-1).action,'race_report_paid','payment waits behind in-flight heartbeat');
 pendingPulse();await pulse;await paid;await flush();deferPulse=false;
 const actions=requests.map(x=>x.action);assert.equal(actions.at(-1),'race_report_paid');
 assert.equal(api.inspect().status.race_flow.stage,'complete');
 assert(!actions.slice(actions.lastIndexOf('race_enter')+1).includes('race_cancel'),'reordered payment never triggers a false cancel');

 status=active('cones','new-round');api.applyStatus(status);deferPulse=true;
 const oldPulse=api.pulse();await flush();const oldReply=pendingPulse;
 await nodes.get('exit-auto').fire('click');
 status=active('cones','newer-round');api.applyStatus(status);oldReply();await oldPulse;deferPulse=false;
 assert.equal(api.inspect().status.race_flow.run_token,'newer-round','old reply cannot revive or replace a new round');

 status=active('payment','pay');api.applyStatus(status);
 const n=requests.length;for(const fn of windowEvents.blur)fn();await flush();assert.equal(requests.length,n,'WeChat switch preserves payment deadline');
 await nodes.get('emergency').fire('click');assert.equal(requests.at(-1).action,'emergency');

 status=base();status.race_flow.readiness={ready:false,missing:['转向实测缺失','速度反馈缺失']};status.race_flow.parking_missing=['停车地面标定'];
 api.applyStatus(status);assert.equal(nodes.get('enter-auto').disabled,true);
 assert.deepEqual(nodes.get('readiness-missing').children.map(n=>n.textContent),['转向实测缺失','速度反馈缺失','停车地面标定']);
 const blockers=Array.from({length:13},(_,i)=>'实测条件'+i);
 status=base();status.race_flow.readiness={ready:false,missing:blockers,
  groups:Array.from({length:4},(_,i)=>({ready:false,label:'检查分组'+i,items:[{reason:blockers[i]}]}))};
 status.race_flow.parking_missing=blockers;api.applyStatus(status);
 assert.deepEqual(nodes.get('readiness-missing').children.map(n=>n.textContent),blockers,'group headings never inflate concrete blockers');
 assert(nodes.get('readiness-title').textContent.includes('13'),'readiness count follows the 13 actual blockers');
 assert.equal(nodes.get('enter-auto').disabled,true,'display deduplication preserves the full-race gate');

 const throughCones=()=>{const value=base();value.race_flow={...value.race_flow,
  start_readiness:{ready:true,missing:[],scope:'through_cones',parking_ready:false},
  readiness:{ready:false,missing:blockers},parking_missing:blockers};return value;};
 status=throughCones();const beforeFrontStatus=requests.length;api.applyStatus(status);
 assert.equal(nodes.get('enter-auto').disabled,false,'front-stage readiness permits the explicitly supported start scope');
 assert.equal(api.inspect().status.race_flow.readiness.ready,false,'parking readiness is never relabelled as ready');
 assert.deepEqual(nodes.get('readiness-missing').children.map(n=>n.textContent),blockers,'all 13 parking blockers stay visible');
 assert.match(nodes.get('readiness-title').textContent,/停车条件待补.*13.*前段可启动.*两桶避障后停车/);
 await api.poll();await api.pulse();await api.motion();api.render();await flush();
 assert.equal(requests.length,beforeFrontStatus,'status/render/idle timers never prepare or start the vehicle');
 await nodes.get('enter-auto').fire('click');await flush();
 assert.equal(requests.length,beforeFrontStatus+1,'one explicit operator click sends exactly one request');
 assert.equal(requests.at(-1).action,'race_enter');
 assert.equal(requests.at(-1).body.stationary_confirmed,true);
 assert.equal(requests.at(-1).body.in_switch_zone,true);
 assert.equal(requests.at(-1).body.fb_mode_confirmed,true);

 status=throughCones();status.race_flow.can_enter=false;api.applyStatus(status);
 const beforeDeniedStart=requests.length;
 assert.equal(nodes.get('enter-auto').disabled,true,'start readiness cannot bypass backend ownership/cleanup/live gates');
 await api.enterAuto();assert.equal(requests.length,beforeDeniedStart);
 status=throughCones();status.cone_trial={avoidance:{active:true}};api.applyStatus(status);
 assert.equal(nodes.get('enter-auto').disabled,true,'front-stage readiness cannot bypass another task');

 status=base();status.race_flow.start_readiness={ready:false,missing:['configuration_unavailable'],scope:'through_cones',parking_ready:true};
 api.applyStatus(status);
 assert.equal(nodes.get('enter-auto').disabled,true,'explicit new start failure overrides old full readiness true');
 assert.match(nodes.get('readiness-title').textContent,/启动条件未就绪/);
 status=base();status.race_flow.start_readiness={ready:true,missing:[],scope:'full_race',parking_ready:true};api.applyStatus(status);
 assert.equal(nodes.get('enter-auto').disabled,false);
 assert.equal(nodes.get('readiness-title').textContent,'自主驾驶条件已齐备');
 status=base();status.race_flow.readiness={ready:false,missing:blockers};api.applyStatus(status);
 assert.equal(nodes.get('enter-auto').disabled,true,'servers without start_readiness retain the old full-race gate');
 status=base();api.applyStatus(status);failGet=true;await api.poll();assert.equal(nodes.get('enter-auto').disabled,true);failGet=false;await api.poll();

 const readyDisabled=()=>({...base(),boot_test_phase:'ready',mode:'disabled'});
 status=readyDisabled();const beforeReenable=requests.length;api.applyStatus(status);
 assert.equal(nodes.get('prepare-manual').disabled,false,'a reset prepared worker can be explicitly re-enabled');
 assert.equal(nodes.get('prepare-manual').textContent,'电调已打开且静止 · 重新启用遥控');
 await api.poll();api.render();await flush();
 assert.equal(requests.length,beforeReenable,'ready/disabled status refresh never enables the worker automatically');
 await nodes.get('prepare-manual').fire('click');await flush();
 assert.equal(requests.length,beforeReenable+1);
 assert.equal(requests.at(-1).action,'enable');
 assert.equal(requests.at(-1).body.esc_state,'on_static');
 assert.equal(requests.at(-1).body.bench_ready,true);
 for(const mode of ['settling','enabled']){
  status={...readyDisabled(),mode};api.applyStatus(status);
  assert.equal(nodes.get('prepare-manual').disabled,true,'settling/enabled workers cannot be enabled again');
  assert.equal(nodes.get('prepare-manual').textContent,mode==='settling'?'正在启用遥控…':'遥控已准备');
  const before=requests.length;await nodes.get('prepare-manual').fire('click');assert.equal(requests.length,before);
 }
 for(const blocked of ['race','foreign_task','payment']){
  status=readyDisabled();
  if(blocked==='race')status.race_flow={...status.race_flow,active:true,owner_client:'other-page',run_token:'other-owner'};
  if(blocked==='foreign_task')status.cone_trial={avoidance:{active:true,owner_client:'other-page'}};
  if(blocked==='payment')status.parking_payment.motion_locked=true;
  api.applyStatus(status);assert.equal(nodes.get('prepare-manual').disabled,true,'reenable preserves '+blocked+' ownership gate');
  const before=requests.length;await nodes.get('prepare-manual').fire('click');assert.equal(requests.length,before);
 }
 status=base();api.applyStatus(status);
 await nodes.get('prepare-manual').fire('click');api.press('key:KeyW','forward');await flush();
 assert(requests.some(x=>x.action==='command'&&x.body.motor==='forward'),'manual command API preserved');
 api.release('key:KeyW');await flush();assert.equal(requests.at(-1).action,'stop');
 await api.changeSpeed(25);assert.equal(requests.at(-1).action,'speed');
 assert.equal(requests.at(-1).body.speed_percent,25);assert.equal(nodes.get('forward-speed-number').value,'25');
 const accepted=status.last_speed_sequence;
 const oldSpeedReply=snapshot({...status,last_speed_sequence:accepted-1,speed_percent:15,selected_forward_pulse_us:1575});
 assert.equal(api.applyStatus(oldSpeedReply),false,'old speed acknowledgement cannot roll back the slider');
 assert.equal(nodes.get('forward-speed-number').value,'25');
 status=snapshot(status);api.applyStatus(status);
 api.press('key:KeyW','forward');await flush();await api.changeSpeed(0);await flush();
 assert.equal(requests.at(-1).action,'speed');assert.equal(requests.at(-1).body.speed_percent,0);
 const afterZero=requests.length;await api.motion();assert.equal(requests.length,afterZero,'zero throttle clears held input and cannot resume movement');
 assert.equal(nodes.get('forward-speed-slider').value,'0');
 status={...base(),application_mode:'debug'};api.applyStatus(status);
 assert.equal(nodes.get('auto-section').hidden,true);assert.equal(nodes.get('payment-section').hidden,true);
 assert.equal(nodes.get('prepare-manual').disabled,false,'full race readiness cannot block debug manual preparation');
 const beforeDebugEnter=requests.length;await api.enterAuto();await api.pulse();
 assert.equal(requests.length,beforeDebugEnter,'debug manual page cannot enter or renew race mode');
 console.log(JSON.stringify({hardware_output:false,checks:13}));
})().catch(error=>{console.error(error);process.exitCode=1;});
