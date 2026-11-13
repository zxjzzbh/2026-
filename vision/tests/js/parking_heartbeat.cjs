const fs=require('node:fs'),assert=require('node:assert/strict');
const {createDebugClient}=require(process.argv[2]);
const input=JSON.parse(fs.readFileSync(0,'utf8')),nodes=new Map(),events={},documentEvents={},requests=[];
const harness=fs.readFileSync(__dirname+'/debug_static.cjs','utf8');
// Reuse the existing offline DOM so this exercises production client code.
eval(harness.slice(harness.indexOf('function element('),harness.indexOf('let rev=0')));
let rev=0,failPoll=false,failHeartbeat=false,pendingStart=null,deferHeartbeat=false;
const pending=[];
const stamp=s=>({...structuredClone(s),application_status_s:++rev});
let status=stamp({application_mode:'debug',control_session_id:'session',setup_token:'setup',trial_token:'trial',
 parking_bay:{can_observe:true,active:false,settings:{}},parking_trial:{can_prepare:true,can_start:false},crosswalk_trial:{can_prepare:true},traffic_trial:{can_prepare:true}});
const response=(s,ok=true)=>({ok,json:async()=>structuredClone(s)});
const ack=seq=>response({heartbeat_ack:true,accepted:true,heartbeat_sequence:seq,page_lease_s:2});
const http=async(url,options={})=>{
 if(options.method!=='POST'){if(failPoll)throw new TypeError('synthetic slow status');return response(status);}
 const action=url.split('/').at(-1),body=JSON.parse(options.body);requests.push({action,body});
 if(action==='parking_trial_prepare'){
  status.parking_trial={active:true,owner_client:'page',run_token:'token',phase:'ready',can_start:true,can_request_start:true};
  status=stamp(status);return response(status);
 }
 if(action==='parking_page_heartbeat'){
  assert.equal(body.run_token,'token');assert(Number.isInteger(body.heartbeat_sequence));assert(!('trial_sequence'in body));
  if(failHeartbeat){failHeartbeat=false;throw new TypeError('one reply lost');}
  if(deferHeartbeat)return new Promise(resolve=>pending.push(()=>resolve(ack(body.heartbeat_sequence))));
  return ack(body.heartbeat_sequence);
 }
 if(action==='parking_trial_start')return new Promise(resolve=>{pendingStart=()=>{status=stamp(status);resolve(response(status));};});
 if(action==='parking_trial_cancel'||action==='emergency'){
  status.parking_trial={active:false,phase:'cancelled',can_prepare:true};status=stamp(status);return response(status);
 }
 throw new Error('unexpected control '+action);
};
// eval above declares document/window lexically, so create them here from
// the same element table and event maps.
const document={hidden:false,getElementById:id=>nodes.get(id),createElement:tag=>element(tag),addEventListener:(k,fn)=>(documentEvents[k]??=[]).push(fn)};
const window={addEventListener:(k,fn)=>(events[k]??=[]).push(fn)};
const api=createDebugClient({document,window,fetch:http,bootstrap:{key:'key',session:'session'},clientId:'page',setTimeout:()=>1,clearTimeout(){},setInterval(){}});
const flush=async()=>{for(let i=0;i<6;i++)await new Promise(setImmediate);};
(async()=>{
 api.init();await flush();assert(api.choose(input.mode));
 for(const id of ['confirm-esc-off','confirm-fb'])nodes.get(id).checked=true;
 api.render();await api.prepare();
 deferHeartbeat=true;const first=api.pulse(),second=api.pulse();await flush();
 assert.equal(pending.length,2,'two independent lightweight requests may be in flight');
 const count=requests.length;await api.pulse();assert.equal(requests.length,count,'pending requests remain bounded');
 pending.pop()();await second;pending.pop()();await first;
 assert(api.inspect().status.parking_trial.active,'reordered ACKs never replace full status');
 deferHeartbeat=false;
 for(const id of ['confirm-on-static','confirm-field'])nodes.get(id).checked=true;
 api.render();const starting=api.startTask();await flush();assert(pendingStart);
 await api.pulse();assert.equal(requests.at(-1).action,'parking_page_heartbeat','heartbeat bypasses slow control-response queue');
 pendingStart();await starting;
 failPoll=true;await api.poll();assert.equal(api.inspect().online,false);
 const cancellations=requests.filter(r=>r.action==='parking_trial_cancel').length;
 await api.pulse();assert.equal(requests.at(-1).action,'parking_page_heartbeat','status failure does not suppress supervision');
 failHeartbeat=true;await api.pulse();await api.pulse();
 assert.equal(requests.filter(r=>r.action==='parking_trial_cancel').length,cancellations,'one network error must not cancel the task');
 deferHeartbeat=true;const late=api.pulse();await flush();assert(pending.length);
 await api.stop();assert.equal(requests.at(-1).action,'parking_trial_cancel','stop bypasses in-flight heartbeat');
 pending.pop()();await late;assert(!api.inspect().status.parking_trial.active,'late ACK cannot revive stopped task');
 console.log(JSON.stringify({hardware_output:false,checks:8}));
})().catch(error=>{console.error(error);process.exitCode=1});
