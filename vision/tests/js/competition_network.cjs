// Real shipped client; all network, DOM and outputs are simulated.
const fs=require('node:fs'),assert=require('node:assert/strict');
const {createCompetitionClient}=require(process.argv[2]);
const input=JSON.parse(fs.readFileSync(0,'utf8')),nodes=new Map(),all=[],requests=[],windowEvents={},documentEvents={};
const existing=fs.readFileSync(__dirname+'/competition_static.cjs','utf8');
eval(existing.slice(existing.indexOf('function element('),existing.indexOf('const document=')));
const document={hidden:false,hasFocus:()=>true,getElementById:id=>nodes.get(id),createElement:tag=>element(tag),
 querySelectorAll:s=>all.filter(n=>s==='[data-direction]'?'direction'in n.dataset:'stage'in n.dataset),
 addEventListener:(k,fn)=>(documentEvents[k]??=[]).push(fn)};
const window={addEventListener:(k,fn)=>(windowEvents[k]??=[]).push(fn)};
let rev=0,round=0,failPoll=false,failHeartbeat=false,deferStops=false,deferHeartbeats=false,deferEnter=false;
let loseEnter=false,rejectEndedHeartbeat=false,deferPoll=false;
const polls=[];
const stopReplies=[],heartbeats=[],enterReplies=[];
const stamp=s=>({...structuredClone(s),race_flow_status_s:++rev});
const manual=()=>stamp({application_mode:'race',control_session_id:'session',mode:'enabled',boot_test_phase:'ready',
 setup_token:'setup',trial_token:'manual-token',manual_owner_client:'page',owner_client:'page',
 race_flow:{active:false,stage:'idle',can_enter:true,readiness:{ready:true,missing:[]}},parking_payment:{candidates:[]}});
let server=manual();
const response=(data,ok=true)=>({ok,json:async()=>structuredClone(data)});
const http=async(url,options={})=>{
 if(options.method!=='POST'){
  if(failPoll)throw new TypeError('one status timeout');
  if(deferPoll){const old=structuredClone(server);return new Promise(resolve=>polls.push(()=>resolve(response(old))));}
  return response(server);
 }
 const action=url.split('/').at(-1),body=JSON.parse(options.body);requests.push({action,body});
 assert.equal(options.headers['X-Drive-Key'],'key');
 if(action==='stop'&&deferStops){const old=structuredClone(server);return new Promise(resolve=>stopReplies.push(()=>resolve(response(old))));}
 if(action==='race_enter'){
  server=stamp({...server,race_flow:{active:true,owner_client:'page',run_token:'auto-'+(++round),stage:'handoff',can_enter:false}});
  const accepted=structuredClone(server);
  if(loseEnter){loseEnter=false;throw new TypeError('accepted enter reply lost');}
  if(deferEnter)return new Promise(resolve=>enterReplies.push(()=>resolve(response(accepted))));
  return response(accepted);
 }
 if(action==='race_page_heartbeat'){
  if(rejectEndedHeartbeat){rejectEndedHeartbeat=false;server=manual();return {...response({error:'round ended'},false),status:400};}
  assert(Number.isInteger(body.heartbeat_sequence));assert(!('race_sequence' in body));
  if(failHeartbeat){failHeartbeat=false;throw new TypeError('one heartbeat reply lost');}
  const ack=()=>response({heartbeat_ack:true,accepted:true,heartbeat_sequence:body.heartbeat_sequence,page_lease_s:2});
  if(deferHeartbeats)return new Promise(resolve=>heartbeats.push(()=>resolve(ack())));
  return ack();
 }
 if(action==='race_cancel'||action==='emergency'){server=manual();return response(server);}
 if(action==='stop')return response(server);
 throw new Error('unexpected action '+action);
};
const api=createCompetitionClient({document,window,fetch:http,bootstrap:{key:'key',session:'session'},clientId:'page',
 setTimeout:()=>1,clearTimeout(){},setInterval(){},navigator:{getGamepads:()=>[]}});
const flush=async()=>{for(let i=0;i<6;i++)await new Promise(setImmediate);};
(async()=>{
 api.start();await flush();assert.equal(requests.length,0);
 deferStops=true;
 const stopping=api.stopOwned(false,'key_release');await flush();assert.equal(stopReplies.length,1);
 assert.equal(requests.at(-1).body.control_scope,'manual');
 await api.enterAuto();assert(api.inspect().status.race_flow.active);
 const token=api.inspect().status.race_flow.run_token;
 await api.pulse();assert.equal(requests.at(-1).action,'race_page_heartbeat','old pending manual stop does not block supervision');
 stopReplies.shift()();await stopping;
 assert.equal(api.inspect().status.race_flow.run_token,token,'late manual reply cannot replace the autonomous status');
 deferStops=false;

 deferHeartbeats=true;
 const first=api.pulse(),second=api.pulse();await flush();assert.equal(heartbeats.length,2);
 const count=requests.length;await api.pulse();assert.equal(requests.length,count,'request count is bounded');
 heartbeats.pop()();await second;heartbeats.pop()();await first;
 assert.equal(api.inspect().status.race_flow.run_token,token,'ACKs do not replace state');
 deferHeartbeats=false;
 failPoll=true;await api.poll();
 assert.equal(nodes.get('exit-auto').disabled,false,'stop remains available during a status timeout');
 await api.pulse();assert.equal(requests.at(-1).action,'race_page_heartbeat');
 failHeartbeat=true;await api.pulse();await api.pulse();
 assert(!requests.some(r=>r.action==='race_cancel'),'isolated status/heartbeat errors do not cancel autonomous driving');
 failPoll=false;await api.poll();

 deferHeartbeats=true;const late=api.pulse();await flush();
 await api.stopOwned(false,'exit_auto',false,true);
 assert.equal(requests.at(-1).action,'race_cancel');assert.equal(requests.at(-1).body.run_token,token);
 assert.equal(requests.at(-1).body.control_scope,'race');
 heartbeats.shift()();await late;
 assert(!api.inspect().status.race_flow.active,'late heartbeat cannot revive a stopped round');
 deferHeartbeats=false;

 // The HTTP enter reply may lag behind the independent status snapshot.
 deferEnter=true;const entering=api.enterAuto();await flush();assert.equal(enterReplies.length,1);
 await api.poll();await api.pulse();
 assert.equal(requests.at(-1).action,'race_page_heartbeat','heartbeat bypasses pending start response');
 await api.stopOwned(true,'emergency_button');
 const stopped=api.inspect().status;
 enterReplies.shift()();await entering;
 assert.strictEqual(api.inspect().status,stopped,'late enter reply cannot undo emergency');
 assert(!api.inspect().status.race_flow.active);

 // Operator stop before the handoff reply/status must not be misclassified
 // as an old automatic manual-key release and ignored by the server.
 const unacknowledged=api.enterAuto();await flush();assert.equal(enterReplies.length,1);
 await api.stopOwned(false,'stop_button');assert.equal(requests.at(-1).action,'emergency');
 enterReplies.shift()();await unacknowledged;
 assert(!api.inspect().status.race_flow.active);
 deferEnter=false;
 // An older in-flight status cannot be used as the timeout reconciliation.
 deferPoll=true;const oldPoll=api.poll();await flush();
 loseEnter=true;const beforeEnter=requests.filter(r=>r.action==='race_enter').length;
 const lost=api.enterAuto();await flush();
 deferPoll=false;polls.shift()();await oldPoll;await lost;
 assert(api.inspect().status.race_flow.active,'a fresh poll discovers accepted handoff');
 assert.equal(requests.filter(r=>r.action==='race_enter').length,beforeEnter+1,'never retransmit start');
 rejectEndedHeartbeat=true;await api.pulse();await flush();
 assert(!nodes.get('enter-auto').disabled,'ended-run 400 must not permanently poison the page');
 server=stamp({...server,race_flow:{...server.race_flow,manual_handoff_ready:false}});await api.poll();
 assert(nodes.get('enter-auto').disabled,'missing prepared manual executor blocks handoff');
 server=stamp({...server,race_flow:{...server.race_flow,manual_handoff_ready:true}});await api.poll();
 assert(!nodes.get('enter-auto').disabled);
 console.log(JSON.stringify({hardware_output:false,checks:17}));
})().catch(error=>{console.error(error);process.exitCode=1});
