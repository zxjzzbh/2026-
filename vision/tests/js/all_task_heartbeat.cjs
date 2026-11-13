const fs=require('node:fs'),assert=require('node:assert/strict');
const {createDebugClient}=require(process.argv[2]);
const input=JSON.parse(fs.readFileSync(0,'utf8'));
const harness=fs.readFileSync(__dirname+'/debug_static.cjs','utf8');
async function exercise(task,key,endpoint){
 const nodes=new Map(),events={},documentEvents={};
 eval(harness.slice(harness.indexOf('function element('),harness.indexOf('let rev=0')));
 const document={hidden:false,getElementById:id=>nodes.get(id),createElement:tag=>element(tag),addEventListener:(k,fn)=>(documentEvents[k]??=[]).push(fn)};
 const window={addEventListener:(k,fn)=>(events[k]??=[]).push(fn)};
 let rev=1,failStatus=false,failBeat=false,serverFailure=false,defer=false;
 const pending=[],requests=[];
 const state={application_mode:'debug',control_session_id:'session',setup_token:'setup',application_status_s:rev,
   crosswalk_trial:{},traffic_trial:{},cone_trial:{avoidance:{}},parking_trial:{},parking_bay:{}};
 const row=()=>key==='cones'?state.cone_trial.avoidance:state[key];
 const response=(value,status=200)=>({ok:status<400,status,json:async()=>structuredClone(value)});
 const http=async(url,opts={})=>{
  if(opts.method!=='POST'){if(failStatus)throw new TypeError('status response lost');return response(state);}
  const action=url.split('/').at(-1),body=JSON.parse(opts.body);requests.push({action,body});
  if(action===endpoint){
   assert.equal(body.run_token,'current');assert(!('trial_sequence' in body));
   if(failBeat){failBeat=false;throw new TypeError('heartbeat ACK lost');}
   if(serverFailure){serverFailure=false;return response({error:'temporary upstream error'},503);}
   const reply=()=>response({heartbeat_ack:true,heartbeat_sequence:body.heartbeat_sequence,accepted:true,page_lease_s:3});
   if(defer)return new Promise(resolve=>pending.push(()=>resolve(reply())));
   return reply();
  }
  if(action.endsWith('_cancel')||action==='emergency'){
   Object.assign(row(),{active:false});state.application_status_s=++rev;return response(state);
  }
  throw new Error('Unexpected motion/control request: '+action);
 };
 const api=createDebugClient({document,window,fetch:http,clientId:'page',bootstrap:{key:'key',session:'session'},setTimeout:()=>1,clearTimeout(){},setInterval(){}});
 api.init();await new Promise(setImmediate);assert(api.choose(task));
 Object.assign(row(),{active:true,owner_client:'page',run_token:'current',phase:'ready'});
 state.application_status_s=++rev;api.applyStatus(structuredClone(state));
 failStatus=true;await api.poll();await api.pulse();
 assert.equal(requests.length,1,'a failed status request must not issue stop');
 failBeat=true;await api.pulse();await api.pulse();
 serverFailure=true;await api.pulse();await api.pulse();
 assert(!api.inspect().stale,'temporary network/server errors must keep retrying');
 assert(requests.every(r=>r.action===endpoint));
 defer=true;const a=api.pulse(),b=api.pulse();await new Promise(setImmediate);
 assert.equal(pending.length,2);const n=requests.length;await api.pulse();assert.equal(requests.length,n);
 await api.stop();assert(!row().active,'explicit stop bypasses pending heartbeat requests');
 pending.pop()();pending.pop()();await Promise.all([a,b]);
 const stopped=requests.length;await api.pulse();assert.equal(requests.length,stopped,'late replies cannot restart a stopped task');
}
(async()=>{
 for(const spec of [['crosswalk','crosswalk_trial','crosswalk_page_heartbeat'],['traffic','traffic_trial','traffic_page_heartbeat'],
                    ['cone','cones','cone_page_heartbeat'],['visual','parking_trial','parking_page_heartbeat']])await exercise(...spec);
 console.log(JSON.stringify({hardware_output:false,tasks:4}));
})().catch(e=>{console.error(e);process.exit(1);});
