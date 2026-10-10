// Offline DOM-state regression for the actual traffic render functions.
// Input contains source plus saved field observations; no browser/server/UART.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const data = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const nodes = new Map();
const env = {
  trafficActive:false, trafficOwned:false, trafficToken:null, trafficSettingsRevision:null,
  trafficSaveBusy:false, trafficStartBusy:false, trafficStartError:'', trafficLastStatusAt:-Infinity,
  trafficDirty:false, stale:false, pageSession:'session', client:'page', lastStatus:null,
  heartbeatDue:false, needsRelease:true, allowed:false, turnAvailable:false, motorAvailable:false,
  reverseAvailable:false, latency:0, held:new Map(), gamepadControl:{requireNeutral(){}},
  feedback(){}, expiredPage(){env.expired=true;}, document:{querySelectorAll(){return [];}},
  el(id){
    if(!nodes.has(id))nodes.set(id,{disabled:false,textContent:'',dataset:{},hidden:false,value:''});
    return nodes.get(id);
  },
};
vm.createContext(env);
vm.runInContext(data.source,env);
const observed = [];
for(const s of data.snapshots){
  if(env.acceptTrafficSnapshot(s))env.renderTraffic(s);
  observed.push(env.el('traffic-start').disabled);
}
assert(observed.every(disabled=>disabled===false), 'Prepared button must not follow per-frame recognition changes');
const last=data.snapshots.at(-1);
const running={...last,traffic_status_s:last.traffic_status_s+1,
  traffic_trial:{...last.traffic_trial,phase:'arming',can_request_start:false,can_start:false}};
assert(env.acceptTrafficSnapshot(running));env.renderTraffic(running);
assert(env.el('traffic-start').disabled);
assert(!env.acceptTrafficSnapshot(last), 'Late ready response must not overwrite arming');
assert(env.el('traffic-start').disabled);
assert(!env.acceptTrafficSnapshot({...running,control_session_id:'another-session'}));
assert(env.expired);
console.log(JSON.stringify({replayed_frames:observed.length,button_disabled_transitions:0,
  rejects_out_of_order:true,rejects_expired_page:true,vehicle_commands_sent:false}));
