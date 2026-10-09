const assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const source=JSON.parse(fs.readFileSync(0,'utf8')).script;
const flush=()=>new Promise(resolve=>setImmediate(resolve));
async function setup(initial={}){
 const nodes=new Map(),pending=[],requests=[],timers=new Map(),intervals=[];
 const el=id=>{if(!nodes.has(id))nodes.set(id,{value:1650,textContent:'',disabled:false});return nodes.get(id);};
 let s={mode:'enabled',hardware_output:true,steering_pwm_available:true,motor_available:false,
        steering_center_us:1715,steering_pulse_us:1715,release_required:false,...initial};
 let api,stops=0,errors=0,otherInput=false;
 const options={el,request:(action,body)=>{
  requests.push({action,body});
  return new Promise((resolve,reject)=>pending.push({action,body,resolve,reject}));
 },status:()=>({allowed:s.mode==='enabled',otherInput}),
 render:state=>{s=state;api.update(s);},stop:()=>{stops++;api.cancel();s={...s,steering_pwm_active:false};api.update(s);},
 onError:()=>errors++};
 const context={document:{querySelectorAll:()=>[]},setInterval:fn=>intervals.push(fn),
 setTimeout:fn=>{const key=timers.size+1;timers.set(key,fn);return key;},clearTimeout:key=>timers.delete(key),options};
 vm.createContext(context);vm.runInContext(source+';result=createSteeringPWMControls(options);',context);api=context.result;api.update(s);
 return {el,api,pending,requests,timers,intervals,stop:options.stop,errors:()=>errors,stops:()=>stops,
  input:value=>el('pwm-slider').oninput({target:{value}}),setOther:value=>{otherInput=value;},
  reply:()=>{const p=pending.shift();p.resolve({...s,release_required:false,
    steering_pwm_active:p.action==='steering_pwm',steering_pulse_us:p.body.pulse_us??1715});}};
}
(async()=>{
 const h=await setup();assert.equal(h.el('pwm-slider').value,1715);
 h.input(1700);assert.equal(h.requests[0].action,'command');h.reply();await flush();
 assert.equal(h.requests.at(-1).body.pulse_us,1700);
 h.input(1680);h.input(1660);h.intervals[0]();
 assert.equal(h.requests.length,2);h.reply();await flush();
 assert.equal(h.requests.at(-1).body.pulse_us,1660); // latest only
 const before=h.requests.length;h.stop();h.reply();await flush();h.intervals[0]();await flush();
 assert.equal(h.requests.length,before);assert.equal(h.api.isActive(),false);
 const n=await setup();n.input(1700);n.stop();n.reply();await flush();
 assert.equal(n.requests.length,1); // release reply cannot restart tuning
 const t=await setup();t.input(1700);t.reply();await flush();t.timers.values().next().value();
 t.reply();await flush();assert.equal(t.stops(),1);assert.equal(t.api.isActive(),false);
 for(const state of [{mode:'disabled'},{mode:'emergency'},{steering_pwm_available:false},{motor_available:true}]){
  const p=await setup(state);p.input(1700);assert.equal(p.requests.length,0);assert.equal(p.el('pwm-slider').disabled,true);
 }
 const bad=await setup();bad.input(1751);assert.equal(bad.requests.length,0);assert.equal(bad.errors(),1);
 bad.setOther(true);bad.input(1700);assert.equal(bad.requests.length,0);
 const fault=await setup();fault.input(1700);fault.reply();await flush();
 fault.pending.shift().reject(new Error('offline'));await flush();assert.equal(fault.api.isActive(),false);assert.equal(fault.stops(),1);
 console.log('PWM browser checks passed: latest target, cancellation, timeout, scopes and errors');
})().catch(error=>{console.error(error);process.exitCode=1;});
