const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const payload=JSON.parse(fs.readFileSync(0,'utf8'));

function pad(){return {index:0,id:'mock standard USB pad',connected:true,mapping:'standard',axes:[0,0,0,0],
 buttons:Array.from({length:17},()=>({value:0,pressed:false}))};}
function setup(){
 const arm={checked:false,disabled:false},info={textContent:''},actions=[];
 let device=pad(),pads=[device],now=0;
 const state={allowed:true,ready:true,stale:false,focused:true,otherInput:false,
  capabilities:{forward:true,reverse:true,left:true,right:true}};
 const context={};vm.createContext(context);vm.runInContext(payload.component,context);
 const controls=context.createUSBGamepadControls({arm,info,getPads:()=>pads,now:()=>now,state:()=>state,
  stop:(reason,emergency)=>actions.push({type:'stop',reason,emergency}),
  directions:directions=>actions.push({type:'directions',directions:Array.from(directions)})});
 function sample(ms=50){now+=ms;controls.sample();}
 function button(index,value){device.buttons[index]={value,pressed:value>0.5};}
 function armNeutral(){sample();button(5,1);sample();button(5,0);sample();}
 function forward(){button(7,1);sample();}
 return {arm,info,actions,state,controls,sample,button,armNeutral,forward,
  get device(){return device;},set device(v){device=v;pads=[v];},set pads(v){pads=v;}};
}
let count=0;
function check(name,run){run();count++;}


check('default detection never commands',()=>{const h=setup();h.button(7,1);h.sample();assert.equal(h.actions.length,0);assert.equal(h.controls.armed,false);});
check('RB press toggles on, hold and release do not toggle',()=>{const h=setup();h.sample();h.button(5,1);h.sample();assert.equal(h.controls.armed,true);h.sample();h.button(5,0);h.sample();assert.equal(h.controls.armed,true);h.forward();assert.deepEqual(h.actions.at(-1).directions,['forward']);});
check('second RB press stops and toggles off',()=>{const h=setup();h.armNeutral();h.forward();h.button(5,1);h.sample();assert.equal(h.controls.armed,false);assert.equal(h.actions.at(-1).reason,'gamepad_toggle_off');h.sample();assert.equal(h.actions.length,2);});
check('held RT cannot arm through RB',()=>{const h=setup();h.sample();h.button(7,1);h.button(5,1);h.sample();assert.equal(h.controls.armed,false);assert.equal(h.actions.length,0);h.button(7,0);h.button(5,0);h.sample();h.button(5,1);h.sample();assert.equal(h.controls.armed,true);});
check('first connection with held RB never arms',()=>{const h=setup();h.button(5,1);h.sample();h.sample();assert.equal(h.controls.armed,false);});
check('forward and steering is one chord',()=>{const h=setup();h.armNeutral();h.device.axes[0]=-.7;h.forward();assert.deepEqual(h.actions.at(-1).directions,['forward','left']);});
check('RT forward and LT reverse do not require held RB',()=>{const h=setup();h.armNeutral();h.button(6,.8);h.sample();assert.deepEqual(h.actions.at(-1).directions,['reverse']);assert.equal(h.device.buttons[5].value,0);});
check('holding a direction emits no new direction transition',()=>{const h=setup();h.armNeutral();h.forward();for(let i=0;i<10;i++)h.sample();assert.equal(h.actions.length,1);});
check('steering release while RT held continues forward',()=>{const h=setup();h.armNeutral();h.device.axes[0]=.7;h.forward();h.device.axes[0]=0;h.sample();assert.deepEqual(h.actions.at(-1),{type:'directions',directions:['forward']});assert.equal(h.controls.armed,true);});
check('left to right while driving changes steering without stop',()=>{const h=setup();h.armNeutral();h.device.axes[0]=-.7;h.forward();h.device.axes[0]=.7;h.sample();assert.deepEqual(h.actions.at(-1).directions,['forward','right']);assert.equal(h.actions.some(x=>x.type==='stop'),false);});
check('trigger release while steering preserves steering',()=>{const h=setup();h.armNeutral();h.device.axes[0]=.7;h.forward();h.button(7,0);h.sample();assert.deepEqual(h.actions.at(-1).directions,['right']);});
check('full release stops but preserves RB ownership',()=>{const h=setup();h.armNeutral();h.forward();h.button(7,0);h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_release');assert.equal(h.controls.armed,true);h.sample();h.forward();assert.deepEqual(h.actions.at(-1).directions,['forward']);});
check('pending stop reply does not toggle the mode',()=>{const h=setup();h.armNeutral();h.forward();h.button(7,0);h.sample();h.state.allowed=false;h.sample();assert.equal(h.controls.armed,true);h.state.allowed=true;h.sample();h.forward();assert.deepEqual(h.actions.at(-1).directions,['forward']);});
check('B emergency latches once and disarms',()=>{const h=setup();h.armNeutral();h.forward();h.button(1,1);h.sample();h.sample();assert.equal(h.actions.length,2);assert.equal(h.actions[1].emergency,true);assert.equal(h.controls.armed,false);});
check('B also works while RB ownership is off',()=>{const h=setup();h.sample();h.button(1,1);h.sample();assert.equal(h.actions.at(-1).emergency,true);});
check('external stop cannot restart held pad',()=>{const h=setup();h.armNeutral();h.forward();h.controls.requireNeutral();h.sample();h.sample();assert.equal(h.actions.length,1);});
check('disable and reenable cannot restore ownership',()=>{const h=setup();h.armNeutral();h.forward();h.state.allowed=h.state.ready=false;h.sample();h.state.allowed=h.state.ready=true;h.sample();assert.equal(h.controls.armed,false);assert.equal(h.actions.length,2);});
check('disconnect stops and disarms',()=>{const h=setup();h.armNeutral();h.forward();h.pads=[];h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_disconnected');assert.equal(h.controls.armed,false);});
check('reconnect with held RB and RT cannot autoarm',()=>{const h=setup();h.armNeutral();h.forward();h.pads=[];h.sample();h.device=pad();h.button(5,1);h.button(7,1);h.sample();assert.equal(h.actions.length,2);assert.equal(h.controls.armed,false);});
check('replacement device stops',()=>{const h=setup();h.armNeutral();h.forward();h.device.id='replacement';h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_device_changed');assert.equal(h.controls.armed,false);});
check('multiple devices stop',()=>{const h=setup();h.armNeutral();h.forward();h.pads=[h.device,{...pad(),index:1}];h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_disconnected');});
check('loss of focus disarms without restarting held input',()=>{const h=setup();h.armNeutral();h.forward();h.state.focused=false;h.sample();h.state.focused=true;h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_focus_lost');assert.equal(h.controls.armed,false);});
check('sampling gap stops and disarms',()=>{const h=setup();h.armNeutral();h.forward();h.sample(250);h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_poll_gap');assert.equal(h.controls.armed,false);});
check('conflicting triggers stop until neutral',()=>{const h=setup();h.armNeutral();h.forward();h.button(6,1);h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_conflict');h.button(6,0);h.sample();assert.equal(h.actions.length,2);});
check('unknown mapping is refused',()=>{const h=setup();h.device.mapping='';h.armNeutral();h.forward();assert.equal(h.actions.length,0);assert.equal(h.controls.armed,false);});
check('NaN input stops',()=>{const h=setup();h.armNeutral();h.forward();h.device.axes[0]=NaN;h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_invalid');});
check('invalid button stops',()=>{const h=setup();h.armNeutral();h.forward();h.device.buttons[7].value=2;h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_invalid');});
check('missing axes stops',()=>{const h=setup();h.armNeutral();h.forward();h.device.axes=null;h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_invalid');});
check('throwing getter stops',()=>{const h=setup();h.armNeutral();h.forward();Object.defineProperty(h.device,'axes',{get(){throw new Error('mock failure');}});h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_read_failure');});
check('dead zone and hysteresis avoid drift',()=>{const h=setup();h.armNeutral();h.device.axes[0]=.2;h.sample();assert.equal(h.actions.length,0);h.device.axes[0]=.3;h.sample();assert.deepEqual(h.actions[0].directions,['right']);h.device.axes[0]=.2;h.sample();assert.equal(h.actions.length,1);h.device.axes[0]=.1;h.sample();assert.equal(h.actions.at(-1).reason,'gamepad_release');});
check('unavailable reverse never submitted',()=>{const h=setup();h.armNeutral();h.state.capabilities.reverse=false;h.button(6,1);h.sample();assert.equal(h.actions.length,0);});
check('checkbox and RB share one ownership state',()=>{const h=setup();h.armNeutral();h.forward();h.arm.checked=false;h.arm.onchange();assert.equal(h.actions.at(-1).reason,'gamepad_toggle_off');assert.equal(h.controls.armed,false);});

// Integration check executes the delivered dashboard script with mock DOM and
// HTTP responses. No browser, network, controller or vehicle is used.
async function integration(){
 const elements=new Map(),events={},intervals={},requests=[];
 const directions=['forward','reverse','left','right'].map(d=>({dataset:{drive:d},style:{},disabled:false,
  classList:{add(){},remove(){},toggle(){}},setPointerCapture(){}}));
 function el(id){if(!elements.has(id))elements.set(id,{textContent:'',style:{},checked:false,hidden:false,
  disabled:false,value:20,focus(){},classList:{add(){},remove(){},toggle(){}}});return elements.get(id);}
 let device=pad(),now=0;
 let status={mode:'enabled',reason:'mock',hardware_output:false,controls_paused:false,
  continuous_simulation:false,motor_available:true,steering_available:true,reverse_available:true,
  motor_direction:'stop',steering_direction:'center',motor_pulse_us:1500,session_remaining_s:60,
  control_session_id:'session',release_required:false};
 const context={navigator:{getGamepads:()=>[device]},document:{hidden:false,hasFocus:()=>true,
  getElementById:el,querySelectorAll:selector=>selector==='[data-drive]'?directions:[],
  addEventListener:(name,fn)=>{events[name]=fn;}},window:{addEventListener:(name,fn)=>{events[name]=fn;},location:{reload(){}}},
  crypto:{getRandomValues:array=>array.fill(1)},Uint8Array,AbortController,performance:{now:()=>now},
  setTimeout:()=>1,clearTimeout(){},setInterval:(fn,ms)=>{intervals[ms]=fn;},
  fetch:async(url,options={})=>{
   if(options.body){const body=JSON.parse(options.body);requests.push({url,body});
    if(url.endsWith('/emergency'))status={...status,mode:'emergency',motor_direction:'stop'};
    else if(url.endsWith('/reset'))status={...status,mode:'disabled',motor_direction:'stop',steering_direction:'center'};
    else if(url.endsWith('/enable'))status={...status,mode:'enabled',motor_direction:'stop',steering_direction:'center'};
    else if(url.endsWith('/stop'))status={...status,motor_direction:'stop',steering_direction:'center'};
    else if(url.endsWith('/command'))status={...status,motor_direction:body.motor,steering_direction:body.steering};}
   return {ok:true,json:async()=>({...status})};
  }};
 vm.createContext(context);
 const script=payload.dashboard.replace(/^<script>\s*/,'').replace(/\s*<\/script>$/,'')
  .replace('__DRIVE_KEY__','"key"').replace('__DRIVE_SESSION__','"session"');
 vm.runInContext(script,context);
 const flush=()=>new Promise(resolve=>setImmediate(resolve));await flush();
 const tick=async()=>{now+=50;intervals[50]();await flush();};
 function button(index,value){device.buttons[index]={value,pressed:value>0.5};}
 const arm=el('gamepad-arm');await tick();button(5,1);await tick();button(5,0);await tick();
 button(7,1);await tick();
 const motion=requests.filter(r=>r.url.endsWith('/command')&&r.body.motor==='forward');
 assert.equal(motion.length,1);assert.equal(motion[0].body.input_source,'gamepad');
 assert.equal(requests[0].body.motor,'stop');assert.equal(requests[0].body.steering,'center');count++;
 // A keyboard direction cannot join a selected gamepad's command chord.
 events.keydown({code:'KeyA',key:'a',repeat:false,target:{tagName:'DIV'},preventDefault(){}});await flush();
 assert.equal(requests.filter(r=>r.url.endsWith('/command')&&r.body.steering==='left').length,0);count++;
 events.keydown({code:'Space',key:' ',repeat:false,target:{tagName:'DIV'},preventDefault(){}});await flush();
 assert.equal(requests.at(-1).url,'/api/drive/emergency');
 const before=requests.length;await tick();intervals[70]();await flush();assert.equal(requests.length,before);count++;
 // Even after a mock reset/enable status, holding RT and RB cannot restart.
 await el('drive-reset').onclick();el('bench-ready').checked=true;await el('drive-enable').onclick();
 assert.equal(requests.at(-1).url,'/api/drive/enable');
 const enabledCount=requests.length;await tick();intervals[70]();await flush();assert.equal(requests.length,enabledCount);count++;
}
integration().then(()=>process.stdout.write(JSON.stringify({checks_passed:count,hardware_output:false,simulated:true})+'\n'))
 .catch(error=>{console.error(error);process.exitCode=1;});
