const fs=require('node:fs'),assert=require('node:assert/strict');
const {createDebugClient}=require(process.argv[2]);
const input=JSON.parse(fs.readFileSync(0,'utf8')),nodes=new Map(),events={},documentEvents={},requests=[];
function element(tag,attrs={}){let id=attrs.id||'';const listeners={};const n={tagName:tag.toUpperCase(),value:attrs.value||'',name:'',disabled:'disabled'in attrs,hidden:'hidden'in attrs,checked:false,textContent:'',children:[],
 set id(v){id=v;nodes.set(v,this)},get id(){return id},setAttribute(){},append(...items){this.children.push(...items)},replaceChildren(...items){this.children=items},removeAttribute(){},checkValidity(){return true},reportValidity(){},
 addEventListener(k,fn){(listeners[k]??=[]).push(fn)},async fire(k,arg={}){for(const fn of listeners[k]||[])await fn(arg)}};return n;}
for(const [,tag,body] of input.html.matchAll(/<([a-zA-Z][\w-]*)\b([^>]*)>/g)){const attrs={};for(const [,k,v]of body.matchAll(/([\w-]+)(?:="([^"]*)")?/g))attrs[k]=v??'';const n=element(tag,attrs);if(attrs.id)nodes.set(attrs.id,n);}
const document={hidden:false,getElementById:id=>nodes.get(id),createElement:tag=>element(tag),addEventListener:(k,fn)=>(documentEvents[k]??=[]).push(fn)};
const window={addEventListener:(k,fn)=>(events[k]??=[]).push(fn)};
let rev=0,status,deferPulse=false,pendingPulse=null,lastSeq=-1;
const stamp=s=>({...structuredClone(s),application_status_s:++rev});
const base=()=>stamp({application_mode:'debug',control_session_id:'session',setup_token:'setup',trial_token:'manual-token',mode:'disabled',boot_test_phase:'esc_off',
 crosswalk_trial:{can_prepare:true,can_start:false,settings_revision:'cw',brake_trigger:{distance_m:.55}},
 traffic_trial:{can_prepare:true,can_start:false,settings_revision:'tr',settings:{pwm:{search_us:1570},max_approach_s:10,red_confirm_percent:50}},
 cone_trial:{can_observe:true,observing:false,low_speed_trial_available:true,settings:{scene:{surface:'red_running_track',white_v_min:200}},avoidance:{can_prepare:true,can_start:false}},
 departure_trial:{can_start:true,can_save:true,settings_revision:'dep',settings:{min_blue_percent:19.5,clear_confirm_s:.6}},
 parking_trial:{can_prepare:true,can_start:false,metric_missing:['缺停车标定']},
 parking_bay:{can_observe:true,active:false,settings_revision:'park',selection:{selected_slot:'right'},settings:{surface:'auto',lane_width_m:1.23,blocked_blue_fraction:.03,clear_ground_fraction:.65,visual_entry:{slow_forward_us:1550,slow_line_y_normalized:null,stop_line_y_normalized:null}}}});
status=base();
const rowFor=action=>action.startsWith('crosswalk_')?status.crosswalk_trial:action.startsWith('traffic_')?status.traffic_trial:action.startsWith('cone_')?status.cone_trial.avoidance:action.startsWith('departure_')?status.departure_trial:status.parking_trial;
const response=(value,ok=true)=>({ok,json:async()=>structuredClone(value)});
const http=async(url,opts={})=>{if(opts.method!=='POST')return response(status);
 const action=url.split('/').at(-1),body=JSON.parse(opts.body);requests.push({action,body});assert.equal(opts.headers['X-Drive-Key'],'key');
 if(action==='emergency'){status=base();return response(status);}
 const row=rowFor(action);
 if(action.endsWith('_keepalive')){const process=()=>{if(body.trial_sequence<=lastSeq)return response({error:'old sequence'},false);lastSeq=body.trial_sequence;return response(stamp(status));};
  if(deferPulse)return new Promise(resolve=>{pendingPulse=()=>resolve(process())});return process();}
 if(action.endsWith('_prepare')){assert.equal(body.esc_off,true);assert.equal(body.fb_mode_confirmed,true);Object.assign(row,{active:true,owner_client:'page',run_token:'token-'+action,phase:'ready',can_start:true,can_request_start:true});lastSeq=-1;}
 if(action.endsWith('_start')&&action!=='departure_start'){assert.equal(body.esc_on_static,true);assert.equal(body.field_ready,true);assert(body.trial_sequence>lastSeq);lastSeq=body.trial_sequence;Object.assign(row,{phase:'approach',can_start:false,can_request_start:false});}
 if(action==='departure_start')Object.assign(row,{active:true,owner_client:'page',phase:'wait_present',can_start:false});
 if(action.endsWith('_cancel')||action==='departure_stop'){Object.assign(row,{active:false,phase:'cancelled',can_prepare:true,can_start:action==='departure_stop',can_request_start:false});}
 if(action==='cone_observe')status.cone_trial.observing=true;
 if(action==='cone_stop')status.cone_trial.observing=false;
 if(action==='parking_bay_observe')status.parking_bay.active=true;
 if(action==='parking_bay_stop')status.parking_bay.active=false;
 if(action==='traffic_settings')Object.assign(status.traffic_trial.settings,{max_approach_s:body.max_approach_s,red_confirm_percent:body.red_confirm_percent,pwm:{search_us:body.forward_us}});
 if(action==='cone_settings')status.cone_trial.settings.scene.surface=body.scene_surface;
 if(action==='cone_white_threshold')status.cone_trial.settings.scene.white_v_min=body.white_v_min;
 status=stamp(status);return response(status);
};
const api=createDebugClient({document,window,fetch:http,bootstrap:{key:'key',session:'session'},clientId:'page',setTimeout:()=>1,clearTimeout(){},setInterval(){}});
const flush=async()=>{for(let i=0;i<5;i++)await new Promise(setImmediate);};
function check(id){nodes.get(id).checked=true;api.render();}
(async()=>{
 api.init();await flush();assert.equal(requests.length,0,'initial load is read-only');
 await api.prepare();assert.equal(requests.length,0,'unchecked confirmations cannot be forged');
 check('confirm-esc-off');check('confirm-fb');await api.prepare();
 assert.equal(requests.at(-1).action,'crosswalk_prepare');assert.equal(requests.at(-1).body.setup_token,'setup');
 await api.startTask();assert.equal(requests.at(-1).action,'crosswalk_prepare','no implicit on/static confirmation');
 check('confirm-on-static');check('confirm-field');await api.startTask();
 assert.equal(requests.at(-1).action,'crosswalk_start');assert.equal(requests.at(-1).body.path_mode,'straight');
 assert.equal(api.choose('traffic'),false,'active task cannot be changed');
 await api.stop();assert.equal(requests.at(-1).action,'crosswalk_cancel');assert(api.choose('traffic'));

 nodes.get('field-forward_us').value='1580';await nodes.get('field-forward_us').fire('input');
 await api.save();assert.equal(requests.at(-1).action,'traffic_settings');assert.equal(requests.at(-1).body.forward_us,1580);
 assert.equal(requests.at(-1).body.red_confirm_percent,50);assert(!('red_window_s'in requests.at(-1).body));
 check('confirm-esc-off');check('confirm-fb');await api.prepare();
 deferPulse=true;const pulse=api.pulse();await flush();assert(pendingPulse);
 check('confirm-on-static');check('confirm-field');const start=api.startTask();await flush();assert.equal(requests.at(-1).action,'traffic_keepalive','start waits behind heartbeat');
 pendingPulse();await pulse;await start;deferPulse=false;assert.equal(requests.at(-1).action,'traffic_start');
 assert(requests.at(-1).body.trial_sequence>requests.at(-2).body.trial_sequence);
 deferPulse=true;const latePulse=api.pulse();await flush();const lateReply=pendingPulse;
 await api.stop();assert(api.choose('cone'));lateReply();await latePulse;deferPulse=false;
 assert.equal(api.inspect().task,'cone');assert.equal(api.inspect().status.traffic_trial.active,false,'late reply cannot restore cancelled task');
 nodes.get('field-scene_surface').value='blue_running_track';await nodes.get('field-scene_surface').fire('change');await api.save();
 assert.deepEqual(requests.slice(-2).map(r=>r.action),['cone_settings','cone_white_threshold']);assert.equal(requests.at(-2).body.scene_surface,'blue_running_track');
 await api.observe();assert.equal(requests.at(-1).action,'cone_observe');await api.stop();assert.equal(requests.at(-1).action,'cone_stop');
 assert(api.choose('parking'));check('confirm-esc-off');check('confirm-fb');assert.equal(nodes.get('prepare-task').disabled,true,'metric missing prevents prepare');
 assert(api.choose('visual'));check('confirm-esc-off');check('confirm-fb');await api.prepare();
 assert.equal(requests.at(-1).body.controller_mode,'visual_trial');assert.equal(requests.at(-1).body.selected_slot,'right');
 assert(nodes.get('task-description').textContent.includes('不计为正式停车'));
 await api.stop();assert(api.choose('departure'));await api.startTask();assert.equal(requests.at(-1).action,'departure_start');
 assert(!('esc_on_static'in requests.at(-1).body),'speech-only API receives no fabricated motor confirmation');await api.stop();

 assert(api.choose('traffic'));status.traffic_trial={...status.traffic_trial,active:true,owner_client:'other',run_token:'foreign',phase:'ready'};status=stamp(status);api.applyStatus(status);
 const before=requests.length;for(const fn of events.blur)fn();await flush();assert.equal(requests.length,before,'observer background cannot stop another owner');
 await nodes.get('emergency').fire('click');assert.equal(requests.at(-1).action,'emergency','explicit emergency stays global');
 console.log(JSON.stringify({hardware_output:false,checks:15}));
})().catch(e=>{console.error(e);process.exitCode=1});
