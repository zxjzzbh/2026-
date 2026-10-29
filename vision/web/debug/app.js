/* Task descriptions map the existing API; no legacy client is composed here. */
(function(global){
  'use strict';
  const number=(key,label,min,max,step,value,path)=>({key,label,min,max,step,value,path,type:'number'});
  const select=(key,label,choices,value,path)=>({key,label,choices,value,path,type:'select'});
  const parkingFields=[select('surface','识别地面',[['auto','自动红 / 绿'],['red_running_track','红色跑道'],['indoor_green','绿色地面']],'auto','parking_bay.settings.surface'),
    number('lane_width_m','赛道总宽（米）',.6,2,.005,1.23,'parking_bay.settings.lane_width_m'),
    number('blocked_blue_fraction','入口蓝色占比阈值',.005,.3,.005,.03,'parking_bay.settings.blocked_blue_fraction'),
    number('clear_ground_fraction','可见地面最低占比',.5,.95,.01,.65,'parking_bay.settings.clear_ground_fraction'),
    select('selected_slot','初始方向（路径模式会重新确认）',[['left','左半区'],['right','右半区']],'right','parking_bay.selection.selected_slot'),
    number('forward_us','诊断前进油门（μs）',1501,1625,1,1575),number('steering_gain','诊断转向增益',1,3,.05,1.25)];
  const TASKS={
    crosswalk:{name:'斑马线停车与播报',key:'crosswalk_trial',prepare:'crosswalk_prepare',start:'crosswalk_start',stop:'crosswalk_cancel',heartbeat:'crosswalk_keepalive',
      description:'首次识别制动短停0.5秒，再完成原定点停车与10秒播报。触发距离不是最终停车距离。',
      fields:[number('brake_distance_cm','刹车触发距离（厘米）',25,65,1,55),select('path_mode','接近方式',[['straight','直线接近'],['lane','沿赛道线接近']],'straight')]},
    traffic:{name:'红绿灯停车',key:'traffic_trial',prepare:'traffic_prepare',start:'traffic_start',stop:'traffic_cancel',heartbeat:'traffic_keepalive',image:'/api/traffic/frame.jpg',
      description:'红灯1秒、绿灯2秒，各路分别计票；350ms制动。单项确认绿灯后仅播报并保持停车。当前灯色不满足条件时，历史绿票不能放行。',
      fields:[number('forward_us','接近油门（μs）',1501,1625,1,1570,'traffic_trial.settings.pwm.search_us'),number('max_approach_s','接近最长时间（秒）',.5,10,.5,10,'traffic_trial.settings.max_approach_s'),number('red_confirm_percent','红绿共用比例（%）',1,100,1,50,'traffic_trial.settings.red_confirm_percent')]},
    cone:{name:'避障场地测试',key:'cone_trial.avoidance',prepare:'cone_prepare',start:'cone_start',stop:'cone_cancel',heartbeat:'cone_keepalive',observe:'cone_observe',observeStop:'cone_stop',image:'/api/cone/frame.jpg',
      description:'红地蓝桶 / 蓝地红桶 / 室内白线场地。白线恢复总时限1秒，单边预测最多0.25秒；越界、画面中断或失联时停止。',
      fields:[select('scene_surface','场地',[['red_running_track','红色跑道（蓝桶）'],['blue_running_track','蓝色跑道（红桶）'],['indoor_white_line_track','室内白线场地']],'red_running_track','cone_trial.settings.scene.surface'),
        number('white_v_min','白线最低亮度',0,255,1,200,'cone_trial.settings.scene.white_v_min'),number('test_forward_us','前进油门（μs）',1501,1625,1,1600),number('test_left_us','左转端点（μs）',1650,1870,5,1870),number('test_right_us','右转端点（μs）',1350,1570,5,1350),number('test_steering_gain','转向增益',1,3,.1,1.5,'cone_trial.avoidance.default_steering_gain')]},
    departure:{name:'蓝色挡板识别（仅播报）',key:'departure_trial',start:'departure_start',stop:'departure_stop',image:'/api/departure/frame.jpg',
      description:'只识别挡板和播报，不输出前进；无需电调准备。开始后保持车辆和镜头不动，由人移开挡板。',
      fields:[number('min_blue_percent','出现所需蓝色面积（%）',.5,100,.5,19.53125,'departure_trial.settings.min_blue_percent'),number('clear_confirm_s','移开确认时间（秒）',.5,3,.1,.6,'departure_trial.settings.clear_confirm_s')]},
    parking:{name:'米制停车入位',key:'parking_trial',prepare:'parking_trial_prepare',start:'parking_trial_start',stop:'parking_trial_cancel',heartbeat:'parking_trial_keepalive',observe:'parking_bay_observe',observeStop:'parking_bay_stop',image:'/api/parking-bay/frame.jpg',mode:'metric_path',
      description:'正式路径模式：当前车位、位姿、运动标定必须有效；缺项会阻止准备。前进与曲率采用实测表，下方诊断油门不参与路径模式。',fields:parkingFields},
    visual:{name:'历史视觉诊断（不计正式入位）',key:'parking_trial',prepare:'parking_trial_prepare',start:'parking_trial_start',stop:'parking_trial_cancel',heartbeat:'parking_trial_keepalive',observe:'parking_bay_observe',observeStop:'parking_bay_stop',image:'/api/parking-bay/frame.jpg',mode:'visual_trial',
      description:'历史诊断会实际行驶，但图像参考和光流停稳不证明四轮入位，也不计为正式停车任务。',
      fields:[...parkingFields,number('slow_forward_us','末段油门（μs）',1501,1625,1,1550,'parking_bay.settings.visual_entry.slow_forward_us'),
        {...number('slow_line_y_normalized','减速线高度（0–1，可留空）',.3,.93,.001,'','parking_bay.settings.visual_entry.slow_line_y_normalized'),optional:true},
        {...number('stop_line_y_normalized','停止线高度（0–1，可留空）',.32,.95,.001,'','parking_bay.settings.visual_entry.stop_line_y_normalized'),optional:true}]}
  };
  const path=(obj,key)=>key?.split('.').reduce((value,part)=>value?.[part],obj);
  function createDebugClient(options){
    const doc=options.document,win=options.window,http=options.fetch,bootstrap=options.bootstrap;
    const client=options.clientId||Array.from(global.crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join('');
    const every=options.setInterval||setInterval,later=options.setTimeout||setTimeout,clearLater=options.clearTimeout||clearTimeout;
    const el=id=>doc.getElementById(id),fieldNodes=new Map();
    let status=null,online=false,stale=false,busy=false,dirty=false,task='crosswalk',epoch=0,token=null,sequence=-1;
    let queue=Promise.resolve(),pulseBusy=false,selectedOnce=false,observationOwner=null,stopping=false,started=false,lastSnapshot=-Infinity;
    const selected=()=>TASKS[task],state=()=>path(status,selected().key)||{},owned=()=>!!state().active&&state().owner_client===client;
    const activeRows=()=>[status?.crosswalk_trial,status?.traffic_trial,status?.cone_trial?.avoidance,status?.cone_trial?.short_test,status?.departure_trial,status?.parking_trial,status?.race_flow,status?.plan_b].filter(r=>r?.active);
    const validPage=()=>online&&!stale&&status?.application_mode==='debug';
    const error=text=>{el('message').textContent=text||'';};
    function superseded(){const e=new Error('任务轮次已结束');e.superseded=true;return e;}
    function fillFields(){
      el('settings-fields').replaceChildren();fieldNodes.clear();
      for(const f of selected().fields){const label=doc.createElement('label'),text=doc.createElement('span'),input=doc.createElement(f.type==='select'?'select':'input');
        text.textContent=f.label;input.id='field-'+f.key;input.name=f.key;
        if(f.type==='select'){for(const [value,title] of f.choices){const option=doc.createElement('option');option.value=value;option.textContent=title;input.append(option);}}
        else{input.type='number';input.min=f.min;input.max=f.max;input.step=f.step;input.required=!f.optional;}
        let value=path(status,f.path)??f.value;
        if(task==='crosswalk'&&f.key==='brake_distance_cm')value=Math.round((status?.crosswalk_trial?.brake_trigger?.distance_m??.55)*100);
        input.value=value==null?'':String(value);input.addEventListener('input',()=>{dirty=true;render();});input.addEventListener('change',()=>{dirty=true;render();});
        label.append(text,input);el('settings-fields').append(label);fieldNodes.set(f.key,input);
      }
      dirty=false;
    }
    function values(){const result={};for(const f of selected().fields){const input=fieldNodes.get(f.key);
      if(input.checkValidity&&!input.checkValidity()){input.reportValidity?.();throw new Error('请检查参数：'+f.label);}
      result[f.key]=f.type==='select'?input.value:input.value===''&&f.optional?null:Number(input.value);
      if(f.type!=='select'&&result[f.key]!==null&&(!Number.isFinite(result[f.key])||result[f.key]<f.min||result[f.key]>f.max))throw new Error('参数越界：'+f.label);
    }return result;}
    function missing(){const r=state(),list=[];
      const add=value=>{if(Array.isArray(value))value.forEach(add);else if(typeof value==='string')list.push(value);};
      add(r.metric_missing);add(r.missing);add(r.readiness?.missing);
      if(task==='cone')add(status?.cone_trial?.readiness?.missing);
      if(r.settings_error)list.push(r.settings_error);
      if(task==='visual')list.push('历史视觉诊断不计正式入位，也不验证四轮物理位置。');
      return Array.from(new Set(list));
    }
    function render(){const d=selected(),r=state(),own=owned(),active=activeRows().length>0,valid=validPage();
      el('connection').textContent=stale?'页面已过期，请刷新':online?'已连接':'连接中断';
      el('task-description').textContent=d.description;
      el('task-select').disabled=busy||active||!!observationOwner;
      el('manual-link').setAttribute('aria-disabled',active?'true':'false');
      el('save-settings').disabled=!valid||busy||active||!!observationOwner||r.can_save===false||r.can_save_distance===false||!!r.settings_error;
      for(const [key,input] of fieldNodes)input.disabled=!valid||busy||active||!!observationOwner||(task==='parking'&&['forward_us','steering_gain','selected_slot'].includes(key));
      el('settings-state').textContent=dirty?'参数已修改，点击保存后再准备。':'参数已载入；保存调用车端原设置接口。';
      el('prepare-confirmations').hidden=!d.prepare;el('start-confirmations').hidden=!d.prepare;
      const canPrepare=!!r.can_prepare&&!(task==='parking'&&missing().length)&&(task!=='cone'||status?.cone_trial?.low_speed_trial_available===true);
      el('prepare-task').disabled=!valid||busy||active||!d.prepare||!canPrepare||dirty||!el('confirm-esc-off').checked||!el('confirm-fb').checked;
      const canStart=d.prepare?(r.can_request_start??r.can_start):r.can_start;
      el('start-task').disabled=!valid||busy||!canStart||dirty||(d.prepare&&(!own||!el('confirm-on-static').checked||!el('confirm-field').checked||!el('confirm-fb').checked));
      el('prepare-task').textContent=d.prepare?'准备（电调关闭）':'本项仅识别，无电调准备';
      el('stop-task').disabled=!valid||stopping||(!own&&observationOwner!==task);
      el('observe-task').hidden=!d.observe;
      const canObserve=task==='cone'?status?.cone_trial?.can_observe:status?.parking_bay?.can_observe;
      el('observe-task').disabled=!valid||busy||active||!!observationOwner||!canObserve;
      el('confirmation-help').textContent=d.prepare?'现场勾选只代表你的明确确认；不会替你确认电调状态或实测标定。':'挡板测试仅识别与播报，开始不会发送前进指令。';
      el('task-phase').textContent=r.phase||'尚未准备';el('task-reason').textContent=r.reason||'';
      el('task-owner').textContent=r.active?(own?'本页持有任务控制权':'另一页面持有控制权，本页只观察'):'';
      const intent=r.intent;el('task-output').textContent=intent?`当前请求 ${intent.action||''} · 油门 ${intent.esc_us??'—'} μs · 转向 ${intent.steering_us??'—'} μs`:'';
      el('task-figure').hidden=!d.image;
      let diagnostics=[];
      if(task==='crosswalk'){diagnostics.push('本项显示下方双摄实际画面及停车状态。');
        const distance=intent?.distance_estimate_m??r.distance_observation?.distance_estimate_m;
        if(typeof distance==='number')diagnostics.push('当前测距 '+(distance*100).toFixed(1)+' 厘米');
        if(typeof intent?.hold_remaining_s==='number')diagnostics.push('停车保持剩余 '+intent.hold_remaining_s.toFixed(1)+' 秒');}
      if(task==='traffic'){
        const votes=intent?.signal_votes,current=votes?.raw_state||r.observation?.signal?.state||'unknown';
        const names={red:'红灯',yellow:'黄灯',green:'绿灯',off:'熄灯',unknown:'未确认灯色'};
        el('traffic-current').hidden=false;el('traffic-current').textContent='当前识别：'+(names[current]||'未知')+(votes?.green_confirmed?' · 绿灯条件已确认':' · 未授权放行');
        for(const [camera,v] of Object.entries(votes?.cameras||{}))diagnostics.push((camera==='primary'?'第一路':'第二路')+` 历史窗口：红 ${v.red_frames}/${v.total_frames}，绿 ${v.green_frames}/${v.green_total_frames??v.total_frames}`);
        diagnostics.push('红窗1秒 / 绿窗2秒 / 制动350ms；单项绿灯只播报，保持停车。');
      }else el('traffic-current').hidden=true;
      if(task==='cone'){const n=status?.cone_trial?.latest?.navigation;
        if(r.low_speed_stop_for_cones===false)diagnostics.push('锥桶碰撞停车已关闭，继续按白线引导行驶。');
        diagnostics.push(n?.line_missing?`白线恢复总时限${r.line_recovery_s??1}秒，持续不足停止。`:n?.reason||'等待新鲜边界与锥桶观测');}
      if(task==='departure'){for(const [camera,row] of Object.entries(r.observations||{}))diagnostics.push(`${camera}：${row.state} #${row.frame_id}`);}
      if(task==='parking'||task==='visual'){const p=status?.parking_bay;diagnostics.push('识别建议：'+(p?.selection?.selected_slot||'未确认'));if(task==='visual')diagnostics.push('此模式不计正式入位。');}
      el('task-diagnostics').textContent=diagnostics.join('\n');
      el('task-missing').replaceChildren();for(const item of missing()){const node=doc.createElement('li');node.textContent=item;el('task-missing').append(node);}
      el('missing-details').hidden=missing().length===0;
    }
    function applyStatus(next,expected){
      if(expected!==undefined&&expected!==epoch)return false;
      if(next.control_session_id&&next.control_session_id!==bootstrap.session){stale=true;epoch++;render();return false;}
      const times=['application_status_s','traffic_status_s','cone_status_s','parking_trial_status_s','departure_status_s'].map(k=>next[k]).filter(x=>typeof x==='number');
      const at=times.length?Math.max(...times):null;if(at!==null&&at<lastSnapshot)return false;if(at!==null)lastSnapshot=at;
      const nextRun=path(next,selected().key)||{},nextToken=nextRun.run_token||null;
      if(nextToken!==token){token=nextToken;sequence=-1;epoch++;}
      status=next;if(!selectedOnce){fillFields();selectedOnce=true;}
      if(observationOwner===task&&!(task==='cone'?status.cone_trial?.observing:status.parking_bay?.active))observationOwner=null;
      render();return true;
    }
    async function rawRequest(action,body={},keepalive=false){
      const controller=new AbortController(),timer=later(()=>controller.abort(),2000);
      try{const response=await http('/api/drive/'+action,{method:'POST',cache:'no-store',keepalive,signal:controller.signal,
        headers:{'Content-Type':'application/json','X-Drive-Key':bootstrap.key},body:JSON.stringify({client,setup_token:status?.setup_token,trial_token:status?.trial_token,...body})});
        const next=await response.json();if(!response.ok){if(next.code==='control_page_rejected'){stale=true;epoch++;render();}throw new Error(next.error||'操作未确认');}return next;
      }finally{clearLater(timer);}
    }
    function enqueue(action,body={},sequenced=false){const generation=epoch,selectedTask=task,currentToken=token;
      const run=async()=>{if(generation!==epoch||selectedTask!==task||!validPage())throw superseded();
        if(sequenced&&(!owned()||token!==currentToken))throw superseded();
        const payload=sequenced?{...body,run_token:currentToken,trial_sequence:++sequence}:body;
        const next=await rawRequest(action,payload);if(generation!==epoch||selectedTask!==task)throw superseded();applyStatus(next,generation);return next;};
      const pending=queue.then(run,run);queue=pending.catch(()=>{});return pending;
    }
    async function action(work){if(busy||!validPage())return;busy=true;error('');render();try{return await work();}
      catch(e){if(!e.superseded)error(e.name==='AbortError'?'连接超时，请核对车辆状态。':e.message);}finally{busy=false;render();}}
    async function save(){return action(async()=>{if(activeRows().length||observationOwner)throw new Error('请先停止当前测试或识别');
      const v=values(),d=selected();
      if(task==='crosswalk')await enqueue('crosswalk_settings',{brake_distance_cm:v.brake_distance_cm,settings_revision:state().settings_revision});
      if(task==='traffic')await enqueue('traffic_settings',{forward_us:v.forward_us,max_approach_s:v.max_approach_s,red_confirm_percent:v.red_confirm_percent,settings_revision:state().settings_revision});
      if(task==='cone'){await enqueue('cone_settings',{scene_surface:v.scene_surface});await enqueue('cone_white_threshold',{white_v_min:v.white_v_min});}
      if(task==='departure')await enqueue('departure_save',{min_blue_percent:v.min_blue_percent,clear_confirm_s:v.clear_confirm_s,settings_revision:state().settings_revision});
      if(task==='parking'||task==='visual'){
        const body={surface:v.surface,lane_width_m:v.lane_width_m,blocked_blue_fraction:v.blocked_blue_fraction,clear_ground_fraction:v.clear_ground_fraction,settings_revision:status.parking_bay?.settings_revision};
        if(task==='visual')body.visual_entry={...(status.parking_bay?.settings?.visual_entry||{}),slow_forward_us:v.slow_forward_us,slow_line_y_normalized:v.slow_line_y_normalized,stop_line_y_normalized:v.stop_line_y_normalized};
        await enqueue('parking_bay_save',body);
      }
      dirty=false;
    });}
    async function prepare(){if(el('prepare-task').disabled)return;return action(async()=>{
      const v=values(),d=selected(),body={esc_off:el('confirm-esc-off').checked,fb_mode_confirmed:el('confirm-fb').checked};
      if(task==='cone')Object.assign(body,{profile:'legacy_low_speed',test_forward_us:v.test_forward_us,test_left_us:v.test_left_us,test_right_us:v.test_right_us,test_steering_gain:v.test_steering_gain});
      if(task==='parking'||task==='visual')Object.assign(body,{selected_slot:v.selected_slot,forward_us:v.forward_us,steering_gain:v.steering_gain,controller_mode:d.mode});
      await enqueue(d.prepare,body);
      el('confirm-esc-off').checked=false;el('confirm-on-static').checked=false;el('confirm-field').checked=false;
    });}
    async function startTask(){if(el('start-task').disabled)return;return action(async()=>{const v=values(),d=selected();
      const body=d.prepare?{fb_mode_confirmed:el('confirm-fb').checked,esc_on_static:el('confirm-on-static').checked,field_ready:el('confirm-field').checked}:
        {min_blue_percent:v.min_blue_percent,clear_confirm_s:v.clear_confirm_s,settings_revision:state().settings_revision};
      if(task==='crosswalk')body.path_mode=v.path_mode;
      await enqueue(d.start,body,!!d.prepare);
    });}
    async function stop(explicit=false,source='stop_button',keepalive=false){const own=owned(),observe=observationOwner===task;
      if(!explicit&&!own&&!observe)return;
      epoch++;stopping=true;error('');
      // Explicit emergency/stop must not wait behind a delayed keepalive. Its
      // generation invalidates every queued old command and late response.
      try{const d=selected();const next=await rawRequest(explicit?'emergency':own?d.stop:d.observeStop,{stop_source:source},keepalive);applyStatus(next);observationOwner=null;}
      catch(e){error('停止确认未收到：'+e.message);}finally{stopping=false;render();}
    }
    async function observe(){if(el('observe-task').disabled)return;return action(async()=>{await enqueue(selected().observe);observationOwner=task;});}
    async function pulse(){const d=selected();if(!validPage()||doc.hidden||pulseBusy||stopping||!owned()||!d.heartbeat)return;
      pulseBusy=true;const generation=epoch;
      try{await enqueue(d.heartbeat,{},true);}catch(e){if(!e.superseded&&generation===epoch){error(e.message);await stop(false,'connection_lost');}}
      finally{pulseBusy=false;}}
    async function poll(){const controller=new AbortController(),timer=later(()=>controller.abort(),2000);
      try{const response=await http('/api/drive/status',{cache:'no-store',signal:controller.signal});if(!response.ok)throw new Error('状态读取失败');online=true;applyStatus(await response.json());}
      catch(e){online=false;render();if(owned())void stop(false,'connection_lost');}finally{clearLater(timer);}}
    function choose(next){if(!TASKS[next]||busy||activeRows().length||observationOwner)return false;
      task=next;epoch++;token=null;sequence=-1;selectedOnce=false;fillFields();selectedOnce=!!status;
      for(const id of ['confirm-esc-off','confirm-fb','confirm-on-static','confirm-field'])el(id).checked=false;
      error('');render();return true;}
    function camera(id,url){const image=el('camera-'+id),label=el('camera-'+id+'-state');
      if(doc.hidden||id==='task'&&!selected().image){image.hidden=true;later(()=>camera(id,id==='task'?selected().image:url),400);return;}
      const source=id==='task'?selected().image:url,currentTask=task,pending=doc.createElement('img');let ended=false;
      const finish=ok=>{if(ended)return;ended=true;clearLater(timer);pending.onload=pending.onerror=null;
        if(ok&&(id!=='task'||currentTask===task)){image.src=pending.src;image.hidden=doc.hidden;label.textContent='刚刚更新';}else{image.hidden=true;label.textContent='等待新鲜画面';}
        later(()=>camera(id,url),ok?200:400);};
      const timer=later(()=>{finish(false);pending.removeAttribute('src');},1200);pending.onload=()=>finish(true);pending.onerror=()=>finish(false);pending.src=source+(source.includes('?')?'&':'?')+'t='+Date.now();
    }
    function bind(){el('task-select').addEventListener('change',()=>{if(!choose(el('task-select').value))el('task-select').value=task;});
      el('settings-form').addEventListener('submit',e=>{e.preventDefault();void save();});
      el('save-settings').addEventListener('click',save);el('prepare-task').addEventListener('click',prepare);el('start-task').addEventListener('click',startTask);
      el('stop-task').addEventListener('click',()=>stop());el('emergency').addEventListener('click',()=>stop(true,'keyboard_or_button'));
      el('observe-task').addEventListener('click',observe);
      for(const id of ['confirm-esc-off','confirm-fb','confirm-on-static','confirm-field'])el(id).addEventListener('change',render);
      el('manual-link').addEventListener('click',e=>{if(activeRows().length){e.preventDefault();error('请先停止当前任务，再打开遥控页。');}});
      win.addEventListener('keydown',e=>{if(e.code==='Space'||e.code==='Escape'){e.preventDefault();void stop(true,'keyboard_emergency');}});
      win.addEventListener('blur',()=>void stop(false,'window_blur'));
      win.addEventListener('pagehide',()=>void stop(false,'page_exit',true));
      doc.addEventListener('visibilitychange',()=>{if(doc.hidden)void stop(false,'page_hidden');});
    }
    function init(){if(started)return;started=true;bind();fillFields();render();void poll();every(poll,700);every(pulse,100);
      camera('primary','/frame.jpg?camera=primary&view=raw');camera('secondary','/frame.jpg?camera=secondary&view=raw');camera('task',selected().image);}
    return {init,poll,pulse,applyStatus,render,choose,prepare,startTask,save,stop,observe,values,enqueue,client,
      inspect:()=>({task,status,online,busy,stale,epoch,sequence,token,dirty})};
  }
  if(typeof module!=='undefined'&&module.exports)module.exports={TASKS,createDebugClient};
  else createDebugClient({document,window,fetch:window.fetch.bind(window),bootstrap:JSON.parse(document.getElementById('control-bootstrap').textContent)}).init();
})(globalThis);
