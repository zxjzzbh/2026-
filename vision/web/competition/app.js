/* Independent operator client. No debug panels, injected source or hardware API. */
(function (global) {
  'use strict';
  function createCompetitionClient(options) {
    const doc=options.document, win=options.window, http=options.fetch;
    const later=options.setTimeout||setTimeout, every=options.setInterval||setInterval;
    const clearLater=options.clearTimeout||clearTimeout, now=options.now||Date.now;
    const bootstrap=options.bootstrap;
    const client=options.clientId||Array.from(global.crypto.getRandomValues(new Uint8Array(16)),x=>x.toString(16).padStart(2,'0')).join('');
    const el=id=>doc.getElementById(id), held=new Map();
    let status=null, online=false, stale=false, busy=false, manualOwned=false, manualToken=null;
    let raceToken=null, raceSequence=-1, epoch=0, lastSnapshot=-Infinity, manualSequence=0;
    let raceQueue=Promise.resolve(), pulseBusy=false, motionBusy=false, motionGeneration=0;
    let releaseRequired=true, stopped=false, started=false, gamepadNeutral=false;
    let speedSequence=-1,lastSpeedAck=-1,speedToken=null,speedWanted=null,speedInFlight=null,speedBusy=false;
    const labels={handoff:'正在切换控制',arming:'自动模式初始化',wait_board:'等待发车挡板移开',crosswalk:'斑马线停车与播报',traffic:'红绿灯',cones:'两桶避障',parking_search:'寻找停车区',parking_entry:'停车入位',payment:'30 秒扫码缴费',complete:'流程结束',fault:'已停止，需检查',cancelled:'已结束自主驾驶'};
    const message=text=>{el('message').textContent=text||'';};
    const race=()=>status?.race_flow||{};
    const raceOwned=()=>!!race().active&&race().owner_client===client;
    const foreignTask=()=>['crosswalk_trial','traffic_trial','departure_trial','parking_trial','plan_b'].some(k=>status?.[k]?.active)||!!status?.cone_trial?.avoidance?.active||!!status?.cone_trial?.short_test?.active;
    const manualReady=()=>online&&!stale&&!busy&&!race().active&&!foreignTask()&&!status?.parking_payment?.motion_locked&&manualOwned&&status?.mode==='enabled';
    const speedReady=()=>manualReady()&&status?.manual_speed_adjustable===true;
    const clearInput=()=>{held.clear();releaseRequired=true;motionGeneration++;gamepadNeutral=false;paintDirections();};
    function superseded() {const error=new Error('本轮控制已结束');error.superseded=true;return error;}
    async function request(action, body={}, keepalive=false) {
      const controller=new AbortController(), timer=later(()=>controller.abort(),2000);
      try {
        const response=await http('/api/drive/'+action,{method:'POST',cache:'no-store',keepalive,signal:controller.signal,
          headers:{'Content-Type':'application/json','X-Drive-Key':bootstrap.key},
          body:JSON.stringify({client,setup_token:status?.setup_token,trial_token:status?.trial_token,...body})});
        const result=await response.json();
        if(!response.ok){if(result.code==='control_page_rejected'){stale=true;clearInput();render();}throw new Error(result.error||'操作未确认');}
        return result;
      } finally {clearLater(timer);}
    }
    function applyStatus(next, expectedEpoch) {
      if(expectedEpoch!==undefined&&expectedEpoch!==epoch)return false;
      if(next.control_session_id&&next.control_session_id!==bootstrap.session){stale=true;clearInput();render();message('控制服务已更新，请刷新页面。');return false;}
      const snapshot=next.application_status_s??next.race_flow_status_s;
      if(typeof snapshot==='number'&&snapshot<lastSnapshot)return false;
      const nextSpeedToken=next.trial_token||null,ack=next.last_speed_sequence;
      if(nextSpeedToken===speedToken&&typeof ack==='number'&&ack<lastSpeedAck)return false;
      if(typeof snapshot==='number')lastSnapshot=snapshot;
      if(nextSpeedToken!==speedToken){speedToken=nextSpeedToken;speedSequence=lastSpeedAck=typeof ack==='number'?ack:-1;speedWanted=null;}
      else if(typeof ack==='number'){lastSpeedAck=Math.max(lastSpeedAck,ack);speedSequence=Math.max(speedSequence,ack);}
      const nextToken=next.race_flow?.run_token||null;
      if(nextToken!==raceToken){raceToken=nextToken;raceSequence=-1;epoch++;}
      const claimed=next.manual_owner_client??next.owner_client;
      if(claimed!==undefined&&claimed!==null)manualOwned=claimed===client;
      else if(manualToken&&next.trial_token&&next.trial_token!==manualToken)manualOwned=false;
      if(next.race_flow?.active){manualOwned=false;held.clear();releaseRequired=true;}
      if(next.mode==='disabled'&&next.boot_test_phase==='esc_off')manualOwned=false;
      manualSequence=Math.max(manualSequence,next.last_command_sequence??-1);
      status=next;render();return true;
    }
    function missingItems() {
      const r=race(), ready=r.readiness, result=[];
      const collect=value=>{if(Array.isArray(value))value.forEach(collect);else if(typeof value==='string')result.push(value);else if(value&&typeof value==='object'){
        if(value.ready===false||value.ok===false){if(value.reason||value.label)result.push(value.reason||value.label);}
        if(value.missing)collect(value.missing);
        if(value.checks)collect(value.checks);
        if(value.blockers)collect(value.blockers);
        if(value.items)collect(value.items);
        if(value.groups)Object.values(value.groups).forEach(collect);
      }};
      // The explicit blocker list is authoritative; group titles explain it.
      collect(Array.isArray(ready?.missing)?ready.missing:ready);collect(r.parking_missing);
      if(r.configuration_error)result.push(r.configuration_error);
      return Array.from(new Set(result));
    }
    function paintDirections() {
      const values=new Set(held.values());
      for(const node of doc.querySelectorAll('[data-direction]')){
        node.classList.toggle('active',values.has(node.dataset.direction));
        node.disabled=!manualReady()||(node.dataset.direction==='reverse'&&status?.reverse_available===false);
      }
    }
    function render() {
      const r=race(), own=raceOwned(), active=!!r.active, payment=['payment','complete'].includes(r.stage);
      const debugManual=status?.application_mode==='debug';
      el('auto-section').hidden=debugManual;el('payment-section').hidden=debugManual;
      el('debug-back').hidden=!debugManual;
      el('manual-help').textContent=debugManual?'调试遥控页：准备后使用键盘、屏幕按钮或手柄。':'遥控到切换区，停稳后切入自主驾驶。';
      if(debugManual){doc.title='启航队 · 5G 遥控调试';const heading=doc.querySelector?.('h1');if(heading)heading.textContent='5G 遥控调试';}
      if(debugManual)el('handoff-state').textContent='调试遥控页不提供比赛流程入口';
      el('connection').textContent=stale?'页面已过期，请刷新':online?'已连接':'连接中断';
      el('connection').dataset.state=online&&!stale?'online':'offline';
      const phase=status?.boot_test_phase, reenableManual=phase==='ready'&&status?.mode==='disabled';
      el('manual-state').textContent=active?(own?'自主驾驶已接管':'另一页面正在自主驾驶，本页仅观察'):status?.reason||'等待车辆状态';
      el('prepare-manual').textContent=reenableManual?'电调已打开且静止 · 重新启用遥控':['esc_off','fault'].includes(phase)?'电调已关闭 · 准备遥控':phase==='esc_on'?'电调已打开且静止 · 启用遥控':phase==='ready'?(status?.mode==='settling'?'正在启用遥控…':'遥控已准备'):'正在准备遥控…';
      el('prepare-manual').disabled=!online||stale||busy||active||foreignTask()||status?.parking_payment?.motion_locked||(!['esc_off','esc_on','steering_ready'].includes(phase)&&!reenableManual);
      el('reset-manual').hidden=phase!=='fault'&&status?.mode!=='emergency';
      el('reset-manual').disabled=!online||stale||busy||active;
      const selectedSpeed=speedWanted??speedInFlight??status?.speed_percent;
      for(const id of ['forward-speed-slider','forward-speed-number']){el(id).disabled=!speedReady();if(typeof selectedSpeed==='number')el(id).value=String(selectedSpeed);}
      el('forward-speed-state').textContent=typeof status?.speed_percent==='number'?
        `已选 ${status.speed_percent}%${status.selected_forward_pulse_us!=null?' / '+status.selected_forward_pulse_us+' μs':''} · 当前输出 ${status.motor_pulse_us??'—'} μs`+(speedBusy?' · 正在确认新档位':''):
        '等待读取已保存档位；打开网页不会修改油门。';
      const missing=missingItems(), readiness=r.readiness, startReadiness=r.start_readiness;
      const startReady=typeof startReadiness?.ready==='boolean'?startReadiness.ready:readiness?.ready;
      const parkingPending=startReady===true&&(startReadiness?.parking_ready===false||startReadiness?.scope==='through_cones');
      el('enter-auto').disabled=debugManual||!online||stale||busy||active||foreignTask()||!r.can_enter||startReady===false;
      el('enter-auto').textContent=busy&&!active?'正在切入自主驾驶…':'已在切换区停稳 · 切入自主驾驶（F/B）';
      el('exit-auto').disabled=!online||stale||!own;
      el('handoff-state').textContent=r.stage==='handoff'?(r.reason||'正在停止遥控并释放原控制，随后接管。'):active?(own?'本页持有自主驾驶控制权。':'另一页面持有控制权，本页只显示状态。'):busy?'正在请求控制切换，请保持车辆静止。':'';
      el('race-stage').textContent=r.stage_label||labels[r.stage]||'尚未启动';
      el('race-reason').textContent=r.reason||'';
      el('race-output').textContent=r.intent?`油门 ${r.intent.esc_us} μs · 转向 ${r.intent.steering_us} μs`:'';
      for(const item of doc.querySelectorAll('[data-stage]'))item.classList.toggle('active',item.dataset.stage.split(' ').includes(r.stage));
      el('readiness-title').textContent=parkingPending?`停车条件待补${missing.length?`（${missing.length}项）`:''} · 前段可启动，两桶避障后停车`:
        startReadiness?.ready===false?'前段启动条件未就绪':missing.length?`自主驾驶仍缺 ${missing.length} 项条件`:(readiness?.ready===true?'自主驾驶条件已齐备':'等待自主驾驶条件检查');
      el('readiness-missing').replaceChildren();
      for(const text of missing){const item=doc.createElement('li');item.textContent=text;el('readiness-missing').append(item);}
      el('readiness-details').open=missing.length>0;
      el('payment-state').textContent=payment?(r.reason||'停车付款'): '等待正确入位和停稳确认';
      el('payment-time').textContent=r.payment_remaining_s==null?'尚未开始':r.stage==='complete'?'已结束':Number(r.payment_remaining_s).toFixed(1)+' 秒';
      el('report-paid').disabled=!online||stale||busy||!own||!r.can_report_paid;
      el('payment-receipt').textContent=r.operator_reported_paid?'已记录队员付款声明，请展示微信成功页；未自动核验到账。':'';
      const p=status?.parking_payment||{};
      el('scan-payment').textContent=p.scanning?'停止识别付款码':'识别付款码';
      el('scan-payment').disabled=!online||stale||busy||!own||!payment||(!p.can_scan&&!p.scanning);
      el('payment-scan-state').textContent=p.scan_reason||'';
      el('payment-code').hidden=!p.selected;
      if(p.selected)el('payment-code').src='/api/payment/image.jpg?id='+encodeURIComponent(p.selected.id);
      el('payment-candidates').replaceChildren();
      for(const c of p.candidates||[]){const button=doc.createElement('button'),image=doc.createElement('img');
        image.src='/api/payment/image.jpg?id='+encodeURIComponent(c.id);image.alt='相机识别的付款码候选';button.append(image);
        button.disabled=!own||!payment||p.scanning||!c.decoded;
        button.addEventListener('click',()=>runAction(()=>request('payment_select',{candidate_id:c.id}).then(applyStatus)));
        el('payment-candidates').append(button);}
      paintDirections();
    }
    function raceRequest(action, body={}) {
      const token=raceToken, generation=epoch;
      const run=async()=>{
        if(generation!==epoch||token!==raceToken||!raceOwned())throw superseded();
        const result=await request(action,{...body,run_token:token,race_sequence:++raceSequence});
        if(generation!==epoch||token!==raceToken)throw superseded();
        applyStatus(result,generation);return result;
      };
      const result=raceQueue.then(run,run);raceQueue=result.catch(()=>{});return result;
    }
    async function runAction(work) {
      if(busy||stale||!online)return;
      busy=true;message('');render();
      try{return await work();}catch(error){if(!error.superseded)message(error.name==='AbortError'?'连接超时，请核对车辆状态。':error.message);}
      finally{busy=false;render();}
    }
    async function enterAuto() {
      if(el('enter-auto').disabled||status?.application_mode==='debug')return;
      clearInput();
      return runAction(async()=>{
        const result=await request('race_enter',{stationary_confirmed:true,in_switch_zone:true,fb_mode_confirmed:true});
        applyStatus(result);
      });
    }
    async function stopOwned(explicit=false, source='stop_button', keepalive=false, exit=false) {
      const ownAuto=raceOwned(), ownManual=manualOwned;
      clearInput();
      if(!explicit&&!ownAuto&&!ownManual)return;
      if(!explicit&&ownAuto&&['payment','complete'].includes(race().stage)&&!exit)return;
      epoch++;stopped=true;
      try{applyStatus(await request(explicit?'emergency':ownAuto?'race_cancel':'stop',{stop_source:source},keepalive));}
      catch(error){message('停车确认未收到，请核对车辆：'+error.message);}
      finally{stopped=false;render();}
    }
    async function pulse() {
      if(status?.application_mode==='debug'||!online||stale||doc.hidden||pulseBusy||stopped||!raceOwned()||['payment','complete'].includes(race().stage))return;
      pulseBusy=true;const generation=epoch;
      try{await raceRequest('race_keepalive');}
      catch(error){if(!error.superseded&&generation===epoch){message(error.message);await stopOwned(false,'auto_connection_lost');}}
      finally{pulseBusy=false;}
    }
    async function motion() {
      if(!manualReady()||motionBusy||stopped||!held.size||doc.hidden)return;
      const values=new Set(held.values());
      if(values.has('forward')&&values.has('reverse')||values.has('left')&&values.has('right'))return stopOwned(false,'conflicting_keys');
      motionBusy=true;const generation=motionGeneration;
      try{
        if(releaseRequired){const result=await request('command',{sequence:++manualSequence,motor:'stop',steering:'center'});
          if(generation!==motionGeneration)return;applyStatus(result);releaseRequired=false;}
        if(generation!==motionGeneration||!manualReady()||!held.size)return;
        const current=new Set(held.values());
        const result=await request('command',{sequence:++manualSequence,motor:current.has('forward')?'forward':current.has('reverse')?'reverse':'stop',
          steering:current.has('left')?'left':current.has('right')?'right':'center',input_source:Array.from(held.keys()).some(k=>k.startsWith('pad:'))?'gamepad':'keyboard'});
        if(generation===motionGeneration)applyStatus(result);
      }catch(error){if(generation===motionGeneration){message(error.message);await stopOwned(false,'manual_connection_lost');}}
      finally{motionBusy=false;}
    }
    async function changeSpeed(value) {
      if(!speedReady())return;
      const wanted=Number(value);
      if(!Number.isInteger(wanted)||wanted<0||wanted>100){message('前进油门请输入0–100的整数');render();return;}
      speedWanted=wanted;render();
      if(speedBusy)return;
      speedBusy=true;const session=speedToken;
      try{
        while(speedWanted!==null){
          const target=speedWanted;speedWanted=null;speedInFlight=target;
          if(target===0){clearInput();await stopOwned(false,'speed_zero');}
          if(session!==speedToken||!speedReady()){speedWanted=null;break;}
          const body={speed_percent:target,speed_sequence:++speedSequence};
          if(Number.isInteger(status.reverse_speed_percent))body.reverse_speed_percent=status.reverse_speed_percent;
          const reply=await request('speed',body);
          if(session!==speedToken||race().active){speedWanted=null;break;}
          applyStatus(reply);
          speedInFlight=null;
        }
      }catch(error){speedWanted=null;message('油门设置未确认：'+error.message);await stopOwned(false,'speed_failure');}
      finally{speedBusy=false;speedInFlight=null;render();}
    }
    function press(source,direction) {
      if(!manualReady()||direction==='reverse'&&status?.reverse_available===false)return;
      held.set(source,direction);paintDirections();void motion();
    }
    function release(source) {if(!held.has(source))return;held.delete(source);paintDirections();
      if(!held.size)void stopOwned(false,'key_release');else void motion();}
    async function poll() {
      const controller=new AbortController(),timer=later(()=>controller.abort(),2000);
      try{const response=await http('/api/drive/status',{cache:'no-store',signal:controller.signal});
        if(!response.ok)throw new Error('状态读取失败');const next=await response.json();online=true;applyStatus(next);}
      catch(error){online=false;render();if(manualOwned||raceOwned())void stopOwned(false,'status_connection_lost');}
      finally{clearLater(timer);}
    }
    function gamepad() {
      if(!el('gamepad-enable').checked||!manualReady()||doc.hidden||!doc.hasFocus()){gamepadNeutral=false;return;}
      const pad=Array.from((options.navigator||global.navigator)?.getGamepads?.()||[]).find(Boolean);
      if(!pad){el('gamepad-state').textContent='尚未连接手柄';gamepadNeutral=false;return;}
      const axis=pad.axes?.[0]||0, forward=pad.buttons?.[7]?.value||0, reverse=pad.buttons?.[6]?.value||0;
      const neutral=Math.abs(axis)<.25&&forward<.2&&reverse<.2;
      if(!gamepadNeutral){gamepadNeutral=neutral;el('gamepad-state').textContent=neutral?'手柄已就绪':'先将摇杆和扳机回中位';return;}
      const directions=[];
      if(axis<-.35)directions.push('left');if(axis>.35)directions.push('right');if(forward>.2)directions.push('forward');if(reverse>.2&&status.reverse_available!==false)directions.push('reverse');
      const before=Array.from(held).filter(([k])=>k.startsWith('pad:')).map(([,v])=>v).sort().join(',');
      if(before===directions.slice().sort().join(','))return;
      for(const key of Array.from(held.keys()))if(key.startsWith('pad:'))held.delete(key);
      directions.forEach(d=>held.set('pad:'+d,d));paintDirections();if(held.size)void motion();else void stopOwned(false,'gamepad_release');
    }
    function camera(id,url) {
      const image=el('camera-'+id),label=el('camera-'+id+'-state');
      if(doc.hidden||id==='auto'&&!race().active){image.hidden=true;later(()=>camera(id,url),400);return;}
      const pending=doc.createElement('img');let ended=false;
      const finish=ok=>{if(ended)return;ended=true;clearLater(timer);pending.onload=pending.onerror=null;
        if(ok){image.src=pending.src;image.hidden=doc.hidden;label.textContent='刚刚更新';}else{image.hidden=true;label.textContent='画面已失效';}
        later(()=>camera(id,url),ok?150:400);};
      const timer=later(()=>{finish(false);pending.removeAttribute('src');},1200);
      pending.onload=()=>finish(true);pending.onerror=()=>finish(false);pending.src=url+(url.includes('?')?'&':'?')+'t='+now();
    }
    const keyDirections={KeyW:'forward',ArrowUp:'forward',KeyS:'reverse',ArrowDown:'reverse',KeyA:'left',ArrowLeft:'left',KeyD:'right',ArrowRight:'right'};
    function bind() {
      el('prepare-manual').addEventListener('click',()=>{if(el('prepare-manual').disabled)return;return runAction(async()=>{
        const result=await request('enable',{bench_ready:true,esc_state:status?.boot_test_phase==='esc_off'?'off':'on_static'});
        manualOwned=true;manualToken=result.trial_token||null;applyStatus(result);
      });});
      el('reset-manual').addEventListener('click',()=>runAction(async()=>applyStatus(await request('reset',{bench_ready:true,esc_state:'off'}))));
      el('enter-auto').addEventListener('click',enterAuto);
      el('exit-auto').addEventListener('click',()=>stopOwned(false,'exit_auto',false,true));
      el('emergency').addEventListener('click',()=>stopOwned(true,'emergency_button'));
      el('manual-stop').addEventListener('click',()=>stopOwned(false,'stop_button'));
      el('report-paid').addEventListener('click',()=>{if(!el('report-paid').disabled)void runAction(()=>raceRequest('race_report_paid',{operator_paid:true}));});
      el('scan-payment').addEventListener('click',()=>runAction(async()=>applyStatus(await request(status?.parking_payment?.scanning?'payment_scan_stop':'payment_scan'))));
      el('gamepad-enable').addEventListener('change',()=>{gamepadNeutral=false;void stopOwned(false,'gamepad_switch');});
      el('forward-speed-slider').addEventListener('input',()=>void changeSpeed(el('forward-speed-slider').value));
      el('forward-speed-number').addEventListener('change',()=>void changeSpeed(el('forward-speed-number').value));
      for(const button of doc.querySelectorAll('[data-direction]')){
        button.addEventListener('pointerdown',e=>{e.preventDefault();button.setPointerCapture?.(e.pointerId);press('pointer:'+e.pointerId,button.dataset.direction);});
        for(const event of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(event,e=>release('pointer:'+e.pointerId));
      }
      win.addEventListener('keydown',e=>{if(e.code==='Space'||e.code==='Escape'){e.preventDefault();void stopOwned(true,'keyboard_emergency');return;}
        if(e.target&&['INPUT','SELECT','TEXTAREA','BUTTON'].includes(e.target.tagName))return;
        if(keyDirections[e.code]&&!el('gamepad-enable').checked){e.preventDefault();press('key:'+e.code,keyDirections[e.code]);}});
      win.addEventListener('keyup',e=>{if(keyDirections[e.code]){e.preventDefault();release('key:'+e.code);}});
      win.addEventListener('blur',()=>void stopOwned(false,'window_blur'));
      win.addEventListener('pagehide',()=>void stopOwned(false,'page_exit',true));
      doc.addEventListener('visibilitychange',()=>{if(doc.hidden){for(const id of ['primary','secondary','auto'])el('camera-'+id).hidden=true;void stopOwned(false,'page_hidden');}});
    }
    function start() {if(started)return;started=true;bind();render();void poll();every(poll,700);every(pulse,100);every(motion,90);every(gamepad,50);
      camera('primary','/frame.jpg?camera=primary&view=raw');camera('secondary','/frame.jpg?camera=secondary&view=raw');camera('auto','/api/race-flow/frame.jpg');}
    return {start,bind,poll,render,applyStatus,raceRequest,enterAuto,stopOwned,pulse,motion,press,release,changeSpeed,client,
      inspect:()=>({status,online,stale,busy,epoch,raceSequence,manualOwned})};
  }
  if(typeof module!=='undefined'&&module.exports)module.exports={createCompetitionClient};
  else {const bootstrap=JSON.parse(document.getElementById('control-bootstrap').textContent);
    createCompetitionClient({document,window,fetch:window.fetch.bind(window),bootstrap}).start();}
})(globalThis);
