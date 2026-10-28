"""Browser gamepad: fresh RB presses toggle ownership; input leases stay finite."""

SCRIPT = r'''
function createUSBGamepadControls(api){
 const {arm,info}=api;
 let selected=null,active=[],neutralRequired=true,lastSample=null,rbDown=true,emergencyDown=true;
 const pressed=b=>b.pressed||b.value>0.5;
 function requireNeutral(){neutralRequired=true;active=[];}
 function halt(reason,emergency=false){
  const moving=active.length>0;requireNeutral();
  if(moving||emergency)api.stop(reason,emergency);
 }
 function disarm(reason,emergency=false){
  arm.checked=false;
  halt(reason,emergency);
 }
 function read(p){
  if(!p||p.connected!==true||p.mapping!=='standard'||!Array.isArray(p.axes)||!Array.isArray(p.buttons)||p.axes.length<2||p.buttons.length<17)return null;
  if(!p.axes.every(v=>Number.isFinite(v)&&v>=-1&&v<=1))return null;
  if(!p.buttons.every(b=>b&&typeof b.pressed==='boolean'&&Number.isFinite(b.value)&&b.value>=0&&b.value<=1))return null;
  const x=p.axes[0],forward=p.buttons[7].value,reverse=p.buttons[6].value;
  return {x,forward,reverse,rb:pressed(p.buttons[5]),emergency:pressed(p.buttons[1]),
   neutral:Math.abs(x)<=0.15&&forward<=0.08&&reverse<=0.08&&!pressed(p.buttons[1])};
 }
 function setOwnership(wanted,state,input){
  if(!wanted){disarm('gamepad_toggle_off');info.textContent='手柄待机 · RB 接管';return;}
  if(!state.allowed||state.stale||!state.focused){arm.checked=false;info.textContent='车辆未就绪';return;}
  if(state.otherInput){arm.checked=false;api.stop('gamepad_mode_change',false);info.textContent='先松开键盘或屏幕方向';return;}
  if(!input.neutral){arm.checked=false;info.textContent='松开扳机、摇杆回中，再按 RB';return;}
  arm.checked=true;neutralRequired=false;active=[];info.textContent='手柄已接管 · RB 退出';
 }
 arm.checked=false;
 arm.onchange=()=>{
  const wanted=arm.checked;arm.checked=!wanted;
  try{
   const pads=Array.from(api.getPads?.()||[]).filter(p=>p&&p.connected),input=pads.length===1?read(pads[0]):null;
   if(!input){disarm('gamepad_invalid');info.textContent='请连接一个标准手柄';return;}
   selected={index:pads[0].index,id:pads[0].id};rbDown=input.rb;emergencyDown=input.emergency;
   setOwnership(wanted,api.state(),input);
  }catch(e){disarm('gamepad_read_failure');}
 };
 function sampleUnchecked(){
  const now=api.now(),state=api.state();
  if(lastSample!==null&&now-lastSample>=200){disarm('gamepad_poll_gap');rbDown=true;}
  lastSample=now;
  let pads;
  try{
   if(typeof api.getPads!=='function')throw new Error('unavailable');
   pads=Array.from(api.getPads()||[]).filter(p=>p&&p.connected);
  }catch(e){disarm('gamepad_read_failure');arm.disabled=true;info.textContent='浏览器无法读取手柄';return;}
  arm.disabled=false;
  if(pads.length!==1){disarm('gamepad_disconnected');selected=null;rbDown=true;emergencyDown=true;
   info.textContent=pads.length?'请只连接一个手柄':'未检测到手柄';return;}
  const pad=pads[0],input=read(pad);
  if(!input){disarm('gamepad_invalid');selected=null;info.textContent='手柄映射不兼容';return;}
  if(!selected||selected.index!==pad.index||selected.id!==pad.id){
   disarm('gamepad_device_changed');selected={index:pad.index,id:pad.id};
   rbDown=input.rb;emergencyDown=input.emergency;
  }
  const rbEdge=input.rb&&!rbDown,emergencyEdge=input.emergency&&!emergencyDown;
  rbDown=input.rb;emergencyDown=input.emergency;
  if(state.stale||!state.focused){disarm('gamepad_focus_lost');info.textContent='手柄待机 · 返回页面后按 RB';return;}
  if(emergencyEdge){disarm('gamepad_emergency',true);info.textContent='已请求急停';return;}
  if(rbEdge){setOwnership(!arm.checked,state,input);return;}
  if(!arm.checked){info.textContent='已连接 · RB 接管';return;}
  // A pending stop acknowledgement must not change input ownership.
  if(!state.allowed){
   if(state.ready!==true)disarm('gamepad_not_ready');
   else requireNeutral();
   info.textContent=arm.checked?'等待停车确认':'车辆未就绪 · 手柄待机';return;
  }
  if(state.otherInput){disarm('gamepad_mixed_input');info.textContent='检测到其他方向输入';return;}
  if(neutralRequired){
   if(input.neutral){neutralRequired=false;info.textContent='手柄已接管 · RB 退出';}
   else info.textContent='松开扳机、摇杆回中';
   return;
  }
  const f=input.forward>(active.includes('forward')?0.15:0.25);
  const r=input.reverse>(active.includes('reverse')?0.15:0.25);
  if(f&&r){halt('gamepad_conflict');info.textContent='两只扳机冲突，请松开';return;}
  const directions=[];
  if(f)directions.push('forward');if(r)directions.push('reverse');
  if(input.x<-(active.includes('left')?0.15:0.25))directions.push('left');
  if(input.x>(active.includes('right')?0.15:0.25))directions.push('right');
  if(directions.some(d=>!state.capabilities[d])){halt('gamepad_unavailable_direction');info.textContent='当前模式不支持该动作';return;}
  if(directions.join(',')!==active.join(',')){
   if(!directions.length){halt('gamepad_release');info.textContent='手柄已接管 · 停车';return;}
   active=directions;api.directions(directions);info.textContent='手柄已接管 · RB 退出';
  }
 }
 function sample(){try{sampleUnchecked();}catch(e){disarm('gamepad_read_failure');info.textContent='手柄读取异常';}}
 return {sample,requireNeutral,get armed(){return arm.checked;}};
}
'''
