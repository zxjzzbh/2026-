"""Optional browser gamepad input, routed through the existing finite controls.

Only the browser's standard mapping is accepted. No device output or new
vehicle endpoint is introduced. Actual device/browser compatibility is a
separate field check.
"""

SCRIPT = r'''
function createUSBGamepadControls(api){
 const {arm,info}=api;
 let selected=null, active=[], neutralRequired=true, lastSample=null, emergencyDown=false;
 const pressed=b=>b.pressed||b.value>0.5;
 function requireNeutral(){neutralRequired=true;active=[];}
 function halt(reason,emergency=false){
  const moving=active.length>0;requireNeutral();
  if(moving||emergency)api.stop(reason,emergency);
 }
 function disarm(reason){halt(reason);arm.checked=false;selected=null;}
 function read(p){
  if(!p||p.connected!==true||p.mapping!=='standard'||!Array.isArray(p.axes)||!Array.isArray(p.buttons)||p.axes.length<2||p.buttons.length<17)return null;
  if(!Array.from(p.axes).every(v=>Number.isFinite(v)&&v>=-1&&v<=1))return null;
  if(!Array.from(p.buttons).every(b=>b&&typeof b.pressed==='boolean'&&Number.isFinite(b.value)&&b.value>=0&&b.value<=1))return null;
  const x=p.axes[0],forward=p.buttons[7].value,reverse=p.buttons[6].value;
  return {x,forward,reverse,permit:pressed(p.buttons[5]),emergency:pressed(p.buttons[1]),
          neutral:Math.abs(x)<=0.15&&forward<=0.08&&reverse<=0.08&&!pressed(p.buttons[5])&&!pressed(p.buttons[1])};
 }
 arm.checked=false;
 arm.onchange=()=>{
  if(!arm.checked){disarm('gamepad_disarmed');info.textContent='手柄控制已关闭。';return;}
  const state=api.state();
  if(state.otherInput){api.stop('gamepad_mode_change',false);}
  selected=null;requireNeutral();emergencyDown=false;
  info.textContent='先松开 RB、扳机和摇杆，等待手柄回中确认。';
 };
 function sampleUnchecked(){
  const now=api.now(),state=api.state();
  if(lastSample!==null&&now-lastSample>=200){halt('gamepad_poll_gap');}
  lastSample=now;
  let pads;
  try{
   if(typeof api.getPads!=='function')throw new Error('unavailable');
   pads=Array.from(api.getPads()||[]).filter(p=>p&&p.connected);
  }catch(e){disarm('gamepad_read_failure');arm.disabled=true;info.textContent='当前浏览器无法读取手柄，请检查浏览器支持和页面权限。';return;}
  arm.disabled=false;
  if(!arm.checked){info.textContent=pads.length===1?(read(pads[0])?'已检测到 '+pads[0].id+'；勾选后才接管方向。':'已检测到 '+pads[0].id+'，但映射或输入不符合要求，需要单独适配。'):pads.length>1?'检测到多个手柄，请只保留本次使用的一个。':'未检测到手柄：插入 USB 后按一下手柄按钮。';return;}
  if(pads.length!==1){disarm('gamepad_disconnected');info.textContent='手柄断开或设备数量改变；控制已关闭，需重新勾选。';return;}
  const pad=pads[0];
  if(selected&&(selected.index!==pad.index||selected.id!==pad.id)){
   disarm('gamepad_device_changed');info.textContent='手柄设备改变；控制已关闭，需重新勾选。';return;
  }
  const input=read(pad);
  if(!input){disarm('gamepad_invalid');info.textContent='手柄映射不是 standard，或输入无效；需要单独适配，当前不接管。';return;}
  if(!selected)selected={index:pad.index,id:pad.id};
  if(state.stale||!state.focused){halt('gamepad_focus_lost');info.textContent='页面未获得焦点或已过期；已停止手柄请求，回中后再操作。';return;}
  const emergencyEdge=input.emergency&&!emergencyDown;emergencyDown=input.emergency;
  if(emergencyEdge){halt('gamepad_emergency',true);info.textContent='已发出手柄急停请求；请观察车辆停车。';return;}
  if(!state.allowed){requireNeutral();info.textContent='手柄已识别；先启用本轮测试并等待就绪，再松开全部控制。';return;}
  if(state.otherInput){const moving=active.length>0;halt('gamepad_mixed_input');if(!moving)api.stop('gamepad_mixed_input',false);info.textContent='检测到其他方向输入；请松开全部控制后再操作。';return;}
  if(neutralRequired){
   if(input.neutral){neutralRequired=false;info.textContent='手柄已回中：按住 RB，RT 前进、LT 倒车，左摇杆转向，B 急停。';}
   else info.textContent='请松开 RB、两只扳机并让左摇杆回中；保持按住不会重新启动。';
   return;
  }
  if(!input.permit){halt('gamepad_deadman_release');info.textContent='RB 已松开；停车请求已回零，回中后再操作。';return;}
  const f=input.forward>(active.includes('forward')?0.15:0.25);
  const r=input.reverse>(active.includes('reverse')?0.15:0.25);
  if(f&&r){halt('gamepad_conflict');info.textContent='两只扳机同时按下；请全部松开后再操作。';return;}
  const directions=[];
  if(f)directions.push('forward');if(r)directions.push('reverse');
  if(input.x<-(active.includes('left')?0.15:0.25))directions.push('left');
  if(input.x>(active.includes('right')?0.15:0.25))directions.push('right');
  if(directions.some(d=>!state.capabilities[d])){halt('gamepad_unavailable_direction');requireNeutral();info.textContent='本轮没有开放该动作；请全部松开后再操作。';return;}
  // Match keyboard short-trial release behavior: releasing any component stops
  // the chord and requires a neutral input before starting another short trial.
  if(active.some(d=>!directions.includes(d))){halt('gamepad_release');return;}
  if(directions.join(',')!==active.join(',')){
   active=directions;api.directions(directions);
   info.textContent='手柄方向已提交；沿用本轮固定档位和短测时限，扳机幅度不代表真实速度。';
  }
 }
 function sample(){
  try{sampleUnchecked();}catch(e){disarm('gamepad_read_failure');info.textContent='手柄读取异常，控制已关闭；请检查设备后重新勾选。';}
 }
 return {sample,requireNeutral,get armed(){return arm.checked;}};
}
'''
