"""Optional real-camera cone observation card on the existing driving console."""

HTML = '''<section class="drive-panel" id="cone-test">
<h3>蓝色锥桶 · 交错绕行</h3>
<p id="cone-state" role="status">等待锥桶识别服务</p>
<p>第二路 H65 为主视角，第一路辅助观察。两路画面中的同一锥桶不会直接计作两个。</p>
<p>两侧白线内侧为边界，整车含轮胎不得越线。蓝色用于找锥桶；红色跑道模式还会检查地面颜色。</p>
<label>场地<select id="cone-surface"><option value="red_running_track">红色跑道 + 白线</option><option value="indoor_white_line_track">室内赛道 + 白线（地面不限颜色）</option></select></label>
<button id="cone-save-surface" type="button">保存场地选择</button>
<p>切换场地先停止识别。室内模式仍检查左右白线、实测标定和停车保护；不会自动放宽行驶范围。</p>
<div class="cone-actions"><button type="button" id="cone-observe">开始识别</button>
<button type="button" id="cone-stop">停止识别</button></div>
<p>“开始识别”只观察与选路。实车测试使用下面的“准备避障”和“开始避障”。</p>
<div class="cone-actions"><button type="button" id="cone-prepare">准备避障（电调关闭）</button>
<button type="button" id="cone-drive-start" disabled>开始避障</button></div>
<button type="button" id="cone-drive-stop">停止避障</button>
<p id="cone-run-state" role="status"></p><p id="cone-navigation" role="status"></p>
<p id="cone-slalom" role="status">等待确认两只锥桶及交错路线</p>
<p>必须按“第一桶左侧 → 第二桶右侧”或相反顺序通过，再回归车道。左右指车从锥桶的哪一侧经过。路线不可行时停车，不退回同侧绕行。</p>
<p>首次准备时保持 F/B、电调关闭；准备完成后打开电调，确认车辆静止、场地空旷，再点“开始避障”。3 秒中位后低速绕行，完成两桶后停车；空格／Esc 可停止。</p>
<div class="cone-viewer"><img id="cone-preview" alt="实际双摄锥桶识别：左第一路，右第二路 H65" width="960" height="360" hidden>
<span id="cone-placeholder">开始识别后显示真实画面</span></div>
<p id="cone-cameras"></p>
<details id="cone-low-speed-section" open><summary>旧数据 · 低速绕桶测试（无需重新标定）</summary>
<p>沿用已有相机参考、车宽 29.5 cm、车长 42 cm、白线内宽 125 cm，以及此前约 17 cm 刹车记录。逐段前进、有限 F/B 刹车后重新看图；1570 右转／1650 左转。不会将估算参数标成已验证。</p>
<div class="cone-settings"><label>测试油门（μs）<input id="cone-test-forward" type="number" min="1501" max="1600" step="1" value="1600"></label>
<label>每段前进（秒）<input id="cone-test-burst" type="number" min="0.15" max="0.4" step="0.05" value="0.25"></label></div>
<p>每段后制动 0.12 秒、中位观察 0.55 秒；本轮累计通电前进最多 4 秒，整个会话最多 60 秒。任一路可靠白线即可继续；两路短暂漏检沿用最近边界最多 0.8 秒，期间前进总预算 0.15 秒。持续丢线先制动等待，靠近边界、近处蓝桶挡路、画面或页面失联会停车。看不见边界时不能保证不越线，不会无限盲走。</p>
<p id="cone-test-line-state" role="status">白线依据：等待两路画面</p>
<div class="cone-actions"><button id="cone-test-prepare" type="button">准备低速测试（电调关闭）</button><button id="cone-test-start" type="button" disabled>开始低速绕桶</button></div>
<button id="cone-test-confirm" type="button" disabled>车尾已过当前桶，继续</button>
<button id="cone-test-stop" type="button">急停并结束本轮</button>
<p id="cone-test-progress" role="status">准备后打开电调，确认车辆静止、场地就绪，再开始。第一桶离开近处视野后有限续行并停车，现场确认车尾已通过再处理第二桶；画面消失不会自动算通过。</p>
</details>
<details id="cone-short-section" open><summary>1570／1650 固定档转向与刹车短测</summary>
<p>不需要先填车身尺寸或地面标定。两项转向短测会固定 S3 后低速前进画弧；刹车短测则以 1610 μs 回中直行。均自动限时制动，不是仅原地摆舵。</p>
<div class="cone-settings">
<label>短测油门（μs）<input id="cone-short-forward" type="number" min="1501" max="1625" step="1" value="1575"></label>
<label>前进时长（秒）<input id="cone-short-duration" type="number" min="0.2" max="2" step="0.1" value="0.8"></label>
</div>
<div class="cone-actions"><button id="cone-short-prepare" type="button">准备短测（电调关闭）</button>
<button id="cone-short-stop" type="button">停止并结束短测</button></div>
<div class="cone-short-starts"><button id="cone-short-1570" type="button" disabled>1570 固定转向短测</button>
<button id="cone-short-1650" type="button" disabled>1650 固定转向短测</button>
<button id="cone-short-brake" type="button" disabled>直行 → 刹车短测</button></div>
<p id="cone-short-state" role="status">正在读取短测状态</p><p id="cone-short-output"></p>
<p>保持 F/B、电调关闭时点“准备短测”；随后打开电调，确认静止、场地空旷且可随时断电，再选一项。3 秒中位 → 0.4 秒固定转向 → 定时前进 → 最多 3 次有限刹车 → 回中。</p>
<p>一轮结束后先现场确认停稳，再点下一轮。空格／Esc／停止按钮均可结束；结束短测后恢复原遥控及红绿灯。短测参数只用于本面板。</p>
<p id="cone-short-evidence">记录车端前进、制动触发时刻及对应画面，不自动填写转弯半径或刹车距离。</p></details>
<details><summary>颜色识别调试</summary><p>H 为色相（0–179），S 为饱和度、V 为亮度（0–255）。先停止识别再保存。</p>
<div class="cone-settings">
<label>H 下限<input id="cone-h-low" type="number" min="0" max="179" value="100"></label>
<label>H 上限<input id="cone-h-high" type="number" min="0" max="179" value="135"></label>
<label>S 下限<input id="cone-s-low" type="number" min="0" max="255" value="110"></label>
<label>V 下限<input id="cone-v-low" type="number" min="0" max="255" value="50"></label>
<label>确认帧数<input id="cone-confirm" type="number" min="2" max="8" value="3"></label>
<label>红色地面辅助比例（%）<input id="cone-red-percent" type="number" min="1" max="100" value="15"></label>
<button type="button" id="cone-save">保存参数</button></div></details>
<details><summary>现场尺寸填写（cm）</summary><p>待测项目留空。车宽含轮胎，车长含前后突出部分；后悬指后轴中心到车尾最末端。</p>
<div class="cone-settings">
<label>赛道可行驶宽度<input data-cone-geometry="lane_width_m" type="number" min="0.1" max="2000" step="0.1" placeholder="待测"></label>
<label>车辆最宽处<input data-cone-geometry="vehicle_width_m" type="number" min="0.1" max="2000" step="0.1" placeholder="待测"></label>
<label>车长<input data-cone-geometry="vehicle_length_m" type="number" min="0.1" max="2000" step="0.1" placeholder="待测"></label>
<label>后轴到车头<input data-cone-geometry="rear_axle_to_front_m" type="number" min="0.1" max="2000" step="0.1" placeholder="待测"></label>
<label>后悬长度<input data-cone-geometry="rear_overhang_m" type="number" min="0.1" max="2000" step="0.1" placeholder="待测"></label>
<label>锥桶高度<input data-cone-geometry="cone_height_m" type="number" min="0.1" max="2000" step="0.1" placeholder="待测"></label>
<label>锥桶底座宽／直径<input data-cone-geometry="cone_base_width_m" type="number" min="0.1" max="2000" step="0.1" placeholder="待测"></label>
<label>车身与障碍余量<input data-cone-geometry="clearance_m" type="number" min="0.1" max="2000" step="0.1" placeholder="待定"></label>
<button type="button" id="cone-save-dimensions">保存全部参数</button></div></details>
<details><summary>避障速度与运动标定</summary><div class="cone-settings">
<label>低速油门（μs）<input data-cone-drive="forward_us" type="number" min="1501" max="1625" step="1"></label>
<label>最长行驶（秒）<input data-cone-drive="max_run_s" type="number" min="1" max="60" step="1"></label>
<label>速度上限（m/s）<input data-cone-drive="max_speed_mps" type="number" min="0.02" max="0.5" step="0.01"></label>
<label>路径前视距离（m）<input data-cone-drive="lookahead_m" type="number" min="0.2" max="1" step="0.05"></label>
<label>1570 μs 转弯半径（cm）<input data-cone-drive="steering_radius_1570_cm" type="number" min="-5000" max="5000" step="0.1" placeholder="左负右正，待测"></label>
<label>1650 μs 转弯半径（cm）<input data-cone-drive="steering_radius_1650_cm" type="number" min="-5000" max="5000" step="0.1" placeholder="左负右正，待测"></label>
<label>低速制动距离（cm）<input data-cone-motion="braking_distance" type="number" min="0.1" max="200" step="0.1" placeholder="待测"></label>
<label>允许路径偏差（cm）<input data-cone-motion="maximum_tracking_error" type="number" min="0.1" max="200" step="0.1" placeholder="待定"></label>
<label>光流位移误差上限（m）<input data-cone-drive="visual_error_bound_m" type="number" min="0.001" max="0.1" step="0.001" placeholder="待测"></label>
<label>光流已核对行程（m）<input data-cone-drive="visual_verified_distance_m" type="number" min="0.2" max="20" step="0.1" placeholder="待测"></label>
</div><p>转弯半径按后轴轨迹实测，左转填负值、右转填正值。当前使用已有 1570–1650 μs 转向范围；不能通过增大参数绕过输出范围。</p>
<label><input id="cone-steering-verified" type="checkbox">上述转向、制动距离已实测</label>
<label><input id="cone-odometry-verified" type="checkbox">已用尺核对地面光流的位移、转向和误差</label>
<button id="cone-save-driving" type="button">保存全部参数</button></details>
<details><summary>第二路地面标定导入</summary>
<p>仓库已有 10 月 7 日双摄地面标定。第二路旧记录为 640×480、云台 1600／1150、车头前 0.5–1.5 m；已保存 480×360 换算候选。需核对当前姿态并补白线边缘覆盖，不必把旧测量全部重做。</p>
<p>使用固定姿态的第二路 480×360 原图：至少 4 个拟合点和 2 个独立检查点。ground_m 为以后轴中心为原点、向右 x、向前 y 的米制坐标。手机照片不能代替此标定。</p>
<a href="/frame.jpg?camera=secondary&amp;view=raw" target="_blank" rel="noopener">打开第二路原图</a>
<textarea id="cone-calibration-json" rows="7" aria-label="第二路地面标定 JSON" placeholder='{"camera":"secondary","image_size":[480,360],"points":[],"check_points":[],"valid_polygon_px":[],"accepted_error_m":0.02,"measured_fixed_camera":true}'></textarea>
<button id="cone-save-calibration" type="button">校验并保存地面标定</button>
<p>每个点的格式：{"pixel":[图像x,图像y],"ground_m":[实测x,实测y]}。仅在坐标已测量、摄像头固定后设置 measured_fixed_camera=true。标定后先用“开始识别”查看推车时的位移估计，再填写误差和核对行程；确认不会启动车辆。</p></details>
<details open><summary>避障状态与待补项目</summary>
<p>识别、两桶选路、地面位移、转向与 F/B 制动已接通。缺少标定、白线、可信位移或可通过路线时停车；丢失目标不直接算通过。</p>
<p>蓝色框是锥桶候选，OK 表示多帧确认；橙色框是形状不明或被截断的蓝色物体。画面中的左／中／右不是赛道中的米制坐标。</p>
<p>绿色线为红色跑道上检测到的白线内侧；橙色线表示白线候选但红色地面未确认。室内灰／绿地面仍可检查蓝锥识别。俯视圆形目标可识别，但不据此计算距离。</p>
<p id="cone-readiness"></p></details></section>'''

STATE = '''
let coneActive=false,coneDirty=false,coneBusy=false,coneImageBusy=false,coneImageURL=null,coneError='';
const coneInputs=['cone-h-low','cone-h-high','cone-s-low','cone-v-low','cone-confirm','cone-red-percent'];
let coneDriveActive=false,coneDriveOwned=false,coneToken=null,coneSequence=0,conePulseBusy=false,coneRunQueue=Promise.resolve();
let coneLastSnapshot=-Infinity;
let coneShortActive=false,coneShortOwned=false,coneShortToken=null,coneShortRound=0,coneShortSequence=0,coneShortPulseBusy=false,coneShortStartBusy=false,coneShortQueue=Promise.resolve();
'''

SCRIPT = '''
function renderCone(s){
 const c=s.cone_trial||{};coneActive=!!c.observing;
 const a=c.avoidance||{};coneDriveActive=!!a.active;coneDriveOwned=a.owner_client===client;coneToken=a.run_token;
 const q=c.short_test||{},qo=q.output||{};
 el('cone-surface').disabled=el('cone-save-surface').disabled=stale||coneBusy||coneActive||coneDriveActive||!!q.active;
 if(!coneDirty&&!coneBusy&&c.settings)el('cone-surface').value=c.settings.scene?.surface||'red_running_track';
 coneShortActive=!!q.active;coneShortOwned=q.owner_client===client;coneShortToken=q.run_token;coneShortRound=q.round??0;
 el('cone-short-prepare').disabled=stale||coneBusy||!q.can_prepare;
 const shortLocked=stale||coneBusy||coneShortStartBusy||!q.can_start||!coneShortOwned;
 for(const id of ['cone-short-1570','cone-short-1650','cone-short-brake'])el(id).disabled=shortLocked;
 for(const id of ['cone-short-forward','cone-short-duration'])el(id).disabled=stale||coneShortStartBusy||(coneShortActive&&(!q.can_start||!coneShortOwned));
 el('cone-short-state').textContent=q.reason||'短测模块尚未加载';
 el('cone-short-output').textContent=q.active||q.round?('第 '+(q.round??0)+' 轮 · S3 '+(qo.steering_us??'—')+' μs · S4 '+(qo.esc_us??'—')+' μs'+(qo.phase==='driving'?' · 前进剩余 '+qo.remaining_s.toFixed(2)+' 秒':'')+(qo.brake_count?' · 刹车脉冲 '+qo.brake_count+'/3':'')):'';
 el('cone-short-evidence').textContent=qo.brake_started_s!=null?'本轮制动触发已记录。程序结束不代表已经实测停稳；请量实际停止位置。':'记录车端前进、制动触发时刻及对应画面；不自动填写标定值。';
 el('cone-state').textContent=coneError||c.reason||'锥桶模块未加载';
 el('cone-observe').disabled=stale||coneBusy||!c.can_observe;
 el('cone-stop').disabled=stale||coneBusy||!coneActive;
 for(const id of [...coneInputs,'cone-save','cone-save-dimensions','cone-save-driving','cone-save-calibration','cone-calibration-json','cone-steering-verified','cone-odometry-verified'])el(id).disabled=stale||coneBusy||coneActive||coneDriveActive;
 el('cone-prepare').disabled=stale||coneBusy||!a.can_prepare;
 const oldTrial=a.profile==='legacy_low_speed',trial=a.low_speed_test||{};
 el('cone-drive-start').disabled=stale||coneBusy||!coneDriveOwned||!a.can_request_start||oldTrial;
 el('cone-test-prepare').disabled=stale||coneBusy||!a.can_prepare||!c.low_speed_trial_available;
 el('cone-test-start').disabled=stale||coneBusy||!coneDriveOwned||!a.can_request_start||!oldTrial;
 el('cone-test-confirm').disabled=stale||coneBusy||!coneDriveOwned||!oldTrial||!trial.can_confirm_pass;
 for(const id of ['cone-test-forward','cone-test-burst'])el(id).disabled=stale||coneBusy||coneDriveActive||coneShortActive;
 if(oldTrial)el('cone-test-progress').textContent=(a.reason||'')+' · 已人工确认 '+(trial.operator_confirmed_count||0)+'/2'+(trial.side?' · 当前从'+(trial.side==='left'?'左':'右')+'侧绕过':'')+' · 累计前进指令 '+(trial.powered_s||0).toFixed(2)+' 秒';
 el('cone-drive-start').title=a.start_blocked_reason||'';
 el('cone-run-state').textContent=a.reason||'尚未准备避障';
 const nav=c.latest?.navigation,phases={needs_calibration:'待标定',calibration_only:'位移标定观察',waiting_for_feedback:'等待反馈',waiting_for_two_cones:'等待确认前方两只锥桶',pass_first_cone:'按指定侧绕第一桶',pass_second_cone:'换边绕第二桶',follow_lane:'低速循迹',avoid_cones:'绕行锥桶',return_to_lane:'回归赛道',waiting_for_clear_route:'没有可通过路线',complete:'交错绕桶完成',fault:'已停止'};
 el('cone-test-line-state').textContent=nav?.experimental?('白线依据：'+(nav.line_sources?.length?nav.line_sources.map(x=>x==='primary'?'第一路':'第二路').join('、'):(nav.line_held?'沿用最近边界 '+Number(nav.line_age_s||0).toFixed(2)+' 秒':'等待任一路恢复'))+(nav.source_camera?' · 当前引导：'+(nav.source_camera==='primary'?'第一路':'第二路'):'')):'白线依据：低速测试准备后显示';
 const slalom=nav?.slalom||a.slalom;
 const slalomReasons={cones_too_close_for_alternation:'两桶前后间距不足，无法安排有顺序的换边',need_two_confirmed_cones_ahead:'需要先确认前方两只锥桶',no_feasible_alternating_route:'当前桶距、边界、视野或转弯半径不支持交错路线',slalom_wrong_side_or_clearance:'通过侧别或净空不符合要求',slalom_pose_discontinuity:'位移反馈不连续，请重新准备',slalom_cone_position_changed:'锥桶位置发生变化，请重新准备',slalom_wrong_order:'通过顺序不符合要求',slalom_pass_without_ordered_side_evidence:'缺少按顺序交错通过的位姿证据',slalom_reference_tracking_error:'车辆偏离选定路线，请重新准备',slalom_reference_view_unavailable:'选定路线超出当前可信视野',slalom_reference_blocked:'选定路线的车身净空或白线边界不满足要求',following_locked_alternating_route:'正在跟随选定交错路线',alternating_cones_passed_and_lane_recovered:'两桶已交错通过并回正',no_lane_recovery_route:'暂时没有可行的回正路线'};
 el('cone-slalom').textContent=slalom?.order?.length?('本轮顺序：'+slalom.order.map((p,i)=>(i+1)+'号桶（'+p.id+'）从'+(p.side==='left'?'左':'右')+'侧通过').join(' → ')+' · 已核验 '+(slalom.passed_ids?.length||0)+'/2'+(nav?.reason?' · '+(slalomReasons[nav.reason]||nav.reason):'')):(slalomReasons[nav?.reason]||'等待确认两只锥桶及交错路线');
 if(oldTrial)el('cone-slalom').textContent='旧数据低速试验：白线实时保护，两桶选择相反侧；通过数量由现场确认，不是自动完赛判定。';
 el('cone-navigation').textContent=nav?((phases[nav.phase]||nav.phase)+' · '+(nav.reason||'')+(nav.side?' · 路线 '+({left:'向左',right:'向右',center:'居中'}[nav.side]||nav.side):'')+(nav.passed_ids?' · 已过 '+nav.passed_ids.length+'/2':'')+(nav.motion?.valid?' · 速度 '+nav.motion.speed_mps.toFixed(3)+' m/s · 位移 '+nav.motion.travel_m.toFixed(2)+' m':'')):'开始识别后显示选路、位移和通过计数';
 document.querySelectorAll('[data-cone-drive],[data-cone-motion]').forEach(input=>{
  input.disabled=stale||coneBusy||coneActive||coneDriveActive;
  if(!coneDirty&&!coneBusy&&c.settings){const v=input.dataset.coneDrive?c.settings.driving[input.dataset.coneDrive]:c.settings.motion[input.dataset.coneMotion+'_m'];input.value=v==null?'':input.dataset.coneDrive?v:Math.round(v*1000)/10;}
 });
 if(!coneDirty&&!coneBusy&&c.settings){el('cone-steering-verified').checked=!!c.settings.driving.steering_verified;el('cone-odometry-verified').checked=!!c.settings.driving.odometry_verified;}
 document.querySelectorAll('[data-cone-geometry]').forEach(input=>{
  input.disabled=stale||coneBusy||coneActive;
  if(!coneDirty&&!coneBusy&&c.settings){const v=c.settings.geometry[input.dataset.coneGeometry];input.value=v==null?'':Math.round(v*1000)/10;}
 });
 if(!coneDirty&&!coneBusy&&c.settings){
  const p=c.settings;[p.blue_hsv_lower[0],p.blue_hsv_upper[0],p.blue_hsv_lower[1],p.blue_hsv_lower[2],p.confirm_frames,Math.round((p.scene?.red_ground_min_fraction??.15)*100)].forEach((v,i)=>el(coneInputs[i]).value=v);
 }
 const fresh=coneActive&&c.frame_age_s!=null&&c.frame_age_s>=0&&c.frame_age_s<=.25;
 el('cone-cameras').textContent=fresh?Object.entries(c.latest?.cameras||{}).map(([key,v])=>{
  const visible=v.tracks.filter(t=>t.visible),confirmed=visible.filter(t=>t.confirmed&&t.kind==='cone_candidate');
  return (key==='secondary'?'第二路 H65':'第一路')+'：已确认 '+confirmed.length+' 个，待判断 '+visible.filter(t=>t.kind==='unresolved_blue').length+' 处；白线内边界'+(v.lane_valid?'已识别':'未确认')+'；红色地面 '+Math.round((v.road_context?.red_ground_fraction??0)*100)+'%';
 }).join(' / '):'尚无新鲜识别结果';
 const names={'geometry.rear_axle_to_front_m':'后轴到车头距离','geometry.rear_overhang_m':'后悬长度','geometry.clearance_m':'车身与障碍余量','geometry.measured':'完整车辆尺寸',
  'ground.verified_mapping':'已有地面标定的当前姿态／白线覆盖复核','ground.max_error_m':'已有地面标定的当前姿态／白线覆盖复核','motion.minimum_turn_radius_m':'实测转弯半径','motion.maximum_tracking_error_m':'允许路径偏差','motion.braking_distance_m':'实测制动距离','motion.verified':'转向和制动实测确认','motion.pose_feedback_verified':'光流位移实测确认',
  'driving.fixed_camera_confirmed':'已有地面标定的当前姿态／白线覆盖复核','driving.steering_verified':'转向和制动实测确认','driving.odometry_verified':'光流位移实测确认','driving.steering_radius_1570_cm':'1570 μs 转弯半径','driving.steering_radius_1650_cm':'1650 μs 转弯半径','driving.visual_error_bound_m':'光流误差上限','driving.visual_verified_distance_m':'光流核对行程'};
 el('cone-readiness').textContent=c.readiness?.ready?'软件接入已就绪；场地测试仍需操作者开始。':'待补齐：'+[...new Set((c.readiness?.missing||[]).map(k=>names[k]||k))].join('、');
 if(!fresh){el('cone-preview').hidden=true;el('cone-placeholder').hidden=false;el('cone-placeholder').textContent=coneActive?'等待新鲜摄像头画面':'开始识别后显示真实画面';}
 if(coneDriveActive||coneShortActive){
  lastStatus=s;held.clear();heartbeatDue=false;needsRelease=true;allowed=false;turnAvailable=false;motorAvailable=false;reverseAvailable=false;gamepadControl?.requireNeutral();
  for(const id of ['drive-enable','drive-reset','drive-next','drive-speed','test-mode','gamepad-arm','drive-focus'])if(el(id))el(id).disabled=true;
  document.querySelectorAll('[data-drive],[data-speed]').forEach(b=>b.disabled=true);
  el('drive-title').textContent=coneShortActive?'固定档转向／刹车短测':'独立锥桶避障';el('drive-mode').textContent=coneShortActive?q.reason:a.reason;el('drive-action').textContent=coneShortActive?(qo.phase||'准备'):(a.intent?.action||'准备');
 }
 return coneDriveActive||coneShortActive;
}
for(const id of coneInputs)el(id).addEventListener('input',()=>coneDirty=true);
document.querySelectorAll('[data-cone-geometry]').forEach(input=>input.addEventListener('input',()=>coneDirty=true));
document.querySelectorAll('[data-cone-drive],[data-cone-motion]').forEach(input=>input.addEventListener('input',()=>coneDirty=true));
for(const id of ['cone-steering-verified','cone-odometry-verified'])el(id).addEventListener('change',()=>coneDirty=true);
async function coneAction(action,body={}){
 if(coneBusy)return;coneError='';coneBusy=true;if(lastStatus)render(lastStatus);
 try{const s=await send(action,body);if(action==='cone_settings')coneDirty=false;render(s);}
 catch(e){coneError=e.message;feedback(e.message,'error');}
 finally{coneBusy=false;if(lastStatus)render(lastStatus);}
}
el('cone-surface').addEventListener('change',()=>coneDirty=true);
el('cone-save-surface').addEventListener('click',()=>coneAction('cone_settings',{scene_surface:el('cone-surface').value}));
el('cone-observe').addEventListener('click',()=>coneAction('cone_observe'));
el('cone-stop').addEventListener('click',()=>coneAction('cone_stop'));
function saveConeSettings(){
 for(const id of coneInputs){if(!el(id).checkValidity()){el(id).reportValidity();return;}}
 const geometry={};
 const driving={steering_verified:el('cone-steering-verified').checked,odometry_verified:el('cone-odometry-verified').checked},motion={};
 for(const input of document.querySelectorAll('[data-cone-drive],[data-cone-motion]')){
  if(!input.checkValidity()){input.reportValidity();return;}
  (input.dataset.coneDrive?driving:motion)[input.dataset.coneDrive||input.dataset.coneMotion]=input.value===''?null:Number(input.value);
 }
 for(const input of document.querySelectorAll('[data-cone-geometry]')){
  if(!input.checkValidity()){input.reportValidity();return;}geometry[input.dataset.coneGeometry.replace(/_m$/,'')]=input.value===''?null:Number(input.value);
 }
 coneAction('cone_settings',{blue_hsv_lower:[Number(el('cone-h-low').value),Number(el('cone-s-low').value),Number(el('cone-v-low').value)],
  blue_hsv_upper:[Number(el('cone-h-high').value),255,255],confirm_frames:Number(el('cone-confirm').value),red_ground_percent:Number(el('cone-red-percent').value),geometry_cm:geometry,driving:driving,motion_cm:motion});
}
el('cone-save').addEventListener('click',saveConeSettings);
el('cone-save-dimensions').addEventListener('click',saveConeSettings);
el('cone-save-driving').addEventListener('click',saveConeSettings);
el('cone-save-calibration').addEventListener('click',()=>{
 try{coneAction('cone_calibration',{calibration:JSON.parse(el('cone-calibration-json').value)});}catch(e){coneError='标定 JSON 格式错误：'+e.message;if(lastStatus)render(lastStatus);}
});
el('cone-prepare').addEventListener('click',()=>{
 if(coneDirty){coneError='参数尚未保存，请停止识别并保存参数后再准备';if(lastStatus)render(lastStatus);return;}
 coneSequence=0;coneAction('cone_prepare',{esc_off:true,fb_mode_confirmed:true});
});
function coneRunSend(action,body={}){
 const token=coneToken;
 const run=()=>{if(!coneDriveActive||stale||token!==coneToken)throw new Error('本轮已结束');return send(action,{run_token:token,trial_sequence:++coneSequence,...body});};
 const p=coneRunQueue.then(run,run);coneRunQueue=p.catch(()=>{});return p;
}
el('cone-drive-start').addEventListener('click',async()=>{
 if(coneBusy)return;coneBusy=true;coneError='';if(lastStatus)render(lastStatus);
 try{render(await coneRunSend('cone_start',{fb_mode_confirmed:true,esc_on_static:true,field_ready:true}));}
 catch(e){coneError=e.message;}finally{coneBusy=false;if(lastStatus)render(lastStatus);}
});
el('cone-drive-stop').addEventListener('click',()=>coneAction('cone_cancel'));
el('cone-test-prepare').addEventListener('click',()=>{
 for(const id of ['cone-test-forward','cone-test-burst'])if(!el(id).checkValidity()){el(id).reportValidity();return;}
 coneSequence=0;coneAction('cone_prepare',{profile:'legacy_low_speed',esc_off:true,fb_mode_confirmed:true,
  test_forward_us:Number(el('cone-test-forward').value),test_burst_s:Number(el('cone-test-burst').value)});
});
async function sendConeTest(action,body){
 if(coneBusy)return;coneBusy=true;coneError='';if(lastStatus)render(lastStatus);
 try{render(await coneRunSend(action,body));}catch(e){coneError=e.message;}
 finally{coneBusy=false;if(lastStatus)render(lastStatus);}
}
el('cone-test-start').addEventListener('click',()=>sendConeTest('cone_start',{fb_mode_confirmed:true,esc_on_static:true,field_ready:true}));
el('cone-test-confirm').addEventListener('click',()=>sendConeTest('cone_test_confirm',{tail_passed:true}));
el('cone-test-stop').addEventListener('click',()=>stop('stop',false,'stop_button'));
el('cone-short-prepare').addEventListener('click',()=>{
 coneShortSequence=0;coneAction('cone_short_prepare',{esc_off:true,fb_mode_confirmed:true});
});
el('cone-short-stop').addEventListener('click',()=>stop('stop',false,'stop_button'));
function coneShortSend(action,body={}){
 const token=coneShortToken;
 const run=()=>{if(!coneShortActive||stale||token!==coneShortToken)throw new Error('短测会话已结束');return send(action,{run_token:token,trial_sequence:++coneShortSequence,...body});};
 const p=coneShortQueue.then(run,run);coneShortQueue=p.catch(()=>{});return p;
}
async function startConeShort(mode){
 if(coneShortStartBusy)return;
 for(const id of ['cone-short-forward','cone-short-duration'])if(!el(id).checkValidity()){el(id).reportValidity();return;}
 const round=coneShortRound,forward=Number(el('cone-short-forward').value),duration=Number(el('cone-short-duration').value);
 coneShortStartBusy=true;coneError='';if(lastStatus)render(lastStatus);
 try{render(await coneShortSend('cone_short_start',{mode,forward_us:forward,duration_s:duration,expected_round:round,fb_mode_confirmed:true,esc_on_static:true,field_ready:true}));}
 catch(e){coneError=e.message;}finally{coneShortStartBusy=false;if(lastStatus)render(lastStatus);}
}
el('cone-short-1570').addEventListener('click',()=>startConeShort('turn_1570'));
el('cone-short-1650').addEventListener('click',()=>startConeShort('turn_1650'));
el('cone-short-brake').addEventListener('click',()=>startConeShort('brake'));
setInterval(async()=>{
 if(!coneShortActive||!coneShortOwned||stale||document.hidden||coneShortPulseBusy)return;
 coneShortPulseBusy=true;
 try{render(await coneShortSend('cone_short_keepalive'));}
 catch(e){if(coneShortActive){coneError=e.message;stop('stop',false,'connection_recovery');}}
 finally{coneShortPulseBusy=false;}
},100);
setInterval(async()=>{
 if(!coneDriveActive||!coneDriveOwned||stale||document.hidden||conePulseBusy)return;
 conePulseBusy=true;
 try{render(await coneRunSend('cone_keepalive'));}catch(e){if(coneDriveActive){coneError=e.message;stop('stop',false,'connection_recovery');}}
 finally{conePulseBusy=false;}
},100);
setInterval(async()=>{
 if(!coneActive||stale||document.hidden||coneImageBusy)return;
 coneImageBusy=true;const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1500);
 try{
  const r=await fetch('/api/cone/frame.jpg',{cache:'no-store',signal:controller.signal});if(!r.ok)throw new Error('no fresh frame');
  const blob=await r.blob();if(!coneActive)return;
  const url=URL.createObjectURL(blob),old=coneImageURL;coneImageURL=url;
  el('cone-preview').src=url;el('cone-preview').hidden=false;el('cone-placeholder').hidden=true;if(old)URL.revokeObjectURL(old);
 }catch(e){el('cone-preview').hidden=true;el('cone-placeholder').hidden=false;}
 finally{clearTimeout(timer);coneImageBusy=false;}
},400);
'''


def extend_controls(controls):
    marker='if(!acceptTrafficSnapshot(s))return;'
    sentinel='let gamepadControl=null,pwmControl=null;'
    script=controls.SCRIPT
    if script.count(marker)!=1 or script.count(sentinel)!=1 or not controls.HTML.endswith('</div>') or 'id="autonomy-tests"' not in controls.HTML:
        raise ValueError('锥桶观察扩展需要现有斑马线／红绿灯组合控制台')
    script=script.replace(marker,marker+'\n if(typeof s.cone_status_s===\'number\'){if(s.cone_status_s<coneLastSnapshot)return;coneLastSnapshot=s.cone_status_s;}\n const coneDrivingBusy=renderCone(s);')
    script=script.replace('if(brakeBusy||trafficBusy)return;','if(brakeBusy||trafficBusy||coneDrivingBusy)return;')
    script=script.replace('async function heartbeat(){','async function heartbeat(){\n if(coneDriveActive||coneShortActive){heartbeatDue=false;return;}')
    script=script.replace(sentinel,sentinel+STATE)
    end=script.rfind('})();')
    if end<0:raise ValueError('console script ending missing')
    controls.SCRIPT=script[:end]+SCRIPT+script[end:]
    controls.HTML=controls.HTML[:-6]+HTML+'</div>'
    controls.STYLE=getattr(controls,'STYLE','')+'''<style>
#cone-test{background:#fff;border-radius:18px;padding:20px;display:grid;gap:12px;min-width:0}
#cone-test p,#cone-test h3{margin:0;line-height:1.6}#cone-test p{color:var(--text-secondary,#75677e)}
#cone-test .cone-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px}
#cone-short-section{padding:12px;border:1px solid #dfd3ec;border-radius:12px}#cone-short-section summary{font-weight:650;margin-bottom:12px}
#cone-short-section p{margin:10px 0}#cone-short-section .cone-actions{margin:12px 0}#cone-test .cone-short-starts{display:grid;grid-template-columns:1fr;gap:8px}
#cone-test #cone-short-stop{background:#fff1f2;color:#b42335;border-color:#fecdd3}
#cone-test button,#cone-test input{min-height:42px;border-radius:10px;border:1px solid #d8d8e4}
#cone-test select{min-height:42px;border-radius:10px;border:1px solid #d8d8e4;background:#fff;color:#24212b;padding:8px}
#cone-test button:enabled{background:#8e35e8;color:#fff}#cone-test button:disabled{opacity:.45}
#cone-test .cone-settings{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
#cone-test label{display:grid;gap:6px}#cone-test input{width:100%;min-width:0;box-sizing:border-box;padding:8px;background:#fff;color:#24212b}
#cone-test input[type=checkbox]{width:auto;min-height:0}#cone-test textarea{width:100%;box-sizing:border-box;font:12px monospace;border:1px solid #d8d8e4;border-radius:8px;padding:8px}
#cone-test .cone-viewer{position:relative;aspect-ratio:8/3;background:#f7f4fa;display:grid;place-items:center;border-radius:10px;overflow:hidden}
#cone-preview{position:absolute;width:100%;height:100%;object-fit:contain}#cone-placeholder{padding:16px;text-align:center;color:#75677e}
</style>'''


def add_preview_endpoint(server,drive):
    from urllib.parse import urlsplit
    original=server.RequestHandlerClass
    class Handler(original):
        def do_GET(self):
            if urlsplit(self.path).path!='/api/cone/frame.jpg':return super().do_GET()
            with drive.lock:
                jpg=drive.preview
                fresh=drive.observing and drive.updated is not None and 0<=drive.clock()-drive.updated<=drive.settings['max_frame_age_s']
            try:
                return self.send_body(jpg if fresh and jpg else b'No fresh cone frame',
                    'image/jpeg' if fresh and jpg else 'text/plain',200 if fresh and jpg else 503)
            except (OSError,ConnectionError):self.close_connection=True
    server.RequestHandlerClass=Handler
    return server
