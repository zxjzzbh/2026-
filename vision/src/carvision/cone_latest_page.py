"""Add an independent latest-cone entry to the existing debug page. Author: 13246."""
from types import SimpleNamespace


def once(text,old,new):
    if text.count(old)!=1:raise ValueError('当前调试网页接口有变化，未修改原网页：'+old[:60])
    return text.replace(old,new,1)


TASK="""    cone_latest:{name:'最新版锥桶 · 连续绕行',key:'cone_latest.avoidance',prepare:'latest_cone_prepare',start:'latest_cone_start',stop:'latest_cone_stop',heartbeat:'latest_cone_keepalive',observe:'latest_cone_observe',observeStop:'latest_cone_stop',image:'/api/cone-latest/frame.jpg',
      description:'实车回正修正版：锁定绕行侧，先收转角并对齐赛道，车尾通过后再回中；保持持续前进。与原避障测试分别保存参数。',
      fields:[select('scene_surface','场地（蓝色锥桶）',[['red_running_track','红色跑道'],['indoor_white_line_track','室内白线场地']],'red_running_track','cone_latest.settings.scene.surface'),
        number('white_v_min','白线最低亮度',0,255,1,200,'cone_latest.settings.scene.white_v_min'),
        number('test_forward_us','前进油门（μs）',1501,1625,1,1600,'cone_latest.parameters.test_forward_us'),
        number('test_left_us','左转端点（μs）',1650,1870,5,1870,'cone_latest.parameters.test_left_us'),
        number('test_right_us','右转端点（μs）',1350,1570,5,1350,'cone_latest.parameters.test_right_us'),
        number('test_steering_gain','转向增益',1,3,.1,1,'cone_latest.parameters.test_steering_gain'),
        select('record_enabled','双摄逐帧记录',[['on','开启'],['off','关闭']],'on','cone_latest.recording_settings.mode'),
        number('record_max_mb','单轮记录上限（MB）',32,2048,32,512,'cone_latest.recording_settings.max_mb')]},
"""

BANNER='''<section class="card" id="cone-latest-section" style="border:2px solid #2563eb">
<h2>最新版锥桶 · 连续绕行</h2><p>11月7日白线回正与心跳修正版 · 锁定绕行侧、提前对齐道路、车尾通过后再回中；原避障测试继续保留。</p>
<button id="open-latest-cone" type="button">进入最新版锥桶测试</button>
<a id="latest-cone-replay" target="_blank" rel="noopener" hidden>打开本轮实拍回放</a>
<p id="latest-cone-recording">准备或观察后自动记录双摄画面；停止后可回放。</p></section>'''


def extend_controls(controls):
    html=once(controls.HTML,'<main>','<main>'+BANNER)
    html=once(html,'<option value="cone">','<option value="cone_latest">最新版锥桶 · 连续绕行</option><option value="cone">')
    script=once(controls.SCRIPT,'  const TASKS={','  const TASKS={\n'+TASK)
    script=once(script,'status?.cone_trial?.short_test,','status?.cone_trial?.short_test,status?.cone_latest?.avoidance,')
    script=once(script,"task='crosswalk',epoch=0","task=(global.location?.search||'').includes('task=cone_latest')?'cone_latest':'crosswalk',epoch=0")
    script=once(script,"function missing(){const r=state(),list=[];",
        "function missing(){const r=state(),list=[];if(task==='cone_latest')return r.settings_error?[r.settings_error]:[];")
    script=once(script,"task==='cone'?status?.cone_trial?.can_observe:status?.parking_bay?.can_observe",
        "task==='cone_latest'?status?.cone_latest?.can_observe:task==='cone'?status?.cone_trial?.can_observe:status?.parking_bay?.can_observe")
    script=once(script,"task==='cone'?status.cone_trial?.observing:status.parking_bay?.active",
        "task==='cone_latest'?status.cone_latest?.observing:task==='cone'?status.cone_trial?.observing:status.parking_bay?.active")
    script=once(script,"'application_status_s','traffic_status_s'","'application_status_s','cone_latest_status_s','traffic_status_s'")
    script=once(script,"      el('task-diagnostics').textContent=diagnostics.join('\\n');","""      if(task==='cone_latest'){
        const p=r.low_speed_test||{},n=status?.cone_latest?.latest?.navigation||{};
        diagnostics.push('版本：白线回正与心跳修正 20261107；原避障测试和参数独立保留。');
        diagnostics.push('已通过：'+(p.passed_count??0)+' / 2；两桶后仍持续前进，由操作者停止。');
        diagnostics.push('当前绕行侧：'+(p.side||'尚未选侧')+'；白线纠偏：'+(p.boundary_correction||'无'));
        const imageGuide=p.image_guidance;
        if(imageGuide){
          const phases={avoiding:'绕行',hold_target:'短时保留目标',hold_side:'保持绕行侧',align:'对齐赛道',clearance_tail:'保持走廊通过车尾',recenter:'平缓回中',search:'寻找道路方向'};
          diagnostics.push('图像控制阶段：'+(phases[imageGuide.phase]||imageGuide.phase));
          diagnostics.push(imageGuide.geometry?.feedback_weight>0?'正在使用白线方向反馈回正':'白线方向暂不可用；轮向回中不代表车身已对齐');
        }
        if(typeof p.boundary_lateral_allowance_m==='number')diagnostics.push('缺测横向预留：'+(p.boundary_lateral_allowance_m*100).toFixed(1)+' cm');
        diagnostics.push(n.reason||'等待双摄实际画面');
        const rec=status?.cone_latest?.recording||{},link=el('latest-cone-replay');
        link.hidden=!rec.replay_url;if(rec.replay_url)link.href=rec.replay_url;
        el('latest-cone-recording').textContent='已保存 '+(rec.written?.frame||0)+' 帧；缺帧 '+(rec.dropped?.frame||0)+'；缺失事件 '+(rec.dropped?.event||0)+'；'+(rec.error||rec.tap_error||(rec.active?'正在后台记录':rec.finalized?'本轮记录已结束':'尚无本轮记录／记录已关闭'));
        diagnostics.push('记录只保存实际画面与指令；回放中的转向值不是前轮角度反馈。');
      }
      el('task-diagnostics').textContent=diagnostics.join('\\n');""")
    script=once(script,"      if(task==='crosswalk')await enqueue('crosswalk_settings'",
        "      if(task==='cone_latest')await enqueue('latest_cone_settings',v);\n      if(task==='crosswalk')await enqueue('crosswalk_settings'")
    script=once(script,"function bind(){el('task-select')","""function bind(){
      el('open-latest-cone').addEventListener('click',()=>{
        if(choose('cone_latest')){el('task-select').value='cone_latest';el('task-select').closest?.('.card')?.scrollIntoView({behavior:'smooth',block:'start'});}
        else error('先停止当前测试或识别，再切换最新版锥桶。');
      });
      el('task-select')""")
    script=once(script,'started=true;bind();fillFields();','started=true;bind();el(\'task-select\').value=task;fillFields();')
    # Reuse the independent, compact heartbeat path; status/image polling must
    # not queue or cancel the current latest-cone lease.
    script=once(script,"pageHeartbeat('cone_page_heartbeat')", "pageHeartbeat(task==='cone_latest'?'latest_cone_page_heartbeat':'cone_page_heartbeat')")
    script=script.replace("task!=='cone'", "!['cone','cone_latest'].includes(task)")
    script=once(script,"if(task==='cone')return coneHeartbeat()", "if(['cone','cone_latest'].includes(task))return coneHeartbeat()")
    return SimpleNamespace(HTML=html,SCRIPT=script,STYLE=controls.STYLE)
