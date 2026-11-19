'use strict';

(() => {
  const $ = id => document.getElementById(id);
  const debugPage = location.pathname.startsWith('/debug');
  const modes = {idle:debugPage ? '待测试' : '未准备',prepared:'已准备 · 等待现场操作',manual:'5G 遥控',test_preparing:'正在准备测试',starting:'正在交接并启动',autonomous:'车端自主运行',observing:'仅观察识别 · 不驱动车辆',stopping:'正在停止',fault:'已停止 · 请查看原因'};
  const state = {session:'',csrf:'',status:null,connected:false,busy:false,pendingAction:'',actionId:0,commandBusy:false,sequence:Date.now()*1000,held:new Map(),keys:new Set(),brakeUntil:0,lastError:'',networkError:false,lastStatus:0};
  const buttons = [...document.querySelectorAll('[data-drive]')];
  let stoppedByVisibility = false;

  $('page-title').textContent = debugPage ? '单项调试' : '比赛控制';
  document.title = `智能车 · C++ ${debugPage ? '单项调试' : '比赛控制'}`;
  $('page-description').textContent = debugPage ? '选好项目，点击开始测试。准备中或运行中都可以结束。' : '遥控到达切换区，停稳后交给车端自主运行。';
  $('task-heading').textContent = debugPage ? '现场测试' : '自主驾驶';
  $('task-picker').hidden = !debugPage;
  if (debugPage) {
    $('manual-card').hidden = $('observe').hidden = $('task-number').hidden = $('handoff-help').hidden = true;
    $('task-actions').classList.add('button-row','test-actions');
    $('task-actions').appendChild($('stop'));
    $('stop').textContent = '结束测试';
    try {
      const previous = localStorage.getItem('smartcar.testTask');
      if ([...$('task').options].some(option => option.value === previous)) $('task').value = previous;
    } catch (_) { /* Storage is optional. */ }
  }
  $(debugPage ? 'debug-link' : 'race-link').setAttribute('aria-current','page');

  function error(message) {
    state.lastError = message || '';
    $('error').textContent = state.lastError;
    $('error').hidden = !state.lastError;
  }

  async function request(path, options = {}, timeout = 3000) {
    const abort = new AbortController();
    const timer = setTimeout(() => abort.abort(), timeout);
    try {
      const response = await fetch(path, {...options, cache:'no-store',credentials:'same-origin',signal:abort.signal});
      let body;
      try { body = await response.json(); } catch (_) { throw new Error('车端返回的数据无法读取，请检查服务状态。'); }
      if (!response.ok) {
        const failure = new Error(body.error || `请求失败（${response.status}）`);
        failure.status = response.status;
        throw failure;
      }
      return body;
    } catch (e) {
      if (e.name === 'AbortError') throw new Error('请求超时，请检查 5G 连接；车端当前状态需要重新读取。');
      throw e;
    } finally { clearTimeout(timer); }
  }

  function ownManual() { return !debugPage && state.status?.mode === 'manual' && state.status.owner === state.session; }
  function canManual() { return ownManual() && state.connected && !state.busy && !document.hidden && Date.now() >= state.brakeUntil; }
  function authorized() { return !state.status?.owner || state.status.owner === state.session; }
  function controlsConfirmed() { return $('supervised').checked && $('esc-on').checked && $('stationary').checked; }
  function currentTask() { return debugPage ? $('task').value : 'race'; }

  function render() {
    const s = state.status;
    const mode = s?.mode || '';
    const free = authorized();
    const available = state.connected && !state.busy && free;
    $('connection').textContent = state.connected ? '车端已连接' : (s ? '连接中断 · 正在重试' : '正在连接');
    $('connection').classList.toggle('offline',!state.connected);
    $('mode').textContent = s ? (modes[mode] || mode) : '正在读取车端状态…';
    $('reason').textContent = debugPage && mode === 'idle' ? '可以开始下一轮测试。' : s?.reason || (s ? '请按现场状态选择操作。' : '读取完成前不会开放操作。');
    $('ownership').textContent = !s?.owner ? '当前未占用控制。' : free ? '当前浏览器会话持有控制。' : '控制由其他浏览器会话持有，请先在原控制页停止。';
    $('prepare').disabled = !available || !['idle','fault'].includes(mode) || !$('esc-off').checked || !$('supervised').checked || !!s?.process_running;
    $('enable').disabled = !available || mode !== 'prepared' || !controlsConfirmed();
    const readyToStart = debugPage ? ['idle','prepared','fault'].includes(mode) : ['prepared','manual'].includes(mode) && controlsConfirmed();
    $('start').disabled = !available || !readyToStart || !!s?.process_running;
    $('start').textContent = debugPage ? '开始测试' : '停稳 · 切入自主驾驶';
    $('observe').disabled = !available || !['idle','prepared','fault'].includes(mode) || !!s?.process_running;
    $('stop').disabled = !state.csrf || state.pendingAction === 'stop' || !s || (!state.busy && !s.owner && !s.process_running && mode === 'idle');
    $('task').disabled = state.busy || mode === 'test_preparing' || !!s?.process_running;
    if (debugPage && s?.task && (mode === 'test_preparing' || s.process_running)) $('task').value = s.task;
    $('throttle').disabled = state.busy || mode === 'autonomous';
    for (const button of buttons) button.disabled = !canManual();
    $('neutral').disabled = !canManual();
    $('brake').disabled = !canManual();
    if (debugPage) {
      let help;
      if (mode === 'test_preparing') help = '车端正在保持中位，点“结束测试”可取消启动。';
      else if (s?.process_running) help = '本轮运行中。点“结束测试”可释放车辆控制。';
      else if (!free || mode === 'manual') help = '当前控制已占用，请先结束当前操作。';
      else if (currentTask() === 'observe') help = '只采集图像进行识别。';
      else if (currentTask() === 'race') help = '在切换区交接并停稳后，由现场监管人点击开始测试。';
      else help = '车辆通电、停稳后，由现场监管人点击开始测试。';
      $('start-help').textContent = help;
    }
    else if (mode === 'autonomous') $('start-help').textContent = '车端正在独立运行。网页不发送遥控指令或保活指令。';
    else if (!free) $('start-help').textContent = '请先释放其他浏览器会话的控制。';
    else if (['prepared','manual'].includes(mode)) $('start-help').textContent = '请打开电调、确认停稳并勾选现场监管，再开始任务。';
    else $('start-help').textContent = '需要先关闭电调并准备，再完成开机、停稳确认。';
    const t = s?.telemetry || {};
    const stage = t.state || t.stage || '—';
    $('run-stage').textContent = stage;
    const throttle = s?.throttle_us;
    $('run-output').textContent = !s?.output_fresh || throttle == null ? '暂无新鲜输出记录' : `油门 ${throttle} μs · 转向 ${s.steering_us ?? '—'} μs`;
    $('run-dir').textContent = s?.run_dir || '—';
    $('telemetry').textContent = t.reason || (s?.process_running ? '任务进程运行中' : '未运行自主任务');
  }

  function clearHeld() {
    state.held.clear(); state.keys.clear();
    buttons.forEach(button => button.classList.remove('active'));
    $('manual-output').textContent = '当前遥控请求：中位';
  }

  function desiredCommand() {
    const held = [...state.held.values()];
    const forward = state.keys.has('ArrowUp') || held.some(value => value.startsWith('forward'));
    const left = state.keys.has('ArrowLeft') || held.some(value => value.endsWith('left'));
    const right = state.keys.has('ArrowRight') || held.some(value => value.endsWith('right'));
    const throttleSetting = Math.max(1570,Math.min(1600,Number($('throttle').value) || 1570));
    return {throttle_us:forward ? Math.round(throttleSetting) : 1500,steering_us:left === right ? 1610 : left ? 1870 : 1350};
  }

  async function rawAction(action, fields = {}, timeout = 6000) {
    const payload = {action,csrf:state.csrf,...fields};
    if (['enable','start','start_test','command','brake'].includes(action)) {
      if (!Number.isSafeInteger(state.status?.generation)) throw new Error('尚未读取最新控制状态，请稍后重试。');
      payload.generation = state.status.generation;
    }
    return request('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)},timeout);
  }

  function acceptStatus(result) {
    if (!result || !result.mode) return;
    if (state.networkError) { error(''); state.networkError = false; }
    const oldMode = state.status?.mode;
    const oldGeneration = state.status?.generation;
    state.status = result;
    state.connected = true;
    state.lastStatus = Date.now();
    if (result.mode !== 'manual' || result.owner !== state.session || oldGeneration !== result.generation) clearHeld();
    if (oldMode !== result.mode && result.mode === 'prepared') {
      $('esc-on').checked = false;
      $('stationary').checked = false;
    }
    render();
  }

  async function manualCommand(forceNeutral = false) {
    if (!ownManual() || !state.csrf || state.commandBusy || state.busy || Date.now() < state.brakeUntil) return;
    const command = forceNeutral || document.hidden || !state.connected ? {throttle_us:1500,steering_us:1610} : desiredCommand();
    state.commandBusy = true;
    const sequence = ++state.sequence;
    const actionId = state.actionId;
    $('manual-output').textContent = `当前遥控请求：油门 ${command.throttle_us} μs · 转向 ${command.steering_us} μs`;
    try { await rawAction('command',{...command,sequence},1000); }
    catch (e) { if (actionId === state.actionId && ownManual()) { clearHeld(); error(e.message); } }
    finally { state.commandBusy = false; }
  }

  async function act(action, fields = {}) {
    if ((state.busy && action !== 'stop') || !state.csrf) return;
    const actionId = ++state.actionId;
    clearHeld();
    state.busy = true; state.pendingAction = action; error(''); render();
    try {
      // A held or in-flight manual forward command must not survive the handoff.
      if (action === 'start' && ownManual()) {
        await rawAction('command',{throttle_us:1500,steering_us:1610,sequence:++state.sequence},2000);
        if (actionId !== state.actionId) return;
      }
      if (action === 'brake') state.brakeUntil = Date.now()+500;
      const result = await rawAction(action,fields,['start','start_test'].includes(action) ? 15000 : 8000);
      if (actionId === state.actionId) acceptStatus(result);
    } catch (e) { if (actionId === state.actionId) error(e.message); }
    finally { if (actionId === state.actionId) { state.busy = false; state.pendingAction = ''; render(); } }
  }

  $('prepare').addEventListener('click',() => act('prepare',{esc_off:$('esc-off').checked,supervised:$('supervised').checked}));
  $('enable').addEventListener('click',() => act('enable',{esc_on:$('esc-on').checked,stationary:$('stationary').checked}));
  $('start').addEventListener('click',() => {
    const task = currentTask();
    if (debugPage) return act('start_test',task === 'race' ? {task,handoff_complete:true} : {task});
    return act('start',{task,supervised:$('supervised').checked,stationary:$('stationary').checked,handoff_complete:true});
  });
  $('observe').addEventListener('click',() => act('start',{task:'observe'}));
  $('stop').addEventListener('click',() => act('stop'));
  $('brake').addEventListener('click',() => act('brake'));
  $('neutral').addEventListener('click',() => { clearHeld(); void manualCommand(true); });
  for (const id of ['supervised','esc-off','esc-on','stationary','task']) {
    $(id).addEventListener('change',() => {
      if (debugPage && id === 'task') {
        try { localStorage.setItem('smartcar.testTask',currentTask()); } catch (_) { /* Storage is optional. */ }
      }
      if (id === 'esc-off' && $('esc-off').checked) $('esc-on').checked = false;
      if (id === 'esc-on' && $('esc-on').checked) $('esc-off').checked = false;
      render();
    });
  }
  $('throttle').addEventListener('change',() => { $('throttle').value = String(Math.round(Math.max(1570,Math.min(1600,Number($('throttle').value) || 1570)))); });

  for (const button of buttons) {
    button.addEventListener('contextmenu',event => event.preventDefault());
    button.addEventListener('pointerdown',event => {
      if (!canManual() || event.button !== 0) return;
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      state.held.set(event.pointerId,button.dataset.drive);
      button.classList.add('active');
      void manualCommand();
    });
    const release = event => {
      state.held.delete(event.pointerId);
      button.classList.remove('active');
      void manualCommand();
    };
    button.addEventListener('pointerup',release);
    button.addEventListener('pointercancel',release);
    button.addEventListener('lostpointercapture',release);
  }

  const editable = target => target instanceof Element && !!target.closest('input,select,textarea,[contenteditable="true"]');
  window.addEventListener('keydown',event => {
    if (!canManual() || editable(event.target)) return;
    if (event.code === 'Space') { event.preventDefault(); if (!event.repeat) void act('brake'); return; }
    if (!['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(event.key)) return;
    event.preventDefault();
    if (event.key === 'ArrowDown') clearHeld(); else state.keys.add(event.key);
    void manualCommand();
  });
  window.addEventListener('keyup',event => {
    if (!state.keys.has(event.key)) return;
    state.keys.delete(event.key);
    void manualCommand();
  });
  function releaseOnBlur() {
    clearHeld();
    // Do not send any command/stop while autonomous. Its progress is owned by the Pi.
    if (ownManual()) void manualCommand(true);
  }
  window.addEventListener('blur',releaseOnBlur);
  document.addEventListener('visibilitychange',() => {
    stoppedByVisibility = document.hidden;
    if (stoppedByVisibility) releaseOnBlur();
    render();
  });

  async function pollStatus() {
    try {
      if (!state.csrf) {
        const session = await request('/api/session');
        state.session = session.session; state.csrf = session.csrf;
      }
      const actionId = state.actionId;
      const status = await request('/api/status',{},2500);
      if (actionId === state.actionId && !state.busy) acceptStatus(status);
    } catch (e) {
      state.connected = false; clearHeld(); render();
      state.networkError = true; error(e.message);
      if (e.status === 403 || e.status === 401) { state.csrf = ''; state.session = ''; }
    } finally { setTimeout(pollStatus,500); }
  }

  async function pollFrame(camera) {
    const img = $(`${camera}-frame`);
    const abort = new AbortController();
    const timer = setTimeout(() => abort.abort(),2500);
    try {
      if (!state.csrf || document.hidden) return;
      const response = await fetch(`/api/frame.jpg?camera=${camera}`,{cache:'no-store',credentials:'same-origin',signal:abort.signal});
      if (!response.ok) throw new Error('frame unavailable');
      const blob = await response.blob();
      if (!blob.type.startsWith('image/')) throw new Error('invalid image');
      const url = URL.createObjectURL(blob);
      const old = img.dataset.url;
      img.src = url; img.dataset.url = url; img.classList.add('loaded');
      if (old) URL.revokeObjectURL(old);
      img.dataset.updated = String(Date.now());
      $('video-state').textContent = '刚刚更新';
    } catch (_) {
      if (!img.dataset.updated || Date.now()-Number(img.dataset.updated)>4000) {
        img.classList.remove('loaded');
        $('video-state').textContent = '等待新画面';
      }
    } finally { clearTimeout(timer); setTimeout(() => pollFrame(camera),350); }
  }

  setInterval(() => {
    if (ownManual() && !state.busy && !stoppedByVisibility) void manualCommand();
    if (state.status && state.connected && Date.now()-state.lastStatus>3500) {
      state.connected = false; clearHeld(); render();
    }
  },100);
  render();
  void pollStatus();
  void pollFrame('primary');
  void pollFrame('secondary');
})();
