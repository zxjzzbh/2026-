# C++ 5G 控制前端

`/competition/` 与 `/debug/` 共用 `index.html`，分别提供比赛全流程和单项任务选择。静态文件路径为 `/static/app.js`、`/static/style.css`。

## 接口

- `GET /api/session`：HttpOnly、SameSite=Strict 的会话 Cookie；响应 `{session, csrf}`，session 是会话的 SHA-256 显示 ID，原始 Cookie 不在 JSON 中公开。
- `GET /api/status`：`mode` 为 idle/prepared/manual/starting/autonomous/observing/stopping/fault；另含 owner（同样的显示 ID）、generation、reason、telemetry、run_dir、process_running、throttle_us、steering_us、output_fresh、output_source。
- `GET /api/frame.jpg?camera=primary|secondary`：当前 JPEG，无帧可返回非 200。每路单飞刷新，上一请求完成后等待 350 ms 再请求。
- `POST /api/action`：JSON 含 action、csrf。成功响应状态对象；失败返回非 200 和 `{error: "中文原因"}`。
- prepare：esc_off、supervised；enable：esc_on、stationary；command：throttle_us、steering_us、sequence；brake：350 ms 制动；stop：停止释放；start：task、supervised、stationary、handoff_complete。observe 不需要运动确认。enable/start/command/brake 必须携带当前状态 generation，后端拒绝上一轮延迟包；start 前的中位 command 也携带该值。

## 操作与控制边界

现场确认不会自动勾选。电调开/关确认互斥；准备完成后清除“已打开”和“停稳”确认，需操作员确认当前事实。

遥控模式每 100 ms 最多一个在途 command，松开输入/失焦回中位。前进 1570–1600 μs，转向 1350/1610/1870 μs。刹车是独立动作，前端暂停普通遥控提交至少 500 ms，后端也应确保在途指令不截断制动。

自主交接前清空按键并提交一次中位。只有后端确认进入自主模式后，前端按该状态展示；自主期间不发送 command、心跳或离页停止指令，保留显式停止。后端必须独立校验会话、准备状态、指令顺序和串口归属。

状态轮询每次完成后等待 500 ms，超时 2.5 s；网络失败只展示错误并停发前进，不把网页连接故障转换成自主 stop。画面失败不会影响控制状态。所有车端文本通过 textContent 显示。

执行输出只展示状态接口的新鲜输出记录（output_fresh 为真，使用 status.throttle_us/steering_us），不把遥测中的 requested_throttle_us 显示为已经执行；过期或缺失记录显示“暂无新鲜输出记录”。控制 generation 变化后清除按键状态，避免把上一轮持按延续到重新启用后。

语法检查：`node --check vision/web/app.js`。此检查不代表实车交接已验证。
