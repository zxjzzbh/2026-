# C++ 5G 控制前端

`/competition/` 与 `/debug/` 共用 `index.html`，分别提供比赛全流程和单项任务选择。静态文件路径为 `/static/app.js`、`/static/style.css`。

调试页选好项目后，按“开始测试”即可启动，按“结束测试”回到待测试状态。行驶项目前由车端保持中位 3 秒，期间可以取消。观察项目直接启动。浏览器只记忆测试项目，刷新页面不会发车。

## 接口

- `GET /api/session`：HttpOnly、SameSite=Strict 的会话 Cookie；响应 `{session, csrf}`，session 是会话的 SHA-256 显示 ID，原始 Cookie 不在 JSON 中公开。
- `GET /api/status` 返回 idle、prepared、manual、test_preparing、starting、autonomous、observing、stopping 或 fault，以及本轮控制权与执行输出等信息。
- `GET /api/frame.jpg?camera=primary|secondary`：当前 JPEG，无帧可返回非 200。每路单飞刷新，上一请求完成后等待 350 ms 再请求。
- `POST /api/action`：JSON 含 action、csrf。成功响应状态对象；失败返回非 200 和 `{error: "中文原因"}`。
- 调试页发送 `start_test`，携带 `task` 和当前 `generation`；全流程测试还需要确认交接。车端独立准备、启动并记录本轮。结束按钮发送 `stop`，同时取消待启动的任务并使旧轮次失效。
- 比赛页沿用 `prepare`、`enable`、`command`、`brake`、`start` 和 `stop`。遥控和自主交接指令携带当前轮次。

## 操作与控制边界

比赛页的电调开关确认互斥，准备完成后由操作员确认电调已打开、车辆已停稳。调试页由现场监管人员在通电和停稳后按“开始测试”。

遥控模式每 100 ms 最多一个在途 command，松开输入/失焦回中位。前进 1570–1600 μs，转向 1350/1610/1870 μs。刹车是独立动作，前端暂停普通遥控提交至少 500 ms，后端也应确保在途指令不截断制动。

自主交接前清空按键并提交一次中位。只有后端确认进入自主模式后，前端按该状态展示；自主期间不发送 command、心跳或离页停止指令，保留显式停止。后端必须独立校验会话、准备状态、指令顺序和串口归属。

状态轮询每次完成后等待 500 ms，超时 2.5 s；网络失败只展示错误并停发前进，不把网页连接故障转换成自主 stop。画面失败不会影响控制状态。所有车端文本通过 textContent 显示。

执行输出只展示状态接口的新鲜输出记录（output_fresh 为真，使用 status.throttle_us/steering_us），不把遥测中的 requested_throttle_us 显示为已经执行；过期或缺失记录显示“暂无新鲜输出记录”。控制 generation 变化后清除按键状态，避免把上一轮持按延续到重新启用后。

语法检查：`node --check vision/web/app.js`。此检查不代表实车交接已验证。
