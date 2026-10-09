# race-observation-0.2：树莓派本地实时观测

标准输入为 UTF-8 JSONL，每行一个完整观测；测试可读记录文件。实际执行的 `t_s` 来自当前树莓派 `time.monotonic()`，到达年龄不超过 250 ms。只消费最新观测，不积压回放。

有效、任务完成和认证字段须由可信传感器/网关适配器产生，不接受浏览器填写或把 unknown 变成 true。未列出的字段拒绝接收。字段定义以 `src/carvision/race.py` 的 `Observation` 为准。

| 字段 | 含义 |
| --- | --- |
| `source_ok` | 当前有效的实际观测源 |
| `telemetry_valid`, `telemetry_age_s`, `speed_mps` | 实测车速及年龄，禁止把目标速度当实测速度 |
| `start_line_crossed` | 实测跨过起点，开始计时 |
| `in_switch_zone`, `enable_autonomy` | 切换区与人工切换请求，仍需实测停稳 |
| `emergency_stop` | true 后锁定停车 |
| `board_monitor_valid`, `start_board_state` | 蓝板可靠监测；present/removed/unknown，无候选不等于 confirmed removed |
| `lane_valid`, `lane_offset`, `lane_width_m` | 道路有效、目标偏移（右正左负，归一化 -1..1）、米制宽度 |
| `task_monitor_valid`, `crosswalk_distance_m` | 可靠任务监控，车头到斑马线距离（m），负值越界；null 仅表示可靠搜索范围内暂未定位 |
| `crosswalk_state` | 2026-10-09 增加，present/absent/unknown，默认 unknown；已确认存在但距离未知时保持停车，防止已见目标出画后加速 |
| `announcement_done` | 规划/模拟音频回执；实际运行由本地 WAV 成功播放覆盖 |
| `traffic_zone_entered`, `traffic_stop_distance_m`, `traffic_light_state` | 进入传感器与灯之间、前轮到灯距离（m）、red/yellow/green/unknown |
| `cone_monitor_valid`, `cones`, `passed_cone_ids` | 障碍可靠监测、米制位置和经过的独立跟踪 ID |
| `parking_zone_visible`, `parking_geometry_valid`, `parking_slots` | 停车区与两车位可靠几何；未检出蓝板不能自动当 clear |
| `parking_remaining_m`, `parked_slot_id`, `wheels_inside` | 到目标距离、车位 ID、实际入位轮数 0..4 |
| `payment_confirmed`, `payment_amount_cents` | 人工支付后的可信回执，金额须整数 1；不触发资金转移 |
| `remote` | 人工段请求及认证、5G 核验、deadman 持续使能、指令年龄；自动段忽略驾驶遥控 |

锥桶对象：`{"lateral_m": -0.15, "forward_m": 0.8, "radius_m": 0.039}`。横向右正，纵向前正。净空含半车宽、锥桶半径与余量；这是局部通道目标，不是完整阿克曼转向扫掠轨迹，执行前仍需轨迹验证。

车位对象：`{"id": "left", "center_lateral_m": -0.3, "availability": "blocked"}` 与另一位 `clear`，坐标必须与道路、锥桶一致。每帧两个不同 ID，恰好一个被挡一个空闲；未知或重复 ID 都停车。

人工指令：`{"authenticated": true, "verified_5g": true, "deadman": true, "command_age_s": 0.04, "speed_mps": 0.1, "steering_normalized": 0}`。程序没有建立或证明 5G 隧道，核验字段必须来自可信上游；不要向网络直接暴露未认证的标准输入适配器。

输出 `race-intent-0.2` 为目标速度、转向归一化值、横向目标、状态与事件，单独规划不输出硬件信号。`pi-run` 才通过速度反馈及标定脉宽映射执行。当前未标定，干运行脉宽为 null 并报告缺失项。

斑马线软件调试入口使用同一 `Observation`，限定 `scope=crosswalk`；完整实车运行层仍默认 `full_race`。决策增加 `crosswalk_seen/served/hold_s/elapsed_s/remaining_s`；完成事件为 `crosswalk_completed`，原因改为不写死秒数的 `crosswalk_configured_hold_completed`。单任务完成状态为 `follow_after_crosswalk`，整场完成此任务后仍转入红绿灯阶段。3 秒配置和运行说明见 `vision/CROSSWALK_3S.md`。
