# 树莓派主控与车辆任务

当前实现位于 `../vision/src/carvision/`，不是空项目，也没有 STM32 下位机。暂不复制第二套源码到此目录。

- `race.py`：比赛状态机和米制运动意图。
- `race_flow.py`、`race_flow_camera.py`、`race_flow_controls.py`：当前完整比赛流程、缓存观测读取与网页控制会话。
- `departure*`、`brake_*`、`traffic_*`、`cone_*`、`parking_*`：比赛分项控制器及独立试验。
- `autonomy_runtime.py`：离线/实时输入、期限与清理。
- `autonomy_observation.py`、`autonomy_geometry.py`：双摄、地面坐标、反馈与任务观测。
- `autonomy_motion.py`：速度表/PI/局部轨迹检查及 RasAdapter UART 输出门控。
- `manual_drive.py`、`pi5_pwm.py`：独立的有限手动测试路径。

221 条合成观测的历史决策演示可完成；米制自主配置仍有准备缺项，不能视作自主实车已能完赛。功能对应关系见 [内容概览](../docs/overview.md)，实际验收和后续工作见 [当前进度](../docs/current-status.md)。
