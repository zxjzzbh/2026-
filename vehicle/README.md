# 树莓派主控与车辆任务

当前实现位于 `../vision/src/carvision/`，不是空项目，也没有 STM32 下位机。暂不复制第二套源码到此目录。

- `race.py`：比赛状态机和米制运动意图。
- `autonomy_runtime.py`：离线/实时输入、期限与清理。
- `autonomy_observation.py`、`autonomy_geometry.py`：双摄、地面坐标、反馈与任务观测。
- `autonomy_motion.py`：速度表/PI/局部轨迹检查及 RasAdapter UART 输出门控。
- `manual_drive.py`、`pi5_pwm.py`：独立的有限手动测试路径。

221 条合成观测的决策演示可完成；当前真实配置仍有 18 个缺项，不能视作自主实车已能完赛。当前问题见 [审查](../docs/reviews/2026-10-08-source-review.md)。
