# 参数与标定来源

整理于 2026-10-20，依据主线 `cdf8d31`。本页说明同名参数在不同路径中的用途；本次未修改任何配置、脉宽或实测标志。

## 现行与历史参数

| 数据 | 主线记录 | 依据与适用范围 |
| --- | --- | --- |
| 共享 S3 端点 | 右 1350、中位 1610、左 1870 μs | [steering.py](../vision/src/carvision/steering.py)，比赛流程与已对齐的方向控制使用；不是实测轮角模型 |
| 独立手动 PWM 调试 | 1550–1750 μs，限时试验 | [manual_drive.py](../vision/src/carvision/manual_drive.py)、[STEERING_PWM](../vision/STEERING_PWM.md)；不能写成所有控制路径的转向范围 |
| 米制自主配置的旧中位 | 1650 μs | [autonomy-pi5.json](../vision/configs/autonomy-pi5.json)；运动标定未完成，保留原值 |
| 电调 | F/B；中位 1500 μs | 各任务油门/刹车值见其配置；PWM 不等于实测速度或制动力 |
| 锥桶／停车车体几何 | 宽 29.5 cm、后轴到车头 33.5 cm、后悬 8.5 cm，总长 42 cm | [cone-trial.json](../vision/configs/cone-trial.json)、[parking-bay.json](../vision/configs/parking-bay.json) 中的 10 月 11 日记录；仍保留未验证项 |
| 自主框架历史几何 | 宽 30 cm、轴距 24.5 cm、轮距 24 cm、车头到后轴 34 cm | `autonomy-pi5.json` 的 10 月 7 日基线，未批量替换为任务试验值 |
| 双摄历史地面映射 | 640×480，车头前 50–150 cm | 固定姿态；独立检查点最大误差约 2.2 cm，仅代表该区域的静态验收 |
| 第二路历史标定姿态 | S1=1600、S2=1150 μs | 姿态/分辨率改变后需核验，不能直接外推为当前 480×360 任务图像标定 |
| 地面坐标 | 后轴中心原点，x 向右、y 向前 | 像素、车头距离和米制后轴坐标分别处理 |

不同配置面向不同运行路径，已有数值不能因为名称相近就互相覆盖。转向曲率、速度—PWM、制动距离、真实位姿反馈和近场停车仍需现场验证；以对应配置的 `verified`、`measured` 等字段及实际证据为准。

## 保留的静态证据

- [主摄地面验收](../run/pi5-calibration-20261007-01/145544-765763-native-ground-reference/primary-ground-acceptance.json)。
- [第二路地面验收](../run/pi5-calibration-20261007-01/152800-603601-native-ground-reference/secondary-ground-acceptance.json)。
- [10 月 9 日原标定说明](archive/2026-10-09/calibration-status.md)。

开机模板及历史 `boot_id` 用于追溯，不替代本次现场确认。完整任务的状态和缺项见 [当前进度](current-status.md)。
