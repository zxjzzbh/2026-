# 操作指南导航

先查 [当前进度](../current-status.md)。下表按操作目的整理已有指南；指南内不同日期的段落保留当时状态，不能把旧步骤中的参数直接当作当前配置。

## 安装、连接与预览

| 目的 | 入口 | 说明 |
| --- | --- | --- |
| 新电脑安装、离线测试 | [安装与复现](../reproduce.md) | Python 虚拟环境、Node.js、pytest 和回放 |
| 连接日常控制台 | [CONSOLE_ACCESS](../../vision/CONSOLE_ACCESS.md) | 根 `connect-car.cmd`，本机 8082，连接后不自动驾驶 |
| 使用另一套 Windows 启动器 | [pc-launcher 说明](../../pc-launcher/使用说明.txt) | `pc-launcher/start.cmd`，`SMARTCAR_HOST`，本机 18080 |
| 开机等待与现场确认 | [BOOT_TEST](../../vision/BOOT_TEST.md) | 依赖已有车端服务 |
| 查看双摄画面 | [LIVE_PREVIEW](../../vision/LIVE_PREVIEW.md) | 图片、录像、USB 相机与网页预览 |
| 查双摄与云台 | [CAMERAS_GIMBAL](../../vision/CAMERAS_GIMBAL.md) | 结合 [当前硬件](../../hardware/README.md) 阅读历史接线段落 |

8082 和 18080 是两套电脑入口；8091 用于红绿灯采集，车端已有服务通常为 8080。端口不同不代表另有一份比赛程序。

## 比赛分项与完整流程

| 任务 | 指南 | 重点区别 |
| --- | --- | --- |
| 蓝挡板发车 | [DEPARTURE_TEST](../../vision/DEPARTURE_TEST.md) | 独立测试只播报；总流程负责发车衔接 |
| 斑马线 F/B 制动 | [BRAKE_PARKING](../../vision/BRAKE_PARKING.md) | 实车试验入口；距离参考与真实停点分开 |
| 斑马线 3 秒逻辑实验 | [CROSSWALK_3S](../../vision/CROSSWALK_3S.md) | 软件意图模块；不替代当前整场 10 秒配置 |
| 红绿灯 | [TRAFFIC_SIGNAL](../../vision/TRAFFIC_SIGNAL.md) | 独立测试绿灯只播报，总流程衔接前进 |
| 循迹与锥桶 | [CONE_TEST](../../vision/CONE_TEST.md) | 区分识别观察、连续低速试验和严格米制规划 |
| 停车选区与低速入位 | [PARKING_BAY](../../vision/PARKING_BAY.md)、[parking_closed_loop_v2更新](../imports/2026-10-20-parking-fix/README.md) | 新版边界融合、有限暂停、末段减速与图像参考；实际入位仍待验收 |
| 路径跟踪入库 | [PARKING_PATH](../../vision/PARKING_PATH.md) | 独立页/总流程共用，实测表和反馈缺失时禁止启动 |
| 停车闭环方案 | [PARKING_CLOSED_LOOP](../../vision/PARKING_CLOSED_LOOP.md) | 反馈和标定准备条件另查 |
| 扫码缴费 | [PARKING_PAYMENT](../../vision/PARKING_PAYMENT.md) | 队员实际支付，程序保存人工回执 |
| 完整比赛流程 | [RACE_FLOW](../../vision/RACE_FLOW.md) | 根 `check-race-flow.cmd` 检查，`update-race-flow.cmd` 显式部署 |
| 米制自主框架 | [AUTONOMY_PI5](../../vision/AUTONOMY_PI5.md) | 观测、回放、准备检查与受限执行 |

## 硬件调试与标定

先查 [标定来源](../calibration-status.md) 和 [硬件基线](../../hardware/README.md)。

| 内容 | 指南 |
| --- | --- |
| 手动台架与有限试验 | [MANUAL_BENCH](../../vision/MANUAL_BENCH.md) |
| 网页 S3 脉宽调试 | [STEERING_PWM](../../vision/STEERING_PWM.md) |
| 转向台架 | [STEERING_BENCH](../../vision/STEERING_BENCH.md) |
| 电调台架 | [ESC_BENCH](../../vision/ESC_BENCH.md) |
| 台架测量与标定 | [BENCH_CALIBRATION](../../vision/BENCH_CALIBRATION.md) |

## 采集、标注与算法开发

| 内容 | 指南 |
| --- | --- |
| 红绿灯双摄采集和人工标注 | [TRAFFIC_LIGHT_CAPTURE](../../vision/TRAFFIC_LIGHT_CAPTURE.md) |
| 场地数据采集 | [DATA_COLLECTION](../../vision/DATA_COLLECTION.md) |
| 标注规范 | [ANNOTATION](../../vision/ANNOTATION.md) |
| 算法验证流程 | [VALIDATION](../../vision/VALIDATION.md) |
| 入门开发 | [BEGINNER_GUIDE](../../vision/BEGINNER_GUIDE.md)，结合当前硬件/目录阅读 |
| 模型来源和状态 | [模型索引说明](../../models/README.md) |

工具入口集中在 [工具导航](../../tools/README.md)；早期 `VISION_LEGACY`、相机测试和参考审计等说明的定位见 [历史索引](../history/README.md)。
