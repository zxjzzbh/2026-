# 工具与快捷入口导航

从仓库根目录使用下列入口。Python 依赖和离线环境见 [复现说明](../docs/reproduce.md)，任务操作说明见 [指南导航](../docs/guides/README.md)。

## 根目录 Windows 快捷入口

| 入口 | 用途 | 执行范围 |
| --- | --- | --- |
| [connect-car.cmd](../connect-car.cmd) | 检查/恢复本机连接，打开8082控制台 | 会访问车辆网络；不自动启用驾驶 |
| [check-race-flow.cmd](../check-race-flow.cmd) | 配置检查及总流程软件测试 | 离线 |
| [check-parking-local.cmd](../check-parking-local.cmd) | 停车软件测试 | 离线，要求根 `.venv` |
| [run-cone-observe.cmd](../run-cone-observe.cmd) | 读取8082画面观察锥桶 | 在线观察，要求根 `.venv` |
| [update-departure-test.cmd](../update-departure-test.cmd) | 更新发车测试 | 显式连接并部署 |
| [update-parking-bay.cmd](../update-parking-bay.cmd) | 更新停车模块 | 显式连接并部署 |
| [update-race-flow.cmd](../update-race-flow.cmd) | 更新完整比赛流程 | 显式连接并部署 |
| [pc-launcher/start.cmd](../pc-launcher/start.cmd) | 另一套Windows连接启动器 | 18080入口，先配置 `SMARTCAR_HOST` |

`update-*` 依赖现有车端环境和个人连接配置，不是离线检查。新电脑优先建立本仓库 `.venv`，不依赖部分旧脚本对相邻工作目录的兼容回退。

## 应用配套工具

以下路径相对 [vision/tools/](../vision/tools/)。同一工具的检查/实车行为取决于子命令和参数，执行前阅读对应任务指南。

| 类别 | 工具 |
| --- | --- |
| 连接与预览 | `console_forward.py`、`connection_helpers.ps1`、`manage_pi_preview.py` |
| 数据采集与分析 | `traffic_capture.py`、`traffic_signal.py`、`capture_crosswalk_reference.py`、`crosswalk_lab.py` |
| 单项观察与检查 | `cone_trial.py`、`parking_bay.py`、`race_flow.py`、`check_parking_feedback.py` |
| 车端服务 | `serve_boot_test.py`、`serve_bench_drive.py`、`serve_race_flow.py`、`serve_parking_payment.py` |
| 设备探测 | `probe_camera.py`、`probe_5g_module.py`、`probe_rasadapter5.py`、`probe_tts_module.py` |
| 台架与有限输出 | `bench_pi5_neutral.py`、`bench_pi5_gimbal.py`、`bench_original_esc.py`、`brake_parking.py` |
| 打包和部署 | `build_delivery.py`、`build_autonomy_delivery.py`、`deploy_*.py` |

## 仓库级工具

实际相机、5G、串口、云台、台架、打包与显式部署工具位于 `vision/tools/`，具体实车入口依赖平台及准备条件。

本目录的 `audit_snapshot_20261008.py` 是离线审查复现脚本：在根目录安装 vision[test] 后运行 `python tools/audit_snapshot_20261008.py`，不访问摄像头、UART 或 GPIO。它展示已知问题，不是实车验收或自动修复。

日志默认保存在本地 run/；不要将密码、个人连接配置或大录像提交到公开仓库。

`verify_publication.py` 只核验 **10月9日历史快照**的文件哈希，应在 `archive/pi5-verified-20261009` 标签使用；不适用于已继续演进的主线。`serve_pwm_demo.py` 是本机模拟网页，默认端口8931；其中1715 μs为独立模拟示例，不是当前实车中位。
