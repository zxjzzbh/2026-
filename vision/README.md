# 车辆应用工程

实际运行包位于 [src/carvision/](src/carvision/)。目录沿用早期名称 `vision`，目前已经包含双摄感知、网页遥控、比赛任务、执行器适配与软件测试。

[项目概览](../docs/overview.md) · [当前进度](../docs/current-status.md) · [操作指南](../docs/guides/README.md) · [原开发记录](../docs/history/vision-development-through-20261013.md)

## 工程布局

| 路径 | 用途 |
| --- | --- |
| [src/carvision/](src/carvision/) | Python 包，`python -m carvision` / `carvision` CLI |
| [configs/](configs/) | 感知阈值、分项任务、整场与标定配置 |
| [tests/](tests/) | 离线回归、模拟执行器、网页逻辑与小型夹具 |
| [tools/](tools/) | 采集、观察、台架、检查、打包与部署 |
| [deployment/](deployment/) | systemd 服务、开机检查和模板 |
| [pyproject.toml](pyproject.toml) | 依赖、CLI 注册、可选训练/导出/部署/测试依赖 |
| `*.md` | 具体功能指南；按主题查找见 [指南导航](../docs/guides/README.md) |

## 从任务找到实现

| 任务 | 代码组 | 指南 |
| --- | --- | --- |
| 双摄预览 | `cameras`、`sources`、`web_preview` | [LIVE_PREVIEW](LIVE_PREVIEW.md) |
| 开机、遥控、转向调试 | `boot_test`、`manual_*`、`gamepad_controls`、`steering_pwm_controls` | [BOOT_TEST](BOOT_TEST.md)、[STEERING_PWM](STEERING_PWM.md) |
| 挡板发车 | `departure*` | [DEPARTURE_TEST](DEPARTURE_TEST.md) |
| 斑马线制动 | `crosswalk*`、`brake_*` | [BRAKE_PARKING](BRAKE_PARKING.md) |
| 红绿灯 | `traffic_*` | [TRAFFIC_SIGNAL](TRAFFIC_SIGNAL.md) |
| 白线与锥桶 | `lane`、`cone_*` | [CONE_TEST](CONE_TEST.md) |
| 停车入位 | `parking_bay*`、`parking_guidance`、`parking_entry*`、`parking_mission` | [PARKING_BAY](PARKING_BAY.md) |
| 扫码缴费 | `parking_payment`、`payment_*` | [PARKING_PAYMENT](PARKING_PAYMENT.md) |
| 完整流程 | `race_flow*` | [RACE_FLOW](RACE_FLOW.md) |
| 自主框架与硬件 | `race`、`autonomy_*`、`rasadapter5*`、`pi5_pwm` | [AUTONOMY_PI5](AUTONOMY_PI5.md) |

代码组相对 `src/carvision/`；完整对应表见 [内容概览](../docs/overview.md)。有些功能指南保留不同日期的段落，部署结论与当前状态以 [状态页](../docs/current-status.md) 为准。

## 安装与软件检查

从仓库根目录、已激活的独立 Python 环境执行：

```sh
python -m pip install -e "vision[test]"
python -m pytest vision/tests -q -ra
python -m carvision autonomy-check --profile vision/configs/autonomy-pi5.json
```

浏览器逻辑测试需要 Node.js。安装步骤、回放与历史清单核验的区别见 [复现说明](../docs/reproduce.md)；连接和部署入口见 [工具导航](../tools/README.md)。

当前硬件是 Pi 5 + RasAdapter5A UART。早期 Pi4/GPIO 文档用于追溯；独立 STM32 方案已停用。完整自主驾驶与实际停车仍需完成现场验收。
