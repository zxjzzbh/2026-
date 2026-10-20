# 2026 室外 5G 远程驾驶无人车

基于 **Raspberry Pi 5 + RasAdapter5A 串口扩展板 + 双 USB 摄像头**的比赛项目，包含 5G 网页遥控、视觉感知、分项测试与完整比赛流程。当前不使用独立 STM32，不依赖 ROS。

**从 [项目内容概览](docs/overview.md) 开始。** 概览介绍各项功能、代码位置和完成程度；[当前进度](docs/current-status.md) 区分主线代码、历史部署与待验收项。完整自主赛道和停车入位尚未通过实车验收。

## 按需求查找

| 我要做什么 | 入口 |
| --- | --- |
| 了解项目有哪些内容 | [内容概览](docs/overview.md) |
| 找代码、配置、工具或资料 | [目录与维护约定](docs/repository-structure.md) |
| 连接小车、测试一个比赛任务 | [操作指南导航](docs/guides/README.md) |
| 查看已完成内容和下一步 | [当前进度](docs/current-status.md) |
| 在新电脑安装并做离线检查 | [安装与复现](docs/reproduce.md) |
| 理解数据流、硬件和接口 | [系统架构](docs/architecture.md) · [硬件](hardware/README.md) · [接口](interfaces/README.md) |
| 参与开发和提交代码 | [贡献指南](CONTRIBUTING.md) · [团队分工](docs/team.md) |
| 追溯旧版本与部署记录 | [历史记录](docs/history/README.md) · [源码包归档](docs/archive/README.md) · [分支说明](docs/branches.md) |

## 仓库结构

```text
2026-/
├── vision/                 实际运行工程：感知、控制、比赛任务与网页
│   ├── src/carvision/      Python 运行包
│   ├── configs/            运行配置与标定
│   ├── tests/              离线回归、浏览器逻辑与图片夹具
│   ├── tools/              采集、检查、部署与台架工具
│   └── deployment/         systemd 与开机检查模板
├── pc-launcher/            Windows 连接启动器（18080）
├── docs/                   概览、当前进度、指南导航、历史与归档
├── hardware/ interfaces/   硬件说明与跨模块协议
├── vehicle/ communication/ 职责导航；实现位置见各目录 README
├── configs/ tests/         共享配置／集成测试的职责索引
├── data/ models/           数据与模型来源索引
├── tools/                  仓库级审查、历史核验、模拟工具
├── experiments/            独立实验约定
├── drivers/ firmware/      早期驱动／方案说明，见目录约定
└── run/                    少量历史标定证据；新日志默认留在本地
```

根目录的 `.cmd` 是兼容现有操作习惯的快捷入口；`SOURCE-MANIFEST.json` 与 `source-snapshot.json` 是历史来源清单。各目录的准确职责见 [目录说明](docs/repository-structure.md)。

## 比赛任务

5G 远程驾驶 → 换区停稳并手动切换 → 蓝挡板移开 → 自主循迹 → 斑马线停车播报 → 红绿灯 → 两桶避障 → 空闲半区停车 → 队员扫码缴费。

单项测试与完整流程的行为有区别，例如红绿灯单测在绿灯确认后只播报，完整流程才继续下一阶段。详见 [任务指南](docs/guides/README.md) 和 [规则记录](docs/rules/README.md)。

## 电脑离线检查

推荐 Python 3.12；浏览器逻辑测试需要 Node.js，CI 使用 Node.js 22。从仓库根目录执行：

```sh
python -m venv .venv
# Windows PowerShell:
.venv/Scripts/Activate.ps1
# Linux 使用：source .venv/bin/activate
python -m pip install -e "vision[test]"
python -m pytest vision/tests -q -ra
python -m carvision autonomy-check --profile vision/configs/autonomy-pi5.json
```

这些是软件检查。当前自主配置仍有未完成标定项；离线检查或模拟完成不等于实车完赛。更多回放命令见 [复现说明](docs/reproduce.md)。

公开仓库保留源码、可公开配置和小型测试夹具；个人凭据、大录像、模型权重与大部分运行日志通过本地配置或索引管理。团队尚未选择整体开源许可证，第三方源码保留各自许可证。
