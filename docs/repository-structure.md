# 目录与维护约定

仓库按“运行工程、项目资料、数据索引、历史证据”查找。`vision/` 是当前实际应用；其他顶层模块目录中的 README 负责说明边界和指向实现，目录存在不代表另有一套程序。

## 目录职责

| 位置 | 放什么 | 当前性质 |
| --- | --- | --- |
| [vision/src/carvision/](../vision/src/carvision/) | 感知、控制台、比赛任务、硬件适配 | 实际运行代码 |
| [vision/configs/](../vision/configs/) | 应用默认配置、任务阈值、标定 | 实际配置；本机覆盖使用 `*.local.*` |
| [vision/tests/](../vision/tests/) | 与应用一起维护的测试和小型夹具 | 实际测试 |
| [vision/tools/](../vision/tools/) | 采集、台架、检查、打包、部署 | 应用配套工具 |
| [vision/deployment/](../vision/deployment/) | systemd、开机检查、部署模板 | 依赖已有 Pi 环境 |
| [pc-launcher/](../pc-launcher/) | Windows 一键连接及本地转发 | 18080 入口 |
| [docs/](README.md) | 概览、架构、进度、指南导航、复现、历史 | 项目文档 |
| [hardware/](../hardware/README.md) | 型号、接线和测量边界 | 硬件说明 |
| [interfaces/](../interfaces/README.md) | 观测、命令和当前 UART 协议 | 接口说明 |
| [vehicle/](../vehicle/README.md)、[communication/](../communication/README.md) | 主控、网络职责和实际实现位置 | 职责导航 |
| [configs/](../configs/README.md)、[tests/](../tests/README.md) | 共享配置与跨模块测试的约定 | 当前以应用内目录为准 |
| [data/](../data/README.md)、[models/](../models/README.md) | 数据、模型来源和外部存储索引 | 不存完整录像或大权重 |
| [tools/](../tools/README.md) | 仓库级审查、历史哈希检查、模拟页面 | 不与 `vision/tools/` 重复存源码 |
| [experiments/](../experiments/README.md) | 独立实验及复现说明 | 成熟后并入正式实现 |
| [drivers/pigpio/](../drivers/pigpio/README) | 早期 Pi4 路径第三方驱动 | 保留源码、来源和许可证 |
| [firmware/](../firmware/README.md) | 不采用的 STM32 方案说明 | 历史方案，不是当前开发缺口 |
| [run/](../run/) | 少量已跟踪的标定/核验 JSON | 新运行输出默认被 Git 忽略 |
| [.github/](../.github/) | CI、Issue 和 PR 模板 | 协作配置 |

## 文档分层

```text
docs/
├── latest-code.md          最新主线入口、固定源码包与校验
├── overview.md             项目内容、功能与代码地图
├── current-status.md       当前结论、证据入口、下一步
├── repository-structure.md 目录与维护约定
├── architecture.md         系统职责与数据流
├── calibration-status.md   参数版本与标定来源
├── reproduce.md            安装、离线检查、历史核验边界
├── guides/README.md        按操作任务查找 vision/ 中的指南
├── history/                开发和部署过程记录
├── imports/                外部模块包来源、文件清单与集成验证
├── archive/                按原版本日期归档的旧说明
├── handoff/                指定日期的完整交接材料
├── reviews/                审查、发布映射与核验记录
├── rules/ references/      比赛规则、参考来源
├── testing/ reports/       测试记录、技术报告
└── templates/              模块与实验记录模板
```

操作指南仍在 `vision/*.md`，由统一导航分类。现有启动、打包与交接资料引用这些路径，本轮不批量迁移运行工程或指南路径。

## 本轮文件整理

| 原位置 | 归档位置 |
| --- | --- |
| 根 `SOURCE_HISTORY_20261008.md` | [2026-10-08 源码历史](archive/2026-10-08/SOURCE_HISTORY_20261008.md) |
| 根 `源码包说明.md` | [2026-10-08 源码包说明](archive/2026-10-08/源码包说明.md) |
| 根 `CAMERA_GIMBAL_VERIFICATION.md` | [原包双摄验证](archive/2026-10-08/CAMERA_GIMBAL_VERIFICATION.md) |
| 根 `DEPLOYMENT_REPORT.md` | [原包部署报告](archive/2026-10-08/DEPLOYMENT_REPORT.md) |
| 原 `vision/README.md` 的长篇记录 | [视觉开发记录](history/vision-development-through-20261013.md)，原位置改为工程导航 |
| 原 `docs/current-status.md` 的过程记录 | [开发及部署记录](history/development-through-20261013.md)，原位置改为现状摘要 |
| 原 `docs/calibration-status.md` | [10 月 9 日标定记录](archive/2026-10-09/calibration-status.md)，原位置改为参数来源索引 |

原记录保留历史结论和数值，补充归档提示并调整相对链接。历史发布 JSON 清单保持原样，其中的旧路径与哈希用于还原当时版本，不能用于校验整理后的滚动主线。

## 后续内容放哪里

1. **新功能**：在现有应用包内按职责维护，测试随功能放 `vision/tests/`；在概览和指南导航中添加入口。
2. **当前结论**：更新 `docs/current-status.md`；不要向根 README 继续堆积逐次部署日志。
3. **一次实验/部署**：在 `docs/history/YYYY-MM-DD-主题.md` 记录版本、环境、结果与证据位置；可复用 [实验模板](templates/experiment.md)。
4. **操作步骤**：更新已有功能指南；新指南按明确主题命名，并在 `docs/guides/README.md` 注册。
5. **数据和权重**：更新 [数据索引](../data/catalog.csv) 或 [模型索引](../models/catalog.csv)；可公开小夹具与测试一起维护。
6. **历史版本**：保留 Git 提交/标签和必要说明，不增加“最终版2”等源码副本；分支用途见 [分支说明](branches.md)。

若以后拆分 `carvision`，应另开代码重构任务，逐步调整导入、CLI、部署文件列表与测试。本轮结构整理仅调整文档及其导航。
