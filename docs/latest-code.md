# 最新代码与固定源码包

发布日期：2026-10-20。

## 哪一份是最新代码

| 版本 | 用途 | 获取方式 |
| --- | --- | --- |
| `main` | 持续更新的最新已合并代码；队员日常开发统一入口 | [浏览源码](https://github.com/zxjzzbh/2026-/tree/main) · [下载 main ZIP](https://github.com/zxjzzbh/2026-/archive/refs/heads/main.zip) |
| `source-2026-10-20-r3` | 最新固定快照：r2完整主线加光流点对修复和14项回归测试 | [发布页](https://github.com/zxjzzbh/2026-/releases/tag/source-2026-10-20-r3) · [smartcar-source-20261020-r3.zip](https://github.com/zxjzzbh/2026-/releases/download/source-2026-10-20-r3/smartcar-source-20261020-r3.zip) · [SHA256](https://github.com/zxjzzbh/2026-/releases/download/source-2026-10-20-r3/smartcar-source-20261020-r3.zip.sha256) |
| `source-2026-10-20-r2` | 历史快照：含 PR #6 停车改进，仍有旧自主框架的光流点对异常 | [历史发布页](https://github.com/zxjzzbh/2026-/releases/tag/source-2026-10-20-r2) |
| `source-2026-10-20` | 之前仅合并 PR #7 的历史快照，不含 PR #6；标签和附件保持固定 | [历史发布页](https://github.com/zxjzzbh/2026-/releases/tag/source-2026-10-20) |
| 任务分支 / 未合并 PR | 正在开发或审阅的独立改动，不计入已合并主线 | [Pull requests](https://github.com/zxjzzbh/2026-/pulls) |

固定标签指向实际合并提交，完整提交号记在发布页。后续 `main` 可以继续更新，固定包不会随之改变；不要用解压文件夹的修改时间判断代码新旧。

`r3` 包保留 [PR #6](https://github.com/zxjzzbh/2026-/pull/6) 的功能，并修复后续确认的旧光流点对数量不一致问题。使用新标签和包名区分版本，不覆盖旧包；详见 [修复及验证记录](reviews/2026-10-20-ground-flow-fix.md)。

## 这份源码包包含什么

- `vision/` 的完整主线应用：双摄、5G 网页遥控、比赛分项、完整流程、停车入位试验与自主框架。
- PR #6 新增的路径跟踪入库、实测速度/曲率到 PWM 转换、白线丢失停车、远线跟踪和独立控制续期。
- `GroundVisualOdometry` 的点对完整性修复；覆盖越界、非有限坐标、有效点不足和位姿不可凭空恢复的14项回归。
- 应用配置、测试夹具、测试代码、采集/检查/部署工具及开机模板。
- Windows 启动入口、硬件与接口说明、第三方驱动及来源说明。
- 本次 README 两张结构图、内容概览、当前状态、操作导航和历史归档。
- Git 中已有的少量历史核验/标定证据。原始 JSON 清单描述其原版本，不是本源码包的新校验清单。

ZIP 直接从固定提交生成，只包含 Git 跟踪文件，不附带 `.git`、本地未提交修改、Python 环境、个人连接凭据、大录像或模型权重。压缩包内的根目录为 `smartcar-source-20261020-r3/`。它是源码交付包；部署仍按对应工具和操作指南执行，解压不会安装车端环境或启动车辆。

## 获取、核验与安装

需要继续开发时，用 Git 获取主线：

```sh
git clone --branch main https://github.com/zxjzzbh/2026-.git
cd 2026-
git log -1 --oneline
```

需要固定的本次版本时，在独立目录获取标签：

```sh
git clone --branch source-2026-10-20-r3 --single-branch https://github.com/zxjzzbh/2026-.git smartcar-source-20261020-r3
cd smartcar-source-20261020-r3
git rev-parse HEAD
```

标签检出为固定提交；需要开发时先新建自己的任务分支。通过 ZIP 下载的用户可在 Windows PowerShell 执行：

```powershell
Get-FileHash .\smartcar-source-20261020-r3.zip -Algorithm SHA256
Get-Content .\smartcar-source-20261020-r3.zip.sha256
```

将输出与同一发布页的 SHA256 文件核对，解压后从项目根目录按 [安装与复现](reproduce.md) 建立环境。主要运行包在 `vision/src/carvision/`，实际配置在 `vision/configs/`。

完整功能范围见 [内容概览](overview.md)，实车部署和未验收项见 [当前进度](current-status.md)。主线合并、软件检查通过和实车验收分别记录。
