# 最新代码与固定源码包

主线最近同步：2026-10-28；代码来源为2026-10-26最后本地修复版。

当前完整代码请使用 `main` 或下载 main ZIP。10月20日 r5 仍是最近一个固定发布包，它不会自动包含本次同步；旧标签和附件保持原样。

本次只有仓库上传，未部署车辆。范围和独立下载验证见[本次同步记录](imports/2026-10-26-local-fix/README.md)。

## 哪一份是最新代码

| 版本 | 用途 | 获取方式 |
| --- | --- | --- |
| `main` | 持续更新的最新已合并代码；队员日常开发统一入口 | [浏览源码](https://github.com/zxjzzbh/2026-/tree/main) · [下载 main ZIP](https://github.com/zxjzzbh/2026-/archive/refs/heads/main.zip) |
| `source-2026-10-20-r5` | 最新固定快照：PR #10停车修复版与r4完整主线 | [发布页](https://github.com/zxjzzbh/2026-/releases/tag/source-2026-10-20-r5) · [smartcar-source-20261020-r5.zip](https://github.com/zxjzzbh/2026-/releases/download/source-2026-10-20-r5/smartcar-source-20261020-r5.zip) · [SHA256](https://github.com/zxjzzbh/2026-/releases/download/source-2026-10-20-r5/smartcar-source-20261020-r5.zip.sha256) |
| `source-2026-10-20-r4` | 历史快照：包含PR #9，不含本次parking_closed_loop_v2修复 | [历史发布页](https://github.com/zxjzzbh/2026-/releases/tag/source-2026-10-20-r4) |
| `source-2026-10-20-r3` | 历史快照：包含光流修复，不含PR #9实车模块导入 | [历史发布页](https://github.com/zxjzzbh/2026-/releases/tag/source-2026-10-20-r3) |
| `source-2026-10-20-r2` | 历史快照：含 PR #6 停车改进，仍有旧自主框架的光流点对异常 | [历史发布页](https://github.com/zxjzzbh/2026-/releases/tag/source-2026-10-20-r2) |
| `source-2026-10-20` | 之前仅合并 PR #7 的历史快照，不含 PR #6；标签和附件保持固定 | [历史发布页](https://github.com/zxjzzbh/2026-/releases/tag/source-2026-10-20) |
| 任务分支 / 未合并 PR | 正在开发或审阅的独立改动，不计入已合并主线 | [Pull requests](https://github.com/zxjzzbh/2026-/pulls) |

固定标签指向实际合并提交，完整提交号记在发布页。后续 `main` 可以继续更新，固定包不会随之改变；不要用解压文件夹的修改时间判断代码新旧。

`parking_closed_loop_v2`通过 [PR #10](https://github.com/zxjzzbh/2026-/pull/10) 整合进完整主线，r5由实际合并提交生成；r4及更早固定包保持原样，不覆盖旧附件。来源、兼容处理和完整回归见 [停车修复导入记录](imports/2026-10-20-parking-fix/README.md)。

此前的实车避障/停车模块通过 [PR #9](https://github.com/zxjzzbh/2026-/pull/9) 整合为r4；r5继续保留其完整功能。两个上传包都是部分模块包，均保留包外主线文件。第一轮导入来源见 [r4导入说明](imports/2026-10-20-car-deployed/README.md)。

`r5` 保留 [PR #6](https://github.com/zxjzzbh/2026-/pull/6) 的功能和r3光流修复。使用新标签和包名区分版本，不覆盖旧包；光流修复边界见 [验证记录](reviews/2026-10-20-ground-flow-fix.md)。

## 这份源码包包含什么

- `vision/` 的完整主线应用：双摄、5G 网页遥控、比赛分项、完整流程、停车入位试验与自主框架。
- PR #6 新增的路径跟踪入库、实测速度/曲率到 PWM 转换、白线丢失停车、远线跟踪和独立控制续期。
- PR #9的绕桶边缘PD、目标释放回中、停车中心/朝向PD和配套测试；同时包含包内的首次斑马线短停与固定转向短测基线。
- PR #10的白线/车位证据融合、有限暂停恢复、受控反打、寻位/总时限、末段减速及独立图像停车参考；部署清单同步包含新模块和回归输入。
- `GroundVisualOdometry` 的点对完整性修复；覆盖越界、非有限坐标、有效点不足和位姿不可凭空恢复的14项回归。
- 应用配置、测试夹具、测试代码、采集/检查/部署工具及开机模板。
- Windows 启动入口、硬件与接口说明、第三方驱动及来源说明。
- 本次 README 两张结构图、内容概览、当前状态、操作导航和历史归档。
- Git 中已有的少量历史核验/标定证据。原始 JSON 清单描述其原版本，不是本源码包的新校验清单。

ZIP 直接从固定提交生成，只包含 Git 跟踪文件，不附带 `.git`、本地未提交修改、Python 环境、个人连接凭据、大录像或模型权重。压缩包内的根目录为 `smartcar-source-20261020-r5/`。它是源码交付包；部署仍按对应工具和操作指南执行，解压不会安装车端环境或启动车辆。

## 获取、核验与安装

需要继续开发时，用 Git 获取主线：

```sh
git clone --branch main https://github.com/zxjzzbh/2026-.git
cd 2026-
git log -1 --oneline
```

需要固定的本次版本时，在独立目录获取标签：

```sh
git clone --branch source-2026-10-20-r5 --single-branch https://github.com/zxjzzbh/2026-.git smartcar-source-20261020-r5
cd smartcar-source-20261020-r5
git rev-parse HEAD
```

标签检出为固定提交；需要开发时先新建自己的任务分支。通过 ZIP 下载的用户可在 Windows PowerShell 执行：

```powershell
Get-FileHash .\smartcar-source-20261020-r5.zip -Algorithm SHA256
Get-Content .\smartcar-source-20261020-r5.zip.sha256
```

将输出与同一发布页的 SHA256 文件核对，解压后从项目根目录按 [安装与复现](reproduce.md) 建立环境。主要运行包在 `vision/src/carvision/`，实际配置在 `vision/configs/`。

完整功能范围见 [内容概览](overview.md)，实车部署和未验收项见 [当前进度](current-status.md)。主线合并、软件检查通过和实车验收分别记录。
