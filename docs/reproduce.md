# 复现与公开发布说明

## 离线复现

从仓库根目录开始。推荐 Python 3.12；浏览器逻辑测试需要 Node.js，CI 使用 Node.js 22。以下用于滚动主线：

```sh
python -m venv .venv
# Windows PowerShell:
.venv/Scripts/Activate.ps1
# Linux 使用：source .venv/bin/activate
python -m pip install -e "vision[test]"
python -m pytest vision/tests -q -ra
python -m carvision autonomy-check --profile vision/configs/autonomy-pi5.json
python -m carvision autonomy-demo --profile vision/configs/autonomy-pi5.json --race-config vision/configs/race-2026.json --output run/new-offline-demo
python tools/audit_snapshot_20261008.py
```

输出目录使用未存在的新目录。`autonomy-check` 的 18 个缺项是预期门控结果；demo complete 仅为离线回放。上述检查不启动摄像头、UART、GPIO、车载语音或运动服务。测试的 HTTP/TCP 使用本机回环与模拟设备。

主要运行依赖与测试依赖固定在 [vision/pyproject.toml](../vision/pyproject.toml)；可选训练/导出依赖独立声明。`vision[test]` 已包含部署入口测试需要的 Paramiko；只使用部署工具可安装 `python -m pip install -e "vision[deploy]"`。pigpio 的来源和许可证保留在 `drivers/pigpio/`，其 Pi4 路径不是本车当前后端。整体项目许可证仍由团队决定。

`tools/audit_snapshot_20261008.py` 在现行代码上复现当时审查的问题，不是新的实车验收。`tools/verify_publication.py` 校验固定历史哈希，仅适用于 `archive/pi5-verified-20261009` 标签；不要在滚动主线上把历史清单不匹配当成安装失败，也不要重写旧哈希。版本入口见 [分支说明](branches.md)。

## 电脑入口

日常 8082 控制台使用根目录 `connect-car.cmd`，详细连接检查和恢复流程见 [CONSOLE_ACCESS](../vision/CONSOLE_ACCESS.md)。以下是另外保留的 18080 启动器，使用前明确自己要访问的车端服务。

Windows 安装 Python（含 Tk）及 Tailscale，先获得车辆访问授权。在用户环境变量设置 `SMARTCAR_HOST` 为管理者私下提供的设备地址，重新登录或在带该变量的终端执行 `pc-launcher/start.cmd`。个人访问地址通过环境配置提供，不提交账号、邀请、密码或私钥。未设置地址会明确提示且不会联系车辆。

入口等待已有网络并打开 `http://127.0.0.1:18080/`；只在本地转发，不负责创建 Tailscale 账号、5G 拨号或绕过网络授权。车端已有等待页服务才可使用。网页各准备/启用步骤、失联和急停行为见 [BOOT_TEST](../vision/BOOT_TEST.md) 与 [STEERING_PWM](../vision/STEERING_PWM.md)。本次没有进行这些实车步骤。

## 车端部署资料与边界

车端历史部署根目录为 `/home/pi/smartcar-race-20261002`，服务名为 `smartcar-preview.service`。10 月 9 日等待页入口为 `vision/tools/serve_boot_test.py`；后续比赛流程使用 `vision/tools/serve_race_flow.py --race-live`。实际入口应核对当前服务及部署记录，不能用旧模板直接覆盖。仓库中的 `smartcar-boot-test.service` 是备用模板，不可同时启动两个争用 8080 的服务。

公开目录保留启动、部署、打包脚本及配置。`vision/tools/deploy_autonomy_pi5.py` 保留队友的显式 `--host` 和已信任主机密钥校验，不默认使用个人地址，不会自行启动服务。它是自主组件的显式部署工具，不替代整车第一次系统安装；系统 UART/V4L2/systemd/Tailscale 与 5G 模块配置依赖现有已配置环境，本仓库不是系统镜像。

重新部署前比较车端与发布清单，保留本机标定和服务备份；修改源码需要重跑检查并更新对应清单。不要把本次公开版软件哈希映射理解成刚刚完成车端复验。本次没有连接 UART、发送 PWM/遥控、重启或部署到车。

## 清单、历史与敏感信息

- 10 月 9 日公开快照：[publication-manifest-20261009.json](reviews/publication-manifest-20261009.json)，仅在对应历史标签运行 `tools/verify_publication.py` 校验。
- 10 月 9 日部署的本地原始哈希及核验时间：[deployment-baseline-20261009.json](reviews/deployment-baseline-20261009.json)。仅 39 个条目具有该次部署清单证据，其余文件按旧包/远端来源注明。
- `vision/deployment/boot-test-checks.json` 保留历史 Pi 检查结果，文件哈希映射到当日公开版；附带 public_repack 元数据，明确未在 Pi 上复验。它不会替代运行时现场确认或当前部署清单。
- `SOURCE-MANIFEST.json`、`source-snapshot.json`、[10 月 8 日源码说明](archive/2026-10-08/SOURCE_HISTORY_20261008.md) 与当日发布映射是历史，不覆盖之后新增文件或文档迁移。
- 排除真实密码 `connection.json`、私密 `HANDOFF.md`、SSH 私钥、Tailscale 状态/邀请/令牌、Wi-Fi 凭据、环境、缓存、系统镜像、大录像和模型权重。连接信息只通过个人环境配置提供。
- 模型来源、版本与 SHA256 在 `models/catalog.csv` 和 `vision/configs/model-source.json`；模型尚未通过比赛验收。离线检查不需要下载模型或私人数据。测试使用合成输入及仓库内少量实拍/JSON 回放夹具；完整训练集不在仓库内。
