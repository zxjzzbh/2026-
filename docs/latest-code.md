# 最新源码 · 一键测试版

当前版本为 `smartcar_oneclick_20261119`。[下载完整源码 ZIP](https://github.com/zxjzzbh/2026-/releases/download/smartcar-oneclick-20261119/smartcar-oneclick-20261119.zip) · [下载校验文件](https://github.com/zxjzzbh/2026-/releases/download/smartcar-oneclick-20261119/smartcar-oneclick-20261119.zip.sha256)

调试页 `/debug/` 选择项目后，点“开始测试”。车端保持 3 秒中位后启动；准备中或运行中都可以点“结束测试”。“仅观察识别”直接采集图像。比赛页 `/competition/` 保留正式遥控交接流程。

- `cpp/` 包含比赛入口、控制器、感知算法、配置和离线回归。
- `vision/` 包含 5G 服务、比赛页和调试页。`deployment/` 包含车端服务文件。
- 源码包 SHA-256 为 `abe8e9c295a723f16ad51317390ff226d6530c1f50f0dac59f5ef8403bb2ec53`。仓库运行文件与压缩包逐文件核对。[仓库校验清单](../SOURCE-MANIFEST.json)
- 电脑端 10 组离线测试及其中 32 项 5G 服务测试通过。此版本尚未上车验证。[使用说明](../README.md) · [验证方法](reproduce.md) · [部署状态](../DEPLOYMENT.json)
