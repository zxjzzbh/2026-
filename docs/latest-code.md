# 最新源码 · 一键测试版

当前版本为 `smartcar_oneclick_zebra_reverse1590_20261120`。[下载完整源码 ZIP](https://github.com/zxjzzbh/2026-/releases/download/smartcar-oneclick-zebra-reverse1590-20261120/smartcar-oneclick-zebra-reverse1590-20261120.zip) · [下载校验文件](https://github.com/zxjzzbh/2026-/releases/download/smartcar-oneclick-zebra-reverse1590-20261120/smartcar-oneclick-zebra-reverse1590-20261120.zip.sha256)

接近斑马线和红绿灯时，有当前车道观测的巡航油门为 1590 微秒。斑马线用 1300 微秒反向脉冲，目标时长 0.12 秒，随后回到 1500 微秒中位。实际停车落点待实车测量。

调试页 `/debug/` 选择项目后，点“开始测试”。车端保持 3 秒中位后启动；准备中或运行中都可以点“结束测试”。“仅观察识别”直接采集图像。比赛页 `/competition/` 保留正式遥控交接流程。

- `cpp/` 包含比赛入口、控制器、感知算法、配置和离线回归。
- `vision/` 包含 5G 服务、比赛页和调试页。`deployment/` 包含车端服务文件。
- 源码包 SHA-256 为 `1f3a88964e5c0e69c9291fb4cb65804f0fd9f1f1bc1d95f1d601db54baec7dcd`。仓库运行文件与压缩包逐文件核对。[仓库校验清单](../SOURCE-MANIFEST.json)
- 电脑端 10 组离线测试及其中 32 项 5G 服务测试通过。此版本尚未上车验证。[使用说明](../README.md) · [验证方法](reproduce.md) · [部署状态](../DEPLOYMENT.json)
