# 最新组合源码

当前版本为 smartcar_outdoor_parking1618_cone1115_20261120。[下载完整源码 ZIP](https://github.com/zxjzzbh/2026-/releases/download/smartcar-outdoor-parking1618-cone1115-20261120/smartcar-outdoor-parking1618-cone1115-20261120.zip) · [下载校验文件](https://github.com/zxjzzbh/2026-/releases/download/smartcar-outdoor-parking1618-cone1115-20261120/smartcar-outdoor-parking1618-cone1115-20261120.zip.sha256)

常规巡线、发车和斑马线使用 11 月 20 日斑马线丢失制动修复附件，语音使用扩展板语音修复附件。锥桶使用 GitHub 11 月 15 日比赛版，停车使用本次 16:18 室外停车单项包。完整模块来源、运行方式和参数在 [README](../README.md)。

接近阶段有当前车道时使用 1600 微秒。红绿灯在约一米处先停车，停稳后等待同灯组绿灯。斑马线制动脉冲完成并回中位即开始十秒计时和播报。停车单项和全流程末段共用新停车模块。

Windows 本地构建与 17 组离线测试通过。停车录像回放中，反光第 9 帧选左库，近景第 106 帧选左库，进场 80 帧未锁定库位。三段均未报入库完成。源码已发布，车端部署与实车完整流程验证仍待完成。

源码包 SHA-256 为 c367371dcb16cb1f2fcd3dbe44fa559e7d8b316cb54278bb44c9690e82e548b4。归档运行文件与仓库 cpp、vision、deployment 三个目录逐字节核对，仓库首页额外包含下载入口。[仓库文件校验](../SOURCE-MANIFEST.json) · [历史部署记录](../DEPLOYMENT.json)
