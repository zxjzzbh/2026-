# 当前 5G 与远程通信

当前源码使用 Python HTTP/1.1 控制接口、MJPEG 视频和原生 JavaScript 网页；车端实现位于 `vision/src/carvision/web_preview.py`、`manual_controls.py`、`manual_drive.py`。本包没有 WebRTC 运行链路。

源码包历史记录中的蜂窝模块是移远 RM500U-CNV，使用 USB NCM，经 Tailscale 访问车端；记录有断开有线/Wi-Fi 后的 NR5G-SA 有限低速短测。原始完整网络日志未随包交付，本次没有重新联网或测车。

HTTP 长连接、心跳和控制会话已经实现。持续遥控新鲜输入期限为500 ms；短测/S3调试仍为200 ms，内部输出保护250 ms，具体路径见 [系统架构](../docs/architecture.md)。短时 RTT 不能代表持续视频负载下 P99 时延。网页随机控制键不是登录身份认证，使用时还需可信网络/已审查 Tailscale ACL。

电脑连接入口：根 `connect-car.cmd` 使用本机8082；`pc-launcher/start.cmd` 是保留的18080启动器。前者见 [控制台连接](../vision/CONSOLE_ACCESS.md)，后者见 [启动器说明](../pc-launcher/使用说明.txt)。实际网络/转发工具在 `vision/tools/`，此目录负责通信导航。

新电脑需核对自己的连接配置，历史记录中的地址不保证适用；不把凭证放入 Git。下一步是实测控制与视频时延分布、断网/拥塞、远程到自主的停止与串口所有权交接。详见 [当前状态](../docs/current-status.md)。
