# 当前 5G 与远程通信

当前源码使用 Python HTTP/1.1 控制接口、MJPEG 视频和原生 JavaScript 网页；车端实现位于 `vision/src/carvision/web_preview.py`、`manual_controls.py`、`manual_drive.py`。本包没有 WebRTC 运行链路。

源码包历史记录中的蜂窝模块是移远 RM500U-CNV，使用 USB NCM，经 Tailscale 访问车端；记录有断开有线/Wi-Fi 后的 NR5G-SA 有限低速短测。原始完整网络日志未随包交付，本次没有重新联网或测车。

HTTP 长连接、心跳和控制轮次已经实现。200 ms 控制期限、250 ms 输出期限仍保留；短时 RTT 不能代表持续视频负载下 P99 时延。网页随机控制键不是登录身份认证，使用时还需可信网络/已审查 Tailscale ACL。

个人 IP/热点字段使用占位符；部署地址显式填写，不把凭证放入 Git。下一步是实测控制与视频时延分布、断网/拥塞、远程到自主的停止与串口所有权交接。详见 [当前状态](../docs/current-status.md)。
