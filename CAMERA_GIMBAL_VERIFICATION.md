# 双摄像头与云台验证，2026-10-02

> 2026-10-08 同步说明：当前为 Pi 5 + RasAdapter5A UART，无独立 STM32。此文保留不同日期记录，当前进度以 [状态页](docs/current-status.md) 为准；旧 GPIO 接线、连接占位符和历史 review 不是本次实车启动依据。

- 已更新部署 v0.2.1 到 `/home/pi/smartcar-race-20261002`，82 个源码/配置/驱动源文件与本地 SHA256 清单一致。
- 更新前备份：`/home/pi/smartcar-race-backups/before-camera-gimbal-20261002T031953Z.tar.gz`。
- 本地 100 项测试通过（2.62 s），Pi 100 项通过（14.80 s），JavaScript 语法检查通过。
- 两台 USB 相机：Sunplus 1bcf:2281 与 Microdia H65 0c45:6368；分别用稳定 by-id 路径选择，元数据节点不计为额外相机。
- 双路 YUYV 预览中发现 H65 花屏，改为明确请求并核实 MJPG 后实际照片恢复正常。当前两台 640×480 同时出图，原始照片已人工查看。
- 实时预览运行中，PID 3039，网页 `http://PI_LAN_ADDRESS:8080/` 可切换相机；未设置开机启动。
- 未验证任何米制深度流；不把 UVC 图像当深度数据。
- 本项目目录编译 pigpio 79；以 GPIO 更新掩码 0 临时启动 5 s，只读查询版本和引脚模式。IPv4 回环连接验证通过，未发送真实 servo/wave 命令，驱动已结束。
- 收尾 BCM12/13/17/27 均保持 input/pulldown/low。底盘两路硬件 PWM 的预留不变，云台信号分配 BCM17/物理11 与 BCM27/物理13。
- 舵机型号、供电电压、3.3 V 信号兼容性与三点脉宽未确认，真实运动入口仍关闭。未测试实际动作、负载电源和脉冲抖动。
- 5G 模块未接入，本轮通过网线验证。

接线见 [CAMERAS_GIMBAL.md](vision/CAMERAS_GIMBAL.md)，原始记录见 [verification-cameras-gimbal-20261002](verification-cameras-gimbal-20261002/)。
