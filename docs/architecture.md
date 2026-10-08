# 当前系统架构

更新：2026-10-08。当前使用 Pi 5 + RasAdapter5A UART，不使用独立 STM32，不依赖 ROS。实际程序暂集中在 `vision/src/carvision`，顶层目录是职责索引。

## 数据与控制流

1. `sources/cameras` 明确选择双摄，采集线程只保留新帧；离线录像按帧读取。
2. `lane/classic_detection/detection/traffic_lamps` 生成感知；`pipeline/temporal` 完成统一结果与跨帧确认。
3. `web_preview/manual_controls/gamepad_controls` 提供 HTTP/MJPEG 画面和网页操作。5G/Tailscale 是网络通路，不改变本地感知职责。
4. 手动有限测试由 `manual_drive` 和 `pi5_pwm` 执行；当前板卡由 `rasadapter5` 以 UART/CRC8 适配。S1/S2 云台，S3 转向，S4 ESC。
5. 自主链路为 `autonomy_feed → autonomy_observation → race → autonomy_motion → RasAdapter`。JSONL 使用同一 Pi 单调时钟；实时输出要求完整实测配置，离线输入不能授权运动。
6. `autonomy_audio` 管理真实播报完成；`ground_odometry` 是待验证光流方案，不能充当已安装编码器的证据。

## 已建立的约束

- 图像像素与米制地面坐标分开；后轴中心为地面原点，x 向右、y 向前。
- 相机姿态、分辨率和标定区域必须匹配。当前两路近距仅到车头前 50 cm，斑马线 25 cm 停车和四轮入位仍未串通。
- 遥控与自主必须独占 UART；真实停稳后交接，不能两个进程同时写。
- 接收帧龄不是镜头曝光到浏览器显示的端到端时延；PWM 输出值不是车速/实际舵角。
- 手动心跳 200 ms、输出期限 250 ms 是软件机制，整机失效与扩展板失联行为另需量测。
- 预览开机服务保持 `--controls-paused`；原包的历史 review 不代表本次开机可启用。

## 待完成

闭环反馈与速度/制动/转向标定、已验证比赛检测器、近场地面目标跟踪、红蓝跑道停车占位、播报回执、5G 控制权交接和完整实车验收。细节见 [审查报告](reviews/2026-10-08-source-review.md)。

未来拆包应逐步把感知、比赛、控制、硬件适配和静态网页资源隔离，保留稳定接口与回归测试；本次同步没有做运行代码搬迁。
