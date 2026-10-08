# 当前接口

2026-10-08。现有运行代码在 `vision/src/carvision`，无 Pi→独立 STM32 协议。

| 接口 | 依据 |
|---|---|
| 图像感知 | [vision-result-v0.1](vision-result-v0.1.md)、`results.py` |
| 比赛观测/意图 | [race-observation-0.2](race-observation-0.2.md)、`race.py` |
| 自主实时封装 | `autonomy_runtime.validate_envelope`：schema_version=1，live/replay、sequence、captured_monotonic_s、observation；250 ms 期限 |
| 网页手动命令 | `web_preview.py` + `manual_drive.py`：HTTP JSON、运行期控制键、控制轮次、心跳与急停 |
| Pi→RasAdapter | `rasadapter5.py`：`AA 55`、功能、长度、负载、CRC8；UART 1 Mbaud；S1/S2 云台、S3 转向、S4 ESC |
| 视频 | 原图/处理图/掩膜 JPEG 与 multipart MJPEG；双摄 by-id |
| 真实反馈 | `autonomy_feed.py` 读取本机原子更新 JSON；编码器/位姿采集仍待接通 |

同一 Pi 单调时钟的消息不能与不同机器时钟直接混用。板内 PWM 回读仅说明保存的命令；观测中的 encoder 字段不说明硬件已有编码器。接口的未知、失效、模拟与实测状态必须分别处理。
