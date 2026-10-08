# Pi 5 自主阶段集成版（2026-10-05）

> 2026-10-08 同步说明：当前为 Pi 5 + RasAdapter5A UART，无独立 STM32。此文保留不同日期记录，当前进度以 [状态页](../docs/current-status.md) 为准；旧 GPIO 接线、连接占位符和历史 review 不是本次实车启动依据。

本版把比赛决策、双摄观测、距离转换、速度反馈、S3/S4 执行、播报和故障日志串起来。默认不输出硬件命令。完整模拟走通不代表真实赛道通过；实际配置仍保留未测量项和全部未验证标志。

## 已提供的软件

- `autonomy_runtime.py`：版本化 JSONL 输入、最新帧队列、250 ms 超时、重复/未来/过期输入拒绝、故障锁定、信号退出和 finally 清理；模拟播报回执由运行层生成，外部输入不能伪造成功。
- `autonomy_profile.py`：车辆尺寸、速度曲线、转向、摄像头地面映射、供反馈/播报的配置检查；相机分辨率或云台位置改变时映射失效；地面标定输出独立检查点误差，不自动确认完成。
- `autonomy_observation.py`：主摄优先、主摄缺项时第二路补充、双路红绿灯冲突时输出 unknown；编码器反馈、稳定锥桶 ID、车头/前轮到任务线距离、两停车位和整块轮胎入位检查。
- `autonomy_geometry.py`：已标定地面的斑马线近边、锥桶接地点、黄色胶带与白色跑道边线围成的完整停车框及占位检测。灯箱框不作为地面停车线；停车线支持实测场地地图与真实定位。2026-10-06 修复仅识别白线而漏掉规则黄色胶带的问题；黄线全围合和黄白混合边界、左右挡板的合成图检查通过，真实场景仍待验证。红色跑道等未验证地面仍报告占位未知，不能据此授权驶入。
- `ground_odometry.py`：可选固定主摄地面光流测速和定位，用于没有编码器时的后续验证。无纹理、帧间隔过大、非刚体变化或误差过大时反馈无效；不把电调指令当作车速。
- `autonomy_motion.py`：米/秒目标与实测速度曲线的映射、PI 反馈、车辆四角与障碍物的局部路径检查、独立 250 ms 指令期限和全程时间上限。只使用 S3 1550–1750、S4 1500–1575 的已观察范围，当前配置禁止真实自主运动。原遥控测试的 0.8 秒上限未修改。
- `autonomy_audio.py`：真实 WAV 播放退出码/文件校验，或 0x40 TTS 整句单报文播报；使用 Linux 原始 I2C 写入，整句不再按 SMBus 的 32 字节块拆分，短播报报文设有 64 字节应用上限。TTS 必须观察到已测量的 busy→idle 才能确认完成，超时或未知状态不能成功。busy/idle 尚未在本车测得，默认不可用。2026-10-06 音量 8 经用户确认足够；模块速率和真实完成反馈仍待接入，不能把写入成功当作播报结束。
- `autonomy_feed.py`：明确选择两路相机、读取本机原子更新的反馈快照、输出观测 JSONL；默认只运行 10 秒、5 Hz，不操作底盘或云台，也不随开机启动。

## 电脑上的检查与模拟

从项目根目录执行，先安装 `vision` 的测试依赖或使用已有环境：

```powershell
$env:PYTHONPATH = (Resolve-Path vision/src).Path
python -m carvision autonomy-check --profile vision/configs/autonomy-pi5.json
python -m carvision autonomy-demo --profile vision/configs/autonomy-pi5.json --race-config vision/configs/race-2026.json --output run/your-new-demo-directory
python -m pytest vision/tests/test_autonomy_stack.py -q
```

输出目录必须是新目录。`summary.json` 分别记录 `decision_flow_completed`、`actuation_mapping_ready`、`hardware_motion_updates` 和 `physical_race_completed`，不能用模拟完成替代实车完成。本轮真实配置的正常结果是决策演示可完成、硬件准备不足、真实运动更新为零。

新的带米制运动映射的入口是 `autonomy-run`。第二台电脑的第三阶段也已审查合并：`pi-run --backend rasadapter5` 使用停车/拒绝运动的 bridge，适合独立决策回放，真实模式仍禁止；`pi-run` 默认的 `hardware-pwm` 则是旧 GPIO 路径。两种入口的数据格式不同，不要混用。`autonomy-run --run` 仅接受 Pi 本机实时 stdin、显式运动确认和完整实测配置；文件回放不能启用真实输出。没有校验通过时不打开 UART。

## 实测数据接入约定

`autonomy-observe` 读取本机的传感器包，不应连接公开页面提交的“已停止/已入位”标志。示意结构如下（数值必须来自真实采集，省略项为 unknown）：

```json
{
  "encoder": {"source": "encoder", "sequence": 1, "captured_monotonic_s": 123.4, "count": 1200},
  "pose": {"verified": true, "captured_monotonic_s": 123.4, "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "error_bound_m": 0.02},
  "primary": {
    "frame_id": 1, "captured_monotonic_s": 123.4,
    "image_size": [640, 480], "pose_us": null,
    "perception": {"detector_status": "ok", "lane": {}, "presence": {}, "detections": [], "traffic_light_state": "unknown"},
    "ground_features": {}
  }
}
```

主摄固定安装姿态用 `pose_us:null`；第二路需要真实的已固定云台位置。例如目前观察位置是 `[1650,1150]`，但这两个值本身不代表地面标定完成。移动云台之后必须选择对应姿态的已测量映射，否则拒绝使用其距离。

`autonomy-feed` 从 `--feedback` 指定的本地 JSON 文件读取编码器、姿态、云台位置和人工网关事件；采集程序应写临时文件再原子改名。文件没有、重复或过期时不填充假反馈。USB/串口编码器的实际型号未确认，因此不能预先编造其通信协议。可选择经过验证的地面光流方案，仍需先实测误差和定位原点。

相机生产程序不能与当前预览服务争用同一个视频设备。只有明确安排现场验证时才运行；本轮不启动它。现有 3 fps 预览不能充当满足 250 ms 期限的运动反馈。

实时运行的连接方式为 `autonomy-feed ... | autonomy-run --input - ...`，两者必须在同一台 Pi 上使用同一单调时钟。生产者默认 10 秒结束，EOF 立即停车并锁定。正式连续运行前必须完成真实执行链验证、反馈源验证和期限测量。

## 地面标定文件

`ground-calibrate --input measured-points.json --output new-camera-fit.json` 至少需要 4 个拟合点和 2 个独立检查点：

```json
{
  "image_size": [640,480], "pose_us": null,
  "valid_polygon_px": [[20,200],[620,200],[620,460],[20,460]],
  "points": [
    {"pixel":[100,250],"ground_m":[-0.3,1.0]},
    {"pixel":[540,250],"ground_m":[0.3,1.0]},
    {"pixel":[540,430],"ground_m":[0.3,0.4]},
    {"pixel":[100,430],"ground_m":[-0.3,0.4]}
  ],
  "check_points": [
    {"pixel":[320,300],"ground_m":[0,0.8]},
    {"pixel":[320,380],"ground_m":[0,0.5]}
  ]
}
```

以上只是格式示例，绝不能当作本车数值。坐标原点是后轴中心，x 向右、y 向前，单位米。只用于平地、对应分辨率与固定镜头姿态。镜头畸变较大时应先校正图像，再在同样的校正图像上标定。独立点误差、不同距离、阳光/阴影及运动时效果需要现场核对。

## 尚需现场完成

1. 测量车宽、轴距、轮距和轮胎大小；取得两种以上低速 PWM 的真实速度、制动距离与转向角。
2. 主摄和第二路分别测量地面映射；确定测速/定位来源，验证地面光流或实际编码器。光流弱纹理与累积漂移可能需要改用编码器/其他定位反馈。
3. 收集实际赛道、蓝挡板、灯、锥桶、停车框画面，训练/验证已有 YOLO 管线，记录验证模型哈希。当前实验颜色候选不能直接成为自由通道或挡板移除的驾驶依据。
4. 验证完整句子从车载扬声器播放，并测量该 TTS 的 busy/idle 字节，或校验实际 WAV。
5. 对接已认证 5G 遥控网关的启用、停止、换区事件；网关关于位置的判断必须来自真实定位。真实串口拥有者必须在换区停车后交接，不能同时打开 UART。
6. 实测准确停车与连续停稳 10 秒、红灯等待与绿灯放行、随机两锥桶绕行、随机占位的四轮停车及人工扫码支付；最后连续全程跑圈。

目前没有证据证明这些实车任务已经通过。框线定位、光流、锥桶跟踪和局部路径算法仍是需要场地验收的基线；可能需要根据现场画面调整算法，不能只勾选 verified 就认为已经完赛。

## 双电脑协作

本机新增的集成文件独立于第二台电脑已交付的有界执行器和 stop-only bridge。第三阶段运行层已离线审查、修正并合并，记录见 `../coordination/stage3-review-20261005-01`。完整包包含该代码；新增米制运动层使用更严格的版本化测量来源和独立实测配置。后续协作应对照协议、时钟期限和回执定义合并，不得覆盖实测配置或删除故障锁定。新模块的全部真实运行条件仍保持 false/unknown。
