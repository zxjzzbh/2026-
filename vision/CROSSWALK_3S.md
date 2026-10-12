# 斑马线：循迹、停稳 3 秒、恢复循迹

更新：2026-10-09。当前是可运行的软件调试版本，所有新入口仅输出决策与日志，不连接 UART、不输出舵机/ESC 脉宽。

后续新增的车端停车短测入口见 [停车实测说明](CROSSWALK_PARKING_TEST.md)。本文下面的 replay/plan 仍为无硬件输出入口，两种用途分开。

## 运行入口

Windows 双击本目录的 `run-crosswalk.cmd`，输入真实图片/录像的完整路径；也可以明确输入 `camera:0` 选择已确认的摄像头。程序显示实际输入的检测叠加图和白色掩膜，不生成模拟网页。

如果换电脑且没有依赖，先在项目根目录建立独立 Python 3.12 环境并安装项目：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e "vision[test]"
```

Linux/Pi 已有环境可使用 `.venv/bin/python` 代替 Windows 路径。源码入口自行定位 `vision/src`，无需改系统 PYTHONPATH。

## 调自己的图片和录像

```powershell
.\.venv\Scripts\python.exe vision/tools/crosswalk_lab.py replay --source "D:\实际素材\斑马线.mp4" --output run/crosswalk-video-01 --save-video --show
```

`--source` 也接受 JPG/PNG。上面的素材路径为示例，请替换；不使用窗口时去掉 `--show`。窗口中按 Q 或 Esc 退出。为避免覆盖，输出目录必须是新目录。

可直接使用仓库已有的正样例检查入口：

```powershell
.\.venv\Scripts\python.exe vision/tools/crosswalk_lab.py replay --source vision/tests/fixtures/crosswalk-papers-secondary.jpg --output run/crosswalk-photo-01
```

看 `latest.jpg`、`crosswalk-mask.png` 和 `perception.jsonl`。单张照片只展示候选，无法证明跨帧持续存在。真实照片/录像没有实测速度，所以状态机显示等待反馈、不会凭“画面不动”开始倒计时，这是正常结果。

调参文件为 `configs/crosswalk-vision.json`：先检查 ROI 和白色掩膜，再调 `white_v_min`（亮度下限）、`white_s_max`（饱和度上限），最后看条纹组是否满足数量、排列及间距。至少 3 块完整条纹；屏幕底端被裁切的条纹不当作可靠完整条纹。`confirm_ms=200`、`max_gap_ms=250` 基于输入时间；低帧率录像可能无法满足连续确认，需要有依据地调整检测时序配置。

道路参数在 `configs/default.json`。调试器会排除已检测纸条的内部区域再提取白线，避免把条纹之间的间隙当作道路。参数效果应同时在正样例与无斑马线背景上检查。

若要查看现有标定范围内的距离候选，追加：

```text
--profile vision/configs/autonomy-pi5.json --camera-name secondary --camera-pose-us 1600 1150
```

仅在素材确实来自对应镜头、640×480 和该固定姿态时使用。该参数描述录制姿态，不转动云台。主摄用 `--camera-name primary` 且不传姿态。越出有效标定多边形时距离保持未知；`distance_estimate_m` 只是未经赛道验收的候选，不能当作实测停车依据。

## 观测回放：验证停车和恢复

```powershell
.\.venv\Scripts\python.exe vision/tools/crosswalk_lab.py plan --input "D:\实际日志\observations.jsonl" --output run/crosswalk-plan-01
```

上面的日志路径请替换为实际文件。每行使用 `carvision.race.Observation` 字段。示例一行（数值只是格式示例）：

```json
{"t_s":1.2,"source_ok":true,"telemetry_valid":true,"telemetry_age_s":0.02,"speed_mps":0.08,"lane_valid":true,"lane_offset":0.05,"task_monitor_valid":true,"crosswalk_state":"present","crosswalk_distance_m":0.6}
```

- `t_s`：同一时间轴、严格递增；离线使用记录时间，不使用程序处理速度计时。
- `lane_offset`：目标在右为正，归一化 -1..1，未知时 `lane_valid=false`。
- `crosswalk_state`：present/absent/unknown。`crosswalk_distance_m` 为车头到前缘距离，必须来自有效标定或经过验证的定位追踪；未知填 null。
- `speed_mps` 是实测速度，不能填目标速度或用 PWM 回读代替；停稳条件为绝对速度≤0.01 m/s。
- `task_monitor_valid` 是可靠任务观测，不应因为实验画面有框就写 true。回放接口信任数据提供方，不能据此连接未经验证的执行器。
- 每条新输入最多相隔 250 ms；反馈过期、画面异常、停稳期间又移动或距离丢失都会停止并重新计时。

流程是 `approach_crosswalk → crosswalk_stop → follow_after_crosswalk`。未发现斑马线且观测有效时循迹；发现后减速接近；到配置停车点发停车意图；确认在前缘前 30 cm 内停稳后连续计时 3 秒；下一条有效输入恢复循迹。本轮只处理一条斑马线，完成后不重复触发；重新测试需新建进程。

检测过目标后距离变成未知，会停车等待有效距离恢复，绝不因未检出恢复巡航速度。恢复循迹仍要求有效车道、来源和车速反馈。输入结束、格式错误或中断时，日志最后追加零速度意图。

## 可选摄像头观察入口

只在明确已释放该摄像头、不会与现有双摄服务争用时运行。例如电脑摄像头索引 0：

```powershell
.\.venv\Scripts\python.exe vision/tools/crosswalk_lab.py replay --source camera:0 --output run/crosswalk-camera-01 --seconds 30 --show
```

Pi 使用实际 `/dev/v4l/by-id/` 路径作为 `camera:` 后面的值，并去掉 `--show` 可无桌面运行。默认请求 640×480/MJPG，每秒最多解码 10 帧，模式支持由设备实测决定。该入口不启动、停止或重启现有车端服务。当前更建议先录制素材再离线调试。

## 当前基准和实车前缺项

- 以 10 月 9 日交接为上下文：Pi 5 + RasAdapter5A、双摄，无独立 STM32；手动中位记录为 1610 μs，输入期限 500 ms 与内部 250 ms 保护是不同层次。
- 完整 10 月 9 日部署源码尚未提供。本功能在可取得的 GitHub `9b5d51c` 基础上开发；交接中的 `serve_boot_test.py` 不在该基线里。本次新增模块不修改其网络/启动器/脉宽/保护配置。上下文元数据见 `configs/crosswalk-baseline-20261009.json`，它不是驱动配置。
- 用户指定本轮停 **3 秒**，新入口默认读取 `configs/crosswalk-3s.json`。旧整场 `race-2026.json` 仍保留历史 10 秒；要做 3 秒整场离线回放需显式传新配置，不能误用旧入口并期待自动变更。
- 本阶段 `announcement_required=false`，专注循迹和停车；保留播报事件/完成回执机制，设为 true 后还需有效 `announcement_done` 才放行。本轮未做实物播报验收。
- 现有双摄地面映射只验证车头前 50–150 cm，而停车目标为 25 cm。实际部署前仍需有效近场观测或经验证的位姿追踪、实测车速和制动数据；不能扩大标定多边形来伪造覆盖。
- 本阶段速度 0.15/0.08 m/s 是规划意图，不是已测出的 ESC 档位。软件入口不会把它换成脉宽，也不绕过现有自主准备检查。

## 文件与检查

本轮电脑验证：全量 723 项通过、3 项 Linux 专用检查跳过，其中新增斑马线测试 30 项。仓库已有实拍纸条正样例检出、背景负样例未检出；未连接摄像头或车辆，不能据此声明室外识别率或停车精度。

| 文件 | 用途 |
|---|---|
| `src/carvision/crosswalk.py` | 可调参条纹组检测、跨帧确认，与旧预览共用 |
| `src/carvision/race.py` | 共用比赛状态机：时长配置、已见目标锁存、停稳计时、单任务恢复 |
| `src/carvision/crosswalk_lab.py` | 真实图像/识别回放/观测回放、画面及日志输出 |
| `tools/crosswalk_lab.py` | 源码运行入口 |
| `configs/crosswalk-3s.json` | 本次停车及循迹参数 |
| `configs/crosswalk-vision.json` | 白色条纹检测与时序参数 |
| `tests/test_crosswalk_stage.py` | 3 秒逻辑、丢失、停稳证据、标定边界及入口验证 |

```powershell
.\.venv\Scripts\python.exe -m pytest vision/tests/test_crosswalk_stage.py vision/tests/test_race.py vision/tests/test_classic_detection.py -q
```

结果目录包含 `decisions.jsonl`、`observations.jsonl`、`perception.jsonl`、`summary.json`、`state-transitions.json`，有图像时另含叠加图与掩膜；`--save-video` 输出 MJPG AVI。图像接收/处理耗时不是完整 5G 显示延迟。软件测试与实车验收分别记录；本入口不生成网页。
