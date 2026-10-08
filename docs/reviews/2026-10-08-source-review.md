# 2026-10-08 当前源码审查

范围：队伍提供的 `智能车当前源码_20261008/race2026`，按原包 SHA256 清单核验 184 项，全部一致。审查为电脑离线代码阅读、已有测试和定向复现，没有连接车辆、读取现时硬件状态或执行运动。桌面原包保留。

## 结论与证据等级

项目已经从离线视觉发展到**树莓派上的双摄感知、网页遥控、串口执行和自主任务软件框架**。当前主控是 **Raspberry Pi 5**，经 `/dev/ttyAMA0` 驱动 **Hiwonder RasAdapter5A V1.0**；S1/S2 为云台，S3 为转向，S4 为 ESC。没有队伍自行开发的独立 STM32 下位机。拓展板内部如何实现 PWM，不应被混写成项目采用了“Pi + STM32”双控制器架构。

已有功能要分三层：

1. **本次重新验证**：Windows / Python 3.12 / OpenCV 4.11.0.86 / NumPy 2.2.6，原测试集 **692 passed, 3 skipped**。跳过项为 Linux Unix socket / termios；Node.js 前端检查包含在已通过测试中。自主 demo 处理 221 条合成观测，决策到达 complete，但计划运动 0、硬件运动 0，实际配置仍有 18 个准备缺项。
2. **源码包自带的历史记录**：双摄出图；双路固定姿态静态地面映射；Pi 5 低速短测与纯 5G 短测。部分详细原始录像/日志没有随包提供，不能称为本次复测。最新重复短测的历史软件检查是 158 passed / 2 skipped。
3. **尚未验收**：完整自主赛道、连续地面驾驶、可靠速度/定位反馈、右转机械复测、比赛检测器、真实播报结束反馈、自动/手动控制权交接。

静态映射误差约 2.1 cm 是在特定固定姿态和车头前 50-150 cm 区域的点位检查结果，不是整车停车精度，也不是所有像素都具有该精度。

## 实际技术栈和代码位置

| 层 | 技术 | 实现与当前状态 |
|---|---|---|
| 应用与打包 | Python ≥3.10、setuptools、argparse、dataclass、JSON/JSONL | `vision/pyproject.toml`；当前审查用 Python 3.12 |
| 基础视觉 | OpenCV 4.11.0.86、NumPy 2.2.6 | HSV 白线分割、多行扫描、二次多项式拟合、颜色/形状候选、多帧确认 |
| 元素识别 | 传统 OpenCV + 可选 Ultralytics 8.3.228 | `classic_detection.py`、`cone_shapes.py`、`traffic_lamps.py`；YOLO 有训练/推理入口，没有已验收比赛权重 |
| 模型导出 | 可选 ONNX 1.17.0 / ONNX Runtime 1.20.1 | 可选依赖与导出流程，不代表当前实车正在使用 ONNX |
| 摄像头 | USB UVC、Linux V4L2、MJPG、采集线程、最新帧缓存 | 两路 by-id 选择；原始预览与识别解耦；不是双目深度方案 |
| 网页遥控 | Python `ThreadingHTTPServer`、HTTP/1.1、MJPEG、原生 HTML/CSS/JavaScript | WASD、方向键、空格、心跳、轮次隔离；Gamepad API 有模拟检查，实物手柄未验收 |
| 5G 通信 | RM500U-CNV / NCM / Tailscale（历史记录） | 本包是 HTTP/MJPEG 遥控，不能称为已实现 WebRTC；蜂窝路径由系统网络提供 |
| 比赛决策 | 显式有限状态机、JSONL、单调时钟 | `race.py`、`autonomy_runtime.py`；遥控、挡板、斑马线、红绿灯、两锥桶、停车、人工付款 |
| 几何与定位 | 单应矩阵、地面区域约束、LK 光流、RANSAC | `autonomy_geometry.py`、`ground_odometry.py`；光流目前是可选实验分支 |
| 运动输出 | UART 1,000,000 baud、CRC8、扩展板 PWM 指令 | `rasadapter5.py`、`pi5_pwm.py`、`autonomy_motion.py`；PWM 回读是保存指令，不是舵角/轮速传感器 |
| 播报 | WAV/aplay 或 I2C 0x40 TTS | 完成反馈必须实测；当前 busy/idle 未确定 |
| 部署与测试 | systemd、pytest、Node.js、SHA256 清单 | 默认预览暂停运动；旧 pigpio C/DMA、RP1 PWM 路径仍在包中 |

ROS、STM32 固件、实际编码器协议并非当前已使用技术。这里的版本来自源码依赖声明，不能据此断言部署机器所有包的实际版本。

## 优先修复与改进

### P1：斑马线移出近距离视野后会回到巡航速度

位置：[`race.py`](../../vision/src/carvision/race.py) 的 `CROSSWALK_APPROACH`、[`autonomy_profile.py`](../../vision/src/carvision/autonomy_profile.py) 的 `GroundProjection.point`，以及实际相机配置。

两路已标定地面区域的纵向范围均为后轴前 **0.84-1.84 m**，车头至后轴为 **0.34 m**，对应车头前 **0.50-1.50 m**。停车目标在车头前 0.25 m，即后轴前 0.59 m，映射到两路图像约为 y=584 和 y=564，超出 480 高度及标定区域。不能简单把有效多边形扩大来伪造近场标定。

离线复现：先提供 0.51 m 斑马线距离，再提供距离 `None`，其他图像、车道、反馈均保持有效。状态机仍输出 `follow_lane`，速度由接近档恢复为 **0.25 m/s**。没有保留“已见到斑马线、接近过程中丢失”的锁存状态。当前真实运动被配置禁用，但此逻辑在后续补齐标定后仍需修复。

建议：区分从未发现/正在接近后丢失；接近后禁止因失去距离而加速，先输出停或有证据支持的受限策略。用经过验证的定位持续追踪已观测地面线，或在比赛允许的固定机位范围内取得真正覆盖停车区的观测。用可见距离、速度、处理时延、制动距离共同验收，而非只调整阈值。

### P1：当前停车观察无法证明四轮入位

位置：[`autonomy_geometry.py`](../../vision/src/carvision/autonomy_geometry.py) 的 `_parking`、[`autonomy_observation.py`](../../vision/src/carvision/autonomy_observation.py) 的 `inside_wheels`。

停车框必须四角都落在当前有效投影区；但是两路有效区域最近都在后轴前 0.84 m，而轮胎位于后轴附近 y=0 和前轴 y=0.245 m。即使拿整个已标定区域作为“车位”，计算也只得到 **0 个车轮在内**。当车辆实际进入车位，近边退出相机视野，当前代码没有持续维护车位的世界坐标。

另外，空闲区只在内部平均 HSV 饱和度 `<45` 时判 clear。红/蓝塑胶跑道很可能始终 unknown；源码自己已标明这一限制。这不是把阈值改大就能证明空车位。

建议：在完整车位可见时定位、建立稳定 ID 和世界坐标，再借助已验证位姿推进车轮包络；对位姿误差设限制，失效时退回 unknown。单独验证红/蓝跑道上的黄白边线与挡板占位。加入“车位近边逐渐出画”的序列验收，不只测完整静态框。

### P1：自主控制需要的实测闭环链路仍未接通

位置：[`autonomy-pi5.json`](../../vision/configs/autonomy-pi5.json)、[`autonomy_profile.py`](../../vision/src/carvision/autonomy_profile.py)、[`autonomy_feed.py`](../../vision/src/carvision/autonomy_feed.py)。

实际 readiness 检查返回 **18 个缺项**，包括速度曲线、制动减速度、转向角、速度反馈与位姿、检测器与道路覆盖、交通停车线、播报完成。`telemetry.source=encoder` 只是配置选择；缺少计数来源、每计数距离和真实采集程序。WIT IMU 也没有形成已验证状态估计链路。

手动暂定中位是 **1715 us**，自主配置仍为 **1650 us**；最新 review 又明确 `right_turn_mechanical_retest_pending=true`。不能把较早“转向正常”概括为最新右转问题已解决，也不能直接把 1715 覆盖成最终标定。

建议：建立一份当前 Pi 5 标定档，明确“暂定观察值/已测值/正式可用值”；先解决右转和直行保持，测定低速档、制动和转角，再接入实际反馈。保持现有未就绪门控，不通过修改 true/false 绕过缺测。

### P1（验证缺口）：250 ms 软件期限不等于硬件失联停机保证

位置：[`rasadapter5.py`](../../vision/src/carvision/rasadapter5.py)、[`pi5_pwm.py`](../../vision/src/carvision/pi5_pwm.py)、[`autonomy_motion.py`](../../vision/src/carvision/autonomy_motion.py)。

手动路径有独立进程，自主路径有同进程看门狗线程；它们都依赖 Pi 继续运行、能够把零油门写入 UART。扩展板协议中的运动时间字段不能当作“到期自动撤销 ESC 指令”的保证；包内记录也明确 `hardware_finite_wave=false`。如果整台 Pi 卡死/掉电或 UART 断开，现有软件测试不能证明仍在供电的扩展板和 ESC 会自行停车。

建议：核查厂商实际提供的失联/输出回退能力，在固定架空条件下分别验证主程序退出、输出进程失效、UART 中断、Pi 复位等情况，并量测停车时间。若现有硬件无法满足要求，再评估在纯 Pi 架构下可用的看门狗/动力断开方案；不以增加 STM32 为默认改进。

厂家串口与 PWM 使用依据：[Hiwonder 扩展板教程](https://wiki.hiwonder.com/projects/Raspberry-Pi-5-Expansion-Board/en/latest/docs/2_Expansion_Board_Control_Lesson.html)。该页面并未构成本车失联停机验收证据。

### P2：光流点移出标定区域会造成对应数组长度不一致

位置：[`ground_odometry.py`](../../vision/src/carvision/ground_odometry.py)，投影 `before.append(...)` 与 `after.append(...)` 的循环。

旧点投影成功后就追加到 before，新点投影失败则异常被吞掉，导致只追加一侧。用 21 个跟踪点、其中 1 个新点越出标定区域复现，`estimateAffinePartial2D` 抛出点数不一致的 OpenCV 断言，而不是输出无效反馈。启用该可选方案后可能让采集线程退出；默认 encoder 配置尚未触发它。

建议：先把同一对点都投影成功，再成对追加；对成功点对数量做检查。补上跨出视野、光流缺失和零/异常尺度的回归测试。另外，世界位姿误差目前复用固定 `visual_error_bound_m`，需增加累积漂移/行程预算和重定位证据，不能用每帧拟合残差替代全程定位误差。

### P2：传统视觉和自主执行的接口尚未形成可验收闭环

`ClassicRaceDetector.status=experimental_color_shape`；自主观测只接受 `detector_status=ok` 且该相机 `detector_verified=true`。`autonomy-feed` 未传模型时检测器为 None，而非自动使用传统检测器；配置还要求模型哈希。这个拒绝机制避免实验候选误授权，但意味着网页上的彩色检测框并不能直接驱动自主比赛。

建议：明确下一阶段选择“经过场地验证的传统检测器”还是“比赛专用 YOLO”；为选定实现建立版本、配置哈希和按场景划分的验收数据，不是简单把 status 改为 ok。当前只有通用 COCO 模型的来源记录，不能称为比赛识别模型训练完成。

### P2：网页的随机控制键不能代替操作者身份认证

`web_preview.py` 在 GET 首页时向所有可访问者提供同一个运行期控制键；它提供来源约束与防止旧页面误操作的帮助，不是登录授权系统。Tailscale 网络访问策略是当前外层隔离，但本包没有包含其实际 ACL 验证证据。

建议：保持可信网络范围，确认仅授权操作者能访问控制端口；后续将只读视频与控制权限分开。压测控制消息 P95/P99、丢包与视频负载，不以 30 次 60-80 ms 历史 RTT 推断所有 5G 情况都能满足 200 ms 期限。不要单纯调大超时来掩盖排队。

### P2：多代路径与追加式文档导致入口、参数和进度漂移

旧 Pi 4 DMA、Pi 5 RP1 PWM、当前 RasAdapter UART 共存；代码集中在 `vision/src/carvision`，而仓库目录规划曾把状态机放 vehicle、输出放 firmware。README 中又混有“5G 已通过”和早期“未插卡”等历史段落。

建议：先建立当前状态页与历史页的分离（本次同步已完成文档部分）。后续把采集、感知、比赛决策、手动控制、硬件适配和页面资源按稳定接口拆分；一次只迁移一个职责并保持测试通过。让配置明确选择硬件后端；旧代码标为历史/维护状态，不因文件名含 Pi5 就默认接线匹配。网页长字符串可以迁入静态资源，测试应继续覆盖真实浏览器事件语义。

### P2：拒绝 HTTP 请求的 Windows 检查出现一次间歇失败

原包全量 692/3 通过；同步目录第一次全量有 1 项失败（691 通过/3 跳过）：拒绝 chunked 控制请求时，客户端在读取 HTTP 400 前收到 WinError 10053。对应网页测试集连续两次各 18 项通过，全量重跑恢复 692/3。没有修改或屏蔽该测试。应进一步区分 Windows 未读完请求体即关闭连接的行为与服务端响应时序，当前不能把所有网络异常路径宣称为无波动。

## 下一轮按什么顺序做

| 优先级 | 交付 | 完成标准 |
|---|---|---|
| 1 | 当前硬件/标定单一来源，右转与直行保持复测 | 不同入口读取同一正式参数；观测值、测量值区分清楚 |
| 2 | 修正近场斑马线和入位估计设计 | 50 cm 到停车点全过程可追踪；丢失不加速；停车框出画后仍有可信车轮位置 |
| 3 | 接通速度、定位、制动与播报回执 | 输入均有真实来源、时间和误差；ready 缺项逐一由证据关闭 |
| 4 | 固定一条感知部署路线 | 阳光/阴影/红蓝地面/小锥桶验收，保存失败片段和独立测试集 |
| 5 | 控制权交接与链路故障验收 | 远程停车后独占 UART 交接，串口/进程/网络异常均有量测记录 |
| 6 | 分任务与完整自主验收 | 首先低速单任务，再串联，最后按规则完整执行；软件测试和实车结果分别登记 |

## 如何复现本次检查

在仓库根目录创建 Python 3.12 环境并安装 Node.js 后：

```sh
python -m pip install -e "vision[test]"
python -m pytest vision/tests -q -ra
python -m carvision autonomy-check --profile vision/configs/autonomy-pi5.json
python -m carvision autonomy-demo --profile vision/configs/autonomy-pi5.json --race-config vision/configs/race-2026.json --output run/new-offline-review
python tools/audit_snapshot_20261008.py
```

审查复现脚本使用当前配置、合成观测和模拟光流输入，不打开摄像头/UART/GPIO。它用于展示已知缺口，退出成功不代表缺口已修复。结果摘要见 [verification-20261008.json](verification-20261008.json)。

## 本次 GitHub 同步改了什么

- 导入源包的程序、依赖、测试夹具、接口、驱动来源/许可证和少量自带核验 JSON；保留远端团队成员信息及旧分支。
- 当前架构改为 Pi 5 + RasAdapter UART；STM32 目录标记为不采用的历史规划。
- 新增当前状态、审查报告、离线复现脚本和 Windows/Linux CI。
- 保留源包原 README 为根目录 `SOURCE_HISTORY_20261008.md`，避免把历史追加记录当成最新状态；源包原始清单保留，发布映射清单另行记录。
- 对历史文档中的个人连接地址进行占位替换；部署辅助脚本改为显式 `--host`、可选已可信 `--host-key-alias`，继续拒绝未知主机密钥。未运行部署。
- **没有修改车辆的决策、运动输出、保护期限或实测标定数据**；以上缺口保持明确待修复，不将一次代码同步包装为实车问题已经解决。
