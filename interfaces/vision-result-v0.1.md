# 感知结果 v0.1 草案

维护模块 vision。用于回放和后续联调，未与 vehicle/firmware 冻结，不是 UART 协议。每帧一行 JSON，类型见 `vision/src/carvision/results.py`。

| 字段 | 语义 |
|---|---|
| schema_version/mode | `0.1` / `perception_only`，不表示运动授权 |
| frame_id | 源帧序号，实时可能跳号，不复用旧帧 |
| source_time_ms | 录像名义时间（帧号/FPS），单图/相机为 null |
| receive_monotonic_ms | 本机 monotonic 时间，不是曝光时间，不能跨设备直接相减 |
| image_size | 原图 `[width,height]` 像素 |
| source_status | 当前解码帧为 ok；源失败终止运行并写 summary，不伪造后续帧 |
| processing_ms | 感知计算时间，不含显示/保存 |
| receive_to_result_ms | 主机接收到输出的耗时，不含传感器/驱动内部延迟 |
| lane.valid/reason | 满足初始双边界几何约束，不等于保证可通行 |
| lane.quality | 启发式分数 0-1，非概率 |
| lane.left/right/center | 原图坐标列表，每点 `[x,y]` |
| lane.target | 原图预瞄点；无效 null |
| lane.offset_normalized | `(target_x-width/2)/(width/2)`，正值目标在右，无效 null |
| detector_status | disabled 未加载；ok 成功推理；失败抛错，不返回空检测冒充成功 |
| detections | `label,score,bbox_xyxy`，原图坐标，非米制 |
| presence | 四类分别 present/absent/unknown，默认持续 200 ms 确认、间隔超过 250 ms 重置 |
| traffic_light_state | red/yellow/green/unknown；目前是待实拍验证的候选灯态 |

`absent` 只表示连续未检出，不能解释为挡板已移开、车位空闲或允许通行。单图没有时长证据，即使存在检测框，presence 仍 unknown。类别存在性确认不是目标 ID 跟踪。

实时下游必须有独立接收超时；程序崩溃或源失败不会继续发心跳。失效动作由主控/下位机约定。本版本不接执行机构。

坐标原点左上，x 向右、y 向下。尚未标定距离、航向或车轮区域，不提供虚构占位；不能把图像偏差直接当舵机角度。
