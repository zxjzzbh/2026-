# 视觉软件 v0.1

**新增：电脑浏览器查看树莓派实时识别。** 使用 `carvision serve`，真实相机在树莓派采集与处理，电脑打开树莓派 IP 的网页即可查看，不需要桌面环境。步骤见 [实时网页预览](LIVE_PREVIEW.md)。旧交付包不含该入口，请更新源码。

已实现图片/录像回放、USB 采集入口、白线循迹基线、可选 YOLO 检测、红绿灯候选判断、时序确认、可视化和 JSONL 日志；另有抽帧、标注检查、训练、测速及导出命令。

**当前为感知开发版本，不发送 GPIO、PWM 或串口运动命令。没有比赛专用数据训练和实车验证，不能直接让车自动行驶。** 标定距离、蓝挡板发车事件、斑马线停稳计时、绕桶路径、停车位几何及整车任务状态机仍待实现与联调。

已确认主控架构为 **Raspberry Pi 4B + STM32 F4**：Pi 承担视觉/高层任务，F4 承担底层执行闭环。F4 具体型号、开发板和接口电平待确认；本视觉包不依赖某个 F4 引脚或串口协议。

外接摄像头已完成 Windows 实测：640×480、1280×720、1920×1080 均可出图，720p 关闭预览录制约 29.8 FPS。带预览时本次约 19.9 FPS，不能保证所有工况均为 30 FPS。详见 [相机测试记录](CAMERA_TEST_2026-09-23.md)，树莓派尚未测试。

路线：OpenCV 处理车道几何，轻量 YOLO 处理锥桶/蓝挡板/灯具/斑马线。首个训练和部署候选 **YOLOv8n**，不是最终性能选型。通用 COCO 权重只测速，比赛四类需实拍微调；Pi 4B 性能必须单独实测。

## 本机运行

在本目录打开 PowerShell。当前电脑 `.venv` 已配置，使用其中 Python，避免 PATH 中旧 Python 2.6。

```powershell
.\.venv\Scripts\python.exe -m carvision doctor
.\.venv\Scripts\python.exe -m carvision --help
```

已有演示结果在 `outputs/demo-run/`。重跑使用新目录，程序拒绝覆盖已有结果：

也可执行 `powershell -NoProfile -ExecutionPolicy Bypass -File .\run-demo.ps1`，自动选择新目录生成演示视频；该设置仅作用于本次 PowerShell 进程。

```powershell
.\.venv\Scripts\python.exe -m carvision demo --output outputs/demo2-input
.\.venv\Scripts\python.exe -m carvision replay --source outputs/demo2-input/synthetic-lane.avi --output outputs/demo2-run --save-video --show
```

绿色线是中心路径，红点是预瞄点，offset 正值表示目标在图像右侧。片段包含 20 帧双边界缺失，输出 invalid/null。这是程序绘制的测试素材，不是比赛录像或检测精度证据。

## 新电脑安装

建议 Python 3.12；核心视觉不需要显卡或 PyTorch：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[test]"
```

本台 Windows + RTX 4050 使用 CUDA 12.4 组合：

```powershell
.\.venv\Scripts\python.exe -m pip install torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu124
.\.venv\Scripts\python.exe -m pip install -r requirements-yolo.txt
.\.venv\Scripts\python.exe -m carvision doctor
```

**树莓派不能安装这组 CUDA 包**。待其系统/位数确认后采用 CPU/NCNN 方案实测。[PyTorch 官方版本说明](https://pytorch.org/get-started/previous-versions/)。

## 回放、采集与结果

```powershell
# 图片/视频，不启用 YOLO 时任务 presence 为 unknown
.\.venv\Scripts\python.exe -m carvision replay --source "D:/path/to/race.mp4" --output outputs/race-run1 --save-video
# 训练出四类模型后
.\.venv\Scripts\python.exe -m carvision replay --source "D:/path/to/race.mp4" --weights "D:/path/to/best.pt" --device 0 --output outputs/race-yolo1 --save-video
# 主动执行才打开相机；Q/Esc 结束
.\.venv\Scripts\python.exe -m carvision capture --camera 0 --seconds 30 --output ../data/recordings/day1-run1 --show
# 抽帧保留原分辨率，不自动生成标签
.\.venv\Scripts\python.exe -m carvision extract --source ../data/recordings/day1-run1/raw.avi --output ../data/processed/unlabeled --session day1-run1 --interval-s 1
```

回放生成 `results.jsonl`、`summary.json`、`config.json`、`preview.jpg` 和 `lane-mask.png`；加 `--save-video` 保存 `overlay.avi`。`--show` 需要图形桌面。实时感知输入可写 `--source camera:0`。

- 图片路径支持中文。损坏文件会报告错误，并保存运行错误摘要。
- 录像时间为帧序号/FPS，适用于恒定帧率素材；可变帧率不能据此测精确时长。缺失 FPS 需明确提供 `--fallback-fps`。
- OpenCV 不能可靠区分文件结束与解码错误，文件结束状态为 `eof_or_decode_failure`。
- 相机使用单槽最新帧，不重复旧帧；主机接收时间不等于曝光时间，驱动仍可能缓存。
- 采集 AVI 默认播放速率为 20，不是相机能力声明；`capture-info.json` 记录实际尺寸和保存帧率，`timestamps.csv` 保存主机时间。
- 当前 YOLO 与循迹串行，Pi 实测后再决定异步/分频调度。
- 灯态只针对初稿横向三灯布局，冲突/过曝返回 unknown；需要真实亮/暗灯录像验证。
- 白线为初始基线，阴影、相邻跑道线、斑马线可能造成误检；未标定，不输出米制距离。

字段见 [感知接口草案](../interfaces/vision-result-v0.1.md)，采集要求见 [你需要做什么](DATA_COLLECTION.md)。

## 训练与部署

四类顺序固定：`cone`、`blue_board`、`traffic_light`、`crosswalk`，见 [标注规范](ANNOTATION.md)。不能将 COCO 编号直接映射为比赛类别。

```text
data/processed/race-v1/
  images/train/day1-run1/*.jpg
  images/val/day2-run1/*.jpg
  labels/train/day1-run1/*.txt
  labels/val/day2-run1/*.txt
```

同一拍摄会话完整分在同一集合。确认无目标的图片也需空标签文件；没有标注不等于负样例。

```powershell
.\.venv\Scripts\python.exe -m carvision check-data --data configs/dataset.example.yaml
.\.venv\Scripts\python.exe -m carvision train --data configs/dataset.example.yaml --base ../models/weights/yolov8n.pt --device 0 --epochs 60 --batch 8 --imgsz 416 --output outputs/race-training-v1
.\.venv\Scripts\python.exe -m carvision benchmark --weights ../models/weights/yolov8n.pt --image outputs/demo-input/synthetic-example.jpg --device 0 --imgsz 416 --output outputs/bench-new
.\.venv\Scripts\python.exe -m pip install -r requirements-export.txt
.\.venv\Scripts\python.exe -m carvision export --weights outputs/race-training-v1/training/weights/best.pt --format onnx --imgsz 416
```

数据检查覆盖图片可读性、标签范围、类别缺失、跨集合相同图片和同名会话；不能识别所有被改名的相似场景，仍需人工检查。batch=8/imgsz=416 是试验起点；远处小锥桶需比较高输入尺寸，显存不足则减小 batch。

导出可能额外安装依赖，成功导出不等于 Pi 已验证。参考 [Ultralytics 推理](https://docs.ultralytics.com/modes/predict/)、[训练](https://docs.ultralytics.com/modes/train/)；依赖遵循各自许可证。

## 验证与目录

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

测试覆盖偏差方向、缺失边界、确认中断、禁用模型、暗灯/多灯冲突、中文路径、损坏视频、回放、禁止覆盖、标签越界和数据泄漏。见 [验证记录](VALIDATION.md)。

`src/carvision/`：sources 输入，lane 循迹，detection 模型与灯态，temporal 时序，pipeline 感知，dataset 数据检查，workflows 命令工作流。独立实现，未移植前辈完整程序。
