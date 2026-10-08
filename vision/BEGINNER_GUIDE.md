# 从打开软件到提交第一批录像

要在电脑上看树莓派摄像头的实时识别，请使用新增的 [浏览器实时预览指南](LIVE_PREVIEW.md)。下面是原有本机采集操作，两种入口按实际设备选择。

适用：这台已配置环境的 Windows 电脑。主控架构为树莓派 4B + STM32 F4；当前先在电脑进行视觉开发。外接相机已完成 Windows 出图测试，当前 MSMF 编号 1；重插后需重新确认。详见 [相机测试记录](CAMERA_TEST_2026-09-23.md)，树莓派上的采集参数仍待测试。

今天的完成标准：软件环境能检查、外接摄像头能出图、保存一段可播放的原始录像。今天不要求训练模型或连接车辆执行机构。

## 1. 认识两个目录

- 团队仓库：`D:\5G远程驾驶无人车学习资料\2026-team-repo`
- 视觉软件：`D:\5G远程驾驶无人车学习资料\2026-team-repo\vision`

在资源管理器地址栏粘贴路径即可进入。`vision/src/carvision` 是源码，`.venv` 是已经安装的 Python 环境，`outputs` 是运行结果。原始录像放仓库的 `data/recordings`，不是放进源码目录。

## 2. 打开 PowerShell

按 Windows 键，搜索并打开 Windows PowerShell，普通权限即可。复制以下整行并回车：

```powershell
Set-Location -LiteralPath "D:\5G远程驾驶无人车学习资料\2026-team-repo\vision"
```

之后所有命令都从这个目录执行。如果关闭窗口重新打开，要再次执行上面的命令。每次复制一条命令，等待结束后再执行下一条。

## 3. 检查环境

```powershell
.\.venv\Scripts\python.exe -m carvision doctor
```

正常会显示 Python 3.12、OpenCV、Ultralytics、torch，以及 `cuda_available: true` 和 RTX 4050。输出的是环境信息，不是错误。没有相机也可以完成此步。

本机不需要重复安装或激活环境。命令中的 `.\.venv\Scripts\python.exe` 要完整保留，不要替换为 `python`：这台电脑 PATH 中还存在旧 Python 2.6。

## 4. 先看已有软件演示

打开之前生成的演示图片：

```powershell
Invoke-Item ".\outputs\demo-run\preview.jpg"
```

打开演示录像：

```powershell
Invoke-Item ".\outputs\demo-run\overlay.avi"
```

左右彩线为道路边界，绿色线为中心路径，红点为目标点。显示 `YOLO: disabled` 正常：这个演示只运行循迹，尚未加载比赛专用模型。

演示图是程序绘制的，不是实际识别能力证明。如播放器不能打开 AVI，先看图片即可；软件也能通过 `replay --show` 显示画面。

想自行重跑一次可执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\run-demo.ps1
```

脚本会自动生成新的结果目录并打印路径，不覆盖旧结果；此命令仅设置本次 PowerShell 进程的脚本执行策略。

## 5. 接相机，先录十秒测试

将 USB 摄像头接到电脑，关闭可能占用相机的软件。先运行：

```powershell
.\.venv\Scripts\python.exe -m carvision capture --camera 0 --seconds 10 --output ../data/recordings/camera-test-01 --show
```

程序会创建目录并打开画面，十秒后结束；也可点击画面窗口后按 Q 或 Esc 提前结束。

**确认显示的是外接摄像头。** 笔记本编号 0 可能是自带摄像头。可将物体放在外接摄像头前观察；如果画面不是外接相机，结束本次，再尝试：

```powershell
.\.venv\Scripts\python.exe -m carvision capture --camera 1 --seconds 10 --output ../data/recordings/camera-test-02 --show
```

编号不保证连续，也不保证 1 一定正确。若 0、1 都失败，把错误文字和系统中显示的相机名称告诉我，再有针对性排查。每次重试使用新输出目录，例如 `camera-test-03`；失败尝试也可能已创建文件夹。

本阶段不添加 width/height/fps 参数，先确认默认采集能工作，不把想要的参数当作硬件已经支持。

## 6. 确认文件保存成功

```powershell
Invoke-Item "..\data\recordings"
```

每次录制目录应有：

| 文件 | 用途 |
|---|---|
| raw.avi | 未叠加识别框的原始画面录像 |
| first-frame.jpg | 首帧，方便检查视角和清晰度 |
| capture-info.json | 完成/错误状态、真实尺寸和保存帧率等信息 |
| timestamps.csv | 每帧在电脑上接收的时间 |

双击 `first-frame.jpg`，检查有没有模糊、过曝、画面倒置。再播放 `raw.avi`，确认不是全黑或一直停在一帧。有文件名不等于采集已经成功，需同时看画面和 `capture-info.json` 的状态。

AVI 默认播放速率为 20；实际相机帧率可能不同，录像播放快慢不能直接代表车辆速度。时间信息另有保存，后续由我核对。

## 7. 录第一批真实素材

相机测试通过后，先录 3-5 段，每段 20-40 秒。先用已有的道具，不必等所有道具买齐。

| 建议目录名 | 内容 |
|---|---|
| day1-straight-01 | 跑道白线直道，居中、稍偏左、稍偏右 |
| day1-curve-01 | 弯道，缓慢改变朝向，尽量保持安装视角 |
| day1-shadow-01 | 同一场景有阳光和阴影的画面 |
| day1-cone-01 | 已有红/蓝小锥桶，远近、左中右 |
| day1-board-01 | 蓝挡板存在、移开和空场景的完整过程 |

以下示例假设你已确认外接相机编号为 1；若实际为 0，将 `--camera 1` 改为 `--camera 0`：

```powershell
.\.venv\Scripts\python.exe -m carvision capture --camera 1 --seconds 30 --output ../data/recordings/day1-straight-01 --show
```

下一段把目录名换成 `day1-curve-01` 等即可。正式训练素材尽量用比赛允许的固定安装位置；单纯手持测试也可提供，但请注明，不能与车载视角的验收结果混淆。初稿规定摄像头不得任意加高降低。

保留原始文件，不叠字幕、不加滤镜、不经聊天软件压缩。当前是采集阶段，不需要运行自动驾驶；拍摄时车辆不必由软件驱动。

## 8. 告诉我保存位置

你可以回复：

> 摄像头编号是 1。录像已放在 `D:\5G远程驾驶无人车学习资料\2026-team-repo\data\recordings`。有直道、弯道、蓝挡板三段；今天晴天，相机已装车/尚未装车。画面正常/有某个问题。

同一台电脑上的文件不需要重复上传。我会读取录像、评估可用性、抽帧，并安排标注与模型训练。目标标签仍需要核对，不能把预标注直接当作真值。

## 9. 可选：自己尝试录像循迹回放

先完成采集，再执行下面的命令；源目录名需与实际录制目录一致：

```powershell
.\.venv\Scripts\python.exe -m carvision replay --source ../data/recordings/day1-straight-01/raw.avi --output outputs/my-first-replay-01 --save-video --show
```

这会运行未经过实拍调优的白线基线，保存画面和日志。没有稳定画出线也请保留结果；我们正需要失败样例来判断原因。未提供训练权重时 YOLO 保持 disabled，不会自动识别锥桶等比赛目标。

## 10. 常见提示

| 提示/现象 | 怎么做 |
|---|---|
| 找不到 `.venv\Scripts\python.exe` | 先重新执行第 2 步的 Set-Location；不要改用系统 python |
| `FileExistsError` | 输出目录已存在，换一个新名字；不必删除原始数据 |
| `cannot open camera` | 检查连接、相机是否被占用、编号是否正确；若系统禁止桌面应用访问相机，检查相机隐私权限 |
| 显示笔记本自带摄像头 | 结束后换编号，并用不同输出目录重试 |
| `camera read failed` / `timeout` | 保留错误信息，检查 USB 连接和相机占用，告诉我发生在启动还是录制中 |
| `YOLO: disabled` | 没有加载比赛模型时的正常状态 |
| `insufficient_boundary_pairs` | 本帧没找到足够双边界，保留画面，不自行把它当作正常居中 |
| 只有 JSON 文字，没有画面 | 查看命令是否带 `--show`；doctor 本来只输出文字 |

不要先执行 train：当前缺少真实图片和核对后的标签。等第一批素材检查后，再确定输入尺寸、数据划分和训练配置。
