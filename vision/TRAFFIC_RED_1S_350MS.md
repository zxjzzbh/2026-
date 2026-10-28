# 红灯 1 秒投票、350 ms 制动更新

日期：2026-10-25。
基于用户提供的 `智能车_树莓派当前完整代码_20261025_103056_350ms.zip`，
不基于之前的 65 cm 定位停车更新包。
原包 SHA256：`8bc099829f79671a7dd444836c00ec44ecb75b8a44a02beac39073ec01b7ea10`。

## 检查结论

原包的共享 F/B 输出已允许较长脉冲，斑马线当前方案已使用 0.35 s；
但红绿灯 `traffic-driving.json` 的 `brake_pulse_s` 仍为 0.12，
`traffic_driving.validate_settings()` 也把它固定在 0.12，因而红绿灯并未完整改为 350 ms。
完整流程输出层虽然可能允许 0.35 s，红绿灯控制器仍会按较短请求提前回中位。

另外，原现场覆盖文件 `traffic-driving.local.json` 把比例设成 35%，不是一半。
本次将该值改为 50，保留单项油门 1570 us 和接近最长时间 10 s。
完整比赛流程沿用自身原有的油门配置。

## 修改后的行为

- 不依赖位置或测距；继续按原配置接近，任一路摄像头确认红灯就制动。
- 红灯统计最近完整 **1 秒**，每路单独计票，红灯帧数 **大于等于 50%** 时确认。
  沿用旧比较方式，恰好一半也达标；没有改成严格大于一半。
- unknown、off、绿灯、黄灯都计入该路窗口的总帧数，不借用另一摄像头的票。
  另一摄像头 unknown 不否决有效一路的确认。重复、过期帧不增加票数。
- 新开始或相机恢复后要先收集完整 1 秒窗口；已经收集好窗口时，
  不要求红灯连续存在 1 秒。例如稳定 10 fps 下，最近 10 帧中有 5 帧红灯即可确认。
- 红灯确认后请求 **1400 us、0.35 s** 的有界 F/B 制动；仍未停稳时沿用原最多 3 次脉冲，
  释放间隔和运动反馈逻辑不变。重复制动帧不能延长同一次脉冲。
- 指令租约仍为单项 0.25 s、完整流程 0.2 s；没有为 350 ms 制动而放宽失联期限，
  制动期间仍需正常刷新控制命令。
- **只缩短红灯窗口。绿灯保持原 2 秒窗口**，停车时清空旧票，确认停稳后重新收集。
  红绿确认冲突时红灯优先，黄灯也阻止绿灯放行。
- 独立红绿灯测试确认绿灯后只播报并保持中位；完整比赛流程切换下一阶段续行。
- 不修改红绿灯视觉分类器、灯具跟踪、相机标定、蓝挡板、绕桶或泊车算法。

0.35 s 是制动指令时长，不等于停车距离或实车停车位置验收。

## 修改文件

运行代码/配置共 7 个文件：

1. `vision/configs/traffic-driving.json`：0.35 s 脉冲、红灯 1 s、绿灯 2 s。
2. `vision/configs/traffic-driving.local.json`：现场比例 35 改为 50，其他现场参数保留。
3. `vision/src/carvision/traffic_driving.py`：校验、分开红绿窗口、计时诊断。
4. `vision/src/carvision/traffic_voting.py`：独立的红灯/绿灯窗口队列及各自统计值。
5. `vision/src/carvision/traffic_trial.py`：状态和日志中显示真实时长。
6. `vision/src/carvision/traffic_controls.py`：页面显示红 1 s / 绿 2 s / 350 ms，
   绿灯票数使用其自己的 2 秒总帧数，不再误用红灯 1 秒总帧数。
7. `vision/src/carvision/race_flow_console.py`：共享制动上限覆盖红绿灯的 0.35 s，避免截断。

测试：更新 `test_traffic_driving.py`、`test_traffic_trial.py`；新增 `test_traffic_red_1s.py`。
包内 `validation.json` 记录实际验证结果。未连接、部署或启动实车。

## 另一台电脑如何合并

本包是针对上述完整包的增量更新；不是一个独立安装程序。
建议先在对方仓库的 `smartcar-race-20261002` 项目根目录检查补丁：

```sh
git apply --check /path/to/changes.patch
git apply /path/to/changes.patch
```

也可对照包内 `smartcar-race-20261002/` 中的同路径文件合并。
若对方已有新改动或现场参数不同，不要强制覆盖；尤其要保留对方真实油门和标定。
只把 `.local.json` 的红灯比例更新成 50 即可，不要用默认 JSON 覆盖现场油门。
若此前应用了旧的 65 cm 定位停车包，应明确将相关控制文件合并到本版信号停车逻辑，
不能只改一个窗口参数后继续运行旧的测距启动条件。

共享输出依赖原包已有的 `brake_output.py`、`cone_trial_output.py`、`brake_timing.py`，
本次不再改写它们；包内 manifest 记录其原始哈希。
本包不自动部署、不重启服务，也不更改或绕过软件校验清单。
部署时文件清单必须覆盖上述 7 个运行文件；旧的单项部署脚本可能不包含完整流程文件。

## 检查命令

```sh
PYTHONPATH=vision/src python -m pytest vision/tests/test_traffic_red_1s.py vision/tests/test_traffic_driving.py vision/tests/test_traffic_trial.py vision/tests/test_traffic_voting.py vision/tests/test_race_flow.py vision/tests/test_brake_timing.py vision/tests/test_brake_cleanup.py -q
```

Windows PowerShell 先设置 `$env:PYTHONPATH='vision/src'`，再执行 `python -m pytest`。
缺少 PC 部署辅助依赖时可用项目既有的 `CARVISION_SKIP_PC_DEPLOY_TESTS=1` 跳过该可选检查。
测试使用假输出和合成相机观测；另外验证控制器与输出层在 0.349 s 仍制动、0.35 s 回中位，
以及单路有效、缓存帧、红绿冲突、不同帧率和现场参数优先级。

本机验证结果：相关测试 176 passed / 1 skipped。原包全量为 2007 passed / 145 failed / 6 skipped，
修改版为 2026 passed / 145 failed / 6 skipped；失败用例集合完全一致，没有新增失败。
已有失败分布在绕桶、挡板、停车边界恢复等模块，详细分组见 `validation.json`。
因此本包只报告红绿灯及关联制动检查通过，不把原包称为全量测试通过。
