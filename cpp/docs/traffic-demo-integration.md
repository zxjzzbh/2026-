# 红绿灯识别 demo 的接入

来源为用户提供的 `红绿灯识别demo.zip` 中 `TrafficLight.cpp`、`TrafficLight.hpp`。保留 `ImageProcess::TrafficLightDetector`、动作枚举以及红色双 HSV 区间、圆度/面积/宽高比过滤的思路；未执行附件中的程序。

## 接口

```cpp
ImageProcess::TrafficLightDetector detector;
detector.reset();
auto action = detector.update(bgr_frame, capture_frame_id, capture_time_seconds);
observation.red = action == ImageProcess::TrafficLightAction::RED_STOP ||
                  detector.getState() == ImageProcess::TrafficLightState::WAIT_GREEN;
observation.green = action == ImageProcess::TrafficLightAction::GREEN_GO ||
                    detector.getState() == ImageProcess::TrafficLightState::GREEN_CONFIRMED;
```

时间必须来自单调时钟，帧号必须对应实际采集的新帧。重复/乱序帧不增加票数；空帧、无效时间或超过 0.5 秒的采集间隔会打断连续确认，但不会解除已确认红灯的保持状态。`update(frame)` 仅保留给独立 demo 兼容，真实控制应使用带帧号/时间的接口。模块不输出 PWM，不执行制动，不访问硬件，由主控制器负责制动和停稳。

绿灯确认期间发生空帧、无效格式、时间回退或采集断链，会撤销待使用的绿灯确认并回到 `WAIT_GREEN`，保留原灯箱关联。恢复后的无灯图像不能放行；必须再次连续取得同灯组 3 个新绿帧。

已确认绿灯后出现单帧红候选，同样只撤销放行、回到 `WAIT_GREEN`，不退回 `IDLE`。这样红色反光消失后，同灯组的 3 帧新绿灯仍可恢复；若连续确认了新红灯，则发出一次新的 `RED_STOP`，普通等绿期间的重复红灯不会反复产生事件。

控制层给每次制动分配非零、严格递增的 `brake_event_id`。执行器在 `submit()` 时保存事件，而不是等 20 ms 输出线程取到最新指令才发现制动；因此紧接着的视觉中位请求不会覆盖它。首次实际执行后独立维持配置的制动时长，再回中位并发布完成事件反馈。同一事件不会因超时、图像恢复或反复提交而重复启动。显式人工停止调用 `close()` 可取消尚未完成及排队事件；车辆是否物理停稳仍需另外判断。

## 修改

- 红灯默认连续 2 个新帧，绿灯默认连续 3 个新帧。可以加严确认帧数，不能将其配置成单帧放行。
- 红色和绿色均须通过亮色轮廓的圆度、面积、宽高比筛选，并位于暗色长条灯箱的端部。灯箱长短轴决定横置/竖置；两个方向均支持。
- 绿灯须与已确认红灯的灯箱位置、尺寸、方向相符，灯孔尺寸相近，且位于灯箱另一端。大面积绿地面、同一位置变绿的反光、另一个灯箱的绿灯均不放行。
- 每帧先检查红色；红绿同时出现时保持停车，绿灯连续票归零。绿灯确认后保持 `GREEN_CONFIRMED`（旧 `COOLDOWN` 数值的兼容别名），保证制动脉冲期间到达的绿灯确认不会被主控遗漏。没有倒计时忽略红灯；下一次合格红候选立即撤销绿灯确认，红仍需连续 2 个新帧确认停车。同一输出不会同时为红和绿。
- 删除 `forceWaitGreen()`，不能把挡板或时间点伪造为已识别红灯；删除绿后 5 秒忽略红灯和基于绿像素总数的放行。没有“等够几秒自动放行”。
- 重叠 ROI 取并集；调试直接读取 `getLastDebugInfo()`，不得再次调用检测来画图。诊断包含当前候选、连续票数、灯箱和拒绝原因。

## 验证边界

`tests/test_traffic.cpp` 使用合成 OpenCV 图像验证横/竖灯、红绿优先级、绿地面、异灯箱、重复/空帧、采集间隔和复位行为。它不等于真实光照和车速测试。若曝光让暗色灯箱消失、灯孔太小、红灯组移动超过关联容差，检测可能不能确认或继续保持等待；应依据实际录像调整曝光、ROI 和轮廓阈值，不能降低为任意绿色像素即放行。停车距离仍由主控制器和实测制动距离决定，不能把 2 帧确认宣称为“必然停在灯前”。
