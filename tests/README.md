# 集成与回归验证

当前实际测试位于 `vision/tests/`，包含感知、比赛状态机、模拟硬件、遥控和浏览器逻辑。根目录安装 `vision[test]`，配备 Node.js 后执行：

```sh
python -m pytest vision/tests -q -ra
```

2026-10-08 Windows 原包测试 692 通过/3 跳过；同步树复测同样通过，中间一次 HTTP 连接测试失败已保留记录。Linux/Windows CI 的结果以 Actions 页面为准。离线 demo 与定向复现见 [审查报告](../docs/reviews/2026-10-08-source-review.md)。

测试通过不代表完整实车通过；新硬件结果使用 [实验模板](../docs/templates/experiment.md)，记录环境、版本、实际操作与未覆盖内容。
