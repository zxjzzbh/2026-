# 集成与回归验证

当前实际测试位于 `vision/tests/`，包含感知、比赛状态机、模拟硬件、遥控和浏览器逻辑。根目录安装 `vision[test]`，配备 Node.js 后执行：

```sh
python -m pytest vision/tests -q -ra
```

历史测试数量随版本变化，见 [当前进度](../docs/current-status.md) 和 [开发记录](../docs/history/README.md)。Linux/Windows CI 的结果以 Actions 页面为准；安装、离线 demo 与历史快照校验的区别见 [复现说明](../docs/reproduce.md)。

测试通过不代表完整实车通过；新硬件结果使用 [实验模板](../docs/templates/experiment.md)，记录环境、版本、实际操作与未覆盖内容。
