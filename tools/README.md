# 辅助工具

实际相机、5G、串口、云台、台架、打包与显式部署工具位于 `vision/tools/`，具体实车入口依赖平台及准备条件。

本目录的 `audit_snapshot_20261008.py` 是离线审查复现脚本：在根目录安装 vision[test] 后运行 `python tools/audit_snapshot_20261008.py`，不访问摄像头、UART 或 GPIO。它展示已知问题，不是实车验收或自动修复。

日志默认保存在本地 run/；不要将密码、个人连接配置或大录像提交到公开仓库。
