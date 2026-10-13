# 辅助工具

实际相机、5G、串口、云台、台架、打包与显式部署工具位于 `vision/tools/`，具体实车入口依赖平台及准备条件。

本目录的 `audit_snapshot_20261008.py` 是离线审查复现脚本：在根目录安装 vision[test] 后运行 `python tools/audit_snapshot_20261008.py`，不访问摄像头、UART 或 GPIO。它展示已知问题，不是实车验收或自动修复。

日志默认保存在本地 run/；不要将密码、个人连接配置或大录像提交到公开仓库。

`verify_publication.py` 只核验公开文件哈希，不连接硬件。`serve_pwm_demo.py` 是本机模拟网页，默认端口 8931；其中 1715 μs 为独立模拟示例，不是当前实车中位。当前 Windows 双击入口在 `pc-launcher/`。
