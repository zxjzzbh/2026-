# 历史源码包与说明归档

这些文档用于追溯旧版本。原文中的接线、参数、部署状态和“当前”均有日期范围；最新入口是 [项目概览](../overview.md) 与 [当前进度](../current-status.md)。

## 2026-10-08 源码包中的说明

- [源码历史](2026-10-08/SOURCE_HISTORY_20261008.md)：原源码包 README。
- [源码包说明](2026-10-08/源码包说明.md)：10 月 8 日交付说明。
- [双摄与云台验证](2026-10-08/CAMERA_GIMBAL_VERIFICATION.md)：原包包含的 10 月 2 日验证记录。
- [部署报告](2026-10-08/DEPLOYMENT_REPORT.md)：原包包含的 10 月 2 日部署记录。

目录日期指源码包版本，不把 10 月 2 日的实测日期改成 10 月 8 日。

## 2026-10-09 标定说明

- [当时的标定状态](2026-10-09/calibration-status.md)：保留 1550–1750 调试范围与旧车辆尺寸；现行参数用途见 [标定来源索引](../calibration-status.md)。

## 清单与还原

根目录 [SOURCE-MANIFEST.json](../../SOURCE-MANIFEST.json)、[source-snapshot.json](../../source-snapshot.json) 和 [reviews/](../reviews/) 内的发布清单按原样保留。它们的路径和哈希描述当时的快照，不覆盖之后的新代码或本次文档迁移。

本轮归档仅添加历史提示、调整相对链接及注明未打包证据；没有用新内容改写旧哈希。还原具体版本应使用 [分支说明](../branches.md) 中的提交/标签；10 月 9 日 `verify_publication.py` 仅在对应历史标签上使用。

全部迁移路径见 [目录整理表](../repository-structure.md#本轮文件整理)。
