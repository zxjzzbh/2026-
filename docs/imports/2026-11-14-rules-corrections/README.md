# 2026-11-14 规则审核修复候选

版本：`rules-corrections-20261114`。作者：13246。

基于 `b723950ed8ee568a1fb32e8b959e9770ef40ce62`，修复代码提交为 [`f19493006ee7b7837026a95a9b127490959bfa08`](https://github.com/zxjzzbh/2026-/commit/f19493006ee7b7837026a95a9b127490959bfa08)。后续文档提交沿用同一套运行代码。

- [PR #11：逐项规则修复](https://github.com/zxjzzbh/2026-/pull/11)
- [处理结论、测试与现场待办](../../reviews/2026-11-14-rules-corrections.md)
- [204 个运行源码与网页文件的 SHA-256 清单](source-manifest.json)
- [固定修复代码 ZIP](https://github.com/zxjzzbh/2026-/archive/f19493006ee7b7837026a95a9b127490959bfa08.zip)

文件清单以 Git 中的文件内容计算 SHA-256，另列 LF 换行规范化后的值。核对范围是 `vision/src` 与 `vision/web` 的完整跟踪文件集；测试、文档和部署工具随 Git 版本交付，不计入 204 个运行文件。

本次没有部署、连接或操作车辆。最近一次已核验的车端部署仍为 `race-rules-offline-20261113`。公开模板仍缺 13 项停车实测条件；正式入口在条件齐备前拒绝启动，不把回放通过记作实车验收。

历史版本、固定标签与旧清单保持原样；当前修复的获取入口与历史 r5 包分别列在 [最新代码说明](../../latest-code.md)。
