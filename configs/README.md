# 共享配置

只存跨模块配置模板，例如未来的整车任务配置 `vehicle.example.json`。模块专用阈值放模块自己的 `configs/`。

当前运行配置集中在 `vision/configs/`：`default.json` 为感知、`race-2026.json` 为比赛、`autonomy-pi5.json` 为 Pi5 实测与准备状态。此顶层目录暂不复制第二份配置。配置中的未完成项仍是 false/null。

个人实车参数可保存为 `vehicle.local.json` 等本地文件；`*.local.json/yaml/yml/toml/ini` 已被忽略。共享可公开的默认参数和校准示例应使用不含 `.local` 的文件名，并注明适用硬件。

真实密码和 token 不写入模板；模板用空值或清楚的占位符。复制模板不代表程序已经支持它，需要配套加载代码。
