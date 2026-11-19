# 当前版本的离线验证

安装 CMake、C++17 编译器、OpenCV 4 开发库和 Python 3.10 或更新版本。
在仓库根目录执行：

```sh
cmake -S cpp -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 2
ctest --test-dir build -C Release --output-on-failure
```

九组 CTest 包含 C++ 识别/控制回归和 28 项 Python 5G 检查，使用图像、模拟观测和假执行器，不发车。
GitHub CI 使用 Linux；树莓派部署检查与本地 Windows 检查见 [部署回执](../DEPLOYMENT.json)。
不得重跑历史 `vision/tools` 或桌面更新器；部署须保留实际配置、日志、录像与驱动，并完成单版本完整校验。
