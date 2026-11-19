# 当前版本的离线验证

安装 CMake 3.20 或以上、C++17 编译器、OpenCV 4 开发库，以及 Python 3.10 或以上的开发库。构建时选择的 Python 需要能导入 `cv2` 和 `numpy`。Ubuntu 上可使用 `libopencv-dev python3-dev python3-opencv python3-numpy` 和系统解释器 `/usr/bin/python3`。
在仓库根目录执行：

```sh
python3 tools/verify_release.py
cmake -S cpp -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 2
ctest --test-dir build -C Release --output-on-failure
```

十组 CTest 覆盖控制、识别、组合回放和 5G 网页服务，其中包含 32 项 Python 服务测试，使用图像、模拟观测和假执行器。
GitHub CI 使用 Linux。本版车端状态与上一版部署回执见[部署记录](../DEPLOYMENT.json)。
不得重跑历史 `vision/tools` 或桌面更新器；部署须保留实际配置、日志、录像与驱动，并完成单版本完整校验。
