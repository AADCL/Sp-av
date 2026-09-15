# 隔离 ROS 软件验证

```bash
python3 src/ducted_bringup/test/integration/verify_software.py
python3 src/ducted_bringup/test/integration/verify_software.py --integration
```

统一入口编译、执行保留模块与新 OFFBOARD 的单元测试、解析 96 种模块组合及冻结入口；`--integration` 追加地形、直接 OFFBOARD、分层线程故障三项隔离测试。EGO 生产入口及旧 EGO ROS 联动测试已冻结，算法离线单元测试仍可运行。

也可以分别执行：

```bash
python3 src/ducted_bringup/test/integration/terrain_isolated.py
python3 src/ducted_offboard/test/integration/offboard_isolated.py
catkin_make offboard_controller_test_node -j3 -l3
python3 src/ducted_offboard/test/integration/split_threads_isolated.py
python3 src/ducted_offboard/test/integration/frozen_launch_checks.py
```

主站分别使用 11322、11327、11329，端口占用时拒绝启动。最后一项只解析 launch，不运行节点。分层故障注入仅存在于未安装的测试二进制；生产节点没有注入接口。所有这些模拟均不发送真实飞控指令。

结果在工作空间 `logs/terrain_isolated/`、`logs/offboard_control_isolated/`、`logs/offboard_split_isolated/`。`tf_stages_hardware.py` 是需单独执行的真实传感器只读验证，不属于自动模拟测试。
