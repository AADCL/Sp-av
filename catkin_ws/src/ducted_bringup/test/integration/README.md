# 隔离 ROS 软件验证

在 `/home/nrc/catkin_ws` 编译并 `source devel/setup.bash` 后，按需顺序运行：

```bash
python3 src/ducted_bringup/test/integration/terrain_isolated.py
python3 src/ducted_bringup/test/integration/flight_isolated.py
python3 src/ducted_bringup/test/integration/ego_core_isolated.py
python3 src/ducted_bringup/test/integration/ego_mission_isolated.py
python3 src/ducted_bringup/test/integration/automatic_flight_isolated.py
```

分别使用独立本机主站11322、11323、11325、11326、11327，端口占用时拒绝启动。测试使用合成点云、模拟遥控器及模拟FCU，停止其创建的进程，JSON和控制台日志写入同名 `logs` 子目录。

EGO 核心验证无需 map TF 或占据地图，检查绕障、封堵、绝对高度、失效输入和局部范围。EGO/航点/飞控联调使用当前60×70×20 cm机体包络，覆盖到点、暂停恢复、障碍及数据中断。自动飞行联调从模拟地面状态开始，覆盖门控、起飞、航点、降落、取消及协调节点卡住后的授权失效。它们不是实际飞行记录。

```bash
python3 src/ducted_bringup/test/integration/verify_software.py
python3 src/ducted_bringup/test/integration/verify_software.py --integration
```

第一条编译、运行 catkin 单元测试并解析384种模块组合；第二条再运行上述隔离联调，失败即停止。默认 catkin 测试不自动启动这些主站。`tf_stages_hardware.py` 是另行调用的真实硬件只读验证，不属于自动飞行测试链。
