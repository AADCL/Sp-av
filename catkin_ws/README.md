# 涵道四旋翼 ROS 工作空间

控制指令采用分步操作：`state=1` 只起飞并悬停，`state=2` 才开始／恢复任务，`state=3` 降落；悬停时收到新目标只加载，不自动执行。

本目录由远程 `/home/nrc/catkin_ws` 导出，包含模块源码和现有地图。硬件参数保存在 `src/ducted_bringup/config/`，包括 MID360 网络、MAVROS、机体外参和共享几何。

完整介绍、构建和启动命令见 [仓库 README](../README.md)。四份中文手册位于仓库顶层 [docs](../docs/README.md)，已部署主机中的副本位于 `/home/nrc/catkin_ws/docs`。

2026-09-15：EGO 源码保留但运行冻结。当前 C++ `ducted_offboard` 将任务状态机和飞控执行器拆为独立模块与 20 Hz 线程，保持一个节点，仅支持直接限速航点跟踪，**没有自动避障**。1 起飞后悬停，2 执行／恢复任务，3 使用 PX4 AUTO.LAND 降落。

本机驱动依赖来自 `/home/nrc/ws_livox` 和 `/home/nrc/mavros_catkin_ws`。首次构建前在已加载依赖环境的终端生成catkin顶层文件，再编译：

```bash
if [ ! -e src/CMakeLists.txt ]; then
  catkin_init_workspace src
fi
catkin_make -j3 -l3
source devel/setup.bash
```

先运行 `base_system.launch`，其他功能按需启动。基础层已包含外部里程计转发。建图与重定位二选一；地图在建图进程运行时通过 `./save_map.sh` 显式保存。

## 当前自动控制

使用 `roslaunch ducted_offboard offboard_control.launch`。任务状态机和执行器各自以 20 Hz 运行，单个服务线程调用 MAVROS，有界日志线程异步落盘。0 等待/暂停、1 起飞后悬停、2 执行/恢复、3 降落。默认直接目标推进水平 0.3 m/s、垂直 0.2 m/s、yaw 20°/s；AUTO.LAND 使用 PX4 参数。当前没有自动避障。

七步操作与状态继续条件见[自动飞行任务操作手册](../docs/涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md)。地图由用户通过 `map_file` 选择，手册示例为 `flight_test_01/GlobalMap.pcd`，并未绑定该地图；也可使用建图定位。远程 `nrc` 用户的新 Bash 终端已自动加载环境，首次构建或新部署仍需按安装说明配置。
