# 涵道四旋翼 ROS 工作空间

本目录由远程 `/home/nrc/catkin_ws` 导出，包含模块源码和现有地图。硬件参数保存在 `src/ducted_bringup/config/`，包括 MID360 网络、MAVROS、机体外参和共享几何。

完整介绍、构建和启动命令见 [仓库 README](../README.md)。四份中文手册位于仓库顶层 [docs](../docs/README.md)，已部署主机中的副本位于 `/home/nrc/catkin_ws/docs`。

当前三米自动任务采用 PX4 起飞点加 1 m、前飞 3 m、悬停 5 s，然后请求原生 `AUTO.LAND`。新模式不要求有效 `h_agl`；任务准备后仍需人工解锁并明确执行 `--start`。降落速度采用飞控当前参数，自定义 `slow_land` 保持禁用。完整步骤见[自动飞行任务操作手册](../docs/涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md)。

本机驱动依赖来自 `/home/nrc/ws_livox` 和 `/home/nrc/mavros_catkin_ws`。首次构建前在已加载依赖环境的终端生成catkin顶层文件，再编译：

```bash
if [ ! -e src/CMakeLists.txt ]; then
  catkin_init_workspace src
fi
catkin_make -j3 -l3
source devel/setup.bash
```

先运行 `base_system.launch`，其他功能按需启动。基础层已包含外部里程计转发。建图与重定位二选一；地图在建图进程运行时通过 `./save_map.sh` 显式保存。
