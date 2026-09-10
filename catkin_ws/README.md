# 涵道四旋翼 ROS 工作空间

本目录由远程 `/home/nrc/catkin_ws` 导出，包含七个源码包和现有地图。硬件参数保存在 `src/ducted_bringup/config/`，包括MID360网络、MAVROS、机体外参和共享几何。

完整介绍、构建和启动命令见 [仓库README](../README.md)。三份中文文档位于仓库顶层 [docs](../docs/README.md)。

本机驱动依赖来自 `/home/nrc/ws_livox` 和 `/home/nrc/mavros_catkin_ws`。首次构建前在已加载依赖环境的终端生成catkin顶层文件，再编译：

```bash
if [ ! -e src/CMakeLists.txt ]; then
  catkin_init_workspace src
fi
catkin_make -j3 -l3
source devel/setup.bash
```

先运行 `base_system.launch`，其他功能按需启动。建图与重定位二选一；地图需在建图进程运行时显式调用保存服务。
