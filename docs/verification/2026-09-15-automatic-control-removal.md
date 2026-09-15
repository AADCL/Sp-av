# 旧自动飞行控制代码移除记录

2026-09-15，按用户要求先备份，再从 `/home/nrc/catkin_ws` 移除旧自动控制代码。新自动流程尚未确定，本次没有新增飞行控制实现。

## 已移除

- `ducted_mission`：旧自动任务、航点执行及其配置、测试。
- `ducted_control` 中的飞行控制状态机、位置目标发送、旧降落实现和降落配置工具；该包保留 RC 数据读取与监视。
- `automatic_flight.launch`、`automatic_sequence.launch`、`flight_control.launch`、`waypoint_mission.launch`。
- 工作空间根目录的 `prepare_forward_test.py`、`launch_forward_test.sh`、`configure_native_landing.py`，以及 `tools/px4` 下的旧降落验证工具。
- 对应 `build`、`devel` 中的旧程序入口、任务消息生成物及过时测试结果。组合启动、构建清单、记录话题和验证入口已同步调整。

## 保留

基础通信、MID360、MAVROS、RC 读取、FAST-LIO 建图、动态滤波、地图保存、自动全局重定位、RViz 手动初值、持续地图跟踪、外部里程计转换、TF 和地面高度功能保留。

EGO 局部规划与导航接口保留；控制器就绪门控仍在，当前没有飞行执行器。共享消息类型不等于执行程序。PX4 参数未修改。

## 备份

远程完整删除前备份：

```text
/home/nrc/legacy-auto-backup-20260915T020745Z/
  automatic-control-before-removal.tar.gz
  manifest.json
```

备份覆盖 350 个待删除或修改文件，删除前已逐文件验证 SHA-256。清理涉及 60 个文件或目录入口、20 个更新位置（含手册副本）；清理时核对的 265 个保留源码文件及地图保存脚本均未改变。原备份清单保留删除前版本，本地也保存了副本。

重新编译后，额外清理 20 个旧 CMake 测试目录及安装脚本残留，共 84 个文件；已先保存并校验到同一备份目录的 `generated-remnants.tar.gz` 和对应清单。

## 验证

| 检查 | 结果 |
| --- | --- |
| 远程 `catkin_make -j3 -l3` | 通过 |
| `ducted_bringup` 现有测试 | 133 项通过 |
| RC 读取与监视测试 | 20 项通过 |
| 模块组合解析 | 96 种通过；非法定位选项被拒绝 |
| 基础、建图、重定位 launch 节点解析 | 通过，无旧自动飞行节点 |
| 保留文件校验 | 清理前后一致 |

验证中同步修正了旧 `TerrainHeight` 消息契约测试遗漏 `reason` 字段的问题，仅更新测试预期，未修改地面高度消息或算法。

本次执行了编译、测试和启动文件解析，没有启动实机 ROS 节点或执行飞行。远程证据目录：`/home/nrc/catkin_ws/logs/legacy_auto_removal_20260915/`。
