# Sp-av

涵道四旋翼无人机的 ROS Noetic 开发工作空间。

## 启动分层

- `base_system.launch` 是所有功能的共同前置，后续功能必须在它就绪后启动。
- 建图、重定位、对地高度、飞行控制、避障、任务、相机和记录均保持独立 launch，按需组合。
- 指控平台适配不在当前范围内。

## 高度语义

- 定位、建图与任务状态使用 `map` 坐标系中的绝对高度。
- 避障使用离地高度（AGL），由绝对高度与地面估计共同计算。
- `ducted_msgs/TerrainHeight` 是地面高度与离地高度的公共接口。

## 当前阶段

第 1 步只建立工作空间、公共消息和基础启动接口。`base_system.launch` 中的 MAVROS 与 Livox 开关默认关闭；硬件接入、健康检查和 ready 门控在第 2 步完成。

首次检出后初始化并验证：

```bash
cd /home/nrc/catkin_ws
catkin_init_workspace src
catkin_make
catkin_make run_tests
catkin_test_results
source devel/setup.bash
rosmsg show ducted_msgs/TerrainHeight
roslaunch --files ducted_bringup base_system.launch
```
