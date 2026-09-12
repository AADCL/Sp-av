# 自动全局重定位与 RViz 初值验证

日期：2026-09-12。目标工作空间：`nrc@192.168.50.140:/home/nrc/catkin_ws`。

默认重定位入口使用新 `ducted_localization`：移植 AG-TEST 的 FPFH/RANSAC 全局初始化和点到面 ICP 跟踪。RViz 的 `/ducted/relocalization/initialpose` 保留，初值定义为 map/base_link，补偿现有倾斜雷达外参；有效手动请求中断自动搜索。原外参数值、局部 FAST-LIO、PX4 外部里程计和 TF owner 均未更改。

| 验证 | 结果 |
|---|---|
| 远程 catkin 构建 | 通过 |
| 坐标、质量、超时和会话单元测试 | 7 项通过 |
| 实际 Open3D 算法测试 | 未知初值搜索、手动初值均通过；合成场景 RMSE 约 0.0218 m |
| 隔离 ROS，主站 11328 | 14 项通过；合成机体中心位置误差约 0.00053 m；所有子进程已停止 |
| 真实雷达自动初始化 | AUTO / TRACKING / success=true，map_ready=true |
| 真实雷达持续跟踪采样 | 8 s 内 16 条状态全部有效 |
| 真实雷达手动初始化入口 | MANUAL / TRACKING / success=true，map_ready=true |
| 真实 TF | map → odom → camera_init → body → base_link 连通 |

实机使用 `maps/site_test_02/GlobalMap.pcd`，自动样本的 fitness=0.6356、RMSE=0.0950 m，机体中心地图坐标约 `(-0.289, 2.222, 0.410)` m。随后把当前完整机体位姿通过 RViz 同一话题送入手动初始化，fitness=0.6066、RMSE=0.0956 m，位置约 `(-0.277, 2.232, 0.413)` m。这里验证了配准质量和数据链路，没有独立测量真实机体坐标；RMSE 不等于绝对定位精度。

隔离 ROS 覆盖：未知位姿初始化、成功后持续跟踪、安装补偿、TF 连通、点云中断撤销定位、恢复数据不隐式重搜、已解锁拒绝初始化、显式重试、手动打断自动搜索、手动成功、错误 frame 拒绝，以及无运动指令发布者。

远程 Open3D 0.16.0 安装在工作空间 `.venv-localization`，Python 3.8 / aarch64；其他 ROS 节点继续使用原 Python 环境。本地算法交叉检查使用 Open3D 0.19.0。源码提供 `scripts/setup_python.sh` 以重建独立环境。

原始证据保存在远程：

- `logs/global_relocalization_build.log`
- `logs/global_registration_algorithm.log`
- `logs/global_relocalization_isolated/result.json`
- `logs/global_relocalization_live_result.json`
- `logs/global_relocalization_live.log`

真实 FCU 在验证前后均 connected=true、armed=false、mode=ALTCTL。静态验证关闭了基础层的外部里程计转发，没有请求模式切换、解锁、起飞、降落或发布飞行目标。
