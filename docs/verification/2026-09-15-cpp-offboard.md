# C++ MAVROS 自动控制验证（2026-09-15）

## 交付范围

新增独立功能包 `catkin_ws/src/ducted_offboard`，使用 C++14、roscpp、MAVROS 和 yaml-cpp。它实现地面预发送、显式 OFFBOARD 接管、正常解锁、相对起飞、单点或序列跟踪、暂停恢复、PX4 `AUTO.LAND`、落地反馈确认和正常锁定。建图、重定位和基础系统入口未被替换。

本次没有连接真实 MAVROS 控制服务，没有执行实机切模式、解锁、起飞或降落，也没有上传 GitHub。

## 远程环境

- 主机：`nrc@192.168.50.140`
- 工作空间：`/home/nrc/catkin_ws`
- 架构：`aarch64`
- 系统：Ubuntu 20.04、ROS Noetic
- 飞控接口：MAVROS 1.20，PX4 1.12.3

## 编译与单元测试

执行：

```bash
cd /home/nrc/catkin_ws
catkin_make offboard_controller_node -j3 -l3
catkin_make run_tests_ducted_offboard -j3 -l3
catkin_test_results build/test_results/ducted_offboard
```

结果：控制节点在 ARM64 上编译成功；汇总为 36 项、0 错误、0 失败、0 跳过。核心用例覆盖相对坐标和 yaw、速率限制、遗留命令、地面反馈、当前位姿预发送、四次模式尝试、默认 1 米起飞、连续到点、暂停恢复、任务替换、AUTO.LAND、落地锁定、过期反馈及定位失效。

## 隔离 ROS 验证

测试脚本使用独立 ROS 主站 `http://127.0.0.1:11327` 和完整伪 MAVROS 话题/服务。结果文件为 [offboard_control_result.json](offboard_control_result.json)。11 项检查全部通过：

1. 启动时遗留的 `state=1` 不触发服务操作，同时持续预发送当前位姿。
2. 等待新鲜 OFFBOARD 反馈，再依据实际解锁反馈进入起飞，并自动进入 guiding。
3. 序列逐点、连续到点、暂停和恢复。
4. AUTO.LAND 模式反馈、ON_GROUND 反馈和正常锁定。
5. 不重启节点完成第二轮任务。
6. 独立会话事件、配置和 5 Hz CSV 日志，包含大小轮换。
7. OFFBOARD 首次请求加三次重试失败后，节点以非零状态退出。
8. 服务阻塞期间发布与状态循环继续运行，超时结果不会推进流程。
9. 四次解锁请求被拒绝后停止流程。
10. 位姿断流后停止旧目标输出。
11. 人工切出 OFFBOARD 后停止输出且不自动抢回。

## Launch 入口复核

通过 `roslaunch --nodes` 解析：

- `ducted_bringup/base_system.launch`：MAVROS、Livox、基础健康监控、外部里程计转换。
- `ducted_bringup/mapping.launch`：建图和定位 TF 节点。
- `ducted_bringup/relocalization.launch`：全局重定位、本地 LIO、TF 管理。
- `ducted_offboard/offboard_control.launch`：独立 C++ 控制节点，`required=true`、`respawn=false`。

隔离测试结束后确认端口 11327 的 ROS 主站和 `offboard_controller_node` 均已退出。

## 验证边界

软件验证证明状态机、接口、超时和日志行为符合设计，不代表真实飞行性能。首次实机验证仍需拆桨检查话题和服务，再由操作员在空旷场地保持遥控器接管能力，逐阶段确认 PX4 模式、EKF、本地坐标方向和降落行为。
