# EGO 与 C++ OFFBOARD 联动验证（2026-09-15）

已在远程 `/home/nrc/catkin_ws`（Ubuntu 20.04、ROS Noetic、aarch64）完成编译和针对性软件验证。本次所有飞控反馈与控制服务均来自独立 ROS 主站中的模拟 MAVROS，未执行实机飞行，未上传 GitHub。

## 实现

- `ducted_bringup/ego_offboard.launch` 组合新 C++ 控制器、导航适配器和真实 EGO 节点。基础系统、建图、重定位独立启动。
- 文件、参数和单点／序列话题统一提交最终任务点；控制器保存最终目标，另行接收 EGO 前视点。直接模式保持为独立控制器的默认值。
- 新共享 `PlannerContext` 携带会话唯一编号、许可和有效起飞基准；状态、目标及异步导航响应必须匹配编号。暂停、降落、目标替换和故障撤销旧编号。
- 前视点与状态同时检查源时间和单调接收时间；定位失效、人工切出 OFFBOARD 时停止目标流，其他规划故障暂停悬停并等待明确恢复。
- 起飞基准每轮更新，高度范围约束机体中心相对起飞点的位置，不使用或伪造实测 `h_agl`。降落仍由 PX4 `AUTO.LAND` 执行。

## 与原实现相比的修正

1. 优化超过一个雷达周期时，用最新完整点云重新检查前视短线段；检查期间来新点云最多重试三次。没有发布的结果不推进限速历史。
2. 重新规划造成方向突变时按加速度上限平滑指令速度，并对实际发送线段再次检查碰撞和高度。
3. EGO 可使用上一条近期同终点曲线作优化初值，所有新结果仍重新执行碰撞和导数检查。距离终点小于 0.2 m、速度小于 0.2 m/s 时显式检查末段短线，避免极短样条拟合退化；任务到点容差不变。
4. 起飞基准仅表示中心高度参考，不当作观测地面增加机体半径；实时地面、墙体、人员等点仍参与包络碰撞检查。
5. FCU／外部里程计从有界历史中选择最新完整源时间配对；较新的匹配数据发生偏差时立即拒绝，不沿用更早的正常配对。总空间余量 0.10 m 分别覆盖 0.05 m 快照运动和 0.05 m 坐标偏差。
6. 点云先于配套里程计到达时等待匹配，不发布未经检查的新目标；持续无法配对仍会触发前视点超时暂停。

机体尺寸、安装外参及基础／建图／重定位入口未修改。

## 结果

| 项目 | 结果 |
|---|---|
| C++ OFFBOARD 单元测试 | 28 个实际 test case，0 失败、0 错误 |
| 导航单元测试 | 117 个实际 test case，0 失败、0 错误 |
| 真实 EGO／导航／OFFBOARD 整链 | 17 项通过 |
| 独立 OFFBOARD 回归 | 11 项通过 |
| EGO 核心回归 | 18 项通过 |
| 联动 launch 配置 | 文件／参数／话题 × 单点／序列 × 两种点云入口，共 12 种通过 |
| 原入口解析 | base_system、mapping、relocalization 三个入口通过 |

catkin 对 C++ gtest 的汇总显示 56 项；上表按 XML 中实际 test case 计数，避免重复统计。

整链执行了起飞、围绕竖直障碍的实际位置跟踪、两个任务点推进、完成悬停、暂停替换与恢复、点云中断、旧编号消息、导航服务阻塞及迟到响应、障碍无解、坐标偏差、原生降落、正常锁定、第二轮新高度基准和人工接管。第一任务点共发送 511 个有效前视点，最大接收源年龄约 0.139 s；模拟轨迹最大侧向绕行约 1.217 m。

独立控制回归另覆盖启动遗留命令、OFFBOARD 四次失败退出、解锁失败、服务阻塞时继续输出、定位断流和第二轮循环。模拟测试缩短了预发送、重试和到点停留时间以加快验证；整链速度上限仍使用水平 0.3 m/s、垂直 0.2 m/s 和加速度 0.5 m/s²。结果不代表真实机体的动力学或真实场景通过率。

## 复现

```bash
cd /home/nrc/catkin_ws
source devel/setup.bash
catkin_make offboard_controller_node ego_local_planner -j3 -l3
catkin_make run_tests_ducted_offboard run_tests_ducted_navigation -j3 -l3
catkin_test_results build/test_results/ducted_offboard
catkin_test_results build/test_results/ducted_navigation
python3 src/ducted_offboard/test/integration/ego_launch_checks.py
python3 src/ducted_bringup/test/integration/ego_core_isolated.py
python3 src/ducted_offboard/test/integration/offboard_isolated.py
python3 src/ducted_offboard/test/integration/ego_offboard_isolated.py
```

三个隔离主站端口分别为 11325、11327、11328；脚本检查端口并清理自己创建的进程。不要在同一个端口同时运行两份测试。

结构化结果见 [结果 JSON](2026-09-15-ego-offboard-results.json)。原始构建和测试日志保存在远程 `logs/ego_offboard_validation/`，整链原始状态与轨迹保存在 `logs/ego_offboard_isolated/`。变更前备份为 `logs/ego_offboard_validation/before.tar.gz`。

操作命令见 [自动飞行任务操作手册](../涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md)。
