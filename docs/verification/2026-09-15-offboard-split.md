# EGO 运行冻结与 C++ 分层控制验证

日期：2026-09-15。远程工作空间 `/home/nrc/catkin_ws`，Ubuntu 20.04 / ROS Noetic / aarch64。本记录对应分层实现阶段的软件构建与隔离模拟，助手未执行实机飞行。后续仓库发布的核对范围见[发布记录](2026-09-15-github-update.md)。

## 交付行为

- 保留 EGO 源码、配置和离线单元测试；三个 launch、模块组合中的规划选项、C++ EGO 与 Python 导航生产入口均明确拒绝运行。控制器传入 ego 失败，不静默转为 direct。
- 单个 `ducted_offboard_controller` 节点，TaskMachine 与 FlightExecutor 分模块，各自 20 Hz；服务线程与有界日志写盘线程独立。
- 0 等待/暂停；1 接管、解锁、起飞后悬停；2 执行/恢复；3 原生 AUTO.LAND 降落。任务文件、参数、单点与序列输入保留。当前直接跟踪，没有自动避障。
- 执行器唯一发布 MAVROS 位置目标。任务层不使用 MAVROS 服务，不持有执行循环的长时间锁。内部快照带代次、请求 ID、单调时间，心跳超过 0.5 s 撤销推进，恢复线程后不自动续飞。
- 任务与执行状态、代次、心跳年龄进入状态话题及 CSV，规划字段固定 FROZEN/False；不发布 PlannerContext、不订阅前视点或规划状态。
- 更新四份手册和录包话题；七步主流程将 `flight_test_01/GlobalMap.pcd` 作为可替换的地图示例，也可改用建图定位。

## 构建和测试结果

| 验证 | 结果与范围 |
|---|---|
| 全工作空间 `catkin_make -j3 -l3` | 通过，生产节点、消息及保留功能包可编译 |
| 新任务/执行模块 C++ 单元测试 | 14 个 testcase 通过：分步指令、转换、目标替换、暂停、连续到点、第二轮参考、心跳撤销、旧服务请求、迟到解锁、断流及人工接管 |
| 保留 C++ 回归与任务加载器 | 31 个 testcase 通过，其中 27 个为冻结旧核心/桥接纯算法回归，4 个为共享加载器 |
| 导航离线单元测试 | 135 项通过；旧启动断言更新为冻结契约 |
| EGO 栅格纯 C++ 单元测试 | 5 个 testcase 通过 |
| 直接控制隔离 ROS 主站 11327 | 12 项通过：预发送、起飞后等待 2、多点/暂停恢复、降落锁定与第二轮、日志轮换、4 次模式/解锁失败、服务阻塞、定位断流、人工切出模式 |
| 分层线程与冻结入口隔离主站 11329 | 6 组通过，包括唯一发布者、无规划接口、任务线程停顿、恢复不续飞、显式恢复、降落独立完成及七类入口拒绝 |
| 停顿期间目标流 | 1.4 s 内 28 个目标点；最大间隔约 0.05012 s。任务心跳超时后捕获当前位置，不继续原任务 |
| 启动矩阵 | 96 种组合解析通过；规划选项仅包含冻结拒绝节点；非法定位选项拒绝 |
| 基础/建图/重定位入口 | 解析通过，保留原入口与节点；本轮未启动实机传感器进行额外验收 |

C++ 数量按 XML 中实际 testcase 计数，避免部分 catkin 工具将外层与内层 testsuite 重复相加。控制包合计 45 个实际 testcase，0 失败、0 错误。

## 记录位置与复现

- 修改前源码备份：`logs/offboard_split_20260915/before-remote.tar.gz`。
- 修改前 SHA-256：`logs/offboard_split_20260915/before-manifest.json`。
- 构建、测试、冻结入口和最终源码校验：`logs/offboard_split_20260915/`。
- 直接控制模拟：`logs/offboard_control_isolated/result.json`。
- 线程停顿与冻结模拟：`logs/offboard_split_isolated/result.json`。
- 静态启动矩阵：`logs/launch_matrix_result.json`。
- 本地备份：任务工作目录 `outputs/offboard-split/before-local.tar.gz` 及 `before-local-manifest.json`。

```bash
cd /home/nrc/catkin_ws
catkin_make -j3 -l3
catkin_make run_tests_ducted_offboard -j3 -l3
catkin_test_results build/test_results/ducted_offboard
python3 src/ducted_offboard/test/integration/offboard_isolated.py
catkin_make offboard_controller_test_node -j3 -l3
python3 src/ducted_offboard/test/integration/split_threads_isolated.py
python3 src/ducted_offboard/test/integration/frozen_launch_checks.py
python3 src/ducted_bringup/test/integration/launch_matrix.py
```

测试停顿接口仅存在于非安装的 `offboard_controller_test_node`，生产二进制不含此接口。模拟脚本使用独立主站、模拟 MAVROS，并清理自身进程；没有向真实飞控发命令。

运行中的旧控制器需在落地锁定后关闭并重新启动。机体尺寸和外参没有更改，地图、动态滤波、重定位和外部里程计代码保持原状。任务速度仍为期望位置变化率，不能解释为 PX4 原生降落速度。
