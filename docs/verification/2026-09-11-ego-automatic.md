# EGO 局部规划与自动飞行软件验证

验证日期：2026-09-11。执行主机：`nrc@192.168.50.140`，工作空间：`/home/nrc/catkin_ws`。全部飞行联调使用独立 ROS master 和模拟 FCU；没有在真实飞机上执行解锁、模式切换或飞行位置指令。

## 变更

- 移除全局 A*、Fast-Planner/NLopt 后端及旧 `global_local_planning.launch`。新 `ego_planner.launch` 使用官方 EGO 的回弹 B 样条和 L-BFGS；内部 A* 仅为局部碰撞段提供方向。无需占据地图文件或 map TF 即可进行 odom 局部规划。
- 保留实时动态障碍、机体包络、h_agl、绝对高度和原位置控制接口。EGO 的完整种子和轨迹经过碰撞、时效、感知范围及导数控制点约束检查；短距离位置指令再独立检查。
- 增加 `automatic_flight.launch`：明确启动后依次执行预发送、OFFBOARD、起飞、航点和配置的结束动作。默认悬停；`finish:=land` 才自动请求降落，并检查落地和未解锁反馈。解锁仍由操作员执行。
- 增加取消、异常服务闭锁、起飞通道与高度衔接检查。起飞默认上升1.1 m，绝对目标为当前 odom Z 加上升量；起飞目标必须满足导航 AGL 包络。按扫描时间配对位姿历史，避免用到达时刻不同的最新消息错误判断起飞通道。
- 自动组合通过0.5秒心跳约束飞控与任务。协调进程卡住后，消费者独立停止输出／暂停任务；恢复后不自动续飞。
- 本次未更改雷达安装外参、机体尺寸、定位数据链、分阶段 TF 或建图动态滤波。

上游版本：`ZJU-FAST-Lab/ego-planner@bfda51284c8c1b476043255a8145ef925a3778a5`，GPL-3.0。复制范围和逐项修改见 [源码说明](../../catkin_ws/src/ducted_planning/vendor/ego/PROVENANCE.md)。

## 结果

| 检查 | 结果 | 远程记录 |
|---|---|---|
| catkin 编译 | 通过 | `logs/ego_final_build.log` |
| bringup / control / navigation / mission 回归 | 341项，0错误、0失败 | `logs/ego_final_tests.log`、`build/test_results` |
| 模块组合解析 | 384种通过 | `logs/launch_matrix_result.json` |
| EGO 核心 | 16项通过 | `logs/ego_core_isolated/result.json` |
| EGO、位置控制与航点整链 | 31项通过 | `logs/ego_mission_isolated/result.json` |
| 自动飞行整链 | 28项通过 | `logs/automatic_flight_isolated/result.json` |

EGO 核心覆盖直线、绕柱曲线、封堵、绝对高度、当前机体碰撞、空／截断点云、非有限／越界坐标、连续导数界和局部目标截断。导航整链额外让模拟飞机实际跟随曲线绕柱到点，再验证两航点、暂停恢复、动态障碍及数据中断。

自动整链从模拟地面状态开始，检查默认关闭、导航几何门控未打开时禁止起飞、未解锁拒绝、起飞通道障碍、起飞后航点、降落反馈、完成不重启、预发送期间取消，以及用 SIGSTOP 暂停模拟主站中的协调进程后，飞控和任务独立检测授权超时。测试结束清理其创建的进程。

## 使用范围

这些结果证明代码和模拟接口链路通过上述检查，不代表实机飞行记录。EGO 点云模式将局部感知范围内未命中的体素视为未占据，不证明遮挡区域安全；局部范围之外不可通行，也不提供全局绕行保证。B 样条经过限速后作为位置参考，PX4 不接收本模块的速度／加速度前馈。

默认输出门控继续关闭。操作命令、参数和状态见三份交付文档。
