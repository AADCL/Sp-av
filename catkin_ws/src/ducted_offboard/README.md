# ducted_offboard

控制指令采用分步操作：`state=1` 只起飞并悬停，`state=2` 才开始／恢复任务，`state=3` 降落；悬停时收到新目标只加载，不自动执行。

`ducted_offboard` 是涵道四旋翼的 C++ MAVROS 位置控制功能包。节点接收任务点，在得到明确命令后完成 OFFBOARD 接管、解锁、相对起飞、位置及航向跟踪、暂停、PX4 原生降落和落地锁定。

节点启动本身不会切换模式或解锁。`/ctrl_cmd/state` 必须先出现一次 `0`，之后新的数值变化才会被当作命令。飞行前必须拆桨完成软件验证，并在空旷场地由能立即接管遥控器的操作员执行实机测试。

## 依赖与编译

系统环境为 Ubuntu 20.04、ROS Noetic、MAVROS 1.20 和 C++14。安装构建依赖：

```bash
sudo apt update
sudo apt install ros-noetic-mavros ros-noetic-mavros-msgs libyaml-cpp-dev
```

在工作空间编译：

```bash
cd /home/nrc/catkin_ws
catkin_make -j3 -l3
source devel/setup.bash
```

## 启动顺序

以下三条分别放在三个终端运行并保持前两个终端运行。先启动基础系统和能稳定提供 `odom` 位姿的建图或重定位模块，再启动本节点：

```bash
roslaunch ducted_bringup base_system.launch
roslaunch ducted_bringup mapping.launch
roslaunch ducted_offboard offboard_control.launch
```

使用已有地图时，把第二条换成以下命令（`MAP_FILE` 可改为当前场地地图）：

```bash
MAP_FILE="/home/nrc/catkin_ws/maps/flight_test_01/GlobalMap.pcd"
roslaunch ducted_bringup relocalization.launch map_file:="$MAP_FILE"
```

完整的七步操作及每一步继续条件见[任务操作手册](../../../docs/涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md)。建图与重定位二选一，已有地图不是控制器固定条件。控制节点订阅：

- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/local_position/velocity_local`
- `/mavros/extended_state`

控制节点持续向 `/mavros/setpoint_position/local` 发布 `geometry_msgs/PoseStamped`，通过 `/mavros/set_mode` 请求 `OFFBOARD` 或 `AUTO.LAND`，通过 `/mavros/cmd/arming` 请求正常解锁或锁定。坐标由 MAVROS 转换，节点不再执行 ENU/NED 转换。

检查状态：

```bash
rostopic echo /ctrl_cmd/status
rostopic hz /mavros/setpoint_position/local
```

状态话题包含实际阶段、原因、任务点进度、模式、解锁反馈、输入是否新鲜、当前目标和本次日志目录。

## 命令

命令参数只由操作员或上层任务系统写入，控制节点不会回写。以下为指令对照，按阶段单独执行，不要整段连续发送：

```bash
rosparam set /ctrl_cmd/state 0   # 地面等待；空中暂停并悬停
rosparam set /ctrl_cmd/state 1   # 接管、解锁、起飞后悬停
rosparam set /ctrl_cmd/state 2   # 开始任务或恢复暂停任务
rosparam set /ctrl_cmd/state 3   # 中断任务并请求 AUTO.LAND
```

同一数值持续保持不会重复触发。节点启动后先执行一次 `state 0`，再设置 `state 1`。起飞完成后进入 HOLDING，等待显式 `state 2` 才开始任务；悬停时收到新目标也只加载，不自动执行。序列完成后保持最后位置，等待 `state 3`。

飞行中人工切出 OFFBOARD 后，节点停止自动目标输出并进入错误状态，不会重新抢占。此时由遥控器或 PX4 失效保护处置，落地锁定并排除原因后重启控制节点。

## 任务输入

任务源由 launch 参数 `target_source=file|param|topic` 选择，默认 `topic`。任务坐标默认使用 `mission_start`：原点为本轮起飞位置，X 为起飞时机头方向，Y 向左，Z 向上。也可以明确使用 `odom`。节点不会发布新的 TF 边。

### 单点话题

```bash
roslaunch ducted_offboard offboard_control.launch \
  target_source:=topic topic_mode:=single
```

向 `/ctrl_cmd/target` 发布带新鲜时间戳的 `geometry_msgs/PoseStamped`。新单点替换当前目标，并持续有效到到达、替换或降落。示例：

```bash
rostopic pub -1 /ctrl_cmd/target geometry_msgs/PoseStamped "{header: {stamp: now, frame_id: mission_start}, pose: {position: {x: 3.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

### 序列话题

```bash
roslaunch ducted_offboard offboard_control.launch \
  target_source:=topic topic_mode:=sequence
```

向 `/ctrl_cmd/waypoints` 发布 `nav_msgs/Path`。新序列只会在地面等待、暂停或任务完成悬停阶段接收。暂停期间的新序列从第一个点开始；没有新序列时 `state 2` 恢复原进度。

```bash
rostopic pub -1 /ctrl_cmd/waypoints nav_msgs/Path "{header: {stamp: now, frame_id: mission_start}, poses: [{header: {stamp: now, frame_id: mission_start}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}, {header: {stamp: now, frame_id: mission_start}, pose: {position: {x: 3.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}]}"
```

### YAML 文件

```bash
roslaunch ducted_offboard offboard_control.launch \
  target_source:=file \
  mission_file:=/home/nrc/catkin_ws/src/ducted_offboard/config/mission_example.yaml
```

文件格式：

```yaml
frame_id: mission_start
waypoints:
  - {x: 0.0, y: 0.0, z: 1.0, yaw_deg: 0.0}
  - {x: 3.0, y: 0.0, z: 1.0, yaw_deg: 0.0}
```

### 私有参数

在自定义 launch 中设置节点私有参数 `waypoints`，再令 `target_source:=param`：

```xml
<rosparam param="waypoints">
  - {x: 0.0, y: 0.0, z: 1.0, yaw_deg: 0.0}
  - {x: 3.0, y: 0.0, z: 1.0, yaw_deg: 0.0}
</rosparam>
```

## 默认控制参数

- 起飞高度：相对本轮起飞点 `1.0 m`
- 水平目标推进：`0.3 m/s`
- 垂直目标推进：`0.2 m/s`
- 航向推进：`20 deg/s`
- 到点阈值：水平 `0.15 m`、高度 `0.10 m`、航向 `10 deg`、速度 `0.15 m/s`
- 连续到点时间：`1.0 s`
- 起飞超时：`30 s`
- 单点超时：`120 s`

这些值位于 `config/offboard.yaml`，也可通过 launch 参数覆盖常用项。目标推进限制的是发给 PX4 的期望点变化速度，实际速度和加速度仍受 PX4 参数与控制器限制。

## 日志

每次启动在 `/home/nrc/catkin_ws/logs/offboard_control/session_<UTC>_<PID>/` 建立目录：

- `events.log`：命令、状态切换、目标首发、服务请求与反馈、重试和错误；按大小轮换。
- `telemetry.csv`：默认 5 Hz 记录实际位姿、目标、误差、模式、解锁状态和任务进度。
- `config.yaml`：本次实际配置快照。

查看最近日志：

```bash
ls -td /home/nrc/catkin_ws/logs/offboard_control/session_* | head -1
tail -f /home/nrc/catkin_ws/logs/offboard_control/session_*/events.log
```

## 常见问题

**一直停在 `BOOT_WAIT_ZERO`**：执行 `rosparam set /ctrl_cmd/state 0`。这是防止重启后遗留的 `1` 自动触发起飞。

**OFFBOARD 被拒绝**：确认 `/mavros/setpoint_position/local` 稳定达到 20 Hz、飞控已连接且本地位姿有效。节点共请求 4 次，均失败后以非零状态退出；排除原因后重启。

**解锁失败**：查看 PX4 preflight check、遥控器 kill/arm 开关、电池、EKF 和安全开关。节点不会绕过 PX4 的解锁检查。

**`pose unavailable or stale` 或 `velocity unavailable or stale`**：检查建图或重定位输出以及 MAVROS 本地位姿。定位失效时节点停止发送旧目标，由 PX4 既有失效保护接管。

**任务文件错误**：检查文件存在、`waypoints` 非空、数值有限，并且 `frame_id` 仅为 `mission_start` 或 `odom`。启动错误会明确写入终端。

**落地后仍未锁定**：节点只使用新鲜的 PX4 `ON_GROUND` 反馈，连续 5 秒仍解锁才请求正常锁定。若仍失败，保留错误状态，不会强制停桨；使用遥控器和 PX4 标准流程处置并检查事件日志。

## 软件验证

核心单元测试：

```bash
cd /home/nrc/catkin_ws
catkin_make run_tests_ducted_offboard
catkin_test_results build/test_results/ducted_offboard
```

独立伪 MAVROS 集成测试使用单独的 ROS 主站，不连接真实飞控：

```bash
source /home/nrc/catkin_ws/devel/setup.bash
python3 /home/nrc/catkin_ws/src/ducted_offboard/test/integration/offboard_isolated.py
```

结果写入 `/home/nrc/catkin_ws/logs/offboard_control_isolated/result.json`。

## 任务与执行分层（2026-09-15）

当前只接受 `execution_mode=direct`，没有自动避障。EGO 源码保留但生产入口禁止运行；传入 ego 会启动失败，不静默改成直接飞行。

- `task_machine.cpp`：任务列表、启动先观察 0、指令变化、坐标转换、到点连续判定、任务超时，20 Hz。
- `execution.cpp`：预发送、OFFBOARD/解锁确认、起飞、限速跟踪、悬停、AUTO.LAND、落地与正常锁定，20 Hz；唯一发布 MAVROS 位置目标。
- `execution.hpp`：不可变执行请求/反馈快照，包含任务代次、请求 ID、目标和单调时间；执行器不读取任务列表或命令参数。
- `offboard_node.cpp`：两个独立循环、传感器缓存、单个 MAVROS 服务工作线程和有界异步日志队列。生产目标不链接 `planner_bridge.cpp` 或旧控制器；旧核心只供离线回归测试。

任务授权 `task_timeout` 默认 0.5 s。超时后地面撤销接管/解锁；空中定位有效时悬停。任务恢复、新任务到达均不自动续飞，操作员先 0 再 2；正在进行的降落由执行器继续完成。定位失效或人工切出 OFFBOARD 停止旧目标流。

`/ctrl_cmd/status` 增加 `task_phase`、`executor_phase`、`task_generation`、`task_heartbeat_age`；旧规划字段固定 FROZEN、False、空编号和 -1。日志区分 `[Task]` 与 `[Executor]`，异步队列最多 1024 条，满时优先丢遥测并记录丢弃数。

新增隔离验证：

```bash
catkin_make offboard_controller_test_node -j3 -l3
python3 src/ducted_offboard/test/integration/split_threads_isolated.py
python3 src/ducted_offboard/test/integration/frozen_launch_checks.py
```

线程故障注入只编译到非安装的测试二进制，生产节点没有停顿注入参数。独立测试主站 11329，结果在 `logs/offboard_split_isolated/result.json`；覆盖任务线程停顿、持续目标流、超时悬停、不自动恢复、正常降落、唯一发布者与冻结入口拒绝。

日常完整操作见 `docs/涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md`：基础系统 → 使用 `flight_test_01/GlobalMap.pcd` 重定位 → 控制器 → 加载任务 → 1 起飞悬停 → 2 执行 → 3 降落。也可用建图定位替换重定位，不固定依赖已有地图。
