# Sp-av — 涵道四旋翼无人机

本仓库保存涵道四旋翼无人机的ROS Noetic工作空间源码、硬件配置、地图和中文文档。工程以远程主机 `/home/nrc/catkin_ws` 的当前版本为基础，按功能模块独立启动：先运行 `base_system.launch`，再启动所需功能。

## 文档

| 文档 | 内容 |
|---|---|
| [详细信息表 V1.0](docs/涵道四旋翼无人机_详细信息表_V1.0.md) | 硬件、网络、外参、TF、节点、话题、参数及故障查询 |
| [使用文档 V1.0](docs/涵道四旋翼无人机_使用文档_V1.0.md) | 基础检查、建图保存、重定位、地形、遥控、飞控、避障、航点与记录 |
| [开发实施文档 V1.0](docs/涵道四旋翼无人机_开发实施文档_V1.0.md) | 源码组织、依赖、配置、构建、实现细节和软件验证 |

## 仓库结构

```text
Sp-av/
├─ README.md
├─ catkin_ws/
│  ├─ src/
│  │  ├─ ducted_bringup
│  │  ├─ ducted_msgs
│  │  ├─ ducted_control
│  │  ├─ ducted_navigation
│  │  ├─ ducted_mission
│  │  ├─ fast_lio_sam
│  │  └─ sfast_lio
│  ├─ maps/
│  └─ README.md
└─ docs/
   ├─ 涵道四旋翼无人机_详细信息表_V1.0.md
   ├─ 涵道四旋翼无人机_使用文档_V1.0.md
   ├─ 涵道四旋翼无人机_开发实施文档_V1.0.md
   └─ verification/
```

`catkin_ws/`保存可维护源码、配置和地图；`build/`、`devel/`、运行日志、Python缓存及临时归档由 `.gitignore` 排除。`src/CMakeLists.txt` 是本机catkin生成的链接，首次构建时重新生成。

## 平台与机体

| 项目 | 当前配置 |
|---|---|
| 主控 | NVIDIA Orin NX，aarch64 |
| 系统 | Ubuntu 20.04.6 LTS、ROS Noetic |
| 雷达 | Livox MID360 |
| 飞控链路 | PX4、MAVROS、MAVLink 2 |
| SSH | `nrc@192.168.50.140` |
| 飞控串口 | `/dev/ttyTHS0:921600` |
| 雷达/接收端IP | `192.168.1.109` / `192.168.1.50` |
| 机体尺寸 | 前后0.60 m、左右0.70 m、总高0.20 m |
| 机体坐标 | 中心原点；X朝雷达前方、Y向左、Z向上 |

共享几何文件：[airframe_geometry.yaml](catkin_ws/src/ducted_bringup/config/airframe_geometry.yaml)。八顶点范围为X±0.30 m、Y±0.35 m、Z±0.10 m，由地形和避障共同加载。

## 高度与定位链路

定位保留固定坐标中的位置，地面高度模块单独估计 `h_agl`。原始FAST-LIO地图可能倾斜，适配器建立竖直对齐的 `odom`；这里的“绝对高度”指本地固定坐标高度，不是海拔。

```text
FAST-LIO /Odometry
  → /ducted/localization/odom（launch重映射）
  → fastlio_odometry_adapter.py（随建图/重定位启动，发布完整TF）
  → /ducted/localization/body_odom
  → external_odometry_relay.py（按需启动，默认禁止发送）
  → /mavros/odometry/out
  → MAVROS ENU/FLU → NED/FRD
  → PX4 MAVLink ODOMETRY

TF：odom → camera_init → body → base_link
```

安装位姿采用本机自带涵道程序中的平移 `[0.13, 0, 0]` m、RPY `[0.03, 0.4567, 0]` rad。MID360内部雷达—IMU平移 `[-0.011, -0.02329, 0.04412]` 是另一层内部标定，不重复用于机体安装补偿。

默认 `h_agl`对应机体中心；水平姿态下机底离地高度为 `h_agl - 0.10 m`。避障使用中心AGL评估候选点净空，输出仍为绝对 `odom` 目标，不覆盖定位Z。

## 获取与构建

```bash
git clone https://github.com/AADCL/Sp-av.git
cd Sp-av/catkin_ws
```

本机使用两个已有驱动依赖工作空间，完整依赖说明见开发实施文档。在本机环境中：

```bash
source /opt/ros/noetic/setup.bash
source /home/nrc/mavros_catkin_ws/devel/setup.bash --extend
source /home/nrc/ws_livox/devel/setup.bash --extend
if [ ! -e src/CMakeLists.txt ]; then
  catkin_init_workspace src
fi
catkin_make -j3 -l3
source devel/setup.bash
```

代码使用本机已有的Livox SDK2、GTSAM、Sophus、PCL、Eigen、GeographicLib及Python依赖。配置默认地图和记录路径位于 `/home/nrc/catkin_ws`；部署时以此为运行工作空间，其他路径运行时应显式修改对应配置或launch参数。

## 按需启动

首先启动基础层：

```bash
source /home/nrc/catkin_ws/devel/setup.bash
roslaunch ducted_bringup base_system.launch
```

另开终端加载同一环境，检查 `/ducted/system/ready`，再选择功能：

| 功能 | 命令 |
|---|---|
| 建图 | `roslaunch ducted_bringup mapping.launch` |
| 重定位 | `roslaunch ducted_bringup relocalization.launch map_file:=/home/nrc/catkin_ws/maps/step6_validation_20260910/GlobalMap.pcd` |
| 外部里程计 | `roslaunch ducted_bringup external_odometry.launch` |
| 相对地面高度 | `roslaunch ducted_bringup terrain_height.launch` |
| 遥控处理 | `roslaunch ducted_bringup rc_processing.launch` |
| 飞控执行器 | `roslaunch ducted_bringup flight_control.launch` |
| EGO 局部规划 | `roslaunch ducted_bringup ego_planner.launch` |
| 自动飞行组合 | `roslaunch ducted_bringup automatic_flight.launch` |
| 航点任务 | `roslaunch ducted_bringup waypoint_mission.launch` |
| 记录 | `roslaunch ducted_bringup recording.launch` |

建图与重定位二选一。两者均自动启动 `/localization_frames`；收到有效定位和未解锁飞控姿态后，形成 `odom → camera_init → body → base_link`。查看TF无需启动外部里程计发送模块。飞控入口自带RC监视。外部里程计、地形参考、RC映射及飞行/避障/任务输出保留显式确认参数，默认关闭；启动节点本身不会接管、解锁或执行任务。具体启用方式和接口见使用文档。

组合示例，基础层仍需单独运行：

```bash
roslaunch ducted_bringup modules.launch \
  localization:=relocalization \
  map_file:=/home/nrc/catkin_ws/maps/step6_validation_20260910/GlobalMap.pcd \
  start_recording:=true
```

建图默认启用移植自 AG-TEST 的动态点过滤：距离裁剪、体素降采样、半径离群过滤、贝叶斯时间一致性确认和射线清除。每帧扫描留档，保存时按回环优化后的关键帧位姿重放；只清理静态地图，实时避障仍使用完整扫描。

保存目录必须尚不存在，成功后同时得到静态 `GlobalMap.pcd`、`observed_occupancy.pcd` 和 `mapping_metadata.yaml`。后两者记录实际观测的三维空闲／占据空间，保留用于地图记录，不是 EGO 启动依赖。默认扫描留档上限 2 GiB，达到上限会明确拒绝不完整地图导出。

局部规划采用官方 EGO-Planner 的 ESDF-free 回弹 B 样条优化，取消独立全局规划器。输入为实时完整点云及短时保留／预测障碍，不依赖占据地图文件。输出通过机体、制动、h_agl、时效和实际位置指令扫掠检查，交给 PX4 原生位置控制器；不发送速度或加速度前馈。默认速度0.5 m/s、垂直速度0.3 m/s、加速度0.5 m/s²。

地图在建图节点运行时显式保存。在 `/home/nrc/catkin_ws` 执行 `./save_map.sh`，自动加载环境、按时间命名并显示进度；也可执行 `./save_map.sh site_c` 指定新目录名。地图保存在 `maps/` 下，已有目录不会覆盖。Ctrl+C 只退出进度显示，后台保存继续；`./save_map.sh --status` 查询状态，`./save_map.sh --wait` 重新显示进度。仅 `SUCCEEDED` 或“保存完成”表示完成。保存期间保持机体静止、基础层和建图运行，结束建图进程不会自动保存。原 Python 客户端及同步服务 `/ducted/mapping/save_map` 继续保留。仓库地图的来源和用途见 [maps/README.md](catkin_ws/maps/README.md)。

## 软件验证

```bash
cd /home/nrc/catkin_ws
source devel/setup.bash
python3 src/ducted_bringup/test/integration/verify_software.py --integration
```

2026-09-10当前几何版本已有验证记录：

| 检查 | 结果 |
|---|---:|
| 单元测试 | 291项通过：bringup 89、control 70、navigation 91、mission 41 |
| 模块组合解析 | 384种通过 |
| 地形隔离ROS | 15项通过 |
| 飞控隔离ROS | 32项通过 |
| 避障—任务—飞控整链 | 28项通过 |
| 原始传感器建图回放 | 377帧配准点云与377帧里程计 |

报告摘要保存在 [docs/verification](docs/verification/README.md)。隔离测试使用独立主站与模拟飞控，以上结果不等同于真实飞行性能。

## 本次实现内容

- 将基础通信、建图、重定位、地形、飞控、避障、任务和记录拆成按需启动的模块。
- 修正FAST-LIO速度/协方差契约、安装杆臂补偿及竖直坐标对齐。
- 引入中心AGL、三维机体包络和绝对位置目标，保持定位高度语义一致。
- 采用PX4原生位置控制、显式控制权状态机、RC标定和数据失效处理。
- 增加航点开始/暂停/恢复/取消、实际到点停留确认、末点保持及日志验证入口。

### EGO 与自动飞行软件验证

使用独立 ROS master、合成点云和模拟飞控验证。结果记录见 [EGO 与自动飞行验证](docs/verification/2026-09-11-ego-automatic.md)。

本版按运行阶段划分公共TF（2026-09-11更新）：

- 仅基础系统：无定位TF边，不出现`map`、`odom`、`camera_init`或`body`定位链；`base_link`为机体参考。
- 建图：`odom → camera_init → body → base_link`，不存在`map`。
- 重定位成功且源数据新鲜：`map → odom → camera_init → body → base_link`。

`camera_init → body`由独立的局部FAST-LIO发布。重定位时额外运行局部scan-to-map里程计；原`sfast_lio`负责地图匹配，只发布`/ducted/relocalization/global_odom`（map/body）且不发布TF。`world_tf_owner.py`按相同扫描时间戳配对全局与局部机体位姿，计算`T_map_odom = T_map_base × inverse(T_odom_base)`；全局校正不会重置局部FAST-LIO。数据过期或重定位无效时停止发布全局校正，`/ducted/localization/map_ready=false`。TF缓存可能短暂保留旧变换，不应只看树中是否还显示map判断定位有效。

`body`是MID360内部IMU参考，不是机体中心；`body → base_link`为原安装外参的逆变换，外参数值未改。保存PCD坐标沿用建图的`camera_init`数值，加载后将该固定地图坐标命名为`map`，建图时无需发布map TF。重定位RViz的Fixed Frame使用`map`；建图使用`odom`或`camera_init`。

MAVROS的ENU/NED、FLU/FRD辅助变换移至`/mavros/internal_tf_static`，其内部里程计转换仍使用这些变换，公共TF树不再出现辅助根。旧`odom → map`链已移除。重定位阶段增加一个局部FAST-LIO进程，计算负载高于原单进程方案。

## 自动飞行组合（EGO）

入口 `automatic_flight.launch` 组合飞控执行器、EGO、航点任务和自动协调节点。基础层、建图或重定位、外部里程计、相对高度模块先按需启动。它与单独启动的飞控／导航／任务节点不能重复运行。

默认全部输出关闭。自动流程不调用解锁服务：操作员解锁后，收到明确启动请求才按“预发送与 OFFBOARD 接管 → 垂直起飞 → 航点任务 → 末点悬停或配置降落”执行。起飞目标为启动时绝对 odom Z 加 `takeoff_rise`（默认1.1 m）；航点本身仍配置绝对 map/odom 高度，不能把 h_agl 填进绝对 Z。

~~~bash
# 启动组合仅加载节点，默认不产生飞行输出
roslaunch ducted_bringup automatic_flight.launch \
  mission_file:=/home/nrc/catkin_ws/src/ducted_mission/config/mission.yaml
rostopic echo /ducted/automatic/status
~~~

实际执行需显式启用 `enable_commands:=true enable_flight_output:=true enable_navigation_output:=true geometry_confirmed:=true mapping_confirmed:=true`，并满足外部里程计、地形与遥控器自身门控；之后才可调用：

~~~bash
rosservice call /ducted/automatic/start "{}"
rosservice call /ducted/automatic/cancel "{}"
~~~

`finish:=hold` 是默认值；`finish:=land` 仅在航点任务成功且末点保持确认后请求 PX4 `AUTO.LAND`，并等待落地且未解锁反馈。不会因启动、重启、数据恢复或前一次完成自行开始新任务。

起飞前和爬升期间检查实时点云、机体竖直通道、倾角及数据新鲜度，按扫描时间匹配位姿历史并覆盖期间运动。RC 切回手动／保持、急停、数据中断、任务失败或取消会撤销自动序列；已进入 PX4 降落的取消不反向切回 OFFBOARD。普通取消捕获当前位置保持；数据或控制权失效时停止外部控制输出，由原飞控失效逻辑处理。

状态话题 `/ducted/automatic/status` 为 JSON 字符串，含 `state`、`reason`、`generation`、`target_z`、`finish`。`/ducted/automatic/flight_lease` 与 `mission_lease` 提供0.5秒心跳约束，组合中任一授权中断都会被对应执行器检查。服务超时不重试飞行动作，`FAULT` 闭锁阻止再次自动启动。

原 `waypoint_mission.launch` 仍适合已经接管并悬停后的单独航点操作。官方 EGO 来源和修改记录见 `catkin_ws/src/ducted_planning/vendor/ego/PROVENANCE.md`。
