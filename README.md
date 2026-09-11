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
| 全局＋局部规划 | `roslaunch ducted_bringup global_local_planning.launch occupancy_file:=/home/nrc/catkin_ws/maps/site_b/observed_occupancy.pcd` |
| 独立扇区避障（兼容入口） | `roslaunch ducted_bringup local_avoidance.launch` |
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

保存目录必须尚不存在，成功后同时得到静态 `GlobalMap.pcd`、`observed_occupancy.pcd` 和 `mapping_metadata.yaml`。后两者记录实际观测的三维空闲／占据空间；旧 PCD 不包含空闲观测，不能直接作为完整全局规划输入。默认扫描留档上限 2 GiB，达到上限会明确拒绝不完整地图导出。

全局规划在配置工作范围内对三维已观测空间执行 A*；局部规划复用 Fast-Planner 的运动学 A* 和 B 样条优化。未知空间和地图外部按阻挡处理。输出经机体包络、制动余量、相对地面高度、数据时效和实际位置指令扫掠检查后进入原飞控接口。B 样条作为位置参考，不直接向 PX4 发送速度或加速度前馈。

地图在建图节点运行时显式保存，推荐 `rosrun ducted_bringup save_map.py start --destination /home/nrc/catkin_ws/maps/新目录名` 后台提交，使用 `rosrun ducted_bringup save_map.py status` 查看进度；仅 `SUCCEEDED` 表示完成。保存期间保持机体静止和建图运行，结束进程不会自动保存。旧同步服务 `/ducted/mapping/save_map` 继续保留。仓库地图的来源和用途见 [maps/README.md](catkin_ws/maps/README.md)；验证场景地图不能自动适用于其他现场。

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

### 本次动态滤波与全局／局部规划验证

- 真实未解锁建图：222帧、约10 Hz；导出10,988个静态点和观测占据数据，TF连续，已有地图拒绝覆盖。
- 规划核心隔离测试：9项通过，包含全局绕墙、局部B样条、绝对高度、完全封堵、未知区域和动态障碍。
- 新规划后端与航点／飞控隔离整链：28项通过，包含两航点完成、暂停恢复和数据中断。
- 原有VFH记录仍单独保留，不作为新后端的验证结果；没有进行真实自动飞行。


本版按运行阶段划分公共TF（2026-09-11更新）：

- 仅基础系统：无定位TF边，不出现`map`、`odom`、`camera_init`或`body`定位链；`base_link`为机体参考。
- 建图：`odom → camera_init → body → base_link`，不存在`map`。
- 重定位成功且源数据新鲜：`map → odom → camera_init → body → base_link`。

`camera_init → body`由独立的局部FAST-LIO发布。重定位时额外运行局部scan-to-map里程计；原`sfast_lio`负责地图匹配，只发布`/ducted/relocalization/global_odom`（map/body）且不发布TF。`world_tf_owner.py`按相同扫描时间戳配对全局与局部机体位姿，计算`T_map_odom = T_map_base × inverse(T_odom_base)`；全局校正不会重置局部FAST-LIO。数据过期或重定位无效时停止发布全局校正，`/ducted/localization/map_ready=false`。TF缓存可能短暂保留旧变换，不应只看树中是否还显示map判断定位有效。

`body`是MID360内部IMU参考，不是机体中心；`body → base_link`为原安装外参的逆变换，外参数值未改。保存PCD坐标沿用建图的`camera_init`数值，加载后将该固定地图坐标命名为`map`，建图时无需发布map TF。重定位RViz的Fixed Frame使用`map`；建图使用`odom`或`camera_init`。

MAVROS的ENU/NED、FLU/FRD辅助变换移至`/mavros/internal_tf_static`，其内部里程计转换仍使用这些变换，公共TF树不再出现辅助根。旧`odom → map`链已移除。重定位阶段增加一个局部FAST-LIO进程，计算负载高于原单进程方案。
