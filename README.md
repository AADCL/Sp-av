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
  → fastlio_odometry_adapter.py
  → /mavros/odometry/out
  → MAVROS ENU/FLU → NED/FRD
  → PX4 MAVLink ODOMETRY

TF：odom → map → livox_frame → base_link
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
| 局部避障 | `roslaunch ducted_bringup local_avoidance.launch` |
| 航点任务 | `roslaunch ducted_bringup waypoint_mission.launch` |
| 记录 | `roslaunch ducted_bringup recording.launch` |

建图与重定位二选一。飞控入口自带RC监视。外部里程计、地形参考、RC映射及飞行/避障/任务输出保留显式确认参数，默认关闭；启动节点本身不会接管、解锁或执行任务。具体启用方式和接口见使用文档。

组合示例，基础层仍需单独运行：

```bash
roslaunch ducted_bringup modules.launch \
  localization:=relocalization \
  map_file:=/home/nrc/catkin_ws/maps/step6_validation_20260910/GlobalMap.pcd \
  start_recording:=true
```

地图在建图节点运行时显式调用 `/ducted/mapping/save_map` 保存，结束进程不会自动保存。仓库地图的来源和用途见 [maps/README.md](catkin_ws/maps/README.md)；验证场景地图不能自动适用于其他现场。

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
