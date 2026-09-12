# Sp-av — 涵道四旋翼无人机

当前自动测试任务：以 PX4 起飞点为参考上升 1 m，沿初始机头方向前飞 3 m，悬停 5 s，然后由 PX4 原生 `AUTO.LAND` 完成降落和自动锁定。新任务不要求有效 `h_agl`，保留 EGO 实时障碍检查、人工解锁和明确启动。分终端操作见[自动飞行任务操作手册](docs/涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md)。

默认重定位支持未知位姿自动搜索（FPFH/RANSAC + ICP），同时保留 RViz 手动初值并优先响应。成功后持续地图跟踪。首次安装依赖请执行 `bash catkin_ws/src/ducted_localization/scripts/setup_python.sh`。


本仓库保存涵道四旋翼无人机的ROS Noetic工作空间源码、硬件配置、地图和中文文档。工程以远程主机 `/home/nrc/catkin_ws` 的当前版本为基础，按功能模块独立启动：先运行 `base_system.launch`，再启动所需功能。

## 文档

| 文档 | 内容 |
|---|---|
| [详细信息表 V1.0](docs/涵道四旋翼无人机_详细信息表_V1.0.md) | 硬件、网络、外参、TF、节点、话题、参数及故障查询 |
| [使用文档 V1.0](docs/涵道四旋翼无人机_使用文档_V1.0.md) | 基础检查、建图保存、重定位、地形、遥控、飞控、避障、航点与记录 |
| [开发实施文档 V1.0](docs/涵道四旋翼无人机_开发实施文档_V1.0.md) | 源码组织、依赖、配置、构建、实现细节和软件验证 |
| [自动飞行任务操作手册 V1.0](docs/涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md) | 起飞一米、前飞三米、悬停与 PX4 原生降落的分终端操作 |

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
│  │  ├─ ducted_mapping
│  │  ├─ ducted_localization
│  │  ├─ ducted_planning
│  │  ├─ ducted_mission
│  │  ├─ fast_lio_sam
│  │  └─ sfast_lio
│  ├─ maps/
│  ├─ save_map.sh
│  ├─ prepare_forward_test.py
│  └─ README.md
└─ docs/
   ├─ 涵道四旋翼无人机_详细信息表_V1.0.md
   ├─ 涵道四旋翼无人机_使用文档_V1.0.md
   ├─ 涵道四旋翼无人机_开发实施文档_V1.0.md
   ├─ 涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md
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
  → external_odometry_relay.py（随基础层启动，等待有效定位后发送）
  → /mavros/odometry/out
  → MAVROS ENU/FLU → NED/FRD
  → PX4 MAVLink ODOMETRY

TF：odom → camera_init → body → base_link
```

安装位姿采用本机自带涵道程序中的平移 `[0.13, 0, 0]` m、RPY `[0.03, 0.4567, 0]` rad。MID360内部雷达—IMU平移 `[-0.011, -0.02329, 0.04412]` 是另一层内部标定，不重复用于机体安装补偿。

默认 `terrain` 模式的 `h_agl` 对应机体中心；水平姿态下机底离地高度为 `h_agl - 0.10 m`。该模式使用中心 AGL 评估候选点净空；三米测试的 `takeoff_relative` 模式使用本次起飞参考约束高度，不生成有效雷达测高。两者都输出绝对 `odom` 目标，不覆盖定位 Z。

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
| 外部里程计 | 已包含在 `base_system.launch`，无需另开终端 |
| 相对地面高度 | `roslaunch ducted_bringup terrain_height.launch` |
| 遥控处理 | `roslaunch ducted_bringup rc_processing.launch` |
| 飞控执行器 | `roslaunch ducted_bringup flight_control.launch` |
| EGO 局部规划 | `roslaunch ducted_bringup ego_planner.launch` |
| 自动飞行组合 | `roslaunch ducted_bringup automatic_flight.launch` |
| 航点任务 | `roslaunch ducted_bringup waypoint_mission.launch` |
| 记录 | `roslaunch ducted_bringup recording.launch` |

建图与重定位二选一。两者均自动启动 `/localization_frames`；收到有效定位和未解锁飞控姿态后，形成 `odom → camera_init → body → base_link`。基础层末尾默认启动外部里程计转发，等待基础就绪和有效 `odom/base_link` 数据后送入 PX4，无需重复启动 `external_odometry.launch`。诊断时可用 `base_system.launch start_external_odometry:=false` 关闭转发。飞控入口自带RC监视。地形参考、RC映射及飞行/避障/任务输出保留显式确认参数，默认关闭；启动节点本身不会接管、解锁或执行任务。具体启用方式和接口见使用文档。

组合示例，基础层仍需单独运行：

```bash
roslaunch ducted_bringup modules.launch \
  localization:=relocalization \
  map_file:=/home/nrc/catkin_ws/maps/step6_validation_20260910/GlobalMap.pcd \
  start_recording:=true
```

建图默认启用移植自 AG-TEST 的动态点过滤：距离裁剪、体素降采样、半径离群过滤、贝叶斯时间一致性确认和射线清除。每帧扫描留档，保存时按回环优化后的关键帧位姿重放；只清理静态地图，实时避障仍使用完整扫描。

保存目录必须尚不存在，默认导出 5 cm 静态点云及 `mapping_metadata.yaml`，保留动态障碍物过滤。读取和半径去噪使用两个并行任务，时间过滤仍逐帧有序执行。EGO 使用实时点云，默认不生成额外的占据栅格；需要记录观测空间时，启动建图加 `export_occupancy:=true`，再保存 `observed_occupancy.pcd`。默认扫描留档上限 2 GiB，达到上限会明确拒绝不完整地图导出。

同一批2008帧回放中，优化后约30秒导出，原流程约84秒；点数从9851增至229549。条件与日志见[地图导出验证](docs/verification/2026-09-12-map-export.md)。

局部规划采用官方 EGO-Planner 的 ESDF-free 回弹 B 样条优化，取消独立全局规划器。输入为实时完整点云及短时保留／预测障碍，不依赖占据地图文件。输出通过机体、制动、所选模式的高度约束、时效和实际位置指令扫掠检查，交给 PX4 原生位置控制器；不发送速度或加速度前馈。默认速度0.5 m/s、垂直速度0.3 m/s、加速度0.5 m/s²；三米测试将前飞速度上限设为0.3 m/s。

地图在建图节点运行时显式保存。在 `/home/nrc/catkin_ws` 执行 `./save_map.sh`，自动加载环境、按时间命名并显示进度；也可执行 `./save_map.sh site_c` 指定新目录名。地图保存在 `maps/` 下，已有目录不会覆盖。Ctrl+C 只退出进度显示，后台保存继续；`./save_map.sh --status` 查询状态，`./save_map.sh --wait` 重新显示进度。仅 `SUCCEEDED` 或“保存完成”表示完成。保存期间保持机体静止、基础层和建图运行，结束建图进程不会自动保存。原 Python 客户端及同步服务 `/ducted/mapping/save_map` 继续保留。仓库地图的来源和用途见 [maps/README.md](catkin_ws/maps/README.md)。

## 软件验证

2026-09-12 的 PX4 相对高度与原生降落更新通过远程编译、171 项飞控/任务单元测试、7 项任务准备测试和 24 项隔离 ROS 检查。仅验证软件流程，未执行实机飞行；详情见[本次验证记录](docs/verification/2026-09-12-px4-native-landing.md)。下表保留早期版本的历史结果。

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

`camera_init → body` 由局部 FAST-LIO 发布。默认 `ducted_localization` 使用 FPFH/RANSAC 初始化和 ICP 地图跟踪，发布 `/ducted/relocalization/global_odom`（map/body）而不发布 TF；`use_global_relocalizer:=false` 才回退到旧 `sfast_lio` 后端。`world_tf_owner.py` 按同一扫描时间配对全局与局部位姿，计算 `T_map_odom = T_map_base × inverse(T_odom_base)`；全局校正不会重置局部 FAST-LIO。数据过期或重定位无效时停止发布全局校正，`/ducted/localization/map_ready=false`。TF 缓存可能暂时保留旧变换，应结合 ready 状态判断有效性。

`body`是MID360内部IMU参考，不是机体中心；`body → base_link`为原安装外参的逆变换，外参数值未改。保存PCD坐标沿用建图的`camera_init`数值，加载后将该固定地图坐标命名为`map`，建图时无需发布map TF。重定位RViz的Fixed Frame使用`map`；建图使用`odom`或`camera_init`。

MAVROS 的 ENU/NED、FLU/FRD 辅助变换位于 `/mavros/internal_tf_static`，内部里程计转换继续使用这些变换。默认重定位运行局部 FAST-LIO 与独立地图配准工作进程。

## 自动飞行组合（EGO，PX4 原生降落）

2026-09-12 更新：三米测试任务采用 `height_mode:=takeoff_relative` 和 `finish:=land`。准备时记录 PX4 局部高度 `z₀`，起飞到 `z₀ + 1.0 m`，沿初始机头方向前飞 3 m，悬停 5 s，然后请求 PX4 原生 `AUTO.LAND`。前飞速度上限 0.3 m/s。

下降控制、接地判断和自动锁定均由 PX4 完成。2026-09-12 只读获得的 `MPC_LAND_SPEED` 约为 0.7 m/s、`COM_DISARM_LAND` 为 2 s；实际使用飞控当前参数，本次没有改写这些参数。任务不再要求以 0.2 m/s 缓降，不会先发送自定义下降轨迹，也不按“高度五秒不变”确认落地。协调器等待新鲜的 ON_GROUND、未解锁及控制器退出反馈后报告成功。

一米是相对起飞点的上升量，不是实测离地高度。新模式不要求有效 `h_agl`；EGO 仍检查实时点云、机体包络、轨迹及数据时效。起飞参考平面只作内部高度约束，不发布成有效雷达测高，不提供随地形起伏保持离地高度的能力。机体应从平整地面起飞，落地区域保持空旷。

~~~bash
cd /home/nrc/catkin_ws
source devel/setup.bash
# 基础系统、重定位和外部里程计就绪后，在未解锁、静止状态准备
python3 prepare_forward_test.py --confirm-flat-ground && bash launch_forward_test.sh
~~~

准备参考与当前 ROS run_id 绑定，300 秒有效、单次使用。检查状态与 RC，人工解锁后，在另一终端执行 `python3 prepare_forward_test.py --start`。这一步才真正开始任务。旧准备文件必须重新生成，脚本会拒绝没有 `finish=land` 的旧配置。

通用 `automatic_flight.launch` 仍默认 `height_mode:=terrain`、`finish:=hold`，全部输出默认关闭；上述准备脚本显式选择新模式及原生降落。旧 `terrain` 模式保留实测地面高度检查，可显式使用 `--height-mode terrain --confirm-ground-contact` 准备。自定义 `slow_land` 和 `finish:=slow_land` 仍被阻止；该限制不再影响正常的相对高度起飞、航点任务或原生降落。

任务租约、手动接管、定位一致性、实时障碍、数据中断和服务超时检查继续生效。已进入 PX4 原生降落后，取消上位机任务不会自动切回 OFFBOARD。后续降落策略以实际日志为依据另行调整。

分终端操作见[自动飞行任务操作手册](docs/涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md)。已有自动搜索初值和持续重定位结果见[重定位验证记录](docs/verification/2026-09-12-global-relocalization.md)。
