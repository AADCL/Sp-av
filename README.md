# Sp-av — 涵道四旋翼无人机

控制指令采用分步操作：`state=1` 只起飞并悬停，`state=2` 才开始／恢复任务，`state=3` 降落；悬停时收到新目标只加载，不自动执行。

2026-09-15：EGO 源码保留但运行冻结。当前 C++ `ducted_offboard` 将任务状态机和飞控执行器拆为独立模块与 20 Hz 线程，保持一个节点，仅支持直接限速航点跟踪，**没有自动避障**。1 起飞后悬停，2 执行／恢复任务，3 使用 PX4 AUTO.LAND 降落。

默认重定位支持未知位姿自动搜索（FPFH/RANSAC + ICP），同时保留 RViz 手动初值并优先响应。成功后持续地图跟踪。首次安装依赖请执行 `bash catkin_ws/src/ducted_localization/scripts/setup_python.sh`。


本仓库保存涵道四旋翼无人机的ROS Noetic工作空间源码、硬件配置、地图和中文文档。工程以远程主机 `/home/nrc/catkin_ws` 的当前版本为基础，按功能模块独立启动：先运行 `base_system.launch`，再启动所需功能。

## 文档

| 文档 | 内容 |
|---|---|
| [详细信息表 V1.0](docs/涵道四旋翼无人机_详细信息表_V1.0.md) | 硬件、网络、外参、TF、节点、话题、参数及故障查询 |
| [使用文档 V1.0](docs/涵道四旋翼无人机_使用文档_V1.0.md) | 基础检查、建图保存、可选地图重定位、地形、遥控读取、直接控制与记录 |
| [开发实施文档 V1.0](docs/涵道四旋翼无人机_开发实施文档_V1.0.md) | 源码组织、依赖、配置、构建、实现细节和软件验证 |
| [自动飞行任务操作手册 V1.0](docs/涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md) | C++ 分层直接控制、七步任务操作、暂停恢复和降落 |

## 仓库结构

```text
Sp-av/
├─ README.md
├─ catkin_ws/
│  ├─ src/
│  │  ├─ ducted_bringup
│  │  ├─ ducted_msgs
│  │  ├─ ducted_control
│  │  ├─ ducted_offboard
│  │  ├─ ducted_navigation
│  │  ├─ ducted_mapping
│  │  ├─ ducted_localization
│  │  ├─ ducted_planning
│  │  ├─ fast_lio_sam
│  │  └─ sfast_lio
│  ├─ maps/
│  ├─ save_map.sh
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

默认 `terrain` 模式的 `h_agl` 对应机体中心；水平姿态下机底离地高度为 `h_agl - 0.10 m`。直接控制使用本轮起飞位置为高度参考，不依赖 h_agl，也不覆盖定位 Z；规划已冻结。

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

远程主机 `nrc` 用户的新 Bash 终端已自动加载工作空间。新安装或未配置环境的终端才需要手动 `source /home/nrc/catkin_ws/devel/setup.bash`。首先启动基础层：

```bash
roslaunch ducted_bringup base_system.launch
```

另开终端检查 `/ducted/system/ready`，再选择功能：

| 功能 | 命令 |
|---|---|
| 建图 | `roslaunch ducted_bringup mapping.launch` |
| 重定位 | `roslaunch ducted_bringup relocalization.launch map_file:=/home/nrc/catkin_ws/maps/flight_test_01/GlobalMap.pcd` |
| 外部里程计 | 已包含在 `base_system.launch`，无需另开终端 |
| 相对地面高度 | `roslaunch ducted_bringup terrain_height.launch` |
| 遥控处理 | `roslaunch ducted_bringup rc_processing.launch` |
| C++ 直接控制 | `roslaunch ducted_offboard offboard_control.launch`，分步指令见任务手册 |
| EGO 局部规划 | 已冻结，运行入口明确拒绝启动 |
| 记录 | `roslaunch ducted_bringup recording.launch` |

**地图由用户选择**：表中 `flight_test_01/GlobalMap.pcd` 只是本次操作示例。将 `map_file:=` 后的路径换为当前场地的地图即可；省略该参数仍会读取 launch 原有默认路径 `/home/nrc/catkin_ws/maps/GlobalMap.pcd`。已有地图不是控制器必需项，也可选择建图定位。

建图与重定位二选一。两者均自动启动 `/localization_frames`；收到有效定位和未解锁飞控姿态后，形成 `odom → camera_init → body → base_link`。基础层末尾默认启动外部里程计转发，等待基础就绪和有效 `odom/base_link` 数据后送入 PX4，无需重复启动 `external_odometry.launch`。诊断时可用 `base_system.launch start_external_odometry:=false` 关闭转发。RC读取使用独立的rc_processing.launch。旧 Python 飞控执行与任务模块已删除；当前 C++ OFFBOARD 采用独立任务层与执行器直接跟踪。

组合示例，基础层仍需单独运行：

```bash
roslaunch ducted_bringup modules.launch \
  localization:=relocalization \
  map_file:=/home/nrc/catkin_ws/maps/flight_test_01/GlobalMap.pcd \
  start_recording:=true
```

建图默认启用移植自 AG-TEST 的动态点过滤：距离裁剪、体素降采样、半径离群过滤、贝叶斯时间一致性确认和射线清除。每帧扫描留档，保存时按回环优化后的关键帧位姿重放；只清理保存的静态地图，不改变实时定位点云；当前未启用自动避障。

保存目录必须尚不存在，默认导出 5 cm 静态点云及 `mapping_metadata.yaml`，保留动态障碍物过滤。读取和半径去噪使用两个并行任务，时间过滤仍逐帧有序执行。当前重定位不依赖占据栅格，地图保存默认不生成该额外文件；需要记录观测空间时，启动建图加 `export_occupancy:=true`，再保存 `observed_occupancy.pcd`。默认扫描留档上限 2 GiB，达到上限会明确拒绝不完整地图导出。

同一批2008帧回放中，优化后约30秒导出，原流程约84秒；点数从9851增至229549。条件与日志见[地图导出验证](docs/verification/2026-09-12-map-export.md)。

EGO-Planner 算法源码与离线单元测试保留，生产节点和全部规划 launch 已冻结。当前 OFFBOARD 只接受 direct，不执行自动避障；任务和执行器通过不可变内部快照通信，任务心跳超时后执行器独立悬停。

地图在建图节点运行时显式保存。在 `/home/nrc/catkin_ws` 执行 `./save_map.sh`，自动加载环境、按时间命名并显示进度；也可执行 `./save_map.sh site_c` 指定新目录名。地图保存在 `maps/` 下，已有目录不会覆盖。Ctrl+C 只退出进度显示，后台保存继续；`./save_map.sh --status` 查询状态，`./save_map.sh --wait` 重新显示进度。仅 `SUCCEEDED` 或“保存完成”表示完成。保存期间保持机体静止、基础层和建图运行，结束建图进程不会自动保存。原 Python 客户端及同步服务 `/ducted/mapping/save_map` 继续保留。仓库地图的来源和用途见 [maps/README.md](catkin_ws/maps/README.md)。

## 软件验证

当前分层控制版本已完成远程编译和隔离 ROS 软件验证。发布前在远程重新运行 45 个 C++ testcase 和 2 组启动入口检查，均通过；没有向真实飞控发送控制命令。用户已反馈完成测试，未提供新的量化飞行结果，本次不据此追加飞行性能结论。

| 分层版本验证 | 结果 |
|---|---|
| 任务/执行器与保留 C++ 回归 | 45 个 testcase 通过 |
| 导航与 EGO 离线算法 | 135 项 Python、5 项 C++ 通过；运行仍冻结 |
| 直接控制模拟 | 12 项通过 |
| 线程隔离与冻结入口模拟 | 6 组通过；任务线程停顿时执行线程继续运行 |
| 启动组合解析 | 96 种通过 |

上述完整验证在分层改造时执行，发布前复查范围另列于[发布核对记录](docs/verification/2026-09-15-github-update.md)。原始范围和证据见[分层控制验证](docs/verification/2026-09-15-offboard-split.md)，旧版本结果保留在[验证索引](docs/verification/README.md)，不代表当前启用了旧自动控制或规划功能。

```bash
cd /home/nrc/catkin_ws
python3 src/ducted_bringup/test/integration/verify_software.py --integration
```

## 历史实现记录（自动控制部分已移除）

- 将基础通信、建图、重定位、地形、飞控、避障、任务和记录拆成按需启动的模块。
- 修正FAST-LIO速度/协方差契约、安装杆臂补偿及竖直坐标对齐。
- 引入中心AGL、三维机体包络和绝对位置目标，保持定位高度语义一致。
- 采用PX4原生位置控制、显式控制权状态机、RC标定和数据失效处理。
- 增加航点开始/暂停/恢复/取消、实际到点停留确认、末点保持及日志验证入口。

### 历史 EGO 与自动飞行软件验证

使用独立 ROS master、合成点云和模拟飞控验证。结果记录见 [EGO 与自动飞行验证](docs/verification/2026-09-11-ego-automatic.md)。

本版按运行阶段划分公共TF（2026-09-11更新）：

- 仅基础系统：无定位TF边，不出现`map`、`odom`、`camera_init`或`body`定位链；`base_link`为机体参考。
- 建图：`odom → camera_init → body → base_link`，不存在`map`。
- 重定位成功且源数据新鲜：`map → odom → camera_init → body → base_link`。

`camera_init → body` 由局部 FAST-LIO 发布。默认 `ducted_localization` 使用 FPFH/RANSAC 初始化和 ICP 地图跟踪，发布 `/ducted/relocalization/global_odom`（map/body）而不发布 TF；`use_global_relocalizer:=false` 才回退到旧 `sfast_lio` 后端。`world_tf_owner.py` 按同一扫描时间配对全局与局部位姿，计算 `T_map_odom = T_map_base × inverse(T_odom_base)`；全局校正不会重置局部 FAST-LIO。数据过期或重定位无效时停止发布全局校正，`/ducted/localization/map_ready=false`。TF 缓存可能暂时保留旧变换，应结合 ready 状态判断有效性。

`body`是MID360内部IMU参考，不是机体中心；`body → base_link`为原安装外参的逆变换，外参数值未改。保存PCD坐标沿用建图的`camera_init`数值，加载后将该固定地图坐标命名为`map`，建图时无需发布map TF。重定位RViz的Fixed Frame使用`map`；建图使用`odom`或`camera_init`。

MAVROS 的 ENU/NED、FLU/FRD 辅助变换位于 `/mavros/internal_tf_static`，内部里程计转换继续使用这些变换。默认重定位运行局部 FAST-LIO 与独立地图配准工作进程。

## 当前自动控制

使用 `roslaunch ducted_offboard offboard_control.launch`。任务状态机和执行器各自以 20 Hz 运行，单个服务线程调用 MAVROS，有界日志线程异步落盘。0 等待/暂停、1 起飞后悬停、2 执行/恢复、3 降落。默认直接目标推进水平 0.3 m/s、垂直 0.2 m/s、yaw 20°/s；AUTO.LAND 使用 PX4 参数。当前没有自动避障。

七步操作与每一步的继续条件见[自动飞行任务操作手册](docs/涵道四旋翼无人机_自动飞行任务操作手册_V1.0.md)。手册通过 `MAP_FILE` 选择地图，示例值为 `flight_test_01/GlobalMap.pcd`；用户可替换路径，也可改用建图定位。
