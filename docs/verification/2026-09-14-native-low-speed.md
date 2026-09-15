# PX4 原生低速降落：源码核验与远程部署

日期：2026-09-14。目标为既有“上升一米、前飞三米、悬停五秒”任务增加原生低空下降目标 0.2 m/s。本地实现后，已部署 `/home/nrc/catkin_ws`，完成编译、相关测试及实际飞控参数的备份、应用和重新读取。没有刷写固件、解锁、切换飞行模式或执行实机飞行。

## 核验结论

标准 PX4 v1.12.3 可以通过参数实现这一目标，本次没有修改固件。`MPC_LAND_SPEED` 的 0.6 m/s 最小值是参数描述元数据；MAVLink 参数写入和参数存储没有执行相同下限。`FlightTaskAutoMapper::_getLandSpeed()` 在低于 `MPC_LAND_ALT2` 时读取该参数，向原生轨迹平滑器提供正向 NED 下降速度。高于低空范围会按高度插值，不能把 0.2 m/s 解释为所有高度的下降速度上限。

落地检测的下降意图门槛为 `0.9 * max(MPC_LAND_SPEED, 0.1)`。配置 0.2 后门槛为 0.18 m/s，垂直静止判断相应收紧到约 0.09 m/s；仍要求低推力、运动状态和原生分阶段落地条件。源码会将过大的 `LNDMC_Z_VEL_MAX` 自动收紧到 `MPC_LAND_SPEED`，因此工具将它纳入备份和恢复。

这修正了之前将“参数元数据最小值”直接理解为“原生控制器强制下限”的判断。旧 OFFBOARD 自定义 `slow_land` 的兼容闭锁继续保留。

## 实现

- `catkin_ws/configure_native_landing.py`：默认只读；`--apply` 仅在新鲜 connected、未解锁、ON_GROUND 下备份并写入；`--restore` 显式恢复同一飞控及固件的备份。
- `native_landing.py`：限制写入 `MPC_LAND_SPEED=0.2`、`MPC_LAND_RC_HELP=0`、`LNDMC_Z_VEL_MAX<=0.2`。检查统一速度覆盖、低空范围、下降限速、落地检测与自动锁定参数。备份落盘后才允许写入。
- `native_landing_ros.py`：有界 RPC、强制参数重新拉取、参数 ACK 与最终读回核对。超时后本进程禁止继续 RPC，不自动重试或回滚。恢复先写降落速度，再写落地检测阈值，避免 PX4 再次联动截断。
- `prepare_forward_test.py`：默认相对起飞点三米任务在准备及 `--start` 时检查参数，准备文件绑定飞控 UID、系统号、组件号、固件版本及固件标识。不在任务准备或启动时写参数。旧 `--height-mode terrain` 通用路径没有新增 0.2 m/s 保证。

参数改变影响飞控使用同一参数的其他降落或失效下降行为。禁用 `MPC_LAND_RC_HELP` 后不会由遥控杆在 AUTO.LAND 内叠加速度，模式接管功能仍保留。参数检查只发生在准备及明确启动时，不代表持续参数监控。

## 验证范围

| 验证 | 结果 |
|---|---|
| 提取原始 C++ 函数并编译运行 | 16 项通过；0.7/0.2 目标、RC 倍增、低速接地意图、移动/推力/旋转等拒绝条件 |
| 参数策略及事务 | 12 项通过；备份先于写入、联动恢复、飞控身份绑定、未解锁门控、未知结果停止、读回失败 |
| MAVROS RPC 边界 | 5 项通过；迟到响应不恢复写权限、服务异常、明确拒绝、ACK 不符和限定写集合 |
| 任务准备纯逻辑 | 7 项通过；相对高度、前飞方向、包络及 0.2 m/s 原生任务字段 |
| 既有降落兼容及相对高度回归 | 10 项通过；与新增参数/RPC测试合计 27 项降落测试通过 |
| 远程 ARM 编译 | `catkin_make -j3 -l3` 成功；`logs/native_low_speed_build.log` |
| 远程相关测试 | 降落27项、任务准备7项通过；`logs/native_low_speed_tests.log` |
| 实际参数写入及重新读取 | 两次均成功；`logs/native_low_speed_apply.json`、`logs/native_low_speed_after.json` |
| 实际飞控状态 | connected=true、armed=false、landed_state=1、mode=MANUAL；`logs/native_low_speed_final_state.json` |

实际参数变化：`MPC_LAND_SPEED` 约 0.7→0.2 m/s，`LNDMC_Z_VEL_MAX` 0.5→0.2 m/s；`MPC_LAND_RC_HELP` 原值即为0，`COM_DISARM_LAND` 保持2 s。`MPC_Z_VEL_ALL=-3`、`MPC_LAND_ALT1=10`、`MPC_LAND_ALT2=5`、`MPC_Z_VEL_MAX_DN=1`、`LNDMC_TRIG_TIME=1`、`COM_OBS_AVOID=0` 均未改变。浮点读回0.20000000298视为0.2。

首次参数备份：`logs/native_landing/20260914T022512Z_8e5b5ba4_backup.json`。源文件部署前备份：`logs/native-landing-source-before-20260914T102142.tar.gz`。部署前的21个目标文件均与本地基线或预期新文件状态相符。

原始函数核验工具为仓库根目录 `tools/px4/verify_native_landing.py`，输入官方 v1.12.3 源码目录，输出源文件 SHA256 与[结果 JSON](native_landing_source_result.json)。C++ fixture 只模拟输入、参数和订阅；没有飞行动力学，落地滞回使用合成状态，不能据此声称通过了完整 PX4 SITL、真实触地或滞回时序测试。

示例重现（具备 C++17 编译器）：

```bash
python3 tools/px4/verify_native_landing.py /path/to/PX4-Autopilot-1.12.3 --output /tmp/native-landing-check
python3 catkin_ws/src/ducted_control/test/test_native_landing.py
python3 catkin_ws/src/ducted_control/test/test_native_landing_ros.py
python3 catkin_ws/src/ducted_bringup/test/test_forward_profile.py
```

实际飞控本次报告 PX4 1.12.3，MAVROS 固件标识为 `dee102df42000000`。版本号本身不能证明厂商固件与官方标签逐字一致；标准源码的行为核验与实际参数写入、重新读取分别留证。本次没有重启飞控验证参数持久化，也没有测试真实下降或触地表现。没有生成或刷入新的固件。

## 原始来源

- [参数定义](https://github.com/PX4/PX4-Autopilot/blob/v1.12.3/src/modules/mc_pos_control/mc_pos_control_params.c)
- [MAVLink 参数写入](https://github.com/PX4/PX4-Autopilot/blob/v1.12.3/src/modules/mavlink/mavlink_parameters.cpp)
- [参数存储](https://github.com/PX4/PX4-Autopilot/blob/v1.12.3/src/lib/parameters/parameters.cpp)
- [原生降落目标](https://github.com/PX4/PX4-Autopilot/blob/v1.12.3/src/modules/flight_mode_manager/tasks/AutoMapper/FlightTaskAutoMapper.cpp)
- [落地检测](https://github.com/PX4/PX4-Autopilot/blob/v1.12.3/src/modules/land_detector/MulticopterLandDetector.cpp)
- [统一速度参数与下降限速](https://github.com/PX4/PX4-Autopilot/blob/v1.12.3/src/modules/mc_pos_control/MulticopterPositionControl.cpp)
