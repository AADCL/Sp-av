# 已有软件验证记录

以下文件摘自远程2026-09-10的运行记录，原完整报告目录为 `logs/software_verification_20260910_170650/`。
本目录保存结果摘要，不包含原始传感器bag或全部控制台日志。

| 文件 | 结果 |
|---|---|
| [统一结果](software_result.json) | 七阶段全部通过 |
| [单元测试汇总](unit_summary.txt) | 291项，0错误、0失败、0跳过 |
| [启动组合](launch_matrix.json) | 384种组合解析通过 |
| [地形隔离ROS](terrain_result.json) | 15项通过 |
| [飞控隔离ROS](flight_result.json) | 32项通过 |
| [避障任务整链](navigation_mission_result.json) | 28项通过 |
| [真实传感器回放](mapping_replay_result.json) | 377帧配准点云和377帧里程计 |
| [几何加载](airframe_geometry_audit.json) | 两模块共用八顶点 |
| [外参展开](active_extrinsics_audit.json) | 实际launch加载的外参与几何 |

隔离测试使用独立ROS主站和模拟飞控，回放使用已记录传感器数据；这些记录不等同于真实飞行性能。
