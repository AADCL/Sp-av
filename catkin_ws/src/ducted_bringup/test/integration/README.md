# 隔离 ROS 集成验证

先在 `/home/nrc/catkin_ws` 编译并 `source devel/setup.bash`，再显式运行所需脚本：

```bash
python3 src/ducted_bringup/test/integration/terrain_isolated.py
python3 src/ducted_bringup/test/integration/flight_isolated.py
python3 src/ducted_bringup/test/integration/navigation_mission_isolated.py
```

测试分别创建 `127.0.0.1:11322`、`:11323`、`:11324` 主站，使用合成地形和模拟飞控。
端口被占用时拒绝运行。
不要把这些测试改为真实主站。脚本停止其创建的子进程，并将结果保存在
`logs/terrain_isolated/result.json`、`logs/flight_isolated/result.json` 和
`logs/navigation_mission_isolated/result.json`。验收结果以当次 JSON 为准。

默认 catkin 单元测试不自动启动这些独立主站；脚本应单独顺序运行。避障—任务整链使用
共享配置中用户确认的0.60×0.70×0.20m中心包络；地面、运动、遥控器和飞控反馈
仍是合成数据，只用于软件测试，不代表现场地面真值或飞行验收。

自动验证入口（须先 source devel/setup.bash）：

```bash
python3 src/ducted_bringup/test/integration/verify_software.py
python3 src/ducted_bringup/test/integration/verify_software.py --integration
```

第一条顺序编译、运行四个自主开发包的 catkin 测试并解析 384 种模块组合；
第二条再顺序运行三项隔离联调。失败即停止，记录每阶段输出、退出码和结果 JSON。
`launch_matrix.py` 只解析启动图，不会启动硬件或 ROS 节点；组合通过不表示运行时
依赖就绪。建图组合仍不能代替重定位的有效性信号，外部里程计会保持闭锁。
