# 第 1 步：工作空间与公共接口

目标是在 `/home/nrc/catkin_ws` 建立可编译、可验证的 ROS Noetic 工程骨架。

## 交付物

1. 初始化 catkin 工作空间与 Git 仓库。
2. 创建 `ducted_msgs`，定义地面绝对高度与离地高度消息。
3. 创建 `ducted_bringup`，提供必选入口 `base_system.launch` 和统一配置。
4. 在脚手架阶段默认关闭 MAVROS、Livox，避免验证操作启动硬件。
5. 以 catkin 编译、CTest、消息解析和 roslaunch 解析作为验收条件。

## 边界

- 不启动或控制飞控。
- 不修改开机自启动。
- 不实现地面模态。
- 不接入指控平台。
