# 地图文件

本目录随工作空间上传，保留远程主机中的现有PCD文件。

| 文件/目录 | 来源与用途 |
|---|---|
| `GlobalMap.pcd` | 本机已有环境地图，83,903,901字节、2,621,989点；不是当前小范围验证场景地图 |
| `step4_validation/` | 重定位接口验证时保存的静止场景地图及位姿文件 |
| `step6_validation_20260910/` | 当前场景的软件链路验证地图，包含GlobalMap、SurfMap、过滤地图和位姿文件 |

默认launch读取 `/home/nrc/catkin_ws/maps/GlobalMap.pcd`。使用验证场景时显式传入 `map_file:=/home/nrc/catkin_ws/maps/step6_validation_20260910/GlobalMap.pcd`。
地图必须与实际场景对应，小范围静止记录不能解释为完整飞行区域地图。
新地图在建图节点运行期间调用 `/ducted/mapping/save_map` 保存，destination指向事先建立的独立目录。

`GlobalMap.pcd` 的SHA-256：

```text
71394ae78b9d67fbbb6bf9c502db9a578c11637f8dd0b7c993ab6f1f3ccccd7b
```
