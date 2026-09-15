# 地图文件

地图由用户根据场地选择，控制器没有绑定某一个地图。本目录保存已发布的历史地图，并补充当前远程工作空间的 `flight_test_01` 六个地图与元数据文件。

| 文件/目录 | 来源与用途 |
|---|---|
| `flight_test_01/` | 当前操作手册默认示例；GlobalMap 为 164,366 点，5 cm 静态点云。元数据记录 863 帧、动态过滤及回环优化重放 |
| `GlobalMap.pcd` | 本机原有环境地图，2,621,989 点；仍是省略 map_file 时 launch 的默认文件 |
| `step4_validation/` | 早期静止场景的重定位接口验证地图 |
| `step6_validation_20260910/` | 早期软件链路验证地图，不是必选地图 |

在基础系统运行后，终端中设置本次选用路径：

```bash
MAP_FILE="/home/nrc/catkin_ws/maps/flight_test_01/GlobalMap.pcd"
ls -lh "$MAP_FILE"
roslaunch ducted_bringup relocalization.launch map_file:="$MAP_FILE"
```

第一行可换为其他有效 PCD 地图的完整路径。实际场地应与地图对应；更换地图在落地锁定、关闭控制器及原定位入口后进行。没有已有地图时可使用 `mapping.launch` 提供定位。

新地图在建图节点运行期间保存：

```bash
cd /home/nrc/catkin_ws
./save_map.sh my_new_site
```

`maps/my_new_site` 必须尚不存在，脚本自动创建并等待导出。出现“保存完成”后，下一次可将 `MAP_FILE` 改为 `/home/nrc/catkin_ws/maps/my_new_site/GlobalMap.pcd`。完整保存流程见使用文档。

`flight_test_01/GlobalMap.pcd` 的 SHA-256：

```text
84bc6551b237ff5edf4a63294d68eb93ba28257ee7176cee86f8f278673fdb45
```
