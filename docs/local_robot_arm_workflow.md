# 本地 Robot_Arm_Project 运行规范

`application/` 是小车原有 ROS 1 后端。本流程不构建、不复制、不修改其中任何文件。

## 网络与职责

| 设备 | 系统 | 职责 |
|---|---|---|
| 小车 | ROS 1 Noetic | 保持原样启动 master_node，HTTP 服务为 `http://<VEHICLE_IP>:1819/robot/...` |
| 本地视觉电脑 | ROS 2 Humble | CR5、相机、ICP、AprilTag 和 `vision_arm_executor` |

网线接通后，先设置同网段静态地址；示例见 `config/local_robot_arm.env.example`。先执行
`ping $VEHICLE_IP`，再启动任意服务。不要在同一个终端混 source Noetic 与 Humble。

## 小车：原 master_node

在小车原有 ROS 1 工作区：

```bash
source /opt/ros/noetic/setup.bash
source <vehicle_catkin_ws>/devel/setup.bash
roslaunch master_node master_node.launch
```

只读确认：

```bash
curl -s http://<VEHICLE_IP>:1819/robot/mechanical_arm/check_connection
```

其 NX 通道保持原有 `mapid/poseid` 协议。启用本地NX兼容网关后，小车仍走
原协议，但连接目标由本机模拟，网关再转换成视觉执行器RPC。

## 本地视觉电脑：安全启动顺序

```bash
cd ~/Robot_Arm_Project
cp config/local_robot_arm.env.example config/local_robot_arm.env
# 编辑 IP 后：source config/local_robot_arm.env
./scripts/local_robot_arm.sh check
./scripts/local_robot_arm.sh build
```

按原项目方式分别启动 CR5 驱动、D455 相机和 AprilTag 检测，然后启动执行器：

```bash
# 先打印即将执行的命令，不启动任何节点
./scripts/local_robot_arm.sh all --dry-run

# 现场确认急停、机械臂工作区和IP后，一键启动全部本地栈
./scripts/local_robot_arm.sh all
```

一键脚本启动 CR5 ROS2 驱动、RealSense 点云、步进电机、AprilTag
检测器和 `vision_arm_executor`。如果只需要用户指定的三个硬件命令：

```bash
./scripts/local_robot_arm.sh all --hardware-only
```

小车与本机分开运行时，将 `VISION_RPC_HOST` 设为本机有线网卡 IP，
`VEHICLE_IP` 设为小车 IP，并配置强随机 `VISION_RPC_AUTH_TOKEN`。RPC 只放行
`VEHICLE_IP/32`；不建议监听无限制的公共网卡。

执行器启动后仍是 `execution_enabled=false`。先做健康检查与 dry-run；只有完成 ICP/AprilTag
质量验证、工作空间确认和现场安全确认后，才可以显式请求 `execution_enable`。

## 分阶段验收

1. 网线：双向 ping，小车 HTTP `1819` 可访问。
2. 小车：仅检查 `/mechanical_arm/check_connection`，不发送动作。
3. 本地：CR5、相机、点云和 AprilTag topic 均可见。
4. 本地：`vision_arm_executor` 健康检查和 dry-run。
5. 本地：ICP A/B 和 AprilTag locate/validate，均不执行抓取。
6. 经现场确认后才启用真实运动。

`master_node` 本身保持未修改。未启用兼容网关时，它不会感知本地视觉任务；
启用下述网关后，导航点任务可通过旧NX协议触发视觉动作，并在成功时收到原有
`status=done` 完成信号。

## 本机模拟原NX

小车代码固定连接 `192.168.2.103:8888`。不修改小车后端时，先在本机有线网卡
增加第二地址（把 `eno1` 换成现场实际网卡）：

```bash
sudo ip addr add 192.168.2.103/24 dev eno1
ip -4 addr show dev eno1
```

编辑 `config/local_robot_arm.env`：

```bash
export NX_COMPAT_ENABLED="true"
export NX_COMPAT_HOST="0.0.0.0"
export NX_COMPAT_PORT="8888"
export NX_COMPAT_ALLOWED_CLIENTS="192.168.2.15/32"
export NX_ROUTE_FILE="$HOME/Robot_Arm_Project/src/vision_arm_executor/config/nx_routes.json"
export NX_VISION_RPC_HOST="192.168.2.20"
```

随后编辑 `src/vision_arm_executor/config/nx_routes.json`。配置默认是
`dry_run=true` 且没有任何路由，未知点位会拒绝执行。例如：

```json
{
  "schema_version": 1,
  "defaults": {"timeout_sec": 200, "dry_run": true},
  "teaching_routes": [
    {"mapid": "map_01", "poseid": "icp_a", "action": "vision_icp_record_a"},
    {"mapid": "map_01", "poseid": "icp_b", "action": "vision_icp_record_b"}
  ],
  "routes": [
    {"mapid": "map_01", "poseid": "icp_work", "action": "vision_icp_align_and_move_b"},
    {"mapid": "map_01", "poseid": "tag_locate", "action": "apriltag_locate"},
    {"mapid": "map_01", "poseid": "tag_pick", "action": "apriltag_pick"}
  ]
}
```

路由支持 `mapid`、`poseid`、`label`，值为 `"*"` 表示通配；匹配时最具体的
规则优先。完成dry-run验收后，只对确认需要真实执行的规则设置
`"dry_run": false`。

启动：

```bash
./scripts/local_robot_arm.sh build
./scripts/local_robot_arm.sh all --dry-run
./scripts/local_robot_arm.sh all
```

日志中应出现：

```text
NX compatibility gateway listening on 0.0.0.0:8888
NX vehicle connected: 192.168.2.15:...
```

旧命令转换如下：`1→ResetRobot`、`2→EnableRobot`、`3→DisableRobot`、
`4→execution_enable`、`5→execution_disable`、`6→ClearError`。其中4/5是本地
视觉执行器的远程运动许可开关，进程重启后仍默认关闭。

自动任务成功后，网关才向小车回复 `{"status":"done"}`。失败只回复
`status=failed`，绝不伪装成功；由于旧小车后端只识别 `done`，失败时小车仍会
等待到原有200秒超时，这是保持application零修改时的已知限制。
