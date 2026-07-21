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

其 NX 通道保持原有 `mapid/poseid` 协议；本地视觉流程不会向它发送 NX 指令。

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

当前 master_node 未修改，因此它不会感知本地视觉任务的完成状态。若以后需要“导航到点位后自动触发视觉任务并回写任务状态”，应新增 application 外部的桥接进程，而不是修改小车后端。
