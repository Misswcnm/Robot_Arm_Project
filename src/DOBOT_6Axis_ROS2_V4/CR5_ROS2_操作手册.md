# CR5 ROS 2 操作手册

本文供现场调试和维护人员使用，只说明如何通过当前 ROS 2 驱动操作 CR5 机械臂本体。

## 1. 安全要求

1. 启动驱动本身不会让机械臂运动，但使能、拖拽和运动服务会改变机械臂状态。
2. 执行运动前确认工作区无人、目标点和路径已通过示教器低速验证、急停可触达。
3. 首次联调使用低速，本文采用全局速度 15% 和碰撞等级 5。
4. 同一时间只运行一个 CR5 驱动实例，不要同时使用原始 TCP 客户端下发控制命令。
5. ROS 2 服务返回 `res=0` 仅表示控制器接受命令；运动是否完成必须通过状态和位姿再次确认。

## 2. 驱动结构

当前驱动包位于：

```text
src/DOBOT_6Axis_ROS2_V4/
├── dobot_bringup_v4/   # 包名 cr_robot_ros2，TCP 驱动和 ROS 2 服务
├── dobot_msgs_v4/      # 服务及消息定义
└── cr5_moveit/         # CR5 MoveIt 配置，本手册不要求启动
```

驱动内部建立两条连接：

- `192.168.1.6:29999`：发送控制命令并读取 ASCII 应答。
- `192.168.1.6:30004`：接收机器人实时二进制状态。

ROS 2 服务最终仍由驱动转换成 TCP/IP 指令。工作人员无需自行解析 30004 数据。

## 3. 环境与网络检查

```bash
cd /home/ylx/Robot_Arm_Project
source /opt/ros/humble/setup.bash
source install/setup.bash

ping -c 3 192.168.1.6
nc -vz 192.168.1.6 29999
nc -vz 192.168.1.6 30004
```

如果 `source install/setup.bash` 后仍找不到驱动包，重新编译：

```bash
cd /home/ylx/Robot_Arm_Project
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dobot_msgs_v4 cr_robot_ros2
source install/setup.bash
```

确认包可见：

```bash
ros2 pkg prefix cr_robot_ros2
ros2 interface show dobot_msgs_v4/srv/GetPose
```

## 4. 启动和停止驱动

### 4.1 启动

```bash
cd /home/ylx/Robot_Arm_Project
source /opt/ros/humble/setup.bash
source install/setup.bash
export IP_address=192.168.1.6
export DOBOT_TYPE=cr5
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

正常日志应包含：

```text
robotIp 192.168.1.6
robotType cr5
connect successfully 192.168.1.6:30004
connect successfully 192.168.1.6:29999
```

`IP_address` 和 `DOBOT_TYPE` 是当前启动文件读取的环境变量，必须在启动驱动的同一个终端中设置。

### 4.2 检查 ROS 2 接口

在新终端执行：

```bash
cd /home/ylx/Robot_Arm_Project
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list | grep dobot
ros2 service list | grep /dobot_bringup_ros2/srv/
ros2 topic list | grep -E 'joint_states_robot|RobotStatus|ToolVectorActual'
```

### 4.3 停止

在启动驱动的终端按 `Ctrl-C`。停止驱动不会自动保证机械臂下使能；如需结束作业，应先按第 11 节执行安全下使能。

## 5. 返回值与状态

服务响应通常包含：

```text
res: 0
robot_return: "..."
```

- `res=0`：控制器接受命令。
- `res!=0`：控制器拒绝命令，停止当前操作并排查。
- `robot_return`：查询结果或控制器原始返回内容。

查询机器人模式：

```bash
ros2 service call /dobot_bringup_ros2/srv/RobotMode \
  dobot_msgs_v4/srv/RobotMode '{}'
```

| 模式值 | 状态 | 操作含义 |
|---:|---|---|
| 1 | 初始化 | 等待控制柜启动完成 |
| 2 | 抱闸松开 | 不下发自动运动 |
| 3 | 本体下电 | 按现场流程上电 |
| 4 | 未使能 | 可以清错、使能 |
| 5 | 已使能且空闲 | 可以运动或进入拖拽 |
| 6 | 拖拽 | 人工拖动机械臂 |
| 7 | 正在运行 | 等待完成或停止 |
| 8 | 单次点动 | 点动中 |
| 9 | 报错 | 查询报警并排障 |
| 10 | 暂停 | 可继续或停止 |
| 11 | 碰撞 | 停止、检查并清错 |

## 6. 基础查询

### 6.1 查询笛卡尔位姿

```bash
ros2 service call /dobot_bringup_ros2/srv/GetPose \
  dobot_msgs_v4/srv/GetPose '{user: 0, tool: 0}'
```

返回顺序为 `[x,y,z,rx,ry,rz]`；位置单位为 mm，姿态单位为度。

服务请求中虽然保留了 `user` 和 `tool` 字段，但当前兼容代码实际向旧控制器发送 `GetPose()`，避免部分固件因参数格式返回 `-40001`。

### 6.2 查询关节角

```bash
ros2 service call /dobot_bringup_ros2/srv/GetAngle \
  dobot_msgs_v4/srv/GetAngle '{}'
```

返回六个关节角 `[J1,J2,J3,J4,J5,J6]`，单位为度。

### 6.3 查询报警

```bash
ros2 service call /dobot_bringup_ros2/srv/GetErrorID \
  dobot_msgs_v4/srv/GetErrorID '{}'
```

第一组为控制器/算法报警，后六组对应六个伺服轴。应记录错误码并结合控制器上位机或报警说明定位原因。

### 6.4 查看实时话题

```bash
ros2 topic echo /dobot_msgs_v4/msg/RobotStatus
ros2 topic echo /dobot_msgs_v4/msg/ToolVectorActual --once
ros2 topic echo /joint_states_robot --once
```

- `RobotStatus`：连接、使能等状态。
- `ToolVectorActual`：30004 反馈中的当前末端位姿。
- `joint_states_robot`：当前六轴关节状态，供 ROS 2 可视化或其他节点使用。

若当前 ROS 2 CLI 不支持 `--once`，使用 `-n 1`。

## 7. 标准初始化

先查询 `RobotMode`。如果模式已经是 `5`，不要重复下使能再使能，直接执行第 7.3 节。

### 7.1 清错

```bash
ros2 service call /dobot_bringup_ros2/srv/ClearError \
  dobot_msgs_v4/srv/ClearError '{}'
```

### 7.2 使能

```bash
ros2 service call /dobot_bringup_ros2/srv/EnableRobot \
  dobot_msgs_v4/srv/EnableRobot '{}'
```

使能后再次查询 `RobotMode`，等待模式从 `4` 变为 `5`。如果模式为 `1`，说明控制柜还在启动；如果为 `9` 或 `11`，先排障。

### 7.3 设置安全参数和坐标系

```bash
ros2 service call /dobot_bringup_ros2/srv/SpeedFactor \
  dobot_msgs_v4/srv/SpeedFactor '{ratio: 15}'

ros2 service call /dobot_bringup_ros2/srv/SetCollisionLevel \
  dobot_msgs_v4/srv/SetCollisionLevel '{level: 5}'

ros2 service call /dobot_bringup_ros2/srv/User \
  dobot_msgs_v4/srv/User '{index: 0}'

ros2 service call /dobot_bringup_ros2/srv/Tool \
  dobot_msgs_v4/srv/Tool '{index: 0}'
```

- `ratio` 范围为 1～100。
- 碰撞等级范围为 0～5，越高越灵敏；0 为关闭碰撞检测，不建议现场关闭。
- 用户/工具坐标系索引必须与控制器内的实际标定一致。
- 每次重新使能后，建议重新设置速度、碰撞等级、User 和 Tool。

## 8. 运动操作

运动前必须同时满足：

- `RobotMode` 为 `5`。
- `GetErrorID` 无未处理报警。
- 目标点位、坐标系和运动路径已验证。
- 工作区无人，速度已降至安全值。

### 8.1 笛卡尔点到点运动

接口定义：

```text
mode=false
a,b,c = x,y,z，单位 mm
d,e,f = rx,ry,rz，单位 deg
```

调用模板：

```bash
ros2 service call /dobot_bringup_ros2/srv/MovJ \
  dobot_msgs_v4/srv/MovJ \
  "{mode: false, a: <x_mm>, b: <y_mm>, c: <z_mm>, d: <rx_deg>, e: <ry_deg>, f: <rz_deg>, param_value: []}"
```

尖括号字段必须替换为已确认的安全坐标后才能执行。驱动会为当前旧固件生成：

```text
MovJ(x,y,z,rx,ry,rz)
```

### 8.2 关节运动

接口定义：

```text
mode=true
a,b,c,d,e,f = J1,J2,J3,J4,J5,J6，单位 deg
```

调用模板：

```bash
ros2 service call /dobot_bringup_ros2/srv/MovJ \
  dobot_msgs_v4/srv/MovJ \
  "{mode: true, a: <j1_deg>, b: <j2_deg>, c: <j3_deg>, d: <j4_deg>, e: <j5_deg>, f: <j6_deg>, param_value: []}"
```

驱动会生成旧固件兼容指令 `JointMovJ(j1,...,j6)`。关节运动的末端路径不是直线，必须检查扫掠空间。

### 8.3 笛卡尔直线运动

```bash
ros2 service call /dobot_bringup_ros2/srv/MovL \
  dobot_msgs_v4/srv/MovL \
  "{mode: false, a: <x_mm>, b: <y_mm>, c: <z_mm>, d: <rx_deg>, e: <ry_deg>, f: <rz_deg>, param_value: []}"
```

`MovL` 会沿直线运行。终点可达不代表中间路径安全，首次必须低速验证。

### 8.4 可选运动参数

`param_value` 可以传入控制器支持的附加参数，例如：

```text
["user=0", "tool=0", "a=20", "v=20", "cp=0"]
```

旧固件的可选参数兼容性可能不同。现场基础操作优先保持 `param_value: []`，通过独立服务设置 `SpeedFactor`、`User` 和 `Tool`。

### 8.5 判断运动完成

`MovJ`/`MovL` 返回 `res=0` 只表示运动命令已被控制器接受。随后查询：

```bash
ros2 service call /dobot_bringup_ros2/srv/RobotMode \
  dobot_msgs_v4/srv/RobotMode '{}'

ros2 service call /dobot_bringup_ros2/srv/GetPose \
  dobot_msgs_v4/srv/GetPose '{user: 0, tool: 0}'
```

- 模式 `7`：运动仍在执行。
- 模式回到 `5`：运动队列已经空闲，再比较实际位姿和目标位姿。
- 模式 `9` 或 `11`：运动失败，执行第 10 节故障处理。

需要查看队列执行编号时：

```bash
ros2 service call /dobot_bringup_ros2/srv/GetCurrentCommandId \
  dobot_msgs_v4/srv/GetCurrentCommandId '{}'
```

## 9. 拖拽操作

### 9.1 进入拖拽

先确认模式为 `5`：

```bash
ros2 service call /dobot_bringup_ros2/srv/StartDrag \
  dobot_msgs_v4/srv/StartDrag '{}'

ros2 service call /dobot_bringup_ros2/srv/RobotMode \
  dobot_msgs_v4/srv/RobotMode '{}'
```

确认模式为 `6` 后再人工拖动机械臂。拖拽期间可调用 `GetPose` 记录当前位置。

### 9.2 退出拖拽

```bash
ros2 service call /dobot_bringup_ros2/srv/StopDrag \
  dobot_msgs_v4/srv/StopDrag '{}'

ros2 service call /dobot_bringup_ros2/srv/RobotMode \
  dobot_msgs_v4/srv/RobotMode '{}'
```

如果退出后模式为 `4`，执行 `EnableRobot`；如果已经是 `5`，不要重复使能。随后重新设置速度、碰撞等级、User 和 Tool，并确认最终模式为 `5`。

## 10. 停止、报警与恢复

### 10.1 软件停止

```bash
ros2 service call /dobot_bringup_ros2/srv/Stop \
  dobot_msgs_v4/srv/Stop '{}'
```

软件停止不能代替物理急停。出现人员或设备危险时直接按物理急停。

### 10.2 软件急停接口

```bash
ros2 service call /dobot_bringup_ros2/srv/EmergencyStop \
  dobot_msgs_v4/srv/EmergencyStop '{value: 1}'
```

`value: 1` 表示按下，`value: 0` 表示释放。仅按现场安全流程使用。

### 10.3 报警恢复

1. 调用 `Stop`，必要时按物理急停。
2. 检查实际碰撞、关节限位、工具、负载和线缆。
3. 调用 `GetErrorID` 并记录报警。
4. 原因排除后调用 `ClearError`。
5. 查询 `RobotMode`，确认安全后再调用 `EnableRobot`。
6. 重新设置速度、碰撞等级、User 和 Tool。
7. 确认模式为 `5`，再恢复作业。

禁止在原因未排除时循环清错和使能。

## 11. 下使能与结束作业

确认机械臂已停止并处于安全姿态后：

```bash
ros2 service call /dobot_bringup_ros2/srv/DisableRobot \
  dobot_msgs_v4/srv/DisableRobot '{}'

ros2 service call /dobot_bringup_ros2/srv/RobotMode \
  dobot_msgs_v4/srv/RobotMode '{}'
```

模式应为 `4`。正常完成单个动作或任务后不需要下使能再使能；只在结束作业、维护或安全流程明确要求时下使能。

## 12. 现场标准操作顺序

### 12.1 只读联通验证

1. 启动驱动，确认 29999 和 30004 均连接成功。
2. 调用 `RobotMode`。
3. 调用 `GetPose`、`GetAngle`。
4. 调用 `GetErrorID`。
5. 不执行使能和运动。

### 12.2 第一次低速运动

1. 查询模式和报警。
2. 必要时清错并使能，确认模式 `5`。
3. 设置速度 15%、碰撞等级 5、User(0)、Tool(0)。
4. 再次读取当前位姿。
5. 使用已在示教器验证的安全目标点调用 `MovJ`。
6. 监控模式从 `7` 回到 `5`。
7. 读取最终位姿，确认实际到达。

### 12.3 拖拽记录位姿

1. 确认模式 `5`，调用 `StartDrag`。
2. 确认模式 `6`，人工移动机械臂。
3. 调用 `GetPose` 读取并保存位置。
4. 调用 `StopDrag`。
5. 根据模式决定是否需要重新使能。
6. 恢复安全参数和坐标系，确认模式 `5`。

## 13. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 找不到 `cr_robot_ros2` | 未 source 工作空间或未编译 | 编译两个包并 `source install/setup.bash` |
| 启动日志 IP 为空 | 未设置 `IP_address` | 在同一终端执行 `export IP_address=192.168.1.6` |
| 只连接 29999，30004 失败 | 实时反馈端口不可用或被占用 | 检查控制模式、端口和其他驱动进程 |
| 服务列表为空 | 驱动未启动或 ROS_DOMAIN_ID 不一致 | 检查进程和各终端 ROS 2 环境 |
| `res=-2` | 控制柜未启动完成或不是 TCP 模式 | 等待启动，切换 TCP/IP 二次开发模式 |
| `res=-30001` | 旧固件不接受新版运动格式 | 使用当前已适配的 `MovJ` 服务，不要绕过驱动发新版格式 |
| `res=-40001` | 参数数量或格式不兼容 | `GetPose` 使用本驱动服务，避免自行拼接不完整参数 |
| 服务返回 0 但机械臂仍在运动 | 队列命令只完成接收 | 查询模式，等待 7 回到 5，并核对位姿 |
| 运动被拒绝 | 未使能、报警、碰撞、目标不可达或控制权不正确 | 查询模式和报警，检查目标与控制模式 |
| 日志出现重复发送或应答错配 | 多个节点或 TCP 客户端同时控制 | 保留单个驱动实例，停止其他控制程序 |

## 14. 接口定义与完整协议

- ROS 2 服务定义：`dobot_msgs_v4/srv/`
- 服务注册与转发：`dobot_bringup_v4/src/cr_robot_ros2.cpp`
- TCP 指令拼接：`dobot_bringup_v4/src/parseTool.cpp`
- TCP 连接与实时反馈：`dobot_bringup_v4/src/command.cpp`
- 官方完整协议：`V4新增指令/Dobot TCP_IP二次开发接口文档_V4.6.5_20251015_cn.pdf`

本手册列出的是工作人员常用且已由当前驱动实现的 CR5 操作。其他 IO、轨迹、力控和运动学接口应先查阅服务定义及官方协议，再在安全环境中验证。
