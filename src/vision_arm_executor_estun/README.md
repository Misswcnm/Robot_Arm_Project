# ESTUN 视觉执行器

## 结构

```text
vision_arm_executor_estun/
├── bridge/    # C++ Codroid WebSocket 适配和 ROS2 Service
└── executor/  # ICP、AprilTag、标定、TCP 8888 后端
```

不使用原 `estun_ros2`、ERI、`estun_hardware` 或 `ros2_control`。C++ 直接连接
ESTUN 控制柜 `192.168.2.5:9000`，完成状态/位姿/关节查询、上下电、清错、
MovJ/MovL、停止和 DO 输出。

`bridge` 把控制柜接口转换为本机 ROS 2 Service，供 `executor` 调用。

- 正式后端、手眼标定：自动启动或复用，无需单独启动。
- TCP 标定、Service 调试：尚未运行时手动启动。
- 同一台机械臂只启动一个 `bridge`。

## 构建

```bash
cd ~/Robot_Arm_Project
source /opt/ros/humble/setup.bash
colcon build --packages-select estun_codroid_bridge vision_arm_executor_estun
source install/setup.bash
```

## 手眼标定

自动启动 C++ 适配和 D455：

```bash
ros2 run vision_arm_executor_estun estun_handeye_calibration_all \
  --robot-ip 192.168.2.5 --chess-cols 8 --chess-rows 11 --square-mm 20
```

```text
d 手动模式  s 采样  c 计算保存  e 自动模式  q 退出
```

## TCP 标定

终端 1，仅在 C++ 适配尚未运行时启动：

```bash
ros2 launch estun_codroid_bridge codroid_bridge.launch.py \
  robot_ip:=192.168.2.5 robot_port:=9000
```

终端 2：

```bash
ros2 run vision_arm_executor_estun estun_tcp_calibration \
  --activate
```

```text
d 手动模式  r 记录  s 解算  w 保存  e 自动模式  q 退出
```

## 本地正式启动

自动启动 C++ 适配、D455、AprilTag、视觉执行器和 TCP `8888`：

```bash
ros2 launch vision_arm_executor_estun estun_vision_backend.launch.py \
  robot_ip:=192.168.2.5 robot_port:=9000 \
  nx_allowed_clients:=127.0.0.1/32,192.168.2.15/32
```

只读检查：

```bash
ros2 service call /estun_codroid/get_pose \
  estun_codroid_bridge/srv/GetPose '{}'
```

## AprilTag 运动内核探针

运动模式依赖正在运行的 C++ bridge。若正式后端已经停止，先在终端 1 启动：

```bash
ros2 launch estun_codroid_bridge codroid_bridge.launch.py \
  robot_ip:=192.168.2.5 robot_port:=9000
```

以下探针命令在终端 2 执行；测试期间不得让 NX 或其他节点并发下发机械臂指令。

先完全绕过观察关节位，只以当前笛卡尔位姿为基准沿 base X 移动 `5 mm`：

```bash
ros2 run vision_arm_executor_estun estun_apriltag_motion_probe \
  --mode delta --delta-mm 5,0,0 --execute --speed 5
```

先只读检查缓存坐标链和最终 `MovJ(CPos)` 报文；该命令不会运动：

```bash
ros2 run vision_arm_executor_estun estun_apriltag_motion_probe --mode inspect
```

针对指定站点先测试从观察关节位直接到预接近点：

```bash
ros2 run vision_arm_executor_estun estun_apriltag_motion_probe \
  --mode pregrasp --skip-observation --execute --speed 5 \
  --manifest ~/Robot_Arm_Project/data/vision_arm_estun/tasks/862dbce9-72c5-46cb-98c3-62b6c7b7ec7e/d3f3e006-c6c0-4e88-9806-63aa23b3c6d2/862dbce9-72c5-46cb-98c3-62b6c7b7ec7e_d3f3e006-c6c0-4e88-9806-63aa23b3c6d2.json
```

若直接预接近失败，用同一姿态按 `5% → 100%` 分段探测第一个失败端点：

```bash
ros2 run vision_arm_executor_estun estun_apriltag_motion_probe \
  --mode sweep --execute --speed 5 \
  --manifest ~/Robot_Arm_Project/data/vision_arm_estun/tasks/862dbce9-72c5-46cb-98c3-62b6c7b7ec7e/d3f3e006-c6c0-4e88-9806-63aa23b3c6d2/862dbce9-72c5-46cb-98c3-62b6c7b7ec7e_d3f3e006-c6c0-4e88-9806-63aa23b3c6d2.json
```

探针只使用 `MovJ(CPos)`，不使用雅可比/APos 回退，不操作夹爪，也不会自动复位；
控制器拒绝后会保留现场并输出请求前后位姿、`state/status_flag` 和首个失败比例。
ros2 run step_motor motor_node
ros2 topic pub --once /motor_control step_motor/msg/Motor "{id: 1,speed:
200,dir: 1,mode: 2,angle: 30000,state: 0,sub_divide: 32}"

后端仍使用原 `mapid/poseid/point_type/command` JSON 协议，前后端无需修改。

## 文件

| 功能 | 文件 |
|---|---|
| C++ Codroid | `bridge/src/codroid_bridge_node.cpp` |
| Service | `bridge/srv/` |
| 机械臂适配 | `executor/vision_arm_executor_estun/robot.py` |
| 手眼/TCP | `executor/vision_arm_executor_estun/handeye_calibration.py`、`tcp_calibration.py` |
| 正式启动 | `executor/launch/estun_vision_backend.launch.py` |
| 参数 | `executor/config/executor_estun.yaml` |
