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

### NX 开机自启动

包内置的 `estun-vision-backend.service` 使用上述固定启动参数，默认不启动
RViz。它与历史 `robot-arm.service` 互斥，避免两个进程同时抢占 NX
TCP 8888 端口。编译完成后在 NX 上执行：

```bash
sudo systemctl disable --now robot-arm.service 2>/dev/null || true
sudo install -m 0644 \
  install/vision_arm_executor_estun/share/vision_arm_executor_estun/systemd/estun-vision-backend.service \
  /etc/systemd/system/estun-vision-backend.service
sudo systemctl daemon-reload
sudo systemctl enable --now estun-vision-backend.service
systemctl status estun-vision-backend.service --no-pager -l
```

重启后端：

```bash
sudo systemctl restart estun-vision-backend.service
```

关闭后端：

```bash
sudo systemctl stop estun-vision-backend.service
```

监听日志：

```bash
sudo journalctl -u estun-vision-backend.service -f -o cat
```

RealSense 话题边界：Foxy `4.51.1` 默认发布到 `/camera/...`，较新的
ROS 2/RealSense 组合默认发布到 `/camera/camera/...`。后端会根据
`ROS_DISTRO` 同时调整 AprilTag 图像、相机内参和 ICP 点云订阅；自定义
相机命名空间时，用同一个环境变量显式覆盖：

```bash
export REALSENSE_TOPIC_ROOT=/my_camera
```

启动日志中的 `image_topic=...` 是最终生效的话题。该变量只定义 ROS
话题边界，不改变 D455 配置、标定坐标系或 NX 协议。

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

独立步进电机夹爪测试（不属于 ESTUN Codroid DO 自动夹爪链路）：

先构建嵌套工作区中的 `serial` 和 `step_motor` 包。该节点只订阅
`/motor_control`；正式视觉后端中的 `gripper.enabled` 与
`EstunRobot.set_gripper()` 当前使用 `/estun_codroid/set_do`，不会自动转发到
这个步进电机节点。

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
